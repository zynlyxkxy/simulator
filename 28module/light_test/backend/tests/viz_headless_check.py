#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""viz_headless_check.py — Edge headless + CDP 验证联机版真空/工件可视化。

前置：
  1. python backend/tests/panel_mock.py 9025   （提供 /api/viz 等）
  2. POST /api/workpiece/wafer load（本脚本会自动做）

验证项（PASS 标准）：
  - window.__TD_VIZ__ 场景钩子存在（build_online_html.py 注入）
  - HUD data-viz: mode=scene, active=true, cupsFound=4, wafer=true, dies>0
  - scene 内 __viz_wafer 网格存在且 visible，__viz_die_* 数量 = viz dies 数
  - 真空吸盘 emissive 被染绿（晶圆吸盘材质 emissive != 原色）
  - HUD 脚本错误 = 无；查看器无红色错误弹窗

用法：python backend/tests/viz_headless_check.py [mock端口=9025]
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from edge_headless_check import CDP  # 复用最小 CDP 客户端

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
CDP_PORT = 9224
WAIT_S = 8

EVAL_JS = r"""
(function () {
  var out = { tdviz: null, dataViz: null, hudText: null, errorBox: null,
              waferVisible: null, dieMeshCount: -1, greenMats: -1 };
  out.tdviz = window.__TD_VIZ__ ? Object.keys(window.__TD_VIZ__) : null;
  var divs = document.querySelectorAll('div');
  for (var i = 0; i < divs.length; i++) {
    if (divs[i].textContent.indexOf('联机状态') >= 0 &&
        divs[i].style.zIndex === '9999') {
      out.hudText = divs[i].textContent;
      out.dataViz = divs[i].getAttribute('data-viz');
      break;
    }
  }
  for (var i = 0; i < divs.length; i++) {
    var t = divs[i].textContent;
    if (t.indexOf('三维模型启动失败') >= 0 && t.length < 500) { out.errorBox = t; break; }
  }
  if (window.__TD_VIZ__) {
    var scene = window.__TD_VIZ__.scene;
    var wafer = scene.getObjectByName('__viz_wafer');
    out.waferVisible = wafer ? !!wafer.visible : null;
    var dc = 0, green = 0;
    scene.traverse(function (o) {
      if (o.name && o.name.indexOf('__viz_die_') === 0 && o.visible) dc++;
      var mats = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      mats.forEach(function (m) {
        if (m.emissive && m.emissive.getHex() === 0x18c458) green++;
      });
    });
    out.dieMeshCount = dc;
    out.greenMats = green;
  }
  return JSON.stringify(out);
})()
"""


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9025
    base = 'http://127.0.0.1:%d' % port

    # 保证 wafer 在位（12 wafer dies + flip/head 各 1）
    req = urllib.request.Request(base + '/api/workpiece/wafer',
                                 data=json.dumps({'action': 'load', 'offset': [3, -2]}).encode(),
                                 headers={'Content-Type': 'application/json'})
    print('[mock] wafer load:', urllib.request.urlopen(req, timeout=5).read().decode())

    url = '%s/online?api=%s' % (base, base)
    profile = tempfile.mkdtemp(prefix='edge_viz_')
    proc = subprocess.Popen(
        [EDGE, '--headless=new', '--disable-gpu', '--no-first-run',
         '--no-default-browser-check', '--remote-debugging-port=%d' % CDP_PORT,
         '--user-data-dir=' + profile, 'about:blank'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        t0 = time.time()
        while time.time() - t0 < 20:
            try:
                urllib.request.urlopen('http://127.0.0.1:%d/json/version' % CDP_PORT, timeout=2)
                break
            except Exception:
                time.sleep(0.3)
        from urllib.parse import quote
        req = urllib.request.Request('http://127.0.0.1:%d/json/new?%s' % (CDP_PORT, quote(url)),
                                     method='PUT')
        target = json.loads(urllib.request.urlopen(req, timeout=5).read().decode())
        cdp = CDP(target['webSocketDebuggerUrl'])
        try:
            cdp.call('Runtime.enable')
            print('[wait] 页面运行 %ds（查看器初始化 + viz 轮询生效）...' % WAIT_S)
            time.sleep(WAIT_S)
            res = cdp.call('Runtime.evaluate', {'expression': EVAL_JS, 'returnByValue': True})
            info = json.loads(res['result']['value'])
        finally:
            cdp.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)

    print('\n===== viz 验证结果 =====')
    print('__TD_VIZ__  : %s' % info['tdviz'])
    print('data-viz    : %s' % info['dataViz'])
    print('wafer visible: %s  die meshes: %s  绿色发光材质数: %s'
          % (info['waferVisible'], info['dieMeshCount'], info['greenMats']))
    print('errorBox    : %s' % (info['errorBox'] or '无'))
    if info['hudText']:
        for ln in info['hudText'].split('\n'):
            print('    HUD| %s' % ln)

    ok = True
    def check(cond, msg):
        nonlocal ok
        print(('PASS ' if cond else 'FAIL ') + msg)
        ok = ok and cond

    check(bool(info['tdviz']), '__TD_VIZ__ 场景钩子存在')
    dv = json.loads(info['dataViz']) if info['dataViz'] else {}
    check(dv.get('mode') == 'scene', 'viz 模式 = scene（非灯带兜底）')
    check(dv.get('active') is True, 'viz 轮询在线')
    check(dv.get('cupsFound') == 4, '4 个吸盘对象全部找到（%s）' % dv.get('cupsFound'))
    check(dv.get('cupsOn', 0) >= 3, '≥3 吸盘真空吸附（%s）' % dv.get('cupsOn'))
    check(dv.get('wafer') is True, 'wafer 在位')
    check((dv.get('dies') or 0) == 22, 'viz dies=22（%s）' % dv.get('dies'))
    check(info['waferVisible'] is True, 'scene 中 __viz_wafer 可见')
    check(info['dieMeshCount'] == 22, 'scene 中 die 网格 22 个（%s）' % info['dieMeshCount'])
    check(info['greenMats'] > 0, '吸盘 emissive 绿色发光生效（%s 个材质）' % info['greenMats'])
    check(info['hudText'] and '脚本错误  无' in info['hudText'], 'HUD 脚本错误=无')
    check(not info['errorBox'], '无查看器红色错误弹窗')
    print('\n==== %s ====' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
