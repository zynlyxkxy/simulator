#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_focus_level_verify.py — Edge headless + CDP 验证调焦调平面板改动
（2026-08-27，Phase E 无头浏览器截图验证；迭代 2 扩展）：

  1. 机器常数页出现 focus_level 组卡片及 11 个字段输入框（**有
     confocal_cam_dx_mm、as_head_center_xy_mm；无 confocal_sigma_um**
     ——σ 已迁测校页）；
  2. 工艺页出现"调焦调平"分组：生成并运行按钮 + 模式下拉（3 选项）
     + align_first"先对准"勾选框；
  3. 测校页 wafer_load 卡有 5 个新注入输入框（z/rx/ry/die_head_rx/
     die_head_ry）+ "传感器仿真"卡有 sim_sensors.confocal_sigma_um；
  4. 守卫路径：未满足前置时 POST /api/process/focus_level → 400，页面
     无 JS 异常（window.onerror 收集器为空）；
  5. 三个页面各截图一张存 Temp/panel_fl_<页>_<ts>.png。

用法：python backend/tests/panel_focus_level_verify.py
（9010 被占用时会先结束旧 runtime）
"""
import base64
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402

WS_PORT = 9010
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
CDP_PORT = 9228
PANEL = os.path.join(ROOT, "panel", "index.html")
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp"))

FL_FIELDS = ["confocal_cam_dx_mm", "confocal_cam_dy_mm",
             "as_head_center_xy_mm",
             "confocal_mid_up_um", "confocal_mid_down_um",
             "confocal_range_um", "as_z_chuck_focus_mm",
             "head_z_upfocus_mm", "level_tol_urad", "focus_search_span_um",
             "focus_search_step_um"]
WL_NEW = ["z_um", "rx_urad", "ry_urad", "die_head_rx_urad",
          "die_head_ry_urad"]


def http_json(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def http_post(url, data):
    req = urllib.request.Request(
        url, data=json.dumps(data).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def wait_ready(timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if http_json("http://127.0.0.1:%d/api/health" % WS_PORT).get("ok"):
                return True
        except Exception:
            pass
        time.sleep(0.3)
    return False


def kill_old_runtime():
    try:
        http_json("http://127.0.0.1:%d/api/health" % WS_PORT)
    except Exception:
        return
    print("[warn] 9010 已被占用，结束旧 runtime...")
    out = subprocess.check_output(["netstat", "-ano"], text=True,
                                  errors="replace")
    for line in out.splitlines():
        if ":9010" in line and "LISTENING" in line:
            pid = line.strip().split()[-1]
            subprocess.call(["taskkill", "//F", "//PID", pid],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
            break


# JS 采集表达式：切换三个 tab 并统计各验证点（模板里 __PAGE__ 替换为
# const/flow/calib，截图前先把目标页切到可见）
EVAL_JS = r"""
(function () {
  if (!window.__errs) {
    window.__errs = [];
    window.addEventListener('error', function (e) {
      window.__errs.push(String(e.message || e));
    });
  }
  var out = {};
  // 1) 机器常数页：focus_level 组卡片 + 字段数
  document.getElementById('tabConst').click();
  out.constFields = document.querySelectorAll(
    '#mcGrpGrid [data-ck^="focus_level."]').length;
  var card = document.querySelector('#mcGrpGrid [data-ck="focus_level.level_tol_urad"]');
  out.constCard = !!card;
  // 2) 工艺页：调焦调平分组（按钮 + 模式下拉 + 先对准勾选）
  document.getElementById('tabFlow').click();
  out.flBtnRun = !!document.getElementById('btnFlRun');
  out.flBtnPaused = !!document.getElementById('btnFlPaused');
  var sel = document.getElementById('flMode');
  out.flModeOpts = sel ? sel.options.length : 0;
  out.flAlignFirst = !!document.getElementById('flAlignFirst');
  // 3) 测校页：wafer_load 新注入输入框 + 传感器仿真卡
  document.getElementById('tabCalib').click();
  out.calibWlNew = 0;
  ['z_um','rx_urad','ry_urad','die_head_rx_urad','die_head_ry_urad']
    .forEach(function (k) {
      if (document.querySelector('[data-p="wafer_load.' + k + '"]')) {
        out.calibWlNew++;
      }
    });
  out.calibWlTotal = document.querySelectorAll(
    '#pageCalib [data-p^="wafer_load."]').length;
  out.calibSimSensors = !!document.querySelector(
    '[data-p="sim_sensors.confocal_sigma_um"]');
  out.calibSsSave = !!document.getElementById('btnSsSave');
  out.jsErrs = window.__errs.length;
  // 切到目标截图页（滚动在下一轮 evaluate 里做——切页触发的
  // pollMachineConstants 异步回填完成后滚动才稳定）
  var tab = {const: 'tabConst', flow: 'tabFlow', calib: 'tabCalib'}['__PAGE__'];
  document.getElementById(tab).click();
  return JSON.stringify(out);
})()
"""

SCROLL_JS = r"""
(function () {
  var sel = {const: '#mcGrpGrid [data-ck="focus_level.confocal_cam_dx_mm"]',
             flow: '#flAlignFirst',
             calib: '[data-p="sim_sensors.confocal_sigma_um"]'}['__PAGE__'];
  var el = document.querySelector(sel);
  if (el && el.scrollIntoView) el.scrollIntoView({block: 'center'});
  return JSON.stringify({found: !!el, scrollY: window.scrollY});
})()
"""


def main():
    if not os.path.exists(EDGE):
        print("[FAIL] Edge 不存在：%s" % EDGE)
        return 1
    os.makedirs(SHOT_DIR, exist_ok=True)
    kill_old_runtime()

    proc = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "runtime", "main.py")],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # 0) API 层：focus_level 组字段齐（有 dx、无 sigma）+ 守卫 400
        mc = http_json(base + "/api/machine_constants")
        fl = (mc.get("constants") or {}).get("focus_level") or {}
        api_fields_ok = all(k in fl for k in FL_FIELDS) \
            and "confocal_sigma_um" not in fl
        print("[api] focus_level 字段 %d/11（dx/as_head_center 在、"
              "sigma 已迁出）→ %s"
              % (len([k for k in FL_FIELDS if k in fl]),
                 "OK" if api_fields_ok else "MISMATCH: %s"
                 % [k for k in FL_FIELDS if k not in fl]))
        c, r = http_post(base + "/api/process/focus_level",
                         {"mode": "per_site"})
        guard_ok = c == 400
        print("[api] 守卫（无前置）POST focus_level → %d %s → %s"
              % (c, r.get("error", ""), "OK" if guard_ok else "FAIL"))

        # 1) Edge headless 打开面板
        profile = tempfile.mkdtemp(prefix="edge_panel_fl_")
        url = "file:///" + PANEL.replace(os.sep, "/")
        edge = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check",
             "--remote-debugging-port=%d" % CDP_PORT,
             "--user-data-dir=" + profile, "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        info = {}
        shots = []
        try:
            t0 = time.time()
            while time.time() - t0 < 20:
                try:
                    urllib.request.urlopen(
                        "http://127.0.0.1:%d/json/version" % CDP_PORT,
                        timeout=2)
                    break
                except Exception:
                    time.sleep(0.3)
            from urllib.parse import quote
            req = urllib.request.Request(
                "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
                method="PUT")
            target = json.loads(
                urllib.request.urlopen(req, timeout=5).read().decode())
            cdp = CDP(target["webSocketDebuggerUrl"])
            try:
                cdp.call("Runtime.enable")
                print("[wait] 面板运行 6s（等机器常数/处方首轮填充）...")
                time.sleep(6)
                ts = time.strftime("%Y%m%d_%H%M%S")
                for page in ("const", "flow", "calib"):
                    expr = EVAL_JS.replace("'__PAGE__'", "'%s'" % page)
                    res = cdp.call("Runtime.evaluate",
                                   {"expression": expr,
                                    "returnByValue": True})
                    info = json.loads(res["result"]["value"])
                    time.sleep(1.2)         # 等切页触发的异步回填完成
                    expr = SCROLL_JS.replace("'__PAGE__'", "'%s'" % page)
                    res = cdp.call("Runtime.evaluate",
                                   {"expression": expr,
                                    "returnByValue": True})
                    sc = json.loads(res["result"]["value"])
                    print("[scroll] %s found=%s scrollY=%s"
                          % (page, sc.get("found"), sc.get("scrollY")))
                    time.sleep(0.3)
                    shot = cdp.call("Page.captureScreenshot",
                                    {"format": "png"})
                    p = os.path.join(
                        SHOT_DIR, "panel_fl_%s_%s.png" % (page, ts))
                    with open(p, "wb") as f:
                        f.write(base64.b64decode(shot["data"]))
                    shots.append(p)
                    print("[ok] 截图已保存：%s" % p)
            finally:
                cdp.close()
        finally:
            edge.terminate()
            try:
                edge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                edge.kill()
            shutil.rmtree(profile, ignore_errors=True)

        # 2) 判定
        ok = True
        print("\n===== 判定 =====")
        print("机械常数页 focus_level 字段数 : %s（期望 11：有 dx / "
              "as_head_center，无 sigma）" % info.get("constFields"))
        print("工艺页 调焦调平按钮/暂停按钮  : %s / %s"
              % (info.get("flBtnRun"), info.get("flBtnPaused")))
        print("工艺页 模式下拉选项数         : %s（期望 3）"
              % info.get("flModeOpts"))
        print("工艺页 先对准(align_first)勾选: %s（期望 True）"
              % info.get("flAlignFirst"))
        print("测校页 wafer_load 新输入框    : %s（期望 5，总 %s）"
              % (info.get("calibWlNew"), info.get("calibWlTotal")))
        print("测校页 传感器仿真卡 σ/保存按钮: %s / %s（期望 True / True）"
              % (info.get("calibSimSensors"), info.get("calibSsSave")))
        print("页面 JS 异常数                : %s（期望 0）"
              % info.get("jsErrs"))
        if not api_fields_ok:
            print("[FAIL] API focus_level 字段不齐"); ok = False
        if not guard_ok:
            print("[FAIL] 守卫未返回 400"); ok = False
        if info.get("constFields") != 11 or not info.get("constCard"):
            print("[FAIL] 机械常数页 focus_level 组渲染不符"); ok = False
        if not info.get("flBtnRun") or not info.get("flBtnPaused"):
            print("[FAIL] 工艺页调焦调平按钮缺失"); ok = False
        if info.get("flModeOpts") != 3:
            print("[FAIL] 模式下拉选项数不符"); ok = False
        if not info.get("flAlignFirst"):
            print("[FAIL] 工艺页缺先对准勾选框"); ok = False
        if info.get("calibWlNew") != 5:
            print("[FAIL] 测校页 wafer_load 新注入输入框不齐"); ok = False
        if not info.get("calibSimSensors") or not info.get("calibSsSave"):
            print("[FAIL] 测校页传感器仿真卡缺失"); ok = False
        if info.get("jsErrs") != 0:
            print("[FAIL] 页面存在 JS 异常"); ok = False
        print("\n==== %s ====" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
