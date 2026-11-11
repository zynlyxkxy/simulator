#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""online_confocal_verify.py — Edge headless + CDP 验证联机版共焦光束可视化
（2026-08-27，调焦调平迭代 3）：

  1. 启动 runtime；
  2. Edge headless 打开 `html_model/d2w_online.html` 并等待 WS 连接；
  3. 循环调用 /api/confocal/read（up/down）触发 250ms 可视化窗口；
  4. 读取联机版 HUD 的 `data-viz` 属性，记录 confocalFound/confocalOn 最大值；
  5. 断言：data-viz.confocalFound == 2（上下探头圆柱都已创建）且
     某次 confocalOn >= 1（光束确实点亮过）；
  6. 截图存 Temp/online_confocal_<ts>.png。

用法：python backend/tests/online_confocal_verify.py
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
from urllib.parse import quote

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402

WS_PORT = 9010
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
CDP_PORT = 9229
HTML_PATH = os.path.join(ROOT, "..", "html_model", "d2w_online.html")
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp"))


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


def read_data_viz(cdp):
    eval_js = r"""
(function () {
  var hud = null;
  var divs = document.querySelectorAll('div');
  for (var i = 0; i < divs.length; i++) {
    if (divs[i].textContent.indexOf('联机状态') >= 0 &&
        divs[i].style.zIndex === '9999') { hud = divs[i]; break; }
  }
  var raw = hud ? hud.getAttribute('data-viz') : null;
  var viz = raw ? JSON.parse(raw) : {};
  return JSON.stringify({raw: raw, confocalFound: viz.confocalFound,
                         confocalOn: viz.confocalOn,
                         mode: viz.mode, active: viz.active,
                         hudText: hud ? hud.textContent.substring(0, 120) : null});
})()
"""
    res = cdp.call("Runtime.evaluate",
                   {"expression": eval_js, "returnByValue": True})
    return json.loads(res["result"]["value"])


def main():
    if not os.path.exists(EDGE):
        print("[FAIL] Edge 不存在：%s" % EDGE)
        return 1
    if not os.path.exists(HTML_PATH):
        print("[FAIL] 联机版 HTML 不存在：%s" % HTML_PATH)
        return 1
    os.makedirs(SHOT_DIR, exist_ok=True)
    kill_old_runtime()

    proc = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "runtime", "main.py")],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    edge = None
    profile = tempfile.mkdtemp(prefix="edge_online_confocal_")
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # Edge headless 打开联机版
        url = "file:///" + HTML_PATH.replace(os.sep, "/")
        edge = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check",
             "--remote-debugging-port=%d" % CDP_PORT,
             "--user-data-dir=" + profile, "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 等 CDP
        version = None
        t0 = time.time()
        while time.time() - t0 < 20:
            try:
                with urllib.request.urlopen(
                        "http://127.0.0.1:%d/json/version" % CDP_PORT,
                        timeout=2) as r:
                    version = json.loads(r.read().decode("utf-8"))
                break
            except Exception:
                time.sleep(0.3)
        if not version:
            print("[FAIL] Edge CDP 未就绪")
            return 1
        print("[cdp] %s" % version.get("Browser"))

        req = urllib.request.Request(
            "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
            method="PUT")
        with urllib.request.urlopen(req, timeout=5) as r:
            target = json.loads(r.read().decode("utf-8"))
        ws_url = target["webSocketDebuggerUrl"]

        cdp = CDP(ws_url)
        try:
            cdp.call("Runtime.enable")
            cdp.call("Page.enable")

            # 等查看器初始化 + WS 连接
            print("[wait] 页面初始化 + WS 连接...")
            time.sleep(6.0)

            # 循环触发共焦读数并轮询 data-viz
            print("[loop] 触发 up/down 共焦读数，观察光束...")
            max_found, max_on = 0, 0
            for i in range(16):
                probe = "up" if i % 2 == 0 else "down"
                try:
                    http_json(base + "/api/confocal/read?probe=" + probe)
                except Exception as e:
                    print("[warn] confocal read %s failed: %s" % (probe, e))
                time.sleep(0.15)
                parsed = read_data_viz(cdp)
                found = parsed.get("confocalFound") or 0
                on = parsed.get("confocalOn") or 0
                max_found = max(max_found, found)
                max_on = max(max_on, on)
                if i == 0:
                    print("[viz] first read -> found=%s on=%s" % (found, on))
            print("[viz] max found=%s max on=%s" % (max_found, max_on))

            ok = (max_found == 2 and max_on >= 1)
            print("[check] confocalFound=%s confocalOn(max)=%s -> %s"
                  % (max_found, max_on, "PASS" if ok else "FAIL"))

            # 截图
            ts = time.strftime("%Y%m%d_%H%M%S")
            shot = os.path.join(SHOT_DIR, "online_confocal_%s.png" % ts)
            resp = cdp.call("Page.captureScreenshot", {"format": "png"})
            img = resp.get("data", "")
            with open(shot, "wb") as f:
                f.write(base64.b64decode(img))
            print("[shot] %s" % shot)

            return 0 if ok else 1
        finally:
            cdp.close()
    finally:
        if edge is not None:
            edge.terminate()
            try:
                edge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                edge.kill()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
