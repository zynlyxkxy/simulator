#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_calib_verify.py — Edge headless + CDP 验证面板改动（2026-08-27）：

  1. wafer_load 上片误差注入组已迁入「测校」页（处方页不再有）；
  2. data-p 全局绑定生效：测校页输入被 /api/recipe 值填充；
  3. wafer_sites 新默认值（size 10 / gap 10）生效，派生 149 个键合位；
  4. 测校页截图存 Temp/panel_calib_<ts>.png。

用法：python backend/tests/panel_calib_verify.py
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
CDP_PORT = 9227
PANEL = os.path.join(ROOT, "panel", "index.html")
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp"))


def http_json(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


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


EVAL_JS = r"""
(function () {
  var out = {};
  out.calibWl = document.querySelectorAll('#pageCalib [data-p^="wafer_load."]').length;
  out.recipeWl = document.querySelectorAll('#pageRecipe [data-p^="wafer_load."]').length;
  var lx = document.querySelector('[data-p="wafer_load.load_x_mm"]');
  out.loadXVal = lx ? lx.value : null;
  var gx = document.querySelector('[data-p="wafer_sites.gap_x_mm"]');
  out.gapXVal = gx ? gx.value : null;
  var sx = document.querySelector('[data-p="wafer_sites.size_x_mm"]');
  out.sizeXVal = sx ? sx.value : null;
  out.wlSaveBtn = !!document.getElementById('btnWlSave');
  // 切到测校页用于截图
  document.getElementById('tabCalib').click();
  out.calibVisible = document.getElementById('pageCalib').style.display !== 'none';
  return JSON.stringify(out);
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

        # 1) API 层：新默认值 + 派生数量
        rec = http_json(base + "/api/recipe")["recipe"]
        ws = rec.get("wafer_sites") or {}
        sd = rec.get("wafer_sites_derived") or {}
        api_ok = (ws.get("gap_x_mm") == 0.005 and ws.get("gap_y_mm") == 0.005
                  and ws.get("size_x_mm") == 10.0 and ws.get("size_y_mm") == 10.0
                  and sd.get("total_sites") == 621)
        print("[api] wafer_sites=%s derived total=%s rows=%s cols=%s → %s"
              % (ws, sd.get("total_sites"), sd.get("rows"), sd.get("cols"),
                 "OK" if api_ok else "MISMATCH"))

        # 2) Edge headless 打开面板
        profile = tempfile.mkdtemp(prefix="edge_panel_")
        url = "file:///" + PANEL.replace(os.sep, "/")
        edge = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check",
             "--remote-debugging-port=%d" % CDP_PORT,
             "--user-data-dir=" + profile, "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        info = {}
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
                print("[wait] 面板运行 6s（等 /api/recipe 首轮填充）...")
                time.sleep(6)
                res = cdp.call("Runtime.evaluate",
                               {"expression": EVAL_JS, "returnByValue": True})
                info = json.loads(res["result"]["value"])
                time.sleep(1.0)
                shot = cdp.call("Page.captureScreenshot", {"format": "png"})
                p = os.path.join(
                    SHOT_DIR, "panel_calib_%s.png"
                    % time.strftime("%Y%m%d_%H%M%S"))
                with open(p, "wb") as f:
                    f.write(base64.b64decode(shot["data"]))
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

        # 3) 判定
        ok = True
        print("\n===== 判定 =====")
        print("测校页 wafer_load 输入数 : %s（期望 6）" % info.get("calibWl"))
        print("处方页 wafer_load 输入数 : %s（期望 0）" % info.get("recipeWl"))
        print("load_x 填充值            : %s（期望 0.25）" % info.get("loadXVal"))
        print("gap_x 填充值             : %s（期望 10）" % info.get("gapXVal"))
        print("size_x 填充值            : %s（期望 10）" % info.get("sizeXVal"))
        print("测校页保存按钮           : %s" % info.get("wlSaveBtn"))
        print("测校页已切换可见         : %s" % info.get("calibVisible"))
        if not api_ok:
            print("[FAIL] API 默认值/派生不符"); ok = False
        if info.get("calibWl") != 6 or info.get("recipeWl") != 0:
            print("[FAIL] wafer_load 组迁移不符"); ok = False
        if info.get("loadXVal") not in ("0.25", 0.25):
            print("[FAIL] 测校页输入未被 recipe 值填充"); ok = False
        if str(info.get("gapXVal")) != "10" or str(info.get("sizeXVal")) != "10":
            print("[FAIL] wafer_sites 新默认值未填充"); ok = False
        if not info.get("wlSaveBtn"):
            print("[FAIL] 测校页保存按钮缺失"); ok = False
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
