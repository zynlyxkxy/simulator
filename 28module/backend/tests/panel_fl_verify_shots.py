#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_fl_verify_shots.py — Edge headless + CDP 截图验证迭代 5 调平复测
（2026-08-28）：

  面板工艺页实跑两次调焦调平（API 触发，面板流程日志区可见）：
    1. per_site + align_first（先对准/二次对准）→ 日志应见"晶圆面复测/
       芯片面复测（二次对准…）"与"调平复测 PASS：残余 … ≤ 阈值 30 µrad"；
    2. global（复用调平记忆，跳过测量直接复测）→ 日志应见"晶圆面测点=
       3 点（EGA 切蛋糕最大面积三角形）：3 个站点坐标"与复测 PASS。
  每次跑完把 #flowLog 滚到底部截全页，存 Temp/panel_fl_verify_<模式>_<ts>.png，
  并断言 flow.state.log 含上述关键文本。

用法：python backend/tests/panel_fl_verify_shots.py
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
CDP_PORT = 9231
PANEL = os.path.join(ROOT, "panel", "index.html")
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp"))


def http_json(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def http_post(url, data):
    req = urllib.request.Request(
        url, data=json.dumps(data).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_put(url, data):
    req = urllib.request.Request(
        url, data=json.dumps(data).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="PUT")
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


def run_flow(base, path, body, timeout=600.0):
    c, r = http_post(base + path, body)
    if c != 200 or not r.get("ok"):
        raise RuntimeError("%s 启动失败：%s %s" % (path, c, r.get("error")))
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = http_json(base + "/api/flow/state")
        if s.get("state") in ("done", "aborted"):
            return s
        time.sleep(0.5)
    raise RuntimeError("%s 超时" % path)


SCROLL_BOTTOM_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  var g = document.getElementById('flowLog');
  if (g && g.scrollIntoView) g.scrollIntoView({block: 'center'});
  return 'ok';
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
    edge = None
    shots = []
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # ---- 确定性场景：σ=0、jitter 关、注入 wafer rx=+300 / die ry=-200 ----
        http_put(base + "/api/recipe", {
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "wafer_load": {
                "load_x_mm": 0.0, "load_y_mm": 0.0, "load_rz_rad": 0.0,
                "jitter_on": False, "jitter_xy_um": 0.0,
                "jitter_rz_urad": 0.0, "z_um": 0.0,
                "rx_urad": 300.0, "ry_urad": 0.0,
                "die_head_rx_urad": 0.0, "die_head_ry_urad": -200.0}})

        # ---- 前置链：取片→交接→EGA（强制重上片注入倾斜）----
        print("[flow] flip_pick / flip_handoff / wafer_ega ...")
        run_flow(base, "/api/process/flip_pick", {})
        run_flow(base, "/api/process/flip_handoff", {})
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        run_flow(base, "/api/process/wafer_ega", {"with_load": True})

        # ---- Edge headless 打开面板（工艺页）----
        profile = tempfile.mkdtemp(prefix="edge_panel_flv_")
        url = "file:///" + PANEL.replace(os.sep, "/")
        edge = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check",
             "--remote-debugging-port=%d" % CDP_PORT,
             "--user-data-dir=" + profile, "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t0 = time.time()
        while time.time() - t0 < 20:
            try:
                urllib.request.urlopen(
                    "http://127.0.0.1:%d/json/version" % CDP_PORT, timeout=2)
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
        ts = time.strftime("%Y%m%d_%H%M%S")
        results = {}
        try:
            cdp.call("Runtime.enable")
            print("[wait] 面板运行 5s（等首轮填充）...")
            time.sleep(5)

            for tag, body in (
                    ("per_site", {"mode": "per_site", "align_first": True}),
                    ("global", {"mode": "global"})):
                print("[flow] focus_level %s %s ..." % (tag, body))
                s = run_flow(base, "/api/process/focus_level", body)
                logs = "\n".join(str(e.get("msg", ""))
                                 for e in (s.get("log") or []))
                fv = s.get("fl_verify") or {}
                results[tag] = {
                    "state": s.get("state"),
                    "has_remeasure": "复测" in logs,
                    "has_pass": "调平复测 PASS" in logs,
                    "has_triangle": "切蛋糕" in logs,
                    "judge": fv.get("judge"),
                    "residual": fv.get("residual_urad")}
                time.sleep(2.0)      # 等面板轮询刷新流程日志
                cdp.call("Runtime.evaluate",
                         {"expression": SCROLL_BOTTOM_JS,
                          "returnByValue": True})
                time.sleep(1.0)
                shot = cdp.call("Page.captureScreenshot", {"format": "png"})
                p = os.path.join(
                    SHOT_DIR, "panel_fl_verify_%s_%s.png" % (tag, ts))
                with open(p, "wb") as f:
                    f.write(base64.b64decode(shot["data"]))
                shots.append(p)
                print("[ok] 截图已保存：%s" % p)
        finally:
            cdp.close()

        # ---- 判定 ----
        ok = True
        print("\n===== 判定 =====")
        ps = results.get("per_site", {})
        print("per_site(align_first): state=%s 复测=%s PASS=%s judge=%s "
              "residual=%s" % (ps.get("state"), ps.get("has_remeasure"),
                               ps.get("has_pass"), ps.get("judge"),
                               ps.get("residual")))
        if not (ps.get("state") == "done" and ps.get("has_remeasure")
                and ps.get("has_pass") and ps.get("judge") == "pass"):
            print("[FAIL] per_site 复测验证不符"); ok = False
        gl = results.get("global", {})
        print("global: state=%s 切蛋糕3点=%s PASS=%s judge=%s residual=%s"
              % (gl.get("state"), gl.get("has_triangle"), gl.get("has_pass"),
                 gl.get("judge"), gl.get("residual")))
        if not (gl.get("state") == "done" and gl.get("has_triangle")
                and gl.get("has_pass") and gl.get("judge") == "pass"):
            print("[FAIL] global 复测验证不符"); ok = False
        print("\n==== %s ====" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    finally:
        if edge is not None:
            edge.terminate()
            try:
                edge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                edge.kill()
            shutil.rmtree(profile, ignore_errors=True)
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
