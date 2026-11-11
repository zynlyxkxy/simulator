#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ega_headless_verify.py — Edge headless + CDP 验证晶圆上片 + EGA 流程。

前置：本机须装 Microsoft Edge（默认路径）。
流程：
  1. 启动 runtime/main.py
  2. 注入确定性 wafer_load 误差（关闭 jitter）
  3. POST /api/process/wafer_ega 跑全局对准
  4. 流程完成后用 Edge headless 打开联机版页面
  5. 截图保存到 Temp/ega_verify_<ts>.png
  6. 校验 EGA 拟合结果找回注入值

用法：python backend/tests/ega_headless_verify.py
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

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402

WS_PORT = 9010
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
CDP_PORT = 9225
HTML = os.path.abspath(os.path.join(ROOT, "..", "html_model", "d2w_online.html"))
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp"))


def http_json(url, data=None):
    if data is None:
        with urllib.request.urlopen(url, timeout=5) as r:
            return json.loads(r.read().decode("utf-8"))
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def http_post(url, data):
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_put(url, data):
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"},
                                 method="PUT")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def wait_ready(timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            h = http_json("http://127.0.0.1:%d/api/health" % WS_PORT)
            if h.get("ok"):
                return True
        except Exception:
            pass
        time.sleep(0.3)
    return False


def wait_flow_done(timeout=120.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = http_json("http://127.0.0.1:%d/api/flow/state" % WS_PORT)
        if s.get("state") in ("done", "aborted"):
            return s
        time.sleep(0.5)
    return {"state": "timeout"}


def kill_old_runtime():
    """如果 9010 被旧 runtime 占用，结束它（避免 Windows SO_REUSEADDR 双绑）。"""
    try:
        http_json("http://127.0.0.1:%d/api/health" % WS_PORT)
    except Exception:
        return
    print("[warn] 9010 已被占用，尝试结束旧 runtime...")
    try:
        out = subprocess.check_output(
            ["netstat", "-ano"], shell=False, text=True, errors="replace")
        pid = None
        for line in out.splitlines():
            if ":9010" in line and "LISTENING" in line:
                parts = line.strip().split()
                if parts:
                    pid = parts[-1]
                    break
        if pid:
            subprocess.call(["taskkill", "//F", "//PID", pid],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
    except Exception as e:
        print("[warn] 结束旧进程失败：%s" % e)


def main():
    if not os.path.exists(EDGE):
        print("[FAIL] Edge 不存在：%s" % EDGE)
        return 1
    if not os.path.exists(HTML):
        print("[FAIL] 联机版 HTML 不存在：%s" % HTML)
        return 1
    os.makedirs(SHOT_DIR, exist_ok=True)

    kill_old_runtime()

    # 1) 启动后端
    proc = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "runtime", "main.py")],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")

        base = "http://127.0.0.1:%d" % WS_PORT

        # 1.5) 轴预置：使能 + 回零关键轴（流程 move 依赖 in_position）
        home_axes = ["AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y", "WS_Rz"]
        for ax in home_axes + ["AS_Z", "FA_X", "FA_Z", "BS_Arm_Z"]:
            http_post(base + "/api/fault", {"action": "enable", "axis": ax})
        print("[ok] 关键轴已使能")
        for ax in home_axes:
            http_post(base + "/api/axis/home", {"axis": ax})
        print("[wait] 等关键轴回零...")
        t0 = time.time()
        while time.time() - t0 < 90.0:
            st = http_json(base + "/api/axes")
            states = {}
            for i, a in enumerate(st["axes"]):
                if a["name"] in home_axes:
                    states[a["name"]] = a["home_state"]
            if all(v == "done" for v in states.values()):
                print("[ok] 关键轴回零完成：%s" % states)
                break
            if time.time() - t0 > 5.0 and int(time.time() - t0) % 5 == 0:
                print("   ... 仍在回零 %s" % states)
            time.sleep(0.5)
        else:
            print("[FAIL] 回零超时")
            return 1

        # 2) 复位 + 注入确定性误差
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        injected = {
            "load_x_mm": 0.2,
            "load_y_mm": -0.15,
            "load_rz_rad": 0.0002,
            "jitter_xy_um": 0.0,
            "jitter_rz_urad": 0.0,
            "jitter_on": False,
        }
        c, r = http_put(base + "/api/recipe", {"wafer_load": injected})
        if c != 200 or not r.get("ok"):
            print("[FAIL] 处方注入失败 %s %s" % (c, r))
            return 1
        print("[ok] 注入 wafer_load 误差", injected)

        # 3) 跑 EGA 流程（强制上片）
        c, r = http_post(base + "/api/process/wafer_ega", {"with_load": True})
        if c != 200 or not r.get("ok"):
            print("[FAIL] EGA 流程启动失败 %s %s" % (c, r))
            return 1
        print("[ok] EGA 流程已启动，等待完成...")
        s = wait_flow_done()

        # 为截图验证晶圆旋转：先拍一张 Rz=0，再转到 0.01 rad（约 0.57°）
        # 拍第二张，使晶圆上 die 阵列方向变化可见；拍完照后回 0。
        print("[ok] EGA 完成，准备对比截图（Rz=0 vs Rz=0.01 rad）...")
        ega = s.get("ega_result") or {}
        print("[ok] EGA 状态=%s n=%d tx=%+.4f um ty=%+.4f um theta=%+.4f urad resid_3s=%.3f um"
              % (s.get("state"), ega.get("n", 0),
                 ega.get("tx", 0.0) * 1000, ega.get("ty", 0.0) * 1000,
                 ega.get("theta", 0.0) * 1e6, ega.get("resid_3s", 0.0) * 1000))
        if s.get("state") != "done":
            print("--- 流程失败步骤 ---")
            for step in s.get("steps", []):
                if step.get("status") != "ok":
                    print("  #%d %s %s: %s" % (step.get("i"), step.get("op"),
                                               step.get("status"), step.get("info")))
            for ln in s.get("log", [])[-20:]:
                print("  log [%s] %s" % (ln.get("level"), ln.get("msg")))

        # 4) Edge headless 打开联机版并截图
        profile = tempfile.mkdtemp(prefix="edge_ega_")
        url = "file:///" + HTML.replace(os.sep, "/")
        edge_proc = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check", "--remote-debugging-port=%d" % CDP_PORT,
             "--user-data-dir=" + profile, "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            t0 = time.time()
            while time.time() - t0 < 20:
                try:
                    urllib.request.urlopen("http://127.0.0.1:%d/json/version" % CDP_PORT, timeout=2)
                    break
                except Exception:
                    time.sleep(0.3)
            from urllib.parse import quote
            req = urllib.request.Request(
                "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
                method="PUT")
            target = json.loads(urllib.request.urlopen(req, timeout=5).read().decode())
            cdp = CDP(target["webSocketDebuggerUrl"])
            try:
                cdp.call("Runtime.enable")
                print("[wait] 页面运行 8s（等查看器初始化 + WS 连接）...")
                time.sleep(8)

                def capture(name):
                    res = cdp.call("Page.captureScreenshot", {"format": "png"})
                    p = os.path.join(SHOT_DIR, "%s_%s.png" % (name, ts))
                    with open(p, "wb") as f:
                        f.write(base64.b64decode(res["data"]))
                    print("[ok] 截图已保存：%s" % p)
                    return p

                ts = time.strftime("%Y%m%d_%H%M%S")
                # 图 1：EGA 完成后 Rz=0
                png0 = capture("ega_rz0")

                # 转动 WS_Rz 再拍一张
                print("[ok] 转动 WS_Rz 到 0.01 rad 拍对比图...")
                http_post(base + "/api/axis/move",
                          {"axis": "WS_Rz", "target": 0.01, "speed": 0.5})
                t0 = time.time()
                while time.time() - t0 < 10.0:
                    st = http_json(base + "/api/axes")
                    idx = next(i for i, a in enumerate(st["axes"])
                               if a["name"] == "WS_Rz")
                    if st["axes"][idx]["settled"]:
                        break
                    time.sleep(0.1)
                time.sleep(1.0)   # 等前端渲染一帧
                png1 = capture("ega_rz001")
                png_path = png1

                # 抽取 HUD 与场景信息
                eval_js = r"""
(function () {
  var out = { hudText: null, tdviz: null, dataViz: null, errorBox: null };
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
  out.tdviz = window.__TD_VIZ__ ? Object.keys(window.__TD_VIZ__) : null;
  return JSON.stringify(out);
})()
"""
                res = cdp.call("Runtime.evaluate",
                               {"expression": eval_js, "returnByValue": True})
                info = json.loads(res["result"]["value"])
            finally:
                cdp.close()
        finally:
            edge_proc.terminate()
            try:
                edge_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                edge_proc.kill()
            shutil.rmtree(profile, ignore_errors=True)

        # 5) 判定
        ok = True
        print("\n===== 判定 =====")
        print("EGA state     : %s" % s.get("state"))
        print("EGA n         : %s" % ega.get("n"))
        print("EGA tx/ty/θ   : %+.4f um / %+.4f um / %+.4f urad"
              % (ega.get("tx", 0.0) * 1000, ega.get("ty", 0.0) * 1000,
                 ega.get("theta", 0.0) * 1e6))
        print("EGA resid_3s  : %.3f um" % (ega.get("resid_3s", 0.0) * 1000))
        print("HUD connected : %s" % ("已连接" in (info.get("hudText") or "")))
        print("__TD_VIZ__    : %s" % info.get("tdviz"))
        print("errorBox      : %s" % (info.get("errorBox") or "无"))

        if s.get("state") != "done":
            print("[FAIL] EGA 流程未正常完成"); ok = False
        if abs(ega.get("tx", 0.0) - injected["load_x_mm"]) >= 0.01:
            print("[FAIL] tx 偏离注入值"); ok = False
        if abs(ega.get("ty", 0.0) - injected["load_y_mm"]) >= 0.01:
            print("[FAIL] ty 偏离注入值"); ok = False
        if abs(ega.get("theta", 0.0) - injected["load_rz_rad"]) >= 5e-5:
            print("[FAIL] theta 偏离注入值"); ok = False
        if not info.get("tdviz"):
            print("[FAIL] 联机版 3D 场景钩子未注入"); ok = False
        if info.get("errorBox"):
            print("[FAIL] 查看器报错：%s" % info.get("errorBox")); ok = False

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
