#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_fa_shots.py — Edge headless + CDP 截图验证迭代 6 逐键合位精对准
（2026-08-28）：

  面板工艺页实跑精对准（API 触发，面板流程日志区与"精对准结果"行可见）：
    - 确定性注入（jitter 关、σ=0）→ 取片→交接→EGA→per_site 调焦调平→
      fine_align → 日志应见"精对准测量/解算/补偿…耦合 coup…/确认 第1轮…
      PASS/精对准完成…物理残差"，fine_align_result.judge=pass；
    - 截工艺页（流程日志滚到底 + 精对准卡结果区）；
    - 切视觉页截上/下相机实时视野（up 看 die 角标、down 看 site 标记）。
  截图存 Temp/panel_fa_<场景>_<ts>.png，并断言关键日志文本与 judge。

用法：python backend/tests/panel_fa_shots.py
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
from tests.panel_fl_verify_shots import (  # noqa: E402
    EDGE, CDP_PORT, PANEL, SHOT_DIR, http_json, http_post, http_put,
    wait_ready, kill_old_runtime, run_flow)

WS_PORT = 9010

FLOW_TAB_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  var card = document.getElementById('faResult');
  if (card && card.scrollIntoView) card.scrollIntoView({block: 'center'});
  var box2 = document.getElementById('flowLog');
  if (box2) box2.scrollTop = box2.scrollHeight;
  return 'ok';
})()
"""

VISION_TAB_JS = r"""
(function () {
  document.getElementById('tabVision').click();
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

        # ---- 确定性注入（确认轮次放宽到 5：σθ≈17urad 噪声尾巴兜底，
        #      截图场景要确定性 PASS）----
        http_put(base + "/api/recipe", {
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5},
            "wafer_load": {
                "load_x_mm": 0.2, "load_y_mm": -0.15,
                "load_rz_rad": 0.0002, "jitter_on": False,
                "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0}})

        # ---- 前置链 ----
        print("[flow] flip_pick / flip_handoff / wafer_ega / focus_level ...")
        run_flow(base, "/api/process/flip_pick", {})
        run_flow(base, "/api/process/flip_handoff", {})
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        run_flow(base, "/api/process/wafer_ega", {"with_load": True})
        run_flow(base, "/api/process/focus_level", {"mode": "per_site"})

        # ---- Edge headless 打开面板 ----
        profile = tempfile.mkdtemp(prefix="edge_panel_fa_")
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
        fa = {}
        logs = ""
        try:
            cdp.call("Runtime.enable")
            print("[wait] 面板运行 5s（等首轮填充）...")
            time.sleep(5)

            print("[flow] fine_align ...")
            s = run_flow(base, "/api/process/fine_align", {})
            logs = "\n".join(str(e.get("msg", ""))
                             for e in (s.get("log") or []))
            fa = s.get("fine_align_result") or {}
            time.sleep(2.5)          # 等面板轮询刷新流程日志/结果区
            cdp.call("Runtime.evaluate",
                     {"expression": FLOW_TAB_JS, "returnByValue": True})
            time.sleep(1.0)
            shot = cdp.call("Page.captureScreenshot", {"format": "png"})
            p = os.path.join(SHOT_DIR, "panel_fa_flow_%s.png" % ts)
            with open(p, "wb") as f:
                f.write(base64.b64decode(shot["data"]))
            shots.append(p)
            print("[ok] 截图已保存：%s" % p)

            # ---- 视觉页：上/下相机视野（开双光源 + AS 到吸盘心对准位）----
            for lp in ("SystemMachine.SS_Bonding.M_AS.LightOn_1",
                       "SystemMachine.SS_Bonding.M_AS.LightOn_2"):
                http_post(base + "/api/io/command",
                          {"path": lp, "cmd": "SetBool",
                           "args": {"value": True}})
            for ax, tgt in (("AS_X", 3.8), ("AS_Y", 9.8)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 100.0})
            time.sleep(2.5)
            cdp.call("Runtime.evaluate",
                     {"expression": VISION_TAB_JS, "returnByValue": True})
            print("[wait] 视觉页取帧 6s（~4.5Hz 刷两帧）...")
            time.sleep(6)
            shot = cdp.call("Page.captureScreenshot", {"format": "png"})
            p = os.path.join(SHOT_DIR, "panel_fa_cams_%s.png" % ts)
            with open(p, "wb") as f:
                f.write(base64.b64decode(shot["data"]))
            shots.append(p)
            print("[ok] 截图已保存：%s" % p)
        finally:
            cdp.close()

        # ---- 判定 ----
        ok = True
        print("\n===== 判定 =====")
        need = ["精对准测量完成", "精对准解算", "耦合", "精对准确认 第1轮",
                "物理残差"]
        for kw in need:
            hit = kw in logs
            print("日志含 %r：%s" % (kw, hit))
            ok = ok and hit
        print("judge=%s（期望 pass）" % fa.get("judge"))
        ok = ok and fa.get("judge") == "pass"
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
