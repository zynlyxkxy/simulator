#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_chain_shots.py — Edge headless + CDP 截图验证迭代 7 一键串联
（2026-08-28）：

  1. 交接正压吹离：flip_pick → flip_handoff 实跑，流程日志含"正压吹离"，
     截工艺页日志区 → Temp/panel_handoff_blow_<ts>.png；
  2. 一键串联：面板"一键串联运行"按钮（CDP 真点击）启动五阶段链，
     运行中截阶段进度（运行/等待可见）→ Temp/panel_chain_progress_<ts>.png；
     链完成后滚到精对准结果区 → Temp/panel_chain_done_<ts>.png；
  3. 判定：日志含"正压吹离"；chain.stages 五阶段全 done；
     fine_align_result.judge=pass。

用法：python backend/tests/panel_chain_shots.py
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
  var steps = document.getElementById('flowSteps');
  if (steps) {
    // 滚到含"正压吹离"的步骤行（交接流程步骤 desc 可视化验证）
    var rows = steps.querySelectorAll('*');
    var hit = null;
    for (var i = 0; i < rows.length; i++) {
      if (rows[i].textContent.indexOf('正压吹离') >= 0 &&
          rows[i].children.length < 4) hit = rows[i];
    }
    if (hit && hit.scrollIntoView) hit.scrollIntoView({block: 'center'});
    else if (steps.scrollIntoView) steps.scrollIntoView({block: 'end'});
  }
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  return 'ok';
})()
"""

CHAIN_SCROLL_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var card = document.getElementById('chainStages');
  if (card && card.scrollIntoView) card.scrollIntoView({block: 'center'});
  return 'ok';
})()
"""

FA_SCROLL_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var card = document.getElementById('faResult');
  if (card && card.scrollIntoView) card.scrollIntoView({block: 'center'});
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  return 'ok';
})()
"""

CHAIN_CLICK_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  document.getElementById('btnChainRun').click();
  return 'clicked';
})()
"""


def shot(cdp, name, ts):
    data = cdp.call("Page.captureScreenshot", {"format": "png"})["data"]
    p = os.path.join(SHOT_DIR, "%s_%s.png" % (name, ts))
    with open(p, "wb") as f:
        f.write(base64.b64decode(data))
    print("[ok] 截图已保存：%s" % p)
    return p


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

        # ---- 确定性注入 ----
        http_put(base + "/api/recipe", {
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5},
            "wafer_load": {
                "load_x_mm": 0.2, "load_y_mm": -0.15,
                "load_rz_rad": 0.0002, "jitter_on": False,
                "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0}})

        # ---- Edge headless 打开面板 ----
        profile = tempfile.mkdtemp(prefix="edge_panel_chain_")
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
        blow_logs = ""
        ch = {}
        fa = {}
        try:
            cdp.call("Runtime.enable")
            print("[wait] 面板运行 5s（等首轮填充）...")
            time.sleep(5)

            # ---- 1. 交接正压吹离：取片 → 交接，日志含"正压吹离" ----
            print("[flow] flip_pick / flip_handoff（正压吹离）...")
            run_flow(base, "/api/process/flip_pick", {})
            s = run_flow(base, "/api/process/flip_handoff", {})
            blow_logs = "\n".join(
                str(e.get("msg", "")) + " " + str(e.get("desc", ""))
                for e in (s.get("log") or []))
            # steps 的 desc 在 state.steps 里
            blow_logs += "\n" + "\n".join(
                str(st.get("desc", "")) for st in (s.get("steps") or []))
            time.sleep(2.5)
            cdp.call("Runtime.evaluate",
                     {"expression": FLOW_TAB_JS, "returnByValue": True})
            time.sleep(1.0)
            shots.append(shot(cdp, "panel_handoff_blow", ts))

            # ---- 2. 一键串联：面板按钮真点击（晶圆卸掉让 EGA 阶段也跑） ----
            http_post(base + "/api/workpiece/wafer", {"action": "unload"})
            cdp.call("Runtime.evaluate",
                     {"expression": CHAIN_CLICK_JS, "returnByValue": True})
            print("[chain] 面板按钮已点击，等链启动...")
            # 先等链真的启动（active=True；面板按钮走 POST 有往返延迟）
            started = False
            t0 = time.time()
            while time.time() - t0 < 30:
                ch = (http_json(base + "/api/flow/state")
                      .get("chain") or {})
                if ch.get("active"):
                    started = True
                    break
                time.sleep(0.5)
            print("[chain] 启动=%s stages=%s"
                  % (started, (ch or {}).get("stages")))
            progressed = False
            st = http_json(base + "/api/flow/state")
            t0 = time.time()
            while started and time.time() - t0 < 900:
                st = http_json(base + "/api/flow/state")
                ch = st.get("chain") or {}
                cur = ch.get("current")
                if not progressed and cur in ("flip_handoff", "focus_level",
                                              "fine_align"):
                    time.sleep(1.0)      # 等面板轮询刷新阶段行
                    cdp.call("Runtime.evaluate",
                             {"expression": CHAIN_SCROLL_JS,
                              "returnByValue": True})
                    time.sleep(0.8)
                    shots.append(shot(cdp, "panel_chain_progress", ts))
                    progressed = True
                    print("[chain] 进度截图（当前阶段 %s）" % cur)
                if not ch.get("active"):
                    break
                time.sleep(1.0)
            fa = st.get("fine_align_result") or {}
            time.sleep(2.5)
            cdp.call("Runtime.evaluate",
                     {"expression": FA_SCROLL_JS, "returnByValue": True})
            time.sleep(1.0)
            shots.append(shot(cdp, "panel_chain_done", ts))
        finally:
            cdp.close()

        # ---- 判定 ----
        ok = started
        print("\n===== 判定 =====")
        print("链启动：%s" % started)
        hit = "正压吹离" in blow_logs
        print("交接日志含 '正压吹离'：%s" % hit)
        ok = ok and hit
        st_all = ch.get("stages") or {}
        # done/skip 都合法（状态感知跳过是设计行为）；不允许 fail/wait 残留
        all_ok = all(st_all.get(k) in ("done", "skip") for k in
                     ("wafer_ega", "flip_pick", "flip_handoff",
                      "focus_level", "fine_align")) \
            and st_all.get("fine_align") == "done"
        print("chain.stages=%s（done/skip 合法，fine_align 须 done）：%s"
              % (st_all, all_ok))
        ok = ok and all_ok and not ch.get("error")
        print("judge=%s（期望 pass）" % fa.get("judge"))
        ok = ok and fa.get("judge") == "pass"
        print("截图：%s" % shots)
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
