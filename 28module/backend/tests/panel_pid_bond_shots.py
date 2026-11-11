#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""panel_pid_bond_shots.py — Edge headless + CDP 截图验证迭代 8B
（2026-08-28）：

  1. "温控压控"页（?tab=pid）：三回路卡片可见；对脉冲加热器下发 300℃
     设定（SetTargetTempWithPlan），升温中截趋势曲线
     → Temp/panel_pid_<ts>.png；到带后再截一张（SP 虚线+PV 实线收敛）；
  2. 联机版 3D 脉冲加热器发光：加热中打开联机版，data-viz 断言
     heaterFound=1 且 heaterTemp 明显 >25℃ → Temp/online_heater_<ts>.png；
  3. chain 六阶段（含 bond）：POST /api/process/chain，bond 阶段运行中截
     六格进度 → Temp/panel_chain6_<ts>.png；完成后工艺页日志区截 bond
     流程日志（升温/下压/加压/保压/释放/交接可见）
     → Temp/panel_bond_<ts>.png；
  4. 判定：pid head_temp 到带；data-viz heaterFound/heaterTemp；chain
     六阶段全 done/skip 且 bond=done；bond 日志含关键步骤词；die 落
     晶圆（carrier=wafer）。

用法：python backend/tests/panel_pid_bond_shots.py
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
from urllib.parse import quote

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402
from tests.panel_fl_verify_shots import (  # noqa: E402
    EDGE, CDP_PORT, PANEL, SHOT_DIR, http_json, http_post, http_put,
    wait_ready, kill_old_runtime, run_flow)

WS_PORT = 9010
ONLINE_HTML = os.path.abspath(os.path.join(
    ROOT, "..", "html_model", "d2w_online.html"))

PID_TAB_JS = r"""
(function () {
  location.hash = '';
  document.getElementById('tabPid').click();
  var cv = document.getElementById('pidHeadCv');
  if (cv && cv.scrollIntoView) cv.scrollIntoView({block: 'center'});
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

BOND_LOG_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var steps = document.getElementById('flowSteps');
  if (steps) {
    var rows = steps.querySelectorAll('*');
    var hit = null;
    for (var i = 0; i < rows.length; i++) {
      if (rows[i].textContent.indexOf('加压闭环') >= 0 &&
          rows[i].children.length < 4) hit = rows[i];
    }
    if (hit && hit.scrollIntoView) hit.scrollIntoView({block: 'center'});
  }
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  return 'ok';
})()
"""

DATA_VIZ_JS = r"""
(function () {
  var hud = null;
  var divs = document.querySelectorAll('div');
  for (var i = 0; i < divs.length; i++) {
    if (divs[i].textContent.indexOf('联机状态') >= 0 &&
        divs[i].style.zIndex === '9999') { hud = divs[i]; break; }
  }
  var raw = hud ? hud.getAttribute('data-viz') : null;
  var viz = raw ? JSON.parse(raw) : {};
  return JSON.stringify({heaterFound: viz.heaterFound,
                         heaterTemp: viz.heaterTemp,
                         mode: viz.mode, active: viz.active});
})()
"""


def shot(cdp, name, ts):
    data = cdp.call("Page.captureScreenshot", {"format": "png"})["data"]
    p = os.path.join(SHOT_DIR, "%s_%s.png" % (name, ts))
    with open(p, "wb") as f:
        f.write(base64.b64decode(data))
    print("[ok] 截图已保存：%s" % p)
    return p


def new_tab(url):
    req = urllib.request.Request(
        "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
        method="PUT")
    target = json.loads(
        urllib.request.urlopen(req, timeout=5).read().decode())
    return CDP(target["webSocketDebuggerUrl"])


def flow_state(base):
    return http_json(base + "/api/flow/state")


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
    profile = tempfile.mkdtemp(prefix="edge_panel_pid_")
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # ---- 确定性注入（同 panel_chain_shots） ----
        http_put(base + "/api/recipe", {
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5},
            "wafer_load": {
                "load_x_mm": 0.2, "load_y_mm": -0.15,
                "load_rz_rad": 0.0002, "jitter_on": False,
                "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0}})

        # ---- Edge headless ----
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

        ts = time.strftime("%Y%m%d_%H%M%S")
        pid_ok = False
        heater_viz = {}
        ch = {}
        bond_logs = ""
        die_on_wafer = False

        # ================= 1. PID 页：脉冲加热 300℃ 升温曲线 =================
        panel_url = "file:///" + PANEL.replace(os.sep, "/") + "?tab=pid"
        cdp = new_tab(panel_url)
        try:
            cdp.call("Runtime.enable")
            print("[wait] 面板 PID 页运行 5s（等首轮填充）...")
            time.sleep(5)
            # 下发 300℃ 设定（与面板"下发温度"同命令面）
            c, r = http_post(base + "/api/io/command",
                             {"path": "SystemMachine.PulseHeat_Up",
                              "cmd": "SetTargetTempWithPlan",
                              "args": {"Temperature": 300, "Steady_Time": 0}})
            print("[pid] SetTargetTempWithPlan 300℃ -> %s %s" % (c, r.get("ok")))
            # 升温中段截一张（曲线爬升可见）
            time.sleep(4)
            cdp.call("Runtime.evaluate",
                     {"expression": PID_TAB_JS, "returnByValue": True})
            time.sleep(1.0)
            shots.append(shot(cdp, "panel_pid", ts))
            # 等到带（±2℃）
            t0 = time.time()
            pv = None
            while time.time() - t0 < 60:
                pid = flow_state(base).get("pid") or {}
                ht = pid.get("head_temp") or {}
                pv = ht.get("pv")
                if pv is not None and abs(pv - 300.0) <= 2.0:
                    pid_ok = True
                    break
                time.sleep(0.5)
            print("[pid] head_temp pv=%s 到带=%s" % (pv, pid_ok))
            time.sleep(1.5)   # 让面板曲线补上收敛段
            shots.append(shot(cdp, "panel_pid_intemp", ts))
        finally:
            cdp.close()

        # ================= 2. 联机版 3D 加热器发光（仍在 300℃ 保温中） =================
        online_url = "file:///" + ONLINE_HTML.replace(os.sep, "/")
        cdp2 = new_tab(online_url)
        try:
            cdp2.call("Runtime.enable")
            print("[wait] 联机版运行 10s（等 WS 连接+首轮 viz）...")
            time.sleep(10)
            res = cdp2.call("Runtime.evaluate",
                            {"expression": DATA_VIZ_JS,
                             "returnByValue": True})
            heater_viz = json.loads(res["result"]["value"])
            print("[viz] heater data-viz=%s" % heater_viz)
            shots.append(shot(cdp2, "online_heater", ts))
        finally:
            cdp2.close()
        # 关掉脉冲加热器（避免影响 bond 段时序判读）
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.PulseHeat_Up",
                   "cmd": "StopTargetTempWithPlan", "args": {}})

        # ================= 3. chain 六阶段（含 bond） =================
        panel2_url = "file:///" + PANEL.replace(os.sep, "/") + "?tab=flow"
        cdp3 = new_tab(panel2_url)
        try:
            cdp3.call("Runtime.enable")
            time.sleep(4)
            http_post(base + "/api/workpiece/wafer", {"action": "unload"})
            c, r = http_post(base + "/api/process/chain", {})
            print("[chain] POST -> %s %s" % (c, r.get("ok")))
            started = False
            t0 = time.time()
            while time.time() - t0 < 30:
                ch = flow_state(base).get("chain") or {}
                if ch.get("active"):
                    started = True
                    break
                time.sleep(0.5)
            progressed = False
            st = {}
            t0 = time.time()
            while started and time.time() - t0 < 1200:
                st = flow_state(base)
                ch = st.get("chain") or {}
                if not progressed and ch.get("current") == "bond":
                    time.sleep(2.0)      # 等面板轮询刷新+bond 升温起步
                    cdp3.call("Runtime.evaluate",
                              {"expression": CHAIN_SCROLL_JS,
                               "returnByValue": True})
                    time.sleep(0.8)
                    shots.append(shot(cdp3, "panel_chain6", ts))
                    progressed = True
                    print("[chain] 六阶段进度截图（当前阶段 bond）")
                if not ch.get("active"):
                    break
                time.sleep(1.0)
            ch = (st.get("chain") or ch) or {}
            bond_logs = "\n".join(
                str(e.get("msg", "")) + " " + str(e.get("desc", ""))
                for e in (st.get("log") or []))
            bond_logs += "\n" + "\n".join(
                str(x.get("desc", "")) for x in (st.get("steps") or []))
            time.sleep(2.5)
            cdp3.call("Runtime.evaluate",
                      {"expression": BOND_LOG_JS, "returnByValue": True})
            time.sleep(1.0)
            shots.append(shot(cdp3, "panel_bond", ts))
        finally:
            cdp3.close()

        # die 落晶圆确认
        try:
            viz = http_json(base + "/api/viz")
            die_on_wafer = any((d.get("carrier") == "wafer")
                               for d in (viz.get("dies") or []))
        except Exception:
            pass

        # ================= 判定 =================
        ok = True
        print("\n===== 判定 =====")
        print("PID head_temp 到带（±2℃）：%s" % pid_ok)
        ok = ok and pid_ok
        viz_ok = (heater_viz.get("heaterFound") == 1
                  and (heater_viz.get("heaterTemp") or 0) > 100)
        print("联机版 heaterFound=%s heaterTemp=%s（期望 1 且 >100℃）：%s"
              % (heater_viz.get("heaterFound"), heater_viz.get("heaterTemp"),
                 viz_ok))
        ok = ok and viz_ok
        st_all = ch.get("stages") or {}
        six = ("wafer_ega", "flip_pick", "flip_handoff", "focus_level",
               "fine_align", "bond")
        all_ok = all(st_all.get(k) in ("done", "skip") for k in six) \
            and st_all.get("bond") == "done"
        print("chain.stages=%s（bond 须 done）：%s" % (st_all, all_ok))
        ok = ok and all_ok and not ch.get("error")
        kws = [k for k in ("升温", "下压", "加压", "保压", "卸力", "吹离",
                           "交接", "落位") if k in bond_logs]
        log_ok = len(kws) >= 4
        print("bond 日志关键词命中=%s（≥4 个过关）：%s" % (kws, log_ok))
        ok = ok and log_ok
        print("die 落晶圆（carrier=wafer）：%s" % die_on_wafer)
        ok = ok and die_on_wafer
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
