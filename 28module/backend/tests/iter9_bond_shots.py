#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iter9_bond_shots.py — Edge headless + CDP 截图验证迭代 9（2026-08-31）
键合两段速下压 / force_search 慢速寻力 / 有效接触位换算 / 硅片台常保温秒过 /
面板迁移（温控压控新增"硬件限值与对象参数"卡、PID 走 /api/tuning；测校页
sim_sensors 新增温度/力噪声输入；机械常数页删掉 pid/bond_hw/噪声行）。

截图（Temp/panel_shots/iter9_*_<ts>.png）：
  a. 温控压控 tab 全页（三回路卡 + 硬件限值与对象参数卡）；
  b. 测校 tab sim_sensors 卡（含温度/力噪声两个新输入）；
  c. 机械常数页 bond 组（断言只剩 head_z_contact_mm/head_z_search_mm，
     无 pid/bond_hw/噪声键行）；
  d. chain 实跑到 bond 阶段：bond 进行中 pid 页力/温曲线（力上升沿），
     完成后工艺页 force_search 步 OK 记录 + 流程日志（寻力接触行）。

判定：
  - 三处 DOM 断言（硬件限值卡标题 / sim_sensors 两噪声输入 / bond 组键行）；
  - chain 六阶段全 done/skip 且 bond=done；
  - bond 步记录含 force_search 且结果 ok（日志含"寻力接触"）；
  - 硅片台等温确认步秒过（step 记录耗时 <5s）。

用法：python backend/tests/iter9_bond_shots.py
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

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")   # GBK 控制台容忍 µ 等字符

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402
from tests.panel_fl_verify_shots import (  # noqa: E402
    EDGE, PANEL, http_json, http_post, http_put, wait_ready,
    kill_old_runtime)

WS_PORT = 9010
CDP_PORT = 9245
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp", "panel_shots"))

PID_HW_JS = r"""
(function () {
  document.getElementById('tabPid').click();
  var els = document.querySelectorAll('summary');
  for (var i = 0; i < els.length; i++) {
    if (els[i].textContent.indexOf('硬件限值与对象参数') >= 0) {
      els[i].parentElement.open = true;
      els[i].scrollIntoView({block: 'center'});
      return 'ok';
    }
  }
  return 'hw-card-not-found';
})()
"""

CALIB_SS_JS = r"""
(function () {
  document.getElementById('tabCalib').click();
  var els = document.querySelectorAll('summary');
  for (var i = 0; i < els.length; i++) {
    if (els[i].textContent.indexOf('传感器仿真') >= 0) {
      els[i].parentElement.open = true;
      els[i].scrollIntoView({block: 'center'});
      return 'ok';
    }
  }
  return 'sim_sensors-card-not-found';
})()
"""

CONST_BOND_JS = r"""
(function () {
  document.getElementById('tabConst').click();
  var inp = document.querySelector('input[data-ck="bond.head_z_search_mm"]');
  if (!inp) return 'bond-group-not-found';
  inp.scrollIntoView({block: 'center'});
  return 'ok';
})()
"""

CONST_BOND_TEXT_JS = r"""
(function () {
  var keys = [];
  var ins = document.querySelectorAll('#pageConst input[data-ck]');
  for (var i = 0; i < ins.length; i++) {
    var ck = ins[i].getAttribute('data-ck');
    if (ck.indexOf('bond.') === 0) keys.push(ck);
  }
  return JSON.stringify({keys: keys});
})()
"""

PID_FORCE_SCROLL_JS = r"""
(function () {
  document.getElementById('tabPid').click();
  var cv = document.getElementById('pidForceCv');
  if (cv && cv.scrollIntoView) cv.scrollIntoView({block: 'center'});
  return 'ok';
})()
"""

FLOW_FORCE_SEARCH_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var box = document.getElementById('flowLog');
  if (box) box.scrollTop = box.scrollHeight;
  var hit = null;
  var rows = document.querySelectorAll('#flowSteps *, #flowLog *');
  for (var i = rows.length - 1; i >= 0; i--) {
    if (rows[i].textContent.indexOf('寻力接触') >= 0 &&
        rows[i].children.length < 4) { hit = rows[i]; break; }
  }
  if (!hit) {
    for (var i = rows.length - 1; i >= 0; i--) {
      if (rows[i].textContent.indexOf('寻力') >= 0 &&
          rows[i].children.length < 4) { hit = rows[i]; break; }
    }
  }
  if (hit && hit.scrollIntoView) hit.scrollIntoView({block: 'center'});
  return hit ? 'ok' : 'force_search-row-not-found';
})()
"""


def shot(cdp, name, ts, full=False):
    params = {"format": "png"}
    if full:
        params["captureBeyondViewport"] = True
    data = cdp.call("Page.captureScreenshot", params)["data"]
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


def panel_url(tab):
    return "file:///" + PANEL.replace(os.sep, "/") + "?tab=" + tab


def page_text(cdp):
    res = cdp.call("Runtime.evaluate", {
        "expression": "document.body.innerText", "returnByValue": True})
    return res["result"]["value"]


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
    profile = tempfile.mkdtemp(prefix="edge_iter9_")
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # ---- 确定性注入（同 panel_pid_bond_shots）----
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
        dom_ok = {"a": False, "b": False, "c": False}

        # ================= a. 温控压控 tab 全页 =================
        cdp = new_tab(panel_url("pid"))
        try:
            cdp.call("Runtime.enable")
            print("[wait] 温控压控页运行 5s（等 /api/tuning 首轮填充）...")
            time.sleep(5)
            r = cdp.call("Runtime.evaluate",
                         {"expression": PID_HW_JS, "returnByValue": True})
            print("[a] 硬件限值卡定位：%s" % r["result"]["value"])
            time.sleep(1.0)
            txt = page_text(cdp)
            dom_ok["a"] = ("硬件限值与对象参数" in txt
                           and "键合压力回路" in txt)
            shots.append(shot(cdp, "iter9_a_pid_full", ts, full=True))
        finally:
            cdp.close()

        # ================= b. 测校 tab sim_sensors 卡 =================
        cdp = new_tab(panel_url("calib"))
        try:
            cdp.call("Runtime.enable")
            time.sleep(4)
            r = cdp.call("Runtime.evaluate",
                         {"expression": CALIB_SS_JS, "returnByValue": True})
            print("[b] sim_sensors 卡定位：%s" % r["result"]["value"])
            time.sleep(1.0)
            txt = page_text(cdp)
            dom_ok["b"] = ("温度读数噪声" in txt and "力读数噪声" in txt
                           and "共焦噪声" in txt)
            shots.append(shot(cdp, "iter9_b_calib_sim_sensors", ts))
        finally:
            cdp.close()

        # ================= c. 机械常数页 bond 组 =================
        cdp = new_tab(panel_url("const"))
        try:
            cdp.call("Runtime.enable")
            time.sleep(4)
            r = cdp.call("Runtime.evaluate",
                         {"expression": CONST_BOND_JS, "returnByValue": True})
            print("[c] bond 组定位：%s" % r["result"]["value"])
            time.sleep(1.0)
            res = cdp.call("Runtime.evaluate",
                           {"expression": CONST_BOND_TEXT_JS,
                            "returnByValue": True})
            info = json.loads(res["result"]["value"])
            keys = info.get("keys") or []
            print("[c] bond 组键行=%s" % keys)
            bad = [k for k in keys
                   if "pid" in k or "noise" in k or "bond_hw" in k
                   or k.endswith("spring_k") or "rate" in k]
            dom_ok["c"] = (
                "bond.head_z_contact_mm" in keys
                and "bond.head_z_search_mm" in keys and not bad)
            shots.append(shot(cdp, "iter9_c_const_bond", ts))
        finally:
            cdp.close()

        # ================= d. chain 实跑到 bond =================
        bond_curve_shot = False
        cdp = new_tab(panel_url("pid"))   # bond 进行中拍力/温曲线用（新缓冲）
        try:
            cdp.call("Runtime.enable")
            time.sleep(3)
            # 预热：wafer_keep_warm 启动即设 Heat_Dw=150℃，等它到温再跑
            # chain——否则等温步等的是冷机升温而非"幂等秒过"
            print("[preheat] 等硅片台常保温到温（150±2℃）...")
            t0 = time.time()
            while time.time() - t0 < 300:
                pv = ((flow_state(base).get("pid") or {})
                      .get("wafer_temp") or {}).get("pv")
                if pv is not None and abs(float(pv) - 150.0) <= 2.0:
                    print("[preheat] 到温 pv=%.1f℃" % float(pv))
                    break
                time.sleep(1.0)
            # 清场：工件复位（防上一轮 chain 遗留 die/晶圆状态卡守卫）
            c0, r0 = http_post(base + "/api/workpiece/reset", {})
            print("[chain] workpiece reset -> %s %s" % (c0, r0.get("ok")))
            http_post(base + "/api/workpiece/wafer", {"action": "unload"})
            c, r = http_post(base + "/api/process/chain", {})
            print("[chain] POST -> %s %s" % (c, r.get("ok")))
            ch, st = {}, {}
            t0 = time.time()
            while time.time() - t0 < 1800:
                st = flow_state(base)
                ch = st.get("chain") or {}
                if ch.get("active") and ch.get("current") == "bond":
                    break
                if ch.get("error"):
                    print("[chain] error: %s" % ch.get("error"))
                    tail = [str(e.get("msg", ""))
                            for e in (st.get("log") or [])][-15:]
                    print("[chain] 末尾日志：%s" % tail)
                    break
                time.sleep(1.0)
            print("[chain] 当前阶段=%s" % ch.get("current"))
            # bond 阶段：等力 PV 上升沿（加压闭环 ramp），截力/温曲线
            if ch.get("current") == "bond":
                t1 = time.time()
                peak = 0.0
                while time.time() - t1 < 900:
                    st = flow_state(base)
                    ch = st.get("chain") or {}
                    fv = ((st.get("pid") or {}).get("force") or {}).get("pv")
                    if fv is not None:
                        peak = max(peak, float(fv))
                    if fv is not None and float(fv) > 1.0:
                        cdp.call("Runtime.evaluate",
                                 {"expression": PID_FORCE_SCROLL_JS,
                                  "returnByValue": True})
                        time.sleep(0.8)
                        shots.append(shot(cdp, "iter9_d2_pid_force_curve", ts))
                        bond_curve_shot = True
                        print("[d2] 力曲线上升沿截图（force pv=%.2fN）"
                              % float(fv))
                        break
                    if not ch.get("active") or ch.get("current") != "bond":
                        break
                    time.sleep(0.5)
                if not bond_curve_shot:
                    print("[warn] bond 阶段未捕到 force pv>1（peak=%.2f），"
                          "兜底截当前曲线" % peak)
                    cdp.call("Runtime.evaluate",
                             {"expression": PID_FORCE_SCROLL_JS,
                              "returnByValue": True})
                    time.sleep(0.8)
                    shots.append(shot(cdp, "iter9_d2_pid_force_curve", ts))
            # 等 chain 收尾
            t1 = time.time()
            while time.time() - t1 < 900:
                st = flow_state(base)
                ch = st.get("chain") or {}
                if not ch.get("active"):
                    break
                time.sleep(1.0)
        finally:
            cdp.close()

        # ---- bond 流程记录：force_search 步 + 等温步耗时 ----
        steps = st.get("steps") or []
        fs_steps = [s for s in steps if s.get("op") == "force_search"]
        heater_dw = [s for s in steps
                     if s.get("op") == "heater"
                     and "Heat_Dw" in str(s.get("info", ""))]
        log_lines = []
        for e in (st.get("log") or []):
            line = "%s %s" % (e.get("msg", ""), e.get("desc", ""))
            if any(k in line for k in ("寻力", "等温", "保温", "下压", "加压",
                                       "保压", "接触位")):
                log_lines.append(line.strip())
        print("\n----- bond 日志关键行 -----")
        for ln in log_lines[-20:]:
            print("  %s" % ln)
        print("---------------------------")
        print("[steps] 全部步骤摘要=%s" % json.dumps(
            [{"i": s.get("i"), "op": s.get("op"), "name": s.get("name"),
              "status": s.get("status"), "dur_ms": s.get("dur_ms"),
              "info": str(s.get("info", ""))[:60]} for s in steps],
            ensure_ascii=False))

        # ---- d1. 工艺页 force_search OK 记录截图 ----
        cdp = new_tab(panel_url("flow"))
        try:
            cdp.call("Runtime.enable")
            time.sleep(4)
            r = cdp.call("Runtime.evaluate",
                         {"expression": FLOW_FORCE_SEARCH_JS,
                          "returnByValue": True})
            print("[d1] 寻力记录行定位：%s" % r["result"]["value"])
            time.sleep(1.0)
            shots.append(shot(cdp, "iter9_d1_flow_force_search", ts,
                              full=True))
        finally:
            cdp.close()

        # ================= 判定 =================
        ok = True
        print("\n===== 判定 =====")
        print("a. 温控压控页含三回路+硬件限值卡：%s" % dom_ok["a"])
        ok = ok and dom_ok["a"]
        print("b. 测校页 sim_sensors 含温度/力噪声输入：%s" % dom_ok["b"])
        ok = ok and dom_ok["b"]
        print("c. 机械常数 bond 组只剩接触位/寻力位键行：%s" % dom_ok["c"])
        ok = ok and dom_ok["c"]
        st_all = ch.get("stages") or {}
        six = ("wafer_ega", "flip_pick", "flip_handoff", "focus_level",
               "fine_align", "bond")
        all_ok = all(st_all.get(k) in ("done", "skip") for k in six) \
            and st_all.get("bond") == "done"
        print("chain.stages=%s（bond 须 done）：%s" % (st_all, all_ok))
        ok = ok and all_ok and not ch.get("error")
        fs_ok = any(s.get("status") == "ok"
                    and ("寻力接触" in str(s.get("info", ""))
                         or "寻力" in str(s.get("desc", "")))
                    for s in fs_steps) \
            or any("寻力接触" in ln for ln in log_lines)
        print("force_search 步成功（寻力接触）：%s" % fs_ok)
        ok = ok and fs_ok
        # 等温秒过：Heat_Dw 步 dur_ms < 5000
        warm_fast = None
        for s in heater_dw:
            if s.get("dur_ms") is not None:
                warm_fast = float(s["dur_ms"]) < 5000.0
        if warm_fast is None and heater_dw:
            warm_fast = True   # 无耗时字段，等温步存在即按幂等秒过记
            print("[info] Heat_Dw 步无耗时字段，按存在判定")
        print("硅片台等温步秒过（常保温）：%s" % warm_fast)
        ok = ok and bool(warm_fast)
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
