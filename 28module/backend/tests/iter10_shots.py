#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iter10_shots.py — Edge headless + CDP 截图验证迭代 10（2026-09-01）：
  1. 3D 联机版：键合完成后晶圆表面可见已键合芯片（die 不再埋在吸盘体内）；
     die map 已键合 site 金 pad 隐藏（数值核对 + 截图）。
  2. 面板工艺页"选片选位图"卡：双 canvas 渲染、已键合 site 绿色；
     点选 die/site 写入 fpDie/chainSite/flSite/faSite 并高亮。
  3. 已键合 site 重跑 focus_level → 守卫报错"已键合，禁止重复调平/对准"。

截图（Temp/panel_shots/iter10_*_<ts>.png）：
  a. 3D 联机版键合后晶圆区域（相机对准已键合 die）；
  b. 选片选位图卡（已键合 site 绿色）；
  c. 点选另一 site + 一颗 die 后（输入框写入 + 黄框高亮 + dmMsg）；
  d. 已键合 site 重跑 focus_level 的守卫报错（toast 文案）。

数值证据：
  - GET /api/viz：carrier=wafer 的 die id/pos；GET /api/workpiece/map：
    bonded site 行列；
  - 页面内 scene 遍历：__viz_die_<id> 世界坐标 vs __viz_wafer 包围盒顶面
    （die z 须在晶圆顶面之上，不再埋在吸盘体内）；
  - POST /api/process/focus_level {site:[r,c]} 直接探测 400 文案。

用法：python backend/tests/iter10_shots.py
（会清理 9010/9101/9102/9190/1883/9200 占用后自起 runtime）
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
    sys.stdout.reconfigure(errors="replace")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from tests.edge_headless_check import CDP  # noqa: E402
from tests.panel_fl_verify_shots import (  # noqa: E402
    EDGE, PANEL, http_json, http_post, http_put, wait_ready)

WS_PORT = 9010
CDP_PORT = 9247
SHOT_DIR = os.path.abspath(os.path.join(ROOT, "..", "Temp", "panel_shots"))
ONLINE_HTML = os.path.abspath(os.path.join(
    ROOT, "..", "html_model", "d2w_online.html"))
PORTS = (9010, 9101, 9102, 9190, 1883, 9200)

# ---------------- JS 片段 ----------------
DM_SCROLL_JS = r"""
(function () {
  document.getElementById('tabFlow').click();
  var cv = document.getElementById('dmFilm');
  if (!cv) return 'dmFilm-not-found';
  cv.scrollIntoView({block: 'center'});
  return 'ok';
})()
"""

DM_STATE_JS = r"""
(function () {
  if (!window.dmData || !dmData.ok) return JSON.stringify({ok: false});
  var bonded = (dmData.wafer_sites || []).filter(function (s) { return s.bonded; });
  var filmN = (dmData.film_dies || []).filter(function (d) {
    return d.carrier === 'film'; }).length;
  return JSON.stringify({ok: true, dies: (dmData.film_dies || []).length,
                         sites: (dmData.wafer_sites || []).length,
                         filmAlive: filmN,
                         bonded: bonded.map(function (s) {
                           return [s.row, s.col]; })});
})()
"""

# 点选：一颗在膜 die + 一个未键合 site（避开已键合位），写输入框
DM_CLICK_JS = r"""
(function () {
  if (!window.dmData || !dmData.ok) return 'no-dmData';
  function clickCv(cv, x, y, dia) {
    var v = dmView(cv, dia);
    var p = dmXY(v, x, y);
    var rect = cv.getBoundingClientRect();
    cv.dispatchEvent(new MouseEvent('click', {
      clientX: rect.left + p[0], clientY: rect.top + p[1], bubbles: true }));
  }
  var geo = dmData.geometry || {film: {}, wafer: {}};
  var die = null;
  (dmData.film_dies || []).forEach(function (d) {
    if (!die && d.carrier === 'film') die = d; });
  if (!die) return 'no-film-die';
  var site = null;
  (dmData.wafer_sites || []).forEach(function (s) {
    if (!site && !s.bonded) site = s; });
  if (!site) return 'no-free-site';
  clickCv(document.getElementById('dmFilm'), die.x, die.y,
          Number(geo.film.diameter_mm) || 200);
  clickCv(document.getElementById('dmWafer'), site.x, site.y,
          Number(geo.wafer.diameter_mm) || 300);
  return JSON.stringify({
    die: die.id, site: [site.row, site.col],
    fpDie: document.getElementById('fpDie').value,
    chainSite: document.getElementById('chainSite').value,
    flSite: document.getElementById('flSite').value,
    faSite: document.getElementById('faSite').value,
    dmMsg: (document.getElementById('dmMsg') || {}).textContent});
})()
"""

# 已键合 site 重跑 focus_level（面板路径：写 flSite + 点按钮 → toast）
FL_GUARD_JS = r"""
(function (site) {
  document.getElementById('tabFlow').click();
  document.getElementById('flSite').value = site;
  document.getElementById('btnFlRun').click();
  var fl = document.getElementById('flSite');
  if (fl && fl.scrollIntoView) fl.scrollIntoView({block: 'center'});
  return 'clicked:' + site;
})(__SITE__)
"""

TOAST_JS = r"""
(function () {
  var t = document.getElementById('toast');
  return JSON.stringify({display: t.style.display, text: t.textContent,
                         flMsg: (document.getElementById('flMsg') || {}).textContent});
})()
"""

# 3D：数值核对（die 世界坐标 vs 晶圆包围盒顶面）+ 相机探测
VIZ_DUMP_JS = r"""
(function () {
  var V = window.__TD_VIZ__;
  if (!V) return JSON.stringify({hook: false});
  var out = {hook: true, dies: [], cams: [], waferBox: null, waferVisible: null};
  V.scene.traverse(function (o) {
    if (o.isCamera) out.cams.push(o.type);
    if (o.name === '__viz_wafer') {
      out.waferVisible = o.visible;
      var b = new V.Box3().setFromObject(o);
      out.waferBox = {min: [b.min.x, b.min.y, b.min.z],
                      max: [b.max.x, b.max.y, b.max.z]};
    }
    if (o.name && o.name.indexOf('__viz_die_') === 0 &&
        o.name !== '__viz_die_map' && o.name !== '__viz_die_decal') {
      var p = new V.V3();
      o.getWorldPosition(p);
      out.dies.push({name: o.name, visible: o.visible,
                     parent: o.parent && o.parent.name,
                     world: [p.x, p.y, p.z]});
    }
  });
  var hud = null;
  document.querySelectorAll('div').forEach(function (d) {
    if (!hud && d.style && d.style.zIndex === '9999') hud = d; });
  out.dataViz = hud ? hud.getAttribute('data-viz') : null;
  return JSON.stringify(out);
})()
"""

# 3D：相机对准已键合 die（若场景里找得到相机）
VIZ_CAM_JS = r"""
(function (tx, ty, tz) {
  var V = window.__TD_VIZ__;
  if (!V) return 'no-hook';
  var cam = null;
  V.scene.traverse(function (o) { if (!cam && o.isCamera) cam = o; });
  if (!cam) return 'no-camera';
  cam.position.set(tx + 18, ty - 26, tz + 30);
  cam.lookAt(tx, ty, tz);
  if (cam.updateProjectionMatrix) cam.updateProjectionMatrix();
  return 'ok';
})(__TX__, __TY__, __TZ__)
"""

HIDE_TOP_JS = r"""
(function (names) {
  var V = window.__TD_VIZ__;
  var proj = window.__TWIN_DOM_PROJECT__;
  if (!V || !proj) return 'no-hook';
  var hidden = [];
  (proj.objects || []).forEach(function (def) {
    for (var i = 0; i < names.length; i++) {
      if (def.name && def.name.indexOf(names[i]) >= 0) {
        var o = V.obj3d.get(def.id);
        if (o) { o.visible = false; hidden.push(def.name); }
      }
    }
  });
  return JSON.stringify(hidden);
})(__NAMES__)
"""


def shot(cdp, name, ts, full=False):
    params = {"format": "png"}
    if full:
        params["captureBeyondViewport"] = True
    data = cdp.call("Page.captureScreenshot", params)["data"]
    p = os.path.join(SHOT_DIR, "%s_%s.png" % (name, ts))
    with open(p, "wb") as f:
        f.write(base64.b64decode(data))
    print("[ok] 截图：%s" % p)
    return p


def new_tab(url):
    req = urllib.request.Request(
        "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
        method="PUT")
    target = json.loads(
        urllib.request.urlopen(req, timeout=5).read().decode())
    return CDP(target["webSocketDebuggerUrl"])


def ev(cdp, expr):
    r = cdp.call("Runtime.evaluate",
                 {"expression": expr, "returnByValue": True})
    return r.get("result", {}).get("value")


def flow_state(base):
    return http_json(base + "/api/flow/state")


def panel_url(tab):
    return "file:///" + PANEL.replace(os.sep, "/") + "?tab=" + tab


def kill_ports():
    out = subprocess.check_output(["netstat", "-ano"], text=True,
                                  errors="replace")
    killed = []
    for line in out.splitlines():
        if "LISTENING" not in line:
            continue
        for p in PORTS:
            if (":%d " % p) in line:
                pid = line.strip().split()[-1]
                if pid not in killed and pid != "0":
                    print("[port] %d 被 PID %s 占用，结束" % (p, pid))
                    subprocess.call(["taskkill", "//F", "//PID", pid],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
                    killed.append(pid)
    if killed:
        time.sleep(1.5)


def main():
    if not os.path.exists(EDGE):
        print("[FAIL] Edge 不存在：%s" % EDGE)
        return 1
    os.makedirs(SHOT_DIR, exist_ok=True)
    kill_ports()

    proc = subprocess.Popen(
        [sys.executable, os.path.join(ROOT, "runtime", "main.py")],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    edge = None
    shots = []
    profile = tempfile.mkdtemp(prefix="edge_iter10_")
    try:
        if not wait_ready():
            print("[FAIL] runtime 启动失败")
            return 1
        print("[ok] runtime 就绪")
        base = "http://127.0.0.1:%d" % WS_PORT

        # 确定性注入（同 iter9）
        http_put(base + "/api/recipe", {
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8},
            "wafer_load": {
                "load_x_mm": 0.2, "load_y_mm": -0.15,
                "load_rz_rad": 0.0002, "jitter_on": False,
                "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0}})

        # 等硅片台常保温到温（启动即设 Heat_Dw=150）
        print("[preheat] 等硅片台到温（150±2℃）...")
        t0 = time.time()
        while time.time() - t0 < 300:
            pv = ((flow_state(base).get("pid") or {})
                  .get("wafer_temp") or {}).get("pv")
            if pv is not None and abs(float(pv) - 150.0) <= 2.0:
                print("[preheat] 到温 pv=%.1f℃（%ds）"
                      % (float(pv), time.time() - t0))
                break
            time.sleep(1.0)

        # 清场 → 一键串联（默认中心位）
        c0, r0 = http_post(base + "/api/workpiece/reset", {})
        print("[chain] workpiece reset -> %s %s" % (c0, r0.get("ok")))
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        c, r = http_post(base + "/api/process/chain", {})
        print("[chain] POST -> %s %s %s" % (c, r.get("ok"), r.get("error")))
        ch, st = {}, {}
        t0 = time.time()
        while time.time() - t0 < 2400:
            st = flow_state(base)
            ch = st.get("chain") or {}
            if not ch.get("active"):
                break
            time.sleep(2.0)
        print("[chain] 结束：stages=%s error=%s（%ds）"
              % (ch.get("stages"), ch.get("error"), time.time() - t0))
        stages = ch.get("stages") or {}
        bond_done = stages.get("bond") == "done"
        if not bond_done:
            tail = [str(e.get("msg", "")) for e in (st.get("log") or [])][-15:]
            print("[chain] bond 未完成，末尾日志：%s" % tail)

        # ---- 数值证据：map + viz ----
        mp = http_json(base + "/api/workpiece/map")
        bonded = [s for s in (mp.get("wafer_sites") or []) if s.get("bonded")]
        print("[map] bonded sites=%s"
              % [[s["row"], s["col"], s["x"], s["y"]] for s in bonded])
        viz = http_json(base + "/api/viz")
        wafer_dies = [d for d in (viz.get("dies") or [])
                      if d.get("carrier") == "wafer"]
        print("[viz] wafer dies=%s"
              % [{k: d.get(k) for k in ("id", "carrier", "pos", "rz")}
                 for d in wafer_dies])
        print("[viz] wafer=%s" % viz.get("wafer"))

        # ---- Edge headless ----
        edge = subprocess.Popen(
            [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
             "--no-default-browser-check", "--window-size=1600,1000",
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

        # ================= a. 3D 联机版 =================
        if ONLINE_HTML and os.path.exists(ONLINE_HTML):
            cdp = new_tab("file:///" + ONLINE_HTML.replace(os.sep, "/"))
            try:
                cdp.call("Runtime.enable")
                print("[a] 联机版加载，等 12s（初始化+WS+viz 首帧）...")
                time.sleep(12)
                dump = json.loads(ev(cdp, VIZ_DUMP_JS) or "{}")
                print("[a] 场景相机=%s waferVisible=%s waferBox=%s"
                      % (dump.get("cams"), dump.get("waferVisible"),
                         dump.get("waferBox")))
                for dd in dump.get("dies", []):
                    print("[a] die %s parent=%s visible=%s world=%s"
                          % (dd["name"], dd["parent"], dd["visible"],
                             ["%.2f" % v for v in dd["world"]]))
                print("[a] data-viz=%s" % dump.get("dataViz"))
                wdie = None
                for dd in dump.get("dies", []):
                    if dd["parent"] == "晶圆吸盘":
                        wdie = dd
                        break
                if wdie and dump.get("waferBox"):
                    dz = wdie["world"][2]
                    wtop = dump["waferBox"]["max"][2]
                    print("[a] die z=%.3f vs 晶圆顶面 z=%.3f（差 %.3f，"
                          "应≈die半高+浮起>0）" % (dz, wtop, dz - wtop))
                if wdie:
                    tx, ty, tz = wdie["world"]
                    r = ev(cdp, VIZ_CAM_JS
                           .replace("__TX__", "%.4f" % tx)
                           .replace("__TY__", "%.4f" % ty)
                           .replace("__TZ__", "%.4f" % tz))
                    print("[a] 相机对准：%s" % r)
                    if r == "no-camera":
                        print("[a] 场景内无相机对象，退化为遮挡隐藏+默认视角")
                        print("[a] 隐藏上方遮挡：%s"
                              % ev(cdp, HIDE_TOP_JS.replace(
                                  "__NAMES__", '["键合头", "上相机"]')))
                    time.sleep(2.0)
                    shots.append(shot(cdp, "iter10_a_3d_bonded_die", ts))
                    # 再拉近一档
                    if wdie:
                        ev(cdp, VIZ_CAM_JS
                           .replace("__TX__", "%.4f" % tx)
                           .replace("__TY__", "%.4f" % ty)
                           .replace("__TZ__", "%.4f" % tz)
                           .replace("18, ty - 26, tz + 30",
                                    "8, ty - 12, tz + 13"))
                        time.sleep(1.5)
                        shots.append(shot(cdp, "iter10_a2_3d_bonded_die_close",
                                          ts))
            finally:
                cdp.close()

        # ================= b. 选片选位图卡 =================
        cdp = new_tab(panel_url("flow"))
        try:
            cdp.call("Runtime.enable")
            print("[b] 工艺页加载，等 6s（die map 1Hz 首轮）...")
            time.sleep(6)
            print("[b] dmFilm 定位：%s" % ev(cdp, DM_SCROLL_JS))
            time.sleep(2.0)
            dm = json.loads(ev(cdp, DM_STATE_JS) or "{}")
            print("[b] die map 状态=%s" % dm)
            shots.append(shot(cdp, "iter10_b_diemap_card", ts))

            # ================= c. 点选 die + site =================
            r = ev(cdp, DM_CLICK_JS)
            print("[c] 点选结果=%s" % r)
            time.sleep(0.8)
            shots.append(shot(cdp, "iter10_c_diemap_picked", ts))

            # ================= d. 已键合 site 重跑 focus_level =================
            if bonded:
                bs = bonded[0]
                site_s = "%s,%s" % (bs["row"], bs["col"])
                # 直接 API 探测拿权威文案
                cg, rg = http_post(base + "/api/process/focus_level",
                                   {"mode": "per_site",
                                    "site": [bs["row"], bs["col"]]})
                print("[d] focus_level 重跑 -> HTTP %s error=%s"
                      % (cg, rg.get("error")))
                # 面板路径：写 flSite + 点按钮 → toast
                print("[d] 面板触发：%s" % ev(
                    cdp, FL_GUARD_JS.replace("__SITE__",
                                             json.dumps(site_s))))
                time.sleep(1.2)
                print("[d] toast=%s" % ev(cdp, TOAST_JS))
                shots.append(shot(cdp, "iter10_d_focus_level_guard", ts))
        finally:
            cdp.close()

        print("\n==== DONE ====")
        print("截图：%s" % shots)
        return 0 if bond_done else 2
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
