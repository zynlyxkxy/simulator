#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_link.py — 端到端验证：后端指令 → WS 帧 → 细化前端 19 轴跟随 + 出图。

用法：
    python light_test/tools/verify_link.py [--shots]

流程（一个浏览器会话跑完三个姿态）：
  1. 通过 HTTP 把 9030 后端摆到姿态（零位 / 交接 / 键合）；
  2. 等 WS 帧把值送到页面，读回每根轴的 current；
  3. 按 联机注入_light.js 的投影规则算期望值，逐轴比对（容差 0.05 模型单位）；
  4. --shots 时每个姿态存一张 PNG 到 light_test/verify_shots/。
"""
import argparse
import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from light_cdp import Session, DEFAULT_PAGE  # noqa: E402

BASE = "http://127.0.0.1:9030"
LT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAD2DEG = 180 / 3.141592653589793

# 与 联机注入_light.js 的 MAP 一致：(sim 轴, 模型轴, rot, sign, scale)
MAP = [
    ("WS_LS_X", "axis_WS_X", 0, +1, 1),
    ("WS_LS_Y", "axis_WS_Y", 0, +1, 1),
    ("WS_Rz", "axis_WS_RZ", 1, +1, 1),
    ("E_EPIN_Z", "axis_WS_EPIN_Z", 0, +1, 1, -2.5),   # 页面补丁：压紧零位偏 -2.5
    ("BS_Arm_Z", "axis_BS_Z", 0, +1, 1),
    ("BS_CAL_Rx", "axis_BS_Rx", 1, +1, 1),
    ("BS_CAL_Ry", "axis_BS_Ry", 1, +1, 1),
    ("AS_X", "axis_AS_X", 0, +1, 1),
    ("AS_Y", "axis_AS_Y", 0, +1, 1),
    ("AS_Z", "axis_AS_Z", 0, +1, 1),
    ("AS_Rx", "axis_AS_Rx", 1, +1, 1),
    ("AS_Ry", "axis_AS_Ry", 1, +1, 1),
    ("FA_X", "axis_ISS_FA_X", 0, +1, 1),
    ("FA_Z", "axis_ISS_FA_Z", 0, +1, 1),
    ("FA_Rx", "axis_ISS_FA_RX", 1, -1, 1),
    ("CH_Rz", "axis_ISS_FA_RZ", 1, +1, 1),
    ("ES_X", "axis_filmstage_x_0001", 0, +1, 1),
    ("ES_Y", "axis_filmstage_y_0001", 0, +1, 1),
    ("ES_PunchZ", "axis_filmpin_z_0001", 0, +1, 1),
]

POSES = {
    "零位": {"WS_LS_X": 0, "WS_LS_Y": 0, "WS_Rz": 0, "BS_Arm_Z": 0, "FA_X": 0,
             "FA_Z": 0, "FA_Rx": -3.1415927, "AS_X": 0, "AS_Y": 0, "AS_Z": 0,
             "CH_Rz": 0, "E_EPIN_Z": 0, "ES_X": 0, "ES_Y": 0},
    "交接": {"FA_X": -1242.35, "FA_Rx": 0, "FA_Z": 0, "BS_Arm_Z": -54.05,
             "AS_X": 0, "AS_Y": 115, "WS_LS_X": 0, "WS_LS_Y": 0},
    "键合": {"FA_X": 0, "FA_Rx": -3.1415927, "FA_Z": 0, "BS_Arm_Z": -148.0,
             "WS_LS_X": -40.02, "WS_LS_Y": -3.78, "WS_Rz": 0, "AS_Z": 0},
}

READ_JS = r"""
(function(){
  var H = window.__TD_VIZ__ || {};
  var P = window.__TWIN_DOM_PROJECT__;
  var AX = (H.axes && H.axes.length) ? H.axes : ((P && P.axes) || []);
  if (!AX.length) return JSON.stringify({err:'no project axes'});
  var inputs = document.querySelectorAll('input[type=range]');
  var cur = {};
  for (var i = 0; i < AX.length; i++) {
    if (inputs[i]) {
      var v = Number(inputs[i].value);
      if (isFinite(v)) { cur[AX[i].id] = v; continue; }
    }
    cur[AX[i].id] = Number(AX[i].current);
  }
  var hud = null, divs = document.querySelectorAll('div');
  for (var k = 0; k < divs.length; k++) {
    var t = divs[k].textContent || '';
    if (t.indexOf('联机状态') >= 0 && t.length < 600) { hud = divs[k]; break; }
  }
  return JSON.stringify({cur: cur, hud: hud ? hud.textContent : null});
})()
"""


def post(path, obj):
    req = urllib.request.Request(BASE + path, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def expected(sim_values):
    out = {}
    for row in MAP:
        sim, light, rot, sign, scale = row[:5]
        offset = row[5] if len(row) > 5 else 0.0
        if sim not in sim_values:
            continue
        v = sim_values[sim]
        if rot:
            v = v * RAD2DEG
        out[light] = v * scale * sign + offset
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", action="store_true")
    ap.add_argument("--page", default=DEFAULT_PAGE + "?ws=9030")
    ap.add_argument("--wait", type=float, default=22.0)
    a = ap.parse_args()

    shotdir = os.path.join(LT, "verify_shots")
    if a.shots:
        os.makedirs(shotdir, exist_ok=True)

    total_bad = 0
    with Session(a.page, wait=a.wait, gpu=True) as s:
        for name, pose in POSES.items():
            for ax in ("FA_X", "BS_Arm_Z", "FA_Rx", "FA_Z", "WS_LS_X", "WS_LS_Y",
                       "WS_Rz", "AS_X", "AS_Y", "AS_Z", "CH_Rz"):
                if ax in pose:
                    post("/api/axis/move", {"axis": ax, "target": pose[ax]})
            # E_PIN 是 21 轴里的三根（模型只有一根，注入侧取均值），逐个下发
            for ep in ("E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"):
                post("/api/axis/move", {"axis": ep,
                                        "target": pose.get("E_EPIN_Z", 0)})
            time.sleep(3.0)
            info = json.loads(s.eval_js(READ_JS))
            if info.get("err"):
                print("[FAIL] %s" % info["err"])
                return 1
            exp = expected(pose)
            print("\n===== 姿态 %s =====" % name)
            bad = []
            for light, want in exp.items():
                got = info["cur"].get(light)
                if got is None:
                    bad.append((light, want, None))
                    continue
                if abs(got - want) > 0.05:
                    bad.append((light, want, got))
            print("  轴跟随：%d/%d 一致" % (len(exp) - len(bad), len(exp)))
            for light, want, got in bad:
                print("    ✗ %-24s 期望 %8.3f  实际 %s" %
                      (light, want, "缺失" if got is None else "%8.3f" % got))
            hud = (info.get("hud") or "").split("\n")
            print("  HUD：%s" % " | ".join(x.strip() for x in hud[:4]))
            if a.shots:
                p = os.path.join(shotdir, "pose_%s.png" % name)
                s.screenshot(p)
                print("  png：%s" % p)
            total_bad += len(bad)

    print("\n==== %s（不一致 %d 项）====" % ("PASS" if total_bad == 0 else "FAIL", total_bad))
    return 0 if total_bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
