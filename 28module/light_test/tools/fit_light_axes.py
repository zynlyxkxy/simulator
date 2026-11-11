#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fit_light_axes.py — 逐轴驱动细化前端，量被驱动对象的世界位移，得出方向/行程事实。

用法：
    python light_test/tools/fit_light_axes.py [--out light_test/轴标定_light.json]

对 19 根轴依次：置 lo → mid → hi，每次用 Box3.setFromObject 取包围盒中心世界坐标。
产出：每轴 位移向量 / 主轴分量 / 每单位世界位移（mm per mm 或 mm per deg）。
这些是机械事实（模型自身），不涉及仿真语义；语义映射在联机注入_light.js 的 MAP 表。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from light_cdp import Session, DEFAULT_PAGE  # noqa: E402

JS = r"""
(async function(){

  var H = window.__TD_VIZ__ || {};
  var PROJ = window.__TWIN_DOM_PROJECT__;
  var AXES = (H.axes && H.axes.length) ? H.axes : ((PROJ && PROJ.axes) || []);
  var INPUTS = document.querySelectorAll('input[type=range]');
  function ctlOf(id){
    if (H.controls && H.controls.get) { var c = H.controls.get(id); if (c) return c; }
    for (var i = 0; i < AXES.length; i++) {
      if (AXES[i].id === id) return {input: INPUTS[i] || null, origOninput: INPUTS[i] ? INPUTS[i].oninput : null};
    }
    return null;
  }
  function applyAxis(ax, v){
    var c = ctlOf(ax.id);
    if (c && c.input) {
      c.input.value = v;
      var f = c.origOninput || c.input.oninput;
      if (typeof f === 'function') { f.call(c.input); return 'slider'; }
    }
    if (typeof H.setAxis === 'function') { H.setAxis(ax, v, false); return 'hook'; }
    return 'none';
  }
  function sleep(ms){ return new Promise(function(r){ setTimeout(r, ms); }); }
  window.__LIGHT_CALIB__ = true;
  var h = {Box3: H.Box3, V3: H.V3, axes: AXES};
  function apply(ax, v){ return applyAxis(ax, v) !== 'none'; }
  // 逐网格累加：setFromObject 会被个别 NaN 包围盒污染成空盒，这里跳过非法值
  function center(obj){
    var acc = new h.Box3();
    obj.traverse(function(x){
      if (!x.geometry || !x.geometry.attributes || !x.geometry.attributes.position) return;
      if (!x.geometry.boundingBox) x.geometry.computeBoundingBox();
      var bb = x.geometry.boundingBox;
      if (!bb || !isFinite(bb.min.x) || !isFinite(bb.max.x)) return;
      var t = bb.clone().applyMatrix4(x.matrixWorld);
      if (!isFinite(t.min.x) || !isFinite(t.max.x)) return;
      acc.union(t);
    });
    if (acc.isEmpty()) return [null, null, null];
    var c = acc.getCenter(new h.V3());
    return [c.x, c.y, c.z];
  }
  var out = {glbLoaded: !!window.__TWINDOM_E2E_GLB_LOADED__, axes: []};
  for (var i = 0; i < h.axes.length; i++) {
    var ax = h.axes[i];
    var obj = (H.obj3d && H.obj3d.get) ? H.obj3d.get(ax.movingObjectId) : null;
    if (!obj) { out.axes.push({id: ax.id, err: 'movingObject 缺失'}); continue; }
    var keep = Number(ax.current);
    var lo = Number(ax.min), hi = Number(ax.max), mid = (lo + hi) / 2;
    var rec = {id: ax.id, name: ax.name, type: ax.type, direction: ax.direction,
               min: lo, max: hi, keep: keep, move: ax.movingObjectId, slider: apply(ax, keep)};
    apply(ax, lo);  await sleep(700); rec.curLo = Number(ax.current); rec.pLo = center(obj);
    apply(ax, mid); await sleep(700); rec.curMid = Number(ax.current); rec.pMid = center(obj);
    apply(ax, hi);  await sleep(700); rec.curHi = Number(ax.current); rec.pHi = center(obj);
    apply(ax, keep); await sleep(700); rec.curBack = Number(ax.current);
    out.axes.push(rec);
  }
  return JSON.stringify(out);
})()
"""


def analyse(rec):
    if rec.get("err"):
        return rec
    a, b = rec["pLo"], rec["pHi"]
    if a[0] is None or b[0] is None:
        rec["err"] = "包围盒不可用（无几何或含 NaN）"
        return rec
    d = [b[i] - a[i] for i in range(3)]
    span = rec["max"] - rec["min"]
    dom = max(range(3), key=lambda i: abs(d[i]))
    per_unit = (d[dom] / span) if span else 0.0
    rec["delta"] = [round(x, 4) for x in d]
    rec["dom"] = "XYZ"[dom]
    rec["sign"] = (1 if d[dom] > 0 else -1) if abs(d[dom]) > 1e-9 else 0
    rec["per_unit_mm"] = round(per_unit, 6)
    rec["travel_mm"] = round(abs(d[dom]), 3)
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "轴标定_light.json"))
    ap.add_argument("--page", default=DEFAULT_PAGE)
    ap.add_argument("--wait", type=float, default=25.0)
    ap.add_argument("--no-gpu", action="store_true")
    a = ap.parse_args()

    with Session(a.page, wait=a.wait, gpu=not a.no_gpu) as s:
        raw = s.eval_js(JS, await_promise=True, timeout=240)
    data = json.loads(raw)
    if data.get("err"):
        print("[FAIL] %s" % data["err"])
        return 1
    recs = [analyse(r) for r in data["axes"]]
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump({"glbLoaded": data["glbLoaded"], "axes": recs}, f,
                  ensure_ascii=False, indent=1)

    print("GLB 全部加载: %s" % data["glbLoaded"])
    print("%-24s %-7s %-4s %8s %8s %9s  %s" %
          ("轴 id", "类型", "方向", "行程mm", "主轴", "mm/单位", "世界位移"))
    for r in recs:
        if r.get("err"):
            print("%-24s %s" % (r["id"], r["err"]))
            continue
        print("%-24s %-7s %-4s %8.2f %8s %9.4f  %s" %
              (r["id"], r["type"], r.get("direction", ""), r["travel_mm"],
               r["dom"], r["per_unit_mm"], r["delta"]))
    print("\n[out] %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
