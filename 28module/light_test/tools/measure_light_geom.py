#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""measure_light_geom.py — 量细化前端关键零件的世界包围盒，与机器常数对表。

用法：
    python light_test/tools/measure_light_geom.py                 # 零位姿态
    python light_test/tools/measure_light_geom.py --pose handoff   # 交接对接姿态
    python light_test/tools/measure_light_geom.py --pose bond      # 键合接触姿态

姿态定义（光/仿真口径，见输出）：
  zero     : 全轴 0
  handoff  : FA_X→交接位、FA_RX→交接姿态、BS_Z→-50.93、AS 让开
  bond     : FA_X→0、BS_Z→-150（模型下限）、WS 把晶圆送到键合头下方

输出：每件的最小/最大世界坐标 + 顶/底面 Z，便于和 machine_constants 对表。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from light_cdp import Session, DEFAULT_PAGE  # noqa: E402

# 关注零件：key 用于报告，pat 是名称包含匹配（在 hook.project.objects 里找）
TARGETS = [
    ("晶圆吸盘", "ISX0200028715_A_2-吸盘"),
    ("键合头热压头", "ISG0408010019_C_1-_试_热压头组件"),
    ("翻转吸盘20x20", "ISX0100027209_A_2-吸盘-20x20"),
    ("翻转吸盘外壳", "ISG0412010005_A_1-_试_真空吸盘"),
    ("上相机镜头", "上相机镜头"),
    ("下相机镜头", "下i相机镜头"),
    ("上共焦探头", "上光谱共焦传感器"),
    ("下共焦探头", "下光谱共焦传感器"),
    ("蓝膜", "蓝膜"),
    ("蓝膜顶针帽", "蓝膜顶针顶帽"),
    ("粗对准相机镜头", "粗对准相机-镜头面"),
    ("对准N25滑台", "N25 X滑台"),
]

POSES = {
    "zero": {},
    # 交接：翻转台送到键合头下方（X=-1242.35 与 pick 199.85-1242.35≈-42.5 对齐），
    # 交接姿态 FA_RX=0，键合头按老常数 -50.93 先落一档
    "handoff": {"axis_ISS_FA_X": -1242.35, "axis_ISS_FA_RX": 0, "axis_BS_Z": -54.05,
                "axis_AS_Y": 115},
    # 键合：晶圆吸盘中心送到键合头下方（零位吸盘心 -2.48,10.53 → 头 -42.5,6.75）
    "bond": {"axis_ISS_FA_X": 0, "axis_ISS_FA_RX": 180, "axis_BS_Z": -148.0,
             "axis_WS_X": -40.02, "axis_WS_Y": -3.78},
}

JS_TMPL = r"""
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
  function apply(id, v){
    var ax = null;
    for (var i = 0; i < AXES.length; i++) if (AXES[i].id === id) ax = AXES[i];
    if (!ax) return 'missing';
    return applyAxis(ax, v);
  }
  function bbox(obj){
    var acc = new h.Box3();
    obj.traverse(function(x){
      if (!x.geometry || !x.geometry.attributes || !x.geometry.attributes.position) return;
      if (!x.geometry.boundingBox) x.geometry.computeBoundingBox();
      var bb = x.geometry.boundingBox;
      if (!bb || !isFinite(bb.min.x)) return;
      var t = bb.clone().applyMatrix4(x.matrixWorld);
      if (!isFinite(t.min.x)) return;
      acc.union(t);
    });
    if (acc.isEmpty()) return null;
    return {min: [acc.min.x, acc.min.y, acc.min.z],
            max: [acc.max.x, acc.max.y, acc.max.z]};
  }
  var pose = __POSE__;
  var applied = {};
  for (var k in pose) { applied[k] = apply(k, pose[k]); await sleep(650); }
  // 复位其余轴
  for (var i2 = 0; i2 < AXES.length; i2++) {
    var ax2 = AXES[i2];
    if (pose[ax2.id] === undefined && Math.abs(Number(ax2.current)) > 1e-9) apply(ax2.id, 0);
  }
  await sleep(650);

  var objs = (PROJ && PROJ.objects) || (H.project && H.project.objects) || [], found = {};
  var out = {pose: pose, applied: applied, items: [], currents: {}};
  AXES.forEach(function(a){ out.currents[a.id] = Number(a.current); });
  __TARGETS__.forEach(function(t){
    var pat = t[1], best = null;
    for (var i = 0; i < objs.length; i++) {
      if ((objs[i].name || '').indexOf(pat) >= 0) { best = objs[i]; break; }
    }
    if (!best) { out.items.push({label: t[0], pat: pat, err: '未找到'}); return; }
    var o = (H.obj3d && H.obj3d.get) ? H.obj3d.get(best.id) : null;
    if (!o) { out.items.push({label: t[0], pat: pat, err: '对象未实例化'}); return; }
    var bb = bbox(o);
    out.items.push({label: t[0], id: best.id, name: best.name, box: bb});
  });
  return JSON.stringify(out);
})()
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="zero", choices=list(POSES))
    ap.add_argument("--out")
    ap.add_argument("--page", default=DEFAULT_PAGE)
    ap.add_argument("--wait", type=float, default=25.0)
    ap.add_argument("--no-gpu", action="store_true")
    a = ap.parse_args()

    js = JS_TMPL.replace("__POSE__", json.dumps(POSES[a.pose])) \
                .replace("__TARGETS__", json.dumps(TARGETS, ensure_ascii=False))
    with Session(a.page, wait=a.wait, gpu=not a.no_gpu) as s:
        raw = s.eval_js(js, await_promise=True, timeout=240)
    data = json.loads(raw)
    if data.get("err"):
        print("[FAIL] %s" % data["err"])
        return 1

    print("姿态 %s：施加 %s" % (a.pose, json.dumps(data["applied"], ensure_ascii=False)))
    print("实际轴值：%s" % json.dumps({k: round(v, 3) for k, v in data["currents"].items()
                                    if abs(v) > 1e-6}, ensure_ascii=False))
    print("\n%-14s %-34s %-9s %-9s  %s" % ("零件", "世界 X / Y", "底面Z", "顶面Z", "尺寸"))
    rows = []
    for it in data["items"]:
        if it.get("err"):
            print("%-14s %s" % (it["label"], it["err"]))
            rows.append(it)
            continue
        b = it["box"]
        cx = round((b["min"][0] + b["max"][0]) / 2, 2)
        cy = round((b["min"][1] + b["max"][1]) / 2, 2)
        zmin, zmax = round(b["min"][2], 2), round(b["max"][2], 2)
        size = [round(b["max"][i] - b["min"][i], 2) for i in range(3)]
        print("%-14s (%8.2f,%9.2f) %9.2f %9.2f  %s" %
              (it["label"], cx, cy, zmin, zmax, size))
        it["cx"], it["cy"], it["zmin"], it["zmax"], it["size"] = cx, cy, zmin, zmax, size
        rows.append(it)

    out = a.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "几何实测_%s.json" % a.pose)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print("\n[out] %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
