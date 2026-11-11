# -*- coding: utf-8 -*-
"""analyze_head_vs_align.py — 键合头 × 对准台 穿模对分析（临时工具，可保留）。

2026-08-20 适配零件级 v2（gen_sandboxes.py 重写后）：
- 盒粒度 = 每几何体一盒（pt:<objectId>），不再有 mv:XX 聚合盒；
- 重叠口径 = overlap_depth（三向重叠量最小值），判撞阈值 depth_eps=2mm。

做的事：
1. 打出键合头链（BS_Arm_Z 升降臂子树）与对准台模块（AS 子树）每个对象的
   局部 transform、世界矩阵、世界 AABB，逐层对账变换链；
2. 独立重算零位世界 AABB 并与 runtime/sandboxes.json 逐盒对账；
3. 零位全盒扫描：列出所有重叠对及深度（含固定×固定，供人工审查）；
4. 用户场景复现：对准台 (0,0) + BS_Arm_Z 下降，逐 0.1mm 找首个深度>ε 对。

用法：python backend/tools/analyze_head_vs_align.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_JSON = os.path.normpath(os.path.join(
    HERE, "..", "..", "html_model", "d2w_front.json"))
SB_JSON = os.path.normpath(os.path.join(HERE, "..", "runtime", "sandboxes.json"))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "tools")))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "runtime")))

from gen_sandboxes import (DEPTH_EPS, mat_mul, mat_trs, mat_point, corners,
                           local_aabb, overlap_depth)


def load():
    with open(MODEL_JSON, encoding="utf-8") as f:
        model = json.load(f)
    objs = model["objects"]
    by_id = {o["id"]: o for o in objs}
    children = {}
    for o in objs:
        children.setdefault(o["parentId"], []).append(o["id"])
    return model, objs, by_id, children


def build_world_mats(by_id):
    _wm = {}

    def world_mat(oid):
        if oid in _wm:
            return _wm[oid]
        o = by_id[oid]
        t = o["transform"]
        m = mat_trs(t["position"], t["rotation"], t["scale"])
        pid = o["parentId"]
        m = mat_mul(world_mat(pid), m) if pid else m
        _wm[oid] = m
        return m
    return world_mat


def world_aabb(by_id, world_mat, oid):
    geo = by_id[oid].get("geometry")
    if not geo:
        return None
    mn, mx = local_aabb(geo)
    pts = [mat_point(world_mat(oid), c) for c in corners(mn, mx)]
    return (tuple(min(p[i] for p in pts) for i in range(3)),
            tuple(max(p[i] for p in pts) for i in range(3)))


def fmt(v):
    return "(%.2f, %.2f, %.2f)" % (v[0], v[1], v[2])


def dump_subtree(by_id, children, world_mat, root, label):
    print("\n==== 子树：%s（root=%s %s）====" % (label, root, by_id[root]["name"]))

    def rec(oid, depth):
        o = by_id[oid]
        t = o["transform"]
        geo = o.get("geometry")
        g = geo["type"] if geo else "-"
        wm = world_mat(oid)
        wpos = (wm[0][3], wm[1][3], wm[2][3])
        line = "%s%s | %s | geo=%s | pos=%s rot=%s scale=%s | wpos=%s" % (
            "  " * depth, oid, o["name"], g,
            fmt(t["position"]), fmt(t["rotation"]), fmt(t["scale"]), fmt(wpos))
        if geo:
            mn, mx = world_aabb(by_id, world_mat, oid)
            line += " | wAABB %s~%s" % (fmt(mn), fmt(mx))
        print(line)
        for cid in children.get(oid, ()):
            rec(cid, depth + 1)

    rec(root, 0)


def main():
    model, objs, by_id, children = load()
    world_mat = build_world_mats(by_id)
    axes = model["axes"]

    # ---- 1. 两条子树全量 dump ----
    dump_subtree(by_id, children, world_mat, "box_mssptvdy_4guv68", "键合头链(升降臂)")
    dump_subtree(by_id, children, world_mat, "group_msssqnbg_cvi3bv", "对准台模块")

    # ---- 轴归属 ----
    print("\n==== 轴表（movingObject → 轴）====")
    for a in axes:
        mo = a["movingObjectId"]
        print("  %s type=%s dir=%s sign=%s range=[%s,%s] -> %s(%s) pivot=%s ref=%s"
              % (a["id"], a["type"], a["direction"], a.get("directionSign", 1),
                 a["min"], a["max"], mo, by_id[mo]["name"],
                 a.get("pivot"), a.get("pivotReferenceId")))

    # ---- 2. 与 sandboxes.json 对账：零件级盒 = 单几何体零位世界 AABB ----
    with open(SB_JSON, encoding="utf-8") as f:
        sb = json.load(f)
    sb_boxes = {b["name"]: b for b in sb["boxes"]}

    print("\n==== 运动盒对账（重算零位世界 AABB vs sandboxes.json，零件级）====")
    n_same = n_diff = 0
    for b in sb["boxes"]:
        if b["kind"] != "moving":
            continue
        bb = world_aabb(by_id, world_mat, b["object"])
        if bb is None:
            print("  %s: 对象 %s 无几何（模型变了？）" % (b["name"], b["object"]))
            continue
        mn, mx = bb
        same = all(abs(mn[i] - b["min"][i]) < 0.01 and abs(mx[i] - b["max"][i]) < 0.01
                   for i in range(3))
        # 注意：旋转扫掠会使存档盒膨胀，这里只对比"未扫掠"的零位盒
        if same:
            n_same += 1
        else:
            n_diff += 1
            print("  %s(%s): 重算 %s~%s | 存档 %s~%s（旋转扫掠膨胀 or 不一致）"
                  % (b["name"], b["label"], fmt(mn), fmt(mx),
                     fmt(b["min"]), fmt(b["max"])))
    print("  一致 %d 盒，膨胀/不一致 %d 盒（见上）" % (n_same, n_diff))

    # ---- 3. 零位全盒扫描（含固定×固定，按深度排序） ----
    print("\n==== 零位全盒重叠扫描（sandboxes.json 盒，无任何豁免）====")
    bs = sb["boxes"]
    hits = []
    for i in range(len(bs)):
        for j in range(i + 1, len(bs)):
            d = overlap_depth(bs[i]["min"], bs[i]["max"],
                              bs[j]["min"], bs[j]["max"])
            if d > 1e-6:
                hits.append((d, bs[i]["name"], bs[i]["label"],
                             bs[j]["name"], bs[j]["label"]))
    hits.sort(key=lambda t: -t[0])
    print("  零位重叠对数: %d（深度>ε=%.1f 的 %d 对）"
          % (len(hits), DEPTH_EPS, sum(1 for h in hits if h[0] > DEPTH_EPS)))
    for d, na, la, nb, lb in hits:
        print("  %7.2fmm  %s(%s) × %s(%s)" % (d, na, la, nb, lb))

    # ---- 4. 用户场景复现：对准台(0,0)，BS_Arm_Z 逐 0.1mm 下降 ----
    print("\n==== 用户场景：AS 台零位，BS_Arm_Z 从 0 降到 -190，"
          "首个深度>ε=%.1f 对 ====" % DEPTH_EPS)
    exempt = set()
    for e in sb.get("exempt_pairs", []):
        exempt.add((e["a"], e["b"]))
        exempt.add((e["b"], e["a"]))

    def box_at(b, zoff):
        mn = list(b["min"]); mx = list(b["max"])
        for aid, (fx, fy, fz, _lo, _hi) in b["linear"].items():
            v = zoff if aid == "BS_Arm_Z" else 0.0
            mn[0] += fx * v; mx[0] += fx * v
            mn[1] += fy * v; mx[1] += fy * v
            mn[2] += fz * v; mx[2] += fz * v
        return mn, mx

    for z10 in range(0, 1901):
        z = -z10 / 10.0
        for i in range(len(bs)):
            bi = bs[i]
            if "BS_Arm_Z" not in bi["linear"]:
                continue
            mn1, mx1 = box_at(bi, z)
            for j in range(len(bs)):
                if i == j:
                    continue
                bj = bs[j]
                if (bi["name"], bj["name"]) in exempt:
                    continue
                d = overlap_depth(mn1, mx1, bj["min"], bj["max"])
                if d > DEPTH_EPS:
                    print("  z=%.1f: 深度 %.2fmm  %s(%s) × %s(%s)  [判撞]"
                          % (z, d, bi["name"], bi["label"],
                             bj["name"], bj["label"]))

    # 关键对单独列出（不论豁免）：上气环-左 × 对准相机承载台 / 吸盘基座
    print("\n  -- 上气环-左 下降时与关键固定件的 Z 向关系（豁免对也列出）--")
    air = sb_boxes.get("pt:airring_left_0001")
    for name in ("pt:box_mssty4oj_u3pj08", "pt:chuck_base_notched_0001"):
        bj = sb_boxes.get(name)
        if air is None or bj is None:
            print("    缺盒 %s（模型变了？重新生成）" % name)
            continue
        for z in (0.0, -100.0, -130.0, -160.0, -162.5, -180.0, -190.0):
            mn1, mx1 = box_at(air, z)
            d = overlap_depth(mn1, mx1, bj["min"], bj["max"])
            ex = (air["name"], bj["name"]) in exempt
            print("    z=%7.1f × %-28s 深度=%7.2fmm exempt=%s  (air z=[%.1f,%.1f])"
                  % (z, bj["label"], d, ex, mn1[2], mx1[2]))


if __name__ == "__main__":
    sys.exit(main())
