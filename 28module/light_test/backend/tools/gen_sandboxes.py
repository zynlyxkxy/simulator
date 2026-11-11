# -*- coding: utf-8 -*-
"""gen_sandboxes.py v2 — 零件级沙盒（AABB）表生成器。

用法：python backend/tools/gen_sandboxes.py
输入：html_model/d2w_front.json（TwinDOM 工程原文件，只读）
输出：backend/runtime/sandboxes.json（引擎 runtime/collision.py 加载）

模型事实（2026-08 版，schema 0.2.0）：
- objects 平铺列表：{id,name,type,parentId,transform{position,rotation(deg),scale},
  geometry}；坐标系 RH_XYZ_Z_UP（Z 向上，单位 mm）；geometry：Box/RoundedBox
  （width→X, depth→Y, height→Z）、Cylinder（radius/height，Z 轴）、Lathe
  （profile=[[半径,z]...] 回转体）、Ring/Circle（XY 平面，近零厚度）、Extrude
  （shape=XY 轮廓，depth 沿 Z）。
- axes[i] = {movingObjectId, type(linear/rotary), direction(X/Y/Z, authored-world),
  directionSign, min/max/home/current, pivot(pivotSpace=parent → pivotReferenceId)}。
  轴驱动 = movingObject 子树整体随轴动。

v2 粒度重构（2026-08-20，穿模漏报复盘）：
- v1 把每根轴的整串子树聚成 1 个 AABB（如键合头链 12 件合一），装配体零位互相
  嵌套 → 生成期成批豁免/剪枝把真穿模吞掉（用户实测三例漏报）。v2 改为
  **每个几何体一个盒**，盒标注所属**刚性运动组**（= 最近祖先 movingObject 的
  轴链：自身轴 + 全部祖先运动对象的轴）。
- TwinDOM 轴语义（2026-08-27 迭代 4 明确）：`position = authored − current + 轴值`。
  模型对象 transform 是 authored 姿态（= 轴值=current 时的位姿），因此生成器
  要把世界 AABB 沿各轴链元的 **current 反方向平移**，得到“轴值=0”时的零位盒，
  与 runtime collision（轴值×linear 方向）一致。
- 配对：同组零件永不互查（刚性一体）；固定×固定不查；双方都无直线轴（纯旋转
  不变盒相对静止）不查；直线行程包络恒不相交剪枝。
- **穿模深度阈值 ε**（默认 2mm）：三向重叠量的最小值 > ε 才判撞（="把两个零件
  拔出来最少要走多远"）。面贴合/穿薄膜类工艺动作（压头贴晶圆、吸盘贴蓝膜、
  顶针穿膜）重叠深度天然 < ε 放行；ε=2mm 的依据：晶圆 0.7mm + 压合行程 ~1mm
  要容忍，而结构件厚度均 ≥3mm（蓝膜台体 3mm、吸盘基座 32mm），真穿模必超 ε。
  运行时可经 /api/collision 的 depth_eps 在线改。
- 豁免两类（都进 exempt_pairs 并逐对打印在生成报告里，供人工复审）：
  1. 零位跨组重叠且深度 > ε → 设计贴合/装配嵌套（如顶针在吸盘基座孔位里，
     AABB 没有孔概念）——逐对列出；
  2. PROCESS_EXEMPT 显式登记的工艺/设计接触对（零件级，不许整组豁免）。
  零位重叠深度 ≤ ε 的对**不豁免**（留在对表里，压深超 ε 才报警——正是
  "浅贴合放行、深穿模报警"的语义）。
- 旋转轴仍做生成期扫掠不变盒（逐零件扫掠，比 v1 聚合盒紧得多）。
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_JSON = os.path.normpath(os.path.join(
    HERE, "..", "..", "html_model", "d2w_front.json"))
OUT_JSON = os.path.normpath(os.path.join(HERE, "..", "runtime", "sandboxes.json"))

# 穿模深度阈值 ε（mm）：三向最小重叠量 > ε 才判撞。运行时 /api/collision 可改。
DEPTH_EPS = 2.0

# 工艺/设计接触豁免对（零件级盒名 pt:<对象id>，按生成报告核对后登记）：
# - Epin 顶针/片压 × 多孔陶瓷面：顶针 +30mm 顶出晶圆要走陶瓷面上的过孔，
#   针座片压（半己型片压，随针一体升降）走陶瓷底面的沉孔让位——孔/沉孔在
#   AABB 里不存在，零位不重叠（针尖 873.9 < 片压顶 878.4 < 陶瓷底 878.9）
#   所以不在"设计贴合"自动豁免里，须显式登记。滑块/导轨都不碰陶瓷面，
#   不豁免（导轨随组上升是模型伪影，保留报警，见交接文档遗留风险）。
# - 对准台X/Y轴承接板：层叠滑台副，零位顶贴底（z=1033.0 相接，深度 0 进不了
#   自动豁免），AS_Z≥+2.1 起 Y 板顶进入 X 板底面区（真机为槽/孔让位，
#   AABB 不可见）；AS_Z 全行程 [-2,+6] 只有这一对挡路，home.fault 用例
#   （开关不可达→走到软限位）被它拦而暴露。
PROCESS_EXEMPT = [
    (("pt:epin1_pin_0001", "pt:epin2_pin_0001", "pt:epin3_pin_0001",
      "pt:epin1_tab_0001", "pt:epin2_tab_0001", "pt:epin3_tab_0001"),
     ("pt:ceramic_msslygtz_8wc595",),
     "Epin顶针/片压×多孔陶瓷面：顶出晶圆走过孔/沉孔（AABB 无孔概念，设计接触）"),
    (("pt:mechanicalobject_msssjh6b_teh3bf",),
     ("pt:box_msst6r7i_ppzlv5",),
     "对准台X/Y轴承接板：层叠滑台副，AS_Z 行程内结构交叠（真机槽/孔让位）"),
    (("pt:airring_left_0001", "pt:airring_right_0001",
      "pt:airring_front_0001", "pt:airring_back_0001",
      "pt:box_mssrsdpg_u0q0bi",
      "pt:bondhead_cross_h_0001", "pt:bondhead_cross_v_0001"),
     ("pt:chuck_base_notched_0001",),
     "键合头气环/吸盘/十字槽×吸盘基座：基座是带Epin缺口的环形件，"
     "AABB 把中心孔填成实心方盒；键合头下降到真接触位（BS_Arm_Z≈-167.7）"
     "时从孔中穿过（AABB 无孔概念，设计让位）。遗留风险：晶圆台 X/Y 大偏"
     "（>~60mm）使基座边缘进到键合头正下方时不再报警"),
]
# 说明：以下 v1 链级豁免在零件级 + ε 语义下**全部取消**，经示教位探针验证
# （见生成报告"示教位探针"节）：
# - 键合头×晶圆台：接触对是 键合头吸盘/十字槽 × 吸盘基座顶面，示教位
#   BS_Arm_Z=-162.5 时重叠 0.5~0.7mm < ε 天然放行；压深超 ε 应报警（真穿模）。
# - 翻转取片×蓝膜：蓝膜厚 0.6mm，任何穿膜深度 ≤0.6 < ε 天然放行；示教位
#   FA_Z=-5.0（吸盘面贴膜）；压进台体（3mm 厚）超 ε 报警。
# - 蓝膜顶针×蓝膜：顶针穿膜深度 ≤0.6 < ε 天然放行；针体×台体是零位设计贴合
#   （针在台体孔里，深度 2.4>ε）走自动豁免。
# - 键合头链×微动基座/Tray：v1 聚合盒假象，零件级在示教行程内不相碰。
# - 键合头链×对准相机承载台：上一轮已移出（真机流程先退避 AS 相机），保持不豁免。

EPS = 1e-6          # 重叠判定余量（mm）：严格大于才算重叠
RING_HALF_T = 0.25  # Ring/Circle 平面几何的兜底半厚（mm）

# TwinDOM 轴 id → 仿真轴名（与 runtime/axes_config.FRONTEND_IDS 同一事实源）
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "runtime")))
import axes_config as _ac            # noqa: E402
AXIS_SIM_NAME = _ac.ID_TO_NAME       # 前端轴 id -> 仿真轴名（如 WS_LS_X）


# ---------------- 最小 3D 线性代数（纯标准库，只够用） ----------------
def mat_identity():
    return ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def mat_trs(pos, rot_deg, scale):
    """T·Rz·Ry·Rx·S（模型内旋转全为 Z 向，欧拉阶不影响；非 Z 旋转另作告警）。"""
    rx, ry, rz = (math.radians(v) for v in rot_deg)
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    # R = Rz·Ry·Rx
    r = ((cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
         (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
         (-sy, cy * sx, cy * cx))
    sx_, sy_, sz_ = scale
    return ((r[0][0] * sx_, r[0][1] * sy_, r[0][2] * sz_, pos[0]),
            (r[1][0] * sx_, r[1][1] * sy_, r[1][2] * sz_, pos[1]),
            (r[2][0] * sx_, r[2][1] * sy_, r[2][2] * sz_, pos[2]),
            (0.0, 0.0, 0.0, 1.0))


def mat_point(m, p):
    return (m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
            m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
            m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3])


def corners(mn, mx):
    return [(x, y, z) for x in (mn[0], mx[0])
            for y in (mn[1], mx[1]) for z in (mn[2], mx[2])]


def overlap_depth(mn1, mx1, mn2, mx2):
    """三向重叠量的最小值（="拔出最少要走多远"，mm）；任一向不重叠返回 ≤0。"""
    return min(min(mx1[i], mx2[i]) - max(mn1[i], mn2[i]) for i in range(3))


def aabb_hit(mn1, mx1, mn2, mx2, depth_eps=DEPTH_EPS):
    """判撞口径：三向都重叠且最小重叠深度 > depth_eps。"""
    return overlap_depth(mn1, mx1, mn2, mx2) > depth_eps


def local_aabb(geo):
    """geometry → 局部 AABB（模型坐标：width→X, depth→Y, height→Z，Z 向上）。"""
    t = geo.get("type")
    if t in ("Box", "RoundedBox"):
        return ((-geo["width"] / 2, -geo["depth"] / 2, -geo["height"] / 2),
                (geo["width"] / 2, geo["depth"] / 2, geo["height"] / 2))
    if t == "Cylinder":
        r, h = geo["radius"], geo["height"]
        return ((-r, -r, -h / 2), (r, r, h / 2))
    if t == "Lathe":
        rs = [abs(p[0]) for p in geo["profile"]]
        zs = [p[1] for p in geo["profile"]]
        r = max(rs)
        return ((-r, -r, min(zs)), (r, r, max(zs)))
    if t == "Ring":
        r = geo["outerRadius"]
        return ((-r, -r, -RING_HALF_T), (r, r, RING_HALF_T))
    if t == "Circle":
        r = geo["radius"]
        return ((-r, -r, -RING_HALF_T), (r, r, RING_HALF_T))
    if t == "Extrude":
        xs = [p[0] for p in geo["shape"]]
        ys = [p[1] for p in geo["shape"]]
        return ((min(xs), min(ys), 0.0),
                (max(xs), max(ys), geo.get("depth", 0.0)))
    raise ValueError("未知 geometry 类型: %r" % t)


def rotate_about(p, pivot, axis, rad):
    """Rodrigues：点 p 绕 axis 方向过 pivot 的轴旋转 rad。"""
    ux, uy, uz = axis
    x, y, z = p[0] - pivot[0], p[1] - pivot[1], p[2] - pivot[2]
    c, s = math.cos(rad), math.sin(rad)
    dot = ux * x + uy * y + uz * z
    return (pivot[0] + x * c + (uy * z - uz * y) * s + ux * dot * (1 - c),
            pivot[1] + y * c + (uz * x - ux * z) * s + uy * dot * (1 - c),
            pivot[2] + z * c + (ux * y - uy * x) * s + uz * dot * (1 - c))


def sweep_rotary(mn, mx, pivot, axis, lo_deg, hi_deg):
    """旋转不变盒：零位盒 8 角在 [lo,hi] 行程内扫掠（端点 + 区间内 90° 整数倍
    采样——圆周运动的 AABB 极值必在这些角度取得），取外接盒。"""
    angles = [lo_deg, hi_deg]
    k0, k1 = math.ceil(lo_deg / 90.0), math.floor(hi_deg / 90.0)
    angles += [90.0 * k for k in range(int(k0), int(k1) + 1)]
    pts = [rotate_about(c, pivot, axis, math.radians(a))
           for c in corners(mn, mx) for a in angles]
    return (tuple(min(p[i] for p in pts) for i in range(3)),
            tuple(max(p[i] for p in pts) for i in range(3)))


def main():
    with open(MODEL_JSON, encoding="utf-8") as f:
        model = json.load(f)
    objs = model["objects"]
    by_id = {o["id"]: o for o in objs}
    children = {}
    for o in objs:
        children.setdefault(o["parentId"], []).append(o["id"])

    # ---- 世界矩阵（authored 零位姿态；非 Z 向旋转告警——本模型实测全为 Z 向） ----
    _wm = {}

    def world_mat(oid):
        if oid in _wm:
            return _wm[oid]
        o = by_id[oid]
        t = o["transform"]
        if any(abs(t["rotation"][i]) > 1e-9 for i in (0, 1)):
            print("[warn] 非 Z 向旋转 %s(%s) rot=%s，欧拉阶按 Rz·Ry·Rx 处理"
                  % (oid, o["name"], t["rotation"]))
        m = mat_trs(t["position"], t["rotation"], t["scale"])
        pid = o["parentId"]
        m = mat_mul(world_mat(pid), m) if pid else m
        _wm[oid] = m
        return m

    def world_aabb(oid):
        mn, mx = local_aabb(by_id[oid]["geometry"])
        pts = [mat_point(world_mat(oid), c) for c in corners(mn, mx)]
        return (tuple(min(p[i] for p in pts) for i in range(3)),
                tuple(max(p[i] for p in pts) for i in range(3)))

    # ---- 轴表：movingObject → 轴 ----
    axes = model["axes"]
    # TwinDOM 语义：position = authored − current + 轴值； authored 姿态即
    # 轴值=current 时的位姿。生成零位盒时要把 current 偏移沿链元反方向扣掉。
    axis_current = {a["id"]: float(a.get("current", 0.0)) for a in axes}
    obj_axes = {}
    for a in axes:
        obj_axes.setdefault(a["movingObjectId"], []).append(a)
    moving_ids = set(obj_axes)
    dir_vec = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}

    def group_root(oid):
        """oid 所属刚性运动组：最近祖先（含自身）movingObject；无 → None（固定）。"""
        while oid:
            if oid in moving_ids:
                return oid
            oid = by_id[oid]["parentId"]
        return None

    def ancestor_axes(oid):
        """oid 的轴贡献链：自身（若是运动对象）+ 全部祖先运动对象的轴。"""
        out, cur = [], oid
        while cur:
            if cur in obj_axes:
                out = obj_axes[cur] + out      # 祖先在前（扫掠顺序先祖先后自身）
            cur = by_id[cur]["parentId"]
        return out

    # ---- 零件级盒：每个几何体一盒；运动件按组轴链扫掠/挂直线贡献 ----
    boxes = []
    group_info = {}     # group_root oid -> {"label":..., "sim_axes":...}
    for o in objs:
        oid = o["id"]
        if not o.get("geometry"):
            continue
        g = group_root(oid)
        mn, mx = world_aabb(oid)
        linear, rotary = {}, []
        if g is not None:
            chain = ancestor_axes(g)
            # 扣掉链上各直线轴 current 造成的 authored 偏移，得到轴值=0 的盒
            off = [0.0, 0.0, 0.0]
            for a in chain:
                if a["type"] == "linear":
                    v = dir_vec[a["direction"]]
                    s = a.get("directionSign", 1)
                    cur = axis_current.get(a["id"], 0.0)
                    off[0] += v[0] * s * cur
                    off[1] += v[1] * s * cur
                    off[2] += v[2] * s * cur
            mn = (mn[0] - off[0], mn[1] - off[1], mn[2] - off[2])
            mx = (mx[0] - off[0], mx[1] - off[1], mx[2] - off[2])
            for a in chain:
                if a["type"] == "linear":
                    v = dir_vec[a["direction"]]
                    s = a.get("directionSign", 1)
                    linear[AXIS_SIM_NAME.get(a["id"], a["id"])] = \
                        [v[0] * s, v[1] * s, v[2] * s, a["min"], a["max"]]
                else:
                    pw = mat_point(world_mat(a["pivotReferenceId"]), a["pivot"])
                    pw = (pw[0] - off[0], pw[1] - off[1], pw[2] - off[2])
                    mn, mx = sweep_rotary(mn, mx, pw, dir_vec[a["direction"]],
                                          a["min"], a["max"])
                    rotary.append(AXIS_SIM_NAME.get(a["id"], a["id"]))
            if g not in group_info:
                sim = [AXIS_SIM_NAME.get(a["id"], a["id"])
                       for a in obj_axes[g]]
                group_info[g] = {
                    "label": by_id[g]["name"] + "组（轴:" + "+".join(
                        AXIS_SIM_NAME.get(a["id"], a["id"]) for a in chain) + "）",
                    "sim_axes": sim,
                }
            group_name = "grp:" + g
            group_label = group_info[g]["label"]
        else:
            group_name = "fixed"
            group_label = "固定"
        boxes.append({
            "name": "pt:" + oid,
            "label": "%s〔%s〕" % (o["name"], group_label),
            "part": o["name"],
            "kind": "moving" if g is not None else "fixed",
            "group": group_name,
            "object": oid,
            "min": [round(v, 4) for v in mn],
            "max": [round(v, 4) for v in mx],
            # linear: 仿真轴名 -> [fx,fy,fz,lo,hi]（位移=current×f，mm）
            "linear": linear,
            "rotary": rotary,
        })

    idx = {b["name"]: i for i, b in enumerate(boxes)}

    def proc_exempt(na, nb):
        for pats_a, pats_b, reason in PROCESS_EXEMPT:
            for pa in pats_a:
                for pb in pats_b:
                    if (na == pa and nb == pb) or (na == pb and nb == pa):
                        return reason
        return None

    # ---- 对表：跨组；固定×固定/纯旋转相对静止跳过；零位深重叠=设计贴合豁免 ----
    proc_matched = [False] * len(PROCESS_EXEMPT)
    zero_fits = []      # 零位跨组重叠（深度>ε）→ 自动豁免 + 报告
    pairs, exempt = [], []
    n_same_group = n_fixed_fixed = n_no_linear = 0
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            bi, bj = boxes[i], boxes[j]
            if bi["group"] == bj["group"]:
                n_same_group += 1
                continue                     # 同组刚性一体，永不互查
            if bi["kind"] == "fixed" and bj["kind"] == "fixed":
                n_fixed_fixed += 1
                continue
            if not bi["linear"] and not bj["linear"]:
                n_no_linear += 1
                continue                     # 双方都无直线轴，相对静止
            reason = proc_exempt(bi["name"], bj["name"])
            if reason:
                for k, (pa, pb, r) in enumerate(PROCESS_EXEMPT):
                    if r == reason:
                        proc_matched[k] = True
                exempt.append({"a": bi["name"], "b": bj["name"],
                               "reason": reason})
                continue
            d0 = overlap_depth(bi["min"], bi["max"], bj["min"], bj["max"])
            if d0 > DEPTH_EPS:
                zero_fits.append((bi, bj, d0))
                exempt.append({"a": bi["name"], "b": bj["name"],
                               "reason": "零位设计贴合/装配嵌套（跨组重叠 %.2fmm "
                                         "> ε，零件级复审见生成报告）" % d0})
                continue
            pairs.append({"a": bi["name"], "b": bj["name"], "hit_axes": []})

    # ---- 包络剪枝：每盒沿直线轴行程的可达范围（保守外扩），
    # 三个维度任一包络恒不相交的对子永远撞不上，直接从对表剔除（运行时零成本） ----
    def envelope(b):
        mn = list(b["min"])
        mx = list(b["max"])
        for _aid, (fx, fy, fz, lo, hi) in b["linear"].items():
            for k, f in ((0, fx), (1, fy), (2, fz)):
                if f:
                    mn[k] += min(f * lo, f * hi)
                    mx[k] += max(f * lo, f * hi)
        return mn, mx

    env = {b["name"]: envelope(b) for b in boxes}
    n_pruned = 0
    kept = []
    for p in pairs:
        e1, e2 = env[p["a"]], env[p["b"]]
        if any(e1[0][k] >= e2[1][k] - EPS or e2[0][k] >= e1[1][k] - EPS
               for k in range(3)):
            n_pruned += 1
            continue
        kept.append(p)
    pairs = kept

    # ---- 可撞性扫掠：单轴走行程采样（ε 口径），标记哪些轴能把对子撞上 ----
    # 采样须覆盖"窗口型重叠"（如 Epin 片压仅 v∈[0.5,9.4] 穿过陶瓷面，三点
    # 采样会漏）：按行程自适应步长（≤2mm，rad 轴同理按量纲；上限 400 点）。
    def box_at(b, axis_pos):
        mn = list(b["min"])
        mx = list(b["max"])
        for aid, (fx, fy, fz, _lo, _hi) in b["linear"].items():
            v = axis_pos.get(aid, 0.0)
            mn[0] += fx * v; mx[0] += fx * v
            mn[1] += fy * v; mx[1] += fy * v
            mn[2] += fz * v; mx[2] += fz * v
        return mn, mx

    for p in pairs:
        bi, bj = boxes[idx[p["a"]]], boxes[idx[p["b"]]]
        axes_in_pair = {}
        for b in (bi, bj):
            for aid, (_f, _g, _h, lo, hi) in b["linear"].items():
                axes_in_pair[aid] = (lo, hi)
        for aid, (lo, hi) in axes_in_pair.items():
            n = min(400, max(2, int((hi - lo) / 2.0) + 1))
            for s in range(n + 1):
                v = lo + (hi - lo) * s / n
                mn1, mx1 = box_at(bi, {aid: v})
                mn2, mx2 = box_at(bj, {aid: v})
                if aabb_hit(mn1, mx1, mn2, mx2):
                    p["hit_axes"].append(aid)
                    break

    out = {
        "_comment": ("零件级沙盒（AABB）表——由 tools/gen_sandboxes.py v2 从 "
                     "TwinDOM 模型自动生成，勿手改；模型更新后重新运行生成脚本。"
                     "盒=零件零位世界 AABB（旋转轴已按零件扫掠成不变盒）；运行时按 "
                     "linear 贡献平移；判撞=三向最小重叠深度 > depth_eps。"),
        "schema": 2,
        "depth_eps": DEPTH_EPS,
        "source": os.path.basename(MODEL_JSON),
        "boxes": boxes,
        "pairs": pairs,
        "exempt_pairs": exempt,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
        f.write("\n")

    # ---- 示教位探针：关键工艺姿势下有哪些深度>ε 的跨组重叠 ----
    # （验证示教位本身不是穿模；poses 与 transfers.json/flows.json 对齐）
    probes = [
        ("键合接触位（head_to_wafer：WS 零位 / AS_Y=120 退避 / BS_Arm_Z=-162.5）",
         {"AS_Y": 120.0, "BS_Arm_Z": -162.5}),
        ("翻转交接位（flip_to_head：WS_LS_X=+220 退避 / AS_Y=120 / FA_X=-600 / "
         "BS_Arm_Z=-185）",
         {"WS_LS_X": 220.0, "AS_Y": 120.0, "FA_X": -600.0, "BS_Arm_Z": -185.0}),
        ("蓝膜取片位（film_to_flip：FA_Z=-5.0，其余零位）",
         {"FA_Z": -5.0}),
        ("顶针顶出（ES_PunchZ=+5）", {"ES_PunchZ": 5.0}),
        ("Epin 顶出（E_EPIN_Z×3=+30）",
         {"E_EPIN_Z_1": 30.0, "E_EPIN_Z_2": 30.0, "E_EPIN_Z_3": 30.0}),
    ]
    exempt_set = set()
    for e in exempt:
        exempt_set.add((e["a"], e["b"]))
        exempt_set.add((e["b"], e["a"]))
    probe_lines = []
    for title, pose in probes:
        hits = []
        cur = {}
        for b in boxes:
            cur[b["name"]] = box_at(b, pose)
        for p in pairs:
            mn1, mx1 = cur[p["a"]]
            mn2, mx2 = cur[p["b"]]
            d = overlap_depth(mn1, mx1, mn2, mx2)
            if d > DEPTH_EPS:
                hits.append((p["a"], p["b"], d))
        probe_lines.append((title, hits))

    # ---- 报告 ----
    for k, (pa, pb, _r) in enumerate(PROCESS_EXEMPT):
        if not proc_matched[k]:
            print("[warn] 工艺豁免模式未命中任何对：%s × %s（模型变了？）"
                  % (pa, pb))
    n_mv = sum(1 for b in boxes if b["kind"] == "moving")
    n_fx = len(boxes) - n_mv
    print("盒：%d（运动组内零件 %d / 固定 %d），运动组 %d 个"
          % (len(boxes), n_mv, n_fx, len(group_info)))
    print("候选对剔除：同组 %d / 固定×固定 %d / 纯旋转相对静止 %d"
          % (n_same_group, n_fixed_fixed, n_no_linear))
    print("对表：%d 有效 / %d 豁免（显式工艺 %d 条规则 + 零位设计贴合 %d 对）"
          " / %d 包络剪枝剔除"
          % (len(pairs), len(exempt),
             sum(1 for m in proc_matched if m), len(zero_fits), n_pruned))
    print("\n零位跨组重叠（深度>ε，自动豁免，逐对复审）：%d 对" % len(zero_fits))
    for bi, bj, d in zero_fits:
        print("  [%6.2fmm] %s × %s" % (d, bi["label"], bj["label"]))
    print("\n示教位探针（深度>ε 才算撞；空=该示教位无穿模）：")
    for title, hits in probe_lines:
        print("  %s" % title)
        if not hits:
            print("    （无）")
        for a, b, d in hits:
            print("    [%6.2fmm] %s × %s"
                  % (d, boxes[idx[a]]["label"], boxes[idx[b]]["label"]))
    print("\n可撞对（hit_axes 非空）：%d / %d"
          % (sum(1 for p in pairs if p["hit_axes"]), len(pairs)))
    for p in pairs:
        if p["hit_axes"]:
            print("  %s × %s  <- %s" % (boxes[idx[p["a"]]]["label"],
                                        boxes[idx[p["b"]]]["label"],
                                        ",".join(p["hit_axes"])))
    print("已写出 %s" % OUT_JSON)


if __name__ == "__main__":
    sys.exit(main())
