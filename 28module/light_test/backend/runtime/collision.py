# -*- coding: utf-8 -*-
"""collision.py — 沙盒干涉检查（零件级 AABB + 穿模深度阈值）。

数据源 runtime/sandboxes.json（tools/gen_sandboxes.py v2 从 TwinDOM 前端模型
生成，schema 2）：
- 每盒 = 一个零件的零位世界 AABB（旋转轴已按零件扫掠成"旋转不变盒"，运行时
  不再随旋转轴更新）；运动件盒带 linear 贡献表 {仿真轴名: [fx,fy,fz]}，
  当前盒 = 零位盒 + Σ 轴位移×方向（mm）。
- 配对已在生成期过滤：同刚性运动组不查、固定×固定不查、纯旋转相对静止不查、
  零位设计贴合/工艺接触豁免、轴包络恒不相交剪枝。

判撞口径（与生成器一致）：两盒三向重叠量的最小值（="拔出最少要走多远"）>
depth_eps（默认 2mm，sandboxes.json 带默认值，/api/collision 可在线改）。
面贴合/穿薄膜类工艺接触天然 < ε 放行；真穿模（嵌进结构件）必超 ε 报警。

运行时两条链：
- move 起步检查 check_move：目标位撞入 → 引擎抛 InterlockError（409）；
  已在重叠中的对（tolerated）豁免起步检查，允许从重叠区退出。
- tick 检查 check_tick（500Hz，每 2 tick）：只做增量——轴位置没变的盒不重算，
  不碰脏盒的对不复查；新撞入（非 tolerated）→ 引擎全轴 soft-stop + 记录事件。
"""
import json
from collections import deque

EPS = 1e-6          # 重叠判定余量（mm），与生成脚本一致
EVENTS_MAX = 50     # 干涉事件环形记录
DEPTH_EPS_DEFAULT = 2.0     # 穿模深度阈值默认（mm），json 可覆盖
DEPTH_EPS_MAX = 50.0        # 在线配置上限（防误填）


class CollisionChecker(object):
    def __init__(self, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        self.boxes = data["boxes"]
        self.n_exempt = len(data.get("exempt_pairs", []))
        self.depth_eps = float(data.get("depth_eps", DEPTH_EPS_DEFAULT))
        idx = {b["name"]: i for i, b in enumerate(self.boxes)}
        self._label = {b["name"]: b.get("label", b["name"])
                       for b in self.boxes}
        # base = 零位 AABB；lin = 直线轴贡献（位移 = pos × (fx,fy,fz)）
        self._base = [(list(b["min"]), list(b["max"])) for b in self.boxes]
        self._lin = [{a: tuple(v[:3]) for a, v in b["linear"].items()}
                     for b in self.boxes]
        self._cur = [[list(b["min"]), list(b["max"])] for b in self.boxes]
        # 对表 + 索引（轴→盒→对，两级增量）
        self.pairs = []
        self._pair_axes = []
        self._box_pairs = {}
        self._axis_boxes = {}
        for pi, p in enumerate(data.get("pairs", [])):
            i, j = idx[p["a"]], idx[p["b"]]
            self.pairs.append((i, j))
            self._pair_axes.append(
                set(self._lin[i]) | set(self._lin[j]))
            self._box_pairs.setdefault(i, []).append(pi)
            self._box_pairs.setdefault(j, []).append(pi)
        for bi, lin in enumerate(self._lin):
            for a in lin:
                self._axis_boxes.setdefault(a, []).append(bi)
        self._pos = {}                # 最近一次见到的轴位置
        self.overlapping = set()      # 当前重叠中的对（pair idx）
        self.tolerated = set()        # move 起步时已重叠的对（允许退出重叠区）
        self.events = deque(maxlen=EVENTS_MAX)
        self._first = True

    # ---------------- 内部 ----------------
    def _overlap(self, i, j):
        """判撞：三向都重叠且最小重叠深度 > depth_eps。"""
        a, b = self._cur[i], self._cur[j]
        depth = min(min(a[1][k], b[1][k]) - max(a[0][k], b[0][k])
                    for k in range(3))
        return depth > self.depth_eps

    def set_depth_eps(self, value):
        """在线改穿模深度阈值（mm）。返回生效值；非法值抛 ValueError。"""
        v = float(value)
        if not (0.0 <= v <= DEPTH_EPS_MAX) or v != v:   # NaN 检查
            raise ValueError("depth_eps 须在 [0, %g] 内" % DEPTH_EPS_MAX)
        self.depth_eps = v
        # 阈值变化会改变判定结果：清重叠缓存，下个 tick 全量重建一次
        self.overlapping = set()
        self.tolerated = set()
        self._first = True
        return self.depth_eps

    def _recompute(self, bi):
        mn, mx = self._base[bi]
        out_min = list(mn)
        out_max = list(mx)
        for a, (fx, fy, fz) in self._lin[bi].items():
            v = self._pos.get(a, 0.0)
            out_min[0] += fx * v
            out_max[0] += fx * v
            out_min[1] += fy * v
            out_max[1] += fy * v
            out_min[2] += fz * v
            out_max[2] += fz * v
        self._cur[bi] = [out_min, out_max]

    def _box_at(self, bi, axis, value, positions):
        """假设 axis 位于 value（其余轴取 positions），盒 bi 的 AABB。"""
        mn, mx = self._base[bi]
        out_min = list(mn)
        out_max = list(mx)
        for a, (fx, fy, fz) in self._lin[bi].items():
            v = value if a == axis else positions.get(a, 0.0)
            out_min[0] += fx * v
            out_max[0] += fx * v
            out_min[1] += fy * v
            out_max[1] += fy * v
            out_min[2] += fz * v
            out_max[2] += fz * v
        return out_min, out_max

    def _pair_name(self, pi):
        i, j = self.pairs[pi]
        return (self._label[self.boxes[i]["name"]],
                self._label[self.boxes[j]["name"]])

    # ---------------- 运行时接口（引擎锁内调用） ----------------
    def check_tick(self, positions):
        """每 2 tick 增量检查。返回新撞入的对名列表 [(labelA, labelB), ...]。"""
        changed = [a for a, v in positions.items()
                   if self._pos.get(a) != v]
        self._pos.update(positions)
        if self._first:
            self._first = False
            for bi in range(len(self.boxes)):
                self._recompute(bi)
            self.overlapping = set(
                pi for pi in range(len(self.pairs))
                if self._overlap(*self.pairs[pi]))
            return []
        if not changed:
            return []
        dirty = set()
        for a in changed:
            dirty.update(self._axis_boxes.get(a, ()))
        for bi in dirty:
            self._recompute(bi)
        new_hits = []
        seen = set()
        for bi in dirty:
            for pi in self._box_pairs.get(bi, ()):
                if pi in seen:
                    continue
                seen.add(pi)
                ov = self._overlap(*self.pairs[pi])
                had = pi in self.overlapping
                if ov and not had:
                    self.overlapping.add(pi)
                    if pi not in self.tolerated:
                        new_hits.append(pi)
                elif not ov:
                    if had:
                        self.overlapping.discard(pi)
                    self.tolerated.discard(pi)   # 退出重叠区后解除容忍
        return [self._pair_name(pi) for pi in new_hits]

    def check_move(self, axis, target, positions):
        """move 起步检查：axis 走到 target 时哪些对会撞入（返回对名列表）。
        当前已重叠的对转入 tolerated（不拦退出重叠区的回撤运动）。"""
        if self.overlapping:
            self.tolerated |= self.overlapping
        hits = []
        for pi, axes in enumerate(self._pair_axes):
            if axis not in axes or pi in self.tolerated:
                continue
            i, j = self.pairs[pi]
            mn1, mx1 = self._box_at(i, axis, target, positions)
            mn2, mx2 = self._box_at(j, axis, target, positions)
            depth = min(min(mx1[k], mx2[k]) - max(mn1[k], mn2[k])
                        for k in range(3))
            if depth > self.depth_eps:
                hits.append(self._pair_name(pi))
        return hits

    def record_event(self, sim_time, hits, stopped_axes):
        self.events.append({
            "t": round(sim_time, 3),
            "pairs": [list(h) for h in hits],
            "stopped": list(stopped_axes),
        })

    def state(self):
        n_mv = sum(1 for b in self.boxes if b["kind"] == "moving")
        return {"boxes": len(self.boxes), "moving": n_mv,
                "fixed": len(self.boxes) - n_mv,
                "pairs": len(self.pairs), "exempt": self.n_exempt,
                "depth_eps": self.depth_eps,
                "active": [list(self._pair_name(pi))
                           for pi in sorted(self.overlapping)],
                "events": list(self.events)}
