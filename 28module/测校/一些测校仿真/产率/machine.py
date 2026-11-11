#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W 产率仿真 — 整机模型 (离散事件 + 运动学节拍)
参考: 架构.md / 全流程串联.md

核心思想 (贴合用户要求的"很细致"):
  · 每个模块/组件有可设的空间位置 (mm) 与运动参数 (v, a, jerk).
  · prep/bond 路径 = 一串"动作" (move: 从A到B, 由运动学解算耗时; op: 固定/可设耗时).
  · 每个 move 的耗时 = 由起止位置的行程距离经该轴的 v/a/jerk 曲线算出 (motion.py).
  · A/B 双套并行流水: prep(不占硅片台) 可与另一套 bond(占硅片台) 并行.
    稳态节拍 T_eff = max(T_prep, T_bond); UPH = 3600 / T_eff.

坐标系: 整机右手系, 原点=键合头A吸盘中心 (架构.md §2). 单位 mm / rad / s.
"""

import math
from motion import move_time, move_time_xy


# ===================================================================
#  默认模块空间位置 (mm) — 可在GUI逐项修改
#  以键合头A吸盘中心为原点; A套在+Y侧, B套在-Y侧 (Δy≈-257mm)
# ===================================================================

DEFAULT_POSITIONS = {
    # 关键工位的 XY 位置 (mm). 名称: (x, y)
    "取片位":       (770.0, 0.0),     # 键合臂X到交接/取片位 (蓝膜侧)
    "翻转交接位":   (600.0, 0.0),     # 翻转模块过渡后的接片位
    "FU吸盘位":     (560.0, -150.0),  # FU模块吸盘 (FU模式)
    "Dipping料盒":  (446.0, 130.0),   # 沾胶固定料盒
    "预键合位":     (-100.0, 0.0),    # 键合臂X预键合位 (硅片台上方)
    "键合位":       (0.0, 0.0),       # 键合头正下方=原点
    "对准台停靠":   (300.0, 0.0),     # 对准台待命位
    "对准台插入":   (0.0, 0.0),       # 对准台插入到键合位上下之间
}

# Z 向关键高度 (mm)
DEFAULT_Z = {
    "键合臂Z_上":   0.0,
    "键合臂Z_取片": -30.0,    # 取片下压
    "键合臂Z_沾胶": -25.0,    # Dipping下降
    "键合臂Z_键合": -40.0,    # 键合下压
    "对准台Z_插入": 5.0,
}

# ===================================================================
#  各轴运动参数 (v: mm/s 或 rad/s, a: /s², jerk: /s³)
# ===================================================================

DEFAULT_KINEMATICS = {
    # 轴名: (v, a, jerk, 单位标签)
    "硅片台XY":   (200.0, 2000.0, 20000.0, "mm"),
    "硅片台Z":    (40.0,  500.0,  5000.0,  "mm"),
    "硅片台RxRy": (0.20,  4.0,    40.0,    "rad"),
    "键合臂X":    (320.0, 3000.0, 30000.0, "mm"),
    "键合臂Z":    (120.0, 1200.0, 12000.0, "mm"),
    "键合头Y":    (80.0,  800.0,  8000.0,  "mm"),
    "键合头Rz":   (2.0,   20.0,   200.0,   "rad"),
    "翻转X":      (280.0, 2500.0, 25000.0, "mm"),
    "翻转Z":      (150.0, 1500.0, 15000.0, "mm"),
    "翻转Rx":     (6.0,   40.0,   400.0,   "rad"),
    "拾取头Rz":   (3.0,   30.0,   300.0,   "rad"),
    "FU平台XY":   (160.0, 1500.0, 15000.0, "mm"),
    "对准台XYZ":  (160.0, 1500.0, 15000.0, "mm"),
    "Dipping刮胶": (100.0, 1000.0, 10000.0, "mm"),
}

# ===================================================================
#  固定/可设的"操作"耗时 (s) — 非运动的处理时间
# ===================================================================

DEFAULT_OP_TIMES = {
    "取片吸附":       0.30,   # 真空吸附建立
    "翻转交接":       0.25,   # 交接释放/吸附
    "FU相机定位":     0.60,   # FU相机多标记定位 (每芯片)
    "沾胶接触":       0.40,   # 料盒接触沾料
    "下光谱共焦测量": 0.50,   # 下芯片3点调平测量
    "上光谱共焦测量": 0.40,   # 上芯片测量
    "调平解算补偿":   0.35,   # Δrx/Δry/Δz 解算+补偿动作
    "精对准成像":     0.45,   # 上下相机各测标记 (每标记)
    "对准解算":       0.20,   # Δx/Δy/Δrz 解算
    "精补偿动作":     0.30,   # rz/Y/X 补偿到位额外整定
    "对准确认":       0.50,   # 复测残差 (可选)
    "保压加热":       3.0,    # 保压时间 (处方, 秒)
    "键合温控建立":   0.30,   # 温度/压力建立
}


class D2WMachine:
    """整机节拍模型: 由位置+运动学+操作时间, 组装 prep/bond 路径并解算节拍."""

    def __init__(self, params=None):
        self.p = self._defaults()
        if params:
            self.p.update(params)

    @staticmethod
    def _defaults():
        p = {}
        for k, (x, y) in DEFAULT_POSITIONS.items():
            p[f"pos::{k}::x"] = x
            p[f"pos::{k}::y"] = y
        for k, z in DEFAULT_Z.items():
            p[f"z::{k}"] = z
        for k, (v, a, j, u) in DEFAULT_KINEMATICS.items():
            p[f"kin::{k}::v"] = v
            p[f"kin::{k}::a"] = a
            p[f"kin::{k}::j"] = j
        for k, t in DEFAULT_OP_TIMES.items():
            p[f"op::{k}"] = t
        # 处方/开关
        p["recipe::mode"] = "fd"          # fd=Face-Down, fu=Face-Up
        p["recipe::dip"] = True           # 沾胶
        p["recipe::level"] = True         # 调平调焦
        p["recipe::confirm"] = False      # 对准确认
        p["recipe::align_marks"] = 3      # 精对准标记数 2~4
        p["recipe::dwell_s"] = 3.0        # 保压时间
        p["recipe::N"] = 25               # 本批键合数
        p["recipe::film_dies"] = 25       # 蓝膜余量
        p["profile"] = "scurve"           # 运动剖面
        p["coupled_xy"] = True            # XY 协调插补
        return p

    # ---------- 便捷取值 ----------
    def pos(self, name):
        return (self.p[f"pos::{name}::x"], self.p[f"pos::{name}::y"])

    def zz(self, name):
        return self.p[f"z::{name}"]

    def kin(self, axis):
        return (self.p[f"kin::{axis}::v"], self.p[f"kin::{axis}::a"],
                self.p[f"kin::{axis}::j"])

    def op(self, name):
        return self.p[f"op::{name}"]

    def prof(self):
        return self.p["profile"]

    # ---------- 单轴/平面运动耗时 ----------
    def t_move_xy(self, p_from, p_to, axis):
        v, a, j = self.kin(axis)
        dx = p_to[0] - p_from[0]
        dy = p_to[1] - p_from[1]
        return move_time_xy(dx, dy, v, a, j, self.prof(), self.p["coupled_xy"])

    def t_move_1d(self, d_from, d_to, axis):
        v, a, j = self.kin(axis)
        return move_time(abs(d_to - d_from), v, a, j, self.prof())

    # ===================================================================
    #  PREP 段路径 (不占硅片台)
    #  返回动作列表: [(名称, 类型, 耗时, 段), ...] 段∈{move,op}
    # ===================================================================
    def prep_path(self):
        acts = []
        mode = self.p["recipe::mode"]
        pick = self.pos("取片位")
        flip = self.pos("翻转交接位")
        preb = self.pos("预键合位")
        dip = self.pos("Dipping料盒")

        # 1. 键合头归位取片位 (键合臂X: 预键合→取片位)
        acts.append(("键合头归位取片位", "move",
                     self.t_move_xy(preb, pick, "键合臂X"), "prep"))
        # 2. 键合臂Z 下降取片 + 吸附 + 抬升
        acts.append(("键合臂Z下降取片", "move",
                     self.t_move_1d(self.zz("键合臂Z_上"), self.zz("键合臂Z_取片"),
                                    "键合臂Z"), "prep"))
        acts.append(("取片吸附", "op", self.op("取片吸附"), "prep"))
        acts.append(("键合臂Z抬升", "move",
                     self.t_move_1d(self.zz("键合臂Z_取片"), self.zz("键合臂Z_上"),
                                    "键合臂Z"), "prep"))
        # 3. 翻转 / FU 中转
        if mode == "fd":
            # 翻转模块 rx 翻转 π + X 过渡 + 交接
            v, a, j = self.kin("翻转Rx")
            acts.append(("翻转rx(π)", "move",
                         move_time(math.pi, v, a, j, self.prof()), "prep"))
            acts.append(("翻转X过渡", "move",
                         self.t_move_xy(pick, flip, "翻转X"), "prep"))
            acts.append(("翻转交接", "op", self.op("翻转交接"), "prep"))
        else:
            # FU: 放到FU吸盘 → 相机定位 → 键合头从FU取片
            fu = self.pos("FU吸盘位")
            acts.append(("移至FU吸盘", "move",
                         self.t_move_xy(flip, fu, "FU平台XY"), "prep"))
            acts.append(("FU相机定位", "op", self.op("FU相机定位"), "prep"))
            acts.append(("键合头从FU取片", "move",
                         self.t_move_1d(0, self.zz("键合臂Z_取片"), "键合臂Z"),
                         "prep"))
        # 4. Dipping 沾胶 (可选)
        if self.p["recipe::dip"]:
            here = flip if mode == "fd" else self.pos("FU吸盘位")
            acts.append(("移到Dipping料盒", "move",
                         self.t_move_xy(here, dip, "键合臂X"), "prep"))
            acts.append(("键合臂Z下降沾胶", "move",
                         self.t_move_1d(self.zz("键合臂Z_上"),
                                        self.zz("键合臂Z_沾胶"), "键合臂Z"), "prep"))
            acts.append(("沾胶接触", "op", self.op("沾胶接触"), "prep"))
            acts.append(("键合臂Z抬升(沾胶)", "move",
                         self.t_move_1d(self.zz("键合臂Z_沾胶"),
                                        self.zz("键合臂Z_上"), "键合臂Z"), "prep"))
            last = dip
        else:
            last = flip if mode == "fd" else self.pos("FU吸盘位")
        # 5. 移到预键合位
        acts.append(("移到预键合位", "move",
                     self.t_move_xy(last, preb, "键合臂X"), "prep"))
        return acts

    # ===================================================================
    #  BOND 段路径 (占硅片台)
    # ===================================================================
    def bond_path(self, addr_dist_mm=120.0):
        """addr_dist_mm: 硅片台寻址平均行程 (下芯片间距×随机步)."""
        acts = []
        # 1. 硅片台寻址 (大范围XY, 平均行程)
        v, a, j = self.kin("硅片台XY")
        acts.append(("硅片台寻址", "move",
                     move_time(addr_dist_mm, v, a, j, self.prof()), "bond"))
        # 2. 调焦调平 (可选)
        if self.p["recipe::level"]:
            acts.append(("下光谱共焦测量", "op", self.op("下光谱共焦测量"), "bond"))
            acts.append(("上光谱共焦测量", "op", self.op("上光谱共焦测量"), "bond"))
            acts.append(("调平解算补偿", "op", self.op("调平解算补偿"), "bond"))
            # 硅片台 z/rx/ry 补偿动作 (小行程)
            acts.append(("硅片台Z补偿", "move",
                         self.t_move_1d(0, 0.05, "硅片台Z"), "bond"))
            acts.append(("硅片台RxRy补偿", "move",
                         self.t_move_1d(0, 0.001, "硅片台RxRy"), "bond"))
        # 3. 平面精对准: 对准台插入 + 每标记成像 + 解算
        dock = self.pos("对准台停靠")
        ins = self.pos("对准台插入")
        acts.append(("对准台插入", "move",
                     self.t_move_xy(dock, ins, "对准台XYZ"), "bond"))
        n_marks = int(self.p["recipe::align_marks"])
        for i in range(n_marks):
            # 对准台在标记间移动 (视场小, 每标记需移动) + 成像
            acts.append((f"对准台移标记{i+1}", "move",
                         self.t_move_xy((0, 0), (0.1, 0.1), "对准台XYZ"), "bond"))
            acts.append((f"精对准成像{i+1}", "op", self.op("精对准成像"), "bond"))
        acts.append(("对准解算", "op", self.op("对准解算"), "bond"))
        # 4. 精补偿: rz + Y + X
        acts.append(("键合头rz补偿", "move",
                     self.t_move_1d(0, 0.0003, "键合头Rz"), "bond"))
        acts.append(("键合头Y补偿", "move",
                     self.t_move_1d(0, 0.05, "键合头Y"), "bond"))
        acts.append(("键合臂X补偿", "move",
                     self.t_move_1d(0, 0.05, "键合臂X"), "bond"))
        acts.append(("精补偿整定", "op", self.op("精补偿动作"), "bond"))
        # 5. 对准确认 (可选)
        if self.p["recipe::confirm"]:
            acts.append(("对准确认", "op", self.op("对准确认"), "bond"))
        # 6. 对准台退出
        acts.append(("对准台退出", "move",
                     self.t_move_xy(ins, dock, "对准台XYZ"), "bond"))
        # 7. 热压键合: 键合臂Z下压 + 温控 + 保压 + 抬升
        acts.append(("键合温控建立", "op", self.op("键合温控建立"), "bond"))
        acts.append(("键合臂Z下压", "move",
                     self.t_move_1d(self.zz("键合臂Z_上"), self.zz("键合臂Z_键合"),
                                    "键合臂Z"), "bond"))
        acts.append(("保压加热", "op", self.p["recipe::dwell_s"], "bond"))
        acts.append(("键合臂Z抬升(键合)", "move",
                     self.t_move_1d(self.zz("键合臂Z_键合"), self.zz("键合臂Z_上"),
                                    "键合臂Z"), "bond"))
        return acts

    # ---------- 段总耗时 ----------
    def t_prep(self):
        return sum(a[2] for a in self.prep_path())

    def t_bond(self, addr_dist_mm=120.0):
        return sum(a[2] for a in self.bond_path(addr_dist_mm))

    # ===================================================================
    #  A/B 双套并行流水调度 → UPH
    # ===================================================================
    def pipeline(self, N=None, addr_dist_mm=120.0):
        """
        流水: prep(0,A) → [bond(0,A)‖prep(1,B)] → [bond(1,B)‖prep(2,A)] → ... → bond(N-1)
        稳态 T_eff = max(T_prep, T_bond); UPH = 3600/T_eff.
        总时间 = T_prep(首颗) + N×T_eff (近似, 末颗bond单独).
        """
        if N is None:
            N = int(self.p["recipe::N"])
        N = max(1, N)
        tp = self.t_prep()
        tb = self.t_bond(addr_dist_mm)
        t_eff = max(tp, tb)
        uph = 3600.0 / t_eff if t_eff > 0 else 0.0
        uph_serial = 3600.0 / (tp + tb) if (tp + tb) > 0 else 0.0
        # 总生产时间: 首颗prep + 每颗bond节拍 (稳态并行)
        # 精确: 第0颗 prep 单独, 之后每步推进 max(prep_next, bond_cur), 末颗bond单独
        # 稳态近似: total = tp + N*t_eff (首prep + N个瓶颈节拍)
        total = tp + N * t_eff
        # 蓝膜余量限制
        film = int(self.p["recipe::film_dies"])
        n_bond = min(N, film)
        return {
            "t_prep": tp, "t_bond": tb, "t_eff": t_eff,
            "uph": uph, "uph_serial": uph_serial,
            "bottleneck": "bond" if tb >= tp else "prep",
            "N": N, "n_bond": n_bond, "total_time_s": total,
            "throughput_ratio": uph / uph_serial if uph_serial > 0 else 1.0,
        }

    # ---------- 分段耗时明细 (供图表) ----------
    def breakdown(self, addr_dist_mm=120.0):
        prep = self.prep_path()
        bond = self.bond_path(addr_dist_mm)
        def agg(acts):
            mv = sum(a[2] for a in acts if a[1] == "move")
            op = sum(a[2] for a in acts if a[1] == "op")
            return mv, op
        pmv, pop = agg(prep)
        bmv, bop = agg(bond)
        return {"prep": prep, "bond": bond,
                "prep_move": pmv, "prep_op": pop,
                "bond_move": bmv, "bond_op": bop}


if __name__ == "__main__":
    m = D2WMachine()
    print("=== 默认配置节拍 ===")
    r = m.pipeline()
    print(f"T_prep = {r['t_prep']:.2f} s")
    print(f"T_bond = {r['t_bond']:.2f} s")
    print(f"T_eff  = {r['t_eff']:.2f} s ({r['bottleneck']} 瓶颈)")
    print(f"UPH(双套流水) = {r['uph']:.0f}  UPH(串行) = {r['uph_serial']:.0f}  "
          f"提升 ×{r['throughput_ratio']:.2f}")
    print(f"本批 {r['n_bond']} 颗总耗时 = {r['total_time_s']:.1f} s "
          f"= {r['total_time_s']/60:.1f} min")

    print("\n=== PREP 段明细 ===")
    for name, kind, t, seg in m.prep_path():
        print(f"  [{kind:4s}] {name:20s} {t*1000:7.1f} ms")
    print("\n=== BOND 段明细 ===")
    for name, kind, t, seg in m.bond_path():
        print(f"  [{kind:4s}] {name:20s} {t*1000:7.1f} ms")

    print("\n=== 保压时间敏感性 ===")
    for dwell in (1, 2, 3, 5, 8):
        m.p["recipe::dwell_s"] = float(dwell)
        r = m.pipeline()
        print(f"  保压{dwell}s: T_bond={r['t_bond']:.2f} T_eff={r['t_eff']:.2f} "
              f"UPH={r['uph']:.0f} ({r['bottleneck']})")
