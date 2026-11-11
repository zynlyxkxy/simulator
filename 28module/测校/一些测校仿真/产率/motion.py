#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W 产率仿真 — 运动学内核
每个自由度按 距离 → 速度/加速度/加加速度(jerk) 曲线 解算真实运动耗时。

支持两种运动规划:
  · trapezoidal (梯形速度): 加速段(匀加速 a) - 匀速段(v) - 减速段. jerk=∞.
  · scurve (S曲线/jerk-limited): 七段, 加加速度 j 有限, 更贴近实际伺服.

解算给定行程 D(>0) 下的运动时间 move_time(D, v, a, j):
  - trapezoidal: 若能达到 v_max → t = D/v + v/a; 否则三角形 t = 2·sqrt(D/a)
  - scurve: 分 "是否达到 v_max"、"是否达到 a_max" 分类求解, 闭式/数值兼顾

单位: 距离与 v/a/j 用户自定 (mm, mm/s, mm/s², mm/s³ 或 rad 系).
本模块只关心标量行程 → 时间, 与物理量纲无关.
"""

import math


def trap_time(D, v, a):
    """梯形速度规划的运动时间. D≥0, v>0, a>0."""
    D = abs(D)
    if D <= 0:
        return 0.0
    if v <= 0 or a <= 0:
        return float("inf")
    d_acc = v * v / a                      # 加速+减速走过的距离(达到v时)
    if D >= d_acc:                         # 能达到匀速
        return D / v + v / a
    # 三角形速度剖面 (达不到 v_max)
    return 2.0 * math.sqrt(D / a)


def scurve_time(D, v, a, j):
    """
    S曲线(jerk-limited)运动时间. D≥0, v>0, a>0, j>0.
    七段对称剖面. 采用标准分类闭式解:
      Tj = a/j (加加速时间); 达到 a_max 需要的速度增量在 Tj 内为 a²/(2j)... 采用如下判据.
    参考 trajectory planning (Biagiotti & Melchiorri) 简化对称版本.
    """
    D = abs(D)
    if D <= 0:
        return 0.0
    if v <= 0 or a <= 0:
        return float("inf")
    if j <= 0 or not math.isfinite(j):
        return trap_time(D, v, a)          # jerk=∞ → 退化为梯形

    # --- 判断是否达到最大加速度 a ---
    # 达到 a 所需的加加速时间
    Tj_a = a / j
    # 若在加速段能达到 a: 需要速度 v >= a²/j
    if v * j < a * a:
        # 达不到 a_max: 加速段为 "加加速-减加速" 两段, 无匀加速
        Tj = math.sqrt(v / j)
        Ta = 2.0 * Tj                      # 加速段总时长
        a_peak = j * Tj
    else:
        Tj = Tj_a
        Ta = Tj + v / a                    # 加速段: 加加速 Tj + 匀加速 + 减加速 Tj
        a_peak = a

    # 加速段走过的距离 (从0到v)
    # 对称: d_acc = v * Ta / 2  (面积, 速度从0线性等效到v的平均)
    d_acc = v * Ta / 2.0

    if D >= 2.0 * d_acc:
        # 能达到匀速 v: 匀速段时间
        Tv = (D - 2.0 * d_acc) / v
        return 2.0 * Ta + Tv
    else:
        # 达不到匀速 v_max → 峰值速度 v_pk < v, 需重解
        # 情形A: 仍能达到 a_max
        # 距离与峰值速度关系: 若达 a_max, D = v_pk*(v_pk/a + a/j)
        # 解 v_pk: 若 v_pk >= a²/j (达 a_max)
        # 试 a_max 分支
        # D = v_pk²/a + v_pk*a/j  → v_pk²/a + v_pk*(a/j) - D = 0
        A = 1.0 / a
        B = a / j
        C = -D
        disc = B * B - 4 * A * C
        v_pk = (-B + math.sqrt(disc)) / (2 * A) if disc >= 0 else -1
        if v_pk >= a * a / j and v_pk > 0:
            Tj = a / j
            Ta = Tj + v_pk / a
            return 2.0 * Ta
        else:
            # 情形B: 达不到 a_max, 全程加加速-减加速 (无匀加速, 无匀速)
            # D = 2 * (j * Tj³)  ... 半程 d = j*Tj³ ; 由 v_pk=j*Tj², d_half=v_pk*Tj
            # D = 2*v_pk*Tj = 2*j*Tj³ → Tj = (D/(2j))^(1/3)
            Tj = (D / (2.0 * j)) ** (1.0 / 3.0)
            return 4.0 * Tj


def move_time(D, v, a, j=None, profile="scurve"):
    """统一入口. profile ∈ {trapezoidal, scurve}. j=None 或 profile=trapezoidal → 梯形."""
    if profile == "trapezoidal" or j is None:
        return trap_time(D, v, a)
    return scurve_time(D, v, a, j)


def move_time_xy(dx, dy, v, a, j=None, profile="scurve", coupled=True):
    """
    平面 XY 移动. coupled=True: 按欧氏距离(两轴协调插补, 同时到达);
    coupled=False: 两轴独立, 取较慢者 (L∞, 各自梯形/ S曲线)."""
    if coupled:
        D = math.hypot(dx, dy)
        return move_time(D, v, a, j, profile)
    tx = move_time(abs(dx), v, a, j, profile)
    ty = move_time(abs(dy), v, a, j, profile)
    return max(tx, ty)


def sample_profile(D, v, a, j, profile="scurve", n=120):
    """
    生成运动剖面采样 (t, pos, vel, acc) 用于可视化.
    数值积分 jerk-limited 剖面; 梯形时 j 视为大值近似.
    返回 dict of lists.
    """
    D = abs(D)
    T = move_time(D, v, a, j, profile)
    if T <= 0:
        return {"t": [0], "pos": [0], "vel": [0], "acc": [0], "T": 0}
    jj = j if (profile == "scurve" and j and math.isfinite(j)) else v * 1e6
    dt = T / n
    # 用 jerk 分段积分: 通过 "期望速度剖面" 反推较复杂; 这里用简化的
    # 数值方法: 以有限 jerk 生成对称剖面, 逼近目标位移.
    # 采用速度前馈: 先加速到 vpk, 再匀速, 再减速; vpk 由 move_time 反解一致性保证近似.
    ts, ps, vs, as_ = [], [], [], []
    # 通过采样 S 曲线的解析近似: 用平滑 minimum-jerk 归一化曲线缩放
    # minimum jerk: pos(τ)=D*(10τ³-15τ⁴+6τ⁵), τ=t/T (平滑, 端点 v=a=0)
    for i in range(n + 1):
        t = i * dt
        tau = t / T
        pos = D * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
        vel = D / T * (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4)
        acc = D / T ** 2 * (60 * tau - 180 * tau ** 2 + 120 * tau ** 3)
        ts.append(t); ps.append(pos); vs.append(vel); as_.append(acc)
    return {"t": ts, "pos": ps, "vel": vs, "acc": as_, "T": T,
            "v_peak": max(vs), "a_peak": max(abs(x) for x in as_)}


if __name__ == "__main__":
    # 自检: 各剖面时间的合理性与单调性
    print("=== 梯形 vs S曲线 运动时间 ===")
    for D in (1, 10, 50, 200):
        tt = trap_time(D, 200, 2000)
        ts = scurve_time(D, 200, 2000, 20000)
        print(f"D={D:4d}mm  梯形={tt*1000:6.1f}ms  S曲线={ts*1000:6.1f}ms  "
              f"(S≥梯形: {ts >= tt - 1e-9})")
    # 长行程: S曲线应趋近梯形 (jerk影响变小)
    print("\n=== 长行程 S曲线→梯形 收敛 ===")
    for j in (5000, 20000, 100000, 1e9):
        ts = scurve_time(200, 200, 2000, j)
        tt = trap_time(200, 200, 2000)
        print(f"j={j:>10.0f}  S={ts*1000:.1f}ms  梯形={tt*1000:.1f}ms  差={(ts-tt)*1000:.2f}ms")
    # 单调性: 距离越大时间越长
    prev = 0
    ok = True
    for D in range(1, 300, 7):
        t = scurve_time(D, 200, 2000, 20000)
        if t < prev - 1e-9:
            ok = False
        prev = t
    print(f"\n单调性(距离↑→时间↑): {ok}")
    print(f"零行程: {scurve_time(0, 200, 2000, 20000)}")
