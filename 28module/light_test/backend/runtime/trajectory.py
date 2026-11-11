#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""trajectory.py — 点位运动轨迹规划（S 曲线，jerk 限幅）。

算法改写自 ../812PPT汇报/backend/trajectory.py（同一套数值生成器：
速度域拼接加速段+巡航段+镜像减速段，再梯形积分成位置序列），
默认采样 dt 改为 1ms（物理引擎 tick）。
"""

TRAPEZOID_JERK_FACTOR = 1e9


def _accel_vel(vc, amax, jerk, dt):
    """S 曲线加速段速度采样：v 从 0 到 vc，末态 a=0。"""
    vel = [0.0]
    v = a = 0.0
    for _ in range(2000000):
        if v + a * a / (2 * jerk) < vc - 1e-12:
            a = min(amax, a + jerk * dt)
        else:
            a = max(0.0, a - jerk * dt)
        v = min(vc, v + a * dt)
        vel.append(v)
        if a <= 1e-12 and v >= vc - 1e-9:
            break
    return vel


def _integrate(vel, dt):
    pts = [0.0]
    for i in range(1, len(vel)):
        pts.append(pts[-1] + (vel[i - 1] + vel[i]) * 0.5 * dt)
    return pts


def plan_move(dist, vmax, amax, jerk, dt=0.001):
    """生成 0 → dist 的位置采样列表（含起点 0 与终点 dist）。"""
    dist = float(dist)
    vmax, amax, jerk, dt = (float(abs(vmax)), float(abs(amax)),
                            float(abs(jerk)), float(dt))
    if vmax <= 0 or amax <= 0 or jerk <= 0:
        raise ValueError("vmax/amax/jerk 必须为正数")
    D = abs(dist)
    if D < 1e-9:
        return [0.0, float(dist)]
    sign = 1.0 if dist > 0 else -1.0

    def accel_dist(vc):
        return _integrate(_accel_vel(vc, amax, jerk, dt), dt)[-1]

    hi = vmax
    if 2 * accel_dist(hi) <= D:
        vc = hi
    else:
        lo = 0.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if 2 * accel_dist(mid) > D:
                hi = mid
            else:
                lo = mid
        vc = 0.5 * (lo + hi)
    if vc < 1e-9:
        return [0.0, float(dist)]

    acc = _accel_vel(vc, amax, jerk, dt)
    d_acc = _integrate(acc, dt)[-1]
    cruise = D - 2 * d_acc
    n_cruise = max(0, int(round(cruise / (vc * dt)))) if cruise > 1e-9 else 0

    vel = acc + [vc] * n_cruise + acc[::-1]
    pts = _integrate(vel, dt)
    if pts[-1] > 0:
        scale = D / pts[-1]
        pts = [p * scale for p in pts]
    pts[-1] = D
    return [sign * p for p in pts]


def profile_duration(points, dt=0.001):
    return (len(points) - 1) * dt


def plan_trapezoid(dist, vmax, amax, dt=0.001):
    """梯型速度曲线（等价于 jerk 无限大的 S 型）。"""
    return plan_move(dist, vmax, amax, amax * TRAPEZOID_JERK_FACTOR, dt)
