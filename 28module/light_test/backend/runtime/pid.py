# -*- coding: utf-8 -*-
"""pid.py — TwinCAT 标准型（系列型）位置式 PID（迭代 8，热压键合温控/压控）。

连续形式（TwinCAT PID 标准功能块）：
    G(s) = Kp · (1 + 1/(Tn·s) + Tv·s/(1 + Td·s))

离散位置式实现（周期 cycle_s，默认 4ms 对齐 PLC 扫描）：
- 积分：位式累加  I += e·T/Tn（Tn=0 → 纯 PD，积分通道关闭）；
- 抗饱和：输出限幅联动积分钳位（条件积分法——输出触限且误差仍把输出
  往限内推时冻结积分累加，防 windup 超调尾巴）；
- 微分：Tv·s/(1+Td·s) 一阶滤波的差分实现
    D_k = Td/(Td+T)·D_{k-1} + Tv/(Td+T)·(e_k − e_{k-1})
  （Td=0 退化为 Tv/T·Δe 的硬差分，仍有限）；
- 位置式：输出 u 为绝对执行量（温度回路=功率 %，力回路=压入量 mm），
  不是增量。

每回路参数：kp / tn_s / tv_s / td_s / out_min / out_max / cycle_s。
纯标准库、无状态外依赖；engine tick 内按 4ms 周期调用（ZOH 保持）。
"""
import math


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class PID(object):
    """系列型位置式 PID 单回路。"""

    __slots__ = ("name", "kp", "tn_s", "tv_s", "td_s",
                 "out_min", "out_max", "cycle_s",
                 "_integ", "_e_prev", "_d", "out", "initialized")

    def __init__(self, name, kp, tn_s=0.0, tv_s=0.0, td_s=0.0,
                 out_min=0.0, out_max=100.0, cycle_s=0.004):
        self.name = name
        self.kp = float(kp)
        self.tn_s = max(0.0, float(tn_s))
        self.tv_s = max(0.0, float(tv_s))
        self.td_s = max(0.0, float(td_s))
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.cycle_s = float(cycle_s)
        self._integ = 0.0
        self._e_prev = 0.0
        self._d = 0.0
        self.out = 0.0
        self.initialized = False

    def set_params(self, kp=None, tn_s=None, tv_s=None, td_s=None,
                   out_min=None, out_max=None, cycle_s=None):
        """热改参数（机器常数 pid 组 PUT 后 engine 侧调本方法）。"""
        if kp is not None:
            self.kp = float(kp)
        if tn_s is not None:
            self.tn_s = max(0.0, float(tn_s))
        if tv_s is not None:
            self.tv_s = max(0.0, float(tv_s))
        if td_s is not None:
            self.td_s = max(0.0, float(td_s))
        if out_min is not None:
            self.out_min = float(out_min)
        if out_max is not None:
            self.out_max = float(out_max)
        if cycle_s is not None:
            self.cycle_s = float(cycle_s)

    def reset(self, out0=0.0, sp=0.0, pv=0.0):
        """无扰切换初始化：积分预置 out0/kp，使输出 u=out0+kp·e——
        e=0 时输出连续（真无扰），e≠0 时 P 通路立即参与（快速趋近，
        不拿积分慢爬当"无扰"）。"""
        e = float(sp) - float(pv)
        self._e_prev = e
        self._d = 0.0
        if self.kp != 0.0:
            self._integ = float(out0) / self.kp
        else:
            self._integ = 0.0
        self.out = float(out0)
        self.initialized = True

    def update(self, sp, pv):
        """推进一个周期 -> 位置式输出（限幅后）。sp/pv 同物理单位。"""
        T = self.cycle_s
        e = float(sp) - float(pv)
        if not self.initialized:
            self._e_prev = e
            self.initialized = True
        # 微分通道（Tv·s/(1+Td·s) 差分化）
        if self.tv_s > 0.0:
            den = self.td_s + T
            self._d = (self.td_s / den) * self._d \
                + (self.tv_s / den) * (e - self._e_prev)
        else:
            self._d = 0.0
        # 积分通道（条件积分抗饱和：先试算，触限且误差同向则冻结）
        integ_new = self._integ + (e * T / self.tn_s if self.tn_s > 0.0
                                   else 0.0)
        u = self.kp * (e + integ_new + self._d)
        u_c = _clamp(u, self.out_min, self.out_max)
        if u_c != u and ((u_c >= self.out_max and e > 0.0)
                         or (u_c <= self.out_min and e < 0.0)):
            # 输出触限且误差仍往限外推：冻结积分，重算输出
            u = self.kp * (e + self._integ + self._d)
            u_c = _clamp(u, self.out_min, self.out_max)
        else:
            self._integ = integ_new
        # 数值安全（参数热改成 NaN/Inf 不炸 tick，输出回中位）
        if not math.isfinite(u_c):
            u_c = 0.5 * (self.out_min + self.out_max)
            self._integ = 0.0
            self._d = 0.0
        self._e_prev = e
        self.out = u_c
        return u_c

    def state(self):
        """调试用内部状态（单测断言抗饱和/滤波用）。"""
        return {"integ": self._integ, "e_prev": self._e_prev,
                "d": self._d, "out": self.out}
