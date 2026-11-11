# -*- coding: utf-8 -*-
"""热仿真实验：键合头 300℃ 压到对准台相机上方，相机节点受多大影响？
对照：头远离（BS_Arm_Z=-60）vs 头在相机上（-119.83=上焦面位）。
方法：离线引擎，热积分器直接步进（不等实时），wave 关、PulseHeat_Up
锚定 300℃（power=0 纯锚定=温控稳态等效）。"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
RUNTIME = os.path.abspath(os.path.join(ROOT, "..", "backend", "runtime"))
sys.path.insert(0, RUNTIME)

from points import Points  # noqa: E402
from engine import Engine  # noqa: E402
from error_system import ErrorSystem  # noqa: E402
from error_kinematics import ErrorKinematics  # noqa: E402
from recipe import Recipe  # noqa: E402
from workpieces import Workpieces  # noqa: E402
from machine_constants import MachineConstants  # noqa: E402
from materials import ComponentCatalog, Materials  # noqa: E402
from thermal import ThermalSystem  # noqa: E402

eng = Engine(Points(os.path.join(RUNTIME, "io_points.json")))
eng.errors = ErrorSystem()
eng.recipe = Recipe(os.path.join(RUNTIME, "recipe.json"))
eng.workpieces = Workpieces(os.path.join(RUNTIME, "transfers.json"),
                            recipe=eng.recipe)
eng.kinematics = ErrorKinematics(eng.errors,
                                 os.path.join(RUNTIME, "error_kinematics.json"))
eng.constants = MachineConstants()
eng.components = ComponentCatalog()
eng.materials = Materials(eng.components)
th = ThermalSystem(catalog=eng.components, materials=eng.materials,
                   module_names=list(
                       eng.kinematics.config_public()["modules"]))
eng.thermal = th
eng.errors.thermal = th
eng.kinematics.thermal = th

div = th._net["div"]
dt_th = th._net["dt"]
print("thermal net: div=%d dt=%.3fs nodes=%d links=%d"
      % (div, dt_th, len(th._net["nodes"]), len(th._net["links"])))

# 环境波动关掉（对照实验要干净），开关打开
th.update({"env": {"wave": "off"}})
th.set_enabled(True)

WATCH = ["bond_head", "bond_arm", "bond_beam_b", "bond_beam_a",
         "granite_up", "align_stage", "align_optics", "wafer_chuck",
         "coarse_cam"]


def step(seconds):
    """推进 seconds 仿真秒（每次调用 tick 积分一个 dt_th）。"""
    n = int(seconds / dt_th)
    for _ in range(n):
        th._tick_i = div - 1          # 让下一拍必中 div 分频
        th.tick(0.001, eng)


def snap(tag):
    Ts = {k: th._T[k] for k in WATCH}
    t_as, r_as = th.module_offset("align_stage")
    t_bh, r_bh = th.module_offset("bond_head")
    print("[%s]" % tag)
    for k in WATCH:
        print("   %-14s T=%9.4f ℃  (ΔT=%+.5f)"
              % (k, Ts[k], Ts[k] - 25.0))
    print("   align_stage 热变形 t=[%+.3f,%+.3f,%+.3f] nm  r=[%+.2f,%+.2f,%+.2f] µrad"
          % (t_as[0] * 1e6, t_as[1] * 1e6, t_as[2] * 1e6,
             r_as[0] * 1e6, r_as[1] * 1e6, r_as[2] * 1e6))
    print("   bond_head   热变形 t=[%+.3f,%+.3f,%+.3f] nm  r=[%+.2f,%+.2f,%+.2f] µrad"
          % (t_bh[0] * 1e6, t_bh[1] * 1e6, t_bh[2] * 1e6,
             r_bh[0] * 1e6, r_bh[1] * 1e6, r_bh[2] * 1e6))
    return Ts, t_as, t_bh


snap("冷态基线（全机 25℃）")

# 键合头 300℃（纯锚定=温控稳态），头停在远离位
h = eng.heaters["PulseHeat_Up"]
h.cur = 300.0
h.target = 300.0
h.active = False          # power 注入=0，纯锚定=稳态温控等效
h.power = 0.0
with eng.lock:
    eng.axes["BS_Arm_Z"].pos = -60.0       # 头抬起远离相机

# 分段推进到稳态（align_optics 链 τ 很长）
prev = None
for chunk in range(40):                     # 每段 7200s，最多 ~3.3 天
    step(7200)
    cur = th._T["align_optics"]
    if chunk % 4 == 0 or (prev is not None and abs(cur - prev) < 1e-6):
        print("   ...t=%8ds  align_optics=%.6f ℃  bond_head=%.3f ℃"
              % ((chunk + 1) * 7200, cur, th._T["bond_head"]))
    if prev is not None and abs(cur - prev) < 1e-6:
        print("   稳态到达 @%ds" % ((chunk + 1) * 7200))
        break
    prev = cur

far = snap("头 300℃ 稳态（头远离 BS_Arm_Z=-60）")

# —— 双分支对照（严格证明 BS_Arm_Z 位置是否进热链）——
# 同一状态检查点分叉：分支 B 压头 +2h；分支 A 原位 +2h；同仿真时刻对比
T_ck = dict(th._T)
lp_ck = dict(th._motor_lp)
vel_ck = dict(th._last_vel)
walk_ck = th._walk
tick_ck = th._tick_i

with eng.lock:
    eng.axes["BS_Arm_Z"].pos = -119.83
step(7200)                                  # 分支 B：压头 +2h
near = snap("头 300℃ 压到相机上方（BS_Arm_Z=-119.83，+2h）")
near_full = dict(th._T)

th._T.update(T_ck)                          # 还原检查点
th._motor_lp.update(lp_ck)
th._last_vel.update(vel_ck)
th._walk = walk_ck
th._tick_i = tick_ck
with eng.lock:
    eng.axes["BS_Arm_Z"].pos = -60.0
step(7200)                                  # 分支 A：原位 +2h
far2 = snap("分支 A 对照（头保持 -60，+2h）")
far2_full = dict(th._T)

print("\n=====  proximity 严格差分（同仿真时刻：压头 − 原位） =====")
for k in WATCH:
    d = near[0][k] - far2[0][k]
    print("   %-14s ΔT = %+.10f ℃" % (k, d))
print("   全部 21 节点最大 |ΔT| = %.3e ℃"
      % max(abs(near_full[k] - far2_full[k]) for k in th._T))
print("   align_stage 变形差 Δt = [%+.4f,%+.4f,%+.4f] nm"
      % tuple((near[1][i] - far2[1][i]) * 1e6 for i in range(3)))
print("   bond_head  变形差 Δt = [%+.4f,%+.4f,%+.4f] nm"
      % tuple((near[2][i] - far2[2][i]) * 1e6 for i in range(3)))

# 对照：真实物理辐射换热量级估算（Stefan-Boltzmann，供结论对照）
import math  # noqa: E402
sigma = 5.67e-8
Th, Tc = 573.0, 298.0
for A_cm2, F in ((20.0, 0.3), (20.0, 0.1), (5.0, 0.3)):
    A = A_cm2 * 1e-4
    P = 0.8 * sigma * A * F * (Th ** 4 - Tc ** 4)
    print("辐射估算：A=%.0fcm² F=%.1f → P≈%.2f W（真机相机支架会吃到这个量级）"
          % (A_cm2, F, P))
