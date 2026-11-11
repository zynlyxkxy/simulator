# -*- coding: utf-8 -*-
"""迭代 56 冒烟（离线引擎，热积分器直推）：默认态零变化 + 热态面高/接触位漂移。"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.abspath(__file__))
RUNTIME = os.path.abspath(os.path.join(ROOT, "..", "backend", "runtime"))
sys.path.insert(0, RUNTIME)

import vision_model as vm  # noqa: E402
from points import Points  # noqa: E402
from engine import Engine  # noqa: E402
from error_system import ErrorSystem  # noqa: E402
from error_kinematics import ErrorKinematics  # noqa: E402
from recipe import Recipe  # noqa: E402
from workpieces import Workpieces  # noqa: E402
from machine_constants import MachineConstants  # noqa: E402
from materials import ComponentCatalog, Materials  # noqa: E402
from thermal import ThermalSystem  # noqa: E402
from confocal import ConfocalModel  # noqa: E402

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
_cams = vm.load_cameras(constants=eng.constants)
eng.vision = types.SimpleNamespace(
    cams={k: types.SimpleNamespace(model=c) for k, c in _cams.items()})
cf = ConfocalModel(eng)
eng.workpieces.load_wafer(offset=[0.0, 0.0], rz=0.0)

div = th._net["div"]
dt_th = th._net["dt"]


def step(seconds):
    for _ in range(int(seconds / dt_th)):
        th._tick_i = div - 1
        th.tick(0.001, eng)


def refresh():
    with eng.lock:
        eng._refresh_loop_consts_locked()


# ---- 1) 默认态（热 OFF）：读数/接触位应与修复前逐位一致 ----
refresh()
r0 = cf.read("down")
c0 = eng._bond["eff_contact_mm"]
print("[热 OFF] down read=%.4f z_s=%.6f z_f=%.6f eff_contact=%.6f"
      % (r0["reading_um"] or -1, r0["z_surface"], r0["z_focus"], c0))
assert abs(r0["z_surface"] - 885.9) < 1e-9, "默认面高变了！"
assert abs(eng._bond.get("therm_dz_chuck_mm", 0.0)) < 1e-12

# ---- 2) 热 ON + 吸盘 110/头 300 稳态 ----
th.update({"env": {"wave": "off"}})
th.set_enabled(True)
eng.heaters["Heat_Dw"].cur = 110.0
eng.heaters["Heat_Dw"].active = False
eng.heaters["Heat_Dw"].power = 0.0
eng.heaters["PulseHeat_Up"].cur = 300.0
eng.heaters["PulseHeat_Up"].active = False
eng.heaters["PulseHeat_Up"].power = 0.0
prev = None
for chunk in range(30):
    step(3600)
    cur = th._T["wafer_chuck"]
    if prev is not None and abs(cur - prev) < 1e-7:
        break
    prev = cur
print("稳态：T_chuck=%.3f T_head=%.3f" % (th._T["wafer_chuck"],
                                        th._T["bond_head"]))
refresh()
r1 = cf.read("down")
c1 = eng._bond["eff_contact_mm"]
t_ws, _ = th.module_offset("wafer_stage")
t_bh, _ = th.module_offset("bond_head")
print("[热 ON ] down read=%.4f z_s=%.6f (Δ%+.1fµm) eff_contact=%.6f (Δ%+.1fµm)"
      % (r1["reading_um"] or -1, r1["z_surface"],
         (r1["z_surface"] - r0["z_surface"]) * 1000.0, c1, (c1 - c0) * 1000.0))
print("         deform wafer_stage dz=%+.1fµm bond_head dz=%+.1fµm"
      % (t_ws[2] * 1e6, t_bh[2] * 1e6))
dz_um = (r1["z_surface"] - r0["z_surface"]) * 1000.0
assert 40.0 < dz_um < 80.0, "吸盘 Z 胀量级不对: %.1fµm" % dz_um
exp_contact = t_ws[2] - t_bh[2]
assert abs((c1 - c0) - exp_contact) < 1e-6, \
    "接触位热修正符号不对: Δc=%.7f exp=%.7f" % (c1 - c0, exp_contact)

# ---- 3) 上共焦打裸吸盘（头 300℃）：z_surface 下移 ~1mm ----
ru0 = None
with eng.lock:
    eng.axes["BS_Arm_Z"].pos = -119.83
    eng.axes["AS_X"].pos = 0.0
    eng.axes["AS_Y"].pos = 6.0
    eng.axes["AS_Z"].pos = -1.0
th2_off = None
# 同热态直接读（头已在 300 稳态）
ru1 = cf.read("up")
print("[热 ON ] up   read=%s z_s=%.6f z_f=%.6f（头 300℃ 裸吸盘面）"
      % (ru1["reading_um"], ru1["z_surface"], ru1["z_focus"]))
assert ru1["z_surface"] < 933.9 - 0.8, "头热伸长未到共焦: %.4f" % ru1["z_surface"]
print("SMOKE PASS")
