# -*- coding: utf-8 -*-
"""迭代 54b：上吸盘新纹路（中心真空主孔+宽臂十字凹槽）REAL 预览 + 模板匹配验证。
up 相机随对准台：AS_X 偏移 = 视吸盘不同半径；AS_Y=6 对准吸盘心。"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
BACKEND = os.path.abspath(os.path.join(ROOT, "..", "backend"))
RUNTIME = os.path.join(BACKEND, "runtime")
sys.path.insert(0, RUNTIME)

import vision_model as vm  # noqa: E402
from points import Points  # noqa: E402
from engine import Engine  # noqa: E402
from error_system import ErrorSystem  # noqa: E402
from error_kinematics import ErrorKinematics  # noqa: E402
from recipe import Recipe  # noqa: E402
from workpieces import Workpieces  # noqa: E402
from machine_constants import MachineConstants  # noqa: E402
import vision_service as vs  # noqa: E402

eng = Engine(Points(os.path.join(RUNTIME, "io_points.json")))
eng.errors = ErrorSystem()
eng.recipe = Recipe(os.path.join(RUNTIME, "recipe.json"))
eng.workpieces = Workpieces(os.path.join(RUNTIME, "transfers.json"),
                            recipe=eng.recipe)
eng.kinematics = ErrorKinematics(eng.errors,
                                 os.path.join(RUNTIME, "error_kinematics.json"))
eng.constants = MachineConstants()
cams = vm.load_cameras(constants=eng.constants)
up = cams["up"]

VIEWS = [("center", 0.0), ("holeedge", 1.2), ("xarm", 3.7), ("ring6", 6.0),
         ("ring16", 16.0)]
frames = {}
for tag, r in VIEWS:
    with eng.lock:
        for n, v in (("AS_X", r), ("AS_Y", 6.0), ("AS_Z", -1.0),
                     ("BS_Arm_Z", -119.83)):
            eng.axes[n].pos = v
    positions, poses = vm._engine_poses(eng)
    cam_pos, _ = vm.camera_pose(eng, up, poses, positions)
    marks = vm.collect_marks(eng, up, positions, poses)
    vm.apply_focus(up, cam_pos[2], marks)
    layers = vm.collect_layers(eng, up, positions, poses, cam_pos)
    img = vm.render_frame(eng, up, light_on=True, marks=marks,
                          cam_pos=cam_pos, layers=layers,
                          positions=positions, poses=poses)
    out = os.path.join(ROOT, "_up_v3_%s.png" % tag)
    with open(out, "wb") as f:
        f.write(vm.png_encode_gray(img))
    frames[tag] = (img, marks, cam_pos)
    print("saved", out, "std=%.2f" % float(img.std()),
          "marks=", [m.name for m in marks])

# 模板匹配验证：中心帧上 head_chuck_cross（150µm）应命中 tpl_head_chuck
with open(os.path.join(RUNTIME, "vision_templates.json"),
          encoding="utf-8") as f:
    tpl_meta = json.load(f)["templates"]["tpl_head_chuck"]
img, marks, cam_pos = frames["center"]
entries, ms = vs.Matcher().match(img, [tpl_meta], 0.6)
print("match tpl_head_chuck: entries =", len(entries), " ms=%.0f" % ms)
ok = False
for e in entries[:3]:
    print("   col=%.1f row=%.1f score=%.3f judge=%s"
          % (e["col"], e["row"], e["score"], e.get("judge")))
    m0 = marks[0]
    exp_col, exp_row = up.project(m0.x - cam_pos[0], m0.y - cam_pos[1])
    if abs(e["col"] - exp_col) < 3.0 and abs(e["row"] - exp_row) < 3.0 \
            and e["score"] >= 0.6:
        ok = True
print("REAL 链路：", "PASS（孔底标记命中且峰位对账 ±3px）" if ok else "FAIL")
sys.exit(0 if ok else 1)
