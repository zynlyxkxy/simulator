# -*- coding: utf-8 -*-
"""calibration.py — 测校系统后端：三个标定程序 + 两个重复性测校程序，
全部走真实视觉链路（REAL 模式 acquire+identify），标定测量结果经
machine_constants.py 校验写回（重复性测校为纯测量程序，无写回）。

作业模式（无现成异步框架，简单做）：POST 启动 → job_id，工作线程执行，
GET status 轮询，完成后 GET result；标定期间拒绝其它标定并发（409）。
POST cancel 置协作式取消标志（各测校循环边界检查点生效 → canceled
终态，finally 现场恢复照跑）。

程序（选轴按各相机 marks.source 的载体模块）：
- focus_scan 焦面标定：让"相机↔标记"相对 Z 自由度扫描（up→BS_Arm_Z 扫
  键合头携带的 die；down→AS_Z 扫对准台携带的镜头；coarse 镜头与蓝膜无相对
  Z 自由度 → unsupported），每点 acquire+identify 记 score，抛物线拟合峰值
  → 实测焦面距离 wd_mm。
- pixel_equivalent 像素当量标定：点动已知 Δmm（默认 1.5），测像素位移
  → k_mm_px=Δ/Δpx → mag=pixel_um/k_um_px。
- rotation_direction 旋转与方向标定：分别沿 X、Y 点动，测像素位移向量，
  枚举 (dir_x,dir_y,swap)×连续 rot 拟合最优组合 → rot_mrad/dir_x/dir_y。
- align_repeat_static / align_repeat_dynamic 相机对准重复性（静态连拍 /
  载体轴往复+拍照；实现在 metrology/repeatability.py，结果含散布统计
  + CSV 落盘 runtime/metrology_data/，不写回机器常数）。
  迭代 25 对齐参考框架 HoriMetrologyScirptsFramework：params 支持
  template_name（迭代 34 起**必填**——用视觉库已有模板，不自建/不删；
  空/None→400 列可用名）、动态程序支持绝对坐标
  start_xy/photo_xy（[x,y] mm，多轴时 X→第 1 轴、Y→第 2 轴；coarse 单轴
  只用 x）；travel_mm 保留为兼容别名（photo_xy=各轴当前位、start_xy=
  photo−travel）。作业带操作日志流（job["log"]，环形 200 行，status 带出）。
- camera_microrot 相机微小旋转角标定（迭代 31，执行链迭代 32 迁入
  metrology/extra_routines.py——走位轴改对准台 MICROROT_AXES：
  down→AS_X/AS_Y 动相机=等效动标记 rel=−1、up→AS_X/Y、coarse→FA_X；
  迭代 34 起 down 可选 move_stage="wafer" 改走 WS_LS_X/Y rel=+1）：
  纯 X 平移两点法 φ=−atan2(Δrow,Δcol)，repeat 轮取均值 → rot_mrad 写回。
  template_name/photo_pos1/photo_pos2 迭代 34 起必填（无自建/无缺省
  路径），纯 X 容差放宽到 0.01mm，X 间距上下限与模板可达窗口校验移除。
- ws_rotate_repeat 示教模式迭代 32 改**直接测角法**、迭代 35 改**双示教
  位**（用户口径）：基线逐点走 pose 测世界坐标 P_k → 每循环 WS_Rz 走
  rz0+rot_rad → 逐点走 **pose_after**（用户示教旋后可见位，缺省=pose
  同位复测；预测走位 Δ 段与 start 预估 400 已删，下发前保留软限位
  守卫）→ 实测 Q_k → _fit_rot 闭式拟合实际旋转角 θ_i（u=P−P̄、v=Q−Q̄
  → θ=atan2(Σu×v,Σu·v)，extra_routines 助手，rotate_center/gain 复用）
  → 角度误差 e_i=θ_i−rot_rad（stats.angle_err mean/std/s3 µrad，
  3σ=旋转角重复性；旧字段 stats.dtheta/residual、series.dtheta_urad/
  points[i].d_* 兼容保留；detail.taught_points 逐点带 pose_after）。首选
  参数 rot_rad（rad，(0,0.6]；rot_deg 兼容换算走同一校验）+
  taught_points 2~10 示教点（旧四键映射 2 点）。
- ws_rotate_center 硅片台旋转中心标定（迭代 32，metrology/
  rotate_center.py；迭代 35 双示教位+_fit_rot）：角度档
  ±step_rad×1..n_angles 对称，每 (循环,档) 基线 P_k（走 pose）↔ 旋后
  Q_kj（走 pose_after）→ _fit_rot 闭式 (θ_ij,C_ij)（C=(I−R(θ))⁻¹
  (Q̄−R(θ)P̄) 2×2 直接解，小 θ 病态沿用既有 1/θ 放大防护）；中心=分量
  均值、3σ=hypot(分量散布)；measured.rz_center_offset_mm（拟合中心−
  WS_Rz 机械轴心（运动链名义不动点），与真值注入 sim_align.
  rz_center_true_offset_um 同口径）+ **rz_center_ref_xy_mm=[作业起点
  WS_LS_X,WS_LS_Y]**（标定时机台位）；apply 双写回两键，经组路由写回
  fine_align（校验链 FINE_ALIGN_FIELDS "xy" 原生支持）。
- ws_rotate_gain 硅片台旋转角度增益标定（迭代 32A，metrology/
  rotate_gain.py；迭代 35 双示教位+_fit_rot）：每 (循环,档) _fit_rot
  实测旋转角 θ_ij 对指令 θ_cmd_j 过原点 LS gain=Σθaθc/Σθc²
  （per-repeat，带截距诊断 offset_urad 保留）；measured.rz_gain，apply
  fields=["rz_gain"] 写回 fine_align 组（"num" 校验原生支持）。
- as/ws_stage_nonortho 运动台 Y 导轨非正交角测校（迭代 36 落地、迭代 37
  按用户纠正重写为**栅格跳测法**，metrology/stage_nonortho.py）：基准尺=
  晶圆 die 阵列（wafer_sites 格点+site_marks quad 假定正交），走距=格点
  节距（start 与处方 pitch 校验一致 0.001mm，仿真标记只在格点上）；
  从起点沿走位轴单轴每次走 pitch 共 K 点、每点 identify 取距画幅中心
  最近有效峰（fpp 帧均值），像素→世界偏移 o_i（_context 链），o_i 对步
  号 LS 斜率：X 线斜率方向读栅格安装旋转 ρ̂、Y 线斜率大小给 sin(φ_t+ρ̂)，
  φ(X 基准)=wrap(ψ_y−ρ̂)（ρ̂ 差中抵消）；ref_axis=x 报 φ / =y 报 −φ；
  cam 恒 down（ws=硅片台走位 σ=+1 / as=对准台走位 σ=−1）；measured 含
  phi_urad（ref_axis 口径）/<pfx>nonortho_urad（恒 X 基准值，apply 写回
  stage_geo 组，不动 *_y_rail_dir 真值旋钮）/phi_std_urad/phi_3sigma_urad/
  theta_x/y_urad/resid_rms_nm/resid_max_nm/series_phi_urad；FK 消费在
  error_kinematics（stage_geo→Y 链元平移偏差，链关不生效，φ=0 恒等）。
- as_head_center 上吸盘位置标定（迭代 42，metrology/as_head_center.py，
  cam 恒 up）：动态再逼近直接测量——前置走位（AS_X/Y→wafer_ega.
  as_anchor_xy_mm、AS_Z→as_z_chuck_focus_mm、BS_Arm_Z→head_z_upfocus_mm）
  后每轮 XY ±1mm 往返回锚点再 identify(up,tpl)，solve_pixel 转世界偏移
  δ=标记−相机 → 新锚点=当前 AS 位+δ（相机随台动，写回后复测 δ≈0 符号
  自证）；measured 双键同值联动 as_anchor_xy_mm（→wafer_ega 组）/
  as_head_center_xy_mm（→focus_level 组）（上/下相机同轴，同一物理量）。
- best_focus 相机最佳焦平面标定（迭代 42，metrology/best_focus.py，
  cam=up/down）：区间收缩法（参考 OptimalFocalCal，清晰度改 score 代
  理）——每轮 z_start→z_end 按 step 扫 z-score 曲线、argmax 后区间缩
  到峰点左右各一采样点、step 减半，末轮峰处三点抛物线拟合得亚步距
  z*；down 扫 AS_Z（measured.as_z_chuck_focus_mm）、up 先前置走位再扫
  BS_Arm_Z（measured.head_z_upfocus_mm），两键 apply 组路由→focus_level。

安全：点动全部走 engine.move（联锁/软限位/沙盒干涉检查）；标定前后恢复
轴原位、光源原位、视觉模式原位。

模板口径（迭代 34）：测校软件 9 项（align_repeat_static/dynamic、
ws/as_move_repeat、die_transfer_repeat、ws_rotate_repeat、camera_
microrot、ws_rotate_center、ws_rotate_gain）的模板全部用户 teach 必填
（template_name/coarse_tpl/up_tpl/back_tpl/示教点 tpl——start 存在性
400，作业不自建、结束不删）。**已知边界**：focus_scan/pixel_equivalent/
rotation_direction 三旧标定项无模板参数（不在测校软件 9 项内），仍走
_cal_%d 自建临时模板（结束删）。
"""
import math
import os
import sys
import threading
import time

import vision_model as vm

# metrology 包与 runtime 同级（backend/metrology）——把backend根目录
# 插入 sys.path（幂等），与 main.py 对 runtime/ 的 sys.path 处理同风格
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from metrology import repeatability as _metro_repeat   # noqa: E402
from metrology import extra_routines as _metro_extra   # noqa: E402
from metrology import microrot as _metro_microrot      # noqa: E402,F401
# ^ 迭代 32 起执行链迁 extra_routines.run_camera_microrot（走位轴改对准台
#   MICROROT_AXES）；本 import 保留：启动即加载校验 microrot 模块，且
#   extra_routines 懒导入复用其 _best_peak
from metrology import rotate_center as _metro_rc       # noqa: E402
from metrology import rotate_gain as _metro_rg         # noqa: E402
from metrology import stage_nonortho as _metro_nonortho  # noqa: E402
from metrology import as_head_center as _metro_ahc     # noqa: E402
from metrology import best_focus as _metro_bf          # noqa: E402
from metrology import bs_rxry_axis as _metro_rxry      # noqa: E402
from metrology import handoff_height_calib as _metro_hh  # noqa: E402
from metrology import bond_height_calib as _metro_bh   # noqa: E402
from metrology import temp_force_calib as _metro_tf    # noqa: E402

ROUTINES = ("focus_scan", "pixel_equivalent", "rotation_direction",
            "align_repeat_static", "align_repeat_dynamic",
            "confocal_repeat", "ws_move_repeat", "as_move_repeat",
            "die_transfer_repeat", "ws_rotate_repeat", "camera_microrot",
            "ws_rotate_center", "ws_rotate_gain",
            "as_stage_nonortho", "ws_stage_nonortho",
            "as_head_center", "best_focus",
            "bs_rxry_axis", "flip_handoff_height", "bond_height_calib",
            "bond_temp_calib", "bond_force_calib")

# focus_scan 的 Z 扫描轴（相机↔标记相对 Z 自由度）；coarse 无 → unsupported
SCAN_Z_AXIS = {"up": "BS_Arm_Z", "down": "AS_Z"}
# 点动轴：(轴名, 标记相对相机的符号)——轴正方向运动产生的"标记相对相机"
# 世界位移符号。down/coarse 动的是标记载体（+）；up 动的是相机（-）。
# coarse 的标记载体 = 翻转模块（FA_X 平移；CH_Rz 是纯旋转轴，对居中 die
# 不产生像素位移，不能当点动轴用）。
JOG_AXES = {"down": (("WS_LS_X", +1.0), ("WS_LS_Y", +1.0)),
            "up": (("AS_X", -1.0), ("AS_Y", -1.0)),
            "coarse": (("FA_X", +1.0),)}
# 运动组（2026-09-08 需求②：下相机动态可选对准台动）。键=相机，值=
# {组名: 轴名列表}；缺省组 = JOG_AXES 的轴名。align=相机自身动（标记
# 相对相机反向移动——等效于动标记载体，方向由 guide 符号语义保证）。
JOG_GROUPS = {
    "down": {"wafer": ("WS_LS_X", "WS_LS_Y"),
             "align": ("AS_X", "AS_Y")},
    "up": {"align": ("AS_X", "AS_Y")},
    "coarse": {"flip": ("FA_X",)},
}
# 标记形状标签（建模板用；die 标记源默认 box——recipe.film.marks.shape）
SOURCE_SHAPE = {"wafer_marks": "cross", "film_die_marks": "box",
                "head_die_marks": "box", "flip_die_marks": "box"}
# 新 routine 的相机参数（cam 参数语义：confocal=探头；其余=观测相机）
# as_move_repeat 迭代 34 起相机可选 up（默认）/down（下相机看对准台动，
# 须 jog_group=align——与 metrology_gui StageMoveRepeat 契约一致）；
# ws_move_repeat 恒 down。
ROUTINE_CAM = {"confocal_repeat": ("down", "up"),
               "ws_move_repeat": ("down",),
               "as_move_repeat": ("up", "down"),
               "die_transfer_repeat": ("coarse", "up"),
               "ws_rotate_repeat": ("down",),
               "camera_microrot": ("up", "down", "coarse"),
               "ws_rotate_center": ("down",),
               "ws_rotate_gain": ("down",),
               "as_stage_nonortho": ("down",),
               "ws_stage_nonortho": ("down",),
               "as_head_center": ("up",),
               "best_focus": ("up", "down"),
               # 迭代 49 五项：都不用相机做域切换（bs_rxry_axis 用 up 相机+
               # 共焦，其余四项纯轴/力/温链）——cam 恒 "up" 保持校验简单
               "bs_rxry_axis": ("up",),
               "flip_handoff_height": ("up",),
               "bond_height_calib": ("up",),
               "bond_temp_calib": ("up",),
               "bond_force_calib": ("up",)}
# camera_microrot 走位轴（迭代 32 轴名/rel 口径；迭代 34 起按
# (cam, move_stage) 查表）——down+"align"（默认）：AS_X/AS_Y rel=−1（下
# 相机挂对准台，动相机=等效动标记，标记相对相机反向移动，与 up 同款）；
# down+"wafer"：WS_LS_X/WS_LS_Y rel=+1（动标记载体口径，与 JOG_AXES
# down 组同）；up 恒 align：AS_X/Y rel=−1（传 move_stage="wafer"→400）；
# coarse 恒 FA_X rel=+1（不参与 move_stage）。与 JOG_AXES 分离：像素当量/
# 旋转方向/动态重复性仍走 JOG_AXES 标记载体口径，互不影响。
MICROROT_AXES = {"down": {"align": (("AS_X", -1.0), ("AS_Y", -1.0)),
                          "wafer": (("WS_LS_X", +1.0), ("WS_LS_Y", +1.0))},
                 "up": {"align": (("AS_X", -1.0), ("AS_Y", -1.0))},
                 "coarse": {"align": (("FA_X", +1.0),)}}
# apply 写回组路由（迭代 32；迭代 35 新增 rz_center_ref_xy_mm 双写）：
# 缺省写回 cameras[cam]；下表字段写回对应机器常数组——ws_rotate_center
# 的 rz_center_offset_mm/rz_center_ref_xy_mm→fine_align 组
# （machine_constants FINE_ALIGN_FIELDS 对该两键均为 "xy" 二维向量校验，
# 原生支持，无需改 machine_constants.py 校验分支）。
# 迭代 36：as/ws_stage_nonortho 的非正交角测校结果→stage_geo 组
# （as/ws_nonortho_urad；apply 不动 *_y_rail_dir——真值旋钮非测校结果）。
# 迭代 42：as_head_center 双键同值联动（as_anchor_xy_mm→wafer_ega /
# as_head_center_xy_mm→focus_level——上/下相机同轴同一物理量）；
# best_focus 两焦面键（as_z_chuck_focus_mm/head_z_upfocus_mm）→
# focus_level 组。
APPLY_GROUP_ROUTE = {"rz_center_offset_mm": "fine_align",
                     "rz_center_ref_xy_mm": "fine_align",
                     "rz_gain": "fine_align",
                     "as_nonortho_urad": "stage_geo",
                     "ws_nonortho_urad": "stage_geo",
                     "as_anchor_xy_mm": "wafer_ega",
                     "as_head_center_xy_mm": "focus_level",
                     "as_z_chuck_focus_mm": "focus_level",
                     "head_z_upfocus_mm": "focus_level",
                     # 迭代 49：bs_rxry_axis 旋转轴线标定 → bs_rxry_axis 组；
                     # flip_handoff_height 接触位 → flip_handoff 组；
                     # bond_height_calib 裸接触位 → bond 组；
                     # bond_temp/force_calib 传感链标定 → sensor_calib 组
                     "rx_axis_point_mm": "bs_rxry_axis",
                     "rx_axis_dir": "bs_rxry_axis",
                     "ry_axis_point_mm": "bs_rxry_axis",
                     "ry_axis_dir": "bs_rxry_axis",
                     "head_z_touch_mm": "flip_handoff",
                     "head_z_contact_mm": "bond",
                     "head_temp_gain": "sensor_calib",
                     "head_temp_offset_c": "sensor_calib",
                     "wafer_temp_gain": "sensor_calib",
                     "wafer_temp_offset_c": "sensor_calib",
                     "force_gain": "sensor_calib",
                     "force_offset_n": "sensor_calib"}
# 非正交测校走位轴（迭代 36）：routine -> (X 轴, Y 轴)；ws=硅片台动
# （下相机固定看晶圆标记）、as=对准台动（下相机随对准台看静止晶圆标记）
STAGE_NONORTHO_AXES = {"ws_stage_nonortho": ("WS_LS_X", "WS_LS_Y"),
                       "as_stage_nonortho": ("AS_X", "AS_Y")}


class CalibError(RuntimeError):
    """标定参数/前置条件错误（API 映射 400）。"""


class CalibBusy(RuntimeError):
    """已有标定作业在跑（API 映射 409）。"""


class CalibCanceled(RuntimeError):
    """协作式取消（迭代 29b）：作业循环检查点（repeatability._check_cancel）
    抛出，_run/_run_simple 捕获后置 canceled 终态（finally 现场恢复照跑）。"""


class CalibrationService(object):
    """测校服务主体。main.py 注入 engine.calibration 并随视觉一起拉起。"""

    def __init__(self, engine, points, vision):
        self.engine = engine
        self.points = points
        self.vision = vision
        self._lock = threading.RLock()
        self._jobs = {}                  # job_id -> job dict
        self._order = []                 # 创建顺序
        self._seq = 0
        self._active = None
        # 迭代 36：六自由度合成器的 stage_geo（Y 导轨方向角）常数接线——
        # 口径同 modal/thermal 的引用注入（读时热查）。main.py 装配序
        # kinematics 先于 machine_constants 构造，且 main.py 不在本迭代
        # 可改范围，此处由测校服务构造时兜底接线（getattr 守卫，幂等；
        # 未接线=φ0 恒等，见 error_kinematics._stage_geo_rad）
        try:
            kin = getattr(engine, "kinematics", None)
            const = getattr(engine, "constants", None)
            if kin is not None and const is not None \
                    and getattr(kin, "constants", None) is None:
                kin.constants = const
        except Exception:
            pass

    # ================= API 方法 =================
    def start(self, routine, cam_id, params=None):
        if routine not in ROUTINES:
            raise CalibError("routine 只能是 %s" % (ROUTINES,))
        # 新 routine 的相机/探头取值域（ROUTINE_CAM）；die_transfer 等
        # 多相机 routine 传任一合法相机即可（内部自取两台）
        allowed = ROUTINE_CAM.get(routine)
        if allowed is not None:
            if cam_id not in allowed:
                raise CalibError("routine %s 的 cam 只能是 %s：%r"
                                 % (routine, "/".join(allowed), cam_id))
        elif cam_id not in self.vision.cams:
            raise CalibError("未知相机 %r（有 %s）"
                             % (cam_id, ",".join(self.vision.cams)))
        if vm.cv2_or_none() is None:
            raise CalibError("cv2 未安装，标定走 REAL 链路不可用"
                             "（pip install opencv-python）")
        if routine == "focus_scan" and cam_id not in SCAN_Z_AXIS:
            raise CalibError(
                "coarse 相机镜头固连机架、蓝膜台无 Z 轴——镜头与蓝膜无相对 "
                "Z 自由度，focus_scan 不支持该相机（wd 需机械标定）")
        params = dict(params or {})
        if routine in ("align_repeat_static", "align_repeat_dynamic"):
            # 参数/前置条件同步校验（CalibError→400），规范化值写回 params
            params.update(self._validate_repeatability(routine, cam_id,
                                                        params))
        elif routine in ("confocal_repeat", "die_transfer_repeat",
                         "ws_rotate_repeat", "ws_move_repeat",
                         "as_move_repeat", "camera_microrot",
                         "ws_rotate_center", "ws_rotate_gain",
                         "as_stage_nonortho", "ws_stage_nonortho",
                         "as_head_center", "best_focus",
                         "bs_rxry_axis", "flip_handoff_height",
                         "bond_height_calib", "bond_temp_calib",
                         "bond_force_calib"):
            params.update(self._validate_extra(routine, cam_id, params))
        with self._lock:
            if self._active is not None:
                raise CalibBusy("标定作业 %d 正在运行，拒绝并发" % self._active)
            self._seq += 1
            jid = self._seq
            job = {"job_id": jid, "routine": routine, "camera": cam_id,
                   "params": params, "state": "running", "cancel_req": False,
                   "progress": {"i": 0, "n": 0, "note": "启动"},
                   "t_start": round(time.time(), 3), "t_end": None,
                   "error": None, "result": None, "log": []}
            self._jobs[jid] = job
            self._order.append(jid)
            self._active = jid
        threading.Thread(target=self._run, args=(job,), daemon=True,
                         name="calib-%d" % jid).start()
        return {"ok": True, "job_id": jid, "state": "running"}

    # ---- align_repeat_* 参数/前置校验（start 同步做，CalibError→400）----
    def _validate_repeatability(self, routine, cam_id, p):
        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 50, 10, 1000, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        if out["wait_ms"] == int(out["wait_ms"]):
            out["wait_ms"] = int(out["wait_ms"])
        # 视觉模板名称（迭代 34 起必填，静/动态通用）：空/None/非字符串
        # →400 列可用模板名；给了须已注册（存在性 400 同文案）——用户
        # 模板作业不自建、绝不删
        tpl_name = p.get("template_name")
        if not isinstance(tpl_name, str) or not tpl_name:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "template_name 必填（非空模板名；视觉模板库现有：%s——"
                "先 POST /api/vision/templates 建模板）"
                % ("、".join(avail) if avail else "（空）"))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        out["template_name"] = tpl_name
        if routine == "align_repeat_static":
            return out
        # 工艺在跑（流程 running/paused、全片键合、一键串联）→ 拒绝动轴
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止动态对准重复性测校"
                             % "、".join(busy))
        # 运动组（2026-09-08 需求②）：down 可选 wafer(默认)/align；
        # up/coarse 只有默认组。组内轴名即点动轴。
        groups = JOG_GROUPS.get(cam_id, {})
        group = p.get("jog_group", list(groups)[0] if groups else None)
        if group is not None and group not in groups:
            raise CalibError(
                "相机 %s 的 jog_group 只能是 %s：%r"
                % (cam_id, "/".join(groups), group))
        out["jog_group"] = group
        axes_names = groups.get(group) or tuple(a for a, _s in
                                                JOG_AXES[cam_id])
        out["jog_axes"] = list(axes_names)
        jog = tuple((a, dict(JOG_AXES[cam_id]).get(a, +1.0))
                    for a in axes_names)

        def _xy(key):
            v = p.get(key)
            if not isinstance(v, (list, tuple)) or len(v) != 2 \
                    or any(isinstance(c, bool)
                           or not isinstance(c, (int, float))
                           or not (-1e4 <= float(c) <= 1e4) for c in v):
                raise CalibError("%s 须为 [x, y] 两个数值且分量 ∈ "
                                 "[-10000, 10000]mm：%r" % (key, v))
            return [float(v[0]), float(v[1])]

        has_start = "start_xy" in p
        has_photo = "photo_xy" in p
        if has_start or has_photo:
            # 绝对坐标路径（对齐参考框架 AlignStabilityDynamic）：两点须
            # 成对给出；X 分量对第 1 根点动轴、Y 分量对第 2 根（coarse
            # 单轴只用 x，y 分量接受传入但忽略）
            if not (has_start and has_photo):
                raise CalibError("start_xy / photo_xy 须成对给出")
            start_xy, photo_xy = _xy("start_xy"), _xy("photo_xy")
            for i, (axis, _sign) in enumerate(jog):
                comp = start_xy[i], photo_xy[i]
                with self.engine.lock:
                    lo, hi = self.engine.axes[axis].lo, \
                        self.engine.axes[axis].hi
                if not (lo <= comp[0] <= hi and lo <= comp[1] <= hi):
                    raise CalibError(
                        "轴 %s 的起始位/拍照位分量 %.4f/%.4fmm 超出软限位 "
                        "[%.3f, %.3f]" % (axis, comp[0], comp[1], lo, hi))
            out["start_xy"], out["photo_xy"] = start_xy, photo_xy
            return out
        # travel_mm 兼容别名路径：photo_xy=各轴当前位、start_xy=photo−travel
        # （旧语义：每根点动轴都退 travel）
        out["travel_mm"] = _num("travel_mm", 5.0, 0.1, 20.0)
        travel = out["travel_mm"]
        photo_xy = [0.0, 0.0]
        for i, (axis, _sign) in enumerate(jog):
            with self.engine.lock:
                ax = self.engine.axes[axis]
                q0, lo, hi = ax.pos, ax.lo, ax.hi
            photo_xy[i] = q0
            if q0 - travel < lo or q0 + travel > hi:
                raise CalibError(
                    "轴 %s 拍照位 %.4f ± travel %.3fmm 超出软限位 "
                    "[%.3f, %.3f]" % (axis, q0, travel, lo, hi))
        out["photo_xy"] = photo_xy
        out["start_xy"] = [photo_xy[0] - travel,
                           photo_xy[1] - (travel if len(jog) > 1 else 0.0)]
        return out

    def log(self, job, line):
        """作业操作日志流：append 一行（环形 200 行，超出丢最旧），
        status() 带出给前端滚动显示。线程安全（测校工作线程写、HTTP 读）。"""
        with self._lock:
            job["log"].append(str(line))
            if len(job["log"]) > 200:
                del job["log"][:-200]

    # ---- 新增测校项参数/前置校验（2026-09-08，start 同步做→400）----
    def _validate_extra(self, routine, cam_id, p):
        if routine == "camera_microrot":
            # repeat 域 [3,100]（与参考 SOP 口径一致），独立校验分支
            return self._validate_microrot(cam_id, p)
        if routine in ("ws_rotate_center", "ws_rotate_gain"):
            # 旋转中心/旋转增益（迭代 32/32A）：同一参数面（taught_points
            # ≥2、step_rad/n_angles/repeat [3,100]/wait_ms/WS_Rz 行程/
            # 预测走位预估）独立校验分支
            return self._validate_rotate_sweep(p)
        if routine in ("ws_stage_nonortho", "as_stage_nonortho"):
            # Y 导轨非正交角测校（迭代 37 栅格跳测法——基准尺=晶圆 die
            # 阵列，走距=格点节距与处方校验）：扫描线参数面独立校验分支
            return self._validate_stage_nonortho(routine, cam_id, p)
        if routine == "as_head_center":
            # 上吸盘位置标定（迭代 42）：模板/repeat/流程忙独立校验分支
            return self._validate_as_head_center(cam_id, p)
        if routine == "best_focus":
            # 相机最佳焦平面标定（迭代 42）：z 窗口/step/rounds/行程/模板
            # 独立校验分支
            return self._validate_best_focus(cam_id, p)
        if routine == "bs_rxry_axis":
            # 键合头调平轴旋转轴线标定（迭代 49）：模板/角度/repeat/wait_ms
            # + head 有 die + 流程忙 独立校验分支
            return self._validate_bs_rxry_axis(p)
        if routine == "flip_handoff_height":
            # 翻转交接接触高度标定（迭代 49）：片厚/阈值/速度 + flip 有 die
            # + 流程忙 独立校验分支
            return self._validate_flip_handoff_height(p)
        if routine == "bond_height_calib":
            # 键合接触高度标定（迭代 49）：片厚/晶圆厚/阈值/速度 + 晶圆在位
            # +head 有 die + 流程忙 独立校验分支
            return self._validate_bond_height_calib(p)
        if routine == "bond_temp_calib":
            # 温度传感链标定（迭代 49）：heater/setpoints/samples/interval/
            # tol/timeout + 流程忙 独立校验分支
            return self._validate_bond_temp_calib(p)
        if routine == "bond_force_calib":
            # 力传感链标定（迭代 49）：setpoints/samples/interval/阈值 +
            # 晶圆在位+head 有 die + 流程忙 独立校验分支
            return self._validate_bond_force_calib(p)

        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 10, 3, 1000, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        if routine == "confocal_repeat":
            if self.engine.confocal is None:
                raise CalibError("confocal not attached")
            if cam_id not in ("down", "up"):
                raise CalibError("confocal_repeat 探头只能是 down/up：%r"
                                 % (cam_id,))
            return out
        if routine in ("ws_move_repeat", "as_move_repeat"):
            # 复用 align_repeat_dynamic 的校验（模板必填/存在性、坐标/
            # 软限位/流程忙/jog_group）；repeat/wait_ms 由本方法校验
            # （3~1000，比相机重复性的 10~1000 宽），剔除后不让子校验
            # 重验。as_move_repeat cam=down（下相机看对准台动）由调用方
            # 给 jog_group="align"（metrology_gui 契约），jog_group/
            # jog_axes 一并提取进 out 透传 run_dynamic（原实现丢弃）。
            fake = {k: v for k, v in p.items()
                    if k not in ("repeat", "wait_ms")}
            sub = self._validate_repeatability("align_repeat_dynamic",
                                               cam_id, fake)
            out.update({k: sub[k] for k in
                        ("start_xy", "photo_xy", "jog_group", "jog_axes",
                         "template_name") if k in sub})
            return out
        if routine == "die_transfer_repeat":
            if self.engine.workpieces is None \
                    or not self.engine.workpieces.film["present"]:
                raise CalibError("蓝膜不在位（先上膜：POST /api/workpiece/film）")
            # 三模板存在性校验上移到 start（迭代 34：运行期只保留兜底）——
            # coarse_tpl/up_tpl 缺省取机器常数 vision_marks 默认；back_tpl
            # 可选，缺省=coarse_tpl 的值。运行期 back 段不再自建临时模板。
            try:
                vmarks = (self.engine.constants.get() or {}) \
                    .get("vision_marks") or {}
            except Exception:
                vmarks = {}

            def _dt_tpl(key, mark_key, dfl):
                v = p.get(key)
                if v is not None and (not isinstance(v, str) or not v):
                    raise CalibError("%s 须为非空字符串" % key)
                v = v or vmarks.get(mark_key) or dfl
                if self.vision.templates.get(v) is None:
                    avail = sorted(self.vision.templates.list())
                    raise CalibError(
                        "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                        "/api/vision/templates 建模板）"
                        % (v, "、".join(avail) if avail else "（空）"))
                return v
            out["coarse_tpl"] = _dt_tpl("coarse_tpl", "tpl_coarse_die",
                                        "cv_chipback")
            out["up_tpl"] = _dt_tpl("up_tpl", "tpl_die_head", "tpl_die_head")
            out["back_tpl"] = _dt_tpl("back_tpl", "tpl_coarse_die",
                                      out["coarse_tpl"])
            # measure_back 已废止（迭代 29c：循环内不再 flip_place_back，
            # die 在 翻转↔键合头 间往复，back 段天然在循环内）——传入
            # 忽略不报错，不再透传进规范化 params
            return out
        if routine == "ws_rotate_repeat":
            if self.engine.workpieces is None \
                    or not self.engine.workpieces.wafer["present"]:
                raise CalibError("晶圆不在位（先上片）")
            out["rot_rad"] = self._validate_rot_rad(p)
            # 示教参数化**直接测角法**为唯一模式（迭代 34 起移除全自动
            # 路径）：须给 taught_points（或旧四键，映射 2 点），每点
            # tpl 已注册——缺示教参数（旧 detail.mode="auto" 路径）→400
            if not any(k in p for k in ("taught_points", "tpl_before",
                                        "tpl_after", "pose_before",
                                        "pose_after")):
                raise CalibError(
                    "ws_rotate_repeat 须给 taught_points（2~10 点示教，"
                    "每点 tpl 已注册；全自动模式已于迭代 34 移除）")
            out["taught"] = self._validate_rotate_taught(p)
            # 迭代 35 双示教位：旋转后走访直接走用户示教 pose_after
            # （缺省=pose 同位复测，已逐轴软限位 400）——预测走位 Δ 段
            # （含 start 预估 _check_rotate_pred_limits）已删
            return out
        raise CalibError("未知 routine %r" % routine)

    def _validate_rot_rad(self, p):
        """ws_rotate_repeat 旋转角校验（迭代 31）：首选 rot_rad（rad，
        (0, 0.6]）；rot_deg 保留兼容（×π/180 换算后走同一 (0,0.6] 校验——
        旧域 [0.05,30]° ⊂ 新域）；两参同给时 rot_rad 优先。行程校验按
        rad 与两点版同口径（|rot| ≤ WS_Rz 行程，行程守卫 ±1°）。"""
        v = p.get("rot_rad")
        if v is None and "rot_deg" in p:
            d = p.get("rot_deg")
            if isinstance(d, bool) or not isinstance(d, (int, float)):
                raise CalibError("rot_deg 须为数值：%r" % (d,))
            v = math.radians(float(d))
        if v is None:
            v = math.radians(1.0)              # 缺省 ≈1°
        if isinstance(v, bool) or not isinstance(v, (int, float)) \
                or not (0.0 < float(v) <= 0.6):
            raise CalibError("rot_rad 须为 (0, 0.6] 的数值（rad）：%r"
                             % (v,))
        rot = float(v)
        with self.engine.lock:
            ax = self.engine.axes["WS_Rz"]
            if rot > ax.hi - ax.lo:
                raise CalibError(
                    "rot_rad %.6f rad 超出 WS_Rz 行程 [%.4f, %.4f]"
                    % (rot, ax.lo, ax.hi))
        return rot

    def _validate_rotate_taught(self, p):
        """ws_rotate_repeat/ws_rotate_center/ws_rotate_gain 示教参数校验
        （迭代 29b 旧四键；迭代 31 taught_points 2~10 点；迭代 35 双示教
        位 pose_after，start 同步做→400）。
        taught_points：每元素 {tpl, pose{AS_X,AS_Y,WS_LS_X,WS_LS_Y},
        pose_after 可选}——tpl 必须已注册于 vision.templates（不存在列可
        用名；作业不自建不删）；pose 缺省分量=当前位、未知轴/非数值/逐轴
        软限位逐点校验；**pose_after 缺省=pose（同位复测）**，给出时走同
        一套 _check_taught_pose 校验（缺省分量=当前位——非继承 pose 分量）。
        旧四键（tpl_before/tpl_after+pose_before/after）映射为 2 点数组走
        同一路径（向后兼容，pose_after=pose）。返回规范化
        {"points": [{tpl, pose, pose_after}, ...]}。"""
        raw = p.get("taught_points")
        if raw is not None:
            if any(k in p for k in ("tpl_before", "tpl_after",
                                    "pose_before", "pose_after")):
                raise CalibError(
                    "taught_points 与旧四键（tpl_before/tpl_after/"
                    "pose_before/pose_after）互斥，只能给一种")
            if not isinstance(raw, (list, tuple)) \
                    or not (2 <= len(raw) <= 10):
                raise CalibError(
                    "taught_points 须为 2~10 元素的数组（每元素 "
                    "{tpl, pose, pose_after 可选}）：%r" % (raw,))
            items = []
            for i, it in enumerate(raw):
                if not isinstance(it, dict):
                    raise CalibError(
                        "taught_points[%d] 须为 {tpl, pose[, pose_after]} "
                        "dict：%r" % (i, it))
                pa_raw = it["pose_after"] if "pose_after" in it else None
                items.append((it.get("tpl"), "taught_points[%d].tpl" % i,
                              it.get("pose"), "taught_points[%d].pose" % i,
                              pa_raw, "taught_points[%d].pose_after" % i))
        else:
            # 旧四键 → 2 点数组（pose 缺省=全当前位，由逐点校验补；
            # pose_after 缺省=pose 同位复测）
            items = [(p.get("tpl_before"), "tpl_before",
                      p.get("pose_before"), "pose_before", None, None),
                     (p.get("tpl_after"), "tpl_after",
                      p.get("pose_after"), "pose_after", None, None)]
        points = []
        for tpl, tpl_key, pose, pose_key, pa_raw, pa_key in items:
            pose_v = self._check_taught_pose(pose, pose_key)
            pa_v = self._check_taught_pose(pa_raw, pa_key) \
                if pa_raw is not None else dict(pose_v)
            points.append({"tpl": self._check_taught_tpl(tpl, tpl_key),
                           "pose": pose_v, "pose_after": pa_v})
        return {"points": points}

    def _check_taught_tpl(self, v, key):
        """示教点模板校验：非空模板名 + 已注册（不存在列可用名）。"""
        if not isinstance(v, str) or not v:
            raise CalibError("示教模式 %s 必填（非空模板名）：%r"
                             % (key, v))
        if self.vision.templates.get(v) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (v, "、".join(avail) if avail else "（空）"))
        return v

    def _check_taught_pose(self, v, key):
        """示教点 pose 校验：{AS_X,AS_Y,WS_LS_X,WS_LS_Y} 子集 dict，缺省
        分量=当前位，未知轴/非数值/逐轴软限位。"""
        pose_axes = ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")
        if v is None:
            v = {}
        if not isinstance(v, dict):
            raise CalibError("%s 须为 {轴名: 位置} dict：%r" % (key, v))
        bad = [k for k in v if k not in pose_axes]
        if bad:
            raise CalibError("%s 含未知轴分量 %s（只允许 %s）"
                             % (key, bad, "/".join(pose_axes)))
        pose = {}
        for axis in pose_axes:
            val = v.get(axis)
            with self.engine.lock:
                ax = self.engine.axes[axis]
                lo, hi, cur = ax.lo, ax.hi, ax.pos
            if val is None:
                pose[axis] = cur                     # 缺省分量=当前位
                continue
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                raise CalibError("%s.%s 须为数值：%r" % (key, axis, val))
            if not (lo <= float(val) <= hi):
                raise CalibError(
                    "%s 的轴 %s 分量 %.4f 超出软限位 [%.3f, %.3f]"
                    % (key, axis, float(val), lo, hi))
            pose[axis] = float(val)
        return pose

    # ---- ws_rotate_center/ws_rotate_gain 参数/前置校验（迭代 32/32A）----
    def _validate_rotate_sweep(self, p):
        """旋转类扫描测校（ws_rotate_center 旋转中心标定 / ws_rotate_gain
        旋转角度增益标定）共用参数校验（start 同步→400；迭代 35 双示教
        位——taught_points 每点 pose/pose_after 均逐轴软限位，pose_after
        缺省=pose）：
        taught_points（≥2，复用 _validate_rotate_taught——模板已注册/
        pose/pose_after 四轴/逐轴软限位）+ step_rad（(0,0.3] rad，默认
        0.01）+ n_angles（整数 [2,8]，默认 3——角度档 ±step×1..n 对称）+
        repeat（整数 [3,100]，默认 10）+ wait_ms。
        行程校验：rz0±n_angles×step_rad 须在 WS_Rz 软限位内（±1°行程
        下缺省 0.01×3=±0.03 超程→400 并提示减小 step_rad/n_angles）。
        旋转后走访走用户 pose_after（预测走位 Δ 段及 start 预估
        _check_rotate_pred_limits 已于迭代 35 移除）。返回规范化 dict。"""
        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 10, 3, 100, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        v = p.get("step_rad", 0.01)
        if isinstance(v, bool) or not isinstance(v, (int, float)) \
                or not (0.0 < float(v) <= 0.3):
            raise CalibError("step_rad 须为 (0, 0.3] 的数值（rad）：%r"
                             % (v,))
        out["step_rad"] = float(v)
        out["n_angles"] = _num("n_angles", 3, 2, 8, integer=True)
        if self.engine.workpieces is None \
                or not self.engine.workpieces.wafer["present"]:
            raise CalibError("晶圆不在位（先上片）")
        if not any(k in p for k in ("taught_points", "tpl_before",
                                    "tpl_after", "pose_before",
                                    "pose_after")):
            raise CalibError("旋转扫描测校需要 taught_points（≥2 示教点；"
                             "须使标记在 rz0 可见）")
        out["taught"] = self._validate_rotate_taught(p)
        # WS_Rz 行程：rz0±n_angles×step_rad 须在软限位内
        span = out["n_angles"] * out["step_rad"]
        with self.engine.lock:
            ax = self.engine.axes["WS_Rz"]
            rz0, lo, hi = ax.pos, ax.lo, ax.hi
        if rz0 + span > hi or rz0 - span < lo:
            raise CalibError(
                "角度档 ±%d×%.6f=±%.6f rad 使 WS_Rz 目标 [%.6f, %.6f] 超出"
                "行程 [%.6f, %.6f]（rz0=%.6f）——减小 step_rad 或 n_angles"
                % (out["n_angles"], out["step_rad"], span,
                   rz0 - span, rz0 + span, lo, hi, rz0))
        return out

    # ---- 非正交角测校参数/前置校验（迭代 36，start 同步做→400）----
    def _validate_stage_nonortho(self, routine, cam_id, p):
        """as/ws_stage_nonortho（Y 导轨非正交角测校——迭代 37 栅格跳测法：
        基准尺=晶圆 die 阵列 wafer_sites 格点+site_marks quad 假定正交，
        走距=格点节距；详见 metrology/stage_nonortho.py 模块 docstring）：
        template_name（必填已注册，空/None→400 列可用名）+ ref_axis
        "x"|"y"（默认 "x"，Y 基准输出=X 基准的负值）+ k_points **奇数**
        [3,21]（默认 5；十字对称跳测中心±(K−1)/2 步，偶数→400）+
        pitch_mm [0.1,50]（**默认=处方 wafer_sites pitch**；
        X/Y 都校——auto 模式 pitch_x=pitch_y=size+gap，manual 用 pitch_x/
        pitch_y_mm——与处方不一致（容差 0.001mm）→400 并报告 pitch_x/
        pitch_y 实际值；处方 X/Y 不等且未给 pitch_mm→400 提示必填）+
        frames_per_point [1,10]（默认 3）+ repeat [2,10]（默认 3）+
        wait_ms + center_xy 可选（[x,y] 绝对轴位，逐轴软限位 400；缺省=
        作业启动时两轴当前位——**须已对准某 quad 角**，每 node 可见性靠
        走距=格点节距使相对相位逐 node 复现）+ 行程守卫（十字对称跳测，
        X 线/Y 线各自：center±(K−1)/2×pitch 两端都在走位轴软限位内）+
        流程忙守卫（同 microrot）+
        **视野无标记 start 即 400**（_check_marks 上移 start）。迭代 36 的
        FOV 跨距/模板匹配窗口守卫已删（标记不再漂移出画幅）。返回规范化
        dict。"""
        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 3, 2, 10, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        # 视觉模板必填（同 align_repeat 口径：空/None→400 列可用名）
        tpl_name = p.get("template_name")
        if not isinstance(tpl_name, str) or not tpl_name:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "template_name 必填（非空模板名；视觉模板库现有：%s——"
                "先 POST /api/vision/templates 建模板）"
                % ("、".join(avail) if avail else "（空）"))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        out["template_name"] = tpl_name
        # 基准轴（迭代 37）：X 基准报告 φ（与机器常数同语义），Y 基准报负值
        ref_axis = p.get("ref_axis", "x")
        if ref_axis not in ("x", "y"):
            raise CalibError("ref_axis 只能是 \"x\"（默认）/ \"y\"：%r"
                             % (ref_axis,))
        out["ref_axis"] = ref_axis
        # 工艺在跑（流程 running/paused、全片键合、一键串联）→ 拒绝动轴
        # （与 align_repeat_dynamic/microrot 同款守卫）
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止非正交角测校"
                             % "、".join(busy))
        out["k_points"] = _num("k_points", 5, 3, 21, integer=True)
        # 十字对称跳测须奇数 K（中心±(K−1)/2 步，X/Y 两线共中心=十字）
        if out["k_points"] % 2 != 1:
            raise CalibError(
                "k_points=%d 不合法：十字对称跳测须奇数 K（中心±(K−1)/2 "
                "步）——k_points 取 3,5,…,21" % out["k_points"])
        # 处方 wafer_sites 节距（基准尺物理约束：仿真标记只在格点上）
        if self.engine.recipe is None:
            raise CalibError("recipe not attached（无法读 wafer_sites 节距）")
        try:
            from recipe import derive_wafer_sites
            der = derive_wafer_sites(self.engine.recipe.get())
            pitch_x = float(der["pitch_x_mm"])
            pitch_y = float(der["pitch_y_mm"])
        except CalibError:
            raise
        except Exception as e:
            raise CalibError("处方 wafer_sites 节距读取失败：%s" % e)
        raw_pitch = p.get("pitch_mm")
        if raw_pitch is None:
            if abs(pitch_x - pitch_y) > 1e-6:
                raise CalibError(
                    "pitch_mm 必填（处方 wafer_sites X/Y 节距不等："
                    "pitch_x=%.4f pitch_y=%.4f——X/Y 两线同 pitch，无法"
                    "缺省）" % (pitch_x, pitch_y))
            out["pitch_mm"] = _num("pitch_mm", pitch_x, 0.1, 50.0)
        else:
            out["pitch_mm"] = _num("pitch_mm", pitch_x, 0.1, 50.0)
        if abs(out["pitch_mm"] - pitch_x) > 0.001 \
                or abs(out["pitch_mm"] - pitch_y) > 0.001:
            raise CalibError(
                "pitch_mm=%.4f 与处方 wafer_sites 节距不一致（pitch_x="
                "%.4f pitch_y=%.4f，容差 0.001mm）——仿真标记只在格点上，"
                "走距须=格点节距：改 pitch_mm 或先 PUT /api/recipe 调"
                "wafer_sites" % (out["pitch_mm"], pitch_x, pitch_y))
        out["frames_per_point"] = _num("frames_per_point", 3, 1, 10,
                                       integer=True)
        axes = STAGE_NONORTHO_AXES[routine]
        half = (out["k_points"] - 1) / 2.0 * out["pitch_mm"]
        # 中心位：可选 [x,y] 绝对轴位（逐轴软限位）；缺省=当前两轴位
        cx_raw = p.get("center_xy")
        with self.engine.lock:
            cur = [self.engine.axes[a].pos for a in axes]
            lim = {a: (self.engine.axes[a].lo, self.engine.axes[a].hi)
                   for a in axes}
        if cx_raw is None:
            center = [float(cur[0]), float(cur[1])]
        else:
            if not isinstance(cx_raw, (list, tuple)) or len(cx_raw) != 2 \
                    or any(isinstance(c, bool)
                           or not isinstance(c, (int, float))
                           or not (-1e4 <= float(c) <= 1e4) for c in cx_raw):
                raise CalibError(
                    "center_xy 须为 [x, y] 两个数值且分量 ∈ "
                    "[-10000, 10000]mm：%r" % (cx_raw,))
            center = [float(cx_raw[0]), float(cx_raw[1])]
        # 行程守卫（十字对称跳测）：X 线扫描轴=axes[0]、Y 线=axes[1]，
        # 各自 center±(K−1)/2×pitch 两端须在软限位内（center 亦须在位）
        for i, a in enumerate(axes):
            lo, hi = lim[a]
            t_lo, t_hi = center[i] - half, center[i] + half
            if not (lo <= t_lo and t_hi <= hi):
                raise CalibError(
                    "轴 %s 行程越界：中心 %.4f ± (K−1)/2×pitch=±%.4f → "
                    "[%.4f, %.4f]mm 超软限位 [%.3f, %.3f]——调整 center_xy/"
                    "k_points/pitch_mm"
                    % (a, center[i], half, t_lo, t_hi, lo, hi))
        out["center_xy"] = center
        # 视野无标记 → 400（i37 上移 start；运行期 run_stage_nonortho 兜底）
        _metro_repeat._check_marks(self, cam_id,
                                   self.vision.cams[cam_id].model)
        return out

    # ---- as_head_center 参数/前置校验（迭代 42，start 同步做→400）----
    def _validate_as_head_center(self, cam_id, p):
        """as_head_center（上吸盘位置标定，cam 恒 up——ROUTINE_CAM 已拦）：
        template_name（给了须非空且已注册→否则 400 列可用名；缺省=机器
        常数 vision_marks.tpl_head_chuck，本测校预留模板）+ repeat
        [2,100] 默认 10 + wait_ms + 流程忙守卫（动轴，同 microrot）。
        视野预检**不上移 start**：作业前置走位（AS→锚点/BS_Arm_Z→上焦
        面）在 start 之后才发生，start 时机台未必在测量位，当前位预检
        会误拒——由 run_as_head_center 走位到位后 _check_marks 兜底
        （无标记 → 作业 error 态带清晰文案）。返回规范化 dict。"""
        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 10, 2, 100, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        tpl_name = p.get("template_name")
        if tpl_name is None:
            # 缺省=机器常数 vision_marks.tpl_head_chuck（本测校预留模板）
            try:
                tpl_name = ((self.engine.constants.get() or {})
                            .get("vision_marks") or {}).get("tpl_head_chuck")
            except Exception:
                tpl_name = None
            tpl_name = tpl_name or "tpl_head_chuck"
        if not isinstance(tpl_name, str) or not tpl_name:
            raise CalibError("template_name 须为非空模板名（缺省=机器常数 "
                             "vision_marks.tpl_head_chuck）：%r"
                             % (tpl_name,))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        out["template_name"] = tpl_name
        # 工艺在跑（流程 running/paused、全片键合、一键串联）→ 拒绝动轴
        # （与 microrot 同款守卫）
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止上吸盘位置标定"
                             % "、".join(busy))
        return out

    # ---- best_focus 参数/前置校验（迭代 42，start 同步做→400）----
    def _validate_best_focus(self, cam_id, p):
        """best_focus（相机最佳焦平面标定，cam=up/down——ROUTINE_CAM
        已拦 coarse）：
        template_name（给了须非空且已注册→否则 400 列可用名；缺省按
        相机：up=vision_marks.tpl_head_chuck、down=vision_marks.
        tpl_wafer_site）+ z_start/z_end（z_start<z_end→否则 400；缺省
        =当前焦面常数 ±0.3mm：down=focus_level.as_z_chuck_focus_mm /
        up=head_z_upfocus_mm）+ step_mm（>0 且 ≤ 窗口，默认 0.02）+
        rounds [1,5] 默认 2 + frames_per_point [1,10] 默认 3 +
        wait_ms + 扫描行程在轴软限位内（down=AS_Z / up=BS_Arm_Z）+
        流程忙守卫（动轴）+ **down 视野预检上移 start**（_check_marks
        ——down 不走位、当前位即扫描位；up 前置走位在作业内，预检由
        run_best_focus 走位到位后兜底）。返回规范化 dict（含 axis/
        const_key/preset——up 前置走位：AS_Z→as_z_chuck_focus_mm、
        AS_X/Y→wafer_ega.as_anchor_xy_mm、BS_Arm_Z→head_z_upfocus_mm）。"""
        if cam_id not in ("up", "down"):
            raise CalibError("best_focus 的 cam 只能是 up/down：%r"
                             % (cam_id,))

        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        def _z(key, dfl):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(float(v)):
                raise CalibError("%s 须为有限数值：%r" % (key, v))
            return float(v)

        axis = "AS_Z" if cam_id == "down" else "BS_Arm_Z"
        const_key = ("as_z_chuck_focus_mm" if cam_id == "down"
                     else "head_z_upfocus_mm")
        try:
            const = self.engine.constants.get() or {}
        except Exception:
            const = {}
        fl = const.get("focus_level") or {}
        vmarks = const.get("vision_marks") or {}
        z_const = float(fl.get(const_key,
                               -1.0 if cam_id == "down" else -119.83))
        # 模板（缺省按相机取机器常数 vision_marks）
        tpl_name = p.get("template_name")
        if tpl_name is None:
            tpl_name = vmarks.get("tpl_head_chuck" if cam_id == "up"
                                  else "tpl_wafer_site")
            tpl_name = tpl_name or ("tpl_head_chuck" if cam_id == "up"
                                    else "cv_quad")
        if not isinstance(tpl_name, str) or not tpl_name:
            raise CalibError("template_name 须为非空模板名（缺省=机器常数 "
                             "vision_marks.%s）：%r"
                             % ("tpl_head_chuck" if cam_id == "up"
                                else "tpl_wafer_site", tpl_name))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        # z 窗口（缺省=当前焦面常数 ±0.3mm）
        z_start = _z("z_start", z_const - 0.3)
        z_end = _z("z_end", z_const + 0.3)
        if not z_start < z_end:
            raise CalibError("z_start 须小于 z_end：%r >= %r"
                             % (z_start, z_end))
        step = _num("step_mm", 0.02, 0.0001, z_end - z_start)
        rounds = _num("rounds", 2, 1, 5, integer=True)
        fpp = _num("frames_per_point", 3, 1, 10, integer=True)
        wait_ms = _num("wait_ms", 10, 0, 60000)
        # 行程守卫：扫描窗口两端在轴软限位内
        with self.engine.lock:
            ax = self.engine.axes[axis]
            lo, hi = ax.lo, ax.hi
        if z_start < lo or z_end > hi:
            raise CalibError(
                "扫描窗口 [%.4f, %.4f] 超出轴 %s 软限位 [%.3f, %.3f]"
                % (z_start, z_end, axis, lo, hi))
        # up 前置走位（锚点/焦面基准附近；逐轴软限位一并守卫）
        preset = None
        if cam_id == "up":
            anchor = (const.get("wafer_ega") or {}).get("as_anchor_xy_mm") \
                or fl.get("as_head_center_xy_mm") or [0.0, 6.0]
            preset = {"AS_Z": float(fl.get("as_z_chuck_focus_mm", -1.0)),
                      "AS_X": float(anchor[0]),
                      "AS_Y": float(anchor[1]),
                      "BS_Arm_Z": float(fl.get("head_z_upfocus_mm",
                                               -119.83))}
            with self.engine.lock:
                lim = {a: (self.engine.axes[a].lo, self.engine.axes[a].hi)
                       for a in preset}
            for a, tgt in preset.items():
                alo, ahi = lim[a]
                if not (alo <= tgt <= ahi):
                    raise CalibError(
                        "前置走位轴 %s 目标 %.4f 超出软限位 [%.3f, %.3f]"
                        "（机器常数锚点/焦面基准异常）"
                        % (a, tgt, alo, ahi))
        # 流程忙守卫（动轴，同 microrot）
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止最佳焦平面标定"
                             % "、".join(busy))
        if cam_id == "down":
            # down 不走位（当前位即扫描位）：视野无标记 start 即 400；
            # up 前置走位在作业内，预检由 run_best_focus 走位后兜底
            _metro_repeat._check_marks(self, cam_id,
                                       self.vision.cams[cam_id].model)
        return {"template_name": tpl_name, "axis": axis,
                "const_key": const_key, "z_start": z_start, "z_end": z_end,
                "step_mm": step, "rounds": rounds, "frames_per_point": fpp,
                "wait_ms": wait_ms, "preset": preset}


    # ---- 迭代 49 五项新测校的公共守卫 ----
    @staticmethod
    def _num_field(p, key, dfl, lo, hi, integer=False):
        """数值参数校验（与 _validate_extra 内嵌 _num 同口径）。"""
        v = p.get(key, dfl)
        if isinstance(v, bool) or not isinstance(v, (int, float)) \
                or not (lo <= float(v) <= hi) \
                or (integer and float(v) != int(v)):
            raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                             % (key, lo, hi, "整数" if integer else "数值",
                                v))
        return int(v) if integer else float(v)

    def _check_flow_idle(self, what):
        """工艺在跑（流程 running/paused、全片键合、一键串联）→ 400
        （动轴类测校同款守卫）。"""
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止%s" % ("、".join(busy),
                                                          what))

    def _dies_on(self, carrier):
        wp = getattr(self.engine, "workpieces", None)
        if wp is None:
            return []
        try:
            with wp._lock:
                return [d for d in wp.dies if d.get("carrier") == carrier]
        except Exception:
            return []

    def _wafer_present(self):
        wp = getattr(self.engine, "workpieces", None)
        try:
            return bool(wp is not None and wp.wafer.get("present"))
        except Exception:
            return False

    # ---- bs_rxry_axis 参数/前置校验（迭代 49，start 同步做→400）----
    def _validate_bs_rxry_axis(self, p):
        """bs_rxry_axis（键合头 BS_CAL_Rx/Ry 旋转轴线标定，cam 恒 up——
        ROUTINE_CAM 已拦）：template_name（缺省=机器常数 vision_marks.
        tpl_die_head；给了须已注册）+ angle_mrad（默认 1.5，[0.2,1.7]——
        且 ≤ BS_CAL 软限位 hi）+ repeat（默认 3，[1,10]）+ wait_ms +
        head 有 die 守卫 + 流程忙守卫。"""
        out = {"repeat": self._num_field(p, "repeat", 3, 1, 10,
                                         integer=True),
               "wait_ms": self._num_field(p, "wait_ms", 10, 0, 60000)}
        tpl_name = p.get("template_name")
        if tpl_name is None:
            try:
                tpl_name = ((self.engine.constants.get() or {})
                            .get("vision_marks") or {}).get("tpl_die_head")
            except Exception:
                tpl_name = None
            tpl_name = tpl_name or "tpl_die_head"
        if not isinstance(tpl_name, str) or not tpl_name:
            raise CalibError("template_name 须为非空模板名（缺省=机器常数 "
                             "vision_marks.tpl_die_head）：%r" % (tpl_name,))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        out["template_name"] = tpl_name
        out["angle_mrad"] = self._num_field(p, "angle_mrad", 1.5, 0.2, 1.7)
        # 角度 vs BS_CAL 软限位（限位可在线改，双保险）
        with self.engine.lock:
            for a in ("BS_CAL_Rx", "BS_CAL_Ry"):
                ax = self.engine.axes[a]
                if out["angle_mrad"] / 1000.0 > ax.hi:
                    raise CalibError(
                        "angle_mrad %.3f 超出 %s 软限位 ±%.5f rad"
                        % (out["angle_mrad"], a, ax.hi))
        if not self._dies_on("head"):
            raise CalibError("键合头无芯片（bs_rxry_axis 需要 head 上有 "
                             "die——先做交接或参照 prep_up）")
        self._check_flow_idle("调平轴旋转轴线标定")
        return out

    # ---- flip_handoff_height 参数/前置校验（迭代 49）----
    def _validate_flip_handoff_height(self, p):
        """flip_handoff_height（翻转交接接触高度标定）：chip_thickness_mm
        （默认 0.5，(0.01,5]）+ force_threshold_n（默认 2.0，(0.1,100]）+
        search_speed_mm_s（默认 0.2，(0.01,20]）+ flip 有 die 守卫（无→
        400"flip 上无芯片"）+ 流程忙守卫。"""
        out = {"chip_thickness_mm": self._num_field(
                   p, "chip_thickness_mm", 0.5, 0.01, 5.0),
               "force_threshold_n": self._num_field(
                   p, "force_threshold_n", 2.0, 0.1, 100.0),
               "search_speed_mm_s": self._num_field(
                   p, "search_speed_mm_s", 0.2, 0.01, 20.0)}
        if not self._dies_on("flip"):
            raise CalibError("flip 上无芯片（flip_handoff_height 需要翻转"
                             "拾取头上有 die——先做 flip_pick 取片）")
        self._check_flow_idle("翻转交接接触高度标定")
        return out

    # ---- bond_height_calib 参数/前置校验（迭代 49）----
    def _validate_bond_height_calib(self, p):
        """bond_height_calib（键合接触高度标定）：chip_thickness_mm（默认
        0.5）+ wafer_thickness_mm（默认 1.0，(0.01,10]）+ force_threshold_n
        （默认 2.0）+ search_speed_mm_s（默认 0.2）+ 晶圆在位 + head 有
        die 守卫（缺任一→400"请先放入芯片和晶圆"）+ 流程忙守卫。"""
        out = {"chip_thickness_mm": self._num_field(
                   p, "chip_thickness_mm", 0.5, 0.01, 5.0),
               "wafer_thickness_mm": self._num_field(
                   p, "wafer_thickness_mm", 1.0, 0.01, 10.0),
               "force_threshold_n": self._num_field(
                   p, "force_threshold_n", 2.0, 0.1, 100.0),
               "search_speed_mm_s": self._num_field(
                   p, "search_speed_mm_s", 0.2, 0.01, 20.0)}
        if not self._wafer_present() or not self._dies_on("head"):
            raise CalibError("请先放入芯片和晶圆（bond_height_calib 需要"
                             "晶圆在位 + 键合头上有 die）")
        self._check_flow_idle("键合接触高度标定")
        return out

    # ---- bond_temp_calib 参数/前置校验（迭代 49）----
    def _validate_bond_temp_calib(self, p):
        """bond_temp_calib（温度传感链标定）：heater（"up"→PulseHeat_Up /
        "wafer"→Heat_Dw，默认 "up"）+ setpoints_c（list，2~8 点，每点
        (0,450]℃（up）/(0,511.4]（wafer）；缺省 up=[100,200,300] /
        wafer=[50,80,110]）+ samples（默认 20，[5,200]）+
        sample_interval_ms（默认 100，[10,5000]）+ settle_tol_c（默认
        1.0，(0.1,10]）+ settle_timeout_s（默认 600，(1,3600]）+
        流程忙守卫。"""
        heater = p.get("heater", "up")
        if heater not in ("up", "wafer"):
            raise CalibError("heater 只能是 \"up\"（PulseHeat_Up，默认）/"
                             "\"wafer\"（Heat_Dw）：%r" % (heater,))
        out = {"heater": heater,
               "samples": self._num_field(p, "samples", 20, 5, 200,
                                          integer=True),
               "sample_interval_ms": self._num_field(
                   p, "sample_interval_ms", 100, 10, 5000, integer=True),
               "settle_tol_c": self._num_field(p, "settle_tol_c", 1.0,
                                               0.1, 10.0),
               "settle_timeout_s": self._num_field(p, "settle_timeout_s",
                                                   600, 1, 3600)}
        hi = 450.0 if heater == "up" else 511.4
        sps = p.get("setpoints_c")
        if sps is None:
            sps = [100.0, 200.0, 300.0] if heater == "up" \
                else [50.0, 80.0, 110.0]
        if not isinstance(sps, (list, tuple)) \
                or not (2 <= len(sps) <= 8):
            raise CalibError("setpoints_c 须为 2~8 元素数组：%r" % (sps,))
        vals = []
        for v in sps:
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (0.0 < float(v) <= hi):
                raise CalibError("setpoints_c 元素须在 (0,%s]℃：%r"
                                 % (hi, v))
            vals.append(float(v))
        out["setpoints_c"] = vals
        self._check_flow_idle("温度传感链标定")
        return out

    # ---- bond_force_calib 参数/前置校验（迭代 49）----
    def _validate_bond_force_calib(self, p):
        """bond_force_calib（力传感链标定）：setpoints_n（list，2~8 点，
        每点 (0,2000]N；缺省 [5,10,20,30]）+ samples（默认 20，[5,200]）+
        sample_interval_ms（默认 100，[10,5000]）+ force_threshold_n
        （默认 2.0，(0.1,100]）+ 晶圆在位 + head 有 die 守卫 + 流程忙
        守卫。"""
        out = {"samples": self._num_field(p, "samples", 20, 5, 200,
                                          integer=True),
               "sample_interval_ms": self._num_field(
                   p, "sample_interval_ms", 100, 10, 5000, integer=True),
               "force_threshold_n": self._num_field(
                   p, "force_threshold_n", 2.0, 0.1, 100.0)}
        sps = p.get("setpoints_n")
        if sps is None:
            sps = [5.0, 10.0, 20.0, 30.0]
        if not isinstance(sps, (list, tuple)) \
                or not (2 <= len(sps) <= 8):
            raise CalibError("setpoints_n 须为 2~8 元素数组：%r" % (sps,))
        vals = []
        for v in sps:
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (0.0 < float(v) <= 2000.0):
                raise CalibError("setpoints_n 元素须在 (0,2000]N：%r"
                                 % (v,))
            vals.append(float(v))
        out["setpoints_n"] = vals
        if not self._wafer_present() or not self._dies_on("head"):
            raise CalibError("请先放入芯片和晶圆（bond_force_calib 需要"
                             "晶圆在位 + 键合头上有 die）")
        self._check_flow_idle("力传感链标定")
        return out

    # ---- camera_microrot 参数/前置校验（迭代 31，start 同步做→400）----
    def _validate_microrot(self, cam_id, p):
        """camera_microrot（相机微小旋转角标定，纯 X 平移两点法）：
        template_name（迭代 34 起必填：空/None→400 列可用模板名；给了
        须已注册）；photo_pos1/photo_pos2=[x,y] 绝对轴位（迭代 34 起
        必填——缺省拍照位路径已移除，缺任一→400：成对/必填文案）；
        纯 X 方向——y 分量相等容差 0.01mm（迭代 34 放宽，原 1e-6；coarse
        单轴只有 X，y 分量接受传入但忽略）；**X 间距上下限校验已移除**
        （迭代 34：小杠杆可用性由用户模板匹配可达范围决定，间距过大
        出视场由作业期识别失败自曝）；逐轴软限位校验保留；repeat
        [3,100] 默认 10、wait_ms。
        move_stage（迭代 34 新增，选走位台）：down 可选 "align"（默认，
        对准台 AS_X/AS_Y，动相机=等效动标记 rel=−1）/"wafer"（晶圆台
        WS_LS_X/WS_LS_Y，动标记载体 rel=+1）；up 恒 align（传 "wafer"→
        400）；coarse 恒 FA_X（不参与，传入忽略）。走位轴映射表=
        MICROROT_AXES[(cam, move_stage)]。"""
        def _num(key, dfl, lo, hi, integer=False):
            v = p.get(key, dfl)
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not (lo <= float(v) <= hi) \
                    or (integer and float(v) != int(v)):
                raise CalibError("%s 须为 [%s,%s] 的%s：%r"
                                 % (key, lo, hi, "整数" if integer else "数值",
                                    v))
            return int(v) if integer else float(v)

        out = {"repeat": _num("repeat", 10, 3, 100, integer=True),
               "wait_ms": _num("wait_ms", 10, 0, 60000)}
        # 视觉模板必填（同 align_repeat 口径：空/None→400 列可用名）
        tpl_name = p.get("template_name")
        if not isinstance(tpl_name, str) or not tpl_name:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "template_name 必填（非空模板名；视觉模板库现有：%s——"
                "先 POST /api/vision/templates 建模板）"
                % ("、".join(avail) if avail else "（空）"))
        if self.vision.templates.get(tpl_name) is None:
            avail = sorted(self.vision.templates.list())
            raise CalibError(
                "模板 %r 不存在（视觉模板库现有：%s——先 POST "
                "/api/vision/templates 建模板）"
                % (tpl_name, "、".join(avail) if avail else "（空）"))
        out["template_name"] = tpl_name
        # 工艺在跑（流程 running/paused、全片键合、一键串联）→ 拒绝动轴
        # （与 align_repeat_dynamic 同款守卫）
        busy = []
        flow = getattr(self.engine, "flow", None)
        if flow is not None and flow.state in ("running", "paused"):
            busy.append("工艺流程 %r(%s)" % (flow.flow_name, flow.state))
        fw = getattr(self.engine, "full_wafer", None)
        if fw is not None and fw.status().get("active"):
            busy.append("全片键合")
        ch = getattr(self.engine, "chain", None)
        if ch is not None and ch.status().get("active"):
            busy.append("一键串联")
        if busy:
            raise CalibError("工艺运行中（%s），禁止相机微旋标定"
                             % "、".join(busy))
        # 选走位台（迭代 34）：down 可选 align（默认）/wafer；up 恒 align；
        # coarse 恒 FA_X 不参与（传入忽略）
        if cam_id == "down":
            move_stage = p.get("move_stage", "align")
            if move_stage not in ("align", "wafer"):
                raise CalibError(
                    "down 相机 move_stage 只能是 \"align\"（对准台，默认）/"
                    "\"wafer\"（晶圆台）：%r" % (move_stage,))
        elif cam_id == "up":
            move_stage = p.get("move_stage", "align")
            if move_stage != "align":
                raise CalibError(
                    "up 相机走位恒为对准台 AS_X/AS_Y（move_stage 只能是 "
                    "\"align\"）：%r" % (move_stage,))
        else:                        # coarse：恒 FA_X，不参与 move_stage
            move_stage = "align"
        out["move_stage"] = move_stage
        jog = MICROROT_AXES[cam_id]["align" if cam_id == "coarse"
                                     else move_stage]
        axes = [a for a, _s in jog]

        def _xy(key):
            v = p.get(key)
            if not isinstance(v, (list, tuple)) or len(v) != 2 \
                    or any(isinstance(c, bool)
                           or not isinstance(c, (int, float))
                           or not (-1e4 <= float(c) <= 1e4) for c in v):
                raise CalibError("%s 须为 [x, y] 两个数值且分量 ∈ "
                                 "[-10000, 10000]mm：%r" % (key, v))
            return [float(v[0]), float(v[1])]

        # 拍照位必填（迭代 34：无缺省路径）
        has1, has2 = "photo_pos1" in p, "photo_pos2" in p
        if not has1 and not has2:
            raise CalibError("photo_pos1/photo_pos2 必填（拍照位 1/2 都要"
                             "给，[x, y] 绝对轴位 mm；无后端缺省路径）")
        if not (has1 and has2):
            raise CalibError("photo_pos1 / photo_pos2 须成对给出")
        pos1, pos2 = _xy("photo_pos1"), _xy("photo_pos2")
        if cam_id != "coarse" and abs(pos2[1] - pos1[1]) > 0.01:
            raise CalibError(
                "photo_pos1/photo_pos2 须为纯 X 方向两点（y 分量相等，"
                "容差 0.01mm）：y1=%.6f y2=%.6f" % (pos1[1], pos2[1]))
        # 逐轴软限位（coarse 仅 X——y 分量对单轴相机忽略不校验）
        for i, axis in enumerate(axes):
            comp = pos1[i], pos2[i]
            with self.engine.lock:
                lo, hi = self.engine.axes[axis].lo, self.engine.axes[axis].hi
            if not (lo <= comp[0] <= hi and lo <= comp[1] <= hi):
                raise CalibError(
                    "轴 %s 的拍照位分量 %.4f/%.4fmm 超出软限位 "
                    "[%.3f, %.3f]" % (axis, comp[0], comp[1], lo, hi))
        out["photo_pos1"], out["photo_pos2"] = pos1, pos2
        return out

    def log_keep(self, job, line):   # 兼容占位（未用）
        self.log(job, line)

    def status(self, job_id):
        job = self._job(job_id)
        with self._lock:
            log = list(job["log"])
        return {"ok": True, "job_id": job_id, "routine": job["routine"],
                "camera": job["camera"], "state": job["state"],
                "progress": dict(job["progress"]),
                "t_start": job["t_start"], "t_end": job["t_end"],
                "error": job["error"], "log": log}

    def result(self, job_id):
        job = self._job(job_id)
        if job["state"] == "running":
            raise CalibError("作业 %d 仍在运行" % job_id)
        if job["state"] == "canceled":
            raise CalibError("作业 %d 已被取消（协作式取消于循环边界生效），"
                             "无测量结果" % job_id)
        if job["state"] == "error":
            raise CalibError("作业 %d 失败：%s" % (job_id, job["error"]))
        return {"ok": True, "result": job["result"]}

    def cancel(self, job_id):
        """协作式取消（迭代 29b，POST /api/calibration/cancel）：作业须为
        running——置 cancel_req 标志即返回；各测校循环在下一循环边界检查点
        （repeatability._check_cancel）抛 CalibCanceled，_run 收 canceled
        终态（finally 现场恢复照跑）。作业不存在/非 running → CalibError
        （API 400）。"""
        job = self._job(job_id)
        with self._lock:
            if job["state"] != "running":
                raise CalibError(
                    "作业 %d 不在运行中（state=%s），不能取消"
                    % (job_id, job["state"]))
            job["cancel_req"] = True
        return {"ok": True, "job_id": job_id, "state": "canceling"}

    def jobs_public(self):
        with self._lock:
            return {"ok": True,
                    "active": self._active,
                    "jobs": [{"job_id": j, "routine": self._jobs[j]["routine"],
                              "camera": self._jobs[j]["camera"],
                              "state": self._jobs[j]["state"],
                              "t_start": self._jobs[j]["t_start"]}
                             for j in self._order[-20:]]}

    def apply(self, job_id, fields=None):
        """把指定已完成作业的 measured 值（可只选部分字段）经机器常数校验
        写回并热生效。"""
        job = self._job(job_id)
        if job["state"] != "done" or not job["result"]:
            raise CalibError("作业 %d 未完成，无可应用结果" % job_id)
        measured = job["result"].get("measured")
        if not measured:
            # align_repeat_* 等纯测量程序无 measured 字段——明确 400，
            # 不落到 KeyError
            raise CalibError("作业 %d（%s）是纯测量程序，无机器常数写回字段，"
                             "不可应用" % (job_id, job["routine"]))
        if fields is not None:
            if not (isinstance(fields, list) and fields
                    and all(isinstance(f, str) for f in fields)):
                raise CalibError("fields 须为非空字段名数组")
            bad = [f for f in fields if f not in measured]
            if bad:
                raise CalibError("字段 %s 不在测量结果里（有 %s）"
                                 % (bad, sorted(measured)))
            measured = {f: measured[f] for f in fields}
        # 迭代 49：bond_temp_calib/bond_force_calib 的 measured 同时带显示
        # 别名键（gain/offset_c/offset_n）与常数名同值键（head_temp_gain
        # 等）——panel 测校页按 res.measured 键名渲染勾选，若把别名键
        # 提交上来会落进 cameras 组 400；此处把别名改路由到本作业的
        # 常数名键（同值，写回目标不变）
        if job["routine"] in ("bond_temp_calib", "bond_force_calib"):
            alias = {}
            for k in (job["result"].get("measured") or {}):
                if k not in APPLY_GROUP_ROUTE:
                    continue
                if k.endswith("_gain"):
                    alias["gain"] = k
                elif k.endswith("_offset_c"):
                    alias["offset_c"] = k
                elif k.endswith("_offset_n"):
                    alias["offset_n"] = k
            if alias:
                measured = {alias.get(f, f): v for f, v in measured.items()}
        # 组路由（迭代 32）：非相机字段写回对应机器常数组（如
        # ws_rotate_center 的 rz_center_offset_mm→fine_align；该校验链
        # FINE_ALIGN_FIELDS "xy" 原生支持），相机字段仍写 cameras[cam]
        cam_meas = {}
        grp_patch = {}
        for f, v in measured.items():
            g = APPLY_GROUP_ROUTE.get(f)
            if g is None:
                cam_meas[f] = v
            else:
                grp_patch.setdefault(g, {})[f] = v
        patch = {}
        if cam_meas:
            patch["cameras"] = {job["camera"]: cam_meas}
        patch.update(grp_patch)
        self.vision.constants.update(patch)      # 校验失败抛 ValueError → 400
        self.vision.apply_constants()            # 热生效
        pub = self.vision.constants.public(self.vision.constants_px_lookup)
        return {"ok": True, "applied": sorted(measured), "persisted": True,
                "derived": (pub["derived"]["cameras"].get(job["camera"])
                            if cam_meas else None)}

    def _job(self, job_id):
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None:
            raise CalibError("未知作业 %r" % (job_id,))
        return job

    # ================= 作业执行 =================
    def _run(self, job):
        cam_id = job["camera"]
        routine = job["routine"]
        # 新 routine（2026-09-08）：confocal=探头直读不动视觉；其余走
        # 视觉链（cam 参数可能是逗号组合——die_transfer 用第一台建模板
        # 无意义，统一不自建模板、不点灯切换，走各自实现的现场管理）
        if routine == "confocal_repeat":
            self._run_simple(job, lambda: _metro_extra.run_confocal_repeat(
                self, cam_id, job["params"]["repeat"],
                job["params"]["wait_ms"], job=job))
            return
        if routine == "die_transfer_repeat":
            # 两台相机（coarse+up）两盏灯、模板三件套（coarse_tpl/up_tpl/
            # back_tpl，start 已规范化+存在性校验，缺省=机器常数
            # vision_marks 默认/back=coarse）——现场（模式/光源/模板）
            # 由 worker 内部自管，不自建临时模板（迭代 34）
            self._run_simple(job, lambda: _metro_extra.run_die_transfer_repeat(
                self, job["params"]["repeat"], job["params"]["wait_ms"],
                job=job, coarse_tpl=job["params"].get("coarse_tpl"),
                up_tpl=job["params"].get("up_tpl"),
                back_tpl=job["params"].get("back_tpl"),
                measure_back=job["params"].get("measure_back", True)))
            return
        if routine == "ws_rotate_repeat":
            # 示教参数化直接测角法为唯一模式（迭代 34 起全自动路径移除，
            # start 无 taught_points 即 400）；现场自管
            self._run_simple(job, lambda: _metro_extra.run_ws_rotate_repeat(
                self, job["params"]["repeat"], job["params"]["wait_ms"],
                job["params"]["rot_rad"], job=job,
                taught=job["params"].get("taught")))
            return
        if routine == "ws_rotate_center":
            # 硅片台旋转中心标定（迭代 32）：现场自管（轴位/灯/模式在
            # rotate_center.run_ws_rotate_center 内部 finally 恢复）
            self._run_simple(job, lambda: _metro_rc.run_ws_rotate_center(
                self, job["params"]["repeat"], job["params"]["wait_ms"],
                job["params"]["step_rad"], job["params"]["n_angles"],
                job["params"]["taught"], job=job))
            return
        if routine == "ws_rotate_gain":
            # 硅片台旋转角度增益标定（迭代 32A）：现场自管
            self._run_simple(job, lambda: _metro_rg.run_ws_rotate_gain(
                self, job["params"]["repeat"], job["params"]["wait_ms"],
                job["params"]["step_rad"], job["params"]["n_angles"],
                job["params"]["taught"], job=job))
            return
        if routine in ("ws_stage_nonortho", "as_stage_nonortho"):
            # Y 导轨非正交角测校（迭代 37 栅格跳测法）：现场自管（轴位/
            # 灯/模式在 stage_nonortho.run_stage_nonortho 内部 finally
            # 恢复）；走位轴按 routine 查表 STAGE_NONORTHO_AXES，cam 恒
            # down；ws=硅片台走位 σ=+1 / as=对准台走位 σ=−1
            self._run_simple(job, lambda: _metro_nonortho.run_stage_nonortho(
                self, routine, cam_id, STAGE_NONORTHO_AXES[routine],
                job["params"], job=job))
            return
        if routine == "as_head_center":
            # 上吸盘位置标定（迭代 42）：现场自管（轴位/光源在
            # as_head_center.run_as_head_center 内部 finally 恢复；
            # 不切视觉模式——FAST/REAL 均可）
            self._run_simple(job, lambda: _metro_ahc.run_as_head_center(
                self, job["params"]["template_name"],
                job["params"]["repeat"], job["params"]["wait_ms"], job=job))
            return
        if routine == "best_focus":
            # 相机最佳焦平面标定（迭代 42）：现场自管（轴位/光源在
            # best_focus.run_best_focus 内部 finally 恢复；不切视觉模式）
            self._run_simple(job, lambda: _metro_bf.run_best_focus(
                self, cam_id, job["params"], job=job))
            return
        if routine == "bs_rxry_axis":
            # 键合头调平轴旋转轴线标定（迭代 49）：现场自管（轴位/光源/
            # 视觉模式在 bs_rxry_axis.run_bs_rxry_axis 内部 finally 恢复）
            self._run_simple(job, lambda: _metro_rxry.run_bs_rxry_axis(
                self, job["params"]["template_name"],
                job["params"]["angle_mrad"], job["params"]["repeat"],
                job["params"]["wait_ms"], job=job))
            return
        if routine == "flip_handoff_height":
            # 翻转交接接触高度标定（迭代 49）：纯轴/力链，现场自管
            self._run_simple(job, lambda: _metro_hh.run_flip_handoff_height(
                self, job["params"]["chip_thickness_mm"],
                job["params"]["force_threshold_n"],
                job["params"]["search_speed_mm_s"], job=job))
            return
        if routine == "bond_height_calib":
            # 键合接触高度标定（迭代 49）：纯轴/力链，现场自管
            self._run_simple(job, lambda: _metro_bh.run_bond_height_calib(
                self, job["params"]["chip_thickness_mm"],
                job["params"]["wafer_thickness_mm"],
                job["params"]["force_threshold_n"],
                job["params"]["search_speed_mm_s"], job=job))
            return
        if routine == "bond_temp_calib":
            # 温度传感链标定（迭代 49）：纯温度链，现场自管
            self._run_simple(job, lambda: _metro_tf.run_bond_temp_calib(
                self, job["params"]["heater"],
                job["params"]["setpoints_c"], job["params"]["samples"],
                job["params"]["sample_interval_ms"],
                job["params"]["settle_tol_c"],
                job["params"]["settle_timeout_s"], job=job))
            return
        if routine == "bond_force_calib":
            # 力传感链标定（迭代 49）：纯轴/力链，现场自管
            self._run_simple(job, lambda: _metro_tf.run_bond_force_calib(
                self, job["params"]["setpoints_n"], job["params"]["samples"],
                job["params"]["sample_interval_ms"],
                job["params"]["force_threshold_n"], job=job))
            return
        rt = self.vision.cams[cam_id]
        cam = rt.model
        # 现场保护：视觉模式 / 光源 / 轴位（finally 恢复）
        old_mode = self.vision.mode
        old_light = bool(self.points.get(cam.light_point, False))
        # 模板（迭代 34）：测校 9 项的 template_name 已在 start 校验必填
        # （空→400）——直接用视觉库用户模板，作业不自建、结束不删；
        # focus_scan/pixel_equivalent/rotation_direction 三旧项无模板
        # 参数（不在测校软件 9 项内），仍自建临时模板（_cal_<job_id>，
        # finally 删）——已知边界：三者仍自建临时模板。
        if routine in ("align_repeat_static", "align_repeat_dynamic",
                       "ws_move_repeat", "as_move_repeat",
                       "camera_microrot"):
            tpl = job["params"]["template_name"]
            self_tpl = False
        else:
            tpl = "_cal_%d" % job["job_id"]
            self_tpl = True
        axes_touched = []
        try:
            if old_mode != "real":
                self.vision.set_mode("real")
            # 自建模板（仅三旧项；亮灯后在当前位取，真机示教语义）
            self.vision._set_light(rt, True)
            self.vision._await_light(rt, True)
            if self_tpl:
                self.vision.template_create(
                    cam_id, name=tpl,
                    shape=SOURCE_SHAPE.get(cam.mark_source, ""))
            if routine == "focus_scan":
                res = self._focus_scan(job, cam, tpl, axes_touched)
            elif routine == "pixel_equivalent":
                res = self._pixel_equivalent(job, cam, tpl, axes_touched)
            elif routine == "align_repeat_static":
                res = _metro_repeat.run_static(
                    self, cam_id, tpl, job["params"]["repeat"],
                    job["params"]["wait_ms"], job=job)
            elif routine == "align_repeat_dynamic":
                res = _metro_repeat.run_dynamic(
                    self, cam_id, tpl, job["params"]["repeat"],
                    job["params"]["wait_ms"], job["params"]["start_xy"],
                    job["params"]["photo_xy"], axes_touched, job=job,
                    jog_axes_override=job["params"].get("jog_axes"))
            elif routine == "ws_move_repeat":
                res = _metro_extra.run_ws_move_repeat(
                    self, tpl, job["params"]["repeat"],
                    job["params"]["wait_ms"], job["params"]["start_xy"],
                    job["params"]["photo_xy"], axes_touched, job=job,
                    cam=cam_id,
                    jog_axes_override=job["params"].get("jog_axes"))
            elif routine == "as_move_repeat":
                res = _metro_extra.run_as_move_repeat(
                    self, tpl, job["params"]["repeat"],
                    job["params"]["wait_ms"], job["params"]["start_xy"],
                    job["params"]["photo_xy"], axes_touched, job=job,
                    cam=cam_id,
                    jog_axes_override=job["params"].get("jog_axes"))
            elif routine == "camera_microrot":
                # 迭代 32 起执行链迁 extra_routines（走位轴改对准台
                # MICROROT_AXES；metrology/microrot.py 原版留作参考，
                # _best_peak 仍复用其）；迭代 34 起按 move_stage 选台
                res = _metro_extra.run_camera_microrot(
                    self, cam_id, tpl, job["params"]["repeat"],
                    job["params"]["wait_ms"], job["params"]["photo_pos1"],
                    job["params"]["photo_pos2"], axes_touched, job=job,
                    move_stage=job["params"].get("move_stage", "align"))
            else:
                res = self._rotation_direction(job, cam, tpl, axes_touched)
            job["result"] = res
        except CalibCanceled as e:
            job["state"] = "canceled"
            job["error"] = str(e) or "作业已被取消"
        except Exception as e:
            job["state"] = "error"
            job["error"] = str(e)
        finally:
            # 恢复轴原位
            for ax, pos0 in reversed(axes_touched):
                try:
                    self._move_and_wait(ax, pos0)
                except Exception:
                    pass
            # 删临时模板（仅三旧项的自建；用户模板绝不删）/ 恢复光源 /
            # 恢复模式
            if self_tpl:
                try:
                    self.vision.template_delete(tpl)
                except Exception:
                    pass
            try:
                self.vision._set_light(rt, old_light)
            except Exception:
                pass
            if old_mode != "real":
                try:
                    self.vision.set_mode(old_mode)
                except Exception:
                    pass
            job["t_end"] = round(time.time(), 3)
            # 终态在最后才置：status=done 的语义包含现场已恢复，
            # 否则调用方轮询到 done 时恢复动作可能仍在进行（竞态）
            if job["state"] == "running":
                job["state"] = "done"
            with self._lock:
                if self._active == job["job_id"]:
                    self._active = None

    def _run_simple(self, job, fn):
        """无视觉现场管理的 routine 执行壳（confocal/交接/旋转——各自
        实现内部做轴恢复/物料回收）。"""
        try:
            job["result"] = fn()
        except CalibCanceled as e:
            job["state"] = "canceled"
            job["error"] = str(e) or "作业已被取消"
        except Exception as e:
            job["state"] = "error"
            job["error"] = str(e)
        finally:
            job["t_end"] = round(time.time(), 3)
            if job["state"] == "running":
                job["state"] = "done"
            with self._lock:
                if self._active == job["job_id"]:
                    self._active = None

    # ---- 公共动作 ----
    def _move_and_wait(self, axis, target, timeout=15.0):
        """走引擎正常 move 路径（联锁/限位/干涉检查全过），等到位。"""
        self.engine.move(axis, target)
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.engine.lock:
                ax = self.engine.axes[axis]
                done = (not ax.moving) and ax.state == "idle"
            if done:
                return
            time.sleep(0.01)
        raise CalibError("轴 %s 运动到 %s 超时" % (axis, target))

    def _axis_pos(self, axis):
        with self.engine.lock:
            return self.engine.axes[axis].pos

    def _identify(self, cam_id, tpl):
        """一次真实识别 → 结果 entry；无标记/识别失败抛清晰错误。"""
        entry = self.vision.identify(cam_id, template=tpl)
        res = entry.get("result") or {}
        cols = res.get("lstCol") or []
        if not cols:
            raise CalibError("视野内未找到标记（%s）"
                             % (entry.get("error") or "judge=false"))
        return entry

    @staticmethod
    def _centroid(res, min_score=0.0):
        """结果点云质心（die 多标记/单 cross 通用，平移一致）。
        min_score 过滤亚阈值伪实例：NCC 接受下限 max(0.30, 阈值×0.8) 附近
        会在大标记臂上产生 0.48 级幽灵峰（±~212px），质心只采纳判真点
        （score ≥ 相机阈值）。"""
        pts = [(c, r) for c, r, s in
               zip(res["lstCol"], res["lstRow"], res.get("lstScore") or [])
               if s >= min_score]
        if not pts:
            raise CalibError("无判真匹配点（得分均低于 %.2f）" % min_score)
        return (sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts))

    @staticmethod
    def _score(res):
        return max(res.get("lstScore") or [0.0])

    # ---- 1. 焦面标定 ----
    def _focus_scan(self, job, cam, tpl, axes_touched):
        cam_id = job["camera"]
        axis = SCAN_Z_AXIS[cam_id]
        p = job["params"]
        dof_mm = cam.dof_um / 1000.0
        rng = float(p.get("range_mm", 3.0 * dof_mm))     # 默认 ±3×DOF
        step = float(p.get("step_mm", max(dof_mm / 2.0, 0.0025)))
        if not (0.0005 <= step <= rng):
            raise CalibError("step_mm 须在 [0.0005, range_mm]：%r" % step)
        q0 = self._axis_pos(axis)
        axes_touched.append((axis, q0))
        # 起点几何（峰值 wd 换算基准）：相机 z 与目标标记 z
        positions, poses = vm._engine_poses(self.engine)
        cam_pos = vm.camera_pose(self.engine, cam, poses, positions)[0]
        cam_z0 = cam_pos[2]
        marks = vm.collect_marks(self.engine, cam, positions, poses)
        if not marks:
            raise CalibError("相机 %s 无可用标记（先上片/取 die）" % cam_id)
        tgt = min(marks, key=lambda m: abs(m.x - cam_pos[0])
                  + abs(m.y - cam_pos[1]))
        mark_z0 = tgt.z
        # 扫描（该轴负责"相机↔标记"相对 Z：down=AS_Z 动镜头、up=BS_Arm_Z
        # 动标记——距离变化量都等于轴位移量，符号无关取绝对距离）
        n = int(round(2.0 * rng / step)) + 1
        points = []
        for i in range(n):
            q = q0 - rng + i * step
            job["progress"] = {"i": i + 1, "n": n,
                               "note": "%s→%.4f" % (axis, q)}
            self._move_and_wait(axis, q)
            try:
                entry = self._identify(cam_id, tpl)
                res = entry["result"]
                points.append({"axis_pos": round(self._axis_pos(axis), 6),
                               "score": round(self._score(res), 4),
                               "judge": bool(res.get("judge")),
                               "focus_err_um": entry.get("focus_err_um")})
            except Exception:
                # 找不到标记/识别失败的点也记录（score=0，画曲线用）
                points.append({"axis_pos": round(self._axis_pos(axis), 6),
                               "score": 0.0, "judge": False,
                               "focus_err_um": None})
        best = max(range(n), key=lambda i: points[i]["score"])
        if points[best]["score"] <= 0.0:
            raise CalibError("整个扫描范围都没识别到标记，检查物料/光照/模板")
        # 抛物线拟合峰值（三点；峰在边界则直接用边界点并降置信）
        q_peak = points[best]["axis_pos"]
        fit_note = "parabola"
        if 0 < best < n - 1:
            y1, y2, y3 = (points[best - 1]["score"], points[best]["score"],
                          points[best + 1]["score"])
            denom = y1 - 2.0 * y2 + y3
            if abs(denom) > 1e-9:
                q_peak += 0.5 * step * (y1 - y3) / denom
        else:
            fit_note = "peak_at_edge（峰值在扫描边界，建议扩 range_mm）"
        # 实测 wd：峰值处 |标记 z − 相机 z|（相对 Z 由扫描轴提供）
        dz = q_peak - q0
        if cam_id == "down":     # 相机动：cam_z 变
            wd_meas = abs((cam_z0 + dz) - mark_z0)
        else:                    # up：标记动（BS_Arm_Z 带键合头）
            wd_meas = abs(cam_z0 - (mark_z0 + dz))
        wd_meas = round(wd_meas, 4)
        return {"routine": "focus_scan", "camera": cam_id,
                "measured": {"wd_mm": wd_meas},
                "current": {"wd_mm": cam.wd_mm},
                "points": points,
                "confidence": points[best]["score"],
                "detail": {"axis": axis, "q0": round(q0, 6),
                           "q_peak": round(q_peak, 6), "fit": fit_note,
                           "range_mm": rng, "step_mm": step},
                "ts": round(time.time(), 3)}

    # ---- 2. 像素当量标定 ----
    def _pixel_equivalent(self, job, cam, tpl, axes_touched):
        cam_id = job["camera"]
        p = job["params"]
        axis, rel_sign = JOG_AXES[cam_id][0] if "axis" not in p else \
            self._pick_jog_axis(cam_id, p["axis"])
        delta = float(p.get("delta_mm", 1.5))
        if not (0.1 <= abs(delta) <= 10.0):
            raise CalibError("delta_mm 须在 [0.1,10]：%r" % delta)
        q0 = self._axis_pos(axis)
        axes_touched.append((axis, q0))
        job["progress"] = {"i": 1, "n": 3, "note": "起点识别"}
        c0 = self._centroid(self._identify(cam_id, tpl)["result"],
                            cam.score_threshold)
        job["progress"] = {"i": 2, "n": 3, "note": "%s %+0.3fmm"
                           % (axis, delta)}
        self._move_and_wait(axis, q0 + delta)
        c1 = self._centroid(self._identify(cam_id, tpl)["result"],
                            cam.score_threshold)
        dpx = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
        if dpx < 5.0:
            raise CalibError("像素位移过小（%.2fpx）——标记没动？检查轴/物料"
                             % dpx)
        k_um_meas = abs(delta) / dpx * 1000.0
        mag_meas = cam.pixel_um / k_um_meas
        job["progress"] = {"i": 3, "n": 3, "note": "完成"}
        return {"routine": "pixel_equivalent", "camera": cam_id,
                "measured": {"mag": round(mag_meas, 6)},
                "current": {"mag": cam.mag},
                "points": [{"axis_pos": round(q0, 6),
                            "col": round(c0[0], 3), "row": round(c0[1], 3)},
                           {"axis_pos": round(q0 + delta, 6),
                            "col": round(c1[0], 3), "row": round(c1[1], 3)}],
                "confidence": self._score(
                    self.vision.results[-1]["result"])
                if self.vision.results else None,
                "detail": {"axis": axis, "delta_mm": delta,
                           "delta_px": round(dpx, 3),
                           "k_um_px_measured": round(k_um_meas, 6),
                           "k_um_px_current": round(cam.k_mm * 1000.0, 6)},
                "ts": round(time.time(), 3)}

    # ---- 3. 旋转与方向标定 ----
    def _rotation_direction(self, job, cam, tpl, axes_touched):
        cam_id = job["camera"]
        p = job["params"]
        if len(JOG_AXES[cam_id]) < 2:
            raise CalibError(
                "相机 %s 的标记载体只有一个平移点动轴（%s），"
                "rotation_direction 需要 X/Y 两轴，不支持该相机"
                % (cam_id, JOG_AXES[cam_id][0][0]))
        delta = float(p.get("delta_mm", 1.0))
        if not (0.1 <= abs(delta) <= 10.0):
            raise CalibError("delta_mm 须在 [0.1,10]：%r" % delta)
        vecs = []
        for ax_i, (axis, rel_sign) in enumerate(JOG_AXES[cam_id]):
            q0 = self._axis_pos(axis)
            axes_touched.append((axis, q0))
            job["progress"] = {"i": ax_i * 2 + 1, "n": 4,
                               "note": "%s 起点识别" % axis}
            c0 = self._centroid(self._identify(cam_id, tpl)["result"],
                                cam.score_threshold)
            job["progress"] = {"i": ax_i * 2 + 2, "n": 4,
                               "note": "%s %+0.3fmm" % (axis, delta)}
            self._move_and_wait(axis, q0 + delta)
            c1 = self._centroid(self._identify(cam_id, tpl)["result"],
                                cam.score_threshold)
            self._move_and_wait(axis, q0)          # 先回位再动下一轴
            axes_touched.remove((axis, q0))
            # 标记相对相机的世界位移 = rel_sign × delta × 轴方向单位向量
            w = (rel_sign * delta if ax_i == 0 else 0.0,
                 rel_sign * delta if ax_i == 1 else 0.0)
            vecs.append((w, (c1[0] - c0[0], c1[1] - c0[1])))
        (wx, _), (dux, dvx) = vecs[0]
        (_, wy), (duy, dvy) = vecs[1]
        if math.hypot(dux, dvx) < 5.0 or math.hypot(duy, dvy) < 5.0:
            raise CalibError("像素位移过小——标记没动？检查轴/物料")
        # 枚举 (dir_x, dir_y, swap) × 连续 rot：模型 px_vec = S·D·R(θ)·w/k
        # （与 CameraModel.project_rel 同式）。map_ang 的角度对 θ 精确线性、
        # 斜率 = dx·dy（dir 乘在旋转之后：dy=-1 时角度随 θ 反向变化）——
        # 对每个组合用 X 向量按该斜率解 θ，再用 Y 向量验证角残差，取最优。
        best = None
        for dx_s in (1.0, -1.0):
            for dy_s in (1.0, -1.0):
                for swap in (False, True):
                    def map_ang(w, theta):
                        cr, sr = math.cos(theta), math.sin(theta)
                        x2 = cr * w[0] - sr * w[1]
                        y2 = sr * w[0] + cr * w[1]
                        u, v = x2 * dx_s, y2 * dy_s
                        if swap:
                            u, v = v, u
                        return math.atan2(v, u)
                    base = map_ang((wx, 0.0), 0.0)
                    theta = (math.atan2(dvx, dux) - base) * (dx_s * dy_s)
                    # X 向量残差也必须参与：swap 与 θ+π/2 组合对 Y 向量
                    # 简并（Y 残差相同），只有 X 向量能打破——否则拟合结果
                    # 由噪声抛硬币（2026-08-21 回归实测 swap=True θ=π/2
                    # 与正确组合 resid 并列 6e-4，顺序枚举偶发翻转）
                    resid = abs(_wrap_angle(
                        map_ang((wx, 0.0), theta)
                        - math.atan2(dvx, dux))) + abs(_wrap_angle(
                        map_ang((0.0, wy), theta)
                        - math.atan2(dvy, duy)))
                    if best is None or resid < best[0]:
                        best = (resid, dx_s, dy_s, swap, theta)
        resid, dx_s, dy_s, swap, theta = best
        theta = _wrap_angle(theta)
        # 简并规范化：(D,θ) 与 (−D,θ+π) 产生完全相同的像素映射，点向量
        # 测量无法区分——统一取 |θ|≤π/2 的代表（安装微旋为 mrad 量级，
        # 朝向信息由 dir 符号承担，写回自洽）
        if abs(theta) > math.pi / 2:
            dx_s, dy_s, theta = -dx_s, -dy_s, _wrap_angle(theta + math.pi)
        note = None
        if swap != cam.swap_xy:
            note = ("拟合最优组合 swap_xy=%s 与当前 %s 不一致（未纳入写回）"
                    % (swap, cam.swap_xy))
        return {"routine": "rotation_direction", "camera": cam_id,
                "measured": {"rot_mrad": round(theta * 1000.0, 3),
                             "dir_x": dx_s, "dir_y": dy_s},
                "current": {"rot_mrad": cam.rot * 1000.0,
                            "dir_x": cam.dir_x, "dir_y": cam.dir_y},
                "points": [{"axis": JOG_AXES[cam_id][0][0],
                            "delta_mm": delta,
                            "du": round(dux, 3), "dv": round(dvx, 3)},
                           {"axis": JOG_AXES[cam_id][1][0],
                            "delta_mm": delta,
                            "du": round(duy, 3), "dv": round(dvy, 3)}],
                "confidence": round(1.0 - min(1.0, resid / math.pi), 4),
                "detail": {"angular_resid_rad": round(resid, 6),
                           "note": note},
                "ts": round(time.time(), 3)}

    def _pick_jog_axis(self, cam_id, axis):
        for a, s in JOG_AXES[cam_id]:
            if a == axis:
                return a, s
        raise CalibError("相机 %s 的点动轴只能是 %s"
                         % (cam_id, [a for a, _ in JOG_AXES[cam_id]]))


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
