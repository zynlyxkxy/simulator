# -*- coding: utf-8 -*-
"""machine_constants.py — 机器常数：不随工艺变化的设备固有参数存储与校验。

与处方（recipe.py，管物料/布局）互补：本模块管设备本体常数。纯标准库。
顶层按分组组织，当前开放 "cameras" 组（三相机内参：pixel_um/mag/dir_x/
dir_y/swap_xy/rot_mrad/wd_mm/dof_um + **px_u/px_v 传感器分辨率**（2026-08-24
迁入：用户"像素值也要放到机械常数分别有u和v"，vision_cameras.json 的
px_x/px_y 同步废除）——字段口径参考 d2w_sim_semiconductor_
refactor 的 vision 配置）与 "flip_pick" 组（翻转取片工艺机械常数，
2026-08-24 起，见 docs/工艺流程_翻转取片_V1.md——取片点位/接触面高度/
姿态轴点位/真空超时，全部为设备本体参数，不随处方变化）与
"flip_handoff" 组（芯片交接工艺机械常数，2026-08-26 起，见
docs/工艺流程_芯片交接_V1.md——对准台退避位/键合头安全·接触·抬起高度/
交接位与撤离位 FA_X/真空超时，正逆向交接共用）与 "focus_level" 组
（调焦调平机械常数，2026-08-27 起，见 docs/调焦调平与高度链_V1.md——
共焦装配/读数模型参数与对焦高度链基准）与 "fine_align" 组（逐键合位
精对准机械常数，2026-08-28 迭代 6 起，见 docs/逐键合位精对准_V1.md——
WS_Rz 旋转枢轴标定偏移与轴位→物理角换算）与 "bond" 组（热压键合机械
常数，2026-08-28 迭代 8 起——2026-08-31 迭代 9 起精简为整机标定量：
裸接触位/寻力起始位；弹簧刚度/双加热器热对象/PID 参数迁入
tuning.json（下位机调参），读数噪声迁入 recipe sim_sensors）与
"vision_marks" 组（视觉对准标记模板，2026-09-04 迭代 12 起——每次拍照
必须按本组模板名识别；值须为 vision_templates.json 中已建模板名，
校验含存在性检查）与 "stage_geo" 组（运动台几何机械常数，迭代 36
起——as/ws_y_rail_dir_urad Y 导轨方向角真值/注入旋钮，error_kinematics
六自由度 FK 消费；as/ws_nonortho_urad 非正交角测校结果，as/ws_stage_
nonortho 测校 apply 写回）与 "bs_rxry_axis" 组（键合头 BS_CAL_Rx/Ry
调平轴旋转轴线标定——轴上一点+方向向量，迭代 49 起，bs_rxry_axis
测校 apply 写回；默认与 error_kinematics.json bond_head 链真值对齐）与
"sensor_calib" 组（温度/力传感链标定常数——发布读数=(raw−offset)/
gain，迭代 49 起，bond_temp_calib/bond_force_calib 测校 apply 写回）。
未来扩展 points/hardware 等组（文件头 _comment 已注明）。

- 加载：LoadError（ValueError 子类）抛出即拒绝拉起（main.py 启动路径）。
- 更新：update() 部分字段合并 → 全量校验 → 通过才替换 + 原子写盘
  （tmp+os.replace，同 recipe.py 惯例）；校验失败抛 ValueError，errors[]
  逐条定位字段路径，原常数不被污染。
- 派生量不落盘：k_mm=pixel_um/mag/1000、fov=px_u/px_v×k 由 public() 计算。
"""
import json
import math
import os
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))
CONSTANTS_JSON = os.path.join(ROOT, "machine_constants.json")

# 当前开放的组与字段（白名单；"_"-前缀键为注释，不参与校验）
GROUPS = ("cameras", "flip_pick", "flip_handoff", "wafer_ega", "focus_level",
          "fine_align", "bond", "vision_marks", "stage_geo",
          "bs_rxry_axis", "sensor_calib")
CAMERA_FIELDS = ("pixel_um", "mag", "dir_x", "dir_y", "swap_xy",
                 "rot_mrad", "wd_mm", "dof_um", "px_u", "px_v")
# 字段 -> (最小, 最大)（正数类的合理区间；None 表示只查有限性）
RANGES = {
    "pixel_um": (0.1, 100.0),        # 像元尺寸 µm
    "mag": (0.01, 100.0),            # 镜头放大倍率
    "wd_mm": (0.1, 2000.0),          # 工作距离 mm
    "dof_um": (0.1, 5000.0),         # 景深半范围 µm
    "rot_mrad": (-100.0, 100.0),     # 安装微旋
}
# 传感器分辨率（整数像素）
PX_RANGE = (16, 4096)


def _is_finite_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def _check_vec(errors, pre, key, v, dim):
    """校验 dim 维有限数向量。"""
    if not (isinstance(v, (list, tuple)) and len(v) == dim
            and all(_is_finite_num(x) for x in v)):
        errors.append("%s.%s 须为 %d 维有限数向量: %r" % (pre, key, dim, v))


# flip_pick 组（翻转取片工艺机械常数）：字段 -> 校验类型
#   "xy"=二维有限数向量 / ("num",lo,hi)=区间数值
FLIP_PICK_FIELDS = {
    "pick_point_xy_mm": "xy",            # 取片位三点同心（世界 xy）
    "film_surface_z_mm": ("num", 500.0, 1200.0),   # 蓝膜面世界 z
    "chuck_face_z0_mm": ("num", 500.0, 1200.0),    # 吸盘面 FA_Z=0 世界 z
    "pin_top_z0_mm": ("num", 500.0, 1200.0),       # 顶针帽顶 ES_PunchZ=0
    "fa_x_pick_mm": ("num", -1250.0, 0.0),         # 取片位 FA_X
    "fa_rx_pick_rad": ("num", -3.2, 3.2),          # 取片姿态（拾取头朝下）
    "fa_z_safe_mm": ("num", -40.0, 40.0),          # Rx 旋转安全位
    "fa_rx_flip_rad": ("num", -3.2, 3.2),          # 翻转位（芯片背面朝上）
    "fa_z_view_mm": ("num", -40.0, 40.0),          # 粗对准景深位 FA_Z
    "coarse_die_center_mm": "xy",        # 粗对准芯片中心标定位置（世界 xy）
    "pick_vac_timeout_s": ("num", 0.1, 60.0),      # 拾取真空建立超时
}

# wafer_ega 组（晶圆上片与全局对准机械常数，2026-08-26 起，
# 见 docs/晶圆上片与全局对准_EGA_V1.md）：硅片台 EGA 测量位、对准台↔硅片台
# 锚点、EGA 测点数量/超时等。
WAFER_EGA_FIELDS = {
    "ws_ega_xyz_mm": "xyz",              # 硅片台 EGA 测量位 [WS_X, WS_Y, WS_Z]
    "as_anchor_xy_mm": "xy",             # 对准台对准硅片台中心锚点 [AS_X, AS_Y]
    "ega_n_points": ("int", 3, 25),      # EGA 测量点数
    "ega_measure_timeout_s": ("num", 0.5, 120.0),  # 单点视觉测量超时
}

# flip_handoff 组（芯片交接工艺机械常数，flip↔head 正逆共用）：
# 字段 -> 校验类型（同 flip_pick 组约定）
FLIP_HANDOFF_FIELDS = {
    "as_safe_xy_mm": "xy",               # 对准台退避安全位（AS_X/AS_Y）
    "head_z_safe_mm": ("num", -190.0, 0.0),      # 键合头安全高度 BS_Arm_Z
    "head_z_touch_mm": ("num", -190.0, 0.0),     # 键合头下降接触位（正逆共用）
    "head_z_lift_mm": ("num", -190.0, 0.0),      # 交接后键合臂抬起高度
    "fa_x_handoff_mm": ("num", -1250.0, 0.0),    # 交接位 FA_X（吸盘心=头心）
    "fa_x_retreat_mm": ("num", -1250.0, 0.0),    # 交接后翻转撤离位
    "handoff_vac_timeout_s": ("num", 0.1, 60.0),  # 交接真空建立超时
    # 产率模式待机位（2026-09-05 迭代 15，pipelined=true 时）：翻转模块
    # 携片（预取完成的下一颗 die）等待位 FA_X——取交接位附近安全值，
    # 预取完成后在此等当前颗 bond 结束；**待真机标定**（初值 −1050：
    # −1100 时待机吸盘盒与对准相机承载台在 AS_X≈+4 处相交 >ε 被沙盒
    # 409——冒烟实测暴露后外移 50mm）
    "fa_x_standby_mm": ("num", -1250.0, 0.0),
}


# focus_level 组（调焦调平机械常数，2026-08-27 起，见
# docs/调焦调平与高度链_V1.md）：共焦装配/读数模型参数与对焦高度链基准。
# 注：共焦噪声 confocal_sigma_um 不属"可标定的设备认知"，2026-08-27 迭代 2
# 起移到 recipe sim_sensors 组（测校页"传感器仿真"卡）。
FOCUS_LEVEL_FIELDS = {
    "confocal_cam_dx_mm": ("num", -100.0, 100.0),  # 共焦相对相机 X 偏移
    "confocal_cam_dy_mm": ("num", -100.0, 100.0),  # 共焦相对相机 Y 偏移
    "confocal_mid_up_um": ("num", 0.0, 5000.0),    # 上共焦中点读数（=相机最佳焦面）
    "confocal_mid_down_um": ("num", 0.0, 5000.0),  # 下共焦中点读数
    "confocal_range_um": ("num", 1.0, 10000.0),    # 共焦量程（0~range）
    "as_z_chuck_focus_mm": ("num", -2.0, 6.0),     # 下焦面=裸吸盘面时的 AS_Z
    "head_z_upfocus_mm": ("num", -190.0, 0.0),     # 上焦面=头吸盘面时的 BS_Arm_Z
    "level_tol_urad": ("num", 0.0, 10000.0),       # 调平验收残余阈值
    "focus_search_span_um": ("num", 0.0, 5000.0),  # 共焦无读数时 AS_Z 搜索窗口
    "focus_search_step_um": ("num", 0.1, 1000.0),  # 搜索步距
    "as_head_center_xy_mm": "xy",                   # 上相机认吸盘中心时的 AS_X/Y
}


# fine_align 组（逐键合位精对准机械常数，2026-08-28 迭代 6 起，见
# docs/逐键合位精对准_V1.md）：WS_Rz 旋转枢轴标定偏移与轴位→物理角换算。
# bond_offset_*（2026-09-05 迭代 15）：键合附加补偿——由键合后检测
# measured 的 mean 系统偏差给出（结果分析页"补偿量建议"一键应用），
# bond 流程在精对准完成后/下压前施加（bond_offset_apply op：WS 设计系
# 平移 +offset 经 EGA map 线性部换算、旋转 +orz 含 map 同步）；在
# fl_ready 施加会被精对准补偿环吸收（fa 每次重测 site 标记重算 WS
# 补偿，预偏移净效应为零），只有 fa 之后的偏移才留在 die 落点里。
# 补偿系统偏差，不补偿随机散布，应用后 measured mean→0。默认 0=不补偿。
# frame_cal_*（2026-09-07 迭代 20）：精对准计量帧标定残差——上/下相机
# 两条测量链帧间偏差经出厂标定后的常数残余，控制器按标定值修正测量 Δ
# （fa 解算 delta_used = delta_meas + frame_cal）；0=标定理想，注入非零
# 可演示标定残余（直接平移到 truth）。详见 docs/键合误差模型_V1.md。
FINE_ALIGN_FIELDS = {
    "rz_center_offset_mm": "xy",            # Rz 轴心相对吸盘心偏移·标定值
    # 迭代 35：rz_center_offset_mm 标定时的机台位 [WS_LS_X, WS_LS_Y]
    # （ws_rotate_center measured 双键之一，apply 双写回）——消费侧枢轴
    # 信念按 pivot=吸盘心信念+offset+(当前WS_LS_XY−ref_xy) 平移；[0,0]
    # （旧数据缺省）=平移项关闭=原行为（兼容）
    "rz_center_ref_xy_mm": "xy",
    "rz_gain": ("num", 1e-6, 1e6),          # WS_Rz 轴位→物理角增益
    "rz_sign": ("num", -1.0, 1.0),          # 方向（±1）
    "bond_offset_x_um": ("num", -1000.0, 1000.0),   # 键合附加补偿 X（µm）
    "bond_offset_y_um": ("num", -1000.0, 1000.0),   # 键合附加补偿 Y（µm）
    "bond_offset_rz_urad": ("num", -1e5, 1e5),      # 键合附加补偿 Rz（µrad）
    "frame_cal_x_um": ("num", -100.0, 100.0),       # 计量帧标定残差 X（µm）
    "frame_cal_y_um": ("num", -100.0, 100.0),       # 计量帧标定残差 Y（µm）
    "frame_cal_rz_urad": ("num", -1e4, 1e4),        # 计量帧标定残差 Rz（µrad）
}

# bond 组（热压键合机械常数）：2026-08-31 迭代 9 起只保留整机标定量——
# head_z_contact_mm 重新定义为"无晶圆无芯片、吸盘表面贴陶瓷面"的裸接触位
# （有效接触位 = 裸接触位 + 晶圆厚 + 芯片厚，换算函数见 engine.
# effective_bond_contact_mm）；head_z_search_mm=两段速下压快慢切换高度
# （慢速寻力段起点，同为裸值，流程按厚度换算）。弹簧刚度/热对象/读数噪声
# 已迁出（tuning.json / recipe sim_sensors）。全部 ("num",lo,hi)。
# bond_shift_*（2026-09-07 迭代 20）：压合微滑移随机项 3σ（对照参考项目
# bond_shift 150nm/1.5µrad 语义）——加压/保压瞬间 die 相对 site 的不可
# 再补偿滑移，bond_mark 时按 die 逐颗高斯采样落进 die["resid"] 与
# truth 的 process_shift 分量；默认 0=工艺项不建模。
BOND_FIELDS = {
    "head_z_contact_mm": ("num", -190.0, 0.0),    # BS_Arm_Z 裸接触位
    "head_z_search_mm": ("num", -190.0, 0.0),     # 慢速寻力段起点（裸值）
    # 接触确认容差（2026-09-04 迭代 13）：force_search 实际接触位 vs 期望
    # 有效接触位（裸值+晶圆厚+芯片厚）的允许偏差，超差判"接触位异常
    # （厚度/机械常数不符）"
    "contact_verify_tol_um": ("num", 0.0, 1000.0),
    "bond_shift_xy_um": ("num", 0.0, 100.0),      # 压合微滑移 XY 3σ（µm）
    "bond_shift_rz_urad": ("num", 0.0, 1000.0),   # 压合微滑移 Rz 3σ（µrad）
}

# stage_geo 组（运动台几何机械常数，迭代 36 起）：as/ws_y_rail_dir_urad=
# 对准台/硅片台 Y 导轨方向角真值（注入旋钮，error_kinematics 六自由度
# 合成 FK 消费——Y 链元平移贡献 y·(sinφ,cosφ)，X 基准轴不注入，φ=0
# 逐字节恒等）；as/ws_nonortho_urad=非正交角测校结果（as/ws_stage_
# nonortho 测校 routine apply 写回，apply 不动 y_rail_dir——那是真值
# 旋钮不是测校结果）。单位 µrad，±5000。全部 ("num",lo,hi)。
STAGE_GEO_FIELDS = {
    "as_y_rail_dir_urad": ("num", -5000.0, 5000.0),
    "ws_y_rail_dir_urad": ("num", -5000.0, 5000.0),
    "as_nonortho_urad": ("num", -5000.0, 5000.0),
    "ws_nonortho_urad": ("num", -5000.0, 5000.0),
}

# bs_rxry_axis 组（键合头调平轴旋转轴线标定，迭代 49 起）：BS_CAL_Rx/Ry
# 旋转轴线的空间表示——轴上一点（世界 mm）+ 方向向量（单位向量，量测由
# bs_rxry_axis 测校 routine 给出，apply 写回本组）。默认值与
# error_kinematics.json bond_head 链 BS_CAL_Rx/Ry 的 R 链元真值对齐
# （center=[-42.5,7.0,1215.25]，Rx dir=[1,0,0]、Ry dir=[0,1,0]）。
# 全部 "xyz" 三维向量校验（同 wafer_ega.ws_ega_xyz_mm 分支）。
BS_RXRY_AXIS_FIELDS = {
    "rx_axis_point_mm": "xyz",          # Rx 轴线上一点（世界 mm）
    "rx_axis_dir": "xyz",               # Rx 轴线方向（单位向量）
    "ry_axis_point_mm": "xyz",          # Ry 轴线上一点（世界 mm）
    "ry_axis_dir": "xyz",               # Ry 轴线方向（单位向量）
}

# sensor_calib 组（传感链标定残差·认知侧补偿，迭代 49 起）：温度/力读数
# 的出厂标定常数——发布读数 = (raw − offset_cal)/gain_cal，raw=真值×
# (1+gain_err)+offset_err+noise（gain_err/offset_err=recipe sim_sensors
# 注入真值旋钮）。gain=1.0/offset=0 = 标定理想（默认）。head_*↔
# PulseHeat_Up（键合头脉冲加热器）、wafer_*↔Heat_Dw（硅片台加热盘）、
# force_*↔键合力传感器（M_BS.oActualForce）。bond_temp_calib/
# bond_force_calib 测校 routine apply 写回本组。全部 ("num",lo,hi)。
SENSOR_CALIB_FIELDS = {
    "head_temp_gain": ("num", 0.5, 1.5),        # 头温传感器增益标定
    "head_temp_offset_c": ("num", -50.0, 50.0),  # 头温传感器零偏标定 ℃
    "wafer_temp_gain": ("num", 0.5, 1.5),        # 硅片台温传感器增益标定
    "wafer_temp_offset_c": ("num", -50.0, 50.0),  # 硅片台温零偏标定 ℃
    "force_gain": ("num", 0.5, 1.5),             # 键合力传感器增益标定
    "force_offset_n": ("num", -50.0, 50.0),      # 键合力传感器零偏标定 N
}
# Y 导轨方向角消费的轴映射（error_kinematics 用：轴名 -> stage_geo 键）
STAGE_GEO_RAIL_KEYS = {"WS_LS_Y": "ws_y_rail_dir_urad",
                       "AS_Y": "as_y_rail_dir_urad"}

# vision_marks 组（视觉对准标记模板机械常数，2026-09-04 迭代 12 起）：
# 每次触发相机拍照必须依据本组模板名识别标记（flow_engine 全部拍照点按
# 用途取本组模板名传给 vision_service；FAST/REAL 两模式均做模板存在性
# 校验，模板未建立=不能识别，模拟真机未建模板行为）。值为
# vision_templates.json 中已建模板名。
VISION_MARKS_FIELDS = (
    "tpl_wafer_site",      # 下相机认晶圆 site 标记（ega_measure / fl 晶圆
                           # 侧 align_first / fa_measure 晶圆侧）
    "tpl_coarse_die",      # 粗对准相机认翻转吸盘上 die 背面（coarse_offset）
    "tpl_die_head",        # 上相机认键合头上 die 角标（fa_measure die 侧 /
                           # fl die 侧 align_first）
    "tpl_head_chuck",      # 上相机认键合头吸盘标记（as_head_center 测校预留）
    "tpl_post_inspect",    # 下相机键合后检测认 die 角标（pi_measure）
)
# 模板存在性校验的事实源（vision_service 模板库索引）
TEMPLATES_JSON = os.path.join(ROOT, "vision_templates.json")


def _load_template_names():
    """读 vision_templates.json 模板名集合（文件缺失/损坏返回 None=不做
    存在性校验，由视觉服务启动路径自身报错）。"""
    try:
        with open(TEMPLATES_JSON, encoding="utf-8") as f:
            return set((json.load(f).get("templates") or {}).keys())
    except (OSError, ValueError):
        return None


def _check_vision_marks(data, errors, known_templates=None):
    g = data.get("vision_marks")
    if g is None:
        return
    if not isinstance(g, dict):
        errors.append("vision_marks 须为对象")
        return
    for key in g:
        if key.startswith("_"):
            continue
        if key not in VISION_MARKS_FIELDS:
            errors.append("vision_marks.%s 未知字段（白名单 %s）"
                          % (key, VISION_MARKS_FIELDS))
    for key in VISION_MARKS_FIELDS:
        if key not in g:
            errors.append("vision_marks.%s 缺失" % key)
            continue
        v = g[key]
        if not isinstance(v, str) or not v.strip():
            errors.append("vision_marks.%s 须为非空模板名字符串: %r"
                          % (key, v))
        elif known_templates is not None and v not in known_templates:
            errors.append("vision_marks.%s 模板未建立: %r"
                          "（vision_templates.json 无此模板，先在视觉页建模板）"
                          % (key, v))


def _check_num_group(data, errors, name, fields):
    """纯数值组通用校验（bond/pid）：未知字段 + 缺失 + ("num",lo,hi) 区间。"""
    g = data.get(name)
    if g is None:
        return
    if not isinstance(g, dict):
        errors.append("%s 须为对象" % name)
        return
    for key in g:
        if key.startswith("_"):
            continue
        if key not in fields:
            errors.append("%s.%s 未知字段（白名单 %s）"
                          % (name, key, tuple(fields)))
    for key, spec in fields.items():
        if key not in g:
            errors.append("%s.%s 缺失" % (name, key))
            continue
        _kind, lo, hi = spec
        v = g[key]
        if not _is_finite_num(v) or not (lo <= v <= hi):
            errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                          % (name, key, lo, hi, v))


class LoadError(ValueError):
    """机器常数文件加载/校验失败（启动拒绝拉起）。"""


def validate(data, known_templates=None):
    """全量 schema 校验 -> 错误明细列表（空=合法）。
    known_templates：vision_templates.json 模板名集合（None=不做模板
    存在性校验）。"""
    errors = []
    if not isinstance(data, dict):
        return ["顶层须为对象"]
    for key in data:
        if key.startswith("_"):
            continue
        if key not in GROUPS:
            errors.append("未知分组 %r（当前只开放 %s）" % (key, GROUPS))
    cams = data.get("cameras")
    if not isinstance(cams, dict) or not cams:
        errors.append("cameras 组必填且非空")
        return errors
    for cid, c in cams.items():
        pre = "cameras.%s" % cid
        if not isinstance(c, dict):
            errors.append("%s 须为对象" % pre)
            continue
        for key in c:
            if key.startswith("_"):
                continue
            if key not in CAMERA_FIELDS:
                errors.append("%s.%s 未知字段（内参白名单 %s）"
                              % (pre, key, CAMERA_FIELDS))
        for key in CAMERA_FIELDS:
            if key not in c:
                errors.append("%s.%s 缺失" % (pre, key))
        for key in ("pixel_um", "mag", "wd_mm", "dof_um", "rot_mrad"):
            if key not in c:
                continue
            v = c[key]
            if not isinstance(v, (int, float)) or isinstance(v, bool) \
                    or not math.isfinite(v):
                errors.append("%s.%s 须为有限数: %r" % (pre, key, v))
                continue
            lo, hi = RANGES[key]
            if not (lo <= v <= hi):
                errors.append("%s.%s 须在 [%s,%s]: %r" % (pre, key, lo, hi, v))
        for key in ("dir_x", "dir_y"):
            if key in c and c[key] not in (-1.0, 1.0, -1, 1):
                errors.append("%s.%s 只能是 ±1: %r" % (pre, key, c[key]))
        if "swap_xy" in c and not isinstance(c["swap_xy"], bool):
            errors.append("%s.swap_xy 须为布尔: %r" % (pre, c["swap_xy"]))
        for key in ("px_u", "px_v"):
            if key not in c:
                continue
            v = c[key]
            if not isinstance(v, int) or isinstance(v, bool) \
                    or not (PX_RANGE[0] <= v <= PX_RANGE[1]):
                errors.append("%s.%s 须为 [%d,%d] 整数: %r"
                              % (pre, key, PX_RANGE[0], PX_RANGE[1], v))
    fp = data.get("flip_pick")
    if fp is not None:
        pre = "flip_pick"
        if not isinstance(fp, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in fp:
                if key.startswith("_"):
                    continue
                if key not in FLIP_PICK_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(FLIP_PICK_FIELDS)))
            for key, spec in FLIP_PICK_FIELDS.items():
                if key not in fp:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                v = fp[key]
                if spec == "xy":
                    if not (isinstance(v, (list, tuple)) and len(v) == 2
                            and all(isinstance(x, (int, float))
                                    and not isinstance(x, bool)
                                    and math.isfinite(x) for x in v)):
                        errors.append("%s.%s 须为二维有限数向量: %r"
                                      % (pre, key, v))
                else:
                    _kind, lo, hi = spec
                    if not isinstance(v, (int, float)) or isinstance(v, bool) \
                            or not math.isfinite(v) or not (lo <= v <= hi):
                        errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                                      % (pre, key, lo, hi, v))
    fh = data.get("flip_handoff")
    if fh is not None:
        pre = "flip_handoff"
        if not isinstance(fh, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in fh:
                if key.startswith("_"):
                    continue
                if key not in FLIP_HANDOFF_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(FLIP_HANDOFF_FIELDS)))
            for key, spec in FLIP_HANDOFF_FIELDS.items():
                if key not in fh:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                v = fh[key]
                if spec == "xy":
                    if not (isinstance(v, (list, tuple)) and len(v) == 2
                            and all(isinstance(x, (int, float))
                                    and not isinstance(x, bool)
                                    and math.isfinite(x) for x in v)):
                        errors.append("%s.%s 须为二维有限数向量: %r"
                                      % (pre, key, v))
                else:
                    _kind, lo, hi = spec
                    if not isinstance(v, (int, float)) or isinstance(v, bool) \
                            or not math.isfinite(v) or not (lo <= v <= hi):
                        errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                                      % (pre, key, lo, hi, v))
    we = data.get("wafer_ega")
    if we is not None:
        pre = "wafer_ega"
        if not isinstance(we, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in we:
                if key.startswith("_"):
                    continue
                if key not in WAFER_EGA_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(WAFER_EGA_FIELDS)))
            for key, spec in WAFER_EGA_FIELDS.items():
                if key not in we:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                v = we[key]
                if spec == "xy":
                    _check_vec(errors, pre, key, v, 2)
                elif spec == "xyz":
                    _check_vec(errors, pre, key, v, 3)
                else:
                    _kind, lo, hi = spec
                    if _kind == "int":
                        if not isinstance(v, int) or isinstance(v, bool) \
                                or not (lo <= v <= hi):
                            errors.append("%s.%s 须为 [%s,%s] 整数: %r"
                                          % (pre, key, lo, hi, v))
                    else:
                        if not _is_finite_num(v) or not (lo <= v <= hi):
                            errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                                          % (pre, key, lo, hi, v))
    fl = data.get("focus_level")
    if fl is not None:
        pre = "focus_level"
        if not isinstance(fl, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in fl:
                if key.startswith("_"):
                    continue
                if key not in FOCUS_LEVEL_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(FOCUS_LEVEL_FIELDS)))
            for key, spec in FOCUS_LEVEL_FIELDS.items():
                if key not in fl:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                v = fl[key]
                if spec == "xy":
                    _check_vec(errors, pre, key, v, 2)
                    continue
                _kind, lo, hi = spec
                if not _is_finite_num(v) or not (lo <= v <= hi):
                    errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                                  % (pre, key, lo, hi, v))
    fa = data.get("fine_align")
    if fa is not None:
        pre = "fine_align"
        if not isinstance(fa, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in fa:
                if key.startswith("_"):
                    continue
                if key not in FINE_ALIGN_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(FINE_ALIGN_FIELDS)))
            for key, spec in FINE_ALIGN_FIELDS.items():
                if key not in fa:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                v = fa[key]
                if spec == "xy":
                    _check_vec(errors, pre, key, v, 2)
                    continue
                _kind, lo, hi = spec
                if not _is_finite_num(v) or not (lo <= v <= hi):
                    errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                                  % (pre, key, lo, hi, v))
            if "rz_sign" in fa and fa["rz_sign"] not in (-1, 1, -1.0, 1.0):
                errors.append("fine_align.rz_sign 只能是 ±1: %r"
                              % (fa["rz_sign"],))
    _check_num_group(data, errors, "bond", BOND_FIELDS)
    # stage_geo：纯数值组，校验口径与 wafer_ega 一致（未知字段/缺失/
    # ("num",lo,hi) 区间），复用通用数值组校验
    _check_num_group(data, errors, "stage_geo", STAGE_GEO_FIELDS)
    # sensor_calib：纯数值组（迭代 49，传感链标定常数）
    _check_num_group(data, errors, "sensor_calib", SENSOR_CALIB_FIELDS)
    # bs_rxry_axis：全 "xyz" 向量组（迭代 49，照 wafer_ega 向量分支）
    br = data.get("bs_rxry_axis")
    if br is not None:
        pre = "bs_rxry_axis"
        if not isinstance(br, dict):
            errors.append("%s 须为对象" % pre)
        else:
            for key in br:
                if key.startswith("_"):
                    continue
                if key not in BS_RXRY_AXIS_FIELDS:
                    errors.append("%s.%s 未知字段（白名单 %s）"
                                  % (pre, key, tuple(BS_RXRY_AXIS_FIELDS)))
            for key in BS_RXRY_AXIS_FIELDS:
                if key not in br:
                    errors.append("%s.%s 缺失" % (pre, key))
                    continue
                _check_vec(errors, pre, key, br[key], 3)
    _check_vision_marks(data, errors, known_templates)
    return errors


def _deep_merge(dst, patch):
    """递归合并（只合并对象；数组/标量整体替换）。"""
    for k, v in patch.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v


class MachineConstants(object):
    """机器常数注册表：线程安全读取 + 校验更新 + 原子落盘。"""

    def __init__(self, path=CONSTANTS_JSON):
        self._path = path
        self._lock = threading.RLock()
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        # 2026-08-27 迁移：confocal_sigma_um 由本组移到 recipe sim_sensors
        # （噪声真值≠可标定设备认知）——旧文件含该键时静默剔除再校验，
        # 防老文件启动被拒；下次 PUT/保存落盘即自然消除
        fl = data.get("focus_level")
        if isinstance(fl, dict):
            fl.pop("confocal_sigma_um", None)
        # 2026-08-31 迭代 9 迁移兜底：pid 段与 bond 段 6 硬件键/2 噪声键已
        # 迁往 tuning.json / recipe sim_sensors（正常由 tuning.migrate_legacy
        # 在启动时搬移并落盘）——旧文件含这些键时静默剔除再校验，防老文件
        # 启动被拒；下次 PUT/保存落盘即自然消除
        data.pop("pid", None)
        bd = data.get("bond")
        if isinstance(bd, dict):
            for k in ("force_spring_k_n_per_mm", "ambient_C",
                      "head_heater_max_C", "pulse_heater_rate_cps",
                      "wafer_heater_rate_cps", "wafer_heater_max_C",
                      "temp_noise_C", "force_noise_N"):
                bd.pop(k, None)
        errors = validate(data, _load_template_names())
        if errors:
            raise LoadError("machine_constants.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._d = data

    def get(self):
        """全量快照（深拷贝）。"""
        with self._lock:
            return json.loads(json.dumps(self._d))

    def camera(self, cam_id):
        with self._lock:
            c = self._d["cameras"].get(cam_id)
            return dict(c) if c else None

    def camera_ids(self):
        with self._lock:
            return sorted(self._d["cameras"])

    def update(self, patch):
        """部分字段合并 → 全量校验 → 原子写盘 + 热替换内存。
        校验失败抛 ValueError（消息含逐条 errors），原常数不污染。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            merged = json.loads(json.dumps(self._d))
            _deep_merge(merged, patch)
            errors = validate(merged, _load_template_names())
            if errors:
                raise ValueError("机器常数校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            tmp = self._path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(merged, f, ensure_ascii=False, indent=1)
                f.write("\n")
            os.replace(tmp, self._path)
            self._d = merged

    def public(self, px_lookup=None):
        """GET 响应：常数全量分组 + cameras 每台派生只读字段。
        px_lookup: 旧兼容参数（px_u/px_v 2026-08-24 已迁入本文件，派生 fov
        直接用存储值；该参数忽略）。
        派生：k_um_px = pixel_um/mag（= k_mm×1000）、fov_x/y_mm = px×k_mm。"""
        with self._lock:
            const = {g: json.loads(json.dumps(v))
                     for g, v in self._d.items() if not g.startswith("_")}
        derived = {}
        for cid, c in const["cameras"].items():
            k_um = float(c["pixel_um"]) / float(c["mag"])
            ent = {"k_um_px": round(k_um, 6)}
            ent["fov_x_mm"] = round(int(c["px_u"]) * k_um / 1000.0, 6)
            ent["fov_y_mm"] = round(int(c["px_v"]) * k_um / 1000.0, 6)
            derived[cid] = ent
        return {"ok": True,
                "groups": [g for g in self._d if not g.startswith("_")],
                "constants": const,
                "derived": {"cameras": derived}}
