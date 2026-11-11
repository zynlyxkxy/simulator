# -*- coding: utf-8 -*-
"""axes_config.py — 21 根轴的静态配置（单位与前端一致：直线 mm、旋转 deg）。

顺序固定为《前后端数据协议_V1.md》§3 的 idx 顺序，二进制帧按此排列。

【旁路副本】light_test/backend —— 本文件按细化前端 1928_front_light 的 19 根轴
量程收紧，语义轴名与帧序保持 21 不变（3 根 E_PIN 在模型上合一，由
light_test/联机注入_light.js 的 MAP 表投影）。量程来源：light_test/轴标定_light.json
与 light_test/几何实测_*.json。主线 backend/ 不受影响。
"""

# (仿真轴名, 软限位lo, 软限位hi, 最大速度, 单位, 控制器, 默认回零方式, 前端轴id)
# 全线单位：直线 mm、旋转 rad（旋转速度 rad/s）。
# 限位/速度对齐细化前端 1928_front_light 的 axes 表（2026-10-28）：
# WS_X ±209、WS_Y ±150、AS_X -25~75、AS_Y -25~115、AS_Z -1~5、
# BS_Z -150~0、FA_X -1242.35~0、FA_Z 0~20（模型 -20~0，注入侧取负）、
# FA_Rx -π~0（模型 0~180，注入侧取负）、E_PIN 0~13、ES_* 与旧模型一致。
AXES = [
    ("WS_LS_X",    -220.0,  220.0, 600.0, "mm",  "ACS", 18, "axis_WS_X"),             # idx 0
    ("WS_LS_Y",    -220.0,  220.0, 600.0, "mm",  "ACS", 18, "axis_WS_Y"),             # idx 1
    # 注：WS_Rz 行程原 deg 表为 ±1°=±0.0174533 rad；轴记录表为 ±0.005 rad，此处取协议口径
    # 细化模型的物理行程是 ±7.5°，仿真仍按 ±1° 口径（差距报告已记，放开属独立改动）
    ("WS_Rz",      -0.0174533, 0.0174533, 1.5707963, "rad", "ACS", 18, "axis_WS_RZ"),  # idx 2
    ("E_EPIN_Z_1",    0.0,   13.0,  25.0, "mm",  "PLC", 17, "axis_WS_EPIN_Z"),        # idx 3
    # 细化模型只有一根 WS_EPin；2/3 号槽不建独立前端 id（占位避免 ID_TO_NAME 撞键，
    # 投影由联机注入_light.js 的 MAP 表三合一处理）
    ("E_EPIN_Z_2",    0.0,   13.0,  25.0, "mm",  "PLC", 17, "axis_WS_EPIN_Z_2"),      # idx 4
    ("E_EPIN_Z_3",    0.0,   13.0,  25.0, "mm",  "PLC", 17, "axis_WS_EPIN_Z_3"),      # idx 5
    ("BS_Arm_Z",   -190.0,    0.0, 200.0, "mm",  "PLC", 18, "axis_BS_Z"),             # idx 6
    ("BS_CAL_Rx",  -0.00174533, 0.00174533, 1.5707963, "rad", "PLC", 18, "axis_BS_Rx"),  # idx 7
    ("BS_CAL_Ry",  -0.00174533, 0.00174533, 1.5707963, "rad", "PLC", 18, "axis_BS_Ry"),  # idx 8
    ("AS_X",        -30.0,   80.0, 600.0, "mm",  "ACS", 18, "axis_AS_X"),             # idx 9
    # 注：AS_Y 下限 -30 对齐前端模型原行程（TwinDOM 轴语义 authored−current：
    # AS_Y current=55.4，真实零位 = authored−55.4；迭代 3 把 authored 误当
    # 零位推出 -60，已回退）。上相机认键合头吸盘中心需 AS_Y=+6.0，在区间内。
    ("AS_Y",        -30.0,  120.0, 500.0, "mm",  "ACS", 18, "axis_AS_Y"),             # idx 10
    ("AS_Z",         -2.0,    6.0, 500.0, "mm",  "ACS", 18, "axis_AS_Z"),             # idx 11
    ("AS_Rx",      -0.00174533, 0.00174533, 1.5707963, "rad", "ACS", 18, "axis_AS_Rx"),  # idx 12
    ("AS_Ry",      -0.00174533, 0.00174533, 1.5707963, "rad", "ACS", 18, "axis_AS_Ry"),  # idx 13
    # FA_X：0 = 取片位，-1242.35 = 交接位（键合头在 X=-42.5，取片点在 X=1199.85）。
    # 细化模型源文件把方向接反（0~+1235），联机注入在 boot 时把下限扩到 -1243，
    # 本副本按实测行程 -1242.35（原 -1250 亦可用，此处取实测值）。
    ("FA_X",      -1242.35,    0.0, 600.0, "mm",  "ACS", 18, "axis_ISS_FA_X"),        # idx 14
    ("FA_Z",        -40.0,   40.0, 600.0, "mm",  "PLC", 18, "axis_ISS_FA_Z"),         # idx 15
    ("FA_Rx",      -3.1415927, 3.1415927, 1.5707963, "rad", "PLC", 18, "axis_ISS_FA_RX"),   # idx 16
    ("CH_Rz",      -3.1415927, 3.1415927, 1.5707963, "rad", "PLC", 18, "axis_ISS_FA_RZ"),   # idx 17
    ("ES_X",       -285.0,   85.0, 600.0, "mm",  "PLC", 17, "axis_filmstage_x_0001"),  # idx 18
    ("ES_Y",       -175.0,  195.0, 600.0, "mm",  "PLC", 17, "axis_filmstage_y_0001"),  # idx 19
    ("ES_PunchZ",     0.0,    5.0,  10.0, "mm",  "ACS", 17, "axis_filmpin_z_0001"),    # idx 20
]

AXIS_NAMES = [a[0] for a in AXES]                 # idx -> 仿真轴名
FRONTEND_IDS = [a[7] for a in AXES]               # idx -> 前端轴 id
ID_TO_NAME = dict(zip(FRONTEND_IDS, AXIS_NAMES))
NAME_TO_IDX = {n: i for i, n in enumerate(AXIS_NAMES)}

# 三轴同步别名：E_EPIN_Z 一次驱动三根顶针轴
ALIASES = {"E_EPIN_Z": ["E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"]}

# ---------------- 真值：ACS 轴号 / 回零 buffer 号（源自同事 motion/config1~4，2026-08-19） ----------------
# 配置实证（部分接口信息/motion/motion/）：
#   config1/acs/WS_LS.json   X=0 / Y=2 / Rz=4（双驱占偶数轴号对 0+1/2+3/4+5），home buffer=18
#   config3/acs/AS.json      X=6 / Y=7 / Z=8，home buffer=18
#   config3/acs/PunchZ.json  Z=10，home buffer=17
#   config4/acs/FA_X.json    X=9，home buffer=18
# 口径说明：仿真轴名 AS_X/AS_Y 对应前端"对准台X/Y承接板"，按功能名对齐 acs 6/7；
# 注意点位表 D2 命名交叉（点位表里"对准模块X轴"的 Cordis 名是 AS_Y）——此处按
# motion 配置的功能名 X/Y/Z 落轴号，待真机复核（见 docs/接口规范_V1.md §7 Q5）。
# None = 推不出：PLC 轴（不过 ACS）或点位表未列轴号（AS_Rx/AS_Ry，D4 待确认）。
ACS_INDEX = {
    "WS_LS_X": 0, "WS_LS_Y": 2, "WS_Rz": 4,
    "E_EPIN_Z_1": None, "E_EPIN_Z_2": None, "E_EPIN_Z_3": None,   # PLC 轴
    "BS_Arm_Z": None, "BS_CAL_Rx": None, "BS_CAL_Ry": None,       # PLC 轴
    "AS_X": 6, "AS_Y": 7, "AS_Z": 8,
    "AS_Rx": None, "AS_Ry": None,   # 点位表未列轴号（D4），待确认
    "FA_X": 9,
    "FA_Z": None, "FA_Rx": None, "CH_Rz": None,                   # PLC 轴
    "ES_X": None, "ES_Y": None,     # 外包模块高层命令，非直接轴控（D6）
    "ES_PunchZ": 10,
}
# 真机回零 = 控制器 buffer 程序（同事确认 WS_LS 用 buffer 18）。
# 注意：仿真内 engine.BUFFERS 的 1~3 是自定义动作序列，与真机 buffer 号无对应，
# 不重编号（破坏面大）；此处仅记录真值供将来 W3 ACS 侧接入对齐。
TRUTH_HOME_BUFFER = {"WS_LS": 18, "AS": 18, "FA_X": 18, "ES_PunchZ": 17}

# 动力学默认值（表里没有给加速度/jerk，取速度的比例值，观感与真机相近）
# 单位随 vmax：直线轴 mm/s²、mm/s³；旋转轴 rad/s²、rad/s³（随 rad 化自动重算）
ACCEL_FACTOR = 10.0   # amax = 10 × vmax
JERK_FACTOR = 100.0   # jerk = 10 × amax = 100 × vmax
HOME_SPEED_FACTOR = 0.1

# ---------------- 回零（CiA402 17/18 式序列） ----------------
# 点位表口径：ACS 轴与 PLC 的 Arm_Z/CAL_Rx/CAL_Ry/FA_Rx/FA_Z/CH_Rz 为 18（正向
# 找正限位开关），E_EPIN×3 与 ES_PunchZ 为 17（负向找负限位开关）——见 AXES 第 7 列。
HOME_SPEED_FAST_FACTOR = 0.25    # 快速接近段 = 25% × vmax
HOME_SPEED_SLOW_FACTOR = 0.05    # 退出/锁定段 = 5% × vmax
LIMIT_SWITCH_OVERTRAVEL = 0.02   # 硬限位开关位置 = 软限位外扩行程的 2%（可配置）
LIMIT_SWITCH_HYSTERESIS = 0.005  # 开关释放迟滞 = 0.5% 行程
HOME_FAULT_TRAVEL_FACTOR = 2.0   # 累计行程超 2×行程未触发开关 → homing_fault

# ---------------- 回零类型（按轴分派） ----------------
# acs_limit      普通轴（默认）：CiA402 17/18 式软限位开关序列
# gantry_dual    双驱龙门（点位表 ACS 轴号 0+1/2+3/4+5）：同 17/18 序列，
#                完成后给误差系统注入固定小偏摆（直线 300nm、Rz 2µrad）
#                模拟双驱残余对准误差
# plc_sync_trio  顶针三轴 PLC 同步回零：leader（E_EPIN_Z_1）跑 17 状态机，
#                三轴同位移、同时锁定；联动 DI iZeroEpin_Z
# zero_sensor    校准 Rx/Ry 零位传感器回零：慢速朝 0 搜索零位窗口即锁定；
#                联动 DI 零位/正负限位信号
# vendor_es      蓝膜台 X/Y 外包模块（占位）：恒速 100mm/s 直接回 0
HOME_TYPE_GANTRY_DUAL = {"WS_LS_X", "WS_LS_Y", "WS_Rz"}
HOME_TYPE_PLC_SYNC_TRIO = {"E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"}
HOME_TYPE_ZERO_SENSOR = {"BS_CAL_Rx", "BS_CAL_Ry"}
HOME_TYPE_VENDOR_ES = {"ES_X", "ES_Y"}
EPIN_TRIO_LEADER = "E_EPIN_Z_1"

# 龙门双驱回零后的固定残余偏摆（原生单位：mm / rad）
GANTRY_RESIDUAL_LINEAR = 300e-6    # 300 nm
GANTRY_RESIDUAL_ROTARY = 2e-6      # 2 µrad
# 零位传感器窗口（rad）：|pos| ≤ 窗口认为零位信号触发
ZERO_SENSOR_WINDOW = 2e-4
# 顶针零位 DI 判定阈值（mm）：三轴 |pos| 均 ≤ 阈值时 iZeroEpin_Z 置真
EPIN_ZERO_TOL = 0.05
# 蓝膜台外包模块回零恒速（mm/s，占位实现）
VENDOR_ES_HOME_SPEED = 100.0

# 回零联动 DI 点位（点位表 Cordis 路径）
DI_EPIN_ZERO = "SystemMachine.SS_Bonding.M_WS.iZeroEpin_Z"
DI_CAL_ZERO = {"BS_CAL_Rx": "SystemMachine.SS_Bonding.M_BS.iRxZeroSignal",
               "BS_CAL_Ry": "SystemMachine.SS_Bonding.M_BS.iRyZeroSignal"}
DI_CAL_POS = {"BS_CAL_Rx": "SystemMachine.SS_Bonding.M_BS.iRxPosSignal",
              "BS_CAL_Ry": "SystemMachine.SS_Bonding.M_BS.iRyPosSignal"}
DI_CAL_NEG = {"BS_CAL_Rx": "SystemMachine.SS_Bonding.M_BS.iRxNegSignal",
              "BS_CAL_Ry": "SystemMachine.SS_Bonding.M_BS.iRyNegSignal"}


def home_type(name):
    """轴 -> 回零类型（acs_limit/gantry_dual/plc_sync_trio/zero_sensor/vendor_es）。"""
    if name in HOME_TYPE_GANTRY_DUAL:
        return "gantry_dual"
    if name in HOME_TYPE_PLC_SYNC_TRIO:
        return "plc_sync_trio"
    if name in HOME_TYPE_ZERO_SENSOR:
        return "zero_sensor"
    if name in HOME_TYPE_VENDOR_ES:
        return "vendor_es"
    return "acs_limit"


def home_defaults(name, lo, hi, vmax):
    """每轴回零默认参数（均可被 config_overrides 覆盖）。"""
    travel = hi - lo
    method = {row[0]: row[6] for row in AXES}[name]
    s = 1.0 if method == 18 else -1.0
    return {
        "method": method,
        "fast": vmax * HOME_SPEED_FAST_FACTOR,
        "slow": vmax * HOME_SPEED_SLOW_FACTOR,
        # 开关位置：18→软上限外扩 2% 行程；17→软下限外扩
        "switch": (hi + LIMIT_SWITCH_OVERTRAVEL * travel) if s > 0
                  else (lo - LIMIT_SWITCH_OVERTRAVEL * travel),
        # 触限位后锁定的位置值：默认 = 限位处机械零点（软限位）
        "offset": hi if s > 0 else lo,
        "hysteresis": abs(travel) * LIMIT_SWITCH_HYSTERESIS,
        "fault_travel": abs(travel) * HOME_FAULT_TRAVEL_FACTOR,
    }

# ---------------- 伺服环动态默认值（一阶滞后 + 到位窗口整定） ----------------
# 合成经验值，待真机环参数标定（ACS 的 PE/到位窗口、驱动器时间常数均未拿到实测）。
# 模型：x_actual 以一阶滞后跟随名义值 pos（tau 为等效位置环时间常数），
# pe = pos - x_actual 为跟随误差；轨迹跑完后 |pe| 须在 settle_window 内持续
# settle_time 才判 in_position（对应 ACS 的 PE/到位窗口语义）。
SERVO_TAU_ACS_LINEAR = 0.005       # ACS 精密直轴 tau（s）
SERVO_TAU_ROTARY = 0.008           # 旋转轴 tau（s，ACS/PLC 同档）
SERVO_TAU_PLC_LINEAR = 0.015       # PLC 慢速直轴 tau（s）
SERVO_SETTLE_WINDOW_LINEAR = 0.002  # 到位窗口：直线 mm
SERVO_SETTLE_WINDOW_ROTARY = 20e-6  # 到位窗口：旋转 rad
SERVO_SETTLE_TIME = 0.05           # 到位需持续的时间（s）


def servo_defaults(name):
    """每轴伺服默认值：{'tau','settle_window','settle_time'}（原生单位 mm/rad/s）。"""
    row = {r[0]: r for r in AXES}[name]
    unit, ctl = row[4], row[5]
    if axis_is_rotary(name):
        tau, win = SERVO_TAU_ROTARY, SERVO_SETTLE_WINDOW_ROTARY
    else:
        tau = SERVO_TAU_ACS_LINEAR if ctl == "ACS" else SERVO_TAU_PLC_LINEAR
        win = SERVO_SETTLE_WINDOW_LINEAR
    return {"tau": tau, "settle_window": win, "settle_time": SERVO_SETTLE_TIME}


# ---------------- 误差系统分级 ----------------
# 精密轴（ACS 精密 + 长行程 FA_X）：平移误差 0.2~1µm、旋转 1~5µrad
PRECISION_AXES = {"WS_LS_X", "WS_LS_Y", "WS_Rz", "AS_X", "AS_Y", "AS_Z",
                  "AS_Rx", "AS_Ry", "FA_X"}
# 其余为辅助轴（E_EPIN_*/BS_*/FA_Z/FA_Rx/CH_Rz/ES_*）：平移 1~5µm、旋转 10~30µrad
AUX_AXES = [n for n in AXIS_NAMES if n not in PRECISION_AXES]

# 每轴重复定位精度 3σ（原生单位：直线 mm / 旋转 rad）
# 精密轴 3σ=500nm 级、旋转 2µrad；辅助轴 3µm 级、旋转 10µrad；
# 例外（2026-08-28 迭代 7，用户指定）：WS_LS_X/WS_LS_Y 3σ=40nm；
# 例外（2026-09-04 迭代 12，用户指定）：键合臂调平 BS_CAL_Rx/BS_CAL_Ry 3σ=6µrad；
# 例外（2026-09-04 迭代 13，用户指定）：对准台 AS_X/AS_Y 3σ=40nm
def _repeat_3sigma():
    out = {}
    for name, lo, hi, vmax, unit, ctl, home, _fid in AXES:
        if name in PRECISION_AXES:
            out[name] = 0.0005 if unit == "mm" else 2e-6
        else:
            out[name] = 0.003 if unit == "mm" else 1e-5
    out["WS_LS_X"] = 0.00004
    out["WS_LS_Y"] = 0.00004
    out["AS_X"] = 0.00004
    out["AS_Y"] = 0.00004
    out["BS_CAL_Rx"] = 6e-6
    out["BS_CAL_Ry"] = 6e-6
    return out

REPEAT_3SIGMA = _repeat_3sigma()

# 重复定位精度的显示单位（API 口径）：直线 nm、旋转 µrad
REPEAT_DISPLAY_UNIT = {"mm": "nm", "rad": "urad"}
# 原生单位 -> 显示单位换算（mm->nm ×1e6；rad->µrad ×1e6）
REPEAT_TO_DISPLAY = 1e6


def axis_along_component(name):
    """沿轴分量映射：直线轴 X/Y/Z -> dx/dy/dz；旋转轴 Rx/Ry/Rz -> rx/ry/rz。"""
    for rot in ("Rx", "Ry", "Rz"):
        if name.endswith(rot):
            return rot.lower()
    if name.endswith("_X"):
        return "dx"
    if name.endswith("_Y"):
        return "dy"
    # *_Z / *_PunchZ / E_EPIN_Z_1 等 Z 向直线轴
    if name.endswith("_Z") or "_Z_" in name or name.endswith("Z"):
        return "dz"
    raise KeyError("cannot map along-axis component for %r" % name)


def axis_is_rotary(name):
    return axis_along_component(name) in ("rx", "ry", "rz")


def resolve_axis(name_or_id):
    """前端轴 id / 仿真轴名 / 别名 -> 仿真轴名列表；未知返回 None。"""
    if name_or_id in ID_TO_NAME:
        return [ID_TO_NAME[name_or_id]]
    if name_or_id in ALIASES:
        return list(ALIASES[name_or_id])
    if name_or_id in NAME_TO_IDX:
        return [name_or_id]
    return None
