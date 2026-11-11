# -*- coding: utf-8 -*-
"""engine.py — 物理引擎：固定 1kHz tick（1ms 仿真步长），21 轴共用。

- 轴运动：S 曲线（jerk 限幅）规划 + 逐 tick 积分；软限位；到位/运动中标志。
- 伺服环动态：x_actual 一阶滞后跟随名义值（tau 按轴分级），pe=跟随误差；
  轨迹跑完进入 settling，|pe| 在到位窗口内持续 settle_time 才判 in_position。
- 气路：一阶惯性（tau≈0.2s）压力/流量过渡 + 三态观测。
- 温控：PulseHeat_Up / Heat_Dw 一阶热对象（功率%×满功率速率 − 热损）
  + 速率限 + 量程上限，TwinCAT 系列型位置式 PID（runtime/pid.py，4ms
  周期）输出功率；读数 ±temp_noise_C 噪声；速率/上限/噪声读机器常数
  bond 组（缺失回退 HEATERS/AMBIENT_TEMP 硬编码）。
- 键合力：弹簧接触模型——BS_Arm_Z 低于接触位（机器常数
  bond.head_z_contact_mm）后真值力 F=k·压入量，力回路 PID 位置式输出
  压入量微调轴位；读数 ±force_noise_N 噪声写 oActualForce。
- ACS buffer 程序：顺序轴运动序列，观测 oBufferDo/oBufferNum。
- 全机模态仿真（2026-09-08 迭代 22，runtime/modal.py）：伺服环后每 tick
  解析传播 21 轴 SISO+全机 MIMO 模态（轴加速度 vel 差分激励/地板随机力/
  键合接触脉冲）；POST /api/modal 热开关，默认 OFF 零开销。
- 全局倍速 0.1~8：只改墙钟 tick 速率，tick 仿真步长恒为 1ms。
- PLC 扫描周期：点位 set 写入输入映像（pending），每 plc_scan_ms（默认 4ms）
  个 tick 由 commit() 合并进输出映像（live），外部观测以扫描率采样。
- 故障注入（W2）：每轴 enabled/fault 运行时态（掉使能=自由减速+fault，
  move/jog/buffer/home 对未使能/故障轴按联锁 409 模式拒绝）；沉默目标
  （acs/plc/mcs）吞指令、断线目标关端口拒连；均不持久化，重启复位。
- Windows 计时：timeBeginPeriod(1) + perf_counter 忙等/短睡眠混合。
"""
import ctypes
import json
import os
import random
import struct
import threading
import time
import traceback
from collections import deque

import axes_config as ac
import pid as pidmod
from trajectory import plan_move, plan_trapezoid, profile_duration

TICK_DT = 0.001          # 每 tick 仿真秒数（固定，不随倍速变）
PLC_SCAN_MS = 4          # PLC 扫描周期（ms，合成默认值，真机任务周期待确认）
KILL_DEC_FACTOR = 3.0    # 掉使能自由减速 = 3×amax（合成值：真机取驱动器
                         # KillDec 量级，本仿真未建模 KillDec 参数，故按 3×amax 合成）
FAULT_TARGETS = ("acs", "plc", "mcs")   # 沉默/断线可挂的故障注入目标
AIR_TAU = 0.2            # 气路一阶惯性时间常数（仿真秒）
AIR_POS_KPA = 600.0      # 正压目标（0.5~0.9 MPa 量程内）
AIR_POS_FLOW = 6.0       # L/min
# 负压按密封状态分两档：有工件密封=深真空；无工件开放=浅真空（抽不动）
AIR_NEG_KPA_SEALED = -80.0    # 密封负压目标（kPa）
AIR_NEG_FLOW_SEALED = 0.5     # 密封流量（L/min，几乎不漏）
AIR_NEG_KPA_OPEN = -30.0      # 开放负压目标（kPa）
AIR_NEG_FLOW_OPEN = 10.0      # 开放流量（L/min，大泄漏）
AIR_GRIP_KPA = -60.0          # grip_ok 压强阈值：低于它才算"吸牢"
AMBIENT_TEMP = 25.0        # 环境温度回退值（机器常数 bond.ambient_C 缺失时用）
# 构建标签（迭代 38）：/api/health 携带；panel 顶栏比对，旧 backend 进程
# 占用 9010 时提示"后端版本过旧"；start.bat 健康检查也以此为凭据
BUILD_TAG = "i53"

FRAME_MAGIC = 0xA1
FRAME_VERSION = 1
FRAME_FMT = "<BBHddI"
FRAME_N_AXIS = 21
FRAME_LEN = struct.calcsize(FRAME_FMT) + 4 * FRAME_N_AXIS  # 108

# 气路（来自点位表"气控箱"sheet；supports_neg=False 的只有 OpenPositive/OpenAir）
AIR_BASE = "SystemMachine.SS_ME.M_AIR."
AIR_CIRCUITS = {
    "AS_X_Cooling": False, "AS_Y_Cooling": False,
    "PulseHeatCooling_1": False, "PulseHeatCooling_2": False,
    "GlassCoolingIn": False, "GlassCoolingOut": False,
    "IonicBarPositive": False,
    "PosPressureReserve_1": False, "PosPressureReserve_2": False,
    "WT_AirfootCompPos": False,
    "WT_WaferVac": True, "WT_AirfootCompVac": True,
    "WT_BondingVac_1": True, "WT_BondingVac_2": True, "WT_BondingVac_3": True,
    "WT_TrayVac_1": True, "WT_TrayVac_2": True,
    "WT_TrayVac_3": True, "WT_TrayVac_4": True,
    "WT_ChipVac": True, "ChuckVac": True, "ChipVac": True,
    "NegReserveVac_1": True, "NegReserveVac_2": True,
    "N2Seal": False, "N2Reserve_1": False, "N2Reserve_2": False,
}

HEATER_BASE = "SystemMachine.M_Heater."
HEATER_CMD_BASE = "SystemMachine."
HEATERS = {                       # 满功率升温速率 ℃/s 回退值（机器常数
    "PulseHeat_Up": 200.0,        # bond.*_rate_cps 缺失时用；键名同时是
    "Heat_Dw": 0.7,               # plc_http 加热器路由名单）
}
HEATER_MAX_C = {                  # 量程上限回退值（bond.*_heater_max_C）
    "PulseHeat_Up": 450.0,
    "Heat_Dw": 511.4,
}

# 机器常数 bond 组回退默认值（裸引擎/常数缺失时保持旧语义兼容）。
# 2026-08-31 迭代 9：head_z_contact_mm 重定义为裸接触位（无晶圆无芯片），
# 有效接触位按处方厚度换算（effective_bond_contact_mm）；弹簧刚度/热对象
# 参数改由 tuning.json bond_hw 组供给（下位机调参），读数噪声改由
# recipe sim_sensors 供给——此处仅为兜底回退。
BOND_DEFAULTS = {
    "head_z_contact_mm": -168.93,
    "head_z_search_mm": -166.9,
    "force_spring_k_n_per_mm": 500.0,
    "ambient_C": AMBIENT_TEMP,
    "head_heater_max_C": 450.0,
    "pulse_heater_rate_cps": 200.0,
    "wafer_heater_rate_cps": 0.733,
    "wafer_heater_max_C": 511.4,
    "wafer_cool_rate_cps": 5.0,
    "temp_noise_C": 0.5,
    "force_noise_N": 1.0,
    # flip 交接接触面高度（迭代 49）：flip 吸盘裸面在 BS_Arm_Z 坐标系下的
    # z（tuning bond_hw.flip_contact_z_mm 供给；本值仅为裸引擎兜底）
    "flip_contact_z_mm": -51.43,
}
# 三回路 PID 回退默认值（tuning.json pid 组供给；整定依据见 tuning.py
# DEFAULT_TUNING pid._note）
PID_DEFAULTS = {
    "force_kp": 0.0008, "force_tn_s": 0.1, "force_tv_s": 0.0,
    "force_td_s": 0.01,
    "headtemp_kp": 3.0, "headtemp_tn_s": 1.0, "headtemp_tv_s": 0.0,
    "headtemp_td_s": 0.1,
    "wafertemp_kp": 20.0, "wafertemp_tn_s": 10.0, "wafertemp_tv_s": 0.0,
    "wafertemp_td_s": 1.0,
    "cycle_s": 0.004,
}


def effective_bond_contact_mm(bare_contact_mm, wafer_thickness_mm,
                              chip_thickness_mm):
    """有效接触位换算（2026-08-31 迭代 9，flow 侧与 engine 真值力模型共用
    本函数，防两份逻辑漂移）：裸接触位（无晶圆无芯片、吸盘表面贴陶瓷面）
    + 晶圆厚 + 芯片厚。

    符号约定（BS_Arm_Z 下行=pos 减小，真值力 F=k·(contact−pos)）：片变厚
    → 压头更早接触 → 有效接触位比裸接触位更高（pos 增大方向）。
    数值例：裸 −168.93 + 晶圆 1.0 + 芯片 0.5 = −167.43。"""
    return float(bare_contact_mm) + float(wafer_thickness_mm) \
        + float(chip_thickness_mm)

# ACS buffer 程序（只用 ACS 轴；真机里翻转交接流程就写在 buffer 里）
# 真值（motion/config1~4 cmdbuffer 字段）：回零走控制器 buffer——WS_LS/AS/FA_X
# 用 buffer 18、PunchZ 用 17（见 axes_config.TRUTH_HOME_BUFFER）；下列 1~3 是
# 仿真自定义动作序列，编号刻意不与真机对齐（避免破坏现有回归/面板引用）。
BUFFERS = {
    1: {"name": "flip_handoff", "desc": "翻转交接序列",
        # FA_X 深行程示教位取 -900：零件级沙盒下 FA_X<≈-1086 翻转链零件会撞
        # 对准相机承载台（runtime/sandboxes.json 对表 pt:* × pt:*，阈值最浅对
        # = 翻转模块Rz承载台×对准相机承载台），更深的 -1240 会被干涉检查 409 拦下
        "steps": [("FA_X", -600.0, 600.0), ("ES_PunchZ", 5.0, 10.0),
                  ("ES_PunchZ", 0.0, 10.0), ("FA_X", -900.0, 600.0),
                  ("FA_X", 0.0, 600.0)]},
    2: {"name": "align_scan", "desc": "对准台扫描",
        "steps": [("AS_X", 20.0, 500.0), ("AS_X", -20.0, 500.0),
                  ("AS_X", 0.0, 500.0), ("AS_Y", 30.0, 500.0),
                  ("AS_Y", 0.0, 500.0)]},
    3: {"name": "wafer_stage_step", "desc": "硅片台步进",
        "steps": [("WS_LS_X", 100.0, 200.0), ("WS_LS_Y", 50.0, 200.0),
                  ("WS_LS_X", 0.0, 200.0), ("WS_LS_Y", 0.0, 200.0)]},
}


class InterlockError(RuntimeError):
    """联锁拒绝（EMO 锁存 / 门开未屏蔽）——HTTP 层映射为 409。"""


# ---------------- 联锁点位（点位表 Cordis 路径） ----------------
# EMO：M_EMO 现有 4 个输入点（需求口径 5 个，第 5 个待点位表确认后补进此表）
EMO_POINTS = (
    "SystemMachine.SS_ME.M_EMO.iEFEM_EMO",
    "SystemMachine.SS_ME.M_EMO.iMachine_EMO",
    "SystemMachine.SS_ME.M_EMO.iMachine_Right_EMO",
    "SystemMachine.SS_ME.M_EMO.iPowerControl_EMO",
)
GVL_EMO_POINTS = tuple("GVL_IO." + p.rsplit(".", 1)[-1] for p in EMO_POINTS)
# 门锁：*DoorStatus*，True=门关好（语义假设，待确认）；任一门开即联锁。
# SystemMachine 与 GVL_IO 两族命名不同，是同一物理信号的两个名字：
# 每 tick 做"最后写入方生效"的镜像同步（见 _tick_interlock_locked）。
# 注意点位表笔误：SystemMachine 侧 iFrontockStatus_2 缺 L，且右门只有 _1。
DOOR_MIRROR = (
    ("SystemMachine.SS_ME.M_InterLock.iFrontDoorStatus_1",
     "GVL_IO.iFrontLock_DoorStatus_1"),
    ("SystemMachine.SS_ME.M_InterLock.iFrontDoorStatus_2",
     "GVL_IO.iFrontLock_DoorStatus_2"),
    ("SystemMachine.SS_ME.M_InterLock.iLeftDoorStatus_1",
     "GVL_IO.iLeftLock_DoorStatus_1"),
    ("SystemMachine.SS_ME.M_InterLock.iLeftDoorStatus_2",
     "GVL_IO.iLeftLock_DoorStatus_2"),
    ("SystemMachine.SS_ME.M_InterLock.iRightDoorStatus_1",
     "GVL_IO.iRightLock_DoorStatus_1"),
    ("SystemMachine.SS_ME.M_InterLock.iFrontLockStatus_1",
     "GVL_IO.iFrontLock_LockStatus_1"),
    ("SystemMachine.SS_ME.M_InterLock.iFrontockStatus_2",
     "GVL_IO.iFrontLock_LockStatus_2"),
    ("SystemMachine.SS_ME.M_InterLock.iLeftLockStatus_1",
     "GVL_IO.iLeftLock_LockStatus_1"),
    ("SystemMachine.SS_ME.M_InterLock.iLeftLockStatus_2",
     "GVL_IO.iLeftLock_LockStatus_2"),
    ("SystemMachine.SS_ME.M_InterLock.iRightLockStatus_1",
     "GVL_IO.iRightLock_LockStatus_1"),
)
DOOR_POINTS = tuple(p for pair in DOOR_MIRROR for p in pair)
# 屏蔽：任一置真即放行门锁联锁
MASK_POINTS = (
    "SystemMachine.SS_ME.M_InterLock.iInterlock_Mask",
    "SystemMachine.SS_ME.M_InterLock.oAllDoors_Mask",
    "GVL_IO.iInterlock_Mask",
    "GVL_IO.oInterLock_AllDoors_Mask",
)
# 塔灯（自动逻辑优先于手动 SetBool，见 README）
TOWER_POINTS = {
    "red": "SystemMachine.SS_ME.M_SLAM.E_SignalTower.RedLight.oLightOn",
    "yellow": "SystemMachine.SS_ME.M_SLAM.E_SignalTower.YellowLight.oLightOn",
    "green": "SystemMachine.SS_ME.M_SLAM.E_SignalTower.GreenLight.oLightOn",
}
GVL_TOWER_POINTS = {
    "red": "GVL_IO.oSignalTower_RedLight",
    "yellow": "GVL_IO.oSignalTower_YellowLight",
    "green": "GVL_IO.oSignalTower_GreenLight",
}
# /api/viz 聚合的真空回路（M_AIR 下的回路名，状态取其 oNegtiveState）
VIZ_VACUUM = ("ChuckVac", "WT_BondingVac_1", "WT_BondingVac_2",
              "WT_BondingVac_3", "WT_WaferVac", "WT_ChipVac", "ChipVac")

# ---------------- 电源分系统（点位表 M_PowerSwitch） ----------------
# 轴→分系统映射为假设（点位表只给分系统名，未给轴归属）：
# Bonding=键合分系统（SS_Bonding 全部轴：硅片台/顶针/键合臂/对准台/蓝膜台），
# CH=芯片传输（翻转+CH_Rz）；EV/PLASMA_Acid/Backup/Fan/IonBar 不带轴。
POWER_SUBSYSTEMS = ("Bonding", "CH", "EV", "PLASMA_Acid", "Backup",
                    "Fan", "IonBar")
POWER_AXIS_MAP = {
    "Bonding": ("WS_LS_X", "WS_LS_Y", "WS_Rz",
                "E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3",
                "BS_Arm_Z", "BS_CAL_Rx", "BS_CAL_Ry",
                "AS_X", "AS_Y", "AS_Z", "AS_Rx", "AS_Ry",
                "ES_X", "ES_Y", "ES_PunchZ"),
    "CH": ("FA_X", "FA_Z", "FA_Rx", "CH_Rz"),
}
POWER_BASE = "SystemMachine.SS_ME.M_PowerSwitch."

# 塔灯/蜂鸣器（E_SignalTower 下的元素名 → 观测点叶名）
TOWER_ELEMENTS = {"RedLight": "oLightOn", "YellowLight": "oLightOn",
                  "GreenLight": "oLightOn", "BlueLight": "oLightOn",
                  "Buzzer": "oBuzzOn"}
TOWER_BASE = "SystemMachine.SS_ME.M_SLAM.E_SignalTower."

# 密封数据源（workpieces 层）：吸盘被工件盖住才算密封。
# 交接瞬间吸盘贴着"对侧载体上的 die"（接触建模代价不值），故密封判据
# 取"本载体或对侧载体有工件"：ChuckVac↔flip/film 有 die，
# WT_BondingVac_*/ChipVac↔head/flip 有 die，WT_WaferVac↔晶圆在位，
# WT_ChipVac↔蓝膜还有 die；其余回路恒开放。
SEAL_CARRIERS = {
    "ChuckVac": ("flip", "film"),
    "WT_BondingVac_1": ("head", "flip"),
    "WT_BondingVac_2": ("head", "flip"),
    "WT_BondingVac_3": ("head", "flip"),
    "ChipVac": ("head", "flip"),
    "WT_ChipVac": ("film",),
}
CAPTURES_MAX = 100          # 相机拍照记录环形缓冲


class Axis(object):
    __slots__ = ("name", "lo", "hi", "vmax", "unit", "controller",
                 "home_mode", "amax", "jerk", "pos", "vel", "target",
                 "moving", "in_position", "enabled", "fault", "decel",
                 "homed",
                 "profile", "prof_i", "start", "profile_type",
                 "home_fast", "home_slow", "home_switch", "home_offset",
                 "home_state", "home_phase", "home_err", "home_traveled",
                 "home_follow",
                 "x_actual", "pe", "state", "tau", "settle_window",
                 "settle_time", "settle_acc")

    def __init__(self, row):
        (self.name, self.lo, self.hi, self.vmax, self.unit,
         self.controller, self.home_mode, _fid) = row
        self.amax = self.vmax * ac.ACCEL_FACTOR
        self.jerk = self.vmax * ac.JERK_FACTOR
        self.profile_type = "scurve"     # scurve | trapezoid
        self.pos = 0.0
        self.vel = 0.0
        self.target = 0.0
        self.moving = False
        self.in_position = True
        # 使能/故障是运行时状态（W2 故障注入）：不进 config_overrides 持久化，
        # 重启复位为"已使能、无故障"
        self.enabled = True
        self.fault = False           # 运动中掉使能减速到静止后置真（真机掉使能即报警）
        self.decel = 0.0             # >0 = 掉使能自由减速进行中（减速度幅值）
        self.homed = False
        self.profile = None
        self.prof_i = 0
        self.start = 0.0
        # 伺服环动态（合成经验值，待真机标定）：x_actual 一阶滞后跟随 pos，
        # pe=跟随误差；state: idle/moving/settling
        sd = ac.servo_defaults(self.name)
        self.tau = sd["tau"]
        self.settle_window = sd["settle_window"]
        self.settle_time = sd["settle_time"]
        self.x_actual = 0.0            # 初始等于名义值
        self.pe = 0.0
        self.state = "idle"
        self.settle_acc = 0.0          # |pe| 已在窗口内持续的时间（s）
        # 回零（CiA402 17/18）：home_switch/home_offset=None 表示按当前限位取默认值
        hd = ac.home_defaults(self.name, self.lo, self.hi, self.vmax)
        self.home_fast = hd["fast"]
        self.home_slow = hd["slow"]
        self.home_switch = None          # 显式覆盖的开关位置
        self.home_offset = None          # 显式覆盖的锁定位置
        self.home_state = "never"        # never / homing / done / fault
        self.home_phase = "idle"         # idle/fast_approach/retreat/latch/search/vendor_return/sync_follow/done/fault
        self.home_err = None
        self.home_traveled = 0.0
        self.home_follow = None          # plc_sync_trio 从轴：leader 轴名

    def home_params(self):
        """按当前限位动态计算开关/偏移默认值（限位在线改过也正确）。"""
        return ac.home_defaults(self.name, self.lo, self.hi, self.vmax)


class AirCircuit(object):
    __slots__ = ("name", "supports_neg", "cmd_state", "pressure", "flow")

    def __init__(self, name, supports_neg):
        self.name = name
        self.supports_neg = supports_neg
        self.cmd_state = "air"      # air / pos / neg
        self.pressure = 0.0         # kPa
        self.flow = 0.0             # L/min


class Heater(object):
    __slots__ = ("name", "rate", "max_C", "cur", "target", "steady_time",
                 "active", "preheat", "finish", "remaining", "power",
                 "cool_rate")

    def __init__(self, name, rate):
        self.name = name
        self.rate = rate              # 满功率升温速率 ℃/s（常数热改）
        self.max_C = HEATER_MAX_C.get(name, 511.4)   # 量程上限（常数热改）
        # 主动冷却速率 ℃/s（2026-09-02 迭代 11，仅 Heat_Dw 从 tuning
        # bond_hw.wafer_cool_rate_cps 热改；None=降温仍走自然热损模型）
        self.cool_rate = None
        self.cur = AMBIENT_TEMP
        self.target = AMBIENT_TEMP
        self.steady_time = 0.0
        self.active = False
        self.preheat = False
        self.finish = False
        self.remaining = 0.0
        self.power = 0.0              # PID 输出功率 %（ZOH，4ms 刷新）


class Engine(object):
    def __init__(self, points):
        self.points = points
        self.errors = None             # ErrorSystem，由 main 注入
        self.kinematics = None         # ErrorKinematics（6DOF 合成），由 main 注入
        self.axes = {row[0]: Axis(row) for row in ac.AXES}
        self.air = {n: AirCircuit(n, neg) for n, neg in AIR_CIRCUITS.items()}
        self.heaters = {n: Heater(n, r) for n, r in HEATERS.items()}
        self.force_target = 0.0        # 力回路 setpoint（N）
        self.force_value = 0.0         # 真值力（弹簧接触模型，N）
        self.force_mode = False        # 力控模式：False 时力回路整体旁路
                                       # （不 update/不写轴），轴位纯由 move 驱动
        # 三回路 TwinCAT 系列型位置式 PID（runtime/pid.py；力回路输出=
        # 压入量 mm——对象 F=k·压入量正增益；温度回路输出=功率 %）。
        # force 输出下限 -200：压入量可为负（头在接触位上方），覆盖
        # BS_Arm_Z 全行程（contact−pos），防位置式输出截断漂移
        p = PID_DEFAULTS
        self.pid_loops = {
            "force": pidmod.PID(
                "force", p["force_kp"], p["force_tn_s"], p["force_tv_s"],
                p["force_td_s"], out_min=-200.0, out_max=20.0,
                cycle_s=p["cycle_s"]),
            "headtemp": pidmod.PID(
                "headtemp", p["headtemp_kp"], p["headtemp_tn_s"],
                p["headtemp_tv_s"], p["headtemp_td_s"],
                out_min=0.0, out_max=100.0, cycle_s=p["cycle_s"]),
            "wafertemp": pidmod.PID(
                "wafertemp", p["wafertemp_kp"], p["wafertemp_tn_s"],
                p["wafertemp_tv_s"], p["wafertemp_td_s"],
                out_min=0.0, out_max=100.0, cycle_s=p["cycle_s"]),
        }
        self.pid_cycle_s = p["cycle_s"]
        self._pid_div = max(1, int(round(p["cycle_s"] / TICK_DT)))
        self._bond = dict(BOND_DEFAULTS)   # 调参/常数缓存（热改）
        self._bond["eff_contact_mm"] = BOND_DEFAULTS["head_z_contact_mm"]
        self._const_refresh_sim = -1e9     # 上次重读常数的 sim_time
        self._noise = random.Random()      # 读数噪声源（力/温度）
        self.lock = threading.RLock()
        self.speed_scale = 1.0
        self.estopped = False
        self.sim_time = 0.0
        self.seq = 0
        self._tick_count = 0
        self._frame_listeners = []
        # buffer 状态机
        self.buf_num = 0
        self.buf_running = False
        self.buf_step = 0
        self.buf_step_started = False
        # 统计
        self._intervals = deque(maxlen=4000)
        self._last_tick_t = None
        self._win_t0 = time.perf_counter()
        self._win_ticks = 0
        self._win_frames = 0
        self._last_frame_wall = 0.0
        self.tick_hz = 0.0
        self.push_hz = 0.0
        self.jitter_avg_us = 0.0
        self.jitter_max_us = 0.0
        self._running = False
        self._last_tick_err = 0.0      # tick 异常日志节流（秒，perf_counter 轴）
        # PLC 扫描周期（ms，整数）：物理 tick 每 plc_scan_ms 个 tick 调一次
        # points.commit()，把输入映像合并进输出映像（外部观测以扫描率采样）
        self.plc_scan_ms = PLC_SCAN_MS
        # 在线配置持久化
        self._overrides = {}
        self.overrides_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config_overrides.json")
        # DI 写通路缓存（变化才写点位表）
        self._di_cache = {}
        # SM↔GVL 同名物理信号对的镜像同步缓存：name -> (sm_v, gvl_v, v)
        self._pair_cache = {}
        # 联锁状态（每 tick 由 _tick_interlock_locked 刷新）
        self.interlock_enabled = True
        self.emo_latched = False         # EMO 锁存：解除 EMO + reset 才清
        self.doors_ok = True
        self.interlock_masked = False
        # 工件物料层（main 注入 workpieces.Workpieces）
        self.workpieces = None
        # 沙盒干涉检查（W5）：main 注入 collision.CollisionChecker
        # （runtime/sandboxes.json，由 tools/gen_sandboxes.py 从 TwinDOM 模型
        # 生成）；None=未加载（检查停用）
        self.collision = None
        self.collision_enabled = True
        # 工艺流程编排引擎（W4，main 注入 flow_engine.FlowEngine）
        self.flow = None
        # 视觉服务（main 注入 vision_service.VisionService）：相机触发只做
        # 非阻塞 queue 通知，渲染/TCP/匹配/MQTT 全在视觉线程，绝不拖慢 tick
        self.vision = None
        # 机器常数（main 注入 machine_constants.MachineConstants；视觉内参
        # 单一事实源）与测校服务（calibration.CalibrationService）
        self.constants = None
        self.calibration = None
        # 下位机调参（main 注入 tuning.Tuning：三回路 PID 整定 + bond_hw
        # 被控对象硬件参数；engine 每秒重读热生效）
        self.tuning = None
        # 光谱共焦读数模型（main 注入 confocal.ConfocalModel，调焦调平高度
        # 测量传感层；依赖 constants(focus_level 组)+vision(相机焦面)）
        self.confocal = None
        # 共焦主控侧 MQTT 客户端（main 注入 confocal.ConfocalClient；
        # fl_measure 经 confocal/trigger→confocal/result 真订阅消费读数，
        # 契约见 docs/前后端数据协议_V1.md §12）
        self.confocal_client = None
        # 处方（recipe，main 注入 recipe.Recipe）：晶圆/键合位/蓝膜布局参数
        self.recipe = None
        # 组件材质工程物性（main 注入 materials.Materials / ComponentCatalog，
        # 2026-09-06 迭代 21）：材料库+组件映射与前端模型组件目录
        self.materials = None
        self.components = None
        # 全机模态仿真（main 注入 modal.ModalSystem，2026-09-08 迭代 22）：
        # 每轴 SISO + 全机 MIMO 模态 1kHz 解析传播；None=未挂载（零开销）
        self.modal = None
        # 组件级热仿真（main 注入 thermal.ThermalSystem，2026-09-09 迭代 23）：
        # LPTN 热网络分频解算（加热器 tick 之后调用，消费 power%/轴活动度）；
        # None=未挂载（零开销）
        self.thermal = None
        # 键合接触沿状态（模态接触脉冲触发用：press 由非正转正的上一拍值）
        self._contact_prev = False
        # 最近一次粗对准测量偏差（flow_engine coarse_offset 步写入；
        # {"dx","dy","die","t"}，mm；交接流程 flip_handoff 消费做 FA_X 补偿）
        self.coarse_offset = None
        # 最近一次键合寻力实际接触位（flow_engine force_search 步写入，
        # BS_Arm_Z mm；后续 force 闭环步骤起始/日志消费；重启清零）
        self.bond_touch_pos = None
        # 接触确认证据（2026-09-04 迭代 13，force_search 步写入）：
        # {actual_contact_z_mm, eff_contact_z_mm, dev_um, touch_force_N,
        #  tol_um, judge}；bond_results chain.bond 采集留证；重启清零
        self.bond_contact_evidence = None
        # 调焦调平（flow_engine fl_measure/fl_solve 步写入；focus_level 流程
        # 消费）：分 space 的共焦测高记录、平面拟合结果、global 模式调平记忆
        # （运行时存储，仿 coarse_offset；重启清零）
        self.fl_measurements = {"wafer": [], "die": [], "stage": []}
        self.fl_fit = {}
        self.level_memory = None
        # 逐键合位精对准（flow_engine fa_* 步写入；fine_align 流程消费）：
        # 双侧标记测量记录、Δ 解算结果、最终补偿+确认结果（运行时存储，
        # 仿 ega_result；重启清零）
        self.fa_meas = None
        self.fa_delta = None
        self.fine_align_result = None
        # 键合接触时刻真值（2026-09-07 迭代 20，flow_engine bond_truth 步
        # 写入）：触力停轴后用 6DOF 真值位姿走精对准同一 fit 管线算得的
        # die↔site 真值残差 {site,tx,ty,theta,t,align_resid,process_shift}
        # （mm/rad）；bond_mark 消费落 die["resid"]，bond_results 采集为
        # truth 分量；重启清零
        self.bond_commit = None
        # 电源分系统状态（默认全上电；断电分系统的轴 move 409）
        self.power = {n: True for n in POWER_SUBSYSTEMS}
        # 塔灯/蜂鸣器闪烁参数（UpdateMode 写入，仅供观测回读）
        self.tower_flash = {}
        # 门锁使能（InterLock_<Side>=False 时该侧门不参与联锁）
        self.door_side_enabled = {"Front": True, "Left": True, "Right": True}
        # 相机拍照记录（环形）：{id, t, source, axes}
        self.captures = deque(maxlen=CAPTURES_MAX)
        self._capture_seq = 0
        # ES（蓝膜台外包模块）流程状态
        self.es_running = False
        # 故障注入（W2）：沉默目标（指令吞掉/HTTP 挂起）与断线目标（端口关闭）。
        # 运行时状态，重启复位；端口开关由 main 注册的钩子执行（引擎不持有 socket）
        self.silent = set()          # 子集 of FAULT_TARGETS
        self.dropped = set()
        self._fault_hooks = {}       # target -> (drop_fn, restore_fn)

    # ---------------- 帧输出 ----------------
    def add_frame_listener(self, fn):
        self._frame_listeners.append(fn)

    def _build_frame(self):
        values = [self.axes[n].pos for n in ac.AXIS_NAMES]
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        hdr = struct.pack(FRAME_FMT, FRAME_MAGIC, FRAME_VERSION,
                          FRAME_N_AXIS, self.sim_time, time.time(), self.seq)
        return hdr + struct.pack("<%df" % FRAME_N_AXIS, *values)

    # ---------------- 到位通知（误差系统 repeat 重采样） ----------------
    def _notify_settled(self, ax, was_moving):
        if was_moving and self.errors is not None:
            try:
                self.errors.settle(ax.name)
            except Exception:
                pass

    # ---------------- 联锁 ----------------
    def _axis_power_ok_locked(self, ax_name):
        """轴所属分系统是否上电（未列入映射的轴不受电源联锁约束）。"""
        for sub, names in POWER_AXIS_MAP.items():
            if ax_name in names:
                return self.power.get(sub, True)
        return True

    def _check_interlock_locked(self, ax_name=None):
        """新运动许可检查：EMO 锁存 / 门开未屏蔽 / 分系统断电 → 409（InterlockError）。"""
        if not self.interlock_enabled:
            return
        if self.emo_latched:
            raise InterlockError("EMO active: 解除 EMO 后 reset 恢复")
        if not self.doors_ok and not self.interlock_masked:
            raise InterlockError(
                "door open: 关好门或置 iInterlock_Mask/oAllDoors_Mask 屏蔽")
        if ax_name is not None and not self._axis_power_ok_locked(ax_name):
            raise InterlockError(
                "power off: %s 所属分系统未上电（M_PowerSwitch）" % ax_name)

    def set_interlock(self, enabled):
        with self.lock:
            self.interlock_enabled = bool(enabled)
            return self.interlock_enabled

    def _sync_pair_locked(self, sm, gvl):
        """同一物理信号的两个点位名（SM↔GVL）：最后写入方生效，回写两侧。"""
        sm_v = bool(self.points.get(sm, True))
        gvl_v = bool(self.points.get(gvl, True))
        prev = self._pair_cache.get(sm)
        if prev is None:
            v = sm_v and gvl_v            # 启动对齐
        elif sm_v != prev[0]:
            v = sm_v                      # SM 侧最后被写
        elif gvl_v != prev[1]:
            v = gvl_v                     # GVL 侧最后被写
        else:
            v = prev[2]
        self._pair_cache[sm] = (sm_v, gvl_v, v)
        if self.points.has(sm):
            self._set_di(sm, v)
        if self.points.has(gvl):
            self._set_di(gvl, v)
        return v

    def _tick_interlock_locked(self):
        """每 tick 联锁检查（O(点位数)，成本可忽略）：
        - EMO 任一触发 → 锁存 + 等效 estop（全轴停止；解除 EMO + reset 恢复）
        - 任一门开且未屏蔽 → 进行中的运动 soft-stop（新运动在 _check_interlock_locked 拒绝）
        - 塔灯自动：红=急停/EMO/回零故障，绿=有轴在动，否则黄（自动优先于手动写入）
        - SystemMachine ↔ GVL_IO 两族点位镜像同步（面板读 GVL_IO 一族）
        """
        emo_pairs = zip(EMO_POINTS, GVL_EMO_POINTS)
        if any(self._sync_pair_locked(sm, gvl) for sm, gvl in emo_pairs):
            self.emo_latched = True
        # 门锁：按侧聚合，该侧被 InterLock_<Side> 禁用时不参与
        side_ok = {"Front": True, "Left": True, "Right": True}
        for sm, gvl in DOOR_MIRROR:
            v = self._sync_pair_locked(sm, gvl)
            for side in side_ok:
                if side in sm:
                    side_ok[side] = side_ok[side] and v
        self.doors_ok = all(
            ok for side, ok in side_ok.items()
            if self.door_side_enabled.get(side, True))
        self.interlock_masked = any(bool(self.points.get(p))
                                    for p in MASK_POINTS)
        if self.interlock_enabled:
            if self.emo_latched and not self.estopped:
                self.estop()                    # EMO 等效急停（锁存）
            if not self.doors_ok and not self.interlock_masked:
                for ax in self.axes.values():   # 门开 soft-stop 进行中的运动
                    if ax.moving:
                        self.stop(ax.name)
        fault = any(ax.home_state == "fault" for ax in self.axes.values())
        red = self.estopped or self.emo_latched or fault
        green = (not red) and any(ax.moving for ax in self.axes.values())
        for key, on in (("red", red), ("green", green),
                        ("yellow", not red and not green)):
            self._set_di(TOWER_POINTS[key], on)
            gvl = GVL_TOWER_POINTS.get(key)
            if gvl and self.points.has(gvl):
                self._set_di(gvl, on)

    # ---------------- 沙盒干涉检查（W5） ----------------
    def _axis_positions_locked(self):
        return {n: ax.pos for n, ax in self.axes.items()}

    def _check_collision_locked(self, ax, target):
        """move 起步检查：目标位沙盒重叠 → 409（InterlockError，含盒名）。"""
        if not (self.collision_enabled and self.collision is not None):
            return
        hits = self.collision.check_move(ax.name, target,
                                         self._axis_positions_locked())
        if hits:
            # 零件级盒名可能一次多对：报文列前 3 对 + 总对数
            shown = "；".join("%s × %s" % h for h in hits[:3])
            raise InterlockError(
                "沙盒干涉：%s（%s → %.4g 落在重叠区，共 %d 对冲突）"
                % (shown, ax.name, target, len(hits)))

    def _tick_collision_locked(self):
        """每 2 tick（500Hz）沙盒重叠检查：运动盒撞入 → 全轴 soft-stop + 记录。
        已静止的重叠（如关闭期间停进重叠区）只在 /api/collision 报告，不反复停。"""
        if not (self.collision_enabled and self.collision is not None):
            return
        if self._tick_count % 2 != 1:
            return
        hits = self.collision.check_tick(self._axis_positions_locked())
        if not hits:
            return
        moving = [n for n, a in self.axes.items() if a.moving]
        if moving:
            for n in moving:                 # 与门开 soft-stop 同语义
                self.stop(n)
            self.collision.record_event(self.sim_time, hits, moving)

    def set_collision(self, enabled):
        # 重开时不清缓存：check_tick 按轴位置差量重算，能立刻发现关闭期间
        # 形成的重叠（有轴在动 → soft-stop；静止重叠 → 只在 /api/collision 报告）
        with self.lock:
            self.collision_enabled = bool(enabled)
            return self.collision_enabled

    def set_collision_depth(self, value):
        """在线改穿模深度阈值 depth_eps（mm），非法值抛 ValueError（→400）。"""
        with self.lock:
            if self.collision is None:
                raise ValueError("沙盒干涉检查未加载")
            return self.collision.set_depth_eps(value)

    def collision_state(self):
        with self.lock:
            if self.collision is None:
                return {"ok": True, "loaded": False, "enabled": False}
            out = {"ok": True, "loaded": True,
                   "enabled": self.collision_enabled}
            out.update(self.collision.state())
            return out

    # ---------------- 轴指令 ----------------
    def _controller_silent_locked(self, ax_name):
        """轴所属控制器（ACS/PLC）是否处于沉默：指令被吞（收到但没执行）。"""
        ctl = self.axes[ax_name].controller.lower()
        return ctl in self.silent

    def _check_axis_ready_locked(self, ax):
        """掉使能/故障轴拒绝新运动（沿用联锁 409 模式，HTTP 层映射 409）。"""
        if ax.fault:
            raise InterlockError(
                "axis fault: %s（先 ResetError/Enable 恢复）" % ax.name)
        if not ax.enabled:
            raise InterlockError(
                "axis not enabled: %s（先 Enable）" % ax.name)

    def _start_move_locked(self, ax, target, speed=None):
        self._check_interlock_locked(ax.name)
        if self.estopped:
            raise RuntimeError("estop active, reset first")
        self._check_axis_ready_locked(ax)
        if self._controller_silent_locked(ax.name):
            # 沉默目标：固件挂死语义——指令被吞，轴不动，无错误也无动作
            return 0.0
        if ax.home_state == "homing":
            raise RuntimeError("%s is homing" % ax.name)
        if ax.home_state == "fault":
            raise RuntimeError("%s homing fault, reset first" % ax.name)
        ax.decel = 0.0                       # 新运动取代未完成的自由减速
        target = max(ax.lo, min(ax.hi, float(target)))
        self._check_collision_locked(ax, target)   # 沙盒：起步即撞 → 409
        vmax = min(abs(float(speed)), ax.vmax) if speed else ax.vmax
        if vmax <= 0:
            vmax = ax.vmax
        dist = target - ax.pos
        if ax.profile_type == "trapezoid":
            ax.profile = plan_trapezoid(dist, vmax, ax.amax, TICK_DT)
        else:
            ax.profile = plan_move(dist, vmax, ax.amax, ax.jerk, TICK_DT)
        ax.prof_i = 0
        ax.start = ax.pos
        ax.target = target
        ax.moving = True
        ax.in_position = False
        return profile_duration(ax.profile, TICK_DT)

    def move(self, name_or_id, target, speed=None):
        names = ac.resolve_axis(name_or_id)
        if not names:
            raise KeyError("unknown axis: %r" % name_or_id)
        # 【旁路副本】顶针三轴只有"一个轴控"：任一根下发指令，三根一起同步到位
        # （与 home 的 plc_sync_trio 语义一致；细化前端也只有一根 axis_WS_EPIN_Z）
        if any(n in ac.HOME_TYPE_PLC_SYNC_TRIO for n in names):
            names = sorted(set(names) | ac.HOME_TYPE_PLC_SYNC_TRIO,
                           key=lambda n: ac.NAME_TO_IDX[n])
        dur = 0.0
        with self.lock:
            for n in names:
                dur = max(dur, self._start_move_locked(self.axes[n], target, speed))
        return dur

    def home(self, name_or_id, mode=None):
        """按轴类型分派回零（状态机由 tick 逐 1ms 推进）。

        - acs_limit / gantry_dual：CiA402 17/18 式限位开关序列
          （18 = 正向找正限位开关，17 = 负向找负限位开关）：
          fast_approach → 触发开关 → retreat（慢速退出）→ 开关释放
          → latch（慢速再逼近锁定边沿）→ pos = home_offset → done。
        - plc_sync_trio：任一顶针轴回零 = 三轴整组同步回零（leader 跑状态机）。
        - zero_sensor：慢速朝 0 搜索零位传感器窗口，进窗即锁定。
        - vendor_es：外包模块占位实现，恒速直接回 0。
        """
        names = ac.resolve_axis(name_or_id) if name_or_id != "ALL" else list(self.axes)
        if not names:
            raise KeyError("unknown axis: %r" % name_or_id)
        # 顶针三轴联动：任一成员回零即整组回零
        if any(n in ac.HOME_TYPE_PLC_SYNC_TRIO for n in names):
            names = sorted(set(names) | ac.HOME_TYPE_PLC_SYNC_TRIO,
                           key=lambda n: ac.NAME_TO_IDX[n])
        with self.lock:
            self._check_interlock_locked()
            if self.estopped:
                raise RuntimeError("estop active, reset first")
            for n in names:
                ax = self.axes[n]
                self._check_axis_ready_locked(ax)
                if ax.home_state == "homing":
                    raise RuntimeError("%s already homing" % n)
                if ax.home_state == "fault":
                    raise RuntimeError("%s homing fault, reset first" % n)
            for n in names:
                if self._controller_silent_locked(n):
                    continue             # 沉默目标：指令被吞，轴不动
                ax = self.axes[n]
                m = mode or ax.home_mode
                if m not in (17, 18):
                    raise ValueError("home method 只能是 17(负)/18(正)")
                ax.home_mode = m
                ax.decel = 0.0
                ax.profile = None               # 取消正在进行的点位运动
                ax.moving = True
                ax.in_position = False
                ax.homed = False
                ax.home_state = "homing"
                ax.home_err = None
                ax.home_traveled = 0.0
                t = ac.home_type(n)
                if t == "plc_sync_trio" and n != ac.EPIN_TRIO_LEADER:
                    ax.home_phase = "sync_follow"
                    ax.home_follow = ac.EPIN_TRIO_LEADER
                else:
                    ax.home_follow = None
                    ax.home_phase = {"zero_sensor": "search",
                                     "vendor_es": "vendor_return"}.get(
                                         t, "fast_approach")

    def _tick_homing_locked(self, ax):
        """每 tick 推进回零状态机（恒速段，位置逐 tick 积分），按类型分派。"""
        if ax.home_follow is not None:
            return                       # plc_sync_trio 从轴由 leader 一并驱动
        t = ac.home_type(ax.name)
        if t == "zero_sensor":
            self._tick_home_zero_sensor_locked(ax)
        elif t == "vendor_es":
            self._tick_home_vendor_es_locked(ax)
        else:
            self._tick_home_limit_locked(ax)   # acs_limit/gantry_dual/trio leader

    def _home_done_locked(self, ax, offset):
        ax.pos = offset
        ax.vel = 0.0
        ax.target = offset
        ax.moving = False
        ax.in_position = True
        ax.homed = True
        ax.home_state = "done"
        ax.home_phase = "done"
        ax.home_follow = None
        # 回零完成瞬间重定义位置：x_actual 直接对齐（pe 清零），
        # 避免回零恒速段累积的跟随误差拖出一段假的 settling
        ax.x_actual = offset
        ax.pe = 0.0
        ax.state = "idle"
        ax.settle_acc = 0.0
        self._notify_settled(ax, True)
        if ac.home_type(ax.name) == "gantry_dual" and self.errors is not None:
            # 双驱龙门：回零后注入固定残余偏摆（actual 可见，名义值不受影响）
            res = (ac.GANTRY_RESIDUAL_LINEAR if ax.unit == "mm"
                   else ac.GANTRY_RESIDUAL_ROTARY)
            try:
                self.errors.add_fixed_offset(ax.name, res)
            except Exception:
                pass

    def _home_fault_locked(self, ax, hp, msg):
        ax.moving = False
        ax.vel = 0.0
        # 位置未重定义：in_position 交给伺服 settling 流程按 pe 收敛置位
        ax.in_position = False
        ax.home_state = "fault"
        ax.home_phase = "fault"
        ax.home_err = msg
        self._notify_settled(ax, True)

    def _sync_trio_locked(self, ax, old_pos):
        """plc_sync_trio：leader 位移/结局同步到两根从轴。"""
        if ax.name != ac.EPIN_TRIO_LEADER:
            return
        delta = ax.pos - old_pos
        for fn in sorted(ac.HOME_TYPE_PLC_SYNC_TRIO):
            if fn == ax.name:
                continue
            f = self.axes[fn]
            if f.home_state != "homing":
                continue
            if ax.home_state == "done":
                self._home_done_locked(f, ax.pos)
            elif ax.home_state == "fault":
                self._home_fault_locked(f, None, ax.home_err)
            else:
                f.pos += delta
                f.vel = ax.vel

    def _tick_home_limit_locked(self, ax):
        """acs_limit / gantry_dual / plc_sync_trio(leader)：CiA402 17/18 限位开关序列。"""
        hp = ax.home_params()
        s = 1.0 if ax.home_mode == 18 else -1.0
        sw = ax.home_switch if ax.home_switch is not None else hp["switch"]
        offset = ax.home_offset if ax.home_offset is not None else hp["offset"]
        hyst = hp["hysteresis"]
        old = ax.pos
        if ax.home_phase == "fast_approach":
            d = s * ax.home_fast * TICK_DT
            hit = (ax.pos >= sw) if s > 0 else (ax.pos <= sw)
            if hit:
                ax.home_phase = "retreat"
            else:
                ax.pos += d
                ax.vel = s * ax.home_fast
        elif ax.home_phase == "retreat":
            d = -s * ax.home_slow * TICK_DT
            released = (ax.pos <= sw - hyst) if s > 0 else (ax.pos >= sw + hyst)
            if released:
                ax.home_phase = "latch"
            else:
                ax.pos += d
                ax.vel = -s * ax.home_slow
        elif ax.home_phase == "latch":
            d = s * ax.home_slow * TICK_DT
            hit = (ax.pos >= sw) if s > 0 else (ax.pos <= sw)
            if hit:                              # 锁定开关边沿
                self._home_done_locked(ax, offset)
                self._sync_trio_locked(ax, old)
                return
            ax.pos += d
            ax.vel = s * ax.home_slow
        ax.home_traveled += abs(ax.vel) * TICK_DT
        if ax.home_traveled > hp["fault_travel"]:
            self._home_fault_locked(
                ax, hp, "limit switch not reached within %.3f travel"
                % hp["fault_travel"])
        self._sync_trio_locked(ax, old)

    def _tick_home_zero_sensor_locked(self, ax):
        """zero_sensor（BS_CAL_Rx/Ry）：慢速朝 0 搜索零位传感器窗口，进窗即锁定。"""
        hp = ax.home_params()
        offset = ax.home_offset if ax.home_offset is not None else 0.0
        s = -1.0 if ax.pos > 0 else 1.0
        step = s * ax.home_slow * TICK_DT
        new = ax.pos + step
        if abs(new) <= ac.ZERO_SENSOR_WINDOW or new * ax.pos < 0:
            # 进入零位窗口（或越过 0）→ 锁定到 home_offset（默认 0）
            self._home_done_locked(ax, offset)
            return
        ax.pos = new
        ax.vel = s * ax.home_slow
        ax.home_traveled += abs(ax.vel) * TICK_DT
        if ax.home_traveled > hp["fault_travel"]:
            self._home_fault_locked(
                ax, hp, "zero sensor not found within %.3f travel"
                % hp["fault_travel"])

    def _tick_home_vendor_es_locked(self, ax):
        """vendor_es（蓝膜台 X/Y 外包模块，占位）：恒速直接回 0。"""
        hp = ax.home_params()
        if ax.pos == 0.0:
            self._home_done_locked(ax, 0.0)
            return
        s = -1.0 if ax.pos > 0 else 1.0
        step = s * ac.VENDOR_ES_HOME_SPEED * TICK_DT
        new = ax.pos + step
        if new * ax.pos <= 0:                    # 到达/越过 0
            self._home_done_locked(ax, 0.0)
            return
        ax.pos = new
        ax.vel = s * ac.VENDOR_ES_HOME_SPEED
        ax.home_traveled += abs(ax.vel) * TICK_DT
        if ax.home_traveled > hp["fault_travel"]:
            self._home_fault_locked(
                ax, hp, "vendor return-to-zero exceeded %.3f travel"
                % hp["fault_travel"])

    def stop(self, name_or_id=None):
        with self.lock:
            names = list(self.axes) if name_or_id in (None, "ALL") \
                else (ac.resolve_axis(name_or_id) or [])
            for n in names:
                ax = self.axes[n]
                was = ax.moving
                ax.moving = False
                ax.profile = None
                ax.vel = 0.0
                if ax.decel:                 # 自由减速被截断：仍按掉使能进 fault
                    ax.decel = 0.0
                    ax.fault = True
                    ax.in_position = False
                # 不再直接判到位：进入 settling，pe 收敛后才置 in_position
                # （对本来静止的轴不扰动到位标志）
                elif was:
                    ax.in_position = False
                if ax.home_state == "homing":       # 中断回零
                    ax.home_state = "never"
                    ax.home_phase = "idle"
                    ax.home_follow = None
                self._notify_settled(ax, was)

    def estop(self):
        with self.lock:
            self.estopped = True
            self.buf_running = False
            for ax in self.axes.values():
                was = ax.moving
                ax.moving = False
                ax.profile = None
                ax.vel = 0.0
                if ax.decel:                 # 自由减速被急停截断：仍进 fault
                    ax.decel = 0.0
                    ax.fault = True
                    ax.in_position = False
                if ax.home_state == "homing":       # estop 中断回零
                    ax.home_state = "never"
                    ax.home_phase = "idle"
                    ax.home_follow = None
                self._notify_settled(ax, was)

    def reset(self):
        with self.lock:
            self.estopped = False
            # EMO 锁存只在原始 EMO 点位全部解除后由 reset 清除
            if not any(bool(self.points.get(p)) for p in EMO_POINTS):
                self.emo_latched = False
            for ax in self.axes.values():           # 故障需 reset 恢复
                ax.fault = False                    # 掉使能故障（runtime 态）
                if ax.home_state == "fault":
                    ax.home_state = "never"
                    ax.home_phase = "idle"
                    ax.home_err = None
                    ax.home_follow = None

    def set_speed_scale(self, scale):
        with self.lock:
            self.speed_scale = max(0.1, min(8.0, float(scale)))
            return self.speed_scale

    def _disable_axis_locked(self, ax):
        """掉使能（模拟真 ACS 驱动器）：静止 → 仅去使能不进 fault；
        运动中 → 中止轨迹，按 KillDec 量级自由减速到静止，随后进 fault
        （真机掉使能即报警）；in_position 交伺服 settling 按 pe 收敛置位。"""
        ax.enabled = False
        if ax.fault:
            return
        moving = ax.moving or ax.home_state == "homing" or abs(ax.vel) > 1e-12
        if not moving:
            return
        if ax.home_state == "homing":        # 中断回零序列
            ax.home_state = "never"
            ax.home_phase = "idle"
            ax.home_follow = None
        ax.profile = None                    # 轨迹中止
        ax.decel = KILL_DEC_FACTOR * ax.amax  # 合成减速度（见 KILL_DEC_FACTOR 注释）
        ax.moving = True                     # 保持 moving 直到减速到静止
        ax.in_position = False

    def enable(self, name_or_id, on=True):
        """Enable/Disable。Enable：使能并清 fault（宽松处理，真机需先 ResetError）；
        Disable：静止仅去使能，运动中自由减速后进 fault。"""
        names = ac.resolve_axis(name_or_id) if name_or_id != "ALL" else list(self.axes)
        if not names:
            raise KeyError("unknown axis: %r" % name_or_id)
        with self.lock:
            for n in names:
                ax = self.axes[n]
                if on:
                    ax.enabled = True
                    ax.fault = False
                else:
                    self._disable_axis_locked(ax)

    def reset_error(self, name_or_id):
        """ResetError：清 fault（不改变 enabled，真机语义：复位后再 Enable）；
        回零 fault 态一并清除（stage 级复位等价于真机驱动器 ResetError）。"""
        names = ac.resolve_axis(name_or_id) if name_or_id != "ALL" else list(self.axes)
        if not names:
            raise KeyError("unknown axis: %r" % name_or_id)
        with self.lock:
            for n in names:
                ax = self.axes[n]
                ax.fault = False
                if ax.home_state == "fault":
                    ax.home_state = "never"
                    ax.home_phase = "idle"
                    ax.home_err = None
                    ax.home_follow = None

    # ---------------- 故障注入：沉默 / 断线 ----------------
    def register_fault_hooks(self, target, drop_fn, restore_fn):
        """main 注册端口开关钩子（drop=关 listen socket 拒连，restore=重绑恢复）。"""
        self._fault_hooks[target] = (drop_fn, restore_fn)

    def is_silent(self, target):
        with self.lock:
            return target in self.silent

    def set_silent(self, target, on):
        """沉默 on/off：目标服务"收到指令不回不执行"（模拟固件挂死）。"""
        if target not in FAULT_TARGETS:
            raise ValueError("target 只能是 %s" % "/".join(FAULT_TARGETS))
        with self.lock:
            if on:
                self.silent.add(target)
            else:
                self.silent.discard(target)
            return sorted(self.silent)

    def set_dropped(self, target, on):
        """断线 on/off：关 listen socket 拒绝新连接；restore 重绑恢复。
        端口开关在引擎锁外执行（shutdown 会阻塞 ~0.5s 等 serve 循环退出）。"""
        if target not in FAULT_TARGETS:
            raise ValueError("target 只能是 %s" % "/".join(FAULT_TARGETS))
        hooks = self._fault_hooks.get(target)
        if hooks is None:
            raise RuntimeError("target %s 未注册端口钩子" % target)
        with self.lock:
            cur = target in self.dropped
            if cur == bool(on):
                return sorted(self.dropped)
            if on:
                self.dropped.add(target)
            else:
                self.dropped.discard(target)
        try:
            (hooks[0] if on else hooks[1])()
        except Exception:
            with self.lock:                  # 回滚状态，保持与端口实况一致
                if on:
                    self.dropped.discard(target)
                else:
                    self.dropped.add(target)
            raise
        with self.lock:
            return sorted(self.dropped)

    def fault_state(self):
        """/api/fault GET：当前故障态全量快照。"""
        with self.lock:
            return {"ok": True,
                    "silent": sorted(self.silent),
                    "dropped": sorted(self.dropped),
                    "fault_axes": sorted(n for n, a in self.axes.items()
                                         if a.fault),
                    "axes": {n: {"enabled": a.enabled, "fault": a.fault}
                             for n, a in self.axes.items()}}

    # ---------------- buffer ----------------
    def set_buffer(self, num):
        num = int(num)
        if num not in BUFFERS:
            raise KeyError("buffer %d not defined (have %s)"
                           % (num, sorted(BUFFERS)))
        with self.lock:
            self.buf_num = num

    def run_buffer(self, num=None):
        with self.lock:
            if self.estopped:
                raise RuntimeError("estop active, reset first")
            if num is not None:
                self.set_buffer(int(num))
            if self.buf_num not in BUFFERS:
                raise RuntimeError("no valid buffer selected")
            self.buf_running = True
            self.buf_step = 0
            self.buf_step_started = False

    def stop_buffer(self):
        with self.lock:
            self.buf_running = False   # 保留步号，可 ResumeBuffer

    def resume_buffer(self):
        with self.lock:
            if self.estopped:
                raise RuntimeError("estop active, reset first")
            if self.buf_num in BUFFERS and self.buf_step < len(BUFFERS[self.buf_num]["steps"]):
                self.buf_running = True

    def buffer_state(self):
        with self.lock:
            return {"oBufferDo": self.buf_running, "oBufferNum": self.buf_num}

    # ---------------- PLC 侧物理 ----------------
    def air_command(self, path, op):
        """path 可为全路径 SystemMachine.SS_ME.M_AIR.WT_WaferVac 或回路名。"""
        name = path[len(AIR_BASE):] if path.startswith(AIR_BASE) else path.rsplit(".", 1)[-1]
        c = self.air.get(name)
        if c is None:
            raise KeyError("unknown air circuit: %r" % path)
        with self.lock:
            if "plc" in self.silent:
                # PLC 沉默：气路指令被吞（收到但没执行），回路状态不变
                return {"swallowed": True}
        if op == "OpenPositive":
            c.cmd_state = "pos"
        elif op == "OpenNegtive":
            if not c.supports_neg:
                raise ValueError("%s has no negative line" % name)
            c.cmd_state = "neg"
        elif op == "OpenAir":
            c.cmd_state = "air"
        else:
            raise ValueError("unknown air op: %r" % op)

    def heater_command(self, which, cmd, args=None):
        h = self.heaters.get(which)
        if h is None:
            raise KeyError("unknown heater: %r" % which)
        args = args or {}
        if cmd == "SetTargetTempWithPlan":
            h.target = float(args.get("Temperature", args.get("temperature", 100)))
            h.steady_time = float(args.get("Steady_Time", args.get("steady_time", 0)))
            h.active = True
            h.preheat = False
            h.finish = False
        elif cmd == "StartPreHeat":
            h.active = True
            h.preheat = True
            if h.target <= AMBIENT_TEMP:
                h.target = 100.0
            h.finish = False
        elif cmd == "StopTargetTempWithPlan":
            h.active = False
            h.preheat = False
        else:
            raise ValueError("unknown heater cmd: %r" % cmd)

    def bonding_force(self, force, speed):
        """BondingByForceManual 命令面：设力回路 setpoint（N）。
        speed 参数 2026-08-28 迭代 8 起弃用（旧一阶 tau 映射废除，力回路
        改 TwinCAT 系列型位置式 PID）；签名保留兼容旧命令面。
        2026-09-XX 迭代 29：一律进力控模式（含 Force=0——键合流程的
        受控卸力命令，需力回路抬头卸到 ±tol；退出由 _tick_pid_locked
        在 sp=0 且 pv≤0 时自动完成）。"""
        with self.lock:
            self.force_target = float(force)
            self.force_mode = True

    def reset_thermal_force(self):
        """温控压控回环境态（2026-09-04 迭代 13，/api/full_reset）：三回路
        PID 全复位、加热器全关（active/preheat 落）、温度回环境温度、
        力 setpoint/真值输出回 0。不触碰 tuning/机械常数/处方持久化
        文件；硅片台常保温（wafer_keep_warm）不自动重开——与刚上电
        状态一致需 /api/recipe/apply 或重启恢复。"""
        with self.lock:
            for h in self.heaters.values():
                h.active = False
                h.preheat = False
                h.finish = False
                h.target = AMBIENT_TEMP
                h.cur = AMBIENT_TEMP
                h.power = 0.0
                h.steady_time = 0.0
                h.remaining = 0.0
            self.force_target = 0.0
            self.force_value = 0.0
            self.force_mode = False
            for loop in self.pid_loops.values():
                loop.reset(0.0, 0.0, 0.0)

    def apply_wafer_keep_warm(self):
        """硅片台常保温（处方 bond.wafer_keep_warm=true，2026-08-31 迭代 9）：
        处方 apply 与启动加载时调用——Heat_Dw 目标温度设为 wafer_temp_C 并
        常开 PID（bond 流程外也保温，流程等温步到带内秒过）。关闭时不动
        现状（维持 wafer_preheat=false 旧行为：流程内才升温）。"""
        if self.recipe is None:
            return
        try:
            b = (self.recipe.get().get("bond") or {})
        except Exception:
            return
        if not b.get("wafer_keep_warm"):
            return
        t = float(b.get("wafer_temp_C", 150.0))
        self.heater_command("Heat_Dw", "SetTargetTempWithPlan",
                            {"Temperature": t})

    # ---------------- 三回路 PID（迭代 8） ----------------
    def _refresh_loop_consts_locked(self):
        """重读调参/常数/处方（每秒一次；缺失回退默认值，裸引擎单测兼容）。
        PID 参数与 bond_hw 读 tuning（旧文件兜底读 machine_constants pid）；
        裸接触位/寻力位读 machine_constants bond 组；读数噪声读 recipe
        sim_sensors 优先（旧 machine_constants bond 键兜底）。PID 参数
        set_params 热改不复位积分（无扰）。有效接触位按处方晶圆/芯片厚度
        换算（effective_bond_contact_mm，与 flow 侧同一函数）。"""
        bond = dict(BOND_DEFAULTS)
        p = dict(PID_DEFAULTS)
        # 传感链标定常数（机器常数 sensor_calib 组，迭代 49）与注入真值
        # 旋钮（recipe sim_sensors 组）——发布读数链：raw=真值×(1+gain_err)
        # +offset_err+noise → 发布=(raw−offset_cal)/gain_cal。键名映射：
        # PulseHeat_Up↔head_temp_*、Heat_Dw↔wafer_temp_*、力↔force_*
        calib = {"head_temp_gain": 1.0, "head_temp_offset_c": 0.0,
                 "wafer_temp_gain": 1.0, "wafer_temp_offset_c": 0.0,
                 "force_gain": 1.0, "force_offset_n": 0.0}
        if self.constants is not None:
            try:
                mc = self.constants.get()
            except Exception:
                mc = {}
            b = mc.get("bond")
            if isinstance(b, dict):
                # 裸接触位/寻力位（正常键）+ 旧文件兜底：bond 段硬件/噪声键
                # （正常已迁 tuning/recipe）
                for k in BOND_DEFAULTS:
                    if k in b:
                        bond[k] = float(b[k])
            sc = mc.get("sensor_calib")
            if isinstance(sc, dict):
                for k in calib:
                    v = sc.get(k)
                    try:
                        if v is not None and abs(float(v)) > 1e-12:
                            calib[k] = float(v)
                        elif v is not None and "offset" in k:
                            calib[k] = float(v)
                    except (TypeError, ValueError):
                        pass
            fh = mc.get("flip_handoff")
            if isinstance(fh, dict):
                try:
                    if fh.get("fa_x_handoff_mm") is not None:
                        bond["fa_x_handoff_mm"] = float(
                            fh["fa_x_handoff_mm"])
                except (TypeError, ValueError):
                    pass
            pp = mc.get("pid")
            if isinstance(pp, dict):
                for k in p:
                    if k in pp:
                        p[k] = float(pp[k])
        if self.tuning is not None:
            try:
                tn = self.tuning.get()
            except Exception:
                tn = {}
            hw = tn.get("bond_hw")
            if isinstance(hw, dict):
                for k in hw:
                    if k in bond:
                        bond[k] = float(hw[k])
            pp = tn.get("pid")
            if isinstance(pp, dict):
                for k in p:
                    if k in pp:
                        p[k] = float(pp[k])
        # 厚度换算：有效接触位/寻力位 = 裸值 + 晶圆厚 + 芯片厚（recipe）
        wt = ct = 0.0
        if self.recipe is not None:
            try:
                rec = self.recipe.get()
            except Exception:
                rec = {}
            try:
                wt = float((rec.get("wafer") or {}).get("thickness_mm", 0.0))
                ct = float(((rec.get("film") or {}).get("dies") or {})
                           .get("thickness_mm", 0.0))
            except (TypeError, ValueError):
                wt = ct = 0.0
            # 读数噪声：recipe sim_sensors 优先（2026-08-31 迭代 9 迁入）
            ss = rec.get("sim_sensors")
            if isinstance(ss, dict):
                if "force_noise_N" in ss:
                    try:
                        bond["force_noise_N"] = float(ss["force_noise_N"])
                    except (TypeError, ValueError):
                        pass
                # 温度噪声逐加热器分路（2026-09-08 迭代 22 拆分）：旧标量
                # 值展开为全加热器同值，统一为 {加热器名: 幅值} 字典
                tn = ss.get("temp_noise_C")
                if tn is not None:
                    td = {}
                    for hname in HEATERS:
                        v = tn.get(hname) if isinstance(tn, dict) else tn
                        try:
                            td[hname] = float(v)
                        except (TypeError, ValueError):
                            td[hname] = BOND_DEFAULTS["temp_noise_C"]
                    bond["temp_noise_C"] = td
                # 传感链增益/零偏注入（迭代 49）：温度按加热器分路（同
                # temp_noise_C 口径），力为标量
                for skey, bkey in (("temp_gain_err", "temp_gain_err"),
                                   ("temp_offset_C", "temp_offset_C")):
                    sv = ss.get(skey)
                    if sv is not None:
                        sd = {}
                        for hname in HEATERS:
                            v = sv.get(hname) if isinstance(sv, dict) else sv
                            try:
                                sd[hname] = float(v)
                            except (TypeError, ValueError):
                                sd[hname] = 0.0
                        bond[bkey] = sd
                for skey in ("force_gain_err", "force_offset_N"):
                    if skey in ss:
                        try:
                            bond[skey] = float(ss[skey])
                        except (TypeError, ValueError):
                            pass
        # 传感链缓存键兜底（无常数/处方时的默认；标定理想/零注入）
        bond.setdefault("fa_x_handoff_mm", -1242.35)
        for k, dfl in (("temp_gain_err", None), ("temp_offset_C", None)):
            if not isinstance(bond.get(k), dict):
                bond[k] = {h: 0.0 for h in HEATERS}
        bond.setdefault("force_gain_err", 0.0)
        bond.setdefault("force_offset_N", 0.0)
        # 处方厚度缓存（flip 接触支路与有效接触位共用）
        bond["wafer_thickness_mm"] = wt
        bond["chip_thickness_mm"] = ct
        # flip 交接接触支路前置（迭代 49）：flip 上有 die 才激活（1s 重读
        # 足够——交接动作前 die 已在 flip 稳态，测校寻力分钟级）
        try:
            bond["flip_has_die"] = bool(
                self.workpieces is not None
                and any(d.get("carrier") == "flip"
                        for d in list(self.workpieces.dies)))
        except Exception:
            bond["flip_has_die"] = False
        bond["sensor_calib"] = calib
        # 接触位热修正（迭代 56）：吸盘 Z 胀（+dz_c，chuck_z_growth）与头
        # Z 伸长（+dz_h，head_z_growth，负值=向下）使真实接触提前——
        # 接触条件 head_tool_z+dz_h = chuck_top+t_w+t_d+dz_c →
        # ax_contact = base + dz_c − dz_h；热 OFF/未挂载恒 0（零影响）。
        th = getattr(self, "thermal", None)
        try:
            dz_c = float(th.module_offset("wafer_stage")[0][2]) \
                if th is not None else 0.0
            dz_h = float(th.module_offset("bond_head")[0][2]) \
                if th is not None else 0.0
        except Exception:
            dz_c = dz_h = 0.0
        bond["therm_dz_chuck_mm"] = dz_c     # 观测/测试消费
        bond["therm_dz_head_mm"] = dz_h
        bond["eff_contact_mm"] = effective_bond_contact_mm(
            bond["head_z_contact_mm"], wt, ct) + dz_c - dz_h
        self._bond = bond
        for name, pre in (("force", "force"), ("headtemp", "headtemp"),
                          ("wafertemp", "wafertemp")):
            self.pid_loops[name].set_params(
                kp=p[pre + "_kp"], tn_s=p[pre + "_tn_s"],
                tv_s=p[pre + "_tv_s"], td_s=p[pre + "_td_s"],
                cycle_s=p["cycle_s"])
        self.pid_cycle_s = p["cycle_s"]
        self._pid_div = max(1, int(round(p["cycle_s"] / TICK_DT)))
        h1, h2 = self.heaters["PulseHeat_Up"], self.heaters["Heat_Dw"]
        h1.rate = bond["pulse_heater_rate_cps"]
        h1.max_C = bond["head_heater_max_C"]
        h2.rate = bond["wafer_heater_rate_cps"]
        h2.max_C = bond["wafer_heater_max_C"]
        h2.cool_rate = bond["wafer_cool_rate_cps"]   # 主动冷却速率（迭代 11）

    def _tick_pid_locked(self, due):
        """三回路 PID 输出刷新（due=True 的 tick，默认每 4ms）。"""
        # 温度回路：active 时输出功率；inactive 功率 0 + 复位（防重新
        # 启动时旧积分尾巴）
        for key, hname in (("headtemp", "PulseHeat_Up"),
                           ("wafertemp", "Heat_Dw")):
            h = self.heaters[hname]
            loop = self.pid_loops[key]
            if h.active:
                if due:
                    if not loop.initialized:
                        loop.reset(out0=h.power, sp=h.target, pv=h.cur)
                    h.power = loop.update(h.target, h.cur)
            else:
                h.power = 0.0
                if loop.initialized:
                    loop.reset(0.0, 0.0, 0.0)
                    loop.initialized = False
        # 力回路：位置式输出=压入量 mm（正增益：F=k·压入量），
        # 轴位 = contact − 压入量。外部 move 偏离输出超 0.5mm 时无扰重挂
        # （bumpless reset 到当前压入量），防 PID 与 move 抢轴/瞬跳。
        # 2026-09-XX 迭代 29 力控模式门控：force_mode=False（普通上电/位控
        # 态）时整段旁路——不 update、不写轴，位置式输出不再把轴锚回接触
        # 位（修 <0.5mm 小步微调被 0.5mm re-anchor 阈值弹回的 BUG）。
        # 进入：bonding_force() 一律置位；此处 force_target!=0 兜底兼容
        # 直赋 force_target 的旧路径（i8.loop_force）。退出：sp=0 受控
        # 卸力完成后（弹簧模型 press≤0 → force_value 精确归 0）自动落位，
        # 退出点压入量≤0，位置连续零残压。
        if self.force_target != 0.0:
            self.force_mode = True
        if due and self.force_mode:
            if (self.force_target == 0.0 and self.force_value <= 0.0):
                self.force_mode = False
                loop = self.pid_loops["force"]
                if loop.initialized:
                    loop.reset(0.0, 0.0, 0.0)
                    loop.initialized = False
            else:
                ax = self.axes["BS_Arm_Z"]
                loop = self.pid_loops["force"]
                contact = self._bond["eff_contact_mm"]
                if not loop.initialized:
                    loop.reset(out0=contact - ax.pos, sp=self.force_target,
                               pv=self.force_value)
                out = loop.update(self.force_target, self.force_value)
                if not ax.moving and ax.home_state != "homing" and not ax.decel:
                    cmd = contact - out
                    if abs(ax.pos - cmd) > 0.5:
                        loop.reset(out0=contact - ax.pos, sp=self.force_target,
                                   pv=self.force_value)
                        cmd = contact - loop.out
                    ax.pos = max(ax.lo, min(ax.hi, cmd))
                    ax.target = ax.pos
        elif not self.force_mode:
            loop = self.pid_loops["force"]
            if loop.initialized:
                loop.reset(0.0, 0.0, 0.0)
                loop.initialized = False

    def pid_public(self):
        """三回路 pv/sp/out（+参数回显），FlowEngine.state_public 合入
        /api/flow/state（面板 PID 调试页轮询）。力回路 out=压入量 mm。"""
        with self.lock:
            h1 = self.heaters["PulseHeat_Up"]
            h2 = self.heaters["Heat_Dw"]

            def params(name):
                l = self.pid_loops[name]
                return {"kp": l.kp, "tn_s": l.tn_s, "tv_s": l.tv_s,
                        "td_s": l.td_s}

            return {"ok": True, "cycle_s": self.pid_cycle_s,
                    "force": dict({"pv": round(self.force_value, 3),
                                   "sp": round(self.force_target, 3),
                                   "out": round(self.pid_loops["force"].out,
                                                5),
                                   "active": self.force_mode,
                                   "out_unit": "mm（压入量）"},
                                  **params("force")),
                    "head_temp": dict({"pv": round(h1.cur, 2),
                                       "sp": round(h1.target, 2),
                                       "out": round(h1.power, 2),
                                       "active": h1.active,
                                       "out_unit": "%（功率）"},
                                      **params("headtemp")),
                    "wafer_temp": dict({"pv": round(h2.cur, 2),
                                        "sp": round(h2.target, 2),
                                        "out": round(h2.power, 2),
                                        "active": h2.active,
                                        "out_unit": "%（功率）"},
                                       **params("wafertemp"))}

    # ---------------- 电源分系统（M_PowerSwitch） ----------------
    def power_set(self, name, on):
        """分系统上/断电：更新状态位 + o<Name>_PowerOn/Off 与 GVL_IO 镜像。
        Fan/IonBar 用 base 名（观测 oFan/oIonBar）。"""
        if name not in self.power:
            raise KeyError("unknown power subsystem: %r" % name)
        with self.lock:
            self.power[name] = bool(on)
            if name in ("Fan", "IonBar"):
                outs = ((POWER_BASE + "o" + name,
                         "GVL_IO.o" + name),)
                vals = (self.power[name],)
            else:
                outs = ((POWER_BASE + "o%s_PowerOn" % name,
                         "GVL_IO.o%s_PowerOn" % name),
                        (POWER_BASE + "o%s_PowerOff" % name,
                         "GVL_IO.o%s_PowerOff" % name))
                vals = (self.power[name], not self.power[name])
            for (sm, gvl), v in zip(outs, vals):
                self.points.set(sm, v)
                if self.points.has(gvl):
                    self.points.set(gvl, v)
            return {"subsystem": name, "on": self.power[name]}

    # ---------------- 塔灯/蜂鸣器（E_SignalTower） ----------------
    def signal_tower_set(self, elem, on, flash=None):
        """手动点灯/蜂鸣：写 oLightOn/oBuzzOn + GVL_IO.oSignalTower_* 镜像。
        flash=(on_period, off_period) 来自 UpdateMode，仅记录供回读。
        注意：红/黄/绿每 tick 被自动逻辑覆盖（自动优先），蓝灯/蜂鸣器纯手动。"""
        if elem not in TOWER_ELEMENTS:
            raise KeyError("unknown tower element: %r" % elem)
        with self.lock:
            if flash is not None:
                self.tower_flash[elem] = [int(flash[0]), int(flash[1])]
            base = TOWER_BASE + elem
            self.points.set(base + "." + TOWER_ELEMENTS[elem], bool(on))
            gvl = "GVL_IO.oSignalTower_" + elem
            if self.points.has(gvl):
                self.points.set(gvl, bool(on))
            return {"element": elem, "on": bool(on),
                    "flash": self.tower_flash.get(elem)}

    # ---------------- 互锁命令（M_InterLock） ----------------
    def interlock_command(self, leaf, value):
        """AllDoors_Reset / AllDoors_Mask / InterLock_<Side>。
        Reset：门锁状态全部复位为关好；Mask：写屏蔽位（联动 doors 联锁）；
        InterLock_<Side>：该侧门锁使能开关（False=该侧门不参与联锁）。"""
        with self.lock:
            if leaf == "AllDoors_Reset":
                if value:
                    for sm, gvl in DOOR_MIRROR:
                        self._pair_cache.pop(sm, None)
                        self.points.set(sm, True)
                        if self.points.has(gvl):
                            self.points.set(gvl, True)
                obs = ("SystemMachine.SS_ME.M_InterLock.oAllDoors_Reset",
                       "GVL_IO.oInterLock_AllDoors_Reset")
            elif leaf == "AllDoors_Mask":
                obs = ("SystemMachine.SS_ME.M_InterLock.oAllDoors_Mask",
                       "GVL_IO.oInterLock_AllDoors_Mask")
            elif leaf.startswith("InterLock_"):
                side = leaf[len("InterLock_"):]
                if side not in self.door_side_enabled:
                    raise KeyError("unknown interlock side: %r" % leaf)
                self.door_side_enabled[side] = bool(value)
                obs = ("SystemMachine.SS_ME.M_InterLock.o" + leaf,
                       "GVL_IO.o" + leaf)
            else:
                raise KeyError("unknown interlock cmd: %r" % leaf)
            for p in obs:
                if self.points.has(p):
                    self.points.set(p, bool(value))
            return {"cmd": leaf, "value": bool(value),
                    "door_side_enabled": dict(self.door_side_enabled)}

    # ---------------- 相机/光源/校准（M_AS / M_WS） ----------------
    def camera_trigger(self, source):
        """相机触发：产生拍照记录（id/sim_t/触发源/全部轴位快照）+ 观测位。
        source ∈ AlignCameraTigger_1/2（精对准）、AS_CameraOn（粗对准）。
        尾部非阻塞通知视觉服务（触发→闪光→渲染→TCP→匹配→MQTT 全在视觉线程）。"""
        with self.lock:
            self._capture_seq += 1
            cap = {"id": self._capture_seq, "t": round(self.sim_time, 3),
                   "source": source,
                   "axes": {n: round(ax.pos, 4) for n, ax in self.axes.items()}}
            self.captures.append(cap)
            obs = "SystemMachine.SS_Bonding.M_AS.o" + source
            if self.points.has(obs):
                self.points.set(obs, True)
            vision = self.vision
        if vision is not None:
            try:
                vision.notify_trigger(source, cap)
            except Exception:
                pass                # 视觉链路异常绝不反噬物理线程
        return cap

    def camera_captures(self, limit=50):
        with self.lock:
            return list(self.captures)[-int(limit):]

    def wafer_heat_reset(self):
        """硅片加热复位：清加热盘报警位 + oWaferHeatReset 回执。"""
        with self.lock:
            self.points.set(
                "SystemMachine.SS_Bonding.M_WS.iHeaterPlatePowerCtrl_Alarm",
                False)
            self.points.set(
                "SystemMachine.SS_Bonding.M_WS.iHeaterPlate_TC_Alarm", False)
            self.points.set(
                "SystemMachine.SS_Bonding.M_AS.oWaferHeatReset", True)
            return {"alarms_cleared": True}

    def aram_calibrate(self):
        """键合臂位置传感器校准：点位表 H55/H56 同点（_2 映射到 _1），
        校准完成标志 oAramPosSensorCalibrate_1 置真（锁存）。"""
        with self.lock:
            self.points.set(
                "SystemMachine.SS_Bonding.M_WS.oAramPosSensorCalibrate_1",
                True)
            return {"calibrated": True}

    # ---------------- ES 流程命令（蓝膜台外包模块 M_ES） ----------------
    def es_command(self, leaf):
        """ES_Start/Stop/Reset/EmergencyStop/GvieBackDie/MoveToNextChip。
        流程编排不在仿真范围：只维护运行态 + EmergencyStop 等效 estop。"""
        with self.lock:
            if leaf == "ES_EmergencyStop":
                self.estop()
            elif leaf == "ES_Reset":
                self.reset()
                self.es_running = False
            elif leaf == "ES_Start":
                if self.estopped:
                    raise RuntimeError("estop active, reset first")
                self.es_running = True
            elif leaf == "ES_Stop":
                self.es_running = False
            elif leaf in ("ES_GvieBackDie", "ES_MoveToNextChip"):
                pass            # 单步工艺动作：接受并记录，无仿真行为
            else:
                raise KeyError("unknown ES cmd: %r" % leaf)
            return {"cmd": leaf, "es_running": self.es_running}

    # ---------------- 真空密封 / 吸附判定 ----------------
    def _sealed_locked(self, circuit):
        """吸盘是否被工件盖住（密封）。数据源在 workpieces 层。"""
        wp = self.workpieces
        if wp is None:
            return False
        if circuit == "WT_WaferVac":
            return bool(wp.wafer["present"])
        carriers = SEAL_CARRIERS.get(circuit)
        if not carriers:
            return False
        return any(d["carrier"] in carriers for d in list(wp.dies))

    def air_grip_ok(self, circuit):
        """grip_ok = 负压开 + 密封 + 压强低于阈值（真吸牢）。"""
        with self.lock:
            c = self.air.get(circuit)
            if c is None:
                raise KeyError("unknown air circuit: %r" % circuit)
            return (c.cmd_state == "neg" and c.supports_neg
                    and self._sealed_locked(circuit)
                    and c.pressure < AIR_GRIP_KPA)

    def air_detail(self, circuit):
        """vacuum_detail 单项：{on, pressure, flow, sealed, grip_ok}。"""
        with self.lock:
            c = self.air[circuit]
            sealed = self._sealed_locked(circuit)
            on = c.cmd_state == "neg" and c.supports_neg
            return {"on": on, "pressure": round(c.pressure, 2),
                    "flow": round(c.flow, 2), "sealed": sealed,
                    "grip_ok": bool(on and sealed
                                    and c.pressure < AIR_GRIP_KPA)}

    # ---------------- 在线轴参数 + 持久化 ----------------
    def set_axis_config(self, name_or_id, vmax=None, amax=None, jerk=None,
                        repeat=None, profile=None, limits=None,
                        home_switch=None, home_offset=None,
                        tau=None, settle_window=None, settle_time=None,
                        persist=True):
        names = ac.resolve_axis(name_or_id)
        if not names:
            raise KeyError("unknown axis: %r" % name_or_id)

        def _pos(v, label):
            v = float(v)
            if not (0.0 < v <= 1e6):
                raise ValueError("%s 须在 (0, 1e6] 内" % label)
            return v

        if limits is not None:
            if (not isinstance(limits, (list, tuple)) or len(limits) != 2):
                raise ValueError("limits 须为 [lo, hi]")
            lo, hi = float(limits[0]), float(limits[1])
            if not (lo < hi) or abs(lo) > 1e6 or abs(hi) > 1e6:
                raise ValueError("limits 非法：须 lo < hi")
        with self.lock:
            for n in names:
                ax = self.axes[n]
                if vmax is not None:
                    ax.vmax = _pos(vmax, "vmax")
                    ax.home_fast = ax.vmax * ac.HOME_SPEED_FAST_FACTOR
                    ax.home_slow = ax.vmax * ac.HOME_SPEED_SLOW_FACTOR
                if amax is not None:
                    ax.amax = _pos(amax, "amax")
                if jerk is not None:
                    ax.jerk = _pos(jerk, "jerk")
                if profile is not None:
                    if profile not in ("scurve", "trapezoid"):
                        raise ValueError(
                            "profile 只能是 scurve/trapezoid")
                    ax.profile_type = profile
                if limits is not None:
                    ax.lo, ax.hi = lo, hi
                    # 当前位置若在新限位外则钳位（x_actual 随位置重定义对齐）
                    if ax.pos > hi:
                        ax.pos = ax.target = ax.x_actual = hi
                        ax.pe = 0.0
                    elif ax.pos < lo:
                        ax.pos = ax.target = ax.x_actual = lo
                        ax.pe = 0.0
                if home_switch is not None:
                    ax.home_switch = float(home_switch)
                if home_offset is not None:
                    ax.home_offset = float(home_offset)
                if tau is not None:
                    ax.tau = _pos(tau, "tau")
                if settle_window is not None:
                    ax.settle_window = _pos(settle_window, "settle_window")
                if settle_time is not None:
                    ax.settle_time = _pos(settle_time, "settle_time")
        if repeat is not None:               # 显示单位 nm/µrad，存误差系统
            if self.errors is None:
                raise RuntimeError("error system not attached")
            for n in names:
                self.errors.set_repeat_display(n, repeat)
        if persist:
            self._record_overrides(names, vmax=vmax, amax=amax, jerk=jerk,
                                   repeat=repeat, profile=profile,
                                   limits=limits, home_switch=home_switch,
                                   home_offset=home_offset, tau=tau,
                                   settle_window=settle_window,
                                   settle_time=settle_time)
        return self.axis_config(names[0])

    def _record_overrides(self, names, **kv):
        provided = {k: v for k, v in kv.items() if v is not None}
        if not provided:
            return
        with self.lock:
            for n in names:
                self._overrides.setdefault(n, {}).update(provided)
            self._write_overrides_locked()

    def _write_overrides_locked(self):
        """原子写 config_overrides.json（axes 覆盖 + 顶层 plc_scan_ms）。"""
        tmp = self.overrides_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"axes": self._overrides,
                       "plc_scan_ms": self.plc_scan_ms}, f, indent=1)
            f.write("\n")
        os.replace(tmp, self.overrides_path)

    # ---------------- PLC 扫描周期 ----------------
    def set_plc_scan(self, ms, persist=True):
        """设置 PLC 扫描周期（1~50ms 整数）；持久化到 config_overrides.json。"""
        if isinstance(ms, bool) or (isinstance(ms, float) and not ms.is_integer()):
            raise ValueError("ms 须为 1~50 的整数")
        try:
            ms = int(ms)
        except (TypeError, ValueError):
            raise ValueError("ms 须为 1~50 的整数")
        if not (1 <= ms <= 50):
            raise ValueError("ms 须在 1~50 内")
        with self.lock:
            self.plc_scan_ms = ms
            if persist:
                self._write_overrides_locked()
            return self.plc_scan_ms

    def load_overrides(self, path=None):
        """启动时加载 config_overrides.json 覆盖 axes_config 默认值。"""
        path = path or self.overrides_path
        if not os.path.exists(path):
            return 0
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        n = 0
        for axis, cfg in (data.get("axes") or {}).items():
            if axis in self.axes and isinstance(cfg, dict):
                self.set_axis_config(axis, persist=False, **cfg)
                n += 1
        if data.get("plc_scan_ms") is not None:
            try:
                self.set_plc_scan(data["plc_scan_ms"], persist=False)
            except ValueError:
                pass                # 非法持久化值忽略，保持默认
        return n

    def axis_config(self, name):
        ax = self.axes[name]
        hp = ax.home_params()
        return {"vmax": ax.vmax, "amax": ax.amax, "jerk": ax.jerk,
                "limits": [ax.lo, ax.hi],
                "tau": ax.tau, "settle_window": ax.settle_window,
                "settle_time": ax.settle_time,
                "repeat": (round(self.errors.repeat_display(name), 4)
                           if self.errors else None),
                "repeat_unit": ac.REPEAT_DISPLAY_UNIT[
                    "rad" if ac.axis_is_rotary(name) else "mm"],
                "profile": ax.profile_type,
                "home": {"method": ax.home_mode,
                         "type": ac.home_type(ax.name),
                         "fast": ax.home_fast, "slow": ax.home_slow,
                         "switch": (ax.home_switch if ax.home_switch is not None
                                    else hp["switch"]),
                         "offset": (ax.home_offset if ax.home_offset is not None
                                    else hp["offset"])}}

    # ---------------- 快照 ----------------
    def snapshot_axes(self):
        with self.lock:
            out = []
            for i, row in enumerate(ac.AXES):
                ax = self.axes[row[0]]
                entry = {
                    "idx": i, "id": row[7], "name": ax.name,
                    "controller": ax.controller, "unit": ax.unit,
                    "pos": round(ax.pos, 6), "vel": round(ax.vel, 6),
                    "target": ax.target, "moving": ax.moving,
                    "in_position": ax.in_position, "enabled": ax.enabled,
                    "fault": ax.fault,
                    "homed": ax.homed, "limits": [ax.lo, ax.hi],
                    "home_state": ax.home_state, "home_phase": ax.home_phase,
                    "home_err": ax.home_err,
                    # 伺服环动态：跟随误差 pe / 伺服实际位置 x_actual / 轴状态
                    "pe": round(ax.pe, 9),
                    "x_actual": round(ax.x_actual, 9),
                    "state": ax.state,
                    # 双轨：value 名义值 / actual 误差注入后的实际值
                    "value": round(ax.pos, 9),
                    # settled = 彻底静止（settling 整定期不算）
                    "settled": ax.state == "idle",
                    "config": self.axis_config(ax.name),
                }
                if self.errors is not None:
                    # actual 基于伺服实际位置 x_actual 叠加误差；errormap 仍按
                    # 名义值 value 查表，repeat/scale 照旧（两机制正交）
                    actual, detail = self.errors.actual(ax.name, ax.pos,
                                                        base=ax.x_actual)
                    entry["actual"] = round(actual, 9)
                    entry["measured"] = detail["measured"]
                    entry["errormap"] = {"enabled": detail["enabled"],
                                         "coverage": detail["coverage"]}
                else:
                    entry["actual"] = entry["value"]
                    entry["measured"] = False
                    entry["errormap"] = {"enabled": False, "coverage": None}
                out.append(entry)
            return {"ok": True, "sim_time": self.sim_time, "t": self.sim_time,
                    "speed_scale": self.speed_scale, "estop": self.estopped,
                    "interlock": {"enabled": self.interlock_enabled,
                                  "emo": self.emo_latched,
                                  "doors_ok": self.doors_ok,
                                  "masked": self.interlock_masked},
                    "axes": out}

    def viz(self):
        """/api/viz 聚合：3D/面板用的工件 + 真空 + 联锁 + 塔灯一屏视图。
        vacuum 保持 bool 字典不动（前端联机注入.js 依赖）；
        密封/吸附细节走新增的 vacuum_detail。"""
        with self.lock:
            vacuum = {n: bool(self.points.get(
                AIR_BASE + n + ".oNegtiveState")) for n in VIZ_VACUUM}
            vacuum_detail = {n: self.air_detail(n) for n in VIZ_VACUUM}
            tower = {k: bool(self.points.get(p))
                     for k, p in TOWER_POINTS.items()}
            out = {"ok": True, "t": self.sim_time, "vacuum": vacuum,
                   "vacuum_detail": vacuum_detail,
                   "wafer": None, "dies": [], "transfer_log": [],
                   "interlock": {"enabled": self.interlock_enabled,
                                 "emo": self.emo_latched,
                                 "doors_ok": self.doors_ok,
                                 "masked": self.interlock_masked},
                   "tower": tower,
                   "power": dict(self.power),
                   # 脉冲加热器（PulseHeat_Up，3D 前端发光显示用，confocal
                   # 字段同款）：温度/目标/功率%
                   "heater": {"temp_C": round(
                                  self.heaters["PulseHeat_Up"].cur, 2),
                              "target_C": round(
                                  self.heaters["PulseHeat_Up"].target, 2),
                              "power_pct": round(
                                  self.heaters["PulseHeat_Up"].power, 2),
                              "active": self.heaters["PulseHeat_Up"].active},
                   "confocal": (self.confocal.measuring_public()
                                if self.confocal is not None else None)}
        if self.workpieces is not None:
            out.update(self.workpieces.viz())
        return out

    def health(self, clients=0):
        with self.lock:
            return {"ok": True, "build": BUILD_TAG,
                    "tick_hz": round(self.tick_hz, 1),
                    "tick_jitter_avg_us": round(self.jitter_avg_us, 1),
                    "tick_jitter_max_us": round(self.jitter_max_us, 1),
                    "push_hz": round(self.push_hz, 1), "seq": self.seq,
                    "sim_time": round(self.sim_time, 3),
                    "speed_scale": self.speed_scale, "estop": self.estopped,
                    "ws_clients": clients, "axes": len(self.axes),
                    "io_points": len(self.points),
                    "plc_scan_ms": self.plc_scan_ms,
                    # 故障注入状态（W2）：沉默/断线目标 + fault 轴列表
                    "faults": {"silent": sorted(self.silent),
                               "dropped": sorted(self.dropped),
                               "fault_axes": sorted(
                                   n for n, a in self.axes.items() if a.fault)},
                    # 沙盒干涉检查（W5）：loaded=False 表示 sandboxes.json 未加载
                    "collision": ({"enabled": self.collision_enabled,
                                   "loaded": True,
                                   "boxes": len(self.collision.boxes),
                                   "pairs": len(self.collision.pairs),
                                   "active": len(self.collision.overlapping)}
                                  if self.collision is not None else
                                  {"enabled": False, "loaded": False})}

    # ---------------- DI 写通路（回零联动） ----------------
    def _set_di(self, name, value):
        """统一 DI 写通路：缓存上次值，变化才写点位表。"""
        value = bool(value)
        if self._di_cache.get(name) != value:
            self._di_cache[name] = value
            self.points.set(name, value)

    def _tick_home_di_locked(self):
        """每 tick 更新回零联动 DI：
        - iZeroEpin_Z：三根顶针 |pos| 均 ≤ EPIN_ZERO_TOL 时置真
        - iRx/iRyZeroSignal：|pos| ≤ 零位窗口
        - iRx/iRyPosSignal/NegSignal：pos ≥ hi / pos ≤ lo（限位触发）
        """
        self._set_di(ac.DI_EPIN_ZERO,
                     all(abs(self.axes[n].pos) <= ac.EPIN_ZERO_TOL
                         for n in ac.HOME_TYPE_PLC_SYNC_TRIO))
        for n in ac.HOME_TYPE_ZERO_SENSOR:
            ax = self.axes[n]
            self._set_di(ac.DI_CAL_ZERO[n],
                         abs(ax.pos) <= ac.ZERO_SENSOR_WINDOW)
            self._set_di(ac.DI_CAL_POS[n], ax.pos >= ax.hi)
            self._set_di(ac.DI_CAL_NEG[n], ax.pos <= ax.lo)

    # ---------------- tick ----------------
    def tick(self):
        """推进一个 1ms 仿真 tick。"""
        k_air = TICK_DT / AIR_TAU
        with self.lock:
            for ax in self.axes.values():
                if ax.decel:
                    self._tick_free_decel_locked(ax)   # 掉使能自由减速优先
                    continue
                if ax.home_state == "homing":
                    self._tick_homing_locked(ax)   # 回零序列逐 tick 推进
                    continue
                if not ax.moving or ax.profile is None:
                    continue
                ax.prof_i += 1
                if ax.prof_i >= len(ax.profile):
                    # 规划到位 ≠ 实际到位：进入 settling，由伺服环按 pe 收敛判定
                    ax.pos = ax.target
                    ax.vel = 0.0
                    ax.moving = False
                else:
                    prev = ax.pos
                    ax.pos = max(ax.lo, min(ax.hi, ax.start + ax.profile[ax.prof_i]))
                    ax.vel = (ax.pos - prev) / TICK_DT
            for ax in self.axes.values():
                self._tick_servo_locked(ax)
            self._tick_modal_locked()
            self._tick_buffer_locked()
            self._tick_air_locked(k_air)
            if self.sim_time - self._const_refresh_sim >= 1.0:
                # 机器常数 bond/pid 组热改重读（每秒一次，热生效）
                self._refresh_loop_consts_locked()
                self._const_refresh_sim = self.sim_time
            self._tick_pid_locked(self._tick_count % self._pid_div == 0)
            self._tick_heaters_locked()
            self._tick_thermal_locked()
            self._tick_force_locked()
            self._tick_home_di_locked()
            self._tick_interlock_locked()
            self._tick_collision_locked()
            if self.workpieces is not None:
                # 工件交接判定（O(规则数×(真空点数+姿态轴数))，可忽略）
                self.workpieces.check_transfers(self, self.points, self.sim_time)
            self.sim_time += TICK_DT
            self._tick_count += 1
            # PLC 扫描：每 plc_scan_ms 个 tick 提交一次点位输入映像
            if self._tick_count % max(1, int(self.plc_scan_ms)) == 0:
                self.points.commit()
            # WS 帧推送与 tick 解耦（迭代 11c 倍速保护）：scale≤1 维持原
            # 每 2 tick 一帧（500Hz 红线语义不变）；scale>1 改按墙钟节流
            # ≤500Hz——否则 ×3 会推到 1500Hz，物理线程内 sendall 与浏览器
            # 客户端都吃不消。
            if self.speed_scale <= 1.0:
                frame = self._build_frame() if self._tick_count % 2 == 0 \
                    else None
            else:
                now_wall = time.perf_counter()
                if now_wall - self._last_frame_wall >= 0.0019:
                    self._last_frame_wall = now_wall
                    frame = self._build_frame()
                else:
                    frame = None
        if frame is not None:
            self._win_frames += 1
            for fn in self._frame_listeners:
                try:
                    fn(frame)
                except Exception:
                    pass

    def _tick_free_decel_locked(self, ax):
        """掉使能自由减速（每 tick）：按 ax.decel 减速度对速度积分到静止
        （梯形积分保持位置连续），减速完成即进 fault 态（真机掉使能即报警）；
        到位标志不直接置位，交 _tick_servo_locked 按 pe 收敛。"""
        v = ax.vel
        if v > 0.0:
            nv = max(0.0, v - ax.decel * TICK_DT)
        elif v < 0.0:
            nv = min(0.0, v + ax.decel * TICK_DT)
        else:
            nv = 0.0
        ax.pos = max(ax.lo, min(ax.hi, ax.pos + 0.5 * (v + nv) * TICK_DT))
        ax.vel = nv
        if nv == 0.0:
            ax.moving = False
            ax.decel = 0.0
            ax.fault = True
            ax.in_position = False
            self._notify_settled(ax, True)

    def _tick_servo_locked(self, ax):
        """伺服环动态（每 tick，所有轴）：x_actual 一阶滞后跟随名义值 pos，
        pe = pos - x_actual 为跟随误差；轨迹跑完（moving=False）后进入
        settling，|pe| 在 settle_window 内持续 settle_time 才判 in_position。
        estop/停止冻结 pos 后 x_actual 自然泄放收敛，无需特殊处理。"""
        ax.x_actual += (ax.pos - ax.x_actual) * (TICK_DT / ax.tau)
        ax.pe = ax.pos - ax.x_actual
        if ax.moving or ax.home_state == "homing":
            ax.state = "moving"
            ax.in_position = False
            ax.settle_acc = 0.0
            return
        if ax.in_position:
            ax.state = "idle"
            ax.settle_acc = 0.0
            return
        ax.state = "settling"
        if abs(ax.pe) < ax.settle_window:
            ax.settle_acc += TICK_DT
            if ax.settle_acc >= ax.settle_time:
                # 到位：残余 pe 已小于窗口，直接对齐（读数稳定，不再漂移）
                ax.x_actual = ax.pos
                ax.pe = 0.0
                ax.in_position = True
                ax.state = "idle"
                ax.settle_acc = 0.0
                self._notify_settled(ax, True)
        else:
            ax.settle_acc = 0.0

    def _tick_modal_locked(self):
        """全机模态仿真推进（2026-09-08 迭代 22）：伺服环后每 tick 解析
        传播 SISO/MIMO 模态（轴加速度由 modal 内部按 ax.vel 差分限幅）。
        未挂载或未启用零开销早退（默认 OFF，POST /api/modal 热开）。"""
        m = self.modal
        if m is None or not m.enabled:
            return
        m.tick(TICK_DT, self)

    def _tick_thermal_locked(self):
        """组件级热仿真推进（2026-09-09 迭代 23）：加热器 tick 后分频解算
        LPTN 热网络（2~20Hz 自适应，内部按构建期 div 分频），消费 heater
        power%/cur 与轴活动度；未挂载或未启用零开销早退（默认 OFF，
        POST /api/thermal 热开，OFF 状态保持）。"""
        t = self.thermal
        if t is None or not t.enabled:
            return
        t.tick(TICK_DT, self)

    def _tick_buffer_locked(self):
        if not self.buf_running or self.estopped:
            return
        steps = BUFFERS[self.buf_num]["steps"]
        if self.buf_step >= len(steps):
            self.buf_running = False
            return
        name, target, speed = steps[self.buf_step]
        ax = self.axes[name]
        if not self.buf_step_started:
            try:
                self._start_move_locked(ax, target, speed)
            except (InterlockError, RuntimeError):
                self.buf_running = False        # 联锁/急停拒绝：buffer 中止
                return
            self.buf_step_started = True
        elif ax.in_position:
            self.buf_step += 1
            self.buf_step_started = False
            if self.buf_step >= len(steps):
                self.buf_running = False

    def _tick_air_locked(self, k):
        for c in self.air.values():
            sealed = self._sealed_locked(c.name)
            if c.cmd_state == "pos":
                p_t, f_t = AIR_POS_KPA, AIR_POS_FLOW
            elif c.cmd_state == "neg":
                # 密封=深真空小流量；开放=浅真空大流量（抽不动）
                p_t = AIR_NEG_KPA_SEALED if sealed else AIR_NEG_KPA_OPEN
                f_t = AIR_NEG_FLOW_SEALED if sealed else AIR_NEG_FLOW_OPEN
            else:
                p_t, f_t = 0.0, 0.0
            c.pressure += (p_t - c.pressure) * k
            c.flow += (f_t - c.flow) * k
            base = AIR_BASE + c.name
            pos_st = c.pressure > 0.5 * AIR_POS_KPA
            neg_st = c.supports_neg and c.pressure < 0.5 * AIR_NEG_KPA_SEALED
            self.points.set(base + ".oPositiveState", pos_st)
            self.points.set(base + ".oNegtiveState", neg_st)
            self.points.set(base + ".oAirState", not pos_st and not neg_st)
            self.points.set(base + ".PressureSwitch.oValue", round(c.pressure, 2))
            self.points.set(base + ".FlowMeter.oValue", round(c.flow, 2))

    def _tick_heaters_locked(self):
        """热对象（每 tick 积分；功率由 _tick_pid_locked 按 4ms 周期刷新）：
        dT/dt = 功率%×满功率速率 − rate·(T−amb)/(max−amb)（热损一阶项，
        满功率平衡点=量程上限），再按 ±rate 限速、夹到 [amb, max]；降温
        方向（active 且目标<当前）且配置了 cool_rate 的加热器改用主动冷却
        线性速率（迭代 11，Heat_Dw ← tuning wafer_cool_rate_cps）。
        命令面语义不变：active=加热计划开关，finish=到温（±0.5℃ 带），
        remaining=到温预估+保温倒计时；oCurTemp 读数加 ±temp_noise_C 噪声
        （recipe sim_sensors；真值 h.cur 不含噪，流程等温判定用真值）。"""
        amb = self._bond["ambient_C"]
        noise_cfg = self._bond["temp_noise_C"]
        gain_err_cfg = self._bond.get("temp_gain_err") or {}
        off_err_cfg = self._bond.get("temp_offset_C") or {}
        calib = self._bond.get("sensor_calib") or {}
        # 传感链注入/补偿（迭代 49）：raw=真值×(1+gain_err)+offset_err+noise
        # （gain_err/offset_err=recipe sim_sensors 真值旋钮）→ 发布读数=
        # (raw−offset_cal)/gain_cal（机器常数 sensor_calib 组，标定后补偿
        # 到真值）。**PID 控制仍用真值 h.cur**（本函数上行），只有发布读数
        # 带注入/补偿——控制行为不变
        for h in self.heaters.values():
            # 温度噪声逐加热器分路（2026-09-08 迭代 22）；旧标量（机械常数
            # bond 段兜底路径）全加热器同值
            noise = float(noise_cfg.get(h.name, 0.5)) \
                if isinstance(noise_cfg, dict) else noise_cfg
            span = max(1.0, h.max_C - amb)
            # 主动冷却（2026-09-02 迭代 11）：配置了 cool_rate 的加热器
            # （Heat_Dw）在降温方向（active 且目标<当前）按 cool_rate 线性
            # 逼近目标，替代自然热损对称速率（模拟真实设备风冷/水冷）；
            # 升温方向与未配置 cool_rate 的加热器模型不变
            cooling = h.active and h.cool_rate and h.target < h.cur
            if cooling:
                dT = -float(h.cool_rate)
            else:
                heat = (h.power / 100.0) * h.rate if h.active else 0.0
                loss = h.rate * (h.cur - amb) / span
                dT = heat - loss
            if not cooling:
                if dT > h.rate:
                    dT = h.rate
                elif dT < -h.rate:
                    dT = -h.rate
            h.cur += dT * TICK_DT
            if cooling and h.cur < h.target:      # 主动冷却不过冲目标
                h.cur = h.target
            if h.cur > h.max_C:
                h.cur = h.max_C
            elif h.cur < amb:
                h.cur = amb
            if h.active:
                if abs(h.cur - h.target) <= 0.5:
                    if h.remaining > 0:
                        h.remaining = max(0.0, h.remaining - TICK_DT)
                    h.finish = True
                else:
                    h.finish = False
                    if cooling:
                        h.remaining = (h.cur - h.target) / float(h.cool_rate) \
                            + h.steady_time
                    else:
                        h.remaining = max(0.0, (h.target - h.cur)) / h.rate \
                            + h.steady_time
            else:
                h.finish = False
                h.remaining = 0.0
            # 传感链读数（迭代 49）：raw=真值×(1+增益误差)+零偏+噪声；
            # 发布=(raw−offset_cal)/gain_cal（标定补偿；增益标定恒非零，
            # 机器常数校验域 [0.5,1.5]，上方载入处另有守卫）
            ge = float(gain_err_cfg.get(h.name, 0.0))
            oe = float(off_err_cfg.get(h.name, 0.0))
            if h.name == "PulseHeat_Up":
                g_cal = float(calib.get("head_temp_gain", 1.0))
                o_cal = float(calib.get("head_temp_offset_c", 0.0))
            else:
                g_cal = float(calib.get("wafer_temp_gain", 1.0))
                o_cal = float(calib.get("wafer_temp_offset_c", 0.0))
            raw = h.cur * (1.0 + ge) + oe \
                + (self._noise.uniform(-noise, noise) if noise > 0.0 else 0.0)
            rd = (raw - o_cal) / g_cal if abs(g_cal) > 1e-12 else raw
            base = HEATER_BASE + h.name
            self.points.set(base + ".oCurTemp", round(rd, 2))
            self.points.set(base + ".PreHeatFinish", h.finish)
            self.points.set(base + ".PreHeatTime", round(h.remaining, 3))

    def _tick_force_locked(self):
        """键合力（弹簧接触模型）：BS_Arm_Z 低于有效接触位（裸接触位+晶圆
        厚+芯片厚，effective_bond_contact_mm，随处方热改）时真值力
        F=k·压入量（k=tuning bond_hw.force_spring_k_n_per_mm），否则 0；
        轴位由力回路 PID（_tick_pid_locked）按 setpoint 微调。
        flip 交接接触支路（迭代 49）：flip 上有 die 且 FA_X 在交接位
        （|FA_X−flip_handoff.fa_x_handoff_mm|≤1.0）且 BS_Arm_Z 进入交接
        邻域（pos<flip_eff+2）时，flip 有效接触位=tuning bond_hw.
        flip_contact_z_mm（flip 吸盘裸面在 BS_Arm_Z 坐标系的 z，真值
        旋钮）+处方芯片厚，压入力 F=k·press 与键合支路取 max。
        oActualForce 读数链（迭代 49）：raw=真值力×(1+force_gain_err)+
        force_offset_N+±force_noise_N 噪声（recipe sim_sensors 注入），
        发布=(raw−force_offset_n)/force_gain（机器常数 sensor_calib 组
        标定补偿；力回路 PID 控制仍用真值 force_value，不读本链）。"""
        ax = self.axes["BS_Arm_Z"]
        contact = self._bond["eff_contact_mm"]
        k = self._bond["force_spring_k_n_per_mm"]
        press = contact - ax.pos
        # flip 交接接触支路（缓存键 1s 重读，见 _refresh_loop_consts_locked）
        press_flip = 0.0
        if self._bond.get("flip_has_die"):
            flip_eff = float(self._bond["flip_contact_z_mm"]) \
                + float(self._bond.get("chip_thickness_mm", 0.0)) \
                - float(self._bond.get("therm_dz_head_mm", 0.0))  # 头热伸长
                # →交接接触同机理提前（迭代 56，与键合支路同符号）
            if ax.pos < flip_eff + 2.0 and abs(
                    self.axes["FA_X"].pos
                    - float(self._bond["fa_x_handoff_mm"])) <= 1.0:
                pf = flip_eff - ax.pos
                if pf > 0.0:
                    press_flip = pf
        press_eff = max(press if press > 0.0 else 0.0, press_flip)
        self.force_value = k * press_eff if press_eff > 0.0 else 0.0
        # 接触沿 → 模态接触脉冲（2026-09-08 迭代 22）：press 由非正转正
        # 触发一次（抬起再接触重新触发），给 head/wafer 相关模态速度冲量
        in_contact = press_eff > 0.0
        if in_contact and not self._contact_prev:
            m = self.modal
            if m is not None and m.enabled:
                m.contact_pulse()
        self._contact_prev = in_contact
        noise = self._bond["force_noise_N"]
        # 力读数链（迭代 49）：raw=真值×(1+增益误差)+零偏+噪声 → 发布=
        # (raw−force_offset_n)/force_gain（sensor_calib 标定补偿）
        calib = self._bond.get("sensor_calib") or {}
        fge = float(self._bond.get("force_gain_err", 0.0))
        foe = float(self._bond.get("force_offset_N", 0.0))
        raw = self.force_value * (1.0 + fge) + foe \
            + (self._noise.uniform(-noise, noise) if noise > 0.0 else 0.0)
        g_cal = float(calib.get("force_gain", 1.0))
        o_cal = float(calib.get("force_offset_n", 0.0))
        rd = (raw - o_cal) / g_cal if abs(g_cal) > 1e-12 else raw
        self.points.set("SystemMachine.SS_Bonding.M_BS.oActualForce",
                        round(rd, 3))
        self.points.set("SystemMachine.M_ACS_Buffer.oBufferDo", self.buf_running)
        self.points.set("SystemMachine.M_ACS_Buffer.oBufferNum",
                        self.buf_num if self.buf_running else 0)
        # 蓝膜台外包模块位置观测
        self.points.set("SystemMachine.SS_Bonding.M_ES.oX_CurrentPos",
                        round(self.axes["ES_X"].pos, 4))
        self.points.set("SystemMachine.SS_Bonding.M_ES.oY_CurrentPos",
                        round(self.axes["ES_Y"].pos, 4))

    # ---------------- 主循环 ----------------
    def run(self):
        """物理线程：1kHz × 倍速 的墙钟速率驱动 tick。"""
        try:
            winmm = ctypes.windll.winmm
            winmm.timeBeginPeriod(1)
        except Exception:
            winmm = None
        try:
            # 真机实时线程语义：物理线程提级（THREAD_PRIORITY_HIGHEST），
            # 压过视觉渲染/匹配等重计算线程的 GIL/调度竞争；非 Windows 或
            # 失败静默降级。坑：必须 c_void_p 收伪句柄——默认 c_int 会把
            # GetCurrentThread 的 -2 零扩展成 0xFFFFFFFE，SetThreadPriority
            # 静默失败（已实测验证）。
            k32 = ctypes.windll.kernel32
            k32.GetCurrentThread.restype = ctypes.c_void_p
            k32.SetThreadPriority(k32.GetCurrentThread(), 2)
        except Exception:
            pass
        self._running = True
        next_t = time.perf_counter()
        last_stat = next_t
        while self._running:
            with self.lock:
                scale = self.speed_scale
            next_t += TICK_DT / scale
            spin = 0
            while True:
                rem = next_t - time.perf_counter()
                if rem <= 0:
                    break
                if rem > 0.0012:
                    time.sleep(min(rem - 0.001, 0.01))
                else:
                    spin += 1
                    if spin % 50 == 0:
                        time.sleep(0)      # 让出 GIL，避免饿死发送线程
            t0 = time.perf_counter()
            if self._last_tick_t is not None:
                self._intervals.append(t0 - self._last_tick_t)
            self._last_tick_t = t0
            try:
                self.tick()
            except Exception:
                # 物理线程红线：单拍异常只记录不死亡（节流 5s 一条；
                # 打印本身也可能撞断管——守卫，绝不再抛）
                if t0 - self._last_tick_err >= 5.0:
                    self._last_tick_err = t0
                    try:
                        traceback.print_exc()
                    except Exception:
                        pass
            self._win_ticks += 1
            if t0 - next_t > 0.02:          # 落后超 20ms 丢弃积压，防追帧突发
                next_t = time.perf_counter()
            if t0 - last_stat >= 5.0:
                self._update_stats(t0 - last_stat)
                last_stat = t0
        if winmm:
            try:
                winmm.timeEndPeriod(1)
            except Exception:
                pass

    def _update_stats(self, window):
        self.tick_hz = self._win_ticks / window
        self.push_hz = self._win_frames / window
        self._win_ticks = 0
        self._win_frames = 0
        if self._intervals:
            target = TICK_DT / max(0.1, self.speed_scale)
            dev = [abs(i - target) for i in self._intervals]
            self.jitter_avg_us = 1e6 * sum(dev) / len(dev)
            self.jitter_max_us = 1e6 * max(dev)
        try:
            print("[engine] tick=%.1fHz jitter avg=%.0fus max=%.0fus | push=%.1fHz "
                  "sim_t=%.1fs scale=%.2f" %
                  (self.tick_hz, self.jitter_avg_us, self.jitter_max_us,
                   self.push_hz, self.sim_time, self.speed_scale), flush=True)
        except OSError:
            # stdout 管道断裂（宿主退出/读端关闭 → BrokenPipeError）：统计行
            # 可丢，物理线程绝不可随打印死亡（迭代 47 仿真冻结事件根因）
            pass

    def shutdown(self):
        self._running = False
