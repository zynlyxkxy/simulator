# -*- coding: utf-8 -*-
"""flow_engine.py — 工艺流程编排引擎（W4）。

定位：把将来 L2 主控层的"工艺流程"活先在仿真里干起来——用户在面板/flows.json
里定义步骤序列，引擎顺序执行；上位机流程定义到位后，flows.json 可整体退役
或降级为测试脚本（见 docs/仿真实现原理.md "工艺流程编排"一节）。

- 流程定义存 runtime/flows.json（用户可编辑；启动加载，PUT /api/flows 在线改，
  POST /api/flow/restore_example 恢复内置示例）。
- 单实例挂 engine.flow（main.py 启动），独立线程顺序执行；同一时刻只跑一条
  流程。estop / 联锁 / 掉使能导致的指令拒绝 → 流程 abort，记录失败步骤与原因
  （W2 故障注入的联动价值：工艺流程会被真实故障打断）。
- 状态机：idle / running / paused / done / aborted。
  pause 在步骤边界生效（不打断进行中的步骤）；stop 立即 abort 并全轴 stop；
  step 仅在 paused 态推进一步（run 可带 paused=true 起步即单步模式）。
- 状态经 HTTP API 查询（面板 5Hz 轮询），不走 WS 二进制帧。

步骤类型（op 字段；所有步骤可带 desc 中文字段供面板显示）：
  move      {axis, target, speed?, timeout?}   发 move 并等 in_position
              （含伺服 settling；等待中轴进 fault 立即判失败）
  home      {axis 或 "ALL", timeout?}          回零并等 home_state==done
  io        {path, cmd, args?}                 走 plc_command 命令路由
              （气路/塔灯/光源/温控/电源/互锁全能用；PLC 沉默时记"被吞"）
  wait_grip {circuit, expect, timeout?, err?}  等 grip_ok（真空吸附判定；
              err=自定义失败文本，超时/失败时以此报警，如取片拾取判定）
  wait_io   {path, expect, timeout?}           等任意观测位等于 expect
  wait_die  {carrier, timeout?, err?}          等任一 die 载体变为 carrier
              （film/flip/head/wafer；交接是否真实发生的确认步）
  transfer  {rule, timeout?}                   宏步骤：按 transfers.json 规则名
              执行一次交接——建立接收侧真空 → 驱动姿态轴到示教位（按文件顺序）
              → 释放来源侧真空 → 等 die 载体确实变化；超时按当前条件逐项诊断
  camera    {source}                           相机触发（AlignCameraTigger_1/2、
              AS_CameraOn），产生拍照记录
  coarse_offset {cam?, err?}                   粗对准测量：拍照识别 die 背面
              特征像素 → 反投影世界坐标（像素当量/方向性/安装微旋按处方逆
              映射，焦面平面）→ 与机械常数标定中心 coarse_die_center_mm
              对比，偏差写流程日志与步骤信息（对准策略的数据输入）
  delay     {ms}                               延时（可被 stop 打断）
  if        {cond:{io:path,expect:val} 或 {grip:circuit,expect:bool},
             then:[...], else:[...]?}          条件分支，最多嵌套一层
  log       {msg}                              写流程日志
  wafer     {action:"load"|"unload", offset?, rz?, z?, rx?, ry?,
             inject_recipe?}                工件上下片（等价面板手动上片；
              inject_recipe:true=按处方 wafer_load 注入误差，显式字段
              覆盖注入默认；load 幂等：已在位则跳过）
  film      {action:"load"|"unload"}           蓝膜上下机（load 幂等：置在位
              + 按处方重铺 die；unload 时 film 上剩余 die 随机移除）
  fl_measure {probe:"up"|"down", point:[x,y],
              space:"wafer"|"die"|"stage",
              aim?:[sx,sy], align_first?:bool,
              timeout?}                        调焦调平共焦测高：组合走位使
              共焦探头对准目标点（走位全走"相机光轴+confocal_cam_dx/dy_mm
              标定偏移"认知链；wafer=晶圆设计坐标，aim=EGA 映射+dx/dy 补偿
              后的对准系坐标，缺省运行时取 eng.ega_result 现算；die=相对
              die 中心偏移，仅动 AS；stage=世界 XY，仅动 AS；align_first=
              先相机 identify 实测标记再送探头到实测点）
              → 读共焦（MQTT 触发→订阅消费，契约见数据协议 §12）→ 无读数时
              AS_Z 按 focus_search_span/step 扫描找
              有效窗 → 面高换算（读数+焦面，与搜索位无关）后恢复 AS_Z
              → 记录到 eng.fl_measurements[space]
  fl_solve  {mode:"wafer"|"die"|"compensate"}  调焦调平解算：wafer/die=
              对 fl_measurements 最小二乘拟合平面 z=a·x+b·y+c（wafer 系
              空间在晶圆台基准系 plane_xy、其余世界系，≥3 点）；compensate=原子补偿：BS_CAL_Rx/Ry 使芯片面∥晶圆面
              + AS_Z 定下相机焦面=测点中心晶圆面 + BS_Arm_Z 定上相机焦面
              =芯片中心面（各调一次，不反复动）→ 残余 urad 与焦面残差写
              日志，补偿结果存 eng.level_memory（global 模式复用）；
        fl_verify=调平复测验证：拟合复测点（wafer_v/die_v）→ 残余坡度
              与 level_tol_urad 判定，结果存 eng.fl_fit["verify"]，FAIL
              抛错流程 abort（迭代 5）
  fa_measure {site_x, site_y,
              wafer_pts:[[x,y]...],
              die_pts:[[x,y]...],
              timeout?}                        精对准测量（迭代 6）：WS/BS
              锁轴快照校验（测量前后比对，动则抛错）→ AS-only 逐标记寻址
              （晶圆侧=EGA map+WS 冻结补偿，die 侧=as_head_center+角标偏
              移）→ 双侧各 N 点相机 identify+solve_pixel → eng.fa_meas
  fa_solve  {}                               双侧 fit（算法=处方
              fine_align.algorithm：rigid/similarity/affine/ransac；
              n=2 强制拒绝 affine）→ Δ=T_d∘T_w⁻¹=(tx,ty,θ) → eng.fa_delta
  fa_compensate_confirm {site_x, site_y,
              wafer_pts, die_pts, timeout?}    补偿+确认闭环：先 Rz 后 XY
              （θ 经 rz_gain/rz_sign，枢轴=吸盘心世界+rz_center_offset_mm，
              耦合 coup=R(δθ)(p−c)−(p−c) 进平移；WS_Rz ±1° 行程守卫；
              旋转复合进 ega_result 仿射保持后续寻址）→ 确认环逐轮
              复测-重fit-残余判定（confirm_tol_xy_um/confirm_tol_rz_urad，
              confirm_max_iter，超次按 on_unconfirmed=abort|continue_warn）
              → 物理残差双通道 → eng.fine_align_result（轮次历史+judge）
  heater    {name, action:"set"|"wait"|"off",
             temp?, tol_c?, timeout?}        温控宏步（迭代 8）：set=设定目标
              温度并等到 ±tol_c 带（默认 2℃）；wait=只对当前目标等到温
              （硅片台常开保温的幂等确认）；off=StopTargetTempWithPlan
  force     {target, tol_n?, timeout?}       键合力宏步（迭代 8）：设力回路
              setpoint（BondingByForceManual）并等真值力到 ±tol_n 带
              （默认 1N）；target=0=释放
  bond_mark {site_x, site_y}                 die 落位修正（迭代 8）：把最近
              head_to_wafer 交接的 die 的 site 从 transfers.json [0,0]
              占位改写为当前键合位设计坐标（已键合标记/3D 显示消费）
  bond_truth {}                              键合接触时刻真值采样（迭代 20，
              迭代 20 起 build_bond_flow 插在 force_search 之后）：触力停轴
              后用 6DOF 真值位姿走精对准同一 fit 管线（fit_rigid 双侧+
              relative_delta）算 die↔site 真值残差——键合头下降链（BS_Arm_Z
              地图差+阿贝）自然进入；写 eng.bond_commit，bond_mark 消费
"""
import copy
import json
import math
import os
import threading
import time
from collections import deque

import axes_config as ac
import ega
import fine_align
from engine import AIR_BASE, InterlockError
from plc_http import CAMERA_TRIGGERS, plc_command

FLOW_OPS = ("move", "home", "io", "wait_grip", "wait_io", "wait_die",
            "transfer", "camera", "coarse_offset", "delay", "if", "log",
            "wafer", "film", "ega_measure", "ega_fit",
            "fl_ready", "fl_measure", "fl_solve", "fl_verify",
            "fa_measure", "fa_solve", "fa_compensate_confirm",
            "heater", "force", "force_search", "bond_mark", "bond_truth",
            "bond_offset_apply", "pi_measure")
DEFAULT_MOVE_TIMEOUT = 30.0     # move/transfer 到位等待默认超时（s）
DEFAULT_WAIT_TIMEOUT = 10.0     # wait_grip/wait_io 默认超时（s）
DEFAULT_HOME_TIMEOUT = 60.0     # 回零默认超时（s）
MAX_IF_DEPTH = 2                # if 最多嵌套一层（顶层算第 1 层）
LOG_MAX = 200                   # 流程日志环形缓冲条数
# PI 单对套刻 drz 物理门限（2026-09-09 迭代 23）：EGA+精对准后真值 drz
# 不可能超过确认容差（默认 30~60µrad），|drz_w|>500µrad 必是角度测量伪
# 差（REAL 几何角离群）——旋转修正项（杠杆 |d0|≈3.8mm，伪差 ×3.8mm 会
# 把 ±10µm 假误差注进 measured dx/dy）按 0 处理；measured drz 仍按实测
# 值落记录（伪差对人工可见、不进 xy）
PI_DRZ_GATE_RAD = 500e-6

# 内置示例流程（flows.json 缺失时由它播种；restore_example 恢复成它）。
# 交接示教位一律引用 transfers.json 规则里的占位值，不在此新造坐标。
EXAMPLE_FLOWS = {
    "_comment": "工艺流程定义（W4）：用户可编辑；op 全集见 flow_engine.py "
                "docstring。交接示教位为占位待标定（见 transfers.json 各规则 "
                "_note），本文件不重复定义坐标。",
    "flows": [
        {"name": "晶圆键合示范流程",
         "description": "蓝膜上机→上片→取片→退避对准台/晶圆台→翻转交接→对准→晶圆台回位"
                        "→键合→回安全位→下片 全链路演示。交接坐标取自 "
                        "transfers.json 示教位（占位值待真机标定，W5 零件级沙盒后"
                        "已按接触面反推敲正）；对准台扫描幅值 ±20mm 沿用 ACS "
                        "buffer2 align_scan 占位值。退避步与真机一致：键合头下降前"
                        "先让开对准相机与晶圆台（沙盒干涉检查会拦不退避的下降）。"
                        "跑前请确认各轴已回零。",
         "steps": [
             {"op": "log", "msg": "=== 晶圆键合示范流程 开始 ===",
              "desc": "流程开始"},
             {"op": "film", "action": "load",
              "desc": "蓝膜上机：film 台蓝膜在位 + 按处方重铺 die（幂等，"
                      "已在位也允许重铺）"},
             {"op": "wafer", "action": "load",
              "desc": "上片：晶圆加载到晶圆台（已在位则跳过）"},
             {"op": "transfer", "rule": "film_to_flip",
              "desc": "取片：翻转吸盘从蓝膜台取一颗 die（ChuckVac 真空 + "
                      "FA_X/FA_Z 到取片示教位）"},
             {"op": "move", "axis": "AS_Y", "target": 120.0,
              "desc": "退避对准台相机（真机流程先退避 AS 相机再降键合头；"
                      "Y+120 退出键合头下方）"},
             {"op": "move", "axis": "WS_LS_X", "target": 220.0,
              "desc": "退避晶圆台（键合头下降通道让开；X+220 时吸盘台移出"
                      "压头投影区）"},
             {"op": "move", "axis": "FA_Z", "target": 20.0,
              "desc": "翻转 Z 抬传送安全位（=flip_pick.fa_z_safe_mm；FA_Z=-5"
                      " 扫掠会撞晶圆台 Rz 盘 Epin 缺口，2026-08-26 回归实测"
                      " FA_X 在 -773 被 soft-stop）"},
             {"op": "transfer", "rule": "flip_to_head",
              "desc": "翻转交接：键合头三吸盘接管 die（WT_BondingVac_1..3 "
                      "建立、ChuckVac 释放；FA_X=-1242.35 吸盘心=头心 / "
                      "BS_Arm_Z=-50.93 接触 die 背面，2026-08-26 起真实几何，"
                      "见 docs/工艺流程_芯片交接_V1.md）"},
             {"op": "move", "axis": "FA_X", "target": 0.0,
              "desc": "翻转撤离交接位（真实几何下交接位就在键合头正下方，"
                      "不退则后续键合臂下降撞空吸盘）"},
             {"op": "move", "axis": "AS_X", "target": 20.0,
              "desc": "对准台移到扫描位（幅值沿用 buffer2 align_scan ±20mm）"},
             {"op": "move", "axis": "BS_Arm_Z", "target": -118.83,
              "desc": "上相机对焦：键合臂从交接位 -50.93 降到 up 相机工作距"
                      "（BS_Arm_Z≈-118.83 时 up 镜头到焦面，见 "
                      "vision_cameras.json up._note；对准台/晶圆台已退避"
                      "不干涉）"},
             {"op": "camera", "source": "AlignCameraTigger_1",
              "desc": "对准拍照 1（精对准相机 1）"},
             {"op": "move", "axis": "AS_X", "target": 0.0,
              "desc": "对准台回零位"},
             {"op": "camera", "source": "AlignCameraTigger_2",
              "desc": "对准拍照 2（精对准相机 2）"},
             {"op": "move", "axis": "BS_Arm_Z", "target": 0.0,
              "desc": "键合臂抬起（晶圆台回位前必须先抬头，否则臂在低位挡"
                      "回位通道）"},
             {"op": "move", "axis": "WS_LS_X", "target": 0.0,
              "desc": "晶圆台回零位（键合头已抬起，无干涉）"},
             {"op": "transfer", "rule": "head_to_wafer",
              "desc": "键合：die 放到晶圆（晶圆真空建立、键合头三吸盘释放；"
                      "BS_Arm_Z=-162.5 键合接触位，压头贴面 <ε 不判撞）"},
             {"op": "move", "axis": "BS_Arm_Z", "target": 0.0,
              "desc": "回安全位：键合臂抬起"},
             {"op": "move", "axis": "AS_Y", "target": 0.0,
              "desc": "回安全位：对准台相机退回零位（键合头已抬起，无干涉）"},
             {"op": "move", "axis": "FA_X", "target": 0.0,
              "desc": "回安全位：翻转臂退出交接位"},
             {"op": "move", "axis": "FA_Z", "target": 0.0,
              "desc": "回安全位：翻转 Z 抬起"},
             {"op": "wafer", "action": "unload",
              "desc": "下片：成品晶圆离场（已键合 die 随片移除）"},
             {"op": "log", "msg": "=== 晶圆键合示范流程 完成 ===",
              "desc": "流程完成"},
         ]},
    ],
}


class FlowStateError(RuntimeError):
    """流程状态冲突（如运行中再 run / 未暂停就 step）——HTTP 层映射 409。"""


class _FlowAbort(Exception):
    """流程中止内部信号（stop 或步骤失败）。"""
    def __init__(self, reason):
        super(_FlowAbort, self).__init__(reason)
        self.reason = reason


class FlowEngine(object):
    def __init__(self, eng, points, flows_path):
        self.eng = eng
        self.points = points
        self.flows_path = flows_path
        self._lock = threading.RLock()
        self._ctl = threading.Condition(self._lock)   # pause/stop/step 控制信号
        self.state = "idle"
        self.flow_name = None
        self.steps = []                # 本次运行的步骤结果（与流程 steps 对齐）
        self.current_step = -1
        self.log = deque(maxlen=LOG_MAX)
        self._pause_req = False
        self._stop_req = False
        self._step_req = False
        self._t_start = None
        self._thread = None
        self.flows = self._load_or_seed()

    # ---------------- flows.json 加载/保存 ----------------
    def _load_or_seed(self):
        if os.path.exists(self.flows_path):
            with open(self.flows_path, encoding="utf-8") as f:
                data = json.load(f)
            return data.get("flows", [])
        self._write_file(EXAMPLE_FLOWS)
        return copy.deepcopy(EXAMPLE_FLOWS["flows"])

    def _write_file(self, data):
        """原子写 flows.json。"""
        tmp = self.flows_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.flows_path)

    def save_flows(self, flows):
        """PUT /api/flows：整体替换（调用方须先 validate）。"""
        with self._ctl:
            if self.state in ("running", "paused"):
                raise FlowStateError("流程运行/暂停中禁止修改流程定义（先 stop）")
            data = {"_comment": EXAMPLE_FLOWS["_comment"], "flows": flows}
            self._write_file(data)
            self.flows = copy.deepcopy(flows)
            return len(self.flows)

    def restore_example(self):
        """恢复内置示例流程（ flows.json 整体重置）。"""
        with self._ctl:
            if self.state in ("running", "paused"):
                raise FlowStateError("流程运行/暂停中禁止恢复示例（先 stop）")
            self._write_file(EXAMPLE_FLOWS)
            self.flows = copy.deepcopy(EXAMPLE_FLOWS["flows"])
            return len(self.flows)

    def flows_public(self):
        with self._lock:
            return {"ok": True, "flows": copy.deepcopy(self.flows),
                    "state": self.state, "current": self.flow_name}

    # ---------------- schema 校验（PUT 用，逐条报错） ----------------
    def validate(self, data):
        """-> errors 列表（空 = 合法）。错误信息带 flows[i]/steps[j] 定位。"""
        errors = []
        if not isinstance(data, dict) or not isinstance(data.get("flows"), list):
            return ["顶层须为对象且含 flows 数组：{\"flows\":[...]}"]
        names = set()
        for fi, f in enumerate(data["flows"]):
            loc = "flows[%d]" % fi
            if not isinstance(f, dict):
                errors.append(loc + "：流程须为对象")
                continue
            name = f.get("name")
            if not name or not isinstance(name, str):
                errors.append(loc + "：缺 name（非空字符串）")
            elif name in names:
                errors.append(loc + "：name 重复 %r" % name)
            else:
                names.add(name)
                loc = "流程%r" % name
            steps = f.get("steps")
            if not isinstance(steps, list) or not steps:
                errors.append(loc + "：steps 须为非空数组")
                continue
            self._validate_steps(steps, loc, errors, depth=1)
        return errors

    def _validate_steps(self, steps, loc, errors, depth):
        for si, st in enumerate(steps):
            sloc = "%s steps[%d]" % (loc, si)
            if not isinstance(st, dict):
                errors.append(sloc + "：步骤须为对象")
                continue
            op = st.get("op")
            if op not in FLOW_OPS:
                errors.append(sloc + "：op 非法 %r（合法：%s）"
                              % (op, "/".join(FLOW_OPS)))
                continue
            if op == "move":
                self._req(st, sloc, errors, axis=str, target=(int, float))
                if st.get("axis") is not None and isinstance(st.get("axis"), str) \
                        and not ac.resolve_axis(st["axis"]):
                    errors.append(sloc + "：未知轴 %r" % st["axis"])
                self._opt_num(st, sloc, errors, "speed", positive=True)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "home":
                if not isinstance(st.get("axis"), str) or not st.get("axis"):
                    errors.append(sloc + "：home 缺 axis（轴名或 \"ALL\"）")
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "io":
                self._req(st, sloc, errors, path=str, cmd=str)
                if "args" in st and not isinstance(st["args"], dict):
                    errors.append(sloc + "：args 须为对象")
            elif op == "wait_grip":
                self._req(st, sloc, errors, circuit=str, expect=bool)
                c = st.get("circuit")
                if isinstance(c, str) and c not in self.eng.air:
                    errors.append(sloc + "：未知气路 %r" % c)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
                if "err" in st and not isinstance(st["err"], str):
                    errors.append(sloc + "：err 须为字符串（自定义报警文本）")
            elif op == "wait_die":
                self._req(st, sloc, errors, carrier=str)
                if isinstance(st.get("carrier"), str) \
                        and st["carrier"] not in ("film", "flip", "head",
                                                  "wafer"):
                    errors.append(sloc + "：carrier 非法 %r" % st["carrier"])
                # vacated: 可选载体列表，须全部清空才算满足（放回蓝膜用——
                # film 恒非空，"flip 清空"才是放回完成的真信号）
                if "vacated" in st:
                    v = st["vacated"]
                    if not (isinstance(v, list) and v
                            and all(isinstance(x, str)
                                    and x in ("film", "flip", "head", "wafer")
                                    for x in v)):
                        errors.append(sloc + "：vacated 须为载体名列表")
                self._opt_num(st, sloc, errors, "timeout", positive=True)
                if "err" in st and not isinstance(st["err"], str):
                    errors.append(sloc + "：err 须为字符串（自定义报警文本）")
            elif op == "wait_io":
                self._req(st, sloc, errors, path=str)
                if "expect" not in st:
                    errors.append(sloc + "：wait_io 缺 expect")
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "transfer":
                self._req(st, sloc, errors, rule=str)
                known = self._known_rules()
                if isinstance(st.get("rule"), str) and known is not None \
                        and st["rule"] not in known:
                    errors.append(sloc + "：未知交接规则 %r（transfers.json 现有：%s）"
                                  % (st["rule"], "/".join(sorted(known))))
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "camera":
                if st.get("source") not in CAMERA_TRIGGERS:
                    errors.append(sloc + "：source 非法 %r（合法：%s）"
                                  % (st.get("source"), "/".join(CAMERA_TRIGGERS)))
            elif op == "coarse_offset":
                self._req(st, sloc, errors, cam=str)
                if "err" in st and not isinstance(st["err"], str):
                    errors.append(sloc + "：err 须为字符串（自定义报警文本）")
            elif op == "delay":
                self._req(st, sloc, errors, ms=(int, float))
                if isinstance(st.get("ms"), (int, float)) \
                        and not isinstance(st.get("ms"), bool) \
                        and st["ms"] <= 0:
                    errors.append(sloc + "：ms 须为正数")
            elif op == "if":
                if depth >= MAX_IF_DEPTH:
                    errors.append(sloc + "：if 最多嵌套一层")
                cond = st.get("cond")
                if not isinstance(cond, dict) or (
                        "io" not in cond and "grip" not in cond) \
                        or "expect" not in cond:
                    errors.append(sloc + "：cond 须为 {io:path,expect:val} 或 "
                                  "{grip:circuit,expect:bool}")
                then = st.get("then")
                if not isinstance(then, list) or not then:
                    errors.append(sloc + "：then 须为非空步骤数组")
                else:
                    self._validate_steps(then, sloc + ".then", errors, depth + 1)
                if "else" in st:
                    if not isinstance(st["else"], list):
                        errors.append(sloc + "：else 须为步骤数组")
                    else:
                        self._validate_steps(st["else"], sloc + ".else",
                                             errors, depth + 1)
            elif op == "log":
                self._req(st, sloc, errors, msg=str)
            elif op == "wafer":
                if st.get("action") not in ("load", "unload"):
                    errors.append(sloc + "：action 只能是 load/unload")
                if st.get("offset") is not None and not (
                        isinstance(st["offset"], (list, tuple)) and
                        len(st["offset"]) in (2, 3) and
                        all(isinstance(v, (int, float)) and not isinstance(v, bool)
                            for v in st["offset"])):
                    errors.append(sloc + "：offset 须为 [x,y] 或 [x,y,rz]")
                for k in ("z", "rx", "ry"):
                    if st.get(k) is not None and not (
                            isinstance(st[k], (int, float))
                            and not isinstance(st[k], bool)):
                        errors.append(sloc + "：%s 须为数值（上片垂向真值 "
                                      "mm/rad）" % k)
                if "rz" in st and not isinstance(st["rz"], (int, float)):
                    errors.append(sloc + "：rz 须为数值")
            elif op == "film":
                if st.get("action") not in ("load", "unload"):
                    errors.append(sloc + "：action 只能是 load/unload")
            elif op == "ega_measure":
                self._req(st, sloc, errors, site_x=(int, float), site_y=(int, float))
                self._opt_num(st, sloc, errors, "timeout", positive=True)
                if "cam" in st and st["cam"] not in ("down",):
                    errors.append(sloc + "：cam 当前只支持 down")
            elif op == "ega_fit":
                pass
            elif op == "fl_measure":
                if st.get("probe") not in ("up", "down"):
                    errors.append(sloc + "：probe 只能是 up/down")
                if st.get("space") not in ("wafer", "die", "stage",
                                           "wafer_v", "die_v"):
                    errors.append(sloc + "：space 只能是 wafer/die/stage/"
                                  "wafer_v/die_v")
                for k in ("point", "aim"):
                    if st.get(k) is not None and not (
                            isinstance(st[k], (list, tuple)) and len(st[k]) == 2
                            and all(isinstance(v, (int, float))
                                    and not isinstance(v, bool)
                                    for v in st[k])):
                        errors.append(sloc + "：%s 须为 [x,y] 数值对" % k)
                if st.get("point") is None:
                    errors.append(sloc + "：缺/非法字段 point")
                if "align_first" in st and not isinstance(
                        st["align_first"], bool):
                    errors.append(sloc + "：align_first 须为布尔")
                if "freeze_ws" in st and not isinstance(
                        st["freeze_ws"], bool):
                    errors.append(sloc + "：freeze_ws 须为布尔")
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "fl_ready":
                for k in ("site_x", "site_y"):
                    if st.get(k) is None or not isinstance(st[k], (int, float)):
                        errors.append(sloc + "：%s 须为数值" % k)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "fl_solve":
                if st.get("mode") not in ("wafer", "die", "compensate"):
                    errors.append(sloc + "：mode 只能是 wafer/die/compensate")
            elif op == "fl_verify":
                pass
            elif op == "fa_measure" or op == "fa_compensate_confirm":
                for k in ("site_x", "site_y"):
                    if st.get(k) is None or not isinstance(st[k], (int, float)):
                        errors.append(sloc + "：%s 须为数值" % k)
                for k in ("wafer_pts", "die_pts"):
                    v = st.get(k)
                    if not (isinstance(v, list) and len(v) >= 2
                            and all(isinstance(p, (list, tuple)) and len(p) == 2
                                    and all(isinstance(x, (int, float))
                                            and not isinstance(x, bool)
                                            for x in p) for p in v)):
                        errors.append(sloc + "：%s 须为 ≥2 个 [x,y] 数值对" % k)
                if (isinstance(st.get("wafer_pts"), list)
                        and isinstance(st.get("die_pts"), list)
                        and len(st["wafer_pts"]) != len(st["die_pts"])):
                    errors.append(sloc + "：wafer_pts/die_pts 点数须一致")
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "fa_solve":
                pass
            elif op == "heater":
                self._req(st, sloc, errors, name=str, action=str)
                if isinstance(st.get("name"), str) \
                        and st["name"] not in self.eng.heaters:
                    errors.append(sloc + "：未知加热器 %r" % st["name"])
                if st.get("action") not in ("set", "wait", "off"):
                    errors.append(sloc + "：action 只能是 set/wait/off")
                if st.get("action") == "set":
                    self._req(st, sloc, errors, temp=(int, float))
                self._opt_num(st, sloc, errors, "tol_c", positive=True)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "force":
                self._req(st, sloc, errors, target=(int, float))
                self._opt_num(st, sloc, errors, "tol_n", positive=True)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
            elif op == "force_search":
                self._req(st, sloc, errors, axis=str,
                          touch_force_N=(int, float),
                          min_pos=(int, float))
                if st.get("axis") is not None and isinstance(st.get("axis"), str) \
                        and not ac.resolve_axis(st["axis"]):
                    errors.append(sloc + "：未知轴 %r" % st["axis"])
                self._opt_num(st, sloc, errors, "speed", positive=True)
                self._opt_num(st, sloc, errors, "timeout", positive=True)
                # 接触确认（迭代 13，可缺省=不核对）：期望有效接触位 + 容差
                self._opt_num(st, sloc, errors, "eff_contact")
                self._opt_num(st, sloc, errors, "contact_verify_tol_um",
                              positive=True)
            elif op == "bond_mark":
                self._req(st, sloc, errors, site_x=(int, float),
                          site_y=(int, float))
            elif op == "pi_measure":
                self._req(st, sloc, errors, site_x=(int, float),
                          site_y=(int, float), die_id=str)
                v = st.get("die_pts")
                if not (isinstance(v, list) and len(v) >= 2
                        and all(isinstance(p, (list, tuple)) and len(p) == 2
                                and all(isinstance(x, (int, float))
                                        and not isinstance(x, bool)
                                        for x in p) for p in v)):
                    errors.append(sloc + "：die_pts 须为 ≥2 个 [x,y] 数值对")
                self._opt_num(st, sloc, errors, "timeout", positive=True)

    @staticmethod
    def _req(st, sloc, errors, **fields):
        for k, t in fields.items():
            v = st.get(k)
            if not isinstance(v, t) or isinstance(v, bool) and t is not bool:
                errors.append(sloc + "：缺/非法字段 %s" % k)

    @staticmethod
    def _opt_num(st, sloc, errors, key, positive=False):
        if key not in st:
            return
        v = st[key]
        if not isinstance(v, (int, float)) or isinstance(v, bool) \
                or (positive and v <= 0):
            errors.append(sloc + "：%s 须为%s数" % (key, "正" if positive else ""))

    def _known_rules(self):
        wp = self.eng.workpieces
        if wp is None:
            return None
        with wp._lock:
            return set(r["name"] for r in wp.rules)

    # ---------------- 控制接口（API 层调用） ----------------
    def run(self, name, paused=False):
        """启动流程（paused=true 起步即暂停在第一步前，配合 step 单步）。"""
        flow = next((f for f in self.flows if f.get("name") == name), None)
        if flow is None:
            raise KeyError("未知流程：%r" % name)
        return self._start(flow, name, paused)

    def run_steps(self, steps, name, paused=False):
        """运行**动态步骤列表**（process_flows.py 参数化生成的工艺流程，如
        翻转取片）——与 flows.json 具名流程共用同一状态机/pause/stop/step/
        日志/运行冲突 409 语义，面板工艺流程 tab 直接可见。启动前过一遍与
        PUT 相同的步骤 schema 校验（坏步骤拒绝启动）。"""
        errors = []
        self._validate_steps(steps, "流程%r" % name, errors, depth=1)
        if errors:
            raise ValueError("动态流程 %r 步骤校验失败（%d 条）：%s"
                             % (name, len(errors), "；".join(errors)))
        r = self._start({"name": name, "steps": steps}, name, paused)
        # 生成结果随响应返回（上位机打样：参数化目标值可直接核对/展示）
        r["steps_def"] = copy.deepcopy(steps)
        return r

    def _start(self, flow, name, paused):
        with self._ctl:
            if self.state in ("running", "paused"):
                raise FlowStateError("已有流程在运行：%s（先 stop）" % self.flow_name)
            self.flow_name = name
            self.current_step = -1
            self.steps = [{"i": i, "op": s["op"], "desc": s.get("desc", ""),
                           "status": "pending", "dur_ms": None, "info": ""}
                          for i, s in enumerate(flow["steps"])]
            self._pause_req = bool(paused)
            self._stop_req = False
            self._step_req = False
            self._t_start = time.time()
            self.state = "paused" if paused else "running"
            # 测量类状态按流程内容重置：EGA/调焦调平结果是机器状态，只被
            # 含对应 op 的流程清空重测——其它流程（取片/交接等）启动不得
            # 冲掉 eng.ega_result（focus_level 生成时要消费它）
            ops_in_flow = set(s.get("op") for s in flow["steps"])
            if "ega_measure" in ops_in_flow or "ega_fit" in ops_in_flow:
                self.eng.ega_measurements = []
                self.eng.ega_result = None
            if "fl_measure" in ops_in_flow or "fl_solve" in ops_in_flow:
                self.eng.fl_measurements = {"wafer": [], "die": [],
                                            "stage": [], "wafer_v": [],
                                            "die_v": []}
                self.eng.fl_fit = {}
                self.eng.fl_ws_frozen = None
            if "fa_measure" in ops_in_flow or "fa_solve" in ops_in_flow \
                    or "fa_compensate_confirm" in ops_in_flow:
                self.eng.fa_meas = None
                self.eng.fa_delta = None
                self.eng.fine_align_result = None
                # 精对准重测即旧 site 的接触时刻真值失效（迭代 20）
                self.eng.bond_commit = None
            self._log("info", "流程启动：%s%s"
                      % (name, "（单步模式，暂停在起点）" if paused else ""))
            self._thread = threading.Thread(
                target=self._run, args=(flow,), name="flow", daemon=True)
            self._thread.start()
            return {"flow": name, "steps": len(self.steps),
                    "paused": bool(paused)}

    def pause(self):
        with self._ctl:
            if self.state != "running":
                raise FlowStateError("当前状态 %s，不能暂停" % self.state)
            self._pause_req = True          # 在下一步边界生效
            return {"state": "pausing",
                    "note": "在步骤边界生效，进行中的步骤会跑完"}

    def resume(self):
        with self._ctl:
            if self.state != "paused":
                raise FlowStateError("当前状态 %s，不能继续" % self.state)
            self._pause_req = False
            self._ctl.notify_all()
            return {"state": "running"}

    def step(self):
        """单步：paused 态推进恰好一步后再次暂停。"""
        with self._ctl:
            if self.state != "paused":
                raise FlowStateError("单步需在 paused 态（run 可带 paused=true）")
            self._step_req = True
            self._ctl.notify_all()
            return {"state": "stepping", "next_step": self.current_step + 1}

    def stop(self):
        with self._ctl:
            if self.state not in ("running", "paused"):
                raise FlowStateError("当前无运行中的流程（state=%s）" % self.state)
            self._stop_req = True
            self._pause_req = False
            self._ctl.notify_all()
        # 运动轴全停（引擎锁外调用，避免与流程线程的等待循环互相持锁）
        self.eng.stop()
        return {"state": "stopping"}

    def reset(self):
        with self._ctl:
            if self.state in ("running", "paused"):
                raise FlowStateError("先 stop 再 reset（state=%s）" % self.state)
            self.state = "idle"
            self.flow_name = None
            self.steps = []
            self.current_step = -1
            self._t_start = None
            self._log("info", "流程状态复位")
            return {"state": "idle"}

    def state_public(self):
        with self._ctl:
            return {"ok": True, "state": self.state, "flow": self.flow_name,
                    "current_step": self.current_step,
                    "steps": [dict(s) for s in self.steps],
                    "log": list(self.log)[-100:],
                    # 最近一次粗对准偏差（交接补偿输入；None=未测过）
                    "coarse_offset": (dict(self.eng.coarse_offset)
                                      if self.eng.coarse_offset else None),
                    "ega_result": (dict(self.eng.ega_result)
                                   if getattr(self.eng, "ega_result", None)
                                   else None),
                    # 调焦调平：最近一次 compensate 结果与 global 调平记忆
                    "fl_result": (dict(self.eng.fl_fit["compensate"])
                                  if getattr(self.eng, "fl_fit", None)
                                  and self.eng.fl_fit.get("compensate")
                                  else None),
                    # 调平复测验证结果（fl_verify；迭代 5）
                    "fl_verify": (dict(self.eng.fl_fit["verify"])
                                  if getattr(self.eng, "fl_fit", None)
                                  and self.eng.fl_fit.get("verify")
                                  else None),
                    "level_memory": (dict(self.eng.level_memory)
                                     if getattr(self.eng, "level_memory", None)
                                     else None),
                    # 精对准：最近一次 fa_compensate_confirm 结果（迭代 6）
                    "fine_align_result": (
                        dict(self.eng.fine_align_result)
                        if getattr(self.eng, "fine_align_result", None)
                        else None),
                    # 三回路 PID pv/sp/out（迭代 8，面板温控压控页轮询）
                    "pid": self.eng.pid_public(),
                    "elapsed_s": (round(time.time() - self._t_start, 2)
                                  if self._t_start else 0.0)}

    # ---------------- 执行线程 ----------------
    def _log(self, level, msg):
        with self.eng.lock:
            t = self.eng.sim_time
        self.log.append({"t": round(t, 3), "level": level, "msg": msg})

    def _run(self, flow):
        try:
            for i, st in enumerate(flow["steps"]):
                self._await_gate(i)        # 步骤边界：pause/stop 在此生效
                self._exec_step(i, st)
            self._finish("done", "流程完成")
        except _FlowAbort as a:
            self._finish("aborted", a.reason)
        except Exception as e:             # 兜底：引擎 bug 不应炸线程
            self._finish("aborted", "内部异常：%s" % e)

    def _await_gate(self, i):
        with self._ctl:
            self.current_step = i
            while self._pause_req and not self._step_req and not self._stop_req:
                self.state = "paused"
                self._ctl.wait(0.2)
            if self._stop_req:
                raise _FlowAbort("手动停止（步骤 #%d 边界）" % (i + 1))
            self._step_req = False
            self.state = "running"
            self.steps[i]["status"] = "running"

    def _exec_step(self, i, st):
        t0 = time.time()
        ok, info = True, ""
        try:
            info = self._do_step(st, depth=1)
        except _FlowAbort:
            raise                          # stop 信号：直接向上，由 _finish 记账
        except Exception as e:
            ok, info = False, str(e)
        dur_ms = (time.time() - t0) * 1000
        with self._ctl:
            rec = self.steps[i]
            # stop 中途打断时 rec 可能已被 _finish 标记，这里以最新执行为准
            rec["status"] = "ok" if ok else "fail"
            rec["dur_ms"] = round(dur_ms, 1)
            rec["info"] = info
        if not ok:
            self._log("error", "步骤 #%d（%s）失败：%s" % (i + 1, st["op"], info))
            # 失败原因带上当前 fault 轴（呼应 W2：掉使能/故障会打断工艺流）
            fault_axes = self.eng.fault_state()["fault_axes"]
            suffix = ("；fault 轴：%s" % "/".join(fault_axes)) if fault_axes else ""
            raise _FlowAbort("步骤 #%d（%s）失败：%s%s" % (i + 1, st["op"],
                                                         info, suffix))

    def _finish(self, state, reason):
        with self._ctl:
            for s in self.steps:
                if s["status"] == "running":
                    s["status"] = "fail" if state == "aborted" else "ok"
                    s["info"] = s["info"] or reason
                elif s["status"] == "pending":
                    s["status"] = "skip"
            self.state = state
            elapsed = (round(time.time() - self._t_start, 2)
                       if self._t_start else 0.0)
        self._log("info" if state == "done" else "error",
                  "%s：%s（总耗时 %.2fs）"
                  % ("流程完成" if state == "done" else "流程中止",
                     reason, elapsed))

    # ---------------- 等待助手 ----------------
    def _wait(self, cond_fn, timeout, desc):
        """轮询 cond_fn() 为真；stop 请求立即抛 _FlowAbort；
        cond_fn 内部抛出的异常（如轴 fault）原样向上传播判步骤失败。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._ctl:
                if self._stop_req:
                    raise _FlowAbort("手动停止（等待%s中断）" % desc)
            if cond_fn():
                return True
            time.sleep(0.01)
        raise RuntimeError("等待%s超时（%.1fs）" % (desc, timeout))

    # ---------------- 步骤执行 ----------------
    def _do_step(self, st, depth):
        """执行单步 -> info 字符串；失败抛异常。if 分支子步骤递归（depth 计层）。"""
        op = st["op"]
        if op == "move":
            return self._do_move(st["axis"], st["target"], st.get("speed"),
                                 float(st.get("timeout", DEFAULT_MOVE_TIMEOUT)))
        if op == "home":
            return self._do_home(st["axis"],
                                 float(st.get("timeout", DEFAULT_HOME_TIMEOUT)))
        if op == "io":
            return self._do_io(st["path"], st["cmd"], st.get("args") or {})
        if op == "wait_grip":
            c, expect = st["circuit"], bool(st["expect"])
            try:
                self._wait(lambda: self.eng.air_grip_ok(c) == expect,
                           float(st.get("timeout", DEFAULT_WAIT_TIMEOUT)),
                           " %s grip_ok=%s" % (c, expect))
            except RuntimeError:
                if st.get("err"):            # 自定义报警文本（拾取判定等）
                    raise RuntimeError(st["err"])
                raise
            return "%s grip_ok=%s 已满足" % (c, expect)
        if op == "wait_die":
            carrier = st["carrier"]
            vacated = st.get("vacated") or []

            def has_carrier():
                wp = self.eng.workpieces
                if wp is None:
                    return False
                with wp._lock:
                    if not any(d["carrier"] == carrier for d in wp.dies):
                        return False
                    # vacated：要求这些载体全部清空（放回蓝膜——film 恒非
                    # 空，"flip 已腾空"才是放回完成的真信号）
                    return all(
                        not any(d["carrier"] == c for d in wp.dies)
                        for c in vacated)
            try:
                self._wait(has_carrier,
                           float(st.get("timeout", DEFAULT_WAIT_TIMEOUT)),
                           " die carrier=%s" % carrier)
            except RuntimeError:
                if st.get("err"):
                    raise RuntimeError(st["err"])
                raise
            return "die 已就位 carrier=%s%s" % (
                carrier, "（%s 已腾空）" % "/".join(vacated) if vacated
                else "")
        if op == "wait_io":
            path, expect = st["path"], st["expect"]
            self._wait(lambda: self._io_eq(path, expect),
                       float(st.get("timeout", DEFAULT_WAIT_TIMEOUT)),
                       " %s == %r" % (path, expect))
            return "%s == %r 已满足" % (path, expect)
        if op == "transfer":
            return self._do_transfer(st["rule"],
                                     float(st.get("timeout",
                                                  DEFAULT_MOVE_TIMEOUT)))
        if op == "camera":
            cap = self.eng.camera_trigger(st["source"])
            return "拍照 #%d（%s）" % (cap["id"], st["source"])
        if op == "coarse_offset":
            return self._do_coarse_offset(st)
        if op == "delay":
            return self._do_delay(float(st["ms"]))
        if op == "if":
            return self._do_if(st, depth)
        if op == "log":
            self._log("log", st["msg"])
            return st["msg"]
        if op == "wafer":
            return self._do_wafer(st["action"], st.get("offset"), st.get("rz"),
                                  z=st.get("z"), rx=st.get("rx"),
                                  ry=st.get("ry"),
                                  inject=bool(st.get("inject_recipe")))
        if op == "film":
            return self._do_film(st["action"])
        if op == "ega_measure":
            return self._do_ega_measure(st)
        if op == "ega_fit":
            return self._do_ega_fit()
        if op == "fl_ready":
            return self._do_fl_ready(st)
        if op == "fl_measure":
            return self._do_fl_measure(st)
        if op == "fl_solve":
            return self._do_fl_solve(st)
        if op == "fl_verify":
            return self._do_fl_verify()
        if op == "fa_measure":
            return self._do_fa_measure(st)
        if op == "fa_solve":
            return self._do_fa_solve()
        if op == "fa_compensate_confirm":
            return self._do_fa_compensate_confirm(st)
        if op == "heater":
            return self._do_heater(st)
        if op == "force":
            return self._do_force(st)
        if op == "force_search":
            return self._do_force_search(st)
        if op == "bond_mark":
            return self._do_bond_mark(st)
        if op == "bond_truth":
            return self._do_bond_truth(st)
        if op == "bond_offset_apply":
            return self._do_bond_offset_apply(st)
        if op == "pi_measure":
            return self._do_pi_measure(st)
        raise RuntimeError("op 未实现：%r" % op)

    # ---------------- 热压键合（迭代 8） ----------------
    def _do_heater(self, st):
        """heater op：包 heater_command + 等温（到 ±tol_c 带，真值判定）。
        action=set 设定目标温度并等到温；wait 只对当前目标等到温（硅片台
        常开保温场景的幂等确认）；off=StopTargetTempWithPlan 停止计划。"""
        name = st["name"]
        action = st["action"]
        h = self.eng.heaters[name]
        if action == "off":
            self.eng.heater_command(name, "StopTargetTempWithPlan")
            return "%s 加热计划已停止（保温回路关闭，自然降温）" % name
        if action == "set":
            self.eng.heater_command(name, "SetTargetTempWithPlan",
                                    {"Temperature": float(st["temp"])})
        tol = float(st.get("tol_c", 2.0))
        timeout = float(st.get("timeout", 600.0))
        with self.eng.lock:
            tgt = h.target

        def in_band():
            with self.eng.lock:
                return h.active and abs(h.cur - tgt) <= tol
        self._wait(in_band, timeout,
                   " %s 到温 %.1f±%.1f℃" % (name, tgt, tol))
        with self.eng.lock:
            cur = h.cur
        return "%s 已到温：%.1f℃（目标 %.1f±%.1f℃）" % (name, cur, tgt, tol)

    def _do_force(self, st):
        """force op：设键合力回路 setpoint（BondingByForceManual 命令面）
        并等真值力到 ±tol_n 带。target=0=释放（PID 抬头卸力）。"""
        target = float(st["target"])
        tol = float(st.get("tol_n", 1.0))
        timeout = float(st.get("timeout", 60.0))
        self.eng.bonding_force(target, 0.0)

        def in_band():
            with self.eng.lock:
                return abs(self.eng.force_value - target) <= tol
        self._wait(in_band, timeout,
                   " 键合力 %.1f±%.1fN" % (target, tol))
        with self.eng.lock:
            fv = self.eng.force_value
        return "键合力已到带：%.2fN（目标 %.1f±%.1fN）" % (fv, target, tol)

    # oActualForce 读数点位（含 ±force_noise_N 噪声，与面板显示同源）
    FORCE_READ_PATH = "SystemMachine.SS_Bonding.M_BS.oActualForce"

    def _do_force_search(self, st):
        """force_search op（2026-08-31 迭代 9，键合慢速寻力段）：以
        speed 慢速下行（目标=min_pos 超程下限），期间每 10ms 读力读数
        oActualForce（含噪声，与面板同源），≥ touch_force_N 即原地停轴
        并把实际接触位记到 engine.bond_touch_pos（供后续 force 闭环步骤
        作起始与流程日志消费）；轴到 min_pos 仍未触力 → FAIL"寻力未接触"。"""
        axis = st["axis"]
        names = ac.resolve_axis(axis)
        if not names:
            raise KeyError("unknown axis: %r" % axis)
        touch = float(st["touch_force_N"])
        min_pos = float(st["min_pos"])
        speed = st.get("speed")
        timeout = float(st.get("timeout", 120.0))
        with self.eng.lock:
            self.eng.bond_touch_pos = None
            self.eng.bond_contact_evidence = None
        self.eng.move(axis, min_pos, speed)   # 联锁/掉使能在此抛 409 族异常
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._ctl:
                if self._stop_req:
                    raise _FlowAbort("手动停止（寻力中断）")
            with self.eng.lock:
                for n in names:
                    if self.eng.axes[n].fault:
                        raise RuntimeError("轴 %s fault（寻力中掉使能/故障）" % n)
                rd = float(self.points.get(self.FORCE_READ_PATH, 0.0) or 0.0)
                settled_low = all(not self.eng.axes[n].moving
                                  and self.eng.axes[n].pos <= min_pos + 1e-6
                                  for n in names)
            if rd >= touch:
                break
            if settled_low:
                raise RuntimeError(
                    "寻力未接触：%s 已到超程下限 %.4f 仍无触力（读数 %.2fN < "
                    "阈值 %.2fN）——检查晶圆/芯片厚度处方或接触位标定"
                    % ("/".join(names), min_pos, rd, touch))
            time.sleep(0.01)
        else:
            raise RuntimeError("寻力超时（%.1fs）" % timeout)
        # 触力：原地停轴（重规划到当前位，走正常减速），记录实际接触位
        with self.eng.lock:
            stop_pos = dict((n, self.eng.axes[n].pos) for n in names)
        # 几何真值快照（2026-09-06 迭代 17 WP-B，contact_verify 双源判定的
        # 几何侧）：触力停轴瞬间的轴位+6DOF 位姿。必须在停轴这一刻取——
        # 停稳等待期间力回路（sp=0）会抬头卸力改动 BS_Arm_Z，之后再用活
        # 轴位算面高就不是"停轴瞬间"的几何了。无晶圆/无 die/无 confocal
        # （裸引擎单测）时快照=None，几何分支整体跳过只留轴链判定。
        geom_snap = None
        try:
            import vision_model as vm
            if (self.eng.confocal is not None
                    and self.eng.workpieces is not None
                    and self.eng.recipe is not None
                    and self.eng.kinematics is not None
                    and self.eng.workpieces.wafer.get("present")):
                geom_snap = vm._engine_poses(self.eng)
        except Exception:
            geom_snap = None
        for n, p in stop_pos.items():
            self.eng.move(n, p, speed)
        self._wait(lambda: all(self.eng.axes[n].in_position
                               and not self.eng.axes[n].moving for n in names),
                   10.0, " %s 寻力停稳" % "/".join(names))
        with self.eng.lock:
            pos = self.eng.axes[names[0]].pos
            self.eng.bond_touch_pos = pos
        # ---- 接触确认（2026-09-04 迭代 13；2026-09-06 迭代 17 改双源判定）
        # 源一（轴链）：实际接触位 vs 期望有效接触位（裸接触位+晶圆厚+芯片
        # 厚，流程生成侧换算传入 eff_contact）——抓常数/处方漂移。源二（几
        # 何真值）：confocal die_surface_z/wafer_surface_z 在键合 site 处
        # 独立核算 geom_gap_um（die 底面−晶圆面，含 wafer z/rx/ry 与 die
        # rx/ry 坡度项）——轴链两端同源测不出的面高变化/真值错位由它兜底。
        # 触力停轴时 press≈touch/k，正常 geom_gap≈−几µm（弹簧压入=接触
        # 上）；|geom_gap| 超容差同样 FAIL。证据写 eng.bond_contact_evidence
        # 供 bond_results 采集留证。
        eff = st.get("eff_contact")
        tol_um = float(st.get("contact_verify_tol_um", 50.0))
        if eff is not None:
            eff = float(eff)
            dev_um = (pos - eff) * 1000.0
            geom_gap_um = None
            if geom_snap is not None:
                try:
                    cf = self.eng.confocal
                    positions, poses = geom_snap
                    rec = self.eng.recipe.get()
                    dc = cf.die_center_world(positions, poses,
                                             self.eng.workpieces)
                    if dc is not None and rec is not None:
                        zd = cf.die_surface_z(dc[0], dc[1], positions, poses,
                                              rec, self.eng.workpieces)
                        zw = cf.wafer_surface_z(dc[0], dc[1], positions,
                                                poses, rec,
                                                self.eng.workpieces)
                        if zd is not None and zw is not None:
                            geom_gap_um = round((zd - zw) * 1000.0, 3)
                except Exception:
                    geom_gap_um = None
            axis_ok = abs(dev_um) <= tol_um
            geom_ok = geom_gap_um is None or abs(geom_gap_um) <= tol_um
            ev = {"actual_contact_z_mm": round(pos, 6),
                  "eff_contact_z_mm": round(eff, 6),
                  "dev_um": round(dev_um, 3),
                  "geom_gap_um": geom_gap_um,
                  "touch_force_N": round(rd, 3),
                  "tol_um": tol_um,
                  "judge": "pass" if (axis_ok and geom_ok) else "fail"}
            with self.eng.lock:
                self.eng.bond_contact_evidence = ev
            geom_txt = ("，几何间隙 %+.1fµm" % geom_gap_um
                        if geom_gap_um is not None else "")
            self._log("info" if ev["judge"] == "pass" else "error",
                      "接触确认：实际接触位 %.4f / 期望有效接触位 %.4f，"
                      "偏差 %+.1fµm（容差 ±%.0fµm）%s，触力 %.2fN → %s"
                      % (pos, eff, dev_um, tol_um, geom_txt, rd,
                         "PASS" if ev["judge"] == "pass" else "FAIL"))
            if not axis_ok:
                raise RuntimeError(
                    "接触位异常（轴链：厚度/机械常数不符）：实际接触位 %.4f "
                    "vs 期望有效接触位 %.4f，偏差 %+.1fµm 超容差 ±%.0fµm"
                    % (pos, eff, dev_um, tol_um))
            if not geom_ok:
                raise RuntimeError(
                    "接触面间隙异常（几何真值）：键合位处 die 底面 - 晶圆面 "
                    "间隙 %+.1fµm 超容差 ±%.0fµm（正=悬空未接触，负=压入）"
                    % (geom_gap_um, tol_um))
            return ("寻力接触：%s 实际接触位 %.4f（读数 %.2fN ≥ 阈值 %.2fN，"
                    "超程下限 %.4f 未触）；接触确认 PASS：偏差 %+.1fµm "
                    "≤ ±%.0fµm%s" % ("/".join(names), pos, rd, touch,
                                     min_pos, dev_um, tol_um, geom_txt))
        return ("寻力接触：%s 实际接触位 %.4f（读数 %.2fN ≥ 阈值 %.2fN，"
                "超程下限 %.4f 未触）" % ("/".join(names), pos, rd, touch,
                                          min_pos))

    def _do_bond_truth(self, st):
        """bond_truth op（2026-09-07 迭代 20，build_bond_flow 插在
        force_search 触力停轴之后、加压/保压/落片之前）：用当前 6DOF
        真值位姿走精对准同一条 fit 管线（_fa_truth_resid：fit_rigid 双侧
        + relative_delta）算 die↔site 真值残差，写 eng.bond_commit。

        物理语义：truth 的"键合头下降链"分量——fa 完成时 BS_Arm_Z 还在
        测量高度（≈-119.83），触力停轴时已到接触位（≈-167.4），BS_Arm_Z
        误差地图在两位姿间的 dx/dy 差（系统项）+ rx/ry 地图差 × 头杠杆
        （阿贝项）真实发生但此前无人采样；本步在接触时刻重采样真值，
        下降链自然进入。fa 时刻的 phys_resid 保留为 align_resid 分量
        （对准残差=相机噪声传播+末次补偿轴重复性）。力闭环/保压期间臂
        位置基本不动（压入量 µm 级，地图梯度可忽略），接触时刻采样即
        落位真值；压合微滑移（工艺项）由 bond_mark 叠加
        （bond.bond_shift_*，默认 0）。"""
        recipe = self.eng.recipe.get()
        fa = getattr(self.eng, "fine_align_result", None) or {}
        meas = getattr(self.eng, "fa_meas", None)
        site = fa.get("site")
        if not site or not meas:
            raise RuntimeError("bond_truth 需要本 site 精对准测量/结果"
                               "（先跑 fine_align 且 judge=pass）")
        st_true = {"site_x": site[0], "site_y": site[1],
                   "wafer_pts": meas["wafer_design"],
                   "die_pts": meas["die_design"]}
        commit = self._fa_truth_resid(st_true, recipe)
        if commit is None:
            raise RuntimeError("bond_truth：键合头上无 die（真值残差不可"
                               "得——先交接再键合）")
        phys = fa.get("phys_resid") or None
        align = ({"tx": float(phys["tx_um"]) / 1000.0,
                  "ty": float(phys["ty_um"]) / 1000.0,
                  "theta": float(phys["theta_urad"]) / 1e6}
                 if phys else None)
        with self.eng.lock:
            t_sim = self.eng.sim_time
        self.eng.bond_commit = {
            "site": [float(site[0]), float(site[1])],
            "tx": commit["tx"], "ty": commit["ty"], "theta": commit["theta"],
            "t": round(t_sim, 3),
            "align_resid": align,
            "process_shift": None}
        if align:
            dtxt = ("下降链 Δ=(%+.3f, %+.3f) µm / %+.2f µrad"
                    % ((commit["tx"] - align["tx"]) * 1000.0,
                       (commit["ty"] - align["ty"]) * 1000.0,
                       (commit["theta"] - align["theta"]) * 1e6))
        else:
            dtxt = "无 fa 时刻对准残差快照（align_resid=null）"
        info = ("键合接触时刻真值采样：die↔site 残差 commit=(%+.3f, %+.3f) "
                "µm / %+.2f µrad（BS_Arm_Z=%.4f 接触位；%s）"
                % (commit["tx"] * 1000.0, commit["ty"] * 1000.0,
                   commit["theta"] * 1e6,
                   self.eng.axes["BS_Arm_Z"].pos, dtxt))
        self._log("info", info)
        return info

    def _do_bond_mark(self, st):
        """bond_mark op：die 落位修正——transfers.json head_to_wafer 的
        site 是 [0,0] 占位，本步把最近一颗 head→wafer 交接的 die 的
        site 改写为当前键合位设计坐标（流程侧落笔，已键合标记/3D
        wafer_dies 层消费）。同时把本 site 键合真值记到 die["resid"]
        ——die 真实落位 = site + resid，视觉 wafer_die 标记真值消费
        （迭代 11b：缺了它 pi_measure 的 measured 永远测不到 truth 里
        的残差）。

        真值口径（2026-09-07 迭代 20 起）：resid = bond_commit（接触时刻
        真值，含下降链）+ process_shift（压合微滑移随机项，机械常数
        bond.bond_shift_xy_um/bond_shift_rz_urad 的 3σ 高斯采样，默认
        0=工艺项不建模）；旧口径（无 bond_commit 的老流程）回退
        fine_align_result.phys_resid（fa 时刻快照，不含下降链）。"""
        wp = self.eng.workpieces
        if wp is None:
            raise RuntimeError("workpieces not attached")
        commit = getattr(self.eng, "bond_commit", None)
        sx, sy = float(st["site_x"]), float(st["site_y"])
        commit_ok = bool(commit) and \
            [round(v, 6) for v in commit.get("site") or []] == \
            [round(sx, 6), round(sy, 6)]
        # ---- 找刚键合的 die ----
        with wp._lock:
            die_id = None
            for ent in reversed(wp.log):
                if ent.get("rule") == "head_to_wafer":
                    die_id = ent.get("die")
                    break
            die = next((d for d in wp.dies
                        if d["id"] == die_id and d["carrier"] == "wafer"),
                       None)
            if die is None:
                raise RuntimeError("bond_mark：找不到刚键合的 die（最近无 "
                                   "head_to_wafer 交接记录）")
        # ---- 压合微滑移（工艺项，默认 0）----
        bmc = self.eng.constants.get().get("bond", {})
        sh_xy = float(bmc.get("bond_shift_xy_um", 0.0))
        sh_rz = float(bmc.get("bond_shift_rz_urad", 0.0))
        shift = [0.0, 0.0, 0.0]
        if commit_ok and (sh_xy or sh_rz):
            import random as _rnd
            from error_system import stable_seed
            rng = _rnd.Random(stable_seed("bond_shift", die_id))
            shift = [rng.gauss(0.0, sh_xy / 3.0) / 1000.0,
                     rng.gauss(0.0, sh_xy / 3.0) / 1000.0,
                     rng.gauss(0.0, sh_rz / 3.0) / 1e6]
            commit["process_shift"] = {"tx": shift[0], "ty": shift[1],
                                       "theta": shift[2]}
        # ---- resid 落笔 ----
        if commit_ok:
            resid = [commit["tx"] + shift[0], commit["ty"] + shift[1],
                     commit["theta"] + shift[2]]
            src = "bond_commit（接触时刻真值，含下降链%s）" % (
                "+压合微滑移" if (sh_xy or sh_rz) else "")
        else:
            fa = getattr(self.eng, "fine_align_result", None) or {}
            phys = fa.get("phys_resid") or {}
            resid = [float(phys.get("tx_um", 0.0)) / 1000.0,
                     float(phys.get("ty_um", 0.0)) / 1000.0,
                     float(phys.get("theta_urad", 0.0)) / 1e6]
            src = "fine_align phys_resid（fa 时刻快照；无 bond_commit "
            "的旧流程回退口径，不含下降链）"
        with wp._lock:
            die["site"] = [sx, sy]
            die["resid"] = resid
        return "die %s 落位修正：site=(%.4f, %.4f) resid=(%+.3f, %+.3f µm, " \
               "%+.3f µrad)（真值来源：%s）" % (
                   die_id, sx, sy, resid[0] * 1000.0, resid[1] * 1000.0,
                   resid[2] * 1e6, src)

    def _do_bond_offset_apply(self, st):
        """bond_offset_apply op（2026-09-05 迭代 15，build_bond_flow 生成
        在精对准之后、下压之前）：把 WS 按机械常数 fine_align.
        bond_offset_* 施加键合附加补偿——设计系平移 +offset（经 EGA map
        线性部换到世界系动 WS_XY）、设计系旋转 +orz（WS_Rz 转动 + EGA
        map 同步，同 _fa_apply_comp ④）。

        施加点考据：在 fl_ready（键合位就位）施加会被其后的精对准测量/
        补偿环完全吸收——fa_measure 每次重测 site 标记并重算 WS 补偿，
        fl_ready 阶段的预偏移在 fa 补偿后净效应为零；只有精对准完成之
        后、键合下压之前施加的 WS 偏移才会留在 die 落点里。符号：die 落
        点相对 site 标记 = truth − offset，故补偿量建议 = measured mean
        （系统偏差），应用后 measured mean→0（补偿系统偏差，不补偿随机
        散布）。默认 0=不补偿（日志记跳过）。"""
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        fa_mc = self.eng.constants.get().get("fine_align", {})
        bo_x = float(fa_mc.get("bond_offset_x_um", 0.0)) / 1000.0
        bo_y = float(fa_mc.get("bond_offset_y_um", 0.0)) / 1000.0
        bo_rz = float(fa_mc.get("bond_offset_rz_urad", 0.0)) / 1e6
        if not (bo_x or bo_y or bo_rz):
            info = "键合附加补偿为 0（fine_align.bond_offset_*），跳过"
            self._log("info", info)
            return info
        fit = getattr(self.eng, "ega_result", None)
        if fit is None:
            raise RuntimeError("bond_offset_apply 需要 EGA 结果"
                               "（先跑 wafer_ega）")
        # 旋转：WS_Rz += orz（设计系；物理角=轴位增量×rz_gain×rz_sign 的
        # 逆——与 _fa_apply_comp 同换算），EGA map 同旋（map 帧随台转动）
        if bo_rz:
            rz_gain = float(fa_mc.get("rz_gain", 1.0))
            rz_sign = float(fa_mc.get("rz_sign", 1.0))
            rz_ax = self.eng.axes["WS_Rz"]
            rz_tgt = rz_ax.pos + bo_rz * rz_sign / rz_gain
            if not (rz_ax.lo - 1e-12 <= rz_tgt <= rz_ax.hi + 1e-12):
                raise RuntimeError(
                    "键合附加补偿 orz=%+.1f µrad 使 WS_Rz 目标 %.6f rad "
                    "超出行程 [%.6f, %.6f]（±1°）"
                    % (bo_rz * 1e6, rz_tgt, rz_ax.lo, rz_ax.hi))
            self._do_move("WS_Rz", round(rz_tgt, 9), None, timeout)
            c_, s_ = math.cos(bo_rz), math.sin(bo_rz)
            a11, a12, a21, a22 = fit["a"]
            fit["a"] = [c_ * a11 - s_ * a21, c_ * a12 - s_ * a22,
                        s_ * a11 + c_ * a21, s_ * a12 + c_ * a22]
            btx, bty = fit["tx"], fit["ty"]
            fit["tx"], fit["ty"] = (c_ * btx - s_ * bty,
                                    s_ * btx + c_ * bty)
            fit["theta"] = 0.5 * (fit["a"][2] - fit["a"][1])
        # 平移：设计系 +offset → 世界系（EGA map 线性部）→ WS_XY 增量
        if bo_x or bo_y:
            a11, a12, a21, a22 = fit["a"]
            dwx = a11 * bo_x + a12 * bo_y
            dwy = a21 * bo_x + a22 * bo_y
            self._do_move("WS_LS_X",
                          round(self.eng.axes["WS_LS_X"].pos + dwx, 6),
                          None, timeout)
            self._do_move("WS_LS_Y",
                          round(self.eng.axes["WS_LS_Y"].pos + dwy, 6),
                          None, timeout)
        # WS 已动：fl_ws_frozen 同步刷新（同 _fa_apply_comp 尾部惯例）
        if getattr(self.eng, "fl_ws_frozen", None):
            self.eng.fl_ws_frozen["WS_LS_X"] = self.eng.axes["WS_LS_X"].pos
            self.eng.fl_ws_frozen["WS_LS_Y"] = self.eng.axes["WS_LS_Y"].pos
        info = ("键合附加补偿已施加：offset=(%+.2f, %+.2f) µm orz=%+.2f "
                "µrad（fine_align.bond_offset_*；精对准后/下压前，die 落点"
                "相对 site 标记 −offset，measured mean→0）"
                % (bo_x * 1000.0, bo_y * 1000.0, bo_rz * 1e6))
        self._log("info", info)
        return info

    def _do_pi_measure(self, st):
        """键合后检测单 site 测量（**2026-09-05 迭代 15 重构：红外套刻、
        一张图两次触发**，替代迭代 11 的逐标记单独寻址拍照方案）。

        真实工艺（用户描述）：下相机红外通道穿透一个芯片厚度，焦面=晶圆
        面（下芯片表面），同一画幅同时看到 site 的 quad 标记（下芯片侧）
        与 die 的 cross 标记（上芯片侧），形成套刻图；一张图两个模板识别，
        再用精对准同一算法族算上下芯片相对偏差。

        测量链：
        1) AS 把首个套刻标记对（quad#0/cross#0，同一角落）送到下相机光
           轴（EGA map 寻址：AS=anchor+m+(WS−ws_ega)，与 fa_measure 晶圆
           侧同式；目标超 AS 行程时联动 WS，迭代 11c 逻辑保留——每 site
           只走这一次位，不再逐标记走位）。下相机 FOV 1.8mm（真机值）
           装不下 ±3.8mm 全组角标，一张图拍同一角落的 quad⊕cross 套刻
           对；
        2) **两次独立 identify**（2026-09-07 迭代 20 起：先 quad 模板
           （vision_marks.tpl_wafer_site）后 cross 模板
           （tpl_post_inspect），各带 ir_overlay=True 红外套刻通道，
           同一机位、中间不动轴——用户实测工艺是两次独立触发；迭代 15
           的一次双模板列表契约（"+"strShapeName）保留但流程不再用。
           REAL 下两次触发各 ~0.7s）；
        3) 按标记名分双组（FAST=scene 名 site_R{r}C{c}#k / die_id#k；
           REAL=模板名；FAST 解析不按模板过滤，两次触发的结果各含全部
           在视标记，须按名各取本组再合并），全部点用**同一 cam_pos**
           solve_pixel 到世界系；
        4) measured 解算两模式：FOV 内 ≥2 对（大 FOV/小标记距布局）→
           双组点集各自拟合（fine_align.ALGORITHMS 算法族，算法取处方
           fine_align.algorithm）后 relative_delta 取组间刚体偏差（参考点
           无关形式，迭代 20）；单对
           （本机默认）→ dx/dy=世界系位置差经 map 线性部逆变换回设计系
           减设计差（差分向量系统差全对消）、drz=两条目投影角差×相机
           方向行列式符号换算回世界系（下相机镜像须翻号，迭代 20），
           位置差另扣 drz×J(d0) 旋转项（迭代 20）。
           结果 dx/dy/drz（mm/rad）→ BondResults.update_measured 回填。

        对消性（相对旧方案的根本改进）：两组标记同帧同相机位姿，归算只做
        组间相对刚体变换——EGA map 拟合残差（~1µm）、下相机绝对位姿系统
        差（~2.5-3µm）、逐标记 AS 重复定位、离焦噪声放大全部组内相消，
        不进 measured；EGA map 仅用于走位寻址（粗定位，误差对两组相同）。
        可见光通道（EGA/调平/精对准）行为不变。"""
        if self.eng.vision is None:
            raise RuntimeError("视觉系统未挂载（pi_measure 需要 vision）")
        fit = getattr(self.eng, "ega_result", None)
        if fit is None:
            raise RuntimeError("pi_measure 需要 EGA 结果（先跑 wafer_ega）")
        br = getattr(self.eng, "bond_results", None)
        if br is None:
            raise RuntimeError("bond results not attached")
        import vision_model as vm
        import fine_align as _fa
        sx, sy = float(st["site_x"]), float(st["site_y"])
        die_id = st["die_id"]
        die_pts = [[float(p[0]), float(p[1])] for p in st["die_pts"]]
        site_rc = st.get("site")            # [row, col]（FAST 按名筛 quad 组）
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        mc = self.eng.constants.get()
        we = mc.get("wafer_ega", {})
        ax, ay = (float(v) for v in we.get("as_anchor_xy_mm", [0.0, 0.0]))
        wx0, wy0, _ = (float(v) for v in
                       we.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
        # ---- 1) AS 把 site 心送到下相机光轴（EGA map 寻址，每 site 一次）----
        # ---- 0) 双组设计点（寻址/归算共用）----
        rec = self.eng.recipe.get()
        from recipe import wafer_sites_chip_size
        sm = (rec.get("wafer") or {}).get("site_marks") or {}
        sratio = sm.get("ratio") or []
        sper = min(max(0, int(sm.get("per_site", len(sratio)))), len(sratio))
        shx, shy = (0.5 * v for v in wafer_sites_chip_size(rec))
        quad_design = [[float(sratio[k][0]) * shx, float(sratio[k][1]) * shy]
                       for k in range(sper)]
        if not quad_design:
            raise RuntimeError("键合后检测需要 wafer.site_marks 至少 1 个 "
                               "quad 标记（per_site=0）")
        # ---- 1) AS 把首个套刻标记对（quad#0/cross#0）送到下相机光轴
        # （EGA map 寻址，每 site 只走这一次位）：下相机 FOV 1.8mm（真机
        # 值）装不下 ±3.8mm 的全组角标，一张图拍的是同一角落的
        # quad⊕cross 套刻对；装得下多对的布局自动走点集拟合（下述）----
        wsx0, wsy0 = self.eng.axes["WS_LS_X"].pos, \
            self.eng.axes["WS_LS_Y"].pos
        aim_dx = 0.5 * (quad_design[0][0] + die_pts[0][0])
        aim_dy = 0.5 * (quad_design[0][1] + die_pts[0][1])
        mx, my = ega.apply_map(fit, sx + aim_dx, sy + aim_dy)
        as_x_t = ax + mx + (wsx0 - wx0)
        as_y_t = ay + my + (wsy0 - wy0)
        as_x_ax, as_y_ax = self.eng.axes["AS_X"], self.eng.axes["AS_Y"]
        if not (as_x_ax.lo <= as_x_t <= as_x_ax.hi
                and as_y_ax.lo <= as_y_t <= as_y_ax.hi):
            # 目标超出 AS 行程：联动移动 WS(X/Y) 把 site 带进 AS 可达域
            # （AS 目标 = anchor + m + (WS−ws_ega)，WS 平移 Δ 则 AS 目标同
            # 平移 Δ——取 Δ=clamp(AS目标)−AS目标；迭代 11c 逻辑）
            dwx = min(max(as_x_t, as_x_ax.lo), as_x_ax.hi) - as_x_t
            dwy = min(max(as_y_t, as_y_ax.lo), as_y_ax.hi) - as_y_t
            wsx1, wsy1 = wsx0 + dwx, wsy0 + dwy
            for name, v in (("WS_LS_X", wsx1), ("WS_LS_Y", wsy1)):
                axo = self.eng.axes[name]
                if not (axo.lo <= v <= axo.hi):
                    raise RuntimeError(
                        "pi_measure die %s site 心目标使 AS 超出行程，联动 "
                        "%s=%.3f 亦超出 WS 行程[%.2f,%.2f]（site 不可达）"
                        % (die_id, name, v, axo.lo, axo.hi))
            self._do_move("WS_LS_X", wsx1, None, timeout)
            self._do_move("WS_LS_Y", wsy1, None, timeout)
            as_x_t += dwx
            as_y_t += dwy
            self._log("info",
                      "pi_measure die %s site 心超出 AS 行程：联动 WS→"
                      "(%.4f, %.4f)，AS 目标→(%.4f, %.4f)"
                      % (die_id, wsx1, wsy1, as_x_t, as_y_t))
        for name, v in (("AS_X", as_x_t), ("AS_Y", as_y_t)):
            axo = self.eng.axes[name]
            if not (axo.lo <= v <= axo.hi):
                raise RuntimeError(
                    "pi_measure die %s site 心目标使 %s=%.3f 超出行程"
                    "[%.2f,%.2f]" % (die_id, name, v, axo.lo, axo.hi))
        self._do_move("AS_X", as_x_t, None, timeout)
        self._do_move("AS_Y", as_y_t, None, timeout)
        # ---- 2) 两次独立触发（2026-09-07 迭代 20：先 quad 模板后 cross
        # 模板，各带 ir_overlay=True 红外套刻通道，同一机位、中间不动轴；
        # 迭代 15 的一次 identify 双模板列表契约保留但流程不再用）----
        tpl_w = self._vision_tpl("tpl_wafer_site")
        tpl_d = self._vision_tpl("tpl_post_inspect")
        entry_w = self._vision_identify("down", template=tpl_w,
                                        ir_overlay=True)
        if entry_w.get("error"):
            raise RuntimeError("键合后检测 quad 触发失败：%s"
                               % entry_w["error"])
        entry_d = self._vision_identify("down", template=tpl_d,
                                        ir_overlay=True)
        if entry_d.get("error"):
            raise RuntimeError("键合后检测 cross 触发失败：%s"
                               % entry_d["error"])
        # ---- 3) 按名分双组合并（FAST=scene 名 site_R{r}C{c}#k /
        # die_id#k；REAL=模板名。FAST 解析不按模板过滤，两次触发的结果
        # 都含全部在视标记，须各取本组再合并；REAL 各次只回本模板命中）----
        site_pre = ("site_R%02dC%02d#" % (int(site_rc[0]), int(site_rc[1]))
                    if site_rc else None)

        def _sel(entry, want):
            """从一次触发结果里筛本组条目 -> [(col,row,name,angle)]。"""
            r_ = entry.get("result") or {}
            c_ = r_.get("lstCol") or []
            w_ = r_.get("lstRow") or []
            n_ = r_.get("lstName") or [""] * len(c_)
            a_ = r_.get("lstAngle") or []
            return ([(c_[i], w_[i], n_[i] if i < len(n_) else "",
                      a_[i] if i < len(a_) else 0.0)
                     for i in range(len(c_)) if want(n_[i])],
                    bool(r_.get("judge")))

        quad_sel, judge_w = _sel(
            entry_w, lambda nm: bool((site_pre and nm.startswith(site_pre))
                                     or nm == tpl_w))
        cross_sel, judge_d = _sel(
            entry_d, lambda nm: bool(nm.startswith(die_id + "#")
                                     or nm == tpl_d))
        cols = [e[0] for e in quad_sel] + [e[0] for e in cross_sel]
        rows = [e[1] for e in quad_sel] + [e[1] for e in cross_sel]
        names = [e[2] for e in quad_sel] + [e[2] for e in cross_sel]
        lst_angle = [e[3] for e in quad_sel] + [e[3] for e in cross_sel]
        quad_idx = list(range(len(quad_sel)))
        cross_idx = list(range(len(quad_sel), len(cols)))
        if len(quad_idx) < 1 or len(cross_idx) < 1:
            raise RuntimeError(
                "键合后检测失败：site (%.4f, %.4f) die %s 套刻图双组标记不"
                "齐（两次独立触发：quad 模板 %s %d 个 judge=%s / cross "
                "模板 %s %d 个 judge=%s，各需 ≥1；红外套刻通道未进焦或"
                "标记缺失）"
                % (sx, sy, die_id, tpl_w, len(quad_idx), judge_w,
                   tpl_d, len(cross_idx), judge_d))
        self._log("info",
                  "PI 两次独立触发完成（同机位）：quad（%s）%d 条 / "
                  "cross（%s）%d 条" % (tpl_w, len(quad_idx), tpl_d,
                                        len(cross_idx)))
        # ---- 4) 同一 cam_pos 反投影世界系（组间系统差相消的关键）----
        rt = self.eng.vision.cams.get("down")
        if rt is None:
            raise RuntimeError("未知相机 'down'")
        positions, poses = vm._engine_poses(self.eng)
        cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses, positions)

        def _world(i):
            wx, wy, _wz = rt.model.solve_pixel(cols[i], rows[i], cam_pos)
            return [wx, wy]
        # 设计↔实测配对：FAST 按名尾 #k 精确配；REAL（模板名无序号）按
        # 组质心对齐后贪心最近邻（套刻偏差 µm 级，配对无歧义）。同时保留
        # 各点的结果条目索引（单对模式取 lstAngle 用）
        def _pairs(idx_list, design):
            by_k = {}
            rest = []
            for i in idx_list:
                nm = names[i] if i < len(names) else ""
                if "#" in nm:
                    try:
                        by_k[int(nm.rsplit("#", 1)[1])] = (_world(i), i)
                        continue
                    except ValueError:
                        pass
                rest.append((_world(i), i))
            if by_k and not rest:
                d, m, ii = [], [], []
                for k in sorted(by_k):
                    if k < len(design):
                        d.append(design[k])
                        m.append(by_k[k][0])
                        ii.append(by_k[k][1])
                return d, m, ii
            cx = sum(p[0][0] for p in rest) / len(rest)
            cy = sum(p[0][1] for p in rest) / len(rest)
            pool = list(rest)
            d, m, ii = [], [], []
            for dp in design:
                tx, ty = cx + dp[0], cy + dp[1]
                j = min(range(len(pool)),
                        key=lambda j2: (pool[j2][0][0] - tx) ** 2
                        + (pool[j2][0][1] - ty) ** 2)
                d.append(dp)
                m.append(pool[j][0])
                ii.append(pool[j][1])
                pool.pop(j)
                if not pool:
                    break
            return d, m, ii

        qd, qm, qi = _pairs(quad_idx, quad_design)
        cd, cm, ci = _pairs(cross_idx, die_pts)
        if len(qd) != len(cd):
            # 两侧在 FOV 内对数不一致：取前 min 对（_pairs 已按 #k 排序/
            # 最近邻对齐）
            nn = min(len(qd), len(cd))
            qd, qm, qi, cd, cm, ci = (qd[:nn], qm[:nn], qi[:nn],
                                      cd[:nn], cm[:nn], ci[:nn])
        # ---- 5) measured 解算 ----
        a11, a12, a21, a22 = (float(v) for v in fit["a"])
        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-12:
            raise RuntimeError("EGA map 奇异（行列式≈0），无法归算")
        pi_warn = None            # drz 物理门限判伪差留证（迭代 23，单对）
        if len(qd) >= 2:
            # 多对（大 FOV/小标记距布局）：双组点集各自拟合（精对准算法
            # 族，算法=处方 fine_align.algorithm），measured=cross 组相对
            # quad 组的刚体偏差——世界系点云距帧原点远，用
            # relative_delta（参考点无关；compose_rigid 在此会引入
            # θ_Δ×J(F) 参考原点耦合假平移，迭代 20）
            algo = ((rec or {}).get("fine_align") or {}).get("algorithm",
                                                             "rigid")
            if algo == "affine" and (len(qd) < 3 or len(cd) < 3):
                raise RuntimeError(
                    "affine 拟合需要双侧各 ≥3 点（quad %d/cross %d），请加"
                    "大标记数或换算法" % (len(qd), len(cd)))
            fit_w = _fa.fit(algo, qd, qm)
            fit_d = _fa.fit(algo, cd, cm)
            delta = _fa.relative_delta(fit_d, fit_w)
            method = "点集拟合 %s（quad %d/cross %d）" % (algo, len(qd),
                                                          len(cd))
        else:
            # 单对模式（本机默认：下相机 FOV 1.8mm（真机值）一画幅只装得
            # 下一对套刻对）：dx/dy=世界系相对偏移经 EGA map 线性部逆变换
            # 回设计系再减设计差（差分向量，平移/相机位姿/EGA 系统差全对
            # 消）；drz=两条目投影角差换算回世界系（2026-09-07 迭代 20 修
            # 正两处此前被小角度掩盖的问题：①投影角是像系量，世界 yaw 差
            # = 像角差 × 相机方向行列式符号 sign(dir_x·dir_y·(swap?−1:1))
            # ——下相机 dir_y=−1 镜像，旧代码直接拿像角差当世界角差，符号
            # 反；②单对位置差里含 die 相对 quad 的旋转项 drz×J(d0)（角标
            # 在半径 ~3.8mm 处，36µrad≈0.14µm），先测 drz 再对 dw 做旋转
            # 修正（多点对模式由 fit 天然处理，单对须显式扣））
            angs = lst_angle
            a_q = math.radians(angs[qi[0]]) if qi[0] < len(angs) else 0.0
            a_c = math.radians(angs[ci[0]]) if ci[0] < len(angs) else 0.0
            cam_sign = rt.model.dir_x * rt.model.dir_y * \
                (-1.0 if rt.model.swap_xy else 1.0)
            drz_w = cam_sign * (a_c - a_q)
            # 物理门限守卫（迭代 23）：|drz_w|>500µrad 判为角度测量伪差
            # （EGA+fa 后真值 drz 不可能达到），旋转修正按 0 处理——否则
            # 伪差 ×|d0|≈3.8mm 杠杆把 ±10µm 假误差注进 measured dx/dy；
            # measured drz（delta["theta"]）仍取原值留证，记录带
            # measured_warning（pi_warn）
            drz_corr = drz_w
            if abs(drz_corr) > PI_DRZ_GATE_RAD:
                pi_warn = ("PI 单对 drz=%+.0f µrad 超物理门限 ±%.0f µrad"
                           "——判为角度测量伪差，旋转修正按 0 处理"
                           % (drz_corr * 1e6, PI_DRZ_GATE_RAD * 1e6))
                self._log("warn", pi_warn)
                drz_corr = 0.0
            # 旋转修正：角标世界偏移 ≈ R(θq+drz)·d0 vs R(θq)·d0，一阶项
            # drz×J(d0)（θq=rz+wrz 小角忽略）；d0 取两侧设计角标均值
            d0x = 0.5 * (cd[0][0] + qd[0][0])
            d0y = 0.5 * (cd[0][1] + qd[0][1])
            dw = (cm[0][0] - qm[0][0] - drz_corr * (-d0y),
                  cm[0][1] - qm[0][1] - drz_corr * d0x)
            ex, ey = ((a22 * dw[0] - a12 * dw[1]) / det,
                      (-a21 * dw[0] + a11 * dw[1]) / det)
            delta = {"tx": ex - (cd[0][0] - qd[0][0]),
                     "ty": ey - (cd[0][1] - qd[0][1]),
                     "theta": drz_w}
            method = "单对套刻（位置差+角度差，旋转修正）"
            self._log("info",
                      "PI 单对：quad(%s#%d) ang=%.6f° cross(%s#%d) ang=%.6f° "
                      "Δang=%+.2f µrad（世界 drz=%+.2f µrad）；旋转修正后 "
                      "dw=(%+.3f,%+.3f) µm"
                      % (site_pre or tpl_w, 0, math.degrees(a_q),
                         die_id, 0, math.degrees(a_c), (a_c - a_q) * 1e6,
                         drz_w * 1e6, dw[0] * 1000.0, dw[1] * 1000.0))
        # ---- 6) measured 回填（迭代 23：(die_id, site) 双键+最新无
        # measured 记录——同 die 复跑不再挂错到上一轮记录；drz 物理门限
        # 判伪差的记录带 measured_warning 留证）----
        with self.eng.lock:
            t_sim = self.eng.sim_time
        if not br.update_measured(die_id, delta["tx"], delta["ty"],
                                  delta["theta"], t_sim, site=site_rc,
                                  warn=pi_warn):
            raise RuntimeError(
                "键合后检测回填失败：无 die %r 的键合记录（先键合再检测）"
                % die_id)
        info = ("键合后检测 site (%.4f, %.4f) die %s：红外套刻两次独立触发"
                "（%s），measured dx=%+.3f µm dy=%+.3f µm drz=%+.3f µrad"
                "（套刻相对测量，EGA/相机位姿系统差已对消）"
                % (sx, sy, die_id, method,
                   delta["tx"] * 1000.0, delta["ty"] * 1000.0,
                   delta["theta"] * 1e6))
        self._log("info", info)
        return info

    def _do_coarse_offset(self, st):
        """粗对准测量（需求 1~3，2026-08-25）：identify 拍照识别 die 背面
        特征像素 → 反投影世界坐标（solve_pixel：像素当量/方向性 dir/swap/
        安装微旋按处方逆映射，焦面平面）→ 与机械常数标定中心 coarse_die_center_mm
        对比 → 偏差进流程日志与步骤信息。误差传递链：膜上片误差 → die 在吸盘真空面上的位置偏差
        （workpieces.pick_offset 可注入）→ 粗对准世界坐标偏差（换算口径
        参考 d2w_sim_semiconductor_refactor sim/vision.py Camera.solve）。"""
        if self.eng.vision is None:
            raise RuntimeError("视觉系统未挂载（coarse_offset 需要 vision）")
        import vision_model as vm
        cam_id = st.get("cam", "coarse")
        entry = self._vision_identify(
            cam_id, template=self._vision_tpl("tpl_coarse_die"))
        if entry.get("error"):
            raise RuntimeError("粗对准拍照失败：%s" % entry["error"])
        res = entry.get("result") or {}
        cols = res.get("lstCol") or []
        rows = res.get("lstRow") or []
        if not cols or not res.get("judge"):
            raise RuntimeError(st.get("err") or
                               "粗对准失败：视野内无芯片背面特征"
                               "（未拾取到芯片或未进焦）")
        rt = self.eng.vision.cams.get(cam_id)
        if rt is None:
            raise RuntimeError("未知相机 %r" % cam_id)
        positions, poses = vm._engine_poses(self.eng)
        cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses, positions)
        wx, wy, wz = rt.model.solve_pixel(cols[0], rows[0], cam_pos)
        mc = self.eng.constants.get()["flip_pick"]
        tx, ty = (float(v) for v in mc["coarse_die_center_mm"])
        off = (wx - tx, wy - ty)
        # 偏差落存（交接流程 flip_handoff 消费做 FA_X 补偿；die=当前 flip
        # 上的 die id，交接守卫按它防串用旧测量）
        die_id = None
        wp = self.eng.workpieces
        if wp is not None:
            with wp._lock:
                flips = [d for d in wp.dies if d["carrier"] == "flip"]
                die_id = flips[0]["id"] if flips else None
        with self.eng.lock:
            t_sim = self.eng.sim_time
        self.eng.coarse_offset = {"dx": off[0], "dy": off[1],
                                  "die": die_id, "t": round(t_sim, 3)}
        info = ("粗对准测量：die 世界 (%.4f, %.4f) mm（焦面 z=%.3f），"
                "标定中心 (%.4f, %.4f)，偏差 (%+.4f, %+.4f) mm"
                % (wx, wy, wz, tx, ty, off[0], off[1]))
        self._log("info", info)
        return info

    def _ega_split_targets(self, site_x, site_y):
        """把 site 名义坐标拆成 AS/WS 两轴的目标位置，使
        (AS - anchor) - (WS - ws_ega) = site，且都在各自软限位内。
        在 AS 小行程 / WS 大行程的本机上，优先让 WS 承担大部分位移。
        返回 {"AS_X":..., "AS_Y":..., "WS_LS_X":..., "WS_LS_Y":...}。
        """
        import vision_model as vm
        mc = self.eng.constants.get().get("wafer_ega", {})
        ax, ay = (float(v) for v in mc.get("as_anchor_xy_mm", [0.0, 0.0]))
        wx, wy, _ = (float(v) for v in mc.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
        axes = ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")
        cur = {n: self.eng.axes[n].pos for n in axes}
        limits = {n: (self.eng.axes[n].lo, self.eng.axes[n].hi) for n in axes}
        best, best_cost = None, float("inf")
        for i in range(21):          # f = 0..1，步长 0.05
            f = i * 0.05
            asx = ax + site_x * f
            asy = ay + site_y * f
            wsx = wx - site_x * (1.0 - f)
            wsy = wy - site_y * (1.0 - f)
            t = {"AS_X": asx, "AS_Y": asy, "WS_LS_X": wsx, "WS_LS_Y": wsy}
            if any(not (limits[n][0] <= t[n] <= limits[n][1]) for n in axes):
                continue
            cost = sum(abs(t[n] - cur[n]) for n in axes)
            if cost < best_cost:
                best, best_cost = t, cost
        if best is None:
            raise RuntimeError(
                "EGA 测点 (%.3f, %.3f) 超出 AS/WS 联合可达范围" % (site_x, site_y))
        return best

    def _do_ega_measure(self, st):
        """EGA 单点测量：移动对准台/硅片台使 site 标记进入下精对准相机视场，
        触发拍照识别，反投影世界坐标，记录（名义坐标，实测坐标）对。"""
        if self.eng.vision is None:
            raise RuntimeError("视觉系统未挂载（ega_measure 需要 vision）")
        import vision_model as vm
        sx, sy = float(st["site_x"]), float(st["site_y"])
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        targets = self._ega_split_targets(sx, sy)
        # 硅片台先动、对准台后动（大行程先就位，减少相机磕碰风险）
        for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y"):
            self._do_move(ax, targets[ax], None, timeout)
        # 触发下相机拍照（与 coarse_offset 用同一相机接口）
        cam_id = st.get("cam", "down")
        entry = self._vision_identify(
            cam_id, template=self._vision_tpl("tpl_wafer_site"))
        if entry.get("error"):
            raise RuntimeError("EGA 拍照失败：%s" % entry["error"])
        res = entry.get("result") or {}
        cols = res.get("lstCol") or []
        rows = res.get("lstRow") or []
        if not cols or not res.get("judge"):
            raise RuntimeError("EGA 测点失败：视野内无标记（site %.3f,%.3f 未进焦或"
                               " 标记缺失）" % (sx, sy))
        rt = self.eng.vision.cams.get(cam_id)
        if rt is None:
            raise RuntimeError("未知相机 %r" % cam_id)
        positions, poses = vm._engine_poses(self.eng)
        cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses, positions)
        wx, wy, _wz = rt.model.solve_pixel(cols[0], rows[0], cam_pos)
        # solve_pixel 返回标记绝对世界坐标；应减去当前相机光轴世界位姿、
        # 加回名义 site 坐标，得到“相对晶圆中心的实际 site 坐标”后再拟合。
        measured_rel = (wx - cam_pos[0] + sx, wy - cam_pos[1] + sy)
        measured = measured_rel
        rec = {"nominal": [round(sx, 6), round(sy, 6)],
               "measured": [round(measured[0], 6), round(measured[1], 6)],
               "site": [round(sx, 6), round(sy, 6)]}
        self.eng.ega_measurements.append(rec)
        info = ("EGA 测点 #%d：名义 (%.4f, %.4f) → 实测 (%.4f, %.4f) mm"
                % (len(self.eng.ega_measurements), sx, sy,
                   measured[0], measured[1]))
        self._log("info", info)
        return info

    def _do_ega_fit(self):
        """对累计的 EGA 测量点做六参数仿射拟合，结果落存 engine.ega_result。"""
        meas = getattr(self.eng, "ega_measurements", [])
        if len(meas) < 3:
            raise RuntimeError("EGA 拟合需要至少 3 个测点，当前 %d 个" % len(meas))
        nominal = [m["nominal"] for m in meas]
        measured = [m["measured"] for m in meas]
        fit = ega.fit_similarity(nominal, measured)
        if fit is None:
            raise RuntimeError("EGA 拟合失败（fit_similarity 返回 None）")
        mc = self.eng.constants.get().get("wafer_ega", {})
        fit["anchor"] = {
            "as_anchor_xy_mm": list(mc.get("as_anchor_xy_mm", [0.0, 0.0])),
            "ws_ega_xyz_mm": list(mc.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0])),
        }
        fit["measurements"] = copy.deepcopy(meas)
        self.eng.ega_result = fit
        info = ("EGA 拟合完成（n=%d）：tx=%+.4f µm ty=%+.4f µm θ=%+.4f urad "
                "sx=%+.2f ppm sy=%+.2f ppm ortho=%+.4f urad 3σ=%.3f µm"
                % (fit["n"], fit["tx"] * 1000, fit["ty"] * 1000,
                   fit["theta"] * 1e6, fit["sx"] * 1e6, fit["sy"] * 1e6,
                   fit["ortho"] * 1e6, fit["resid_3s"] * 1000))
        self._log("info", info)
        return info

    # ---------------- 调焦调平（fl_measure / fl_solve） ----------------
    def _fl_read(self, probe, timeout=5.0):
        """经 MQTT 读一次共焦：发布 confocal/trigger/{probe} → 等
        confocal/result/{probe} 上对应 req_id 的消息（契约见
        docs/前后端数据协议_V1.md §12；ConfocalClient 后台收线线程与流程
        线程用 Condition 解耦，回调不碰引擎状态）。客户端未挂载（单测/
        裸引擎场景）回退进程内直读。"""
        cli = getattr(self.eng, "confocal_client", None)
        if cli is None:
            return self.eng.confocal.read(probe)
        return cli.request(probe, timeout)

    def _vision_tpl(self, key):
        """按用途从机械常数 vision_marks 组取对准标记模板名（2026-09-04
        迭代 12：每次触发相机拍照必须依据模板名识别标记，模板名来源=
        机械常数，不随处方变化）。"""
        vm = self.eng.constants.get().get("vision_marks", {})
        tpl = vm.get(key)
        if not tpl:
            raise RuntimeError(
                "机械常数 vision_marks.%s 未配置（对准识别必须指定模板名）"
                % key)
        return tpl

    def _vision_identify(self, cam_id, template=None, timeout=None,
                         ir_overlay=False):
        """经 MQTT 拍一次照：发布 vision/trigger/{cam} → 等
        vision/result/{cam} 上对应 req_id 的消息（契约见
        docs/前后端数据协议_V1.md §9；VisionClient 与流程线程用
        Condition 解耦，同 _fl_read 模式）。客户端未挂载（单测/裸引擎
        场景）回退进程内 identify。返回 entry 形态 dict（含 result /
        error），调用点判 error 抛 RuntimeError 的语义不变。
        template 可为模板名列表（迭代 15 契约保留：多模板一次触发
        strShapeName "+" 连接；迭代 20 起 post-inspect 流程改为两次独立
        触发，不再用列表）、ir_overlay=红外套刻通道。
        timeout 缺省按当前视觉模式自适应（2026-09-05 迭代 17）：real→30s
        （9MP 渲染+匹配单拍亚秒级，留足余量防误判超时卡链）、fast→10s；
        模式从 eng.vision 直读（本类已持有 eng 句柄，无循环依赖）。"""
        cli = getattr(self.eng, "vision_client", None)
        if timeout is None:
            mode = getattr(getattr(self.eng, "vision", None), "mode", "fast")
            timeout = 30.0 if mode == "real" else 10.0
        if cli is None:
            return self.eng.vision.identify(cam_id, template,
                                            ir_overlay=ir_overlay)
        return cli.request(cam_id, template, timeout,
                           ir_overlay=ir_overlay)

    def _fl_align_first(self, probe, space, px, py, cam_dx, cam_dy,
                        timeout, vm, freeze_ws=False):
        """先对准再测量（align_first，参照参考项目 level_use_meas）：
        相机到设计点 → 视觉 identify 实测标记 → 像素→世界换算 → 按认知链
        （相机光轴+标定偏移）把共焦探头送到**实测**点。返回 info 附加
        文本；identify 失败抛异常（步骤失败→流程 abort，与 ega_measure
        一致）。相机段与共焦段一样走 MQTT（_vision_identify，迭代 8 视觉
        MQTT 化，客户端未挂载回退进程内）。
        2026-08-27 迭代 4 零位纠错：真实零位下上相机光轴=(-42.5,1.0)，
        认键合头吸盘中心只需 AS_Y=+6.0（原行程 [-30,120] 内），die 侧
        align_first 不再降级；wafer 空间支持 freeze_ws（per_site 测高期间 WS
        冻结，仅 AS 走位）。"""
        cam_id = probe              # 探头与相机一一对应：up↔up、down↔down
        # 1) 相机到设计点（不带共焦偏移——先让相机看设计位置）
        if space in ("wafer", "wafer_v"):
            fit = getattr(self.eng, "ega_result", None)
            if fit is None:
                raise RuntimeError("align_first 需要 EGA 结果"
                                   "（先跑 wafer_ega）")
            mx, my = ega.apply_map(fit, px, py)
            if freeze_ws:
                # WS 冻结：仅 AS 承担位移，保持 (AS-anchor)-(WS-ws_ega)=m
                frozen = getattr(self.eng, "fl_ws_frozen", None) or {}
                mc = self.eng.constants.get().get("wafer_ega", {})
                ax, ay = (float(v) for v in mc.get("as_anchor_xy_mm",
                                                    [0.0, 0.0]))
                wx0, wy0, _ = (float(v) for v in
                               mc.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
                cur_ws = {
                    "WS_LS_X": frozen.get("WS_LS_X",
                                          self.eng.axes["WS_LS_X"].pos),
                    "WS_LS_Y": frozen.get("WS_LS_Y",
                                          self.eng.axes["WS_LS_Y"].pos)}
                # 设计点 m 对应的 AS 目标（WS 在 ws_ega 时）
                as_aim_x = ax + mx
                as_aim_y = ay + my
                # WS 当前相对 ws_ega 的偏移需由 AS 同步补偿
                as_x_t = as_aim_x + (cur_ws["WS_LS_X"] - wx0)
                as_y_t = as_aim_y + (cur_ws["WS_LS_Y"] - wy0)
                self._do_move("AS_X", as_x_t, None, timeout)
                self._do_move("AS_Y", as_y_t, None, timeout)
            else:
                targets = self._ega_split_targets(mx, my)
                for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y"):
                    self._do_move(ax, targets[ax], None, timeout)
        else:
            positions, poses = vm._engine_poses(self.eng)
            rt = self.eng.vision.cams[cam_id]
            cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses,
                                           positions)
            cur = {n: self.eng.axes[n].pos for n in ("AS_X", "AS_Y")}
            if space in ("die", "die_v"):
                # 2026-08-27 迭代 3：die 空间寻址基准改为机械常数
                # as_head_center_xy_mm（上相机认吸盘中心时的 AS 轴位）+
                # die 角标设计偏移，不再用 die_center_world 真值定位。
                as_head = self.eng.constants.get()["focus_level"][
                    "as_head_center_xy_mm"]
                as_x_t = as_head[0] + px
                as_y_t = as_head[1] + py
            else:                               # stage：世界 XY 直给
                as_x_t = cur["AS_X"] + (px - cam_pos[0])
                as_y_t = cur["AS_Y"] + (py - cam_pos[1])
            # 迭代 3：行程守卫，超限时硬失败（不再降级）
            if not (self.eng.axes["AS_X"].lo <= as_x_t
                    <= self.eng.axes["AS_X"].hi
                    and self.eng.axes["AS_Y"].lo <= as_y_t
                    <= self.eng.axes["AS_Y"].hi):
                raise RuntimeError(
                    "先对准 %s 相机目标 (%.3f, %.3f) 超出 AS 行程"
                    "（AS_X=[%.2f,%.2f], AS_Y=[%.2f,%.2f]）"
                    % (cam_id, as_x_t, as_y_t,
                       self.eng.axes["AS_X"].lo, self.eng.axes["AS_X"].hi,
                       self.eng.axes["AS_Y"].lo, self.eng.axes["AS_Y"].hi))
            self._do_move("AS_X", as_x_t, None, timeout)
            self._do_move("AS_Y", as_y_t, None, timeout)
        # 2) 视觉实测标记（经 _vision_identify：MQTT 触发→订阅消费，
        #    客户端未挂载回退进程内；模板按用途取机械常数 vision_marks：
        #    晶圆侧=晶圆 site 标记，die 侧=键合头上 die 角标，stage 按相机
        #    兜底——up=吸盘标记 / down=晶圆 site 标记）
        if space in ("die", "die_v"):
            tpl = self._vision_tpl("tpl_die_head")
        elif space in ("wafer", "wafer_v"):
            tpl = self._vision_tpl("tpl_wafer_site")
        else:
            tpl = self._vision_tpl("tpl_head_chuck" if cam_id == "up"
                                   else "tpl_wafer_site")
        entry = self._vision_identify(cam_id, template=tpl)
        if entry.get("error"):
            raise RuntimeError("先对准拍照失败：%s" % entry["error"])
        res = entry.get("result") or {}
        cols = res.get("lstCol") or []
        rows = res.get("lstRow") or []
        if not cols or not res.get("judge"):
            raise RuntimeError("先对准失败：%s 相机视野内无标记（设计点 "
                               "(%.3f, %.3f) 未进焦或标记缺失）"
                               % (cam_id, px, py))
        rt = self.eng.vision.cams.get(cam_id)
        if rt is None:
            raise RuntimeError("未知相机 %r" % cam_id)
        positions, poses = vm._engine_poses(self.eng)
        cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses, positions)
        wx, wy, _wz = rt.model.solve_pixel(cols[0], rows[0], cam_pos)
        # 3) 共焦走位（认知链：探头被认为在 相机光轴+(dx,dy)，送它到实测
        #    点；偏差在相机视场量级，仅 AS 微调，WS 不动）
        cur = {n: self.eng.axes[n].pos for n in ("AS_X", "AS_Y")}
        self._do_move("AS_X", cur["AS_X"] + (wx - (cam_pos[0] + cam_dx)),
                      None, timeout)
        self._do_move("AS_Y", cur["AS_Y"] + (wy - (cam_pos[1] + cam_dy)),
                      None, timeout)
        return "（先对准 %s：实测标记偏移 (%+.4f, %+.4f) mm）" % (
            cam_id, wx - cam_pos[0], wy - cam_pos[1])

    def _do_fl_ready(self, st):
        """调焦调平键合位就位（2026-08-27 迭代 3）：
        1) 键合头先退安全高度（head_z_safe_mm）；
        2) WS 把目标 site 送到键合头吸盘中心正下方（EGA 映射反解 WS 目标）；
        3) AS 到 as_head_center_xy_mm（上相机认吸盘中心时的对准台轴位）；
        4) 记录当前 WS 位置，后续 per_site 测高冻结 WS。
        若已在位（容差 0.05mm/0.5mm）则跳过移动只记日志。"""
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        fl = self.eng.constants.get()["focus_level"]
        fh = self.eng.constants.get()["flip_handoff"]
        fp = self.eng.constants.get()["flip_pick"]
        mc = self.eng.constants.get().get("wafer_ega", {})
        head_z_safe = float(fh["head_z_safe_mm"])
        fa_z_safe = float(fp["fa_z_safe_mm"])
        fa_x_retreat = float(fh["fa_x_retreat_mm"])
        as_head = fl["as_head_center_xy_mm"]
        ax, ay = (float(v) for v in mc.get("as_anchor_xy_mm", [0.0, 0.0]))
        wx0, wy0, _ = (float(v) for v in
                       mc.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
        fit = getattr(self.eng, "ega_result", None)
        if fit is None:
            raise RuntimeError("fl_ready 需要 EGA 结果（先跑 wafer_ega）")
        sx, sy = float(st["site_x"]), float(st["site_y"])
        mx, my = ega.apply_map(fit, sx, sy)
        # 让吸盘中心正对 site：site 在吸盘中心正下方 ⇔ (AS-anchor)-(WS-ws_ega)=m
        # AS 目标 = as_head，反解 WS = ws_ega + (as_head - anchor) - m
        ws_x_t = wx0 + (as_head[0] - ax) - mx
        ws_y_t = wy0 + (as_head[1] - ay) - my
        cur = {n: self.eng.axes[n].pos for n in
               ("BS_Arm_Z", "FA_Z", "FA_X", "WS_LS_X", "WS_LS_Y",
                "AS_X", "AS_Y")}
        # 头/翻转安全位
        for name, target in (("BS_Arm_Z", head_z_safe),
                             ("FA_Z", fa_z_safe),
                             ("FA_X", fa_x_retreat)):
            if abs(cur[name] - target) > 0.05:
                self._do_move(name, target, None, timeout)
            else:
                self._log("info", "%s 已在安全位 %.4f，跳过" % (name, target))
        # WS 送 site 到吸盘中心下
        ws_tol = 0.05
        ws_moved = False
        for name, target in (("WS_LS_X", ws_x_t), ("WS_LS_Y", ws_y_t)):
            if abs(cur[name] - target) > ws_tol:
                self._do_move(name, target, None, timeout)
                ws_moved = True
            else:
                self._log("info", "%s 已使 site 在头下（%.4f≈%.4f），跳过"
                          % (name, cur[name], target))
        # AS 到吸盘中心对准位
        as_moved = False
        for name, target in (("AS_X", as_head[0]), ("AS_Y", as_head[1])):
            if abs(cur[name] - target) > ws_tol:
                self._do_move(name, target, None, timeout)
                as_moved = True
            else:
                self._log("info", "%s 已在吸盘中心对准位 %.4f，跳过"
                          % (name, target))
        # 冻结 WS 位置供后续 freeze_ws 使用
        self.eng.fl_ws_frozen = {
            "WS_LS_X": self.eng.axes["WS_LS_X"].pos,
            "WS_LS_Y": self.eng.axes["WS_LS_Y"].pos,
            "site": [round(sx, 6), round(sy, 6)],
            "as_head": [round(as_head[0], 6), round(as_head[1], 6)]}
        info = ("键合位就位：site(%.3f,%.3f) → WS(%.4f,%.4f) AS(%.4f,%.4f)；"
                "%s" % (sx, sy, self.eng.axes["WS_LS_X"].pos,
                        self.eng.axes["WS_LS_Y"].pos,
                        self.eng.axes["AS_X"].pos,
                        self.eng.axes["AS_Y"].pos,
                        "WS/AS 已冻结" if not (ws_moved or as_moved)
                        else "WS/AS 移动到位并冻结"))
        self._log("info", info)
        return info

    def _do_fl_measure(self, st):
        """共焦单点测高（调焦调平传感步，2026-08-27）：
        走位使共焦探头对准目标点 → 读共焦（**经 MQTT 触发→订阅消费**，
        见 _fl_read）→ 无读数则 AS_Z 扫描找有效窗（每次读数都走 MQTT
        往返）→ 面高换算记录。约定（语义固定，勿随意改）：
        - 记录的面高 z_surface 由读数+焦面换算：down 探头
          zs = z_focus + (reading−mid)/1000，up 探头 zs = z_focus −
          (reading−mid)/1000——与搜索时 AS_Z 位置无关，因此搜索命中后
          **恢复 AS_Z 到测量前位置**再记录，后续测点从同一基准出发；
        - world = 探头实际世界 XY（confocal.probe_pose 真值；搜索只动 Z，
          XY 不变）；wafer 系空间另记 plane_xy（晶圆台基准系=世界 XY−
          (WS−ws_ega)，global 允许测点间 WS 运动，世界系下晶圆面不是
          固定平面，迭代 5 修正），平面拟合用 plane_xy/world（坡度可
          直接与真值坡度对比）；
        - 走位全走**"相机光轴+标定偏移"认知链**：探头被认为在
          相机光轴世界 XY + (confocal_cam_dx_mm, confocal_cam_dy_mm)
          （相机位姿由 vision_model 合成，偏移取机械常数——改错常数 →
          测点偏移 → 拟合面歪，真实测校行为）；wafer 空间点的对准系
          坐标 aim 由流程生成器在生成时刻用 EGA 结果+dx/dy 预算好，
          步骤里缺 aim 时运行时取 eng.ega_result 现算（兜底，手写步骤用）；
        - align_first=true 时先对准再测量（_fl_align_first）：相机到设计
          点 → 视觉 identify 实测标记 → 认知链送探头到实测点；
        - space 的 wafer_v/die_v（调平复测记录空间，迭代 5）走位语义分别
          同 wafer/die，仅记录落存键不同。"""
        if self.eng.confocal is None:
            raise RuntimeError("共焦模型未挂载（fl_measure 需要 confocal）")
        if self.eng.vision is None:
            raise RuntimeError("视觉系统未挂载（fl_measure 走位认知链需要 "
                               "vision）")
        import vision_model as vm
        probe = st["probe"]
        space = st["space"]
        px, py = float(st["point"][0]), float(st["point"][1])
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        fl = self.eng.constants.get()["focus_level"]
        cam_dx = float(fl["confocal_cam_dx_mm"])
        cam_dy = float(fl["confocal_cam_dy_mm"])

        # 1) 走位：探头对准目标点
        freeze_ws = bool(st.get("freeze_ws"))
        align_note = ""
        if bool(st.get("align_first")):
            align_note = self._fl_align_first(
                probe, space, px, py, cam_dx, cam_dy, timeout, vm,
                freeze_ws=freeze_ws)
        elif space in ("wafer", "wafer_v"):
            aim = st.get("aim")
            if aim is None:
                fit = getattr(self.eng, "ega_result", None)
                if fit is None:
                    raise RuntimeError("fl_measure 缺 aim 且无 EGA 结果"
                                       "（先跑 wafer_ega）")
                mx, my = ega.apply_map(fit, px, py)
                aim = [mx - cam_dx, my - cam_dy]
            if freeze_ws:
                # WS 冻结：仅 AS 走位，保持 (AS-anchor)-(WS-ws_ega)=aim
                frozen = getattr(self.eng, "fl_ws_frozen", None) or {}
                mc = self.eng.constants.get().get("wafer_ega", {})
                ax, ay = (float(v) for v in mc.get("as_anchor_xy_mm",
                                                    [0.0, 0.0]))
                wx0, wy0, _ = (float(v) for v in
                               mc.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
                cur_ws = {
                    "WS_LS_X": frozen.get("WS_LS_X",
                                          self.eng.axes["WS_LS_X"].pos),
                    "WS_LS_Y": frozen.get("WS_LS_Y",
                                          self.eng.axes["WS_LS_Y"].pos)}
                as_x_t = ax + float(aim[0]) + (cur_ws["WS_LS_X"] - wx0)
                as_y_t = ay + float(aim[1]) + (cur_ws["WS_LS_Y"] - wy0)
                # 生成时刻守卫：AS 行程超限直接失败
                for name, v in (("AS_X", as_x_t), ("AS_Y", as_y_t)):
                    axo = self.eng.axes[name]
                    if not (axo.lo <= v <= axo.hi):
                        raise RuntimeError(
                            "fl_measure freeze_ws 目标使 %s=%.3f 超出行程"
                            "[%.2f,%.2f]" % (name, v, axo.lo, axo.hi))
                self._do_move("AS_X", as_x_t, None, timeout)
                self._do_move("AS_Y", as_y_t, None, timeout)
            else:
                targets = self._ega_split_targets(float(aim[0]), float(aim[1]))
                # 硅片台先动、对准台后动（同 EGA 测点顺序）
                for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y"):
                    self._do_move(ax, targets[ax], None, timeout)
        else:
            # 认知链：探头被认为在 相机光轴世界 XY + (cam_dx, cam_dy)
            # （不再用 probe_pose 真值——改常数才影响测量点，真实测校行为）。
            # 先取当前相机世界位姿，再把“探头目标”换算成“相机目标 AS 坐标”。
            positions, poses = vm._engine_poses(self.eng)
            rt = self.eng.vision.cams[probe]
            cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses,
                                           positions)
            cur = {n: self.eng.axes[n].pos for n in ("AS_X", "AS_Y")}
            if space in ("die", "die_v"):
                # 迭代 3：die 空间寻址基准改为机械常数 as_head_center_xy_mm
                # + die 角标设计偏移；真值只用于读数物理，不再定位。
                # 相机目标 AS 坐标 = as_head + point - confocal_offset
                as_head = self.eng.constants.get()["focus_level"][
                    "as_head_center_xy_mm"]
                as_x_t = as_head[0] + px - cam_dx
                as_y_t = as_head[1] + py - cam_dy
            else:                                   # stage：世界 XY 直给
                # 相机目标世界 = 目标世界 - confocal_offset，再转 AS 坐标
                as_x_t = cur["AS_X"] + (px - cam_dx - cam_pos[0])
                as_y_t = cur["AS_Y"] + (py - cam_dy - cam_pos[1])
            self._do_move("AS_X", as_x_t, None, timeout)
            self._do_move("AS_Y", as_y_t, None, timeout)

        # 2) 读共焦；无读数 → AS_Z 扫描找有效窗（共焦 700µm 量程 ≫
        #    相机 ±20µm 景深，共焦快速定位焦面）
        z0 = self.eng.axes["AS_Z"].pos
        r = self._fl_read(probe)
        searched, search_steps = False, 0
        if not r["valid"]:
            span = float(fl["focus_search_span_um"]) / 1000.0
            step = float(fl["focus_search_step_um"]) / 1000.0
            lo, hi = self.eng.axes["AS_Z"].lo, self.eng.axes["AS_Z"].hi
            n = max(1, int(span / 2.0 / step + 0.5))
            found = None
            for k in range(1, n + 1):
                for zc in (z0 - k * step, z0 + k * step):
                    if not (lo <= zc <= hi):
                        continue
                    self._do_move("AS_Z", round(zc, 6), None, timeout)
                    search_steps += 1
                    rr = self._fl_read(probe)
                    if rr["valid"]:
                        found = rr
                        break
                if found is not None:
                    break
            # 恢复 AS_Z 到测量前位置（面高换算与搜索位无关，见 docstring）
            self._do_move("AS_Z", z0, None, timeout)
            if found is None:
                raise RuntimeError(
                    "共焦 %s 探头在 (%.3f, %.3f) 无读数，AS_Z ±%.0fµm 扫描 "
                    "%d 步仍未找到有效窗（面高超出搜索范围）"
                    % (probe, px, py, span * 1000, search_steps))
            r, searched = found, True

        # 3) 面高换算 + 记录
        mid = float(fl["confocal_mid_down_um"] if probe == "down"
                    else fl["confocal_mid_up_um"])
        z_est = (r["z_focus"] + (r["reading_um"] - mid) / 1000.0
                 if probe == "down"
                 else r["z_focus"] - (r["reading_um"] - mid) / 1000.0)
        positions, poses = vm._engine_poses(self.eng)
        ppos = self.eng.confocal.probe_pose(probe, positions, poses)
        rec = {"point": [round(px, 6), round(py, 6)], "space": space,
               "probe": probe, "world": [round(ppos[0], 6), round(ppos[1], 6)],
               "reading_um": r["reading_um"], "z_surface": round(z_est, 6),
               "z_focus": r["z_focus"], "searched": searched,
               "align_first": bool(align_note)}
        if space in ("wafer", "wafer_v"):
            # 晶圆刚性固定在吸盘上、随 WS 平移：世界系下不同 WS 轴位测得
            # 的点不构成固定平面（迭代 5 global 复测暴露——global 允许 WS
            # 在测点间运动，世界系拟合出荒谬坡度）。拟合统一改在
            # "ws_ega 基准的晶圆台系"进行：plane_xy = 探头世界 XY −
            # (当前 WS 轴位 − ws_ega 轴位)；平移不改平面坡度，中心面高
            # （=点均值）亦不变。per_site WS 冻结时各点平移量相同，结果
            # 与世界系完全等价。
            mcw = self.eng.constants.get().get("wafer_ega", {})
            wx0, wy0, _ = (float(v) for v in
                           mcw.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
            rec["plane_xy"] = [
                round(ppos[0] - (self.eng.axes["WS_LS_X"].pos - wx0), 6),
                round(ppos[1] - (self.eng.axes["WS_LS_Y"].pos - wy0), 6)]
        self.eng.fl_measurements.setdefault(space, []).append(rec)
        info = ("共焦测高 #%d（%s/%s）：点 (%.3f, %.3f) 世界 (%.3f, %.3f) → "
                "读数 %.2f µm，面高 %.4f mm%s%s"
                % (len(self.eng.fl_measurements[space]), space, probe,
                   px, py, ppos[0], ppos[1], r["reading_um"], z_est,
                   "（AS_Z 搜索 %d 步命中）" % search_steps if searched else "",
                   align_note))
        self._log("info", info)
        return info

    @staticmethod
    def _fit_plane(points):
        """最小二乘拟合平面 z = a·x + b·y + c（≥3 点，纯标准库，复用
        ega._solve3）。points=[(x,y,z)] → {a,b,c,n,rms,centroid}；
        共线/奇异 → None。"""
        n = len(points)
        if n < 3:
            return None
        cxx = cxy = cx = cyy = cy = 0.0
        sxz = syz = sz = 0.0
        for x, y, z in points:
            cxx += x * x
            cxy += x * y
            cx += x
            cyy += y * y
            cy += y
            sxz += x * z
            syz += y * z
            sz += z
        C = [[cxx, cxy, cx], [cxy, cyy, cy], [cx, cy, float(n)]]
        try:
            a, b, c = ega._solve3(C, [sxz, syz, sz])
        except ValueError:
            return None
        sq = [(a * x + b * y + c - z) ** 2 for x, y, z in points]
        return {"a": a, "b": b, "c": c, "n": n,
                "rms": (sum(sq) / n) ** 0.5,
                "centroid": [cx / n, cy / n]}

    def _fl_fit_space(self, space):
        """拟合指定 space 的测高记录并落存 eng.fl_fit[space] -> fit dict。"""
        meas = self.eng.fl_measurements.get(space) or []
        if len(meas) < 3:
            raise RuntimeError("%s 面拟合需要至少 3 个测点，当前 %d 个"
                               % (space, len(meas)))
        # wafer 系空间优先用 plane_xy（晶圆台基准系，见 _do_fl_measure），
        # 其余空间用 world（世界系）。
        pts = [((m.get("plane_xy") or m["world"])[0],
                (m.get("plane_xy") or m["world"])[1],
                m["z_surface"]) for m in meas]
        fit = self._fit_plane(pts)
        if fit is None:
            raise RuntimeError("%s 面拟合失败（测点共线/奇异）" % space)
        self.eng.fl_fit[space] = fit
        return fit

    def _do_fl_solve(self, st):
        """调焦调平解算（2026-08-27）：
        - wafer/die：拟合对应面平面（世界系 z=a·x+b·y+c；a=∂z/∂x、b=∂z/∂y
          即面坡度，小角=倾斜 rad）；
        - compensate：原子补偿（需求 6：进场调一次、调平后调一次，不反复
          动）——①BS_CAL_Rx/Ry 使芯片面∥晶圆面（坡度差一次补齐，钳到
          ±0.1° 行程）；②AS_Z 定下相机焦面=晶圆测点中心面高；③BS_Arm_Z
          定上相机焦面=芯片测点中心面高。残余（urad）与焦面残差写日志，
          结果存 eng.level_memory 供 global 模式直接施加。"""
        mode = st["mode"]
        if mode in ("wafer", "die"):
            fit = self._fl_fit_space(mode)
            info = ("%s 面拟合（n=%d）：坡度 a=%+.2f b=%+.2f µrad，"
                    "中心面高 %.4f mm，rms %.2f nm"
                    % (mode, fit["n"], fit["a"] * 1e6, fit["b"] * 1e6,
                       fit["a"] * fit["centroid"][0]
                       + fit["b"] * fit["centroid"][1] + fit["c"],
                       fit["rms"] * 1e6))
            self._log("info", info)
            return info

        # compensate
        fw = self.eng.fl_fit.get("wafer") or self._fl_fit_space("wafer")
        fd = self.eng.fl_fit.get("die") or self._fl_fit_space("die")
        # ① 调平：芯片面坡度（含当前 cal）一次补到晶圆面坡度。
        #    符号（confocal.die_surface_z 合成约定）：∂z/∂x 方向补
        #    Δcal_ry = a_d − a_w；∂z/∂y 方向补 Δcal_rx = b_w − b_d。
        rx0 = self.eng.axes["BS_CAL_Rx"].pos
        ry0 = self.eng.axes["BS_CAL_Ry"].pos
        crx_t = rx0 + (fw["b"] - fd["b"])
        cry_t = ry0 + (fd["a"] - fw["a"])
        crx_t = max(self.eng.axes["BS_CAL_Rx"].lo,
                    min(self.eng.axes["BS_CAL_Rx"].hi, crx_t))
        cry_t = max(self.eng.axes["BS_CAL_Ry"].lo,
                    min(self.eng.axes["BS_CAL_Ry"].hi, cry_t))
        self._do_move("BS_CAL_Rx", round(crx_t, 9), None,
                      DEFAULT_MOVE_TIMEOUT)
        self._do_move("BS_CAL_Ry", round(cry_t, 9), None,
                      DEFAULT_MOVE_TIMEOUT)
        # 补偿后预期残余坡度（钳位截断才非零）
        res_rx = (fd["b"] + (crx_t - rx0)) - fw["b"]
        res_ry = (fd["a"] - (cry_t - ry0)) - fw["a"]
        # ② AS_Z 定焦：下相机焦面 = 晶圆测点中心面高（焦面随 AS_Z 1:1，
        #    用共焦模型的焦面公式取值，对模块 z 误差免疫）
        z0 = self.eng.axes["AS_Z"].pos
        zw_c = fw["a"] * fw["centroid"][0] + fw["b"] * fw["centroid"][1] \
            + fw["c"]
        zf_down = self.eng.confocal.focus_plane_z("down")
        as_t = z0 + (zw_c - zf_down)
        as_t = max(self.eng.axes["AS_Z"].lo,
                   min(self.eng.axes["AS_Z"].hi, as_t))
        # ③ BS_Arm_Z 定焦：上相机焦面（随新 AS_Z）= 芯片测点中心面高；
        #    芯片面随 BS_Arm_Z 1:1 平移 → 增量式计算对常数误差不敏感
        zf_up_new = self.eng.confocal.focus_plane_z("up") + (as_t - z0)
        zd_c = fd["a"] * fd["centroid"][0] + fd["b"] * fd["centroid"][1] \
            + fd["c"]
        bs0 = self.eng.axes["BS_Arm_Z"].pos
        bs_t = bs0 + (zf_up_new - zd_c)
        bs_t = max(self.eng.axes["BS_Arm_Z"].lo,
                   min(self.eng.axes["BS_Arm_Z"].hi, bs_t))
        self._do_move("AS_Z", round(as_t, 6), None, DEFAULT_MOVE_TIMEOUT)
        self._do_move("BS_Arm_Z", round(bs_t, 6), None, DEFAULT_MOVE_TIMEOUT)
        # 焦面残差（钳位截断才非零）
        foc_res_down = zw_c - (zf_down + (as_t - z0))
        foc_res_up = (zf_up_new - zd_c) - (bs_t - bs0)
        with self.eng.lock:
            t_sim = self.eng.sim_time
        self.eng.level_memory = {
            "cal_rx": crx_t, "cal_ry": cry_t,
            "as_z": as_t, "bs_arm_z": bs_t,
            "residual_urad": [res_rx * 1e6, res_ry * 1e6],
            "t": round(t_sim, 3)}
        self.eng.fl_fit["compensate"] = {
            "cal_rx": crx_t, "cal_ry": cry_t,
            "cal_delta_urad": [(crx_t - rx0) * 1e6, (cry_t - ry0) * 1e6],
            "residual_urad": [res_rx * 1e6, res_ry * 1e6],
            "as_z": as_t, "bs_arm_z": bs_t,
            "focus_resid_um": [foc_res_down * 1000, foc_res_up * 1000],
            "wafer_grad_urad": [fw["a"] * 1e6, fw["b"] * 1e6],
            "die_grad_urad": [fd["a"] * 1e6, fd["b"] * 1e6]}
        info = ("调平补偿完成：晶圆面坡度 (%+.2f, %+.2f) µrad，芯片面 "
                "(%+.2f, %+.2f) µrad → BS_CAL_Rx %+.2f→%+.2f、Ry "
                "%+.2f→%+.2f µrad，预期残余 (%.2f, %.2f) µrad；AS_Z "
                "%.4f→%.4f（下焦面=晶圆中心面 %.4f），BS_Arm_Z %.4f→%.4f"
                "（上焦面=芯片中心面 %.4f），焦面残差 (%.3f, %.3f) µm"
                % (fw["a"] * 1e6, fw["b"] * 1e6, fd["a"] * 1e6,
                   fd["b"] * 1e6, rx0 * 1e6, crx_t * 1e6, ry0 * 1e6,
                   cry_t * 1e6, res_rx * 1e6, res_ry * 1e6,
                   z0, as_t, zw_c, bs0, bs_t, zd_c,
                   foc_res_down * 1000, foc_res_up * 1000))
        self._log("info", info)
        return info

    def _do_fl_verify(self):
        """调平复测验证（迭代 5，2026-08-28）：拟合复测点（wafer_v/die_v，
        补偿已真实施加在 BS_CAL_Rx/Ry 上，die 面物理随动，复测读数直接
        反映补偿后状态）→ 实测残余坡度 = die_v 坡度 − wafer_v 坡度（µrad）
        → 与 focus_level.level_tol_urad（30µrad）判定：
        PASS/FAIL 都落存 eng.fl_fit["verify"]（residual_urad /
        wafer_grad_urad / die_grad_urad / judge），FAIL 抛错使流程 abort
        报警（与真实设备一致；自动迭代补偿属后续迭代）。
        同时复核焦面（参考值，不判定）：复测点中心面高 vs 当前上下焦面。"""
        fl = self.eng.constants.get()["focus_level"]
        tol = float(fl["level_tol_urad"])
        fw = self._fl_fit_space("wafer_v")
        fd = self._fl_fit_space("die_v")
        res_rx = (fd["b"] - fw["b"]) * 1e6
        res_ry = (fd["a"] - fw["a"]) * 1e6
        judge = "pass" if abs(res_rx) <= tol and abs(res_ry) <= tol else "fail"
        self.eng.fl_fit["verify"] = {
            "residual_urad": [res_rx, res_ry],
            "wafer_grad_urad": [fw["a"] * 1e6, fw["b"] * 1e6],
            "die_grad_urad": [fd["a"] * 1e6, fd["b"] * 1e6],
            "judge": judge}
        # 焦面复核（参考值，不判定）：复测点中心面高 vs 当前焦面
        zw_c = fw["a"] * fw["centroid"][0] + fw["b"] * fw["centroid"][1] \
            + fw["c"]
        zd_c = fd["a"] * fd["centroid"][0] + fd["b"] * fd["centroid"][1] \
            + fd["c"]
        dz_down = (zw_c - self.eng.confocal.focus_plane_z("down")) * 1000.0
        dz_up = (zd_c - self.eng.confocal.focus_plane_z("up")) * 1000.0
        info = ("调平复测 %s：残余 (%+.2f, %+.2f) µrad %s 阈值 %.0f µrad"
                "（晶圆面坡度 (%+.2f, %+.2f)，芯片面 (%+.2f, %+.2f) µrad；"
                "焦面复核：晶圆中心面高偏差 %+.3f µm（下焦面）、"
                "芯片中心面高偏差 %+.3f µm（上焦面））"
                % ("PASS" if judge == "pass" else "FAIL",
                   res_rx, res_ry, "≤" if judge == "pass" else ">", tol,
                   fw["a"] * 1e6, fw["b"] * 1e6, fd["a"] * 1e6, fd["b"] * 1e6,
                   dz_down, dz_up))
        self._log("info", info)
        if judge == "fail":
            raise RuntimeError(info)
        return info

    # ---------------- 逐键合位精对准（fa_*；迭代 6） ----------------
    # 锁轴快照：测量期间只允许动对准台 AS（用户需求：测量期 WS 全冻结）
    _FA_LOCK_AXES = ("WS_LS_X", "WS_LS_Y", "WS_Rz",
                     "BS_Arm_Z", "BS_CAL_Rx", "BS_CAL_Ry")

    def _fa_lock_snapshot(self):
        return {n: self.eng.axes[n].pos for n in self._FA_LOCK_AXES}

    def _fa_lock_check(self, snap):
        """测量前后比对锁轴快照：任何被锁轴移动 → 抛错（真实设备的
        "测量期间轴被扰动"报警）。"""
        moved = ["%s %.6f→%.6f" % (n, snap[n], self.eng.axes[n].pos)
                 for n in snap
                 if abs(self.eng.axes[n].pos - snap[n]) > 1e-6]
        if moved:
            raise RuntimeError("精对准测量期间锁轴被扰动（WS/BS 须冻结）："
                               + "；".join(moved))

    def _fa_measure_both(self, st):
        """双侧逐标记测量（fa_measure 与确认环复测共用）：
        晶圆侧 N 点（down 相机）：设计点=site 心+wafer_pts 偏移，寻址=
        EGA map + WS 冻结补偿（AS=anchor+m+(WS−ws_ega)，仅动 AS）；
        die 侧 N 点（up 相机）：设计点=die_pts 偏移，寻址=机械常数
        as_head_center_xy_mm+偏移（同 fl_measure die 空间认知链）。
        每点 identify → solve_pixel → 实测点换算进"头心相对公共帧"
        （晶圆侧减 m_site，die 侧相对 as_head 天然同帧）。
        返回测量 dict；identify 失败/丢标 → 抛错 abort（同 ega_measure）。"""
        if self.eng.vision is None:
            raise RuntimeError("视觉系统未挂载（fa_measure 需要 vision）")
        import vision_model as vm
        fit = getattr(self.eng, "ega_result", None)
        if fit is None:
            raise RuntimeError("fa_measure 需要 EGA 结果（先跑 wafer_ega）")
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        sx, sy = float(st["site_x"]), float(st["site_y"])
        wafer_pts = [[float(p[0]), float(p[1])] for p in st["wafer_pts"]]
        die_pts = [[float(p[0]), float(p[1])] for p in st["die_pts"]]
        mc = self.eng.constants.get()
        we = mc.get("wafer_ega", {})
        ax, ay = (float(v) for v in we.get("as_anchor_xy_mm", [0.0, 0.0]))
        wx0, wy0, _ = (float(v) for v in
                       we.get("ws_ega_xyz_mm", [0.0, 0.0, 0.0]))
        as_head = [float(v) for v in
                   mc["focus_level"]["as_head_center_xy_mm"]]
        m_site = ega.apply_map(fit, sx, sy)
        snap = self._fa_lock_snapshot()
        # 公共帧原点 = 键合头吸盘心（世界）。晶圆侧实测点换算到头心相对
        # 世界帧：W−H = m_meas + (WS−ws_ega) − (as_head−anchor)
        # （推导见 docs/逐键合位精对准_V1.md；m 帧随吸盘平移，只减 m_site
        # 会把平移补偿"测不见"——初测时 fl_ready 关系使两者等价，补偿后
        # WS 变了就必须带 (WS−ws_ega)−(as_head−anchor) 项）。
        wsx0, wsy0 = self.eng.axes["WS_LS_X"].pos, \
            self.eng.axes["WS_LS_Y"].pos
        base = [(wsx0 - wx0) - (as_head[0] - ax),
                (wsy0 - wy0) - (as_head[1] - ay)]
        wafer_meas, die_meas = [], []
        for k in range(len(wafer_pts)):
            # ---- 晶圆侧：对准系寻址（WS 冻结，仅 AS 承担位移）----
            ox, oy = wafer_pts[k]
            mx, my = ega.apply_map(fit, sx + ox, sy + oy)
            as_x_t = ax + mx + (wsx0 - wx0)
            as_y_t = ay + my + (wsy0 - wy0)
            for name, v in (("AS_X", as_x_t), ("AS_Y", as_y_t)):
                axo = self.eng.axes[name]
                if not (axo.lo <= v <= axo.hi):
                    raise RuntimeError(
                        "fa_measure 晶圆侧标记 #%d 目标使 %s=%.3f 超出行程"
                        "[%.2f,%.2f]" % (k, name, v, axo.lo, axo.hi))
            self._do_move("AS_X", as_x_t, None, timeout)
            self._do_move("AS_Y", as_y_t, None, timeout)
            wm = self._fa_identify_point(
                "down", template=self._vision_tpl("tpl_wafer_site"))
            # 实测点 → 头心相对公共帧
            wafer_meas.append([
                round(mx + wm[0] + base[0], 6),
                round(my + wm[1] + base[1], 6)])
            # ---- die 侧：as_head_center + 角标设计偏移（仅动 AS）----
            dx, dy = die_pts[k]
            self._do_move("AS_X", as_head[0] + dx, None, timeout)
            self._do_move("AS_Y", as_head[1] + dy, None, timeout)
            dm = self._fa_identify_point(
                "up", template=self._vision_tpl("tpl_die_head"))
            die_meas.append([round(dx + dm[0], 6), round(dy + dm[1], 6)])
        self._fa_lock_check(snap)
        return {"site": [round(sx, 6), round(sy, 6)],
                "m_site": [round(m_site[0], 6), round(m_site[1], 6)],
                "wafer_design": wafer_pts, "die_design": die_pts,
                "wafer_meas": wafer_meas, "die_meas": die_meas}

    def _fa_identify_point(self, cam_id, template=None):
        """单标记拍照 → 实测相对相机光轴的世界偏移 (dx, dy)（复用
        _do_ega_measure 的 identify→solve_pixel→减相机位姿路径）。
        template=本次识别依据的模板名（机械常数 vision_marks，迭代 12
        起必传）。"""
        import vision_model as vm
        entry = self._vision_identify(cam_id, template=template)
        if entry.get("error"):
            raise RuntimeError("精对准 %s 相机拍照失败：%s"
                               % (cam_id, entry["error"]))
        res = entry.get("result") or {}
        cols = res.get("lstCol") or []
        rows = res.get("lstRow") or []
        if not cols or not res.get("judge"):
            raise RuntimeError("精对准测量失败：%s 相机视野内无标记"
                               "（标记未进焦或缺失）" % cam_id)
        rt = self.eng.vision.cams.get(cam_id)
        if rt is None:
            raise RuntimeError("未知相机 %r" % cam_id)
        positions, poses = vm._engine_poses(self.eng)
        cam_pos, _yaw = vm.camera_pose(self.eng, rt.model, poses, positions)
        wx, wy, _wz = rt.model.solve_pixel(cols[0], rows[0], cam_pos)
        return wx - cam_pos[0], wy - cam_pos[1]

    def _fa_recipe_cfg(self):
        """处方 fine_align 组（缺省播种兜底，recipe.validate 已保证齐全）。"""
        fa = (self.eng.recipe.get() or {}).get("fine_align") or {}
        return {"algorithm": fa.get("algorithm", "rigid"),
                "confirm_enable": bool(fa.get("confirm_enable", True)),
                "confirm_tol_xy_um": float(fa.get("confirm_tol_xy_um", 0.15)),
                "confirm_tol_rz_urad": float(fa.get("confirm_tol_rz_urad",
                                                    30.0)),
                "confirm_max_iter": int(fa.get("confirm_max_iter", 3)),
                "on_unconfirmed": fa.get("on_unconfirmed", "abort")}

    def _fa_solve_from(self, meas, algorithm):
        """双侧 fit → Δ=T_d∘T_w⁻¹（刚体参数）。algorithm 非法/n=2+affine
        由 fine_align.fit 抛 ValueError。

        计量帧标定残差（2026-09-07 迭代 20，机械常数 fine_align.
        frame_cal_x/y_um、frame_cal_rz_urad，默认 0）：上/下相机两条
        测量链的帧间偏差经出厂标定后剩余的常数差，控制器按标定值修正
        测量 Δ（delta_used = delta_meas + frame_cal）；0=标定理想。
        本机两测量侧实际几乎共用同一 AS 位姿（WS 承担大行程，AS 只做
        微调），帧间差 ≈ AS 地图在毫米级跨度上的差值（亚 µm），标定
        后残差常数取 0 即可；注入非零值可演示标定残余（直接平移到
        truth）。确认环每轮重解同口径，闭环把残余驱动到 frame_cal。"""
        fit_w = fine_align.fit(algorithm, meas["wafer_design"],
                               meas["wafer_meas"])
        fit_d = fine_align.fit(algorithm, meas["die_design"],
                               meas["die_meas"])
        delta = fine_align.compose_rigid(fit_d, fit_w)
        fa_mc = self.eng.constants.get().get("fine_align", {})
        cx = float(fa_mc.get("frame_cal_x_um", 0.0)) / 1000.0
        cy = float(fa_mc.get("frame_cal_y_um", 0.0)) / 1000.0
        crz = float(fa_mc.get("frame_cal_rz_urad", 0.0)) / 1e6
        if cx or cy or crz:
            delta = {"tx": delta["tx"] + cx, "ty": delta["ty"] + cy,
                     "theta": delta["theta"] + crz}
        return {"algorithm": algorithm, "delta": delta,
                "fit_w": fit_w, "fit_d": fit_d}

    def _do_fa_measure(self, st):
        """精对准测量步：锁轴快照 → 双侧各 N 点 AS-only 逐标记测量 →
        eng.fa_meas。结果供 fa_solve 拟合。"""
        meas = self._fa_measure_both(st)
        with self.eng.lock:
            t_sim = self.eng.sim_time
        meas["t"] = round(t_sim, 3)
        self.eng.fa_meas = meas
        # 诊断（FA_DEBUG=1 时）：初测实测 Δ vs 真值 Δ 对照（定位测量链/
        # 真值链偏差用，正常运行零开销）
        import os as _os
        if _os.environ.get("FA_DEBUG"):
            try:
                td = self._fa_truth_resid(st, self.eng.recipe.get())
                md = self._fa_solve_from(meas, "rigid")["delta"]
                self._log("info", "FA_DEBUG 初测：实测 θ=%+.2f µrad "
                          "t=(%+.2f,%+.2f) µm；真值 θ=%+.2f µrad "
                          "t=(%+.2f,%+.2f) µm"
                          % (md["theta"] * 1e6, md["tx"] * 1000,
                             md["ty"] * 1000, td["theta"] * 1e6,
                             td["tx"] * 1000, td["ty"] * 1000))
            except Exception as _e:
                self._log("warn", "FA_DEBUG 异常：%s" % _e)
        info = ("精对准测量完成：site(%.3f,%.3f) 双侧各 %d 点（晶圆侧首点 "
                "(%+.4f,%+.4f)，die 侧首点 (%+.4f,%+.4f) mm，锁轴校验通过）"
                % (meas["site"][0], meas["site"][1], len(meas["wafer_meas"]),
                   meas["wafer_meas"][0][0], meas["wafer_meas"][0][1],
                   meas["die_meas"][0][0], meas["die_meas"][0][1]))
        self._log("info", info)
        return info

    def _do_fa_solve(self):
        """精对准解算：双侧 fit（算法=处方 fine_align.algorithm）→
        Δ=(tx,ty,θ) → eng.fa_delta。"""
        meas = getattr(self.eng, "fa_meas", None)
        if meas is None:
            raise RuntimeError("fa_solve 需要 fa_measure 测量（先跑 "
                               "fa_measure）")
        cfg = self._fa_recipe_cfg()
        n = len(meas["wafer_design"])
        if cfg["algorithm"] == "affine" and n < 3:
            raise RuntimeError("affine 拟合需要 ≥3 点（当前 n=%d，请加大 "
                               "fine_align.measure_points 或换算法）" % n)
        res = self._fa_solve_from(meas, cfg["algorithm"])
        if cfg["algorithm"] == "midpoint_2mark" and n > 2:
            self._log("info", "midpoint_2mark 只用双侧各自前 2 个标记"
                              "（共 %d 点，余者仅计入残差统计）" % n)
        self.eng.fa_delta = res
        d = res["delta"]
        info = ("精对准解算（%s，n=%d）：Δ tx=%+.3f µm ty=%+.3f µm "
                "θ=%+.3f µrad（晶圆侧 rms=%.3f µm，die 侧 rms=%.3f µm）"
                % (res["algorithm"], n, d["tx"] * 1000, d["ty"] * 1000,
                   d["theta"] * 1e6, res["fit_w"]["rms"] * 1000,
                   res["fit_d"]["rms"] * 1000))
        self._log("info", info)
        return info

    def _fa_apply_comp(self, delta, st, tag=""):
        """执行一次补偿（先旋后移，旋转耦合进平移；含 EGA map 同步更新）：
        ① θ：WS_Rz 目标=当前+δθ·rz_sign/rz_gain（±1° 行程守卫）；
        ② 枢轴（信念）c=吸盘心世界 XY+机械常数 rz_center_offset_mm，
           耦合 coup=R(δθ)(p−c)−(p−c)，p=site 心世界（信念=K+m_site）；
        ③ 平移：WS_XY += t−coup；
        ④ map 更新：对准系 m=W−K 相对吸盘心，刚体旋转下 m'=R(δθ)·m
           （平移/枢轴在 m 帧相消，推导见 docs/逐键合位精对准_V1.md），
           复合进 eng.ega_result 的 a/tx/ty，保证后续 site 寻址不失效。
        返回补偿记录 dict。"""
        import vision_model as vm
        fit = getattr(self.eng, "ega_result", None)
        if fit is None:
            raise RuntimeError("补偿需要 EGA 结果（先跑 wafer_ega）")
        fa_mc = self.eng.constants.get().get("fine_align", {})
        rz_gain = float(fa_mc.get("rz_gain", 1.0))
        rz_sign = float(fa_mc.get("rz_sign", 1.0))
        rz_off = [float(v) for v in fa_mc.get("rz_center_offset_mm",
                                              [0.0, 0.0])]
        dtheta = float(delta["theta"])
        tx, ty = float(delta["tx"]), float(delta["ty"])
        positions, poses = vm._engine_poses(self.eng)
        kc, _rot = vm._module_pose(poses, "wafer_stage")
        cx, cy = kc[0] + rz_off[0], kc[1] + rz_off[1]
        m_site = ega.apply_map(fit, float(st["site_x"]), float(st["site_y"]))
        px, py = kc[0] + m_site[0], kc[1] + m_site[1]
        # ① 旋转（行程守卫：WS_Rz ±1°）
        rz_ax = self.eng.axes["WS_Rz"]
        rz_tgt = rz_ax.pos + dtheta * rz_sign / rz_gain
        if not (rz_ax.lo - 1e-12 <= rz_tgt <= rz_ax.hi + 1e-12):
            raise RuntimeError(
                "精对准旋转补偿 θ=%+.2f µrad 使 WS_Rz 目标 %.6f rad 超出"
                "行程 [%.6f, %.6f]（±1°）——偏差过大，请检查上片/交接"
                % (dtheta * 1e6, rz_tgt, rz_ax.lo, rz_ax.hi))
        # ② 耦合：绕 c 旋转 δθ 使参考点 p 产生的位移
        c_, s_ = math.cos(dtheta), math.sin(dtheta)
        vx, vy = px - cx, py - cy
        coup = (c_ * vx - s_ * vy - vx, s_ * vx + c_ * vy - vy)
        # ③ 平移（先旋后移：旋转到位后再补 XY）
        ux, uy = tx - coup[0], ty - coup[1]
        timeout = float(st.get("timeout", DEFAULT_MOVE_TIMEOUT))
        if abs(dtheta) > 1e-12:
            self._do_move("WS_Rz", round(rz_tgt, 9), None, timeout)
        else:
            self._log("info", "精对准补偿：θ 残余 <1nrad，跳过 WS_Rz")
        if math.hypot(ux, uy) > 1e-9:
            self._do_move("WS_LS_X",
                          round(self.eng.axes["WS_LS_X"].pos + ux, 6),
                          None, timeout)
            self._do_move("WS_LS_Y",
                          round(self.eng.axes["WS_LS_Y"].pos + uy, 6),
                          None, timeout)
        else:
            self._log("info", "精对准补偿：平移残余 <1nm，跳过 WS_XY")
        # ④ map 更新：m' = R(δθ)·m（A、b 同旋）
        a11, a12, a21, a22 = fit["a"]
        fit["a"] = [c_ * a11 - s_ * a21, c_ * a12 - s_ * a22,
                    s_ * a11 + c_ * a21, s_ * a12 + c_ * a22]
        btx, bty = fit["tx"], fit["ty"]
        fit["tx"], fit["ty"] = (c_ * btx - s_ * bty,
                                s_ * btx + c_ * bty)
        fit["theta"] = 0.5 * (fit["a"][2] - fit["a"][1])
        rec = {"dtheta_urad": dtheta * 1e6,
               "t_um": [tx * 1000, ty * 1000],
               "coup_um": [coup[0] * 1000, coup[1] * 1000],
               "u_um": [ux * 1000, uy * 1000],
               "pivot_mm": [round(cx, 6), round(cy, 6)],
               "ws_rz": round(self.eng.axes["WS_Rz"].pos, 9)}
        fit.setdefault("fine_align_comp", []).append(rec)
        # WS 已动：fl_ws_frozen 同步刷新（后续 fl_measure freeze_ws 基准）
        if getattr(self.eng, "fl_ws_frozen", None):
            self.eng.fl_ws_frozen["WS_LS_X"] = self.eng.axes["WS_LS_X"].pos
            self.eng.fl_ws_frozen["WS_LS_Y"] = self.eng.axes["WS_LS_Y"].pos
        self._log("info",
                  "精对准补偿%s：δθ=%+.3f µrad（WS_Rz→%.6f），平移 t="
                  "(%+.3f,%+.3f) µm − 耦合 coup=(%+.3f,%+.3f) µm → WS_XY "
                  "+(%+.3f,%+.3f) µm（枢轴 (%.4f,%.4f)，EGA map 已同步）"
                  % (tag, dtheta * 1e6, self.eng.axes["WS_Rz"].pos,
                     tx * 1000, ty * 1000, coup[0] * 1000, coup[1] * 1000,
                     ux * 1000, uy * 1000, cx, cy))
        return rec

    def _fa_truth_resid(self, st, recipe):
        """物理残差（真值通道，不混含噪测量）：对真值标记世界坐标走同一
        条 fit 管线（design→truth 双侧 rigid → 组间相对 Δ），返回残余
        (tx,ty,θ)。真值来源：kinematics 6DOF 模块位姿 + workpieces 上片/
        交接真值（与 collect_marks 同一套数学，绕真值枢轴）。

        组间相对 Δ 用 fine_align.relative_delta（2026-09-07 迭代 20）：
        真值点云在世界系（距帧原点 ~43mm），compose_rigid 的
        t_d−R_Δ·t_w 形会引入 θ_Δ×J(F) 参考原点耦合假平移；relative_delta
        = 两侧设计原点像之差（参考点无关）。测量通道（fa_solve，点云在
        头心相对近原帧）仍用 compose_rigid，不动闭环。"""
        import vision_model as vm
        positions, poses = vm._engine_poses(self.eng)
        wp = self.eng.workpieces
        center, rot = vm._module_pose(poses, "wafer_stage")
        rz = positions.get("WS_Rz", 0.0)
        rot = [rot[0], rot[1], rot[2] + rz]
        pivot = vm.wafer_rz_pivot(recipe)
        with wp._lock:
            off = list(wp.wafer["offset"])
            wrz = float(wp.wafer.get("rz", 0.0))
            heads = [dict(d) for d in wp.dies if d["carrier"] == "head"]
        cw, sw = math.cos(wrz), math.sin(wrz)
        sx, sy = float(st["site_x"]), float(st["site_y"])
        wafer_truth = []
        for ox, oy in st["wafer_pts"]:
            lx = off[0] + cw * (sx + ox) - sw * (sy + oy)
            ly = off[1] + sw * (sx + ox) + cw * (sy + oy)
            w = vm._rotvec_apply_pivot([lx, ly, 0.0], rot, pivot)
            wafer_truth.append((center[0] + w[0], center[1] + w[1]))
        die_truth = []
        if heads:
            d = heads[0]
            hc, hrot = vm._module_pose(poses, "bond_head")
            hrot = [hrot[0] + positions.get("BS_CAL_Rx", 0.0),
                    hrot[1] + positions.get("BS_CAL_Ry", 0.0), hrot[2]]
            dsite = d["site"] or [0.0, 0.0]
            for dx, dy in st["die_pts"]:
                w = vm._rotvec_apply([dsite[0] + dx, dsite[1] + dy, 0.0],
                                     hrot)
                die_truth.append((hc[0] + w[0], hc[1] + w[1]))
        if not die_truth:
            return None
        fit_w = fine_align.fit_rigid(st["wafer_pts"], wafer_truth)
        fit_d = fine_align.fit_rigid(st["die_pts"], die_truth)
        return fine_align.relative_delta(fit_d, fit_w)

    def _do_fa_compensate_confirm(self, st):
        """补偿+确认闭环（迭代 6 需求：测-补-复测-判定，逐轮日志）：
        用 eng.fa_delta（缺省时从 fa_meas 现解）执行一次补偿 → 确认环：
        每轮全标记复测（AS-only+锁轴快照）→ 重 fit → 残余 vs
        confirm_tol_xy_um/confirm_tol_rz_urad；未过且未超
        confirm_max_iter → 残差再补一轮；超次按 on_unconfirmed
        （abort=抛错流程中止报警 / continue_warn=警告放行 judge=warn）。
        最终结果（含物理残差双通道、轮次历史、judge）存
        eng.fine_align_result。"""
        recipe = self.eng.recipe.get()
        cfg = self._fa_recipe_cfg()
        algo = cfg["algorithm"]
        res = getattr(self.eng, "fa_delta", None)
        meas = getattr(self.eng, "fa_meas", None)
        if res is None:
            if meas is None:
                raise RuntimeError("补偿确认需要 fa_measure/fa_solve 先行")
            res = self._fa_solve_from(meas, algo)
        comps = [self._fa_apply_comp(res["delta"], st)]
        rounds = []
        judge = "pass"
        if cfg["confirm_enable"]:
            tol_xy = cfg["confirm_tol_xy_um"]
            tol_rz = cfg["confirm_tol_rz_urad"]
            for it in range(1, cfg["confirm_max_iter"] + 1):
                meas2 = self._fa_measure_both(st)
                import os as _os
                if _os.environ.get("FA_DEBUG"):
                    for _k in range(len(meas2["wafer_meas"])):
                        self._log("info",
                            "FA_DEBUG 轮%d #%d 晶圆实测(%+.4f,%+.4f) die实测(%+.4f,%+.4f) um"
                            % (it, _k,
                               meas2["wafer_meas"][_k][0] * 1000,
                               meas2["wafer_meas"][_k][1] * 1000,
                               meas2["die_meas"][_k][0] * 1000,
                               meas2["die_meas"][_k][1] * 1000))
                    try:
                        _td2 = self._fa_truth_resid(st, recipe)
                        self._log("info",
                            "FA_DEBUG 轮%d 真值 Δ：θ=%+.2f urad t=(%+.3f,%+.3f) um"
                            % (it, _td2["theta"] * 1e6, _td2["tx"] * 1000,
                               _td2["ty"] * 1000))
                    except Exception as _e2:
                        self._log("warn", "FA_DEBUG 轮%d 真值异常：%s"
                                  % (it, _e2))
                r2 = self._fa_solve_from(meas2, algo)
                d = r2["delta"]
                res_xy = math.hypot(d["tx"], d["ty"]) * 1000.0
                res_rz = abs(d["theta"]) * 1e6
                ok = res_xy <= tol_xy and res_rz <= tol_rz
                rounds.append({"iter": it,
                               "delta": {"tx_um": round(d["tx"] * 1000, 4),
                                         "ty_um": round(d["ty"] * 1000, 4),
                                         "theta_urad": round(d["theta"] * 1e6,
                                                             3)},
                               "theta_w_urad": round(
                                   r2["fit_w"]["theta"] * 1e6, 3),
                               "theta_d_urad": round(
                                   r2["fit_d"]["theta"] * 1e6, 3),
                               "resid_xy_um": round(res_xy, 4),
                               "resid_rz_urad": round(res_rz, 3),
                               "judge": "pass" if ok else "fail"})
                self._log("info",
                          "精对准确认 第%d轮：复测残余 xy=%.3f µm、rz=%.3f "
                          "µrad（Δ tx=%+.3f ty=%+.3f µm θ=%+.3f µrad；"
                          "阈值 %.2f µm / %.0f µrad）→ %s"
                          % (it, res_xy, res_rz, d["tx"] * 1000,
                             d["ty"] * 1000, d["theta"] * 1e6,
                             tol_xy, tol_rz,
                             "PASS" if ok else "FAIL"))
                if ok:
                    break
                if it < cfg["confirm_max_iter"]:
                    comps.append(self._fa_apply_comp(
                        d, st, tag="（确认第%d轮再补）" % it))
                    continue
                # 超次未收敛
                judge = "fail"
                if cfg["on_unconfirmed"] == "continue_warn":
                    judge = "warn"
                    self._log("warn",
                              "精对准确认 %d 轮未收敛（on_unconfirmed="
                              "continue_warn）：放行，残余 xy=%.3f µm、"
                              "rz=%.3f µrad" % (it, res_xy, res_rz))
                else:
                    self.eng.fine_align_result = self._fa_result(
                        st, res, comps, rounds, "fail", recipe)
                    raise RuntimeError(
                        "精对准确认 %d 轮未收敛（on_unconfirmed=abort）："
                        "末轮残余 xy=%.3f µm、rz=%.3f µrad（阈值 %.2f µm /"
                        " %.0f µrad）" % (it, res_xy, res_rz, tol_xy, tol_rz))
        phys = self._fa_truth_resid(st, recipe)
        self.eng.fine_align_result = self._fa_result(
            st, res, comps, rounds, judge, recipe, phys=phys)
        d0 = res["delta"]
        info = ("精对准%s：Δ tx=%+.3f µm ty=%+.3f µm θ=%+.3f µrad，补偿 %d "
                "次，确认 %d 轮 judge=%s；物理残差 %s"
                % ("完成" if judge == "pass" else "未完全收敛",
                   d0["tx"] * 1000, d0["ty"] * 1000, d0["theta"] * 1e6,
                   len(comps), len(rounds), judge,
                   ("tx=%+.3f µm ty=%+.3f µm θ=%+.3f µrad"
                    % (phys["tx"] * 1000, phys["ty"] * 1000,
                       phys["theta"] * 1e6)) if phys else "（不可用）"))
        self._log("info" if judge == "pass" else "warn", info)
        return info

    def _fa_result(self, st, res, comps, rounds, judge, recipe, phys=None):
        d = res["delta"]
        with self.eng.lock:
            t_sim = self.eng.sim_time
        return {"site": [float(st["site_x"]), float(st["site_y"])],
                "algorithm": res["algorithm"],
                "delta": {"tx_um": d["tx"] * 1000, "ty_um": d["ty"] * 1000,
                          "theta_urad": d["theta"] * 1e6},
                "compensations": comps,
                "rounds": rounds,
                "judge": judge,
                "phys_resid": ({
                    "tx_um": phys["tx"] * 1000, "ty_um": phys["ty"] * 1000,
                    "theta_urad": phys["theta"] * 1e6} if phys else None),
                "t": round(t_sim, 3)}

    def _io_eq(self, path, expect):
        v = self.points.get(path)
        if isinstance(expect, bool):
            return v is not None and bool(v) == expect
        if isinstance(expect, (int, float)):
            try:
                return v is not None and abs(float(v) - float(expect)) < 1e-9
            except (TypeError, ValueError):
                return False
        return v == expect

    def _do_move(self, axis, target, speed, timeout):
        names = ac.resolve_axis(axis)
        if not names:
            raise KeyError("unknown axis: %r" % axis)
        dur = self.eng.move(axis, target, speed)   # 联锁/掉使能在此抛 409 族异常

        def settled():
            with self.eng.lock:
                for n in names:
                    if self.eng.axes[n].fault:
                        raise RuntimeError(
                            "轴 %s fault（运动中掉使能/故障）" % n)
                return all(self.eng.axes[n].in_position
                           and not self.eng.axes[n].moving for n in names)
        self._wait(settled, timeout, " %s 到位" % "/".join(names))
        return "%s → %s 到位（规划 %.2fs）" % ("/".join(names), target, dur)

    def _do_home(self, axis, timeout):
        self.eng.home(axis)
        names = list(self.eng.axes) if axis == "ALL" else ac.resolve_axis(axis)
        if any(n in ac.HOME_TYPE_PLC_SYNC_TRIO for n in names):
            names = sorted(set(names) | ac.HOME_TYPE_PLC_SYNC_TRIO)

        def homed():
            with self.eng.lock:
                for n in names:
                    hs = self.eng.axes[n].home_state
                    if hs == "fault":
                        raise RuntimeError("轴 %s 回零 fault：%s"
                                           % (n, self.eng.axes[n].home_err))
                return all(self.eng.axes[n].home_state == "done" for n in names)
        self._wait(homed, timeout, " %s 回零" % axis)
        return "%s 回零完成" % axis

    def _do_io(self, path, cmd, args):
        if self.eng.is_silent("plc"):
            # PLC 沉默（故障注入）：与 9010 代理同语义——指令被吞，记进 info
            return "PLC 沉默中：指令被吞未执行（%s %s）" % (path, cmd)
        r = plc_command(self.eng, self.points, path, cmd, args)
        if not r.get("ok"):
            raise RuntimeError("io 命令失败：%s" % r)
        return "%s %s ok" % (path.rsplit(".", 1)[-1], cmd)

    def _do_delay(self, ms):
        end = time.time() + ms / 1000.0
        while time.time() < end:
            with self._ctl:
                if self._stop_req:
                    raise _FlowAbort("手动停止（delay 中断）")
            time.sleep(min(0.05, max(0.0, end - time.time())))
        return "延时 %.0fms" % ms

    def _do_if(self, st, depth):
        cond = st["cond"]
        if "grip" in cond:
            hit = self.eng.air_grip_ok(cond["grip"]) == bool(cond["expect"])
            cdesc = "grip %s==%s" % (cond["grip"], cond["expect"])
        else:
            hit = self._io_eq(cond["io"], cond["expect"])
            cdesc = "io %s==%r" % (cond["io"], cond["expect"])
        branch = st.get("then") if hit else st.get("else")
        if not branch:
            return "条件 %s → %s（无分支，跳过）" % (cdesc, hit)
        for sub in branch:
            try:
                self._do_step(sub, depth + 1)
            except _FlowAbort:
                raise
            except Exception as e:
                raise RuntimeError("%s 分支子步骤（%s）失败：%s"
                                   % ("then" if hit else "else",
                                      sub.get("op"), e))
        return "条件 %s → %s 分支 %d 步全 ok" % (cdesc,
                                               "then" if hit else "else",
                                               len(branch))

    def _do_wafer(self, action, offset, rz=None, z=None, rx=None, ry=None,
                  inject=False):
        wp = self.eng.workpieces
        if wp is None:
            raise RuntimeError("workpieces 未挂载")
        if action == "load":
            with wp._lock:
                if wp.wafer["present"]:
                    return "晶圆已在位（跳过 load）"
            kwargs = {}
            if inject:                      # 按处方 wafer_load 注入（含 jitter）
                from recipe import wafer_load_inject_params
                kwargs.update(wafer_load_inject_params(
                    self.eng.recipe.get()["wafer_load"]))
            if offset is not None:          # 显式字段覆盖注入默认
                kwargs["offset"] = offset
            if rz is not None:
                kwargs["rz"] = float(rz)
            for k, v in (("z", z), ("rx", rx), ("ry", ry)):
                if v is not None:
                    kwargs[k] = float(v)
            wp.load_wafer(**kwargs)
            return "上片完成 offset=%s rz=%s z=%s rx=%s ry=%s%s" % (
                kwargs.get("offset", [0.0, 0.0]), kwargs.get("rz", 0.0),
                kwargs.get("z", 0.0), kwargs.get("rx", 0.0),
                kwargs.get("ry", 0.0), inject and "（处方注入）" or "")
        r = wp.unload_wafer()
        return "下片完成，随片移除 die %d 颗" % r["removed_dies"]

    def _do_film(self, action):
        wp = self.eng.workpieces
        if wp is None:
            raise RuntimeError("workpieces 未挂载")
        if action == "load":
            wp.load_film()                 # 幂等：置在位 + 按处方重铺
            return "蓝膜上机完成（film 在位，die 按处方重铺）"
        r = wp.unload_film()
        return "蓝膜下机完成，随机移除 die %d 颗" % r["removed_dies"]

    # ---------------- transfer 宏步骤 ----------------
    @staticmethod
    def _vacuum_circuit(point_path):
        """真空点位路径 → 回路名（如 ...M_AIR.ChuckVac.oNegtiveState → ChuckVac）。"""
        if not point_path.startswith(AIR_BASE):
            raise ValueError("真空点位不在 M_AIR 下：%r" % point_path)
        return point_path[len(AIR_BASE):].split(".", 1)[0]

    def _do_transfer(self, rule_name, timeout):
        """按 transfers.json 规则执行一次交接（只读 workpieces 公开状态，
        不改其内部）：建立接收侧真空 → 姿态轴到示教位 → 释放来源侧真空
        → 等 die 载体变化（引擎每 tick 上升沿触发）。"""
        wp = self.eng.workpieces
        if wp is None:
            raise RuntimeError("workpieces 未挂载")
        rule = next((r for r in wp.rules_public()["rules"]
                     if r["name"] == rule_name), None)
        if rule is None:
            raise KeyError("未知交接规则：%r" % rule_name)

        def count_to():
            with wp._lock:
                return sum(1 for d in wp.dies if d["carrier"] == rule["to"])
        before = count_to()

        # 1) 建立接收侧真空（先吸牢再移动，符合真机取放顺序）
        for p in rule["vacuum_on"]:
            self.eng.air_command(self._vacuum_circuit(p), "OpenNegtive")
        # 2) 姿态轴按文件顺序到示教位（含伺服 settling）
        for axis, (target, _tol) in rule["pose"].items():
            self._do_move(axis, target, None, timeout)
        # 3) 释放来源侧真空（真空交接给接收侧）
        for p in rule["vacuum_off"]:
            self.eng.air_command(self._vacuum_circuit(p), "OpenAir")
        # 4) 等 die 载体确实变化（上升沿触发一颗）
        try:
            self._wait(lambda: count_to() > before, timeout,
                       " 交接 %s 触发" % rule_name)
        except RuntimeError:
            raise RuntimeError("交接 %s 未触发：%s"
                               % (rule_name, self._transfer_diagnose(rule)))
        return "交接 %s 完成：%s→%s（%s 载体 %d→%d 颗）" % (
            rule_name, rule["from"], rule["to"], rule["to"], before,
            count_to())

    def _transfer_diagnose(self, rule):
        """交接超时诊断：逐项列出当前未满足的条件（公开状态重算）。"""
        unmet = []
        wp = self.eng.workpieces
        if "film" in (rule["from"], rule["to"]):
            with wp._lock:
                if not wp.film["present"]:
                    unmet.append("蓝膜不在位（film.present=False）")
        if rule.get("wafer_present") is not None:
            with wp._lock:
                if bool(rule["wafer_present"]) != wp.wafer["present"]:
                    unmet.append("晶圆在位=%s（要求 %s）"
                                 % (wp.wafer["present"], rule["wafer_present"]))
        for p in rule["vacuum_on"]:
            if not bool(self.points.get(p)):
                unmet.append("真空未建立 %s" % p.rsplit(".", 2)[-2])
        for p in rule["vacuum_off"]:
            if bool(self.points.get(p)):
                unmet.append("真空未释放 %s" % p.rsplit(".", 2)[-2])
        with self.eng.lock:
            for axis, (target, tol) in rule["pose"].items():
                ax = self.eng.axes.get(axis)
                if ax is None:
                    unmet.append("未知轴 %s" % axis)
                elif abs(ax.pos - float(target)) > float(tol):
                    unmet.append("%s=%.3f 不在示教位 %.3f±%.3f"
                                 % (axis, ax.pos, target, tol))
        with wp._lock:
            n_from = sum(1 for d in wp.dies if d["carrier"] == rule["from"])
        if n_from == 0:
            unmet.append("来源载体 %s 上无 die" % rule["from"])
        return "；".join(unmet) if unmet else "条件貌似已满足但仍未触发（排查规则表）"
