# -*- coding: utf-8 -*-
"""process_flows.py — 参数化工艺流程生成器（上位机工艺开发打样）。

每个 build_* 函数在**生成时刻**从机器常数（machine_constants.json 各组，
设备本体参数）与处方（recipe.json，物料/工艺参数）实时取参——PUT 改参后
下一次运行自动跟随（天然热生效，无需注册热更钩子）——产出 flow_engine
动态步骤列表，经 FlowEngine.run_steps 执行（与 flows.json 具名流程共用
状态机/pause/stop/step/日志，面板工艺流程 tab 直接可见）。

几何/工艺语义与 docs/工艺流程_翻转取片_V1.md 一一对应；变量命名即
上位机对照表里的名字（常数名=machine_constants 键名，处方名=recipe 键名）。

位姿约定（2026-08-25 用户按整机零位修正）：FA_Rx=0 = 吸盘面朝上的呈现位
（芯片背面朝上对着粗对准相机）；FA_Rx=−π = 取片位（吸盘面朝下够蓝膜）。
**FA_Rx 任何旋转前必须先抬 FA_Z 到 fa_z_safe_mm**（91mm 方板扫掠半径
≈64.2mm，FA_Z=0 直接转会切入蓝膜面）。
"""
import math

from recipe import wafer_sites_cells, wafer_sites_chip_size, \
    wafer_load_inject_params
from engine import effective_bond_contact_mm
from workpieces import site_bonded
import ega

# 真空点位（点位表事实源；回路名=transfers.json film_to_flip 的 vacuum_on）
CHUCK_VAC_PATH = "SystemMachine.SS_ME.M_AIR.ChuckVac"
CHUCK_VAC_CIRCUIT = "ChuckVac"

# 取片失败报警文本（上位机报警语义对照）
ALARM_VAC = "取片失败：ChuckVac 负压未建立（未拾取到芯片）"
ALARM_DIE = "取片失败：真空已建立但 die 未交接（carrier 未变 flip，" \
            "transfers.json film_to_flip 条件未触发）"


def _flip_dies(wp):
    with wp._lock:
        return [dict(d) for d in wp.dies if d["carrier"] == "flip"]


def _check_flip_empty(engine):
    """防重复拾取守卫：翻转吸盘容量 1（workpieces 侧同样硬约束），已有
    芯片时拒绝再取——先放回蓝膜（flip_place_back）或走键合流程。"""
    flips = _flip_dies(engine.workpieces)
    if flips:
        raise ValueError(
            "翻转吸盘已有芯片（%s）：不能重复拾取——先执行放回蓝膜"
            "（/api/process/flip_place_back）或键合交接流程"
            % ",".join(d["id"] for d in flips))


def build_flip_pick_flow(engine, die_id=None):
    """翻转模块拾取蓝膜芯片流程 -> (name, steps)。

    时序（用户工艺，2026-08-25 修正版）：
      1. FA_Z 抬安全位 → 翻转模块到取片姿态（拾取头朝下：FA_X/FA_Rx=−π；
         Rx 旋转前必抬安全位，防板角扫掠撞蓝膜）
      2. 蓝膜台（ES_X/ES_Y）把目标 die 送到取片位（三点同心：吸盘心/顶针/
         粗对准视场中心）
      3. FA_Z 下降到接触位（吸盘面贴 die 顶面，含 die 厚——防过压磨损）
      4. 蓝膜顶针顶起（顶到吸盘面 + 过压 pin_overtravel_um，顶起 die 脱膜）
      5. 开负压拾取 + 按负压状态判定拾到与否（超时/未交接 → 报警中止）
      6. 顶针降 → FA_Z 升安全位 → FA_Rx 翻转回 0（吸盘面朝上，芯片背面
         朝上——标记面朝下贴吸盘）→ FA_Z 调焦位（die 背面进 coarse 景深）

    die_id 缺省 = 离取片位最近的 film die。翻转吸盘已有芯片 → ValueError
    （API 层映射 400，防重复拾取）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    _check_flip_empty(engine)                   # 防重复拾取
    mc = engine.constants.get()["flip_pick"]       # 机械常数（生成时刻取最新）
    recipe = engine.recipe.get()                   # 处方（同上）

    # ---- 常数解包（名字 = machine_constants.json flip_pick 键名）----
    pick_x, pick_y = (float(v) for v in mc["pick_point_xy_mm"])
    film_surface_z_mm = float(mc["film_surface_z_mm"])   # 蓝膜面世界 z
    chuck_face_z0_mm = float(mc["chuck_face_z0_mm"])     # 取片姿态(Rx=−π)吸盘面 FA_Z=0 世界 z
    pin_top_z0_mm = float(mc["pin_top_z0_mm"])           # 顶针帽顶 ES_PunchZ=0
    fa_x_pick_mm = float(mc["fa_x_pick_mm"])
    fa_rx_pick_rad = float(mc["fa_rx_pick_rad"])
    fa_z_safe_mm = float(mc["fa_z_safe_mm"])
    fa_rx_flip_rad = float(mc["fa_rx_flip_rad"])
    fa_z_view_mm = float(mc["fa_z_view_mm"])
    pick_vac_timeout_s = float(mc["pick_vac_timeout_s"])

    # ---- 处方参数 ----
    die_thk_mm = float(recipe["film"]["dies"]["thickness_mm"])   # die 厚
    pin_overtravel_um = float(recipe["film"]["pin_overtravel_um"])  # 顶针过压

    # ---- 目标 die（缺省=离取片位最近的 film die）----
    with engine.workpieces._lock:
        dies = [dict(d) for d in engine.workpieces.dies
                if d["carrier"] == "film"]
    if not dies:
        raise ValueError("蓝膜上无 film die（先上膜/检查物料）")
    kp = engine.kinematics.config_public()
    tool_xy = kp["modules"]["film_stage"]["tool_point"][:2]
    if die_id is not None:
        die = next((d for d in dies if d["id"] == die_id), None)
        if die is None:
            raise ValueError("die %r 不存在或不在 film 上（现有 film die "
                             "%d 颗）" % (die_id, len(dies)))
    else:
        # 离取片位最近：die 世界 ≈ 工具点 + film_pos（姿态误差 µm 级忽略）
        die = min(dies, key=lambda d:
                  (d["film_pos"][0] - (pick_x - float(tool_xy[0]))) ** 2
                  + (d["film_pos"][1] - (pick_y - float(tool_xy[1]))) ** 2)
    die_id = die["id"]

    # ---- 几何推导（轴目标值；单位：直线 mm、旋转 rad）----
    # film 模块零位工具点 xy（error_kinematics.json 事实源，不新造坐标）：
    # die 世界 = 工具点 + ES 位移 + film_pos → 要把 die 送到取片位：
    #   ES_target = pick_point − tool_point_xy − film_pos
    es_x_pick = pick_x - float(tool_xy[0]) - die["film_pos"][0]
    es_y_pick = pick_y - float(tool_xy[1]) - die["film_pos"][1]
    # FA_Z 接触位：吸盘面目标 z = 蓝膜面 + die 厚（贴 die 顶面不过压）
    #   → fa_z_pick = film_surface_z_mm + die_thk_mm − chuck_face_z0_mm
    fa_z_pick = film_surface_z_mm + die_thk_mm - chuck_face_z0_mm
    # 顶针目标：顶针顶面 z = 吸盘面 z + 过压量（把 die 顶离膜面贴紧吸盘）
    #   → es_punch = film_surface_z_mm + die_thk_mm + n/1000 − pin_top_z0_mm
    es_punch_pick = film_surface_z_mm + die_thk_mm \
        + pin_overtravel_um / 1000.0 - pin_top_z0_mm

    name = "翻转取片（process/flip_pick %s）" % die_id
    steps = [
        {"op": "log",
         "msg": "=== 翻转取片 开始：die=%s（film_pos=%s，取片位 (%.2f, %.2f)）"
                % (die_id, die["film_pos"], pick_x, pick_y),
         "desc": "流程开始"},
        # 1. 取片姿态（先抬安全位再转 Rx，防扫掠撞膜；拾取头朝下=Rx=−π）
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 先抬安全位（fa_z_safe_mm；Rx 旋转前必抬，防板角"
                 "扫掠撞蓝膜）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_pick_mm,
         "desc": "翻转模块到取片位 X（机械常数 fa_x_pick_mm）"},
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_pick_rad,
         "speed": 1.5,
         "desc": "翻转模块到取片姿态（拾取头朝下，fa_rx_pick_rad=−π）"},
        # 2. 蓝膜台送 die 到取片位
        {"op": "move", "axis": "ES_X", "target": round(es_x_pick, 4),
         "desc": "蓝膜台 X 送 die 到取片位（pick−工具点−film_pos）"},
        {"op": "move", "axis": "ES_Y", "target": round(es_y_pick, 4),
         "desc": "蓝膜台 Y 送 die 到取片位"},
        # 3. FA_Z 下降到接触位（含 die 厚，防过压）
        {"op": "move", "axis": "FA_Z", "target": round(fa_z_pick, 4),
         "desc": "翻转 Z 下降到接触位：吸盘面=蓝膜面+die厚 %.3fmm"
                 % die_thk_mm},
        # 4. 顶针顶起 + 过压
        {"op": "move", "axis": "ES_PunchZ", "target": round(es_punch_pick, 4),
         "speed": 5.0,
         "desc": "蓝膜顶针顶起 die（过压 %.0fµm，处方 pin_overtravel_um）"
                 % pin_overtravel_um},
        # 5. 负压拾取 + 拾取判定
        {"op": "io", "path": CHUCK_VAC_PATH, "cmd": "OpenNegtive",
         "desc": "拾取头开负压（ChuckVac）"},
        {"op": "wait_grip", "circuit": CHUCK_VAC_CIRCUIT, "expect": True,
         "timeout": pick_vac_timeout_s, "err": ALARM_VAC,
         "desc": "等负压建立（超时→报警：未拾取到芯片）"},
        {"op": "wait_die", "carrier": "flip", "timeout": 5.0,
         "err": ALARM_DIE,
         "desc": "确认 die 已交接 carrier=flip（transfers.json "
                 "film_to_flip 上升沿）"},
        # 6. 收尾：顶针降 → Z 升安全位 → 翻转回 0（背面朝上）→ Z 调焦位
        {"op": "move", "axis": "ES_PunchZ", "target": 0.0, "speed": 5.0,
         "desc": "顶针降回零位"},
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "拾取头 Z 升安全位（fa_z_safe_mm；Rx 旋转前必抬）"},
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_flip_rad,
         "speed": 1.5,
         "desc": "翻转 %s：Rx 回 %s，吸盘面朝上，芯片背面朝上（标记面贴"
                 "吸盘）" % ("π→0" if abs(fa_rx_pick_rad) > 1.0 else
                             "回零", fa_rx_flip_rad)},
        {"op": "move", "axis": "FA_Z", "target": fa_z_view_mm,
         "desc": "调 FA_Z 进粗对准景深（fa_z_view_mm；coarse 焦面 "
                 "z=982.9，die 背面朝上进焦）"},
        {"op": "coarse_offset", "cam": "coarse",
         "desc": "粗对准拍照：测 die 世界坐标，与标定中心 "
                 "coarse_die_center_mm 对比得偏差（对准策略输入）"},
        {"op": "log",
         "msg": "=== 翻转取片 完成：%s 已在 flip（背面朝上，可粗对准）"
                 % die_id,
         "desc": "流程完成"},
    ]
    return name, steps


def build_flip_place_back_flow(engine):
    """放回蓝膜（flip_pick 的逆向流程）-> (name, steps)。

    把翻转吸盘上的芯片原位放回蓝膜（die 保留的 film_pos 即原槽位）：
      1. FA_Z 抬安全位 → FA_Rx 转取片姿态（−π，吸盘面朝下；旋转前必抬）
      2. 蓝膜台把原槽位送到取片位（ES 反算同拾取）
      3. FA_Z 下降到放片段（吸盘面=蓝膜面+die厚，die 底面落回膜面）
      4. 破真空（OpenAir 吹扫释放）→ die 交接回 film（transfers.json
         flip_to_film：真空断 + 位姿窗口 上升沿）
      5. FA_Z 升安全位 → FA_Rx 回 0（吸盘面朝上空载）→ 完成

    翻转吸盘无芯片 → ValueError（API 层映射 400）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    flips = _flip_dies(engine.workpieces)
    if not flips:
        raise ValueError("翻转吸盘上没有芯片（先执行翻转取片）")
    die = flips[0]
    mc = engine.constants.get()["flip_pick"]
    recipe = engine.recipe.get()

    pick_x, pick_y = (float(v) for v in mc["pick_point_xy_mm"])
    film_surface_z_mm = float(mc["film_surface_z_mm"])
    chuck_face_z0_mm = float(mc["chuck_face_z0_mm"])
    fa_x_pick_mm = float(mc["fa_x_pick_mm"])
    fa_rx_pick_rad = float(mc["fa_rx_pick_rad"])
    fa_z_safe_mm = float(mc["fa_z_safe_mm"])
    fa_rx_flip_rad = float(mc["fa_rx_flip_rad"])

    die_thk_mm = float(recipe["film"]["dies"]["thickness_mm"])

    kp = engine.kinematics.config_public()
    tool_xy = kp["modules"]["film_stage"]["tool_point"][:2]
    # 原槽位（die 保留 film_pos）——被其它 film die 占用则拒绝（防叠片）
    slot = [float(v) for v in die["film_pos"]]
    with engine.workpieces._lock:
        occupied = any(
            d["carrier"] == "film"
            and (d["film_pos"][0] - slot[0]) ** 2
            + (d["film_pos"][1] - slot[1]) ** 2 < 0.25   # <0.5mm 视为同槽
            for d in engine.workpieces.dies)
    if occupied:
        raise ValueError("原槽位 %s 已被其它 film die 占用，无法放回"
                         % slot)
    es_x_slot = pick_x - float(tool_xy[0]) - slot[0]
    es_y_slot = pick_y - float(tool_xy[1]) - slot[1]
    # 放片段 = 接触位（吸盘面=蓝膜面+die厚 → die 底面恰落膜面）
    fa_z_place = film_surface_z_mm + die_thk_mm - chuck_face_z0_mm

    name = "放回蓝膜（process/flip_place_back %s）" % die["id"]
    steps = [
        {"op": "log",
         "msg": "=== 放回蓝膜 开始：%s 回原槽位 %s" % (die["id"], slot),
         "desc": "流程开始"},
        # 1. 抬安全位 + 转取片姿态（吸盘面朝下）
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 先抬安全位（Rx 旋转前必抬，防扫掠撞蓝膜）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_pick_mm,
         "desc": "翻转模块到放回位 X（fa_x_pick_mm）"},
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_pick_rad,
         "speed": 1.5,
         "desc": "翻转到放回姿态（吸盘面朝下，fa_rx_pick_rad=−π）"},
        # 2. 蓝膜台送原槽位到放回位
        {"op": "move", "axis": "ES_X", "target": round(es_x_slot, 4),
         "desc": "蓝膜台 X 送原槽位到放回位"},
        {"op": "move", "axis": "ES_Y", "target": round(es_y_slot, 4),
         "desc": "蓝膜台 Y 送原槽位到放回位"},
        # 3. 下降到放片段（die 底面落膜面）
        {"op": "move", "axis": "FA_Z", "target": round(fa_z_place, 4),
         "desc": "翻转 Z 下降到放片段：吸盘面=蓝膜面+die厚 %.3fmm"
                 % die_thk_mm},
        # 4. 破真空放片 + 交接判定
        {"op": "io", "path": CHUCK_VAC_PATH, "cmd": "OpenAir",
         "desc": "破真空（OpenAir 吹扫释放芯片）"},
        {"op": "wait_die", "carrier": "film", "vacated": ["flip"],
         "timeout": 5.0,
         "err": "放片失败：真空已断但 die 未交回蓝膜（transfers.json "
                "flip_to_film 条件未触发）",
         "desc": "确认 die 已交回 carrier=film 且翻转吸盘腾空"
                 "（flip_to_film 上升沿）"},
        # 5. 抬安全位 + 翻转回呈现位（空载）
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 升安全位（Rx 旋转前必抬）"},
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_flip_rad,
         "speed": 1.5,
         "desc": "翻转回呈现位（Rx=0，吸盘面朝上空载）"},
        {"op": "log",
         "msg": "=== 放回蓝膜 完成：%s 已回 film 槽位 %s"
                 % (die["id"], slot),
         "desc": "流程完成"},
    ]
    return name, steps


# ---------------------------------------------------------------------------
# 芯片交接（flip↔head）：几何/符号约定见 docs/工艺流程_芯片交接_V1.md
# ---------------------------------------------------------------------------

# 键合头真空点位（点位表事实源；回路名=transfers.json flip_to_head 的
# vacuum_on 三条）
HEAD_VAC_PATHS = ["SystemMachine.SS_ME.M_AIR.WT_BondingVac_1",
                  "SystemMachine.SS_ME.M_AIR.WT_BondingVac_2",
                  "SystemMachine.SS_ME.M_AIR.WT_BondingVac_3"]
HEAD_VAC_CIRCUITS = ["WT_BondingVac_1", "WT_BondingVac_2", "WT_BondingVac_3"]

# 交接/吸回失败报警文本（上位机报警语义对照）
ALARM_HEAD_VAC = "交接失败：键合头负压未建立（WT_BondingVac 未吸牢）"
ALARM_HANDOFF_DIE = "交接失败：真空已建立但 die 未交接（carrier 未变 head，" \
                    "transfers.json flip_to_head 条件未触发）"
ALARM_CHUCK_VAC = "吸回失败：ChuckVac 负压未建立（翻转吸盘未吸牢）"
ALARM_RECOVER_DIE = "吸回失败：真空已建立但 die 未交回（carrier 未变 flip，" \
                    "transfers.json head_to_flip 条件未触发）"


def _head_dies(wp):
    with wp._lock:
        return [dict(d) for d in wp.dies if d["carrier"] == "head"]


def _handoff_constants(engine):
    """交接用常数解包（生成时刻取最新）-> (fh, fa_z_safe_mm, fa_rx_flip_rad)。
    fh = machine_constants flip_handoff 组全量（键名原样）。"""
    fh = dict(engine.constants.get()["flip_handoff"])
    fp = engine.constants.get()["flip_pick"]
    return fh, float(fp["fa_z_safe_mm"]), float(fp["fa_rx_flip_rad"])


def build_flip_handoff_flow(engine):
    """芯片交接：翻转吸盘上的 die 交给键合头（flip→head）-> (name, steps)。

    时序（用户工艺，2026-08-26；机械常数=flip_handoff 组键名）：
      1. 安全确认：对准台退 as_safe_xy_mm、键合头到 head_z_safe_mm、
         翻转 Z 到 fa_z_safe_mm（传送安全高度，复用 flip_pick 组）
      2. FA_Rx→fa_rx_flip_rad（幂等，确保吸盘面朝上/die 背面朝上）
      3. FA_X 到交接位：**fa_x_handoff_mm − off_x**——off 为粗对准实测
         偏差（eng.coarse_offset，coarse_offset 步落存；符号推导见
         docs/工艺流程_芯片交接_V1.md：die 世界 x=工具点+FA_X+site，
         要 die 心=键合头心则 FA_X 须反向扣掉偏差）
      4. BS_Arm_Z 下降到 head_z_touch_mm（压头底刚接触 die 背面）
      5. 键合头三吸盘开负压 + grip 判定（超时→报警中止）
      6. ChuckVac 正压吹离 200ms → 回大气（真空交给键合头；oNegtiveState
         在正压沿即落 → transfers flip_to_head 上升沿，防两侧真空抢芯片）
         + wait_die 确认交接真实发生
      7. 键合臂抬 head_z_lift_mm → FA_X 撤 fa_x_retreat_mm → 完成
         （die 在 head，**标记面朝下**对晶圆）

    守卫：flip 无 die → ValueError（先取片）；head 已有 die → ValueError
    （防叠片，workpieces 容量 1 硬约束之外的流程层拦截）。无粗对准记录
    （或记录属别的 die）→ off=[0,0] 按名义位交接并日志警告。Y 向无轴
    可补偿（off_y 仅记日志，已知限制见常数 _note）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    flips = _flip_dies(engine.workpieces)
    if _head_dies(engine.workpieces):
        raise ValueError("键合头上已有芯片：不能重复交接——先键合"
                         "（head_to_wafer）或吸回翻转头（head_to_flip）")
    if not flips:
        raise ValueError("翻转吸盘上没有芯片（先执行翻转取片 flip_pick）")
    die = flips[0]
    fh, fa_z_safe_mm, fa_rx_flip_rad = _handoff_constants(engine)

    as_safe_x, as_safe_y = (float(v) for v in fh["as_safe_xy_mm"])
    head_z_safe_mm = float(fh["head_z_safe_mm"])
    head_z_touch_mm = float(fh["head_z_touch_mm"])
    head_z_lift_mm = float(fh["head_z_lift_mm"])
    fa_x_handoff_mm = float(fh["fa_x_handoff_mm"])
    fa_x_retreat_mm = float(fh["fa_x_retreat_mm"])
    handoff_vac_timeout_s = float(fh["handoff_vac_timeout_s"])

    # ---- 粗对准偏差补偿（X 向；Y 向翻转模块无轴，仅记录）----
    off_x, off_y, warn = 0.0, 0.0, None
    co = getattr(engine, "coarse_offset", None)
    if co and co.get("die") == die["id"]:
        off_x, off_y = float(co["dx"]), float(co["dy"])
    else:
        warn = ("未做粗对准（或测量属其它 die）：按名义位交接，"
                "补偿 (0, 0)")
    fa_x_handoff = fa_x_handoff_mm - off_x     # 符号约定见流程文档

    name = "芯片交接（process/flip_handoff %s）" % die["id"]
    steps = [
        {"op": "log",
         "msg": "=== 芯片交接 开始：die=%s（粗对准偏差 (%+.4f, %+.4f)mm → "
                "FA_X 目标 %.4f = fa_x_handoff %.4f − off_x；%s）"
                % (die["id"], off_x, off_y, fa_x_handoff, fa_x_handoff_mm,
                   warn or "Y 向偏差 %.4fmm 无轴补偿，仅记录" % off_y),
         "desc": "流程开始"},
        # 1. 安全确认（用户工艺需求 1：三方都先到位再传送）
        {"op": "move", "axis": "AS_X", "target": as_safe_x,
         "desc": "对准台 X 退安全位（机械常数 as_safe_xy_mm）"},
        {"op": "move", "axis": "AS_Y", "target": as_safe_y,
         "desc": "对准台 Y 退安全位（退出键合头下方）"},
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe_mm,
         "desc": "键合头到安全高度（机械常数 head_z_safe_mm）"},
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 到传送安全高度（fa_z_safe_mm）"},
        # 2. 姿态幂等确认（吸盘面朝上，die 背面朝上）
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_flip_rad,
         "speed": 1.5,
         "desc": "确认呈现位（FA_Rx=%s，吸盘面朝上；已在位则秒过）"
                 % fa_rx_flip_rad},
        # 3. 传送（含粗对准补偿）
        {"op": "move", "axis": "FA_X", "target": round(fa_x_handoff, 4),
         "desc": "翻转模块送 die 到键合头正下方（fa_x_handoff_mm %.4f − "
                 "粗对准补偿 %+.4f；±10mm 示教窗口吸收残差）"
                 % (fa_x_handoff_mm, -off_x)},
        # 4. 键合头下降接触 die 背面
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_touch_mm,
         "desc": "键合头下降到接触位（机械常数 head_z_touch_mm；压头底刚"
                 "接触 die 背面）"},
        # 5. 键合头三吸盘开负压 + 逐路 grip 判定
        {"op": "io", "path": HEAD_VAC_PATHS[0], "cmd": "OpenNegtive",
         "desc": "键合头吸盘 1 开负压"},
        {"op": "io", "path": HEAD_VAC_PATHS[1], "cmd": "OpenNegtive",
         "desc": "键合头吸盘 2 开负压"},
        {"op": "io", "path": HEAD_VAC_PATHS[2], "cmd": "OpenNegtive",
         "desc": "键合头吸盘 3 开负压"},
        {"op": "wait_grip", "circuit": HEAD_VAC_CIRCUITS[0], "expect": True,
         "timeout": handoff_vac_timeout_s, "err": ALARM_HEAD_VAC + "（1）",
         "desc": "等吸盘 1 吸牢（超时→报警）"},
        {"op": "wait_grip", "circuit": HEAD_VAC_CIRCUITS[1], "expect": True,
         "timeout": handoff_vac_timeout_s, "err": ALARM_HEAD_VAC + "（2）",
         "desc": "等吸盘 2 吸牢"},
        {"op": "wait_grip", "circuit": HEAD_VAC_CIRCUITS[2], "expect": True,
         "timeout": handoff_vac_timeout_s, "err": ALARM_HEAD_VAC + "（3）",
         "desc": "等吸盘 3 吸牢"},
        # 6. ChuckVac 正压吹离 → 回大气 → 真空交接 → 确认 die 到 head
        {"op": "io", "path": CHUCK_VAC_PATH, "cmd": "OpenPositive",
         "desc": "翻转吸盘正压吹离（600kPa 小正压推 die 背面主动脱离，"
                 "防键合头/翻转两侧真空「抢芯片」撕扯；oNegtiveState 已落，"
                 "flip_to_head 在此沿触发）"},
        {"op": "delay", "ms": 200,
         "desc": "正压保持 200ms（吹离建立）"},
        {"op": "io", "path": CHUCK_VAC_PATH, "cmd": "OpenAir",
         "desc": "翻转吸盘回大气（吹离结束，气路回常态）"},
        {"op": "wait_die", "carrier": "head", "vacated": ["flip"],
         "timeout": 5.0, "err": ALARM_HANDOFF_DIE,
         "desc": "确认 die 已交接 carrier=head 且翻转吸盘腾空"
                 "（交接已在正压沿触发，此处复核双载体态）"},
        # 7. 抬起 + 撤离
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_lift_mm,
         "desc": "键合臂抬起（机械常数 head_z_lift_mm）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_retreat_mm,
         "desc": "翻转模块撤离（机械常数 fa_x_retreat_mm），准备取下一颗"},
        {"op": "log",
         "msg": "=== 芯片交接 完成：%s 已在 head（标记面朝下对晶圆，"
                "可走上精对准/键合）" % die["id"],
         "desc": "流程完成"},
    ]
    return name, steps


def build_head_to_flip_flow(engine):
    """吸回：键合头上的 die 交回翻转拾取头（head→flip，交接逆向）
    -> (name, steps)。

    时序（与正向镜像；die 在 head 上居中 site=[0,0]——世界 x=头心，
    FA_X 用名义交接位**不补偿**）：
      1. 安全确认同正向（对准台退避/键合头安全高度/翻转 Z 安全位/Rx 幂等）
      2. FA_X 到 fa_x_handoff_mm（吸盘面正对 die 标记面）
      3. BS_Arm_Z 下降到 head_z_touch_mm（正逆共用：两吸盘间隙恒=die 厚，
         die 厚度在两侧相消，几何推导见流程文档）
      4. ChuckVac 开负压 + grip 判定（超时→报警中止）
      5. 键合头三吸盘正压吹离 200ms → 回大气（正压沿即触发 transfers
         head_to_flip，与正向对称）+ wait_die
      6. 键合臂抬起 → 翻转撤离 → 完成（die 回 flip，标记面朝下贴吸盘，
         与取片后状态一致，可再次交接——闭环可逆）

    守卫：head 无 die → ValueError；flip 已有 die → ValueError。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    heads = _head_dies(engine.workpieces)
    if not heads:
        raise ValueError("键合头上没有芯片（先执行芯片交接 flip_handoff）")
    if _flip_dies(engine.workpieces):
        raise ValueError("翻转吸盘上已有芯片：不能吸回——先放回蓝膜"
                         "（flip_place_back）")
    die = heads[0]
    fh, fa_z_safe_mm, fa_rx_flip_rad = _handoff_constants(engine)

    as_safe_x, as_safe_y = (float(v) for v in fh["as_safe_xy_mm"])
    head_z_safe_mm = float(fh["head_z_safe_mm"])
    head_z_touch_mm = float(fh["head_z_touch_mm"])
    head_z_lift_mm = float(fh["head_z_lift_mm"])
    fa_x_handoff_mm = float(fh["fa_x_handoff_mm"])
    fa_x_retreat_mm = float(fh["fa_x_retreat_mm"])
    handoff_vac_timeout_s = float(fh["handoff_vac_timeout_s"])

    name = "吸回翻转头（process/head_to_flip %s）" % die["id"]
    steps = [
        {"op": "log",
         "msg": "=== 吸回翻转头 开始：die=%s（head 上居中，FA_X 名义交接位 "
                "%.4f 不补偿）" % (die["id"], fa_x_handoff_mm),
         "desc": "流程开始"},
        # 1. 安全确认
        {"op": "move", "axis": "AS_X", "target": as_safe_x,
         "desc": "对准台 X 退安全位（as_safe_xy_mm）"},
        {"op": "move", "axis": "AS_Y", "target": as_safe_y,
         "desc": "对准台 Y 退安全位"},
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe_mm,
         "desc": "键合头到安全高度（head_z_safe_mm）"},
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 到安全高度（fa_z_safe_mm）"},
        {"op": "move", "axis": "FA_Rx", "target": fa_rx_flip_rad,
         "speed": 1.5,
         "desc": "确认呈现位（FA_Rx=%s，吸盘面朝上接 die 标记面）"
                 % fa_rx_flip_rad},
        # 2. 翻转模块到交接位（不补偿）
        {"op": "move", "axis": "FA_X", "target": fa_x_handoff_mm,
         "desc": "翻转模块到交接位 X（fa_x_handoff_mm；die 在 head 居中，"
                 "无需补偿）"},
        # 3. 键合头带 die 下降（die 标记面贴上翻转吸盘面）
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_touch_mm,
         "desc": "键合头下降到接触位（head_z_touch_mm；die 标记面贴翻转"
                 "吸盘面）"},
        # 4. ChuckVac 开负压 + grip 判定
        {"op": "io", "path": CHUCK_VAC_PATH, "cmd": "OpenNegtive",
         "desc": "翻转吸盘开负压（ChuckVac）"},
        {"op": "wait_grip", "circuit": CHUCK_VAC_CIRCUIT, "expect": True,
         "timeout": handoff_vac_timeout_s, "err": ALARM_CHUCK_VAC,
         "desc": "等翻转吸盘吸牢（超时→报警）"},
        # 5. 键合头三吸盘正压吹离 → 回大气 → 交接回 flip（与正向对称：
        #    正压主动脱离防两侧真空撕扯；OpenPositive 沿即触发 head_to_flip）
        {"op": "io", "path": HEAD_VAC_PATHS[0], "cmd": "OpenPositive",
         "desc": "键合头吸盘 1 正压吹离"},
        {"op": "io", "path": HEAD_VAC_PATHS[1], "cmd": "OpenPositive",
         "desc": "键合头吸盘 2 正压吹离"},
        {"op": "io", "path": HEAD_VAC_PATHS[2], "cmd": "OpenPositive",
         "desc": "键合头吸盘 3 正压吹离（触发 head_to_flip）"},
        {"op": "delay", "ms": 200,
         "desc": "正压保持 200ms（吹离建立）"},
        {"op": "io", "path": HEAD_VAC_PATHS[0], "cmd": "OpenAir",
         "desc": "键合头吸盘 1 回大气"},
        {"op": "io", "path": HEAD_VAC_PATHS[1], "cmd": "OpenAir",
         "desc": "键合头吸盘 2 回大气"},
        {"op": "io", "path": HEAD_VAC_PATHS[2], "cmd": "OpenAir",
         "desc": "键合头吸盘 3 回大气"},
        {"op": "wait_die", "carrier": "flip", "vacated": ["head"],
         "timeout": 5.0, "err": ALARM_RECOVER_DIE,
         "desc": "确认 die 已交回 carrier=flip 且键合头腾空"
                 "（交接已在正压沿触发，此处复核双载体态）"},
        # 6. 抬起 + 撤离
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_lift_mm,
         "desc": "键合臂抬起（head_z_lift_mm）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_retreat_mm,
         "desc": "翻转模块撤离（fa_x_retreat_mm）"},
        {"op": "log",
         "msg": "=== 吸回翻转头 完成：%s 已回 flip（标记面朝下贴吸盘，"
                "可再次交接）" % die["id"],
         "desc": "流程完成"},
    ]
    return name, steps


# ---------------------------------------------------------------------------
# 晶圆上片 + 全局对准（EGA）
# ---------------------------------------------------------------------------

# 晶圆吸盘真空点位（点位表事实源；回路名=transfers.json head_to_wafer 的
# vacuum_on——上片不开真空，键合交接永不触发）
WAFER_VAC_PATH = "SystemMachine.SS_ME.M_AIR.WT_WaferVac"
WAFER_VAC_CIRCUIT = "WT_WaferVac"
ALARM_WAFER_VAC = "上片失败：晶圆吸盘负压未建立（WT_WaferVac）"


def _wafer_vac_steps(timeout_s):
    """晶圆吸盘开负压 + grip 判定（幂等两步；上片后必做，晶圆已在位分支
    也做——真空可能因故障注入等被关过）。"""
    return [
        {"op": "io", "path": WAFER_VAC_PATH, "cmd": "OpenNegtive",
         "desc": "晶圆吸盘开负压（WT_WaferVac；幂等）"},
        {"op": "wait_grip", "circuit": WAFER_VAC_CIRCUIT, "expect": True,
         "timeout": timeout_s, "err": ALARM_WAFER_VAC,
         "desc": "等晶圆真空建立（超时→报警：晶圆真空未建立）"},
    ]

def build_wafer_ega_flow(engine, with_load=None):
    """晶圆上片 + EGA 全局对准流程 -> (name, steps)。

    时序（用户工艺，2026-08-26；机械常数=wafer_ega / flip_handoff / flip_pick
    组键名；处方参数=recipe.wafer_load）：
      1. 若硅片台上无晶圆（或 with_load=True），执行上片：按 wafer_load
         注入平移/旋转误差 + 随机落片抖动（jitter_on 控制）
      2. 安全确认：对准台退 as_safe_xy_mm、键合头到 head_z_safe_mm、
         翻转模块到撤离位（fa_x_retreat / fa_z_safe）
      3. 硅片台到 EGA 测量位 ws_ega_xyz_mm
      4. 对准台到锚点 as_anchor_xy_mm（此时光轴对准硅片台中心）
      5. 沿晶圆边缘均匀选取 ega_n_points 个 site，每个 site 测第 1 个
         site_mark：下精对准相机拍照 → 反投影世界坐标 → 记录名义/实测对
      6. ega_fit 六参数仿射拟合，结果落存 engine.ega_result

    Args:
        with_load: None=自动（无晶圆则上片）；True=强制上片；False=不上片

    守卫：视觉系统未挂载 → RuntimeError。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    if engine.vision is None:
        raise RuntimeError("晶圆 EGA 流程需要视觉系统（vision not attached）")

    mc = engine.constants.get()
    we = mc["wafer_ega"]
    fh = mc["flip_handoff"]
    fp = mc["flip_pick"]
    recipe = engine.recipe.get()
    wl = recipe["wafer_load"]

    ws_ega_x, ws_ega_y, ws_ega_z = (float(v) for v in we["ws_ega_xyz_mm"])
    as_anchor_x, as_anchor_y = (float(v) for v in we["as_anchor_xy_mm"])
    ega_n_points = int(we["ega_n_points"])

    as_safe_x, as_safe_y = (float(v) for v in fh["as_safe_xy_mm"])
    head_z_safe_mm = float(fh["head_z_safe_mm"])
    fa_x_retreat_mm = float(fh["fa_x_retreat_mm"])
    fa_z_safe_mm = float(fp["fa_z_safe_mm"])

    # 是否要上片
    wafer_present = engine.workpieces.wafer["present"]
    do_load = (with_load is True) or (with_load is None and not wafer_present)

    # 上片误差注入（含随机落片抖动；recipe.wafer_load_inject_params 单一
    # 事实源——手动上片 inject_recipe 通道同款；垂向 z/rx/ry 不参与 jitter）
    _inj = wafer_load_inject_params(wl)
    load_x, load_y = _inj["offset"]
    load_rz = _inj["rz"]
    load_z, load_rx, load_ry = _inj["z"], _inj["rx"], _inj["ry"]

    # EGA 测点：site 中心 → 取第 1 个 site_mark 的名义坐标
    cells = wafer_sites_cells(recipe)
    if len(cells) < ega_n_points:
        raise ValueError("晶圆键合位不足 %d 个（当前 %d），无法执行 EGA"
                         % (ega_n_points, len(cells)))
    site_centers = ega.select_measure_points(cells, n_pts=ega_n_points)
    sx, sy = wafer_sites_chip_size(recipe)
    sm = recipe["wafer"]["site_marks"]
    ratio = sm["ratio"][:int(sm.get("per_site", len(sm["ratio"])))]
    if not ratio:
        raise ValueError("wafer.site_marks 无可用标记比例点")
    # 每个 site 只测第一个 mark（避免流程过长；需要更多点可改 ega_n_points）
    rx, ry = ratio[0]
    mark_pts = [(round(c[0] + rx * sx / 2.0, 6),
                 round(c[1] + ry * sy / 2.0, 6)) for c in site_centers]

    steps = []
    if do_load:
        steps.append({"op": "log",
                      "msg": "=== 晶圆 EGA 开始：无晶圆，先执行上片 ===",
                      "desc": "流程开始"})
        steps.append({"op": "wafer", "action": "load",
                      "offset": [round(load_x, 6), round(load_y, 6)],
                      "rz": round(load_rz, 8),
                      "z": round(load_z, 6),
                      "rx": round(load_rx, 8),
                      "ry": round(load_ry, 8),
                      "desc": "晶圆上片：注入 wafer_load 误差（含随机抖动）"
                               "[%.4f, %.4f] mm / %.4f urad；垂向 z=%.1f um "
                               "rx=%.1f ry=%.1f urad"
                               % (load_x * 1000, load_y * 1000,
                                  load_rz * 1e6, load_z * 1000,
                                  load_rx * 1e6, load_ry * 1e6)})
        # 上片后开晶圆吸盘真空（迭代 8 bug 修复：原流程全程无 WT_WaferVac
        # 气路步，transfers head_to_wafer 的 vacuum_on 永不满足）
        steps.extend(_wafer_vac_steps(float(fh["handoff_vac_timeout_s"])))
    else:
        steps.append({"op": "log",
                      "msg": "=== 晶圆 EGA 开始：晶圆已在位，直接全局对准 ===",
                      "desc": "流程开始"})
        # 晶圆已在位分支同样幂等开真空（真空可能被故障注入等关过）
        steps.extend(_wafer_vac_steps(float(fh["handoff_vac_timeout_s"])))

    # 安全确认
    steps.extend([
        {"op": "move", "axis": "AS_X", "target": as_safe_x,
         "desc": "对准台 X 退安全位（as_safe_xy_mm，防与键合头/翻转干涉）"},
        {"op": "move", "axis": "AS_Y", "target": as_safe_y,
         "desc": "对准台 Y 退安全位"},
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe_mm,
         "desc": "键合头到安全高度（head_z_safe_mm）"},
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 到安全高度（fa_z_safe_mm）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_retreat_mm,
         "desc": "翻转模块撤离（fa_x_retreat_mm）"},
    ])

    # 硅片台/对准台到位
    steps.extend([
        {"op": "move", "axis": "WS_LS_X", "target": ws_ega_x,
         "desc": "硅片台 X 到 EGA 测量位（ws_ega_xyz_mm[0]）"},
        {"op": "move", "axis": "WS_LS_Y", "target": ws_ega_y,
         "desc": "硅片台 Y 到 EGA 测量位（ws_ega_xyz_mm[1]）"},
        {"op": "move", "axis": "WS_Rz", "target": ws_ega_z,
         "desc": "硅片台 Rz 到 EGA 测量位（ws_ega_xyz_mm[2]）"},
        {"op": "move", "axis": "AS_Z", "target": 0.0,
         "desc": "对准台 Z 到焦面（下精对准相机景深中心）"},
        {"op": "move", "axis": "AS_X", "target": as_anchor_x,
         "desc": "对准台 X 到锚点（as_anchor_xy_mm[0]，光轴对硅片台中心）"},
        {"op": "move", "axis": "AS_Y", "target": as_anchor_y,
         "desc": "对准台 Y 到锚点（as_anchor_xy_mm[1]）"},
    ])

    # EGA 逐点测量
    for i, (mx, my) in enumerate(mark_pts, 1):
        steps.append({"op": "ega_measure",
                      "site_x": mx, "site_y": my,
                      "cam": "down",
                      "desc": "EGA 测点 %d/%d：site_mark (%.3f, %.3f)"
                               % (i, len(mark_pts), mx, my)})

    steps.append({"op": "ega_fit",
                  "desc": "EGA 六参数仿射拟合（对准台坐标系↔晶圆设计坐标系）"})
    steps.append({"op": "log",
                  "msg": "=== 晶圆 EGA 完成：拟合结果见 flow.state.ega_result ===",
                  "desc": "流程完成"})

    name = "晶圆上片与全局对准（process/wafer_ega）"
    return name, steps


# ---------------------------------------------------------------------------
# 调焦调平（focus_level：共焦测高 → 平面拟合 → BS_CAL 调平 + 定焦）
# ---------------------------------------------------------------------------

def _fl_point_pattern(n):
    """调焦调平 N 点均布图案（相对 site/die 半宽的比例坐标）。"""
    _PATTERNS = {
        3: [[-0.3, -0.3], [0.3, -0.3], [0.0, 0.35]],
        4: [[-0.3, -0.3], [0.3, -0.3], [-0.3, 0.3], [0.3, 0.3]],
        5: [[0.0, 0.0], [-0.3, -0.3], [0.3, -0.3],
            [-0.3, 0.3], [0.3, 0.3]],
        6: [[-0.3, -0.3], [0.3, -0.3], [-0.3, 0.3], [0.3, 0.3],
            [0.0, -0.35], [0.0, 0.35]],
        7: [[0.0, 0.0], [-0.3, -0.3], [0.3, -0.3], [-0.3, 0.3], [0.3, 0.3],
            [0.0, -0.35], [0.0, 0.35]],
        8: [[-0.3, -0.3], [0.3, -0.3], [-0.3, 0.3], [0.3, 0.3],
            [0.0, -0.35], [0.0, 0.35], [-0.35, 0.0], [0.35, 0.0]],
    }
    return _PATTERNS.get(n, _PATTERNS[4])


def _fl_n_points(n, sx, sy, ratio=None, scale=0.3):
    """生成 N 个局部测点（site/die 半宽×比例）。

    ratio 非空且够长时优先用 ratio（site_marks / film.marks 的相对比例）；
    否则返回均布图案（scale 控制内缩程度）。
    """
    pts = []
    if ratio and len(ratio) >= n:
        for r in ratio[:n]:
            pts.append((round(r[0] * sx / 2.0, 6),
                        round(r[1] * sy / 2.0, 6)))
        return pts
    for r in _fl_point_pattern(n):
        pts.append((round(r[0] * sx * scale, 6),
                    round(r[1] * sy * scale, 6)))
    return pts


def _fl_pick_triangle(sel):
    """global 模式"切蛋糕"选点：从 EGA 站点 sel 中取面积最大的三角形 3 点
    （平面拟合最小点数；等价近似 120° 均布、覆盖最大半径）。
    O(k³) 全组合，k=ega_n_points≤9，开销可忽略。"""
    import itertools
    best, best_area = None, -1.0
    for tri in itertools.combinations(sel, 3):
        (x1, y1), (x2, y2), (x3, y3) = tri
        area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2.0
        if area > best_area:
            best, best_area = tri, area
    if best is None:
        raise ValueError("EGA 站点不足 3 个，无法切蛋糕选 3 点")
    return [tuple(p) for p in best]


def build_focus_level_flow(engine, mode="per_site", site=None,
                           align_first=None, verify=True):
    """调焦调平流程 -> (name, steps)。

    模式（POST 参数，不动 recipe 结构）：
      per_site（默认）：键合位就位 → WS 冻结 → 下相机测晶圆面 N 点 →
        上相机测键合头 die 面 N 点 → 平面拟合 → BS_CAL_Rx/Ry 调平 +
        AS_Z/BS_Arm_Z 定焦 → 可选 N 点复测 + fl_verify 残余阈值判定；
      global：晶圆面固定 3 点（EGA 站点切蛋糕最大面积三角形，允许
        WS 运动），芯片面仍 N 点，其余同 per_site；
        已有 level 记忆（eng.level_memory）时跳过测量，直接施加记忆；
      off：只对焦不调平——进场高度 + 晶圆/芯片中心点共焦确认。

    流程参数：
      align_first：true=先相机 identify 标记再共焦测高；缺省从
        recipe.focus_level.align_first 读取。
      verify：false=跳过最终复测段（默认 true）。
      measure_points：晶圆面与芯片面的测点数 N，从
        recipe.focus_level.measure_points 读取（默认 4，支持 3~8）。

    关键几何（2026-08-27 迭代 3）：
      - 上相机认键合头吸盘中心时的对准台轴位 = 机械常数
        as_head_center_xy_mm（默认由前端几何推出，**待真机测校**）。
      - per_site 模式测高期间冻结 WS，仅 AS 走位（硅片台零移动）。
      - global 模式晶圆面测点为 EGA 站点切蛋糕 3 点，沿用 WS+AS split，
        不冻结；measure_points 只影响芯片侧。
      - die 侧测点以 as_head_center_xy_mm 为基准，不再依赖 die 世界
        真值定位（真值只用于读数物理）。

    守卫：晶圆不在位 / per_site 已键合 / head 无 die / 未 EGA / mode 非法
    → ValueError（API 层映射 400）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    if engine.confocal is None:
        raise RuntimeError("调焦调平流程需要共焦模型（confocal not attached）")
    if mode not in ("per_site", "global", "off"):
        raise ValueError("mode 只能是 per_site/global/off：%r" % (mode,))
    wp = engine.workpieces
    with wp._lock:
        wafer_present = bool(wp.wafer["present"])
    if not wafer_present:
        raise ValueError("晶圆不在位（先上片：/api/process/wafer_ega）")
    # 已键合守卫优先于"head 无 die"（bond 后 head 必然空——site 是用户
    # 显式指定的对象，报更具体的错；与 bond 的守卫顺序一致）。
    # 进程内守卫链单测的伪 engine.recipe 不是 Recipe 实例，取不到处方时跳过
    if mode == "per_site":
        try:
            _cells = wafer_sites_cells(engine.recipe.get())
        except (AttributeError, KeyError, TypeError):
            _cells = []
        if _cells:
            if site is not None:
                _cell = next((c for c in _cells
                              if c[0] == site[0] and c[1] == site[1]), None)
            else:
                _cell = min(_cells, key=lambda c: math.hypot(c[2], c[3]))
            if _cell is not None and site_bonded(wp, _cell[2], _cell[3]):
                raise ValueError("键合位 R%02dC%02d（%.4f, %.4f）已键合，"
                                 "禁止重复调平"
                                 % (_cell[0], _cell[1], _cell[2], _cell[3]))
    if not _head_dies(wp):
        raise ValueError("键合头上没有芯片（先执行芯片交接 flip_handoff）")
    ega_result = getattr(engine, "ega_result", None)
    if ega_result is None:
        raise ValueError("EGA 未完成（先执行 /api/process/wafer_ega 全局对准）")

    mc = engine.constants.get()
    fl = mc["focus_level"]
    fh = mc["flip_handoff"]
    fp = mc["flip_pick"]
    recipe = engine.recipe.get()
    fl_cfg = recipe.get("focus_level", {})

    as_z_chuck_focus = float(fl["as_z_chuck_focus_mm"])
    head_z_upfocus = float(fl["head_z_upfocus_mm"])
    cam_dx = float(fl["confocal_cam_dx_mm"])
    cam_dy = float(fl["confocal_cam_dy_mm"])
    level_tol = float(fl["level_tol_urad"])
    as_head = [float(v) for v in fl["as_head_center_xy_mm"]]

    t_w = float(recipe["wafer"]["thickness_mm"])
    t_d = float(recipe["film"]["dies"]["thickness_mm"])

    # 高度链进场公式（docs/调焦调平与高度链_V1.md 推导）
    as_z_entry = as_z_chuck_focus + t_w
    bs_entry = head_z_upfocus + (as_z_entry - as_z_chuck_focus) + t_d

    as_safe_x, as_safe_y = (float(v) for v in fh["as_safe_xy_mm"])
    head_z_safe_mm = float(fh["head_z_safe_mm"])
    fa_x_retreat_mm = float(fh["fa_x_retreat_mm"])
    fa_z_safe_mm = float(fp["fa_z_safe_mm"])

    n_pts = int(fl_cfg.get("measure_points", 4))
    if n_pts < 3 or n_pts > 8:
        raise ValueError("focus_level.measure_points 须在 3~8：%r" % (n_pts,))
    if align_first is None:
        align_first = bool(fl_cfg.get("align_first", False))

    # ---- 晶圆测点（设计坐标）----
    cells = wafer_sites_cells(recipe)
    if not cells:
        raise ValueError("处方无晶圆键合位（wafer_sites 为空）")
    sx, sy = wafer_sites_chip_size(recipe)
    sm = recipe["wafer"]["site_marks"]
    sm_ratio = sm["ratio"][:int(sm.get("per_site", len(sm["ratio"])))]

    if mode == "per_site":
        if site is not None:
            cell = next((c for c in cells
                         if c[0] == site[0] and c[1] == site[1]), None)
            if cell is None:
                raise ValueError("键合位 %s 不存在（处方共 %d 个）"
                                 % (site, len(cells)))
        else:
            cell = min(cells, key=lambda c: math.hypot(c[2], c[3]))
        cx0, cy0 = cell[2], cell[3]
        center_pt = (cx0, cy0)
        site_desc = "键合位 R%02dC%02d" % (cell[0], cell[1])
        if align_first and len(sm_ratio) < n_pts:
            raise ValueError("align_first 需要 wafer.site_marks 至少 %d 个"
                             " 标记点（当前 %d 个）" % (n_pts, len(sm_ratio)))
        local_wafer = _fl_n_points(n_pts, sx, sy,
                                   ratio=sm_ratio if align_first else None,
                                   scale=0.3)
        wafer_pts = [(round(cx0 + p[0], 6), round(cy0 + p[1], 6))
                     for p in local_wafer]
    elif mode == "global":
        center_pt = (0.0, 0.0)
        site_desc = "全晶圆 EGA 切蛋糕 3 点"
        # 迭代 5：整片晶圆视作大芯片——晶圆面固定 3 点（平面拟合最小点数）。
        # 选点来源：与 wafer_ega 流程同参数复算 EGA 站点，再取最大面积
        # 三角形（"切蛋糕"）；measure_points 不再影响 global 晶圆侧。
        ega_n = int(mc["wafer_ega"]["ega_n_points"])
        sel_all = ega.select_measure_points(cells, n_pts=ega_n)
        tri = _fl_pick_triangle(sel_all)
        if align_first:
            if not sm_ratio:
                raise ValueError("align_first 需要 wafer.site_marks 标记点"
                                 "（ratio 为空）")
            wafer_pts = []
            for sp in tri:
                cell = min(cells, key=lambda c: (c[2] - sp[0]) ** 2
                           + (c[3] - sp[1]) ** 2)
                mr = sm_ratio[0]
                wafer_pts.append((round(cell[2] + mr[0] * sx / 2.0, 6),
                                  round(cell[3] + mr[1] * sy / 2.0, 6)))
        else:
            wafer_pts = [tuple(p) for p in tri]
    else:  # off
        center_pt = (0.0, 0.0)
        site_desc = "off（仅中心点确认）"
        wafer_pts = []

    def to_aim(p):
        """晶圆设计点 → 对准系 aim 坐标（EGA 映射 + 共焦 dx/dy 补偿：
        探头对准 P 时相机光轴须对准 P−(cam_dx,cam_dy)——认知链）。"""
        mx, my = ega.apply_map(ega_result, p[0], p[1])
        return [round(mx - cam_dx, 6), round(my - cam_dy, 6)]

    # ---- die 测点（相对 die 中心；align_first=角标 film.marks 比例
    #      ×die 半宽，否则均布图案）----
    dsx = float(recipe["film"]["dies"]["size_x_mm"])
    dsy = float(recipe["film"]["dies"]["size_y_mm"])
    fm = recipe["film"]["marks"]
    fm_ratio = fm["ratio"][:int(fm.get("per_die", len(fm["ratio"])))]
    if align_first and len(fm_ratio) < n_pts:
        raise ValueError("align_first 需要 film.marks 至少 %d 个角标"
                         "（当前 %d 个）" % (n_pts, len(fm_ratio)))
    die_pts = _fl_n_points(n_pts, dsx, dsy,
                           ratio=fm_ratio if align_first else None,
                           scale=0.35)

    name = "调焦调平（process/focus_level %s%s%s）" % (
        mode, "+align_first" if align_first else "",
        "" if verify else "+no_verify")
    steps = [
        {"op": "log",
         "msg": "=== 调焦调平 开始：mode=%s（%s）；N=%d；align_first=%s；"
                "verify=%s；进场 AS_Z=%.4f，BS_Arm_Z=%.4f ==="
                % (mode, site_desc, n_pts, align_first, verify,
                   as_z_entry, bs_entry),
         "desc": "流程开始"},
    ]
    if mode == "global":
        steps.append({"op": "log",
                      "msg": "晶圆面测点=3 点（EGA 切蛋糕最大面积三角形）："
                             + "、".join("(%.3f, %.3f)" % (p[0], p[1])
                                         for p in wafer_pts)
                             + "；芯片面=%d 点" % n_pts,
                      "desc": "global 选点（EGA 站点切蛋糕 3 点）"})

    if mode == "per_site":
        # 键合位就位：送 site 到键合头吸盘中心正下方并冻结 WS
        steps.append({"op": "fl_ready",
                      "site_x": center_pt[0], "site_y": center_pt[1],
                      "desc": "键合位就位：site 送到键合头吸盘中心正下方，"
                              "WS 冻结（后续测高硅片台不动）"})
    else:
        # global / off：传统安全确认走位
        steps.extend([
            {"op": "move", "axis": "AS_X", "target": as_safe_x,
             "desc": "对准台 X 退安全位（as_safe_xy_mm，防与键合头/翻转干涉）"},
            {"op": "move", "axis": "AS_Y", "target": as_safe_y,
             "desc": "对准台 Y 退安全位"},
            {"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe_mm,
             "desc": "键合头到安全高度（head_z_safe_mm）"},
            {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
             "desc": "翻转 Z 到安全高度（fa_z_safe_mm）"},
            {"op": "move", "axis": "FA_X", "target": fa_x_retreat_mm,
             "desc": "翻转模块撤离（fa_x_retreat_mm）"},
        ])

    # 进场对焦高度
    steps.extend([
        {"op": "move", "axis": "AS_Z", "target": round(as_z_entry, 6),
         "desc": "进场：AS_Z → as_z_chuck_focus %.4f + 片厚 %.4f（下相机"
                 "焦面=晶圆面）" % (as_z_chuck_focus, t_w)},
        {"op": "move", "axis": "BS_Arm_Z", "target": round(bs_entry, 6),
         "desc": "进场：BS_Arm_Z → head_z_upfocus %.4f + AS_Z 差 %.4f + "
                 "die 厚 %.4f（上相机焦面=芯片下表面）"
                 % (head_z_upfocus, as_z_entry - as_z_chuck_focus, t_d)},
    ])

    # global 且有调平记忆：直接施加，跳到复测
    mem = getattr(engine, "level_memory", None)
    if mode == "global" and mem:
        steps.append({"op": "log",
                      "msg": "发现调平记忆（t=%.3f）：直接施加补偿，跳过测量"
                             % float(mem.get("t", 0.0)),
                      "desc": "施加 level 记忆"})
        steps.extend([
            {"op": "move", "axis": "BS_CAL_Rx",
             "target": round(float(mem["cal_rx"]), 9),
             "desc": "施加记忆：BS_CAL_Rx 调平"},
            {"op": "move", "axis": "BS_CAL_Ry",
             "target": round(float(mem["cal_ry"]), 9),
             "desc": "施加记忆：BS_CAL_Ry 调平"},
            {"op": "move", "axis": "AS_Z",
             "target": round(float(mem["as_z"]), 6),
             "desc": "施加记忆：AS_Z 定焦"},
            {"op": "move", "axis": "BS_Arm_Z",
             "target": round(float(mem["bs_arm_z"]), 6),
             "desc": "施加记忆：BS_Arm_Z 定焦"},
        ])
    elif mode != "off":
        # 晶圆面测高
        for i, p in enumerate(wafer_pts, 1):
            stp = {"op": "fl_measure", "probe": "down",
                   "space": "wafer",
                   "point": [round(p[0], 6), round(p[1], 6)],
                   "aim": to_aim(p),
                   "desc": "晶圆面测高 %d/%d：设计点 (%.3f, %.3f)%s"
                           % (i, len(wafer_pts), p[0], p[1],
                              "（先对准 site_mark）" if align_first else "")}
            if align_first:
                stp["align_first"] = True
            if mode == "per_site":
                stp["freeze_ws"] = True
            steps.append(stp)
        steps.append({"op": "fl_solve", "mode": "wafer",
                      "desc": "晶圆面平面拟合（世界系坡度）"})
        # 芯片面测高（die 空间以 as_head_center_xy_mm 为基准，头/WS 不动）
        for i, p in enumerate(die_pts, 1):
            stp = {"op": "fl_measure", "probe": "up",
                   "space": "die",
                   "point": [round(p[0], 6), round(p[1], 6)],
                   "desc": "芯片面测高 %d/%d：die %s (%+.3f, %+.3f)%s"
                           % (i, len(die_pts),
                              "角标" if align_first else "角点", p[0], p[1],
                              "（先对准 die 角标）" if align_first else "")}
            if align_first:
                stp["align_first"] = True
            steps.append(stp)
        steps.append({"op": "fl_solve", "mode": "die",
                      "desc": "芯片面平面拟合（世界系坡度）"})
        steps.append({"op": "fl_solve", "mode": "compensate",
                      "desc": "调平补偿：BS_CAL_Rx/Ry 使芯片面∥晶圆面 + "
                              "AS_Z/BS_Arm_Z 定焦（各调一次；残余验收阈值 "
                              "%.0f µrad）" % level_tol})

    if verify:
        if mode == "off":
            # off=只对焦的一次确认（保持原单点行为）
            steps.append({"op": "fl_measure", "probe": "down",
                          "space": "wafer",
                          "point": [round(center_pt[0], 6),
                                    round(center_pt[1], 6)],
                          "aim": to_aim(center_pt),
                          "desc": "复测：晶圆中心点共焦读数（应≈中点 "
                                  "%.0f µm）"
                                  % float(fl["confocal_mid_down_um"])})
            steps.append({"op": "fl_measure", "probe": "up", "space": "die",
                          "point": [0.0, 0.0],
                          "desc": "复测：芯片中心点共焦读数（应≈中点 "
                                  "%.0f µm）"
                                  % float(fl["confocal_mid_up_um"])})
        else:
            # per_site/global：调平后实测复测——同正式测量点位 N 点复测
            # 晶圆面/芯片面 → fl_verify 拟合判定残余 ≤ level_tol_urad。
            steps.append({"op": "log",
                          "msg": "=== 调平复测开始：晶圆面/芯片面各 %d 点"
                                 "（二次对准=%s）→ 残余阈值 %.0f µrad ==="
                                 % (len(wafer_pts), align_first, level_tol),
                          "desc": "调平复测段"})
            for i, p in enumerate(wafer_pts, 1):
                stp = {"op": "fl_measure", "probe": "down",
                       "space": "wafer_v",
                       "point": [round(p[0], 6), round(p[1], 6)],
                       "aim": to_aim(p),
                       "desc": "晶圆面复测 %d/%d：设计点 (%.3f, %.3f)%s"
                               % (i, len(wafer_pts), p[0], p[1],
                                  "（二次对准 site_mark）"
                                  if align_first else "")}
                if align_first:
                    stp["align_first"] = True
                if mode == "per_site":
                    stp["freeze_ws"] = True
                steps.append(stp)
            for i, p in enumerate(die_pts, 1):
                stp = {"op": "fl_measure", "probe": "up",
                       "space": "die_v",
                       "point": [round(p[0], 6), round(p[1], 6)],
                       "desc": "芯片面复测 %d/%d：die %s (%+.3f, %+.3f)%s"
                               % (i, len(die_pts),
                                  "角标" if align_first else "角点",
                                  p[0], p[1],
                                  "（二次对准 die 角标）"
                                  if align_first else "")}
                if align_first:
                    stp["align_first"] = True
                steps.append(stp)
            steps.append({"op": "fl_verify",
                          "desc": "调平验证：复测面拟合 + 残余阈值判定"
                                  "（level_tol %.0f µrad）" % level_tol})
    steps.append({"op": "log",
                  "msg": "=== 调焦调平 完成（mode=%s）===" % mode,
                  "desc": "流程完成"})
    return name, steps


# ---------------------------------------------------------------------------
# 逐键合位精对准（fine_align：双侧标记测量→拟合→WS 补偿→确认闭环；迭代 6）
# ---------------------------------------------------------------------------

def build_fine_align_flow(engine, site=None):
    """逐键合位精对准流程 -> (name, steps)。

    时序（用户需求，2026-08-28；机械常数=wafer_ega/fine_align 组键名；
    处方参数=recipe.fine_align）：
      1. 守卫：晶圆在位、本 site 未键合（已键合 die 遮挡标记，提前报
         清晰守卫错）、head 有 die、EGA 完成、本 site 已完成调焦调平
         补偿（eng.fl_fit["compensate"] 或 level_memory，否则报错提示先跑
         focus_level——精对准在调平好的焦面上测标记）；且
         eng.fl_ws_frozen 存在且 site 匹配（focus_level per_site 的
         fl_ready 已把 site 送到头下并冻结 WS——精对准信任链，不再重复
         走位；否则会 undo 调平补偿后的实测焦面）
      2. fa_measure（WS/BS 锁轴，AS-only 逐标记，双侧各 N 点；freeze_ws
         寻址基准=fl_ws_frozen）
         → fa_solve（双侧 fit → Δ=(tx,ty,θ)，算法=处方 fine_align.algorithm）
         → fa_compensate_confirm（先 Rz 后 XY 含阿贝耦合，EGA map 同步；
         确认环逐轮复测-重fit-残余判定）
      3. 对准台回安全位：AS_X/AS_Y → flip_handoff.as_safe_xy_mm

    测点：wafer.site_marks 与 film.marks 的前 N 个比例点（N=处方
    fine_align.measure_points，2~8，≤两侧可用标记数；默认 4 角点）。
    site 缺省=离晶圆心最近的键合位（同 focus_level per_site）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    if engine.vision is None:
        raise RuntimeError("精对准流程需要视觉系统（vision not attached）")
    wp = engine.workpieces
    with wp._lock:
        wafer_present = bool(wp.wafer["present"])
    if not wafer_present:
        raise ValueError("晶圆不在位（先上片：/api/process/wafer_ega）")
    # 已键合守卫优先于"head 无 die"（bond 后 head 必然空；已键合 die
    # 遮挡 site 标记，重跑只会报"视野内无标记"——提前给清晰守卫错）。
    # 进程内守卫链单测的伪 engine.recipe 不是 Recipe 实例，取不到处方时跳过
    try:
        _cells = wafer_sites_cells(engine.recipe.get())
    except (AttributeError, KeyError, TypeError):
        _cells = []
    if _cells:
        if site is not None:
            _cell = next((c for c in _cells
                          if c[0] == site[0] and c[1] == site[1]), None)
        else:
            _cell = min(_cells, key=lambda c: math.hypot(c[2], c[3]))
        if _cell is not None and site_bonded(wp, _cell[2], _cell[3]):
            raise ValueError("键合位 R%02dC%02d（%.4f, %.4f）已键合，"
                             "禁止重复对准"
                             % (_cell[0], _cell[1], _cell[2], _cell[3]))
    if not _head_dies(wp):
        raise ValueError("键合头上没有芯片（先执行芯片交接 flip_handoff）")
    if getattr(engine, "ega_result", None) is None:
        raise ValueError("EGA 未完成（先执行 /api/process/wafer_ega 全局对准）")
    fl_fit = getattr(engine, "fl_fit", None) or {}
    if not fl_fit.get("compensate") \
            and getattr(engine, "level_memory", None) is None:
        raise ValueError("本 site 尚未调焦调平（先执行 "
                         "/api/process/focus_level；精对准在调平好的焦面"
                         "上测标记）")

    mc = engine.constants.get()
    recipe = engine.recipe.get()
    fa_cfg = recipe.get("fine_align", {})

    n_pts = int(fa_cfg.get("measure_points", 4))
    if n_pts < 2 or n_pts > 8:
        raise ValueError("fine_align.measure_points 须在 2~8：%r" % (n_pts,))
    algo = fa_cfg.get("algorithm", "rigid")
    if algo == "affine" and n_pts < 3:
        raise ValueError("affine 算法需要 ≥3 测点（当前 "
                         "fine_align.measure_points=%d）" % n_pts)

    # ---- 目标 site（缺省=中心位，同 focus_level per_site）----
    cells = wafer_sites_cells(recipe)
    if not cells:
        raise ValueError("处方无晶圆键合位（wafer_sites 为空）")
    if site is not None:
        cell = next((c for c in cells
                     if c[0] == site[0] and c[1] == site[1]), None)
        if cell is None:
            raise ValueError("键合位 %s 不存在（处方共 %d 个）"
                             % (site, len(cells)))
    else:
        cell = min(cells, key=lambda c: math.hypot(c[2], c[3]))
    cx0, cy0 = cell[2], cell[3]
    site_desc = "键合位 R%02dC%02d" % (cell[0], cell[1])

    # ---- 信任链守卫：focus_level per_site 已把本 site 送到头下并冻结 WS
    #      （fl_ws_frozen 由 fl_ready 落存，fa_measure 的 freeze_ws 寻址
    #      基准）；不在则报错提示先跑 focus_level per_site ----
    frozen = getattr(engine, "fl_ws_frozen", None)
    if not frozen or list(frozen.get("site") or []) != \
            [round(cx0, 6), round(cy0, 6)]:
        raise ValueError("键合位未就位/WS 未冻结（先执行 "
                         "/api/process/focus_level mode=per_site 完成键合位"
                         "就位+WS 冻结；精对准信任链不再重复进场走位）")

    # ---- 双侧测点（比例×半宽；两侧各自取前 N 个，点数一致）----
    sx, sy = wafer_sites_chip_size(recipe)
    sm = recipe["wafer"]["site_marks"]
    sm_ratio = sm["ratio"][:int(sm.get("per_site", len(sm["ratio"])))]
    fm = recipe["film"]["marks"]
    fm_ratio = fm["ratio"][:int(fm.get("per_die", len(fm["ratio"])))]
    if len(sm_ratio) < n_pts or len(fm_ratio) < n_pts:
        raise ValueError("精对准需要 wafer.site_marks 与 film.marks 各至少 "
                         "%d 个标记（当前 %d/%d 个）"
                         % (n_pts, len(sm_ratio), len(fm_ratio)))
    dsx = float(recipe["film"]["dies"]["size_x_mm"])
    dsy = float(recipe["film"]["dies"]["size_y_mm"])
    wafer_pts = [[round(r[0] * sx / 2.0, 6), round(r[1] * sy / 2.0, 6)]
                 for r in sm_ratio[:n_pts]]
    die_pts = [[round(r[0] * dsx / 2.0, 6), round(r[1] * dsy / 2.0, 6)]
               for r in fm_ratio[:n_pts]]

    # 对准台安全位（流程末尾回退用）
    as_safe_x, as_safe_y = (float(v) for v in
                            mc["flip_handoff"]["as_safe_xy_mm"])

    name = "逐键合位精对准（process/fine_align %s）" % site_desc
    steps = [
        {"op": "log",
         "msg": "=== 精对准 开始：%s（site 心 %.3f,%.3f）；N=%d；算法=%s；"
                "确认=%s（阈值 %.2f µm / %.0f µrad，≤%d 轮，超次 %s）==="
                % (site_desc, cx0, cy0, n_pts, algo,
                   fa_cfg.get("confirm_enable", True),
                   float(fa_cfg.get("confirm_tol_xy_um", 0.15)),
                   float(fa_cfg.get("confirm_tol_rz_urad", 30.0)),
                   int(fa_cfg.get("confirm_max_iter", 3)),
                   fa_cfg.get("on_unconfirmed", "abort")),
         "desc": "流程开始"},
        {"op": "fa_measure", "site_x": cx0, "site_y": cy0,
         "wafer_pts": wafer_pts, "die_pts": die_pts,
         "desc": "精对准测量：WS/BS 锁轴，AS-only 逐标记，晶圆侧/die 侧"
                 "各 %d 点" % n_pts},
        {"op": "fa_solve",
         "desc": "精对准解算：双侧 %s 拟合 → Δ=(tx,ty,θ)" % algo},
        {"op": "fa_compensate_confirm", "site_x": cx0, "site_y": cy0,
         "wafer_pts": wafer_pts, "die_pts": die_pts,
         "desc": "补偿+确认：先 Rz 后 XY（含阿贝耦合，EGA map 同步）→ "
                 "逐轮复测-重fit-残余判定"},
        {"op": "move", "axis": "AS_X", "target": as_safe_x,
         "desc": "精对准完成，对准台 X 回安全位（as_safe_xy_mm）"},
        {"op": "move", "axis": "AS_Y", "target": as_safe_y,
         "desc": "精对准完成，对准台 Y 回安全位（退出键合头下方）"},
        {"op": "log",
         "msg": "=== 精对准 完成：结果见 flow.state.fine_align_result ===",
         "desc": "流程完成"},
    ]
    return name, steps


# ---------------------------------------------------------------------------
# 热压键合（process/bond TCB；迭代 8：PID 温控/压控三回路）
# ---------------------------------------------------------------------------

def build_bond_flow(engine, site=None):
    """热压键合流程 -> (name, steps)。

    时序（用户工艺，2026-08-28；机械常数=bond/flip_handoff 组键名；
    处方参数=recipe.bond）：
      1. 守卫（生成时刻）：晶圆在位、EGA 完成、本 site fine_align
         judge=pass、本 site 未键合过（workpieces dies 查 carrier=wafer
         且 site==该位设计坐标）、head 有 die（顺序在"已键合"之后——
         重复键合报更具体的错）；流程内再幂等开晶圆真空
      2. 硅片台陶瓷加热片等温确认（recipe bond.wafer_preheat：平时常开
         保温，流程只做"等到温"幂等确认；已到温秒过）
      3. 脉冲加热器升温到 bond.head_temp_C（±2℃ 进带）
      4. 两段速下压：快速段 BS_Arm_Z → 寻力起始位（机械常数
         bond.head_z_search_mm + 片厚换算，处方 fast_speed_mm_s）→
         force_search 慢速寻力段（search_speed_mm_s 下行读 oActualForce，
         ≥touch_force_N 停轴记实际接触位；超程下限=有效接触位−
         search_overtravel_mm 未触力→FAIL"寻力未接触"）
      5. 加压闭环：force op 目标 bond.force_N——力回路 PID 调 BS_Arm_Z
         压入量，真值力到 ±force_tol_N 带
      6. 保压保温：delay bond.dwell_s（力/温双保持）
      7. 释放：force→0 + 键合头三吸盘 OpenPositive→OpenAir 吹离（沿用
         交接吹离序列）→ wait_die carrier=wafer（head_to_wafer 沿触发）
         → bond_mark 落位修正：die.site 改写为本键合位设计坐标
         （transfers.json 的 [0,0] 占位语义在流程侧落笔）
      8. BS_Arm_Z 抬 head_z_safe → 脉冲加热器按 recipe head_cool_after
         决定关/保温 → 完成

    信任链：fine_align 完成时本 site 已在键合头吸盘中心正下方且 WS
    冻结/对准台回安全位，bond 不再重复走位。site 缺省=离晶圆心最近的
    键合位（同 focus_level/fine_align）。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    wp = engine.workpieces
    with wp._lock:
        wafer_present = bool(wp.wafer["present"])
    if not wafer_present:
        raise ValueError("晶圆不在位（先上片：/api/process/wafer_ega）")
    if getattr(engine, "ega_result", None) is None:
        raise ValueError("EGA 未完成（先执行 /api/process/wafer_ega 全局对准）")

    mc = engine.constants.get()
    recipe = engine.recipe.get()
    bond_mc = mc["bond"]
    fh = mc["flip_handoff"]
    bcfg = recipe["bond"]

    # ---- 目标 site（缺省=中心位，同 focus_level per_site）----
    cells = wafer_sites_cells(recipe)
    if not cells:
        raise ValueError("处方无晶圆键合位（wafer_sites 为空）")
    if site is not None:
        cell = next((c for c in cells
                     if c[0] == site[0] and c[1] == site[1]), None)
        if cell is None:
            raise ValueError("键合位 %s 不存在（处方共 %d 个）"
                             % (site, len(cells)))
    else:
        cell = min(cells, key=lambda c: math.hypot(c[2], c[3]))
    cx0, cy0 = cell[2], cell[3]
    site_desc = "键合位 R%02dC%02d" % (cell[0], cell[1])

    # ---- 守卫：本 site fine_align judge=pass ----
    fa = getattr(engine, "fine_align_result", None) or {}
    if fa.get("judge") != "pass" \
            or [round(v, 6) for v in (fa.get("site") or [])] != \
            [round(cx0, 6), round(cy0, 6)]:
        raise ValueError("本 site 精对准未通过（先执行 "
                         "/api/process/fine_align site=%s 并 judge=pass）"
                         % site_desc)
    # ---- 守卫：本 site 未键合过（site_bonded 容差匹配，统一工具函数）----
    if site_bonded(wp, cx0, cy0):
        raise ValueError("键合位 %s（%.4f, %.4f）已键合，禁止重复键合"
                         % (site_desc, cx0, cy0))
    # ---- 守卫：head 有 die（顺序在"已键合"之后——重复键合报更具体的
    #      错误；chain 的 bond 跳过判定同语义：head 无 die=异常报错）----
    if not _head_dies(wp):
        raise ValueError("键合头上没有芯片（先执行芯片交接 flip_handoff）")

    head_z_contact = float(bond_mc["head_z_contact_mm"])
    head_z_search = float(bond_mc.get("head_z_search_mm",
                                      head_z_contact + 2.0))
    head_z_safe = float(fh["head_z_safe_mm"])
    vac_timeout = float(fh["handoff_vac_timeout_s"])
    force_n = float(bcfg["force_N"])
    force_tol = float(bcfg["force_tol_N"])
    dwell_s = float(bcfg["dwell_s"])
    head_temp = float(bcfg["head_temp_C"])
    wafer_temp = float(bcfg["wafer_temp_C"])
    wafer_preheat = bool(bcfg["wafer_preheat"])
    head_cool_after = bool(bcfg["head_cool_after"])
    fast_speed = float(bcfg["fast_speed_mm_s"])
    search_speed = float(bcfg["search_speed_mm_s"])
    touch_force = float(bcfg["touch_force_N"])
    overtravel = float(bcfg["search_overtravel_mm"])
    # 接触确认容差（2026-09-04 迭代 13，机械常数 bond 组）：实际接触位
    # vs 期望有效接触位超差 → FAIL"接触位异常"
    contact_tol_um = float(bond_mc.get("contact_verify_tol_um", 50.0))
    # 有效接触位/寻力位 = 裸值 + 晶圆厚 + 芯片厚（2026-08-31 迭代 9；
    # 与 engine 真值力模型共用 effective_bond_contact_mm，符号：片变厚→
    # 接触位上移/pos 增大）
    wafer_t = float((recipe.get("wafer") or {}).get("thickness_mm", 0.0))
    chip_t = float(((recipe.get("film") or {}).get("dies") or {})
                   .get("thickness_mm", 0.0))
    eff_contact = effective_bond_contact_mm(head_z_contact, wafer_t, chip_t)
    eff_search = effective_bond_contact_mm(head_z_search, wafer_t, chip_t)
    search_floor = eff_contact - overtravel

    name = "热压键合（process/bond TCB %s）" % site_desc
    steps = [
        {"op": "log",
         "msg": "=== 热压键合 开始：%s（site 心 %.4f, %.4f）；力 %.1f±%.1fN "
                "保压 %.1fs；头温 %.0f℃ 台温 %.0f℃ ==="
                % (site_desc, cx0, cy0, force_n, force_tol, dwell_s,
                   head_temp, wafer_temp),
         "desc": "流程开始"},
    ]
    # 1b. 晶圆真空幂等确认（守卫：晶圆在位+真空开）
    steps.extend(_wafer_vac_steps(vac_timeout))
    # 1c. 键合附加补偿（2026-09-05 迭代 15，机械常数 fine_align.
    #     bond_offset_*，默认 0=跳过）：精对准完成后/下压前施加 WS 附加
    #     补偿——在 fl_ready 施加会被精对准补偿环吸收（fa 每次重测 site
    #     标记重算 WS 补偿，预偏移净效应为零），只有 fa 之后的偏移才留
    #     在 die 落点里（measured mean→0）
    steps.append({"op": "bond_offset_apply",
                  "desc": "键合附加补偿（fine_align.bond_offset_*，默认 0 "
                          "跳过；非 0 时 WS 设计系 +offset/+orz，EGA map "
                          "同步，die 落点相对 site 标记 −offset）"})
    # 2. 硅片台等温确认（常开保温；wafer_preheat=false 时只对当前目标等温）
    if wafer_preheat:
        steps.append({"op": "heater", "name": "Heat_Dw", "action": "set",
                      "temp": wafer_temp, "tol_c": 2.0, "timeout": 900.0,
                      "desc": "硅片台陶瓷加热片等温确认（处方 wafer_temp_C "
                              "%.0f℃；平时常开保温，已到温幂等秒过）"
                              % wafer_temp})
    else:
        steps.append({"op": "heater", "name": "Heat_Dw", "action": "wait",
                      "tol_c": 2.0, "timeout": 900.0,
                      "desc": "硅片台等温确认（wafer_preheat=false：只对"
                              "当前目标等到温）"})
    # 3. 脉冲加热器升温到带
    steps.append({"op": "heater", "name": "PulseHeat_Up", "action": "set",
                  "temp": head_temp, "tol_c": 2.0, "timeout": 120.0,
                  "desc": "脉冲加热器升温（处方 head_temp_C %.0f℃，±2℃ 进带）"
                          % head_temp})
    # 4. 两段速下压 + 慢速寻力（2026-08-31 迭代 9：替代旧一步慢速长行程，
    #    修复"等待 BS_Arm_Z 到位超时(30s)"）
    steps.append({"op": "move", "axis": "BS_Arm_Z", "target": eff_search,
                  "speed": fast_speed,
                  "desc": "快速下压段：BS_Arm_Z → 寻力起始位 %.2f（裸值 %.2f"
                          "+片厚 %.2f；fast_speed %.1f mm/s）"
                          % (eff_search, head_z_search, wafer_t + chip_t,
                             fast_speed)})
    steps.append({"op": "force_search", "axis": "BS_Arm_Z",
                  "speed": search_speed, "touch_force_N": touch_force,
                  "min_pos": search_floor, "timeout": 120.0,
                  "eff_contact": eff_contact,
                  "contact_verify_tol_um": contact_tol_um,
                  "desc": "慢速寻力段：%.2f mm/s 下行读 oActualForce，≥%.1fN "
                          "停轴记实际接触位（有效接触位 %.2f，超程下限 %.2f "
                          "未触力→FAIL；接触确认容差 ±%.0fµm 超差→FAIL）"
                          % (search_speed, touch_force, eff_contact,
                             search_floor, contact_tol_um)})
    # 4b. 键合接触时刻真值采样（2026-09-07 迭代 20）：触力停轴后、加压/
    #     保压/落片前，用 6DOF 真值位姿走精对准同一 fit 管线算 die↔site
    #     真值残差——键合头下降链（BS_Arm_Z 地图差系统项+阿贝项）自此
    #     进入 truth；写 eng.bond_commit 供 bond_mark/bond_results 消费
    steps.append({"op": "bond_truth",
                  "desc": "键合接触时刻真值采样（迭代 20：6DOF 真值走 fa "
                          "同一 fit 管线，下降链进 truth；写 eng."
                          "bond_commit）"})
    # 5. 加压闭环到带（寻力停点即力闭环起点）
    steps.append({"op": "force", "target": force_n, "tol_n": force_tol,
                  "timeout": 30.0,
                  "desc": "加压闭环：力回路 PID 调压入量，真值力到 %.1f±%.1fN"
                          % (force_n, force_tol)})
    # 6. 保压保温
    steps.append({"op": "delay", "ms": int(round(dwell_s * 1000)),
                  "desc": "保压保温 %.1fs（力/温双保持）" % dwell_s})
    # 7. 释放 + 吹离 + 交接确认 + 落位修正
    steps.append({"op": "force", "target": 0.0, "tol_n": force_tol,
                  "timeout": 30.0,
                  "desc": "卸力：力 setpoint 回 0（PID 抬头卸压）"})
    steps.extend([
        {"op": "io", "path": HEAD_VAC_PATHS[0], "cmd": "OpenPositive",
         "desc": "键合头吸盘 1 正压吹离"},
        {"op": "io", "path": HEAD_VAC_PATHS[1], "cmd": "OpenPositive",
         "desc": "键合头吸盘 2 正压吹离"},
        {"op": "io", "path": HEAD_VAC_PATHS[2], "cmd": "OpenPositive",
         "desc": "键合头吸盘 3 正压吹离（oNegtiveState 落 → "
                 "head_to_wafer 沿触发）"},
        {"op": "delay", "ms": 200,
         "desc": "正压保持 200ms（吹离建立）"},
        {"op": "io", "path": HEAD_VAC_PATHS[0], "cmd": "OpenAir",
         "desc": "键合头吸盘 1 回大气"},
        {"op": "io", "path": HEAD_VAC_PATHS[1], "cmd": "OpenAir",
         "desc": "键合头吸盘 2 回大气"},
        {"op": "io", "path": HEAD_VAC_PATHS[2], "cmd": "OpenAir",
         "desc": "键合头吸盘 3 回大气"},
        {"op": "wait_die", "carrier": "wafer", "vacated": ["head"],
         "timeout": 5.0,
         "err": "键合失败：吸盘已释放但 die 未交接（carrier 未变 wafer，"
                "transfers.json head_to_wafer 条件未触发）",
         "desc": "确认 die 已键合 carrier=wafer 且键合头腾空"},
        {"op": "bond_mark", "site_x": cx0, "site_y": cy0,
         "desc": "die 落位修正：site 改写为本键合位设计坐标 (%.4f, %.4f)"
                 "（transfers.json [0,0] 占位流程侧落笔）" % (cx0, cy0)},
    ])
    # 8. 抬头 + 脉冲加热器关/保温
    steps.append({"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe,
                  "desc": "键合臂抬安全高度（head_z_safe_mm %.2f）"
                          % head_z_safe})
    if head_cool_after:
        steps.append({"op": "heater", "name": "PulseHeat_Up", "action": "off",
                      "desc": "脉冲加热器关闭（head_cool_after=true，"
                              "自然降温）"})
    else:
        steps.append({"op": "log",
                      "msg": "脉冲加热器保温（head_cool_after=false）",
                      "desc": "脉冲加热器保温"})
    steps.append({"op": "log",
                  "msg": "=== 热压键合 完成：%s die 已在晶圆 site "
                         "(%.4f, %.4f) ===" % (site_desc, cx0, cy0),
                  "desc": "流程完成"})
    return name, steps


# ---------------------------------------------------------------------------
# 键合后精度检测（post_inspect：降温 → 安全检查 → 逐已键合 site 下相机
# 红外套刻两次独立触发复测（迭代 20：先 quad=site 标记模板后 cross=die
# 角标模板，同机位不动轴）→ measured 回填 bond_results；迭代 11 第二棒
# 新增，迭代 15 重构为套刻相对测量）
# ---------------------------------------------------------------------------

def _bonded_die_at(wp, cx, cy, tol=0.001):
    """site 设计坐标 (cx,cy) 上已键合的 die（carrier=wafer 且 site 容差
    匹配，同 site_bonded 口径）；无 → None。"""
    with wp._lock:
        for d in wp.dies:
            if d["carrier"] == "wafer" and d["site"] is not None \
                    and abs(d["site"][0] - cx) <= tol \
                    and abs(d["site"][1] - cy) <= tol:
                return dict(d)
    return None


def build_post_inspect_flow(engine, sites=None):
    """键合后精度检测流程 -> (name, steps)。

    时序（2026-09-02 迭代 11；**2026-09-05 迭代 15 重构：红外套刻**；
    **2026-09-07 迭代 20：两次独立触发**。机械常数=wafer_ega/flip_handoff/
    focus_level 组键名；处方参数=recipe.post_inspect）：
      1. 降温：wafer_keep_warm 开着时先显式停保温（heater off =
         StopTargetTempWithPlan，否则保温回路会把温度拉回 wafer_temp_C），
         再设 Heat_Dw → post_inspect.cool_target_C 等温（降温方向走
         tuning bond_hw.wafer_cool_rate_cps 主动冷却，默认 5℃/s）
      2. 安全检查（复用 wafer_ega/focus_level 安全位模式）：键合头
         BS_Arm_Z → head_z_safe_mm、翻转 FA_Z 安全位 + FA_X 撤离
      3. AS_Z 对焦：**晶圆面**（下芯片表面）= as_z_chuck_focus + 晶圆厚
         ——红外通道穿透一个芯片厚度，焦面=晶圆面时 site quad 标记与
         die cross 角标在同一画幅都合焦（IR 假设，迭代 15；旧方案对焦
         die 顶面=晶圆面+die厚，用户实测离焦）
      4. 逐已键合 site：pi_measure（每 site **一次走位、两次独立触发**
         ——AS 把 site 心送到下相机光轴（EGA map 寻址，超 AS 行程联动
         WS，迭代 11c），红外套刻通道先 quad 模板后 cross 模板各识别
         一次（quad=晶圆 site 标记 / cross=die 角标；迭代 15 的一次
         双模板列表契约保留但流程不再用），双组点集各自拟合后取组间
         刚体偏差=measured dx/dy/drz → BondResults.update_measured。
         EGA 残差/相机位姿系统差/逐标记 AS 重复性全部对消，不再逐标记
         走位拍照）
      5. 对准台 AS_X/AS_Y 回安全位、AS_Z 回焦面零位

    sites：可选 [[row,col],...] 子集（None/空=全部已键合位）；子集中有
    未键合位/未知位 → ValueError（API 层映射 400）。
    守卫：post_inspect.enabled=false / 晶圆不在位 / EGA 未完成 / 视觉未
    挂载 / 无已键合 die → ValueError/RuntimeError。
    """
    if engine.constants is None or engine.recipe is None \
            or engine.workpieces is None:
        raise RuntimeError("process_flows 需要 engine.constants/recipe/"
                           "workpieces（main.py 启动顺序保证）")
    if engine.vision is None:
        raise RuntimeError("键合后检测流程需要视觉系统（vision not attached）")
    wp = engine.workpieces
    with wp._lock:
        wafer_present = bool(wp.wafer["present"])
    if not wafer_present:
        raise ValueError("晶圆不在位（先上片：/api/process/wafer_ega）")
    if getattr(engine, "ega_result", None) is None:
        raise ValueError("EGA 未完成（先执行 /api/process/wafer_ega 全局对准）")

    mc = engine.constants.get()
    recipe = engine.recipe.get()
    pi = recipe.get("post_inspect") or {}
    if not bool(pi.get("enabled", True)):
        raise ValueError("键合后检测已停用（处方 post_inspect.enabled="
                         "false）")
    cool_target = float(pi.get("cool_target_C", 25.0))
    keep_warm = bool((recipe.get("bond") or {}).get("wafer_keep_warm", False))

    cells = wafer_sites_cells(recipe)
    if sites:
        targets = []
        for s in sites:
            cell = next((c for c in cells
                         if c[0] == int(s[0]) and c[1] == int(s[1])), None)
            if cell is None:
                raise ValueError("键合位 %s 不存在（处方共 %d 个）"
                                 % (s, len(cells)))
            if not site_bonded(wp, cell[2], cell[3]):
                raise ValueError("键合位 R%02dC%02d（%.4f, %.4f）未键合，"
                                 "无可检测 die"
                                 % (cell[0], cell[1], cell[2], cell[3]))
            targets.append(cell)
    else:
        targets = [c for c in cells if site_bonded(wp, c[2], c[3])]
    if not targets:
        raise ValueError("晶圆上无已键合 die（先执行键合流程）")

    # die 标记测点（同 fine_align die 侧：film.marks 比例 × die 半宽）
    dsx = float(recipe["film"]["dies"]["size_x_mm"])
    dsy = float(recipe["film"]["dies"]["size_y_mm"])
    fm = recipe["film"]["marks"]
    fm_ratio = fm["ratio"][:int(fm.get("per_die", len(fm["ratio"])))]
    if len(fm_ratio) < 2:
        raise ValueError("键合后检测需要 film.marks 至少 2 个角标（当前 "
                         "%d 个）" % len(fm_ratio))
    die_pts = [[round(r[0] * dsx / 2.0, 6), round(r[1] * dsy / 2.0, 6)]
               for r in fm_ratio]

    we = mc["wafer_ega"]
    fh = mc["flip_handoff"]
    fp = mc["flip_pick"]
    fl = mc["focus_level"]
    head_z_safe_mm = float(fh["head_z_safe_mm"])
    fa_x_retreat_mm = float(fh["fa_x_retreat_mm"])
    fa_z_safe_mm = float(fp["fa_z_safe_mm"])
    as_safe_x, as_safe_y = (float(v) for v in fh["as_safe_xy_mm"])
    t_w = float(recipe["wafer"]["thickness_mm"])
    # 对焦面=晶圆面（下芯片表面）：红外通道穿透一个芯片厚度，焦面=晶圆
    # 面时 site quad 与 die cross 同画幅都合焦（IR 假设，迭代 15；旧方案
    # 对焦 die 顶面=晶圆面+die厚，用户实测离焦）
    as_z_wafer = float(fl["as_z_chuck_focus_mm"]) + t_w

    name = "键合后精度检测（process/post_inspect %d sites）" % len(targets)
    steps = [
        {"op": "log",
         "msg": "=== 键合后精度检测 开始：%d 个已键合 site；降温目标 "
                "%.0f℃（主动冷却）；下相机红外透视复测 die 标记 ==="
                % (len(targets), cool_target),
         "desc": "流程开始"},
    ]
    # 1. 降温：保温开着须先显式停（否则 SetTargetTempWithPlan 后保温计划
    #    仍把温度拉回 wafer_temp_C）
    if keep_warm:
        steps.append({"op": "heater", "name": "Heat_Dw", "action": "off",
                      "desc": "先停硅片台保温（wafer_keep_warm=true："
                              "StopTargetTempWithPlan，否则保温会拉回 "
                              "wafer_temp_C）"})
    steps.append({"op": "heater", "name": "Heat_Dw", "action": "set",
                  "temp": cool_target, "tol_c": 2.0, "timeout": 900.0,
                  "desc": "硅片台主动降温到检测温度 %.0f℃（tuning "
                          "wafer_cool_rate_cps 速率，默认 5℃/s）"
                          % cool_target})
    # 2. 安全检查（同 wafer_ega 安全位模式）
    steps.extend([
        {"op": "move", "axis": "BS_Arm_Z", "target": head_z_safe_mm,
         "desc": "键合头到安全高度（head_z_safe_mm）"},
        {"op": "move", "axis": "FA_Z", "target": fa_z_safe_mm,
         "desc": "翻转 Z 到安全高度（fa_z_safe_mm）"},
        {"op": "move", "axis": "FA_X", "target": fa_x_retreat_mm,
         "desc": "翻转模块撤离（fa_x_retreat_mm）"},
    ])
    # 3. AS_Z 对焦到晶圆面（下芯片表面；红外套刻焦面，迭代 15）
    steps.append({"op": "move", "axis": "AS_Z", "target": round(as_z_wafer, 6),
                  "desc": "AS_Z 对焦：下相机焦面=晶圆面（as_z_chuck_focus"
                          " %.4f + 晶圆厚 %.4f；红外套刻通道 IR 穿透一个 die "
                          "厚，quad/cross 同画幅都合焦，迭代 15）"
                          % (float(fl["as_z_chuck_focus_mm"]), t_w)})
    # 4. 逐已键合 site 复测（每 site 一次拍照双模板套刻）
    for cell in targets:
        r, c, cx0, cy0 = cell
        die = _bonded_die_at(wp, cx0, cy0)
        die_id = die["id"] if die else None
        if die_id is None:
            raise ValueError("键合位 R%02dC%02d（%.4f, %.4f）找不到已键合 "
                             "die 记录" % (r, c, cx0, cy0))
        steps.append({"op": "pi_measure", "site_x": cx0, "site_y": cy0,
                      "site": [r, c],
                      "die_id": die_id, "die_pts": die_pts,
                      "desc": "检测 site R%02dC%02d die %s：AS 送 site 心到"
                              "下相机光轴（超行程联动 WS），红外套刻一张图"
                              "双模板（quad=site 标记/cross=die 角标各 %d "
                              "个），双组拟合取组间刚体偏差回填 measured"
                              "（系统差对消，迭代 15）"
                              % (r, c, die_id, len(die_pts))})
    # 5. 对准台回安全位
    steps.extend([
        {"op": "move", "axis": "AS_X", "target": as_safe_x,
         "desc": "检测完成，对准台 X 回安全位（as_safe_xy_mm）"},
        {"op": "move", "axis": "AS_Y", "target": as_safe_y,
         "desc": "对准台 Y 回安全位"},
        {"op": "move", "axis": "AS_Z", "target": 0.0,
         "desc": "AS_Z 回焦面零位（下相机景深中心）"},
        {"op": "log",
         "msg": "=== 键合后精度检测 完成：measured 已回填 "
                "/api/bond/results ===",
         "desc": "流程完成"},
    ])
    return name, steps


def maybe_auto_post_inspect(engine, after):
    """单片/全片键合结束后按处方 post_inspect.auto_after_* 自动触发检测
    （迭代 11 第二棒）。after ∈ ("single", "full")。
    流程忙（理论不发生：runner 收尾时 flow 已终态）/开关关/无已键合 die
    → 跳过；异常只打印不阻断调用方（runner 线程收尾路径）。"""
    try:
        pi = (engine.recipe.get() or {}).get("post_inspect") or {}
        if not pi.get("enabled", True):
            return False
        flag = "auto_after_single" if after == "single" \
            else "auto_after_full"
        if not pi.get(flag, False):
            return False
        fl = getattr(engine, "flow", None)
        if fl is None or fl.state in ("running", "paused"):
            print("[process] 自动键合后检测跳过：流程忙（%s）"
                  % (getattr(fl, "state", None),), flush=True)
            return False
        name, steps = build_post_inspect_flow(engine)
        fl.run_steps(steps, name)
        print("[process] %s 后自动触发键合后精度检测（%d 步）"
              % (after, len(steps)), flush=True)
        return True
    except Exception as e:
        print("[process] 自动键合后检测未启动：%s" % e, flush=True)
        return False
