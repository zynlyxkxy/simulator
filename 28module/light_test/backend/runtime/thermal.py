# -*- coding: utf-8 -*-
"""thermal.py — 组件级热仿真（2026-09-09 迭代 23）。

集总参数热网络（LPTN，与误差地图/6DOF 层/模态层正交、独立开关，
**默认 OFF**）——功率-热容-热阻真网络，能量守恒，τ 从物性涌现：

- 节点归并：前端模型 168 组件按 parentId 子树根归并 21 个热节点
  （groups 默认表见下，thermal.json 可覆盖）：花岗岩基座拆上/下 2 节点、
  键合臂横梁/平台架拆 2 节点（分裂节点对温差→一阶弯曲）；组件材料按
  material_map→父级继承→兜底 ss_304；m=ρV、C=m·cp 逐组分求和，
  CTE/λ 按质量加权（工程近似）。体积来自 ComponentCatalog 的几何估算
  （包围盒近似，镂空件高估——已知边界）；表面积由节点包围尺寸
  2(lw+lh+wh)×area_factor 估算。
- 边：links 邻接导热 G=kc·λ_eff·A_contact/L（λ_eff=两节点调和均值，
  A_contact=min(两节点表面积)×0.25，L=两节点 V^(1/3) 均值；kc=接触系数
  默认 0.3，均可逐边覆盖）；每节点对环境对流 G_conv=h·A（h 默认
  5 W/m²K 自然对流）。
- 功率源（W，真能量守恒）：加热器 power%×rated_w（PulseHeat_Up 300W/
  Heat_Dw 500W 默认；降温方向（active 且 cool_rate 且目标<当前）按
  −C_node×cool_rate 等效提取功率）；电机损耗按轴组活动度
  P=P_fl·Σ(0.65(v/vmax)²+0.35(a/amax)²) 30s 低通（轴组=硅片台/对准台/
  键合臂/翻转/蓝膜，P_fl 默认 40/20/15/20/15W）；电子件恒定 100W 入
  electronics 虚拟节点；环境 env.base_c（默认 22℃）±波动档（off/
  fine±0.05/typical±0.1/coarse±0.5℃ 正弦 600s，可选随机游走）。
- 锚定：wafer_chuck/bond_head 节点与 heater.cur 大电导 G_anchor 锚定
  （Heat_Dw 2000/PulseHeat_Up 300 W/K）——heater.cur 是温控环真值，
  热网络单向消费 power%、不反驱温控环；锚定流代表加热器与节点本体间
  的换热（节点温度≈heater.cur+滞后，滞后期=C/G_anchor）。功率注入只算
  加热方向（power%×rated_w）；冷却方向（active 且 cool_rate 且目标<
  当前）注入为 0，冷却完全由锚定追踪 heater.cur 下行实现（等效提取
  功率=锚定回流，state 的 anchor_flow_w 可见），不双重计算。
- 变形基准：ΔT=T−deform_ref_c（默认 25℃=加热器环境/机器冷态标定基准
  bond.ambient_C）；env.base_c 默认同为 25℃（冷态自洽：热平衡后结构
  节点 ΔT≈0，仅 keep-warm 吸盘 +5℃ 等真热态效应）；要模拟实验室 22℃
  场景 PUT env.base_c=22——此时结构相对基准的 −3℃ 收缩是真实物理效应
  （机器搬进更冷实验室确实收缩），不是幻象。modal 热软化 ΔT 仍按 modal
  自己的 REF_TEMP_C=22℃（group_delta_t 口径，与 OFF 回退 heater.cur
  −22℃ 数值一致）。
- 解算：半隐式 Euler（自耦合后向、邻接前向，无条件稳定）；构建期算
  显式稳定 dt=min(C/G_tot) 作参考，热 tick 自适应分频 dt_th=
  clamp(0.25×min_dt, 0.05, 0.5)s（2~20Hz）挂 1kHz 主循环，调用预算
  ≤50µs（21 节点纯算术）。
- 变形：节点 ΔT=T−deform_ref_c（默认 25℃，见"变形基准"条）；膨胀
  ΔL=CTE·ΔT·L_char；分裂节点对温差→一阶弯曲角 θ=CTE·(T1−T2)·L/h
  （杠杆平移 θ×lever）。deform_map 默认五条 6DOF 化映射：花岗岩→
  wafer_stage XY scale_ppm+漂移+摇摆、吸盘→WS 轴 scale（die 间距等效）、
  横梁→bond_head X 偏移+弯曲、键合头→Z 热伸长（向下为负）、对准体→
  align_stage 相机光轴横漂。注入路径：error_kinematics.compute() 模块
  6DOF 叠加 + error_system.actual()/sample6() scale_ppm 分量（modal
  先例，两系统开关正交）。
- 耦合：modal._fn_eff 的 ΔT 源——本系统 ON 时读热节点（head→bond_head、
  wafer→wafer_chuck），OFF 回退 heater.cur（现状不动）。
- 开关：POST /api/thermal{"enabled"} 热切换，运行时内存态不持久化、
  默认 OFF；OFF 时 tick 零开销早退、变形/scale 零注入，**节点温度状态
  保持**（再 ON 续积，与 modal OFF 清零语义不同——热惯性本就当保持）。
- 参数落盘 runtime/thermal.json（tmp+os.replace 原子写，同 modal.py
  惯例）；PUT 深合并→全量校验→构建预检→原子落盘→热替换+状态重置。
- 状态暴露：GET /api/thermal/state（节点 T/ΔT/C/派生 τ/功率明细/邻接
  R+热流/模块变形 6DOF live）；GET /api/thermal/curve?node=&n=（温度
  历史环形缓冲，array('f') 手动环，热 tick 频率×30min 窗）。

全部 CTE/接触系数/额定功率为工程估值（手册典型值量级），待真机热态
实测标定；非 FEA 空间温度场（LPTN+分裂节点对为一阶，覆盖主漂移模态）。
"""
import copy
import json
import math
import os
import random
import re
import threading
from array import array

import axes_config as ac

ROOT = os.path.dirname(os.path.abspath(__file__))
THERMAL_JSON = os.path.join(ROOT, "thermal.json")

REF_TEMP_C = 22.0                  # 变形/热软化参考温度（与 modal 一致）
KEY_RE = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
ENV_WAVE_LEVELS = {"off": 0.0, "fine": 0.05, "typical": 0.1, "coarse": 0.5}
# modal 热软化组 → 热节点键（thermal ON 时 modal._fn_eff 改读节点温度）
THERMAL_GROUP_NODES = {"head": "bond_head", "wafer": "wafer_chuck"}
# 校验区间
H_LO, H_HI = 0.0, 100.0            # 对流系数 h ∈ (0, 100] W/m²K
KC_LO, KC_HI = 0.0, 1.0            # 接触系数 ∈ (0, 1]
RATED_W_HI = 5000.0                # 加热器额定功率 ∈ (0, 5000] W
ANCHOR_HI = 1e5                    # 锚定电导 ∈ (0, 1e5] W/K
P_FL_HI = 1000.0                   # 轴组满载损耗 ∈ (0, 1000] W
CHAR_LEN_HI = 5000.0               # 特征长度 ∈ (0, 5000] mm
GAIN_ABS_MAX = 10.0                # deform scale 增益 / dir 分量限幅
HISTORY_S = 1800.0                 # 温度历史窗（30min，热 tick 频率）
C_FLOOR = 1.0                      # 空节点热容下限（J/K，防除零）

# ---------------- 默认分组（component 子树根 → 热节点；组件向上找首个
# 已登记祖先，全部 168 组件覆盖；2026-09-09 按前端模型 V6 实测归并） ----------------
DEFAULT_GROUPS = {
    "granite_low":     {"label": "花岗岩基座-下",
                        "members": ["machine_mssjivym_9mhbzz"]},
    "granite_up":      {"label": "花岗岩基座-上",
                        "members": ["box_mssjmmim_1tklq3"]},
    "wafer_xy_stage":  {"label": "硅片台XY导轨",
                        "members": ["group_msske7sa_02k07b"]},
    "wafer_fine_stage": {"label": "硅片台微动基座",
                         "members": ["box_mssl7mh8_x6ugcu"]},
    "wafer_chuck":     {"label": "晶圆吸盘（Heat_Dw 锚定）",
                        "members": ["wafer_chuck_msslygtz_gaeah4"]},
    "air_bearing":     {"label": "气浮台",
                        "members": ["cylinder_mssmg2rx_jsjzxg"]},
    "tray":            {"label": "Tray盘座",
                        "members": ["box_mssnz9ya_pdnd24"]},
    "bond_beam_a":     {"label": "键合臂横梁",
                        "members": ["box_mssolgz4_yl7vre"]},
    "bond_beam_b":     {"label": "键合臂平台架",
                        "members": ["box_mssoxg8g_bnpy7g",
                                    "mechanicalobject_mssp0h0p_vvd7df"]},
    "bond_arm":        {"label": "键合臂模块",
                        "members": ["group_msspk4l6_8kpu9x"]},
    "bond_head":       {"label": "键合头（PulseHeat_Up 锚定）",
                        "members": ["box_mssriiqy_ihe0nb"]},
    "align_stage":     {"label": "对准台体",
                        "members": ["group_msssqnbg_cvi3bv"]},
    "align_optics":    {"label": "对准相机承载台",
                        "members": ["box_mssty4oj_u3pj08"]},
    "flip_arm":        {"label": "翻转臂",
                        "members": ["group_mswkixk3_w71c02"]},
    "flip_frame":      {"label": "翻转模块基座",
                        "members": ["box_mswm5i05_kjqr7e",
                                    "box_mswm7jly_sqnw3l"]},
    "die_chuck":       {"label": "芯片吸盘",
                        "members": ["box_mswlxr6w_tkxtwz"]},
    "coarse_cam":      {"label": "粗对准相机",
                        "members": ["coarsecam_0001"]},
    "film_frame":      {"label": "蓝膜台框架",
                        "members": ["box_mswmqsvb_c7q1z6"]},
    "film_stage":      {"label": "蓝膜台滑台",
                        "members": ["filmstage_x_0001"]},
    "base_frame":      {"label": "铸铁底座",
                        "members": ["baseframe_0001"]},
}

# 默认邻接表（同分支父子 + 跨分支挂接；kc=接触系数工程估值，L/A 自动
# 由节点几何估算，可逐边覆盖 l_mm/a_mm2）
DEFAULT_LINKS = [
    {"a": "granite_low", "b": "granite_up", "kc": 0.5},
    {"a": "granite_low", "b": "base_frame", "kc": 0.5},
    {"a": "granite_up", "b": "wafer_xy_stage", "kc": 0.3},
    {"a": "granite_up", "b": "bond_beam_a", "kc": 0.3},
    {"a": "granite_up", "b": "align_stage", "kc": 0.3},
    {"a": "granite_up", "b": "flip_frame", "kc": 0.3},
    {"a": "granite_up", "b": "film_frame", "kc": 0.3},
    {"a": "wafer_xy_stage", "b": "wafer_fine_stage", "kc": 0.3},
    {"a": "wafer_fine_stage", "b": "air_bearing", "kc": 0.3},
    {"a": "air_bearing", "b": "wafer_chuck", "kc": 0.3},
    {"a": "wafer_fine_stage", "b": "tray", "kc": 0.3},
    {"a": "bond_beam_a", "b": "bond_beam_b", "kc": 0.3},
    {"a": "bond_beam_b", "b": "bond_arm", "kc": 0.3},
    {"a": "bond_arm", "b": "bond_head", "kc": 0.3},
    {"a": "align_stage", "b": "align_optics", "kc": 0.3},
    {"a": "flip_frame", "b": "flip_arm", "kc": 0.3},
    {"a": "flip_arm", "b": "die_chuck", "kc": 0.3},
    {"a": "flip_arm", "b": "coarse_cam", "kc": 0.3},
    {"a": "film_frame", "b": "film_stage", "kc": 0.3},
    {"a": "base_frame", "b": "electronics", "kc": 0.3},
]

# ---------------- 辐射链（2026-09-21 迭代 55：位置相关视角系数辐射换热） ----------------
# 固定邻接链只表达结构传导——迭代 54c 实验证明热源压近非接触目标的影响
# 精确为 0。热源（键合头 300℃/热吸盘）对邻近非接触节点的辐射换热走本表，
# 视角系数 F 随轴位置（间隙 gap + 横向 lateral）实时调制：
#   gap      = max(gap_min, gap0 + dir·(pos − pos_ref))   [gap_axis 可空=恒定 gap0]
#   lateral² = Σ(pos_i − ref_i)²                          [lat_axes 0~4 轴]
#   F        = f_max · r_eff² / (gap² + lateral² + r_eff²)   ∈ [0, f_max]
# 每热 tick 线性化：G = ε_eff·σ·A·F·(Ta²+Tb²)(Ta+Tb)（T 用 K），
# ε_eff = 1/(1/εa + 1/εb − 1)（两表面换热有效发射率）。默认覆盖整机两热源
# （bond_head/wafer_chuck）对邻近敏感节点的全部主要辐射路径；近环境节点
# 互辐射 T⁴ 太小不入表（可自行添加）。全部数值工程估值，待真机热态标定。
SIGMA_SB = 5.67e-8                          # Stefan-Boltzmann 常数 W/(m²K⁴)
RAD_T_MAX_K = 400.0 + 273.15                # 稳定 dt 估算用温度上限
DEFAULT_RAD_LINKS = [
    {"a": "bond_head", "b": "align_optics", "enable": True,
     "note": "键合头→上（对准）相机：头压到相机上方时辐射加热相机承载台",
     "eps_a": 0.8, "eps_b": 0.9, "area_mm2": 2000.0,
     "f_max": 0.4, "r_eff_mm": 25.0,
     "gap_axis": {"name": "BS_Arm_Z", "pos_ref_mm": -119.83, "dir": 1.0},
     "gap0_mm": 10.0, "gap_min_mm": 1.0,
     "lat_axes": [{"name": "AS_X", "ref_mm": 0.0},
                  {"name": "AS_Y", "ref_mm": 6.0}]},
    {"a": "wafer_chuck", "b": "align_optics", "enable": True,
     "note": "热吸盘→下（对准）相机：吸盘行至下相机下方时辐射（间隙恒定）",
     "eps_a": 0.8, "eps_b": 0.9, "area_mm2": 2000.0,
     "f_max": 0.4, "r_eff_mm": 60.0,
     "gap0_mm": 13.0, "gap_min_mm": 1.0,
     "lat_axes": [{"name": "WS_LS_X", "ref_mm": -42.394},
                  {"name": "WS_LS_Y", "ref_mm": 7.0}]},
    {"a": "bond_head", "b": "wafer_chuck", "enable": True,
     "note": "键合头→晶圆吸盘：键合接近/接触期辐射加热晶圆与 die",
     "eps_a": 0.8, "eps_b": 0.9, "area_mm2": 2000.0,
     "f_max": 0.5, "r_eff_mm": 25.0,
     "gap_axis": {"name": "BS_Arm_Z", "pos_ref_mm": -168.93, "dir": 1.0},
     "gap0_mm": 0.5, "gap_min_mm": 0.2,
     "lat_axes": [{"name": "WS_LS_X", "ref_mm": -42.394},
                  {"name": "WS_LS_Y", "ref_mm": 7.0}]},
    {"a": "bond_head", "b": "die_chuck", "enable": True,
     "note": "键合头→flip 芯片吸盘：交接位辐射（FA_X 对齐时视角最大）",
     "eps_a": 0.8, "eps_b": 0.9, "area_mm2": 800.0,
     "f_max": 0.4, "r_eff_mm": 15.0,
     "gap_axis": {"name": "BS_Arm_Z", "pos_ref_mm": -51.43, "dir": 1.0},
     "gap0_mm": 1.0, "gap_min_mm": 0.2,
     "lat_axes": [{"name": "FA_X", "ref_mm": -1242.35}]},
]

# 默认变形映射（6DOF 化五映射；全部工程估值，待真机热态标定）
DEFAULT_DEFORM_MAP = {
    # 花岗岩平均温升 → 硅片台 XY 几何 scale + 漂移；上/下基座温差 →
    # 绕 X 一阶弯曲（θ=CTE·ΔT_pair·L/h，杠杆 300mm 产生 Z 向平移）
    "granite_pair": {
        "enable": True,
        "nodes": ["granite_low", "granite_up"],
        "scale_axes": {"WS_LS_X": 1.0, "WS_LS_Y": 1.0},
        "module": "wafer_stage",
        "expand_dir": [0.5, 0.5, 0.0], "expand_len_mm": 300.0,
        "pair": ["granite_low", "granite_up"],
        "bend_axis": [1.0, 0.0, 0.0], "bend_len_mm": 1000.0,
        "bend_h_mm": 285.0, "bend_trans_dir": [0.0, 0.0, 1.0],
        "bend_lever_mm": 300.0,
    },
    # 吸盘温升 → die 间距热胀，等效为 WS 轴 scale_ppm（CTE 吸盘口径）
    "chuck_die_pitch": {
        "enable": True,
        "nodes": ["wafer_chuck"],
        "scale_axes": {"WS_LS_X": 1.0, "WS_LS_Y": 1.0},
    },
    # 横梁平均温升 → 键合头 X 偏移；横梁/平台架温差 → 绕 Y 弯曲
    "beam_to_head": {
        "enable": True,
        "nodes": ["bond_beam_a", "bond_beam_b"],
        "module": "bond_head",
        "expand_dir": [1.0, 0.0, 0.0], "expand_len_mm": 800.0,
        "pair": ["bond_beam_a", "bond_beam_b"],
        "bend_axis": [0.0, 1.0, 0.0], "bend_len_mm": 800.0,
        "bend_h_mm": 500.0, "bend_trans_dir": [0.0, 0.0, 1.0],
        "bend_lever_mm": 400.0,
    },
    # 键合头温升 → Z 向热伸长（头向下伸长，dz 为负）
    "head_z_growth": {
        "enable": True,
        "nodes": ["bond_head"],
        "module": "bond_head",
        "expand_dir": [0.0, 0.0, -1.0], "expand_len_mm": 150.0,
    },
    # 对准相机承载台温升 → 对准相机光轴横向漂移
    "align_optics_drift": {
        "enable": True,
        "nodes": ["align_optics"],
        "module": "align_stage",
        "expand_dir": [1.0, 0.0, 0.0], "expand_len_mm": 100.0,
    },
    # 吸盘温升 → Z 向热胀（晶圆面整体抬高；共焦面高/键合接触位热源，
    # 迭代 56 补——真机 110℃ 工况 ≈+60µm；expand_len=吸盘厚度工程估值）
    "chuck_z_growth": {
        "enable": True,
        "nodes": ["wafer_chuck"],
        "module": "wafer_stage",
        "expand_dir": [0.0, 0.0, 1.0], "expand_len_mm": 30.0,
    },
}

DEFAULT_PARAMS = {
    "_note": "组件级热仿真参数（thermal.py 加载，启动校验+构建预检）。"
             "groups=组件子树根→热节点归并表（组件向上找首个已登记祖先）；"
             "links=邻接导热（G=kc·λA/L，kc 接触系数工程估值，l_mm/a_mm2 "
             "可选覆盖）；rad_links=位置相关辐射链（G=ε_eff·σ·A·F·"
             "(Ta²+Tb²)(Ta+Tb)，F=f_max·r²/(gap²+lateral²+r²) 随轴位调制，"
             "gap0/dir/pos_ref/lat_axes 全可标定，见文件头 DEFAULT_RAD_LINKS "
             "注释）；conv=对流（h W/m²K，area_factor 表面积折算）；"
             "heaters=加热器功率源+锚定（rated_w 额定 W、anchor_w_per_k "
             "大电导锚定 heater.cur，温控环零改动）；motors=轴组电机损耗"
             "（P_fl 满载 W，活动度 0.65(v/vmax)²+0.35(a/amax)²，lp_tau_s "
             "低通）；electronics=机架电子件恒定功率虚拟节点；env=环境"
             "（base_c 默认 25℃=加热器环境/机器冷态基准（冷态自洽，结构"
             "节点 ΔT≈0），波动档 off/fine/typical/coarse/custom 正弦 "
             "period_s，random_walk 可选；deform_ref_c=热变形 ΔT 基准）；"
             "deform_map=热变形 6DOF 映射（nodes 均值"
             "源+pair 温差弯曲+scale_axes ppm+expand 膨胀）；开关 enabled "
             "是运行时内存态（POST /api/thermal），不写入本文件。全部 "
             "CTE/接触系数/额定功率为工程估值，待真机热态实测标定。",
    "groups": DEFAULT_GROUPS,
    "links": DEFAULT_LINKS,
    "rad_links": DEFAULT_RAD_LINKS,
    "conv": {"h_w_m2k": 5.0, "area_factor": 0.7},
    "heaters": {
        "PulseHeat_Up": {"node": "bond_head", "rated_w": 300.0,
                         "anchor_w_per_k": 300.0},
        "Heat_Dw": {"node": "wafer_chuck", "rated_w": 500.0,
                    "anchor_w_per_k": 2000.0},
    },
    "motors": {
        "lp_tau_s": 30.0,
        "groups": {
            "wafer_stage": {"node": "wafer_xy_stage", "p_full_w": 40.0,
                            "axes": ["WS_LS_X", "WS_LS_Y", "WS_Rz",
                                     "E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"]},
            "align_stage": {"node": "align_stage", "p_full_w": 20.0,
                            "axes": ["AS_X", "AS_Y", "AS_Z", "AS_Rx", "AS_Ry"]},
            "bond_arm": {"node": "bond_arm", "p_full_w": 15.0,
                         "axes": ["BS_Arm_Z", "BS_CAL_Rx", "BS_CAL_Ry"]},
            "flip": {"node": "flip_arm", "p_full_w": 20.0,
                     "axes": ["FA_X", "FA_Z", "FA_Rx", "CH_Rz"]},
            "film_stage": {"node": "film_stage", "p_full_w": 15.0,
                           "axes": ["ES_X", "ES_Y", "ES_PunchZ"]},
        },
    },
    "electronics": {"node": "electronics", "label": "机架电子件",
                    "const_w": 100.0, "mass_kg": 30.0,
                    "material": "ss_304", "area_m2": 1.5},
    "env": {"base_c": 25.0, "wave": "typical", "custom_amp_c": 0.1,
            "period_s": 600.0, "random_walk": False, "deform_ref_c": 25.0},
    "deform_map": DEFAULT_DEFORM_MAP,
}


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def _num(errors, path, v, lo, hi, lo_open=True):
    """有限数值 + 区间（默认 (lo, hi]；lo_open=False 时 [lo, hi]）。"""
    if not _is_num(v):
        errors.append("%s 须为有限数值: %r" % (path, v))
        return
    if (lo_open and not (lo < v <= hi)) or (not lo_open and not (lo <= v <= hi)):
        errors.append("%s 超出范围 %s%s, %s]: %r"
                      % (path, "(" if lo_open else "[", lo, hi, v))


def _bool(errors, path, v):
    if not isinstance(v, bool):
        errors.append("%s 须为布尔: %r" % (path, v))


def _vec3(errors, path, v, allow_zero=True):
    if not (isinstance(v, (list, tuple)) and len(v) == 3
            and all(_is_num(x) and abs(x) <= GAIN_ABS_MAX for x in v)):
        errors.append("%s 须为 3 分量向量（各分量 ±%s）: %r"
                      % (path, GAIN_ABS_MAX, v))
        return
    if not allow_zero and not any(v):
        errors.append("%s 不能是零向量" % path)


def _node_keys(cfg):
    keys = set(cfg.get("groups") or ())
    elec = cfg.get("electronics") or {}
    if elec.get("node"):
        keys.add(elec["node"])
    return keys


def validate_params(cfg, module_names=None):
    """热仿真参数全量校验 → 错误明细列表（空=合法）。module_names 非空时
    deform_map 的 module 键做存在性校验（与 error_kinematics.json 对齐）。"""
    errors = []
    if not isinstance(cfg, dict):
        return ["热仿真参数须为对象"]
    for k in cfg:
        if k not in ("groups", "links", "rad_links", "conv", "heaters",
                     "motors", "electronics", "env", "deform_map") \
                and not str(k).startswith("_"):
            errors.append("thermal 未知顶层字段: %r" % k)
    # ---- 节点分组 ----
    groups = cfg.get("groups")
    if not isinstance(groups, dict) or not groups:
        errors.append("thermal.groups 缺字段或须为非空对象")
    else:
        for name in sorted(groups):
            pre = "thermal.groups.%s" % name
            if not KEY_RE.match(name):
                errors.append("%s 节点键非法（须 ^[a-z][a-z0-9_]{0,39}$）" % pre)
            g = groups[name]
            if not isinstance(g, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in g:
                if kk not in ("label", "members"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            mem = g.get("members")
            if not isinstance(mem, list) \
                    or not all(isinstance(x, str) for x in mem):
                errors.append("%s.members 须为组件 id 字符串数组: %r"
                              % (pre, mem))
            if "label" in g and not isinstance(g["label"], str):
                errors.append("%s.label 须为字符串: %r" % (pre, g["label"]))
    keys = _node_keys(cfg) if isinstance(groups, dict) else set()
    # ---- 邻接导热 ----
    links = cfg.get("links")
    if not isinstance(links, list):
        errors.append("thermal.links 缺字段或须为数组")
    else:
        for i, lk in enumerate(links):
            pre = "thermal.links[%d]" % i
            if not isinstance(lk, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in lk:
                if kk not in ("a", "b", "kc", "l_mm", "a_mm2"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            a, b = lk.get("a"), lk.get("b")
            if a not in keys:
                errors.append("%s.a 未知节点: %r" % (pre, a))
            if b not in keys:
                errors.append("%s.b 未知节点: %r" % (pre, b))
            if a is not None and a == b:
                errors.append("%s a/b 不能同节点: %r" % (pre, a))
            _num(errors, pre + ".kc", lk.get("kc"), KC_LO, KC_HI)
            if "l_mm" in lk:
                _num(errors, pre + ".l_mm", lk.get("l_mm"), 0.0, 1e5)
            if "a_mm2" in lk:
                _num(errors, pre + ".a_mm2", lk.get("a_mm2"), 0.0, 1e8)
    # ---- 辐射链（位置相关视角系数） ----
    rad_links = cfg.get("rad_links")
    if not isinstance(rad_links, list):
        errors.append("thermal.rad_links 缺字段或须为数组")
    else:
        for i, lk in enumerate(rad_links):
            pre = "thermal.rad_links[%d]" % i
            if not isinstance(lk, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in lk:
                if kk not in ("a", "b", "enable", "note", "eps_a", "eps_b",
                              "area_mm2", "f_max", "r_eff_mm", "gap_axis",
                              "gap0_mm", "gap_min_mm", "lat_axes"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            a, b = lk.get("a"), lk.get("b")
            if a not in keys:
                errors.append("%s.a 未知节点: %r" % (pre, a))
            if b not in keys:
                errors.append("%s.b 未知节点: %r" % (pre, b))
            if a is not None and a == b:
                errors.append("%s a/b 不能同节点: %r" % (pre, a))
            if "enable" in lk:
                _bool(errors, pre + ".enable", lk.get("enable"))
            if "note" in lk and not isinstance(lk["note"], str):
                errors.append("%s.note 须为字符串" % pre)
            _num(errors, pre + ".eps_a", lk.get("eps_a"), 0.0, 1.0)
            _num(errors, pre + ".eps_b", lk.get("eps_b"), 0.0, 1.0)
            _num(errors, pre + ".area_mm2", lk.get("area_mm2"), 0.0, 1e6)
            _num(errors, pre + ".f_max", lk.get("f_max"), 0.0, 1.0)
            _num(errors, pre + ".r_eff_mm", lk.get("r_eff_mm"), 0.0, 500.0)
            _num(errors, pre + ".gap0_mm", lk.get("gap0_mm"), 0.0, 1e4,
                 lo_open=False)
            _num(errors, pre + ".gap_min_mm", lk.get("gap_min_mm"), 0.1,
                 100.0, lo_open=False)
            ga = lk.get("gap_axis")
            if ga is not None:
                if not isinstance(ga, dict):
                    errors.append("%s.gap_axis 须为对象" % pre)
                else:
                    for kk in ga:
                        if kk not in ("name", "pos_ref_mm", "dir"):
                            errors.append("%s.gap_axis 未知字段: %r" % (pre, kk))
                    if ga.get("name") not in ac.AXIS_NAMES:
                        errors.append("%s.gap_axis.name 未知轴: %r"
                                      % (pre, ga.get("name")))
                    _num(errors, pre + ".gap_axis.pos_ref_mm",
                         ga.get("pos_ref_mm"), -1e5, 1e5, lo_open=False)
                    if ga.get("dir") not in (-1.0, 1.0, -1, 1):
                        errors.append("%s.gap_axis.dir 须为 ±1: %r"
                                      % (pre, ga.get("dir")))
            lat = lk.get("lat_axes")
            if not isinstance(lat, list) or len(lat) > 4:
                errors.append("%s.lat_axes 须为 ≤4 条数组" % pre)
            else:
                for j, la in enumerate(lat):
                    pre2 = "%s.lat_axes[%d]" % (pre, j)
                    if not isinstance(la, dict):
                        errors.append("%s 须为对象" % pre2)
                        continue
                    for kk in la:
                        if kk not in ("name", "ref_mm"):
                            errors.append("%s 未知字段: %r" % (pre2, kk))
                    if la.get("name") not in ac.AXIS_NAMES:
                        errors.append("%s.name 未知轴: %r"
                                      % (pre2, la.get("name")))
                    _num(errors, pre2 + ".ref_mm", la.get("ref_mm"),
                         -1e5, 1e5, lo_open=False)
    # ---- 对流 ----
    conv = cfg.get("conv")
    if not isinstance(conv, dict):
        errors.append("thermal.conv 缺字段或须为对象")
    else:
        for kk in conv:
            if kk not in ("h_w_m2k", "area_factor"):
                errors.append("thermal.conv 未知字段: %r" % kk)
        _num(errors, "thermal.conv.h_w_m2k", conv.get("h_w_m2k"), H_LO, H_HI)
        _num(errors, "thermal.conv.area_factor", conv.get("area_factor"),
             0.0, 1.0)
    # ---- 加热器功率源 + 锚定 ----
    heaters = cfg.get("heaters")
    if not isinstance(heaters, dict):
        errors.append("thermal.heaters 缺字段或须为对象")
    else:
        for name in sorted(heaters):
            pre = "thermal.heaters.%s" % name
            h = heaters[name]
            if not isinstance(name, str) or not name:
                errors.append("%s 加热器名须为非空字符串" % pre)
            if not isinstance(h, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in h:
                if kk not in ("node", "rated_w", "anchor_w_per_k"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            if h.get("node") not in keys:
                errors.append("%s.node 未知节点: %r" % (pre, h.get("node")))
            _num(errors, pre + ".rated_w", h.get("rated_w"), 0.0, RATED_W_HI)
            _num(errors, pre + ".anchor_w_per_k", h.get("anchor_w_per_k"),
                 0.0, ANCHOR_HI)
    # ---- 电机损耗 ----
    motors = cfg.get("motors")
    if not isinstance(motors, dict):
        errors.append("thermal.motors 缺字段或须为对象")
    else:
        for kk in motors:
            if kk not in ("lp_tau_s", "groups"):
                errors.append("thermal.motors 未知字段: %r" % kk)
        _num(errors, "thermal.motors.lp_tau_s", motors.get("lp_tau_s"),
             0.0, 3600.0)
        mg = motors.get("groups")
        if not isinstance(mg, dict):
            errors.append("thermal.motors.groups 缺字段或须为对象")
        else:
            for name in sorted(mg):
                pre = "thermal.motors.groups.%s" % name
                g = mg[name]
                if not isinstance(g, dict):
                    errors.append("%s 须为对象" % pre)
                    continue
                for kk in g:
                    if kk not in ("node", "p_full_w", "axes"):
                        errors.append("%s 未知字段: %r" % (pre, kk))
                if g.get("node") not in keys:
                    errors.append("%s.node 未知节点: %r" % (pre, g.get("node")))
                _num(errors, pre + ".p_full_w", g.get("p_full_w"),
                     0.0, P_FL_HI)
                axes = g.get("axes")
                if not isinstance(axes, list) or not axes:
                    errors.append("%s.axes 须为非空数组" % pre)
                else:
                    for ax in axes:
                        if ax not in ac.NAME_TO_IDX:
                            errors.append("%s.axes 未知轴名: %r" % (pre, ax))
    # ---- 电子件虚拟节点 ----
    elec = cfg.get("electronics")
    if not isinstance(elec, dict):
        errors.append("thermal.electronics 缺字段或须为对象")
    else:
        for kk in elec:
            if kk not in ("node", "label", "const_w", "mass_kg",
                          "material", "area_m2"):
                errors.append("thermal.electronics 未知字段: %r" % kk)
        node = elec.get("node")
        if not isinstance(node, str) or not KEY_RE.match(node):
            errors.append("thermal.electronics.node 须为合法节点键: %r"
                          % (node,))
        elif isinstance(groups, dict) and node in groups:
            errors.append("thermal.electronics.node 与 groups 键冲突: %r"
                          % (node,))
        _num(errors, "thermal.electronics.const_w", elec.get("const_w"),
             0.0, 2000.0, lo_open=False)
        _num(errors, "thermal.electronics.mass_kg", elec.get("mass_kg"),
             0.0, 5000.0)
        _num(errors, "thermal.electronics.area_m2", elec.get("area_m2"),
             0.0, 20.0)
        if not isinstance(elec.get("material"), str):
            errors.append("thermal.electronics.material 须为材料键字符串: %r"
                          % (elec.get("material"),))
    # ---- 环境 ----
    env = cfg.get("env")
    if not isinstance(env, dict):
        errors.append("thermal.env 缺字段或须为对象")
    else:
        for kk in env:
            if kk not in ("base_c", "wave", "custom_amp_c", "period_s",
                          "random_walk", "deform_ref_c"):
                errors.append("thermal.env 未知字段: %r" % kk)
        _num(errors, "thermal.env.base_c", env.get("base_c"),
             -20.0, 60.0, lo_open=False)
        _num(errors, "thermal.env.deform_ref_c", env.get("deform_ref_c"),
             -20.0, 60.0, lo_open=False)
        lv = env.get("wave")
        if lv not in tuple(ENV_WAVE_LEVELS) + ("custom",):
            errors.append("thermal.env.wave 非法: %r（可选 %s/custom）"
                          % (lv, "/".join(ENV_WAVE_LEVELS)))
        _num(errors, "thermal.env.custom_amp_c", env.get("custom_amp_c"),
             0.0, 5.0, lo_open=False)
        _num(errors, "thermal.env.period_s", env.get("period_s"),
             0.0, 86400.0)
        _bool(errors, "thermal.env.random_walk", env.get("random_walk"))
    # ---- 变形映射 ----
    dm = cfg.get("deform_map")
    if not isinstance(dm, dict):
        errors.append("thermal.deform_map 缺字段或须为对象")
    else:
        for name in sorted(dm):
            pre = "thermal.deform_map.%s" % name
            if not KEY_RE.match(name):
                errors.append("%s 映射键非法（须 ^[a-z][a-z0-9_]{0,39}$）" % pre)
            e = dm[name]
            if not isinstance(e, dict):
                errors.append("%s 须为对象" % pre)
                continue
            for kk in e:
                if kk not in ("enable", "nodes", "pair", "scale_axes",
                              "module", "expand_dir", "expand_len_mm",
                              "bend_axis", "bend_len_mm", "bend_h_mm",
                              "bend_trans_dir", "bend_lever_mm"):
                    errors.append("%s 未知字段: %r" % (pre, kk))
            _bool(errors, pre + ".enable", e.get("enable"))
            nodes = e.get("nodes")
            if not (isinstance(nodes, list) and 1 <= len(nodes) <= 8
                    and all(n in keys for n in nodes)):
                errors.append("%s.nodes 须为 1~8 个已登记节点: %r"
                              % (pre, nodes))
            pair = e.get("pair")
            if pair is not None:
                if not (isinstance(pair, list) and len(pair) == 2
                        and all(n in keys for n in pair)):
                    errors.append("%s.pair 须为 2 个已登记节点: %r"
                                  % (pre, pair))
                else:
                    for fld in ("bend_axis", "bend_trans_dir"):
                        _vec3(errors, pre + "." + fld, e.get(fld),
                              allow_zero=False)
                    _num(errors, pre + ".bend_len_mm", e.get("bend_len_mm"),
                         0.0, CHAR_LEN_HI)
                    _num(errors, pre + ".bend_h_mm", e.get("bend_h_mm"),
                         0.0, CHAR_LEN_HI)
                    _num(errors, pre + ".bend_lever_mm",
                         e.get("bend_lever_mm"), 0.0, CHAR_LEN_HI,
                         lo_open=False)
            sa = e.get("scale_axes")
            if sa is not None:
                if not isinstance(sa, dict):
                    errors.append("%s.scale_axes 须为 {轴:增益} 对象" % pre)
                else:
                    for ax, g in sa.items():
                        if ax not in ac.NAME_TO_IDX:
                            errors.append("%s.scale_axes 未知轴名: %r"
                                          % (pre, ax))
                        elif not _is_num(g) or abs(g) > GAIN_ABS_MAX:
                            errors.append("%s.scale_axes.%s 增益须在 ±%s: %r"
                                          % (pre, ax, GAIN_ABS_MAX, g))
            if e.get("module") is not None:
                if module_names and e.get("module") not in module_names:
                    errors.append("%s.module 未知模块（error_kinematics.json "
                                  "无此模块）: %r" % (pre, e.get("module")))
                if e.get("expand_dir") is not None:
                    _vec3(errors, pre + ".expand_dir", e.get("expand_dir"))
                    _num(errors, pre + ".expand_len_mm",
                         e.get("expand_len_mm"), 0.0, CHAR_LEN_HI)
            if not sa and e.get("module") is None:
                errors.append("%s 至少须有一条效应（scale_axes/module）" % pre)
    return errors


def _fill_defaults(cfg):
    """加载迁移：缺失顶层块按默认播种（仅补缺失，非法值仍由 validate 报错）。"""
    d = DEFAULT_PARAMS
    for k in ("conv", "electronics", "env"):
        cur = cfg.setdefault(k, copy.deepcopy(d[k]))
        if isinstance(cur, dict):
            for kk, vv in d[k].items():
                cur.setdefault(kk, copy.deepcopy(vv))
    cfg.setdefault("links", copy.deepcopy(d["links"]))
    cfg.setdefault("rad_links", copy.deepcopy(d["rad_links"]))
    deform = cfg.setdefault("deform_map", {})
    for name, dv in d["deform_map"].items():
        cur = deform.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, copy.deepcopy(vv))
    groups = cfg.setdefault("groups", {})
    for name, dv in d["groups"].items():
        cur = groups.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, copy.deepcopy(vv))
    heaters = cfg.setdefault("heaters", {})
    for name, dv in d["heaters"].items():
        cur = heaters.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, vv)
    motors = cfg.setdefault("motors", {})
    motors.setdefault("lp_tau_s", d["motors"]["lp_tau_s"])
    mg = motors.setdefault("groups", {})
    for name, dv in d["motors"]["groups"].items():
        cur = mg.setdefault(name, {})
        if isinstance(cur, dict):
            for kk, vv in dv.items():
                cur.setdefault(kk, copy.deepcopy(vv))
    return cfg


def _deep_merge(dst, src):
    """递归合并：dict 逐键合并，其余（含数组）整体替换。"""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, path)


class LoadError(ValueError):
    """热仿真参数文件加载/校验/构建失败（启动拒绝拉起）。"""


class ThermalSystem(object):
    """热网络运行时：参数容器 + 分频半隐式 Euler 解算 + 变形/状态暴露。

    线程模型：tick() 由物理线程在 engine.lock 内调用；actual/compute/
    API 读取侧只取基本类型快照（GIL 原子）；参数更新（PUT）与 tick 经
    self._lock 互斥（持锁极短）。enabled 为运行时内存态、默认 OFF 不
    持久化（保护回归基线）；OFF 时 tick 早退、变形/scale 零注入，节点
    温度状态保持（热惯性语义，与 modal OFF 清零不同）。"""

    def __init__(self, path=THERMAL_JSON, catalog=None, materials=None,
                 module_names=None, dt=0.001):
        self._path = path
        self._catalog = catalog
        self._materials = materials
        self._module_names = tuple(module_names or ())
        self._dt = float(dt)
        self._lock = threading.RLock()
        self.enabled = False           # 总开关：运行时内存态，默认 OFF
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                cfg = json.load(f)
            cfg = _fill_defaults(cfg)
        else:
            cfg = copy.deepcopy(DEFAULT_PARAMS)
            _atomic_write(path, cfg)
        errors = validate_params(cfg, self._module_names or None)
        if errors:
            raise LoadError("thermal.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._cfg = cfg
        try:
            self._build()
        except ValueError as e:
            raise LoadError("thermal 网络构建失败：%s" % e)
        self._rng = random.Random()    # 环境随机游走专用（不污染全局序列）
        self._reset_state()

    # ---------------- 网络构建（启动/PUT；构建产物预检通过才允许落盘） ----------------
    def _build(self):
        """按 cfg+catalog+materials 构建节点/边/派生量 → self._net。
        材料键不存在等构建级错误抛 ValueError。catalog 为空（前端模型
        缺失）时组件节点质量为 0（C 取下限，节点趋向环境温度）。"""
        cfg = self._cfg
        # ---- 组件 → 节点/材料 解析表 ----
        owner = {}
        for node, g in cfg["groups"].items():
            for root in g.get("members") or []:
                owner[root] = node
        comps = self._catalog.list() if self._catalog is not None else []
        parent = {c["id"]: c["parentId"] for c in comps}
        cmap = self._materials.get_map() if self._materials is not None else {}
        if self._materials is not None:
            mlib = self._materials.get_materials()
        else:
            # 无材料库注入（进程内单元构造）：回退内置默认库
            from materials import DEFAULT_MATERIALS
            mlib = DEFAULT_MATERIALS

        def node_of(cid):
            cur = cid
            while cur is not None:
                if cur in owner:
                    return owner[cur]
                cur = parent.get(cur)
            return None

        def mat_of(cid):
            cur = cid
            while cur is not None:
                mk = cmap.get(cur)
                if mk:
                    return mk
                cur = parent.get(cur)
            return "ss_304"      # 兜底材料（docstring 已知边界）

        # ---- 节点聚合：m=ΣρV、C=ΣρV·cp、CTE/λ 质量加权、包围尺寸 ----
        agg = {n: {"label": g.get("label") or n, "m": 0.0, "c": 0.0,
                   "cte_m": 0.0, "lam_m": 0.0, "dims": [0.0, 0.0, 0.0],
                   "mats": {}, "n_comp": 0, "n_geo": 0}
               for n, g in cfg["groups"].items()}
        for c in comps:
            node = node_of(c["id"])
            if node is None:
                continue         # 未归并组件（groups 表未覆盖）：跳过
            a = agg[node]
            a["n_comp"] += 1
            vol = c.get("volume_mm3")
            if not vol:
                continue
            mk = mat_of(c["id"])
            props = mlib.get(mk)
            if props is None:
                raise ValueError("组件 %s 解析材料键 %r 不在材料库" %
                                 (c["id"], mk))
            m = float(props["density_kg_m3"]) * vol * 1e-9
            a["m"] += m
            a["c"] += m * float(props["spec_heat_J_kgK"])
            a["cte_m"] += m * float(props["cte_ppm_K"])
            a["lam_m"] += m * float(props["thermal_cond_W_mK"])
            a["mats"][mk] = a["mats"].get(mk, 0.0) + m
            a["n_geo"] += 1
            dims = c.get("dims")
            if dims:
                for k in range(3):
                    a["dims"][k] = max(a["dims"][k], float(dims[k]))
        # ---- 电子件虚拟节点 ----
        elec = cfg["electronics"]
        em = mlib.get(elec["material"])
        if em is None:
            raise ValueError("electronics.material 材料键 %r 不在材料库"
                             % (elec["material"],))
        emass = float(elec["mass_kg"])
        agg[elec["node"]] = {
            "label": elec.get("label") or elec["node"], "m": emass,
            "c": emass * float(em["spec_heat_J_kgK"]),
            "cte_m": emass * float(em["cte_ppm_K"]),
            "lam_m": emass * float(em["thermal_cond_W_mK"]),
            "dims": [0.0, 0.0, 0.0], "mats": {elec["material"]: emass},
            "n_comp": 0, "n_geo": 0, "virtual": True,
            "area_m2": float(elec["area_m2"])}
        nodes = {}
        area_factor = float(cfg["conv"]["area_factor"])
        for n, a in agg.items():
            m = a["m"]
            if a.get("area_m2"):
                area = a["area_m2"]
            else:
                w, h, d = (x * 1e-3 for x in a["dims"])
                area = 2.0 * (w * h + h * d + w * d) * area_factor
            nodes[n] = {
                "label": a["label"], "virtual": bool(a.get("virtual")),
                "mass_kg": m,
                "c_J_K": max(C_FLOOR, a["c"]),
                "empty": a["n_geo"] == 0 and not a.get("virtual"),
                "cte_ppm_K": (a["cte_m"] / m) if m > 0 else 0.0,
                "lam_W_mK": (a["lam_m"] / m) if m > 0 else 10.0,
                "area_m2": area,
                "mats": sorted(a["mats"], key=lambda k: -a["mats"][k])[:3],
                "n_comp": a["n_comp"],
            }
            # 特征长度：包围盒体积立方根（未知时给 50mm 下限量级）
            w, h, d = a["dims"]
            bbox = (w * h * d) * 1e-9
            nodes[n]["char_len_m"] = bbox ** (1.0 / 3.0) if bbox > 0 else 0.05
        # ---- 边：G=kc·λ_eff·A/L（可调覆盖 l_mm/a_mm2） ----
        links = []
        for lk in cfg["links"]:
            na, nb = nodes[lk["a"]], nodes[lk["b"]]
            lam_eff = 2.0 * na["lam_W_mK"] * nb["lam_W_mK"] \
                / max(1e-9, na["lam_W_mK"] + nb["lam_W_mK"])
            leng = float(lk.get("l_mm") or
                         max(10.0, 500.0 * (na["char_len_m"]
                                            + nb["char_len_m"])))
            area = float(lk.get("a_mm2") or
                         max(100.0, 0.25 * min(na["area_m2"], nb["area_m2"])
                             * 1e6))
            g = float(lk["kc"]) * lam_eff * (area * 1e-6) / (leng * 1e-3)
            links.append({"a": lk["a"], "b": lk["b"], "g_w_k": g,
                          "r_k_per_w": 1.0 / g if g > 0 else float("inf"),
                          "kc": float(lk["kc"])})
        # ---- 辐射链：预算常量（ε_eff·σ·A）；电导随 T/轴位每 tick 重算 ----
        rad = []
        for lk in cfg.get("rad_links") or []:
            if not lk.get("enable"):
                continue
            eps_eff = 1.0 / (1.0 / float(lk["eps_a"])
                             + 1.0 / float(lk["eps_b"]) - 1.0)
            rad.append({
                "a": lk["a"], "b": lk["b"],
                "k_w": eps_eff * SIGMA_SB * float(lk["area_mm2"]) * 1e-6,
                "f_max": float(lk["f_max"]),
                "r2": float(lk["r_eff_mm"]) ** 2,
                "gap_axis": lk.get("gap_axis"),
                "gap0": float(lk["gap0_mm"]),
                "gap_min": float(lk.get("gap_min_mm") or 1.0),
                "lat_axes": lk.get("lat_axes") or [],
                "note": lk.get("note") or "",
            })
        # ---- 每节点总电导/派生 τ/稳定 dt ----
        h_conv = float(cfg["conv"]["h_w_m2k"])
        g_tot = {n: h_conv * nd["area_m2"] for n, nd in nodes.items()}
        for lk in links:
            g_tot[lk["a"]] += lk["g_w_k"]
            g_tot[lk["b"]] += lk["g_w_k"]
        for lk in rad:
            # 辐射电导上限（T=RAD_T_MAX_K、F=f_max）计入稳定 dt 余量
            g_max = lk["k_w"] * lk["f_max"] * 4.0 * RAD_T_MAX_K ** 3
            g_tot[lk["a"]] += g_max
            g_tot[lk["b"]] += g_max
        for hname, hcfg in cfg["heaters"].items():
            if hcfg["node"] in g_tot:
                g_tot[hcfg["node"]] += float(hcfg["anchor_w_per_k"])
        stable = [nodes[n]["c_J_K"] / g_tot[n] for n in nodes
                  if g_tot[n] > 0]
        dt_stable = min(stable) if stable else 60.0
        dt_th = min(0.5, max(0.05, 0.25 * dt_stable))
        div = int(round(dt_th / self._dt))
        div = max(50, min(500, div))
        dt_th = div * self._dt
        self._net = {"nodes": nodes, "links": links, "rad": rad,
                     "g_tot": g_tot,
                     "dt_stable": dt_stable, "dt": dt_th, "div": div,
                     "hist_len": max(10, int(round(HISTORY_S / dt_th)))}

    # ---------------- 状态 ----------------
    def _reset_state(self):
        """运行时状态重置（启动/参数替换）：节点温度=环境基准，历史清空。"""
        base = float(self._cfg["env"]["base_c"])
        net = self._net
        self._T = {n: base for n in net["nodes"]}
        self._tick_i = 0
        self._walk = 0.0             # 环境随机游走偏置
        self._last_vel = {n: 0.0 for n in ac.AXIS_NAMES}
        self._motor_lp = {g: 0.0
                          for g in self._cfg["motors"]["groups"]}
        self._pwr = {"heaters": {}, "motors": {}, "electronics_w": 0.0,
                     "env_c": base, "p_in": {n: 0.0 for n in net["nodes"]}}
        self._hist = {n: {"buf": array("f", [0.0]) * net["hist_len"],
                          "head": 0, "count": 0}
                      for n in net["nodes"]}

    def set_enabled(self, on):
        with self._lock:
            self.enabled = bool(on)    # OFF 不清状态（热惯性保持）
            return self.enabled

    # ---------------- 参数（GET/PUT） ----------------
    def public(self):
        with self._lock:
            return {"ok": True, "enabled": self.enabled,
                    "tick": {"dt": self._net["dt"], "div": self._net["div"],
                             "hz": round(1.0 / self._net["dt"], 3),
                             "dt_stable_explicit": round(
                                 self._net["dt_stable"], 4)},
                    "params": copy.deepcopy(self._cfg)}

    def update(self, patch):
        """部分合并 → 全量校验 → 构建预检 → 原子落盘 + 热替换 + 状态重置。
        patch 可带 "enabled"（运行时开关，不落盘）。校验/构建失败抛
        ValueError（消息含逐条 errors），原参数/原网络不污染。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            if "enabled" in patch:      # 运行时开关（同 POST /api/thermal）
                self.set_enabled(patch["enabled"])
            merged = copy.deepcopy(self._cfg)
            _deep_merge(merged, {k: v for k, v in patch.items()
                                 if k != "enabled"})
            errors = validate_params(merged, self._module_names or None)
            if errors:
                raise ValueError("热仿真参数校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            old_cfg, old_net = self._cfg, self._net
            self._cfg = merged
            try:
                self._build()           # 构建预检：失败不落盘不换网络
            except ValueError:
                self._cfg, self._net = old_cfg, old_net
                raise
            _atomic_write(self._path, merged)
            self._reset_state()          # 参数变了，温度从环境基准重新起

    # ---------------- 解算（engine.lock 内 1kHz 主循环分频调用） ----------------
    def tick(self, dt, engine):
        """推进一个仿真 tick；内部按 net['div'] 分频（2~20Hz）真正解算。
        enabled 时才有开销（engine 侧已判）；单次热 tick 纯算术 ≤50µs。"""
        net = self._net
        self._tick_i += 1
        if self._tick_i % net["div"] != 0:
            return
        cfg = self._cfg
        dt_th = net["dt"]
        nodes = net["nodes"]
        T = self._T
        env = cfg["env"]
        # ---- 环境边界温度：基准 + 正弦波动档 (+可选随机游走) ----
        amp = ENV_WAVE_LEVELS.get(env["wave"])
        if amp is None:
            amp = float(env["custom_amp_c"])
        phase = 2.0 * math.pi * engine.sim_time / float(env["period_s"])
        if env.get("random_walk") and amp > 0.0:
            # 随机游走：步 σ=amp·√(dt/period)（工程近似，限幅 ±amp）
            self._walk += self._rng.gauss(
                0.0, amp * math.sqrt(dt_th / float(env["period_s"])))
            self._walk = max(-amp, min(amp, self._walk))
            t_amb = float(env["base_c"]) + self._walk
        else:
            t_amb = float(env["base_c"]) + amp * math.sin(phase)
        # ---- 功率源 1：加热器 power%×额定 W（冷却方向按 −C·cool_rate
        #      等效提取）+ heater.cur 大电导锚定 ----
        p_in = {n: 0.0 for n in nodes}
        h_anchor = {}                  # node -> (g_anchor, t_heater)
        pwr_h = {}
        for hname, hcfg in cfg["heaters"].items():
            h = engine.heaters.get(hname)
            if h is None:
                continue
            node = hcfg["node"]
            if node not in nodes:
                continue
            rated = float(hcfg["rated_w"])
            # 功率注入只算加热方向；冷却方向注入为 0——冷却由锚定追踪
            # heater.cur 下行实现（等效提取功率=锚定回流，见 state 的
            # anchor_flow_w），不与 −C·cool_rate 双重计算
            p_src = (h.power / 100.0) * rated if h.active else 0.0
            p_in[node] += p_src
            g_a = float(hcfg["anchor_w_per_k"])
            prev = h_anchor.get(node)   # 同节点多加热器时电导并联
            if prev is None:
                h_anchor[node] = [g_a, g_a * h.cur]
            else:
                prev[0] += g_a
                prev[1] += g_a * h.cur
            pwr_h[hname] = {"power_pct": round(h.power, 2),
                            "p_w": round(p_src, 3), "node": node,
                            "cur_c": round(h.cur, 3),
                            "anchor_flow_w": round(
                                g_a * (self._T[node] - h.cur), 3)}
        # ---- 功率源 2：电机损耗（轴组活动度 30s 低通） ----
        lp_tau = float(cfg["motors"]["lp_tau_s"])
        k_lp = dt_th / lp_tau
        pwr_m = {}
        for gname, gcfg in cfg["motors"]["groups"].items():
            p_inst = 0.0
            for ax_name in gcfg["axes"]:
                ax = engine.axes.get(ax_name)
                if ax is None or ax.vmax <= 0 or ax.amax <= 0:
                    continue
                acc = abs(ax.vel - self._last_vel[ax_name]) / dt_th
                self._last_vel[ax_name] = ax.vel
                acc = min(acc, ax.amax)
                rv = ax.vel / ax.vmax
                ra = acc / ax.amax
                p_inst += float(gcfg["p_full_w"]) \
                    * (0.65 * rv * rv + 0.35 * ra * ra)
            lp = self._motor_lp[gname] + (p_inst - self._motor_lp[gname]) * k_lp
            self._motor_lp[gname] = lp
            node = gcfg["node"]
            if node in p_in:
                p_in[node] += lp
            pwr_m[gname] = {"p_w": round(lp, 4), "p_inst_w": round(p_inst, 4),
                            "node": node}
        # ---- 功率源 3：电子件恒定功率 ----
        elec = cfg["electronics"]
        p_elec = float(elec["const_w"])
        if elec["node"] in p_in:
            p_in[elec["node"]] += p_elec
        # ---- 半隐式 Euler：自耦合后向、邻接/边界前向（无条件稳定） ----
        # 辐射链：视角系数随轴位、电导随 T 逐 tick 线性化（显式滞后一拍，
        # 2Hz 下非线性温和；读轴 pos 与 motor 段同为 engine.lock 内直读）
        rad_by_node = {}
        rad_pub = []
        for lk in net["rad"]:
            gap = lk["gap0"]
            ga = lk["gap_axis"]
            if ga is not None:
                ax = engine.axes.get(ga["name"])
                if ax is not None:
                    gap = lk["gap0"] + float(ga["dir"]) * (
                        ax.pos - float(ga["pos_ref_mm"]))
            gap = max(lk["gap_min"], gap)
            lat2 = 0.0
            for la in lk["lat_axes"]:
                ax = engine.axes.get(la["name"])
                if ax is not None:
                    d = ax.pos - float(la["ref_mm"])
                    lat2 += d * d
            d2 = gap * gap + lat2
            f = lk["f_max"] * lk["r2"] / (d2 + lk["r2"])
            ta = T[lk["a"]] + 273.15
            tb = T[lk["b"]] + 273.15
            g = lk["k_w"] * f * (ta + tb) * (ta * ta + tb * tb)
            rad_by_node.setdefault(lk["a"], []).append((lk["b"], g))
            rad_by_node.setdefault(lk["b"], []).append((lk["a"], g))
            rad_pub.append({"a": lk["a"], "b": lk["b"], "note": lk["note"],
                            "f": round(f, 5), "gap_mm": round(gap, 3),
                            "d_eff_mm": round(math.sqrt(d2), 3),
                            "g_w_k": round(g, 6),
                            "flow_w": round(g * (T[lk["a"]] - T[lk["b"]]), 4)})
        h_conv = float(cfg["conv"]["h_w_m2k"])
        new_T = {}
        for n, nd in nodes.items():
            g_conv = h_conv * nd["area_m2"]
            influx = p_in[n] + g_conv * t_amb
            g_self = g_conv
            if n in h_anchor:
                g_a, gt = h_anchor[n]
                influx += gt
                g_self += g_a
            for lk in net["links"]:
                if lk["a"] == n:
                    influx += lk["g_w_k"] * T[lk["b"]]
                    g_self += lk["g_w_k"]
                elif lk["b"] == n:
                    influx += lk["g_w_k"] * T[lk["a"]]
                    g_self += lk["g_w_k"]
            for other, rg in rad_by_node.get(n, ()):
                influx += rg * T[other]
                g_self += rg
            c = nd["c_J_K"]
            new_T[n] = (c / dt_th * T[n] + influx) / (c / dt_th + g_self)
        T.update(new_T)
        # ---- 历史环形缓冲（热 tick 频率 × HISTORY_S 窗） ----
        for n, nd in nodes.items():
            hs = self._hist[n]
            hs["buf"][hs["head"]] = T[n]
            hs["head"] = (hs["head"] + 1) % net["hist_len"]
            if hs["count"] < net["hist_len"]:
                hs["count"] += 1
        self._pwr = {"heaters": pwr_h, "motors": pwr_m, "rad": rad_pub,
                     "electronics_w": p_elec, "env_c": round(t_amb, 4),
                     "p_in": p_in}

    # ---------------- 效应读取（actual / sample6 / compute / modal） ----------------
    def _node_dt(self, node):
        """节点变形 ΔT = T − deform_ref_c（默认 25℃，机器冷态标定基准）；
        节点不存在 → None。modal 热软化请用 group_delta_t（REF 22℃口径）。"""
        if node not in self._T:
            return None
        return self._T[node] - float(self._cfg["env"]["deform_ref_c"])

    def group_delta_t(self, grp):
        """modal 热软化 ΔT 源（thermal ON 时 _fn_eff 调用）：
        head→bond_head、wafer→wafer_chuck；REF=22℃（modal REF_TEMP_C
        口径，与 OFF 回退 heater.cur−22 一致）；未映射 → None。"""
        node = THERMAL_GROUP_NODES.get(grp)
        if not node or node not in self._T:
            return None
        return self._T[node] - REF_TEMP_C

    def _entry_src(self, e):
        """变形映射源量 → (ΔT_mean, CTE_eff, ΔT_pair|None)。调用方保证
        enabled；nodes 均值口径，pair=T[a]−T[b]。"""
        nodes = e.get("nodes") or []
        if not nodes:
            return None
        dts, ctes = [], []
        for n in nodes:
            d = self._node_dt(n)
            if d is None:
                return None
            dts.append(d)
            ctes.append(self._net["nodes"][n]["cte_ppm_K"])
        dt_mean = sum(dts) / len(dts)
        cte = sum(ctes) / len(ctes)
        dt_pair = None
        pair = e.get("pair")
        if pair:
            d0, d1 = self._node_dt(pair[0]), self._node_dt(pair[1])
            if d0 is None or d1 is None:
                return None
            dt_pair = d0 - d1
        return dt_mean, cte, dt_pair

    def axis_scale_ppm(self, axis):
        """该轴热 scale_ppm 分量（actual()/sample6() 加进 scale 项；
        未启用零注入——engine 侧热查先例同 modal）。"""
        if not self.enabled:
            return 0.0
        total = 0.0
        for e in self._cfg["deform_map"].values():
            if not e.get("enable"):
                continue
            g = (e.get("scale_axes") or {}).get(axis)
            if not g:
                continue
            src = self._entry_src(e)
            if src is None:
                continue
            total += float(g) * src[1] * src[0]
        return total

    def module_offset(self, module):
        """模块 6DOF 热变形叠加量 → (t[3] mm, r[3] rad)（compute() 读取时
        合成，零 tick 开销；膨胀 ΔL=CTE·ΔT·L·dir，弯曲 θ=CTE·ΔT_pair·L/h
        绕 bend_axis + θ×lever 沿 bend_trans_dir 平移）。"""
        t = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, 0.0]
        if not self.enabled:
            return t, r
        for e in self._cfg["deform_map"].values():
            if not e.get("enable") or e.get("module") != module:
                continue
            src = self._entry_src(e)
            if src is None:
                continue
            dt_mean, cte, dt_pair = src
            if e.get("expand_dir") is not None and any(e["expand_dir"]):
                dl = cte * 1e-6 * dt_mean * float(e["expand_len_mm"])
                for k in range(3):
                    t[k] += dl * e["expand_dir"][k]
            if dt_pair is not None:
                theta = cte * 1e-6 * dt_pair * float(e["bend_len_mm"]) \
                    / float(e["bend_h_mm"])
                for k in range(3):
                    r[k] += theta * e["bend_axis"][k]
                    t[k] += theta * float(e["bend_lever_mm"]) \
                        * e["bend_trans_dir"][k]
        return t, r

    # ---------------- 状态/曲线 API ----------------
    def state_public(self, engine=None):
        """GET /api/thermal/state：节点 T/ΔT/C/派生 τ/功率明细/邻接 R+热流/
        模块变形 6DOF live/能量平衡残差（稳态 →0）。"""
        net = self._net
        ref = float(self._cfg["env"]["deform_ref_c"])
        nodes = {}
        p_in = self._pwr.get("p_in") or {}
        for n, nd in net["nodes"].items():
            g_tot = net["g_tot"][n]
            nodes[n] = {
                "label": nd["label"], "virtual": nd["virtual"],
                "empty": nd["empty"], "materials": nd["mats"],
                "mass_kg": round(nd["mass_kg"], 4),
                "c_J_K": round(nd["c_J_K"], 2),
                "area_m2": round(nd["area_m2"], 5),
                "cte_ppm_K": round(nd["cte_ppm_K"], 3),
                "tau_s": round(nd["c_J_K"] / g_tot, 2) if g_tot > 0 else None,
                "t_c": round(self._T[n], 4),
                "delta_t_c": round(self._T[n] - ref, 4),
                "p_in_w": round(p_in.get(n, 0.0), 4),
            }
        links = []
        max_resid = 0.0
        flow = {}
        for lk in net["links"]:
            f = lk["g_w_k"] * (self._T[lk["a"]] - self._T[lk["b"]])
            flow.setdefault(lk["a"], 0.0)
            flow.setdefault(lk["b"], 0.0)
            flow[lk["a"]] -= f
            flow[lk["b"]] += f
            links.append({"a": lk["a"], "b": lk["b"],
                          "g_w_k": round(lk["g_w_k"], 5),
                          "r_k_per_w": round(lk["r_k_per_w"], 3),
                          "flow_w": round(f, 4)})
        # 辐射链流量（tick 时随视角系数/温度算出并缓存于此）计入节点收支
        rad_pub = self._pwr.get("rad") or []
        for rf in rad_pub:
            flow.setdefault(rf["a"], 0.0)
            flow.setdefault(rf["b"], 0.0)
            flow[rf["a"]] -= rf["flow_w"]
            flow[rf["b"]] += rf["flow_w"]
        # 能量平衡残差：节点净流入 − 源（稳态 →0，=C·dT/dt 瞬态项）
        env_c = self._pwr.get("env_c", float(self._cfg["env"]["base_c"]))
        h_conv = float(self._cfg["conv"]["h_w_m2k"])
        resid = {}
        for n, nd in net["nodes"].items():
            g_conv = h_conv * nd["area_m2"]
            r = p_in.get(n, 0.0) + flow.get(n, 0.0) \
                + g_conv * (env_c - self._T[n])
            for hname, hcfg in self._cfg["heaters"].items():
                if hcfg["node"] == n and engine is not None:
                    h = engine.heaters.get(hname)
                    if h is not None:
                        r += float(hcfg["anchor_w_per_k"]) * (h.cur - self._T[n])
            resid[n] = r
            max_resid = max(max_resid, abs(r))
        deform = {}
        for mod in self._module_names:
            tt, rr = self.module_offset(mod)
            deform[mod] = {
                "t_mm": [round(v, 9) for v in tt],
                "r_rad": [round(v, 12) for v in rr],
                "t_nm": [round(v * 1e6, 3) for v in tt],
                "r_urad": [round(v * 1e6, 6) for v in rr]}
        return {"ok": True, "enabled": self.enabled,
                "tick": {"dt": net["dt"], "div": net["div"],
                         "hz": round(1.0 / net["dt"], 3),
                         "dt_stable_explicit": round(net["dt_stable"], 4)},
                "env_c": env_c,
                "ref_c": float(self._cfg["env"]["deform_ref_c"]),
                "nodes": nodes, "links": links,
                "rad_links": rad_pub,
                "power": {"heaters": self._pwr.get("heaters") or {},
                          "motors": self._pwr.get("motors") or {},
                          "electronics_w": self._pwr.get("electronics_w", 0.0),
                          "balance_residual_w": round(max_resid, 5),
                          "residual_by_node_w": {n: round(v, 5)
                                                 for n, v in resid.items()}},
                "deform": deform}

    def curve_public(self, node, n):
        """GET /api/thermal/curve?node=&n=：温度历史环形缓冲抽取（时间升序）。"""
        hs = self._hist.get(node)
        if hs is None:
            raise ValueError("未知热节点: %r（可选 %s）"
                             % (node, "/".join(sorted(self._hist))))
        net = self._net
        cnt = hs["count"]
        n = min(n, cnt)
        head = hs["head"]
        size = net["hist_len"]
        buf = hs["buf"]
        start = (head - n) % size
        if n <= 0:
            data = []
        elif start + n <= size:
            data = list(buf[start:start + n])
        else:
            data = list(buf[start:]) + list(buf[:(start + n) % size])
        return {"ok": True, "enabled": self.enabled, "node": node,
                "dt": net["dt"], "n": len(data),
                "count": cnt, "data": [round(v, 4) for v in data]}
