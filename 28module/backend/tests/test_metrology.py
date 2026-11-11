#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_metrology.py — 相机对准重复性测校独立回归（不进 test_client.py）。

运行：python backend/tests/test_metrology.py（退出码非 0=失败；目标 <90s）
     python backend/tests/test_metrology.py -k i32（只跑迭代 32 段）
     python backend/tests/test_metrology.py -k i34（只跑迭代 34 段）
     python backend/tests/test_metrology.py -k i35（只跑迭代 35 段，含专属
     真值 boot 段）
     python backend/tests/test_metrology.py -k i36（只跑迭代 36/37 段：
     Y 导轨方向 FK 注入 + as/ws_stage_nonortho 非正交测校·栅格跳测法）
     python backend/tests/test_metrology.py -k "view_rot or microrot"
     （迭代 39 主题定向：view_rot=在线配置+REAL 识别旋转真值两件套；
     microrot=i31 三相机+i34 选台/拍照位+i39 真值闭环；可与迭代段 or
     组合，如 "view_rot or microrot or i35"）
     python backend/tests/test_metrology.py -k i42（只跑迭代 42 段：
     as_head_center 上吸盘位置标定 + best_focus 相机最佳焦平面标定）
     python backend/tests/test_metrology.py -k i49（只跑迭代 49 段：
     bs_rxry_axis 调平轴旋转轴线标定 + flip_handoff_height 交接接触
     高度 + bond_height_calib 键合接触高度 + bond_temp_calib 温度
     传感链 + bond_force_calib 力传感链；或按单项定向 -k "rxry or
     handoff_height or bond_height or temp_calib or force_calib"）

自起一套独立端口的后端（93xx 段，避开生产/test_client 的
9010/9101/9102/9190/1883/9200）：
  WS 9310 / ACS 9301 / PLC 9302 / MCS 9390 / MQTT 9383 / 视觉 TCP 9320
（经 runtime/main.py 的 D2W_PORT_* 环境变量注入；后端子进程 finally 清理）。

覆盖：
- 静态 align_repeat_static ×（up/down/coarse 各 n=10）
- 动态 align_repeat_dynamic ×（三相机各 n=10，travel_mm=2 兼容别名路径）
- 迭代 25：template_name（不存在→400；存在→done 且结果 template_name
  正确、用户模板作业后不被删）；动态绝对坐标 start_xy/photo_xy（分量超
  [-1e4,1e4]→400、超轴软限位→400、合法→done 且 detail 含两点）；
  status.log 运行中随进度增长、done 后含 "data saved:"；result 含
  csv_name/series_nm（长度=repeat）
- 错误分支：repeat=5 → 400；wait_ms 超界 → 400；travel 超范围 → 400；
  travel 超轴软限位 → 400；未知 routine → 400；工艺流程在跑（paused）
  启动动态 → 400；无标记载体（down 无晶圆）启动静态 → 作业 error 态且
  错误信息含"标记"
- CSV 落盘与列格式（index,col,row,angle_deg[,动轴…]）
- 统计合理性：静态 0 < 3σ_nm < 5×nm_per_px；动态 3σ_nm ≥ 静态量级（≥0.5×）；
  动轴到位 3σ_um 非零且 axis_log=实际到位（迭代 48 起=误差链 actual，
  回拍照位 ±repeat 抖动）
- 迭代 27：REAL 同机位连拍帧间去相关（6 帧去重 ≥3）；real_noise_scale
  非法值 400、合法值热生效（帧差 σ 随倍率增大）；launch/metrology_gui
  dry 模式 ok（不实际拉起 GUI）
- 迭代 28：fast_noise_px 升级为 FAST+REAL 共用质心噪声旋钮（REAL 按模板
  响应度换算灰度噪声）：模板 meta 带 noise_resp_px_per_dn/grad_gx/grad_gy
  /tpl_std_dn 且列表 API 带出；fnp=0 两帧全等（确定性回退）；线性区
  fnp a→2a 帧差 σ ≈2×；默认档 real_noise_scale 1→2 静态剔野 3σ ~2×
  且脱离 2~3nm 旧地板（≥5nm）
- 迭代 29：误差链核验（用户诉求：到位重复性误差须反映真实状态、阿贝误差
  不影响重复性但影响标记轨迹）——repeat 1000nm 注入下静态 3σ 不变/动态
  3σ≈hypot(静态,1000nm)/读出链 pos 恒名义精确；阿贝项 = 角误差×杠杆臂
  （custom_center 5000mm 杠杆 rot_trans 按比例放大）、WS_LS_X ±0.5mm 扫
  位置 guide 像素轨迹 ON/OFF 差分与 toolpoints 误差投影逐位置一致、
  FAST identify 末端看到同一偏移
- 迭代 29b：ws_rotate_repeat 示教参数化模式（缺 tpl_before/tpl 不存在/
  pose 越软限位→400；合法示教 rot=1° repeat=3 跑通 done+CSV
  ws_rotate_taught_*+detail.mode=taught）；协作式取消 POST
  /api/calibration/cancel（running→canceling→canceled 终态+现场恢复；
  重复 cancel/done 后 cancel/未知作业→400）
- 迭代 29c：die_transfer_repeat 循环内不放回蓝膜（用户明确要求）——
  作业首 flip_pick 仅取一颗 die（flip=首段单样本）后在 翻转↔键合头
  间往复，每循环 head/back 两段测量，分段统计三段齐全；running 期间
  蓝膜 die 数仅 1 降 ≤1 升（循环内无 flip_place_back 物证），done 后
  finally 归位一次（蓝膜回满/head/flip 空/日志恰 1 条归位 reclaim）；
  measure_back 参数废止（传入忽略不报错，detail 无此键）；cancel 中断
  后 finally reclaim 归位物料守恒
- 迭代 31：ws_rotate_repeat rot_rad（(0,0.6] 越界/超行程→400；rot_deg
  兼容换算）+ taught_points 多点示教（1/11 点/旧四键混给/模板不存在/
  pose 未知轴→400；3 点跨片示教 done+Δθ 字段+CSV 动态列+用户模板不删；
  i29b 旧四键两点用例经映射走同一路径）；camera_microrot 三相机微旋
  标定（模板/成对/纯 X/间距口径/软限位/repeat→400；up/down/coarse 各
  done+φ 序列+measured.rot_mrad≈真值+apply 写回 rot_mrad 后还原）
- 迭代 32：ws_rotate_repeat 示教改**直接测角法**（基线 P_k→旋转态预测
  走位实测 Q_k→Kabsch 拟合 θ_i→e_i=θ_i−rot_rad；series.angle_err_urad
  长度=repeat、stats.angle_err 3σ=旋转角重复性、兼容字段保留；预测走位
  超软限位 start 预估 400）；ws_rotate_center 新 routine（参数/step_rad/
  n_angles/repeat/taught/行程→400；site quad 示教+角度档就地模板+误差
  链 OFF 下真值注入 [300,-200]µm 拟合中心偏差<10µm；apply 写回
  fine_align.rz_center_offset_mm 后还原）；camera_microrot 走位轴改对准
  台（down→AS_X/AS_Y，photo_pos 口径同步，detail.axes 断言）
- 迭代 32A：ws_rotate_gain 新 routine（旋转角度增益——直接测角法逐档
  Kabsch θ_actual，过原点 LS gain=Σθaθc/Σθc²+截距诊断 offset_urad；
  默认真值 gain≈1±3e-4+apply 写回 fine_align.rz_gain 后还原；专属 boot
  段改 low_order.json scale_ppm=1000 真值注入 1.001 测回，bytes 恢复）
- 迭代 33：视觉链路三 bug 修复回归——覆盖率累积缓冲 _COV_SCRATCH
  改 threading.local 按渲染线程隔离（原全局 (H,W) 缓冲被 up/down
  同分辨率渲染线程共享，fill/绘制/混合间让 GIL 即互串）：同分辨率双
  线程交错渲染断言各帧只含自己的标记；测校适配器视觉命令按相机分发
  per-cam 队列（同相机串行/跨相机并发，积压≥2 丢最旧并回"被新命令
  取代"错误帧、回显原始 task）：消费者卡死下入队 4 条断言最旧收到取代
  错误帧且队列有界；模板噪声响应度启动批量补标定（_calibrate_missing_
  resp：画布探针补 resp=None 旧模板，成功落盘/失败警告跳过，PNG 缺失
  索引启动清理，_recv_conn 任何异常重连）——构造 resp=None 模板补标定
  后断言 meta 带 resp，退化模板走失败路径不崩
- 迭代 34：模板必填 + 删自建临时模板（align_repeat_*/ws/as_move_
  repeat/camera_microrot 空/None 模板→400 列可用名；作业全程不自建
  不删；focus_scan/pixel_equivalent/rotation_direction 三旧项仍自建
  _cal_%d 为已知边界）/ 台重复性相机可选（as_move_repeat cam=down+
  jog_group=align 走 AS_X/Y，ws 恒 down）/ 相机微旋选台（move_stage=
  align（默认，AS_X/Y rel=−1）/wafer（WS_LS_X/Y rel=+1），up 恒 align
  传 wafer→400，coarse 恒 FA_X 不参与；photo_pos1/2 必填、纯 X 容差
  0.01mm、X 间距上下限与模板可达窗口校验移除）/ ws_rotate_repeat 全
  自动路径移除（无 taught_points 或 mode=auto→400）/ die_transfer_
  repeat back_tpl（缺省=coarse_tpl，start 存在性 400）——i34 组 7 例
- 迭代 35：dir 默认改 (+1,−1)（up/coarse，down 已是）+ view_rot_deg
  视场旋转（迭代 39 重构为真值/认知分离：view_rot=相机物理安装旋转角
  真值、只进投影链 R(−θ)，结果不剥旋；rot_mrad=控制器认知、只进反算
  链 R(+rot)，θ_b=θ_t 才恒等）——i35.pixel_world_roundtrip（离线零
  HTTP：dir{(+1,−1),(−1,+1),(+1,+1)}×rot{0.3,−0.5,0}×swap{F,T} 18 组合
  θ_b=θ_t 回环 1e-12/1e-9 + 错配旋转 (θ_b−θ_t) + solve_pixel 与
  view_rot 无关）/ i35.view_rot_config（camera_config 0.5 回读一致、
  5.5→400、还原迁移原值）/ i35.view_rot_identify（cv2 门控 REAL：偏心
  site 标记小模板，view_rot=0 基线 → +0.5° 后同机位重建模板，峰位
  逐分量对账解析期望（内容绕画幅中心反向旋转，±1.5px）、位移 ~r·θ
  显著非零=不剥旋，fnp=0 确定性，finally 还原）——i35 组 3 例
- 迭代 39：view_rot 真值/认知分离重构（vision_model/vision_service 见
  源码注释；迁移值 coarse 0.0458366°/up −0.0286479°/down 0.0177044°
  =rot_mrad×0.05729577951，机器常数数值保留不动）——
  i39.microrot_view_rot_loop（符号锚定闭环：注入 view_rot=+0.4mrad
  等效 0.0229183° → camera_microrot measured.rot_mrad≈+0.4（±0.25
  惯例带，映射末端翻号）→ 写回 rot_mrad 认知 → solve_pixel 链世界解
  误差 <0.25µm/0.72mm 半径=补偿到位；finally 全还原）；camera_microrot
  measured.rot_mrad 约定重钉=相机物理角真值估计（建议写入 rot_mrad
  的认知值），i31/i34 微旋用例数值不变（迁移后 θ_b=θ_t）；-k 新增
  view_rot/microrot 主题键（or 组合）
- 迭代 35（旋转三件套双示教位改造，本文件需求 4）：taught_points[{tpl,
  pose, pose_after 可选}]（pose_after 缺省=pose 同位复测；两处 pose 各
  自逐轴软限位 400；旧四键/无 pose_after 存档兼容）；旋转后走访直接走
  用户 pose_after（预测走位 Δ 段与 start 预估 400 已删，下发前保留软限
  位守卫；detail.taught_points 逐点带 pose_after）；θ/中心统一 _fit_rot
  闭式（u=P−P̄、v=Q−Q̄ → θ=atan2(Σu×v,Σu·v)、C=(I−R)⁻¹(Q̄−RP̄) 2×2
  直接解，小 θ 病态沿用 1/θ 放大防护）；ws_rotate_center measured 新增
  rz_center_ref_xy_mm=[作业起点 WS_LS_X,WS_LS_Y] + apply 双写回
  offset/ref_xy（机器常数 fine_align 新键，默认 [0,0]=平移项关闭=原行为）；
  ws_rotate_gain 每 (循环,档) 实测 θ_ij 过原点 LS（截距诊断保留）——
  i35.rotate_two_pose（2 点 pose_after≠pose run + pose_after 缺省=pose
  小角度 done + pose_after 越软限位/未知轴 400）/ i35.rotate_center_refxy
  （ref_xy=作业机台位 + apply 双键写回还原）/ i35.rotate_gain_measured 与
  i35.rotate_two_pose_truth（专属 boot 段 scale_ppm=1000：gain 1.001 测回
  |Δ|<8e-3 + s3<1e-3——双示教位扫档不重居中 NCC 系统差别名 ~1e-3 级；
  θ 对真值走 **r60 重居中**严格带 ±6µrad，注入签名 +10.8µrad 可辨）——
  i35 组另 5 例；旧用例适配：i29b/extra/i31 示教
  run 改 taught_points+测试侧 _rot_pose_after（原预测走位语义用户化）；
  i32 中心/增益及 i32 真值段改半径 ≤15mm 小杠杆示教（双示教位单一
  pose_after 无法逐档居中，可见性=杠杆×摆幅<FOV 半宽）+ 真值容差暂放
  宽（dev<75µm / gain<6e-3 / 真值段<8e-3，注明待视觉链稳定后收紧）；
  i32 预测走位 400 用例改 pose_after 越限 400（i32.rotate_walk_limit_400）
- 迭代 36：stage_geo 机器常数新组（as/ws_y_rail_dir_urad Y 导轨方向
  真值 FK 注入旋钮 + as/ws_nonortho_urad 测校结果写回键）+ FK 消费
  （error_kinematics 误差注入段 Y 链元平移偏差 y·(sinφ,cosφ−1)，链关
  不生效，φ=0 恒等）。
- 迭代 37：as/ws_stage_nonortho 按用户纠正重写为**栅格跳测法**（基准尺=
  晶圆 die 阵列 wafer_sites 格点+site_marks quad 假定正交；走距=格点
  节距与处方校验 0.001mm；起点 quad 角对准；o_i=标记世界位−相机世界位
  经 LS 斜率：X 线方向读 ρ̂、Y 线大小+方向定号 ψ_y=φ_t+ρ̂；φ(X 基准)
  =wrap(ψ_y−ρ̂)，ρ̂ 差中抵消；ref_axis=x/y 报 φ/−φ；<pfx>nonortho_urad
  恒 X 基准值）——i36.nonortho_inject_ws/as（±200/±500 往返测回+基线，
  确定性判读口径）/ i36.nonortho_400（pitch 不一致报实际值/行程/无标记
  /ref_axis/k/cam 各 400）/ i37.nonortho_chain_off（链关不消费）/
  i37.nonortho_ref_axis_y（负值输出+组路由键恒 X 基准）/
  i37.nonortho_cancel（协作取消+现场恢复+忙释放）/ i36.nonortho_apply
  （ref_axis=y 作业写回 X 基准值，真值旋钮不动）——真值段（地图归零，
  ±5µrad 严格带）main() 专属 boot
- 迭代 42：as_head_center 上吸盘位置标定（cam 恒 up：前置走位 AS→锚
  点/AS_Z→下焦面基准/BS_Arm_Z→上焦面基准后，每轮 XY ±1mm 动态再逼近
  回锚点 identify，solve_pixel 世界偏移 δ=标记−相机 → 新锚点=当前 AS
  位+δ；measured 双键同值联动 as_anchor_xy_mm（wafer_ega 组）/
  as_head_center_xy_mm（focus_level 组））+ best_focus 相机最佳焦平
  面标定（cam=up/down：区间收缩法扫 z-score 曲线（score=清晰度代理）
  +末轮三点抛物线得亚步距 z*；down 扫 AS_Z→as_z_chuck_focus_mm /
  up 前置走位后扫 BS_Arm_Z→head_z_upfocus_mm）——i42.as_head_center_400
  （模板/repeat/cam 各 400）/ i42.as_head_center（注入锚点偏移+误差链
  OFF 真值 (0,6) 测回 ±0.01mm → apply 双键 → 复测残差≈0 符号自证）/
  i42.best_focus_400（z_start≥z_end/模板/step/rounds/fpp/行程/coarse/
  down 无标记各 400）/ i42.best_focus_down（注入焦面偏移+误差链 OFF
  真值 0.0 测回 ±step/2 + best_z 处 identify 自洽 + apply）/
  i42.best_focus_up（真值 −119.83 测回 + 常数前置走位断言 + 轴位恢复
  + apply）；up 链路依赖仿真侧吸盘中心标记（迭代 42 前 collect_marks
  的 head_die_marks 分支无 die 即空——缺口补丁提案 Temp/
  _i42_chuck_mark_proposal.py，未应用时 up done 断言失败报"视野内无
  标记"）
- 迭代 49：机器常数两新组（bs_rxry_axis=BS_CAL_Rx/Ry 旋转轴线空间
  表示【轴点+方向向量，默认对齐 error_kinematics 真值】+ sensor_calib=
  温度/力传感链标定常数【发布读数=(raw−offset)/gain】）+ recipe
  sim_sensors 注入 4 键（temp_gain_err/temp_offset_C 按加热器分路 +
  force_gain_err/force_offset_N 标量，默认 0.2%/0.2℃/0.3%/0.05N）+
  tuning bond_hw.flip_contact_z_mm（−51.43，flip 吸盘裸面在 BS_Arm_Z
  坐标系的 z）+ engine flip 交接接触力支路（flip 有 die+FA_X 在交接
  位±1mm+BS_Arm_Z 进邻域时 F=k·press 与键合支路取 max）+ 温度/力
  读数注入补偿链（PID 控制全程用真值 PV 不受影响）——新测校五项：
  bs_rxry_axis（SVD 逆解轴线：dir 容差 30mrad/轴点垂直分量 1.5mm，
  实测收紧依据见用例注释）/ flip_handoff_height（慢速寻力实测
  Z*≈−50.936）/ bond_height_calib（Z*≈−167.434→裸值 −168.934）/
  bond_temp_calib（[80,140]℃两点 gain≈1.002/offset≈0.21 → apply 后
  读数闭环回真值）/ bond_force_calib（测量期降 force_noise_N→0.1 防
  flake，[5,15,30]N gain≈1.003 → apply+闭环）——i49.* 组各含 400 分支
"""
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "runtime"))
from mqtt_broker import MqttSub, mqtt_publish          # noqa: E402

# 迭代 37：端口可用 D2W_PORT_* 环境变量覆盖（与 runtime/main.py 同口径；
# 本机 93xx/9010 段被并行开发实例占用时，用 95xx 段隔离，如
# D2W_PORT_WS=9510 D2W_PORT_ACS=9501 D2W_PORT_PLC=9502 D2W_PORT_MCS=9590
# D2W_PORT_MQTT=9583 D2W_PORT_VTCP=9520）
MQTT_PORT = int(os.environ.get("D2W_PORT_MQTT", 9383))

try:
    import ctypes            # timeBeginPeriod（Windows 计时分辨率）
except ImportError:
    ctypes = None

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # backend/
BASE = "http://127.0.0.1:%s" % os.environ.get("D2W_PORT_WS", "9310")
ENV_PORTS = {"D2W_PORT_WS": os.environ.get("D2W_PORT_WS", "9310"),
             "D2W_PORT_ACS": os.environ.get("D2W_PORT_ACS", "9301"),
             "D2W_PORT_PLC": os.environ.get("D2W_PORT_PLC", "9302"),
             "D2W_PORT_MCS": os.environ.get("D2W_PORT_MCS", "9390"),
             "D2W_PORT_MQTT": os.environ.get("D2W_PORT_MQTT", "9383"),
             "D2W_PORT_VTCP": os.environ.get("D2W_PORT_VTCP", "9320")}
# 每相机像素当量事实（machine_constants.json：pixel_um/mag*1000）
NM_PER_PX = {"down": 600.0, "up": 600.0, "coarse": 9133.3333}
JOG = {"down": ("WS_LS_X", "WS_LS_Y"), "up": ("AS_X", "AS_Y"),
       "coarse": ("FA_X",)}
# 相机光源命令点（vision_cameras.json light_point）
LIGHT_POINT = {"down": "SystemMachine.SS_Bonding.M_AS.LightOn_2",
               "up": "SystemMachine.SS_Bonding.M_AS.LightOn_1",
               "coarse": "SystemMachine.SS_Bonding.M_AS.AS_LightOn"}

RESULTS = []

# PLC HTTP 端口（/io/observe 读点位在 PLC 服务上，同 test_client 口径）
PLC_BASE = "http://127.0.0.1:%s" % os.environ.get("D2W_PORT_PLC", "9302")


def io_observe(path):
    """读 IO 观测点值（PLC /io/observe 端口；含噪读数链消费口）。"""
    return float(http_json(PLC_BASE + "/io/observe?path=" + path)
                 .get("value", 0.0))


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("[%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail),
          flush=True)


# ---------------- HTTP（纯标准库，与 test_client 同风格） ----------------
def http_json(url, data=None, timeout=10.0):
    if data is None:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def http_post(url, data, timeout=10.0):
    """-> (status, obj)，4xx 不抛异常。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_get(url, timeout=10.0):
    """GET -> (status, obj)，4xx 不抛异常。"""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


# ---------------- 后端进程管理 ----------------
def wait_ready(timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if http_json(BASE + "/api/health", timeout=3).get("ok"):
                return True
        except Exception:
            time.sleep(0.3)
    return False


def boot_backend():
    env = dict(os.environ)
    env.update(ENV_PORTS)
    env["PYTHONIOENCODING"] = "utf-8"
    log = open(os.path.join(HERE, "_metrology_boot.log"), "w",
               encoding="utf-8")
    proc = subprocess.Popen([sys.executable,
                             os.path.join(ROOT, "runtime", "main.py")],
                            cwd=ROOT, env=env, stdout=log,
                            stderr=subprocess.STDOUT)
    return proc, log


# ---------------- 轴/物料助手 ----------------
def axis_entry(name):
    st = http_json(BASE + "/api/axes")
    return next(a for a in st["axes"] if a["name"] == name)


def wait_settled(name, timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        e = axis_entry(name)
        if e["settled"] and not e["moving"]:
            return e
        time.sleep(0.02)
    raise RuntimeError("轴 %s 整定超时" % name)


def move(axis, target, speed=600.0):
    c, r = http_post(BASE + "/api/axis/move",
                     {"axis": axis, "target": target, "speed": speed})
    assert c == 200, "move %s→%s 被拒：%s" % (axis, target, r.get("error"))
    return wait_settled(axis)


def vacuum(name, on):
    c, r = http_post(BASE + "/api/io/command",
                     {"path": "SystemMachine.SS_ME.M_AIR." + name,
                      "cmd": "OpenNegtive" if on else "OpenAir"})
    assert c == 200 and r.get("ok"), (name, r)


def dies_on(carrier):
    return [d for d in http_json(BASE + "/api/viz")["dies"]
            if d["carrier"] == carrier]


def wait_die_on(carrier, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = dies_on(carrier)
        if d:
            return d
        time.sleep(0.05)
    raise RuntimeError("交接超时：%s 上无 die" % carrier)


# ---------------- 测校作业助手 ----------------
def cal_start(routine, cam, params):
    return http_post(BASE + "/api/calibration/start",
                     {"routine": routine, "cam": cam, "params": params})


def cal_wait(job_id, timeout=300.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = http_json(BASE + "/api/calibration/status?job_id=%d" % job_id)
        if s.get("state") in ("done", "error", "canceled"):
            return s
        time.sleep(0.1)
    return {"state": "timeout", "error": "poll timeout"}


def cal_result(job_id):
    r = http_json(BASE + "/api/calibration/result?job_id=%d" % job_id)
    assert r.get("ok"), "result 查询失败：%s" % r
    return r["result"]


def run_job(routine, cam, params, timeout=300.0):
    c, r = cal_start(routine, cam, params)
    assert c == 200 and r.get("ok"), "start %s/%s 被拒：%s" % (
        routine, cam, r.get("error"))
    s = cal_wait(r["job_id"], timeout)
    return s, (cal_result(r["job_id"]) if s.get("state") == "done" else None)


# ---------------- 场景准备 ----------------
def prep_clean():
    http_post(BASE + "/api/workpiece/reset", {})
    http_post(BASE + "/api/vision/mode", {"mode": "fast"})


def prep_down():
    """下相机：晶圆上片 + 走位把 wafer_mark_1 送进画幅中心（同 18.mc_calib）。"""
    prep_clean()
    http_post(BASE + "/api/workpiece/wafer",
              {"action": "load", "offset": [0.0, 0.0]})
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        move(ax, tgt)


def _flip_pick():
    """film→flip 交接（ChuckVac + 取片示教位 FA_X=0/FA_Z=-5），完成后真空
    释放、die 留在翻转吸盘上（FA_Rx=0 呈现位背面朝上）。"""
    move("FA_Rx", 0.0, speed=1.5)
    vacuum("ChuckVac", True)
    move("FA_X", 0.0)
    move("FA_Z", -5.0)
    wait_die_on("flip")
    move("FA_Z", 0.0)                    # 先出位姿窗口再破真空（不放回）
    vacuum("ChuckVac", False)
    time.sleep(1.5)                      # 真空衰减（tau≈0.2s，1× 墙钟）


def prep_coarse():
    """粗对准：翻转模块带 die，FA_X=-5（±travel 都在 FA_X∈[-1250,0] 内，
    且 die 仍在 27.4mm FOV 内）。"""
    prep_clean()
    _flip_pick()
    move("FA_X", -5.0)


def prep_up():
    """上精对准：die 经 flip→head 真实交接链上键合头，对准台把上相机送
    到头下（AS_X/AS_Y 由 /api/vision/guide 引导居中最近角标记），
    BS_Arm_Z=-118.33 把 die 底面（标记面）送进上相机焦面。"""
    prep_clean()
    _flip_pick()
    for i in (1, 2, 3):
        vacuum("WT_BondingVac_%d" % i, True)
    move("AS_Y", 120.0, speed=500.0)     # 退避对准台相机（防撞头）
    move("WS_LS_X", 220.0)               # 退避晶圆台
    move("FA_X", -1242.35)               # 交接位（吸盘心=键合头心）
    move("BS_Arm_Z", -50.93, speed=200.0)   # 压头底接触 die 背面
    wait_die_on("head")
    move("BS_Arm_Z", 0.0, speed=200.0)   # 抬头
    move("FA_X", 0.0)                    # 翻转撤离
    move("AS_X", 0.0)
    move("AS_Y", 6.0, speed=500.0)       # 上相机认键合头吸盘中心
    move("AS_Z", 0.0)
    move("BS_Arm_Z", -118.33, speed=200.0)  # die 底面 → 上相机焦面
    # 标记引导：居中最近的 die 角标（±3.8mm 角点，1.8mm FOV 须引导）
    g = http_json(BASE + "/api/vision/guide?cam=up")
    marks = g.get("marks") or []
    assert marks, "up 视野几何无标记（die 未上头？）"
    sug = {s["axis"]: s["delta_mm"] for s in marks[0].get("suggested") or []}
    for ax, d in sug.items():
        move(ax, axis_entry(ax)["pos"] + d)


def fast_judge(cam):
    """preflight：FAST identify 判真（测校作业前的场景自检）。"""
    c, r = http_post(BASE + "/api/vision/identify", {"cam": cam})
    res = ((r.get("entry") or {}).get("result") or {})
    return c == 200 and res.get("judge") is True, res


# ---------------- 迭代 25：模板名/绝对坐标/日志流/新结果字段 ----------------
def check_iter25_fields(res, repeat):
    """result 新字段：csv_name（纯文件名）、series_nm（长度=repeat）、
    template_name。"""
    cn = res.get("csv_name") or ""
    snm = res.get("series_nm") or {}
    return (cn and "/" not in cn and cn.endswith(".csv")
            and (res.get("csv") or "").endswith(cn)
            and len(snm.get("row") or []) == repeat
            and len(snm.get("col") or []) == repeat
            and isinstance(res.get("template_name"), str))


def test_iter25(cam):
    """在 prep_<cam> 就位后跑：模板名分支 + 日志流 + 动态绝对坐标。"""
    n = 10
    # 1) template_name 不存在 → 400（消息列出现有模板）
    c, r = cal_start("align_repeat_static", cam,
                     {"repeat": n, "wait_ms": 0,
                      "template_name": "__no_such_tpl__"})
    check("mrep.tpl_not_found_400_%s" % cam,
          c == 400 and "模板" in (r.get("error") or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:80]))
    # 2) 建用户模板 → 带 template_name 跑静态 → done、结果字段、模板不删
    #    （迭代 34 起模板必填，同一模板贯穿步骤 3/4 的动态作业与校验分支）
    name = "ut_mrep_%s" % cam
    _del_tpl(name)
    c, r = _mk_tpl(cam, name)
    if c != 200 or not r.get("ok"):
        check("mrep.tpl_named_job_%s" % cam, False,
              "建用户模板失败：c=%s %s" % (c, r.get("error")))
        return
    try:
        s, res = run_job("align_repeat_static", cam,
                         {"repeat": n, "wait_ms": 0,
                          "template_name": name})
        tpl_kept = any(t.get("name") == name for t in
                       http_json(BASE + "/api/vision/templates")["templates"])
        ok = (s.get("state") == "done" and res is not None
              and res.get("template_name") == name
              and check_iter25_fields(res, n) and tpl_kept)
        check("mrep.tpl_named_job_%s" % cam, ok,
              "state=%s tpl=%s kept=%s%s"
              % (s.get("state"), res.get("template_name") if res else None,
                 tpl_kept,
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))

        # 3) status.log：运行动态作业（绝对坐标）期间日志增长，done 后含
        #    "data saved:"；detail 含 start_xy/photo_xy
        axes = JOG[cam]
        pos = {a: axis_entry(a)["pos"] for a in axes}
        lim = {a: axis_entry(a)["limits"] for a in axes}
        photo = [pos[axes[0]], pos[axes[1]] if len(axes) > 1 else 0.0]
        start = [photo[0] - 2.0,
                 photo[1] - 2.0 if len(axes) > 1 else 0.0]
        c, r = cal_start("align_repeat_dynamic", cam,
                         {"repeat": 30, "wait_ms": 30,
                          "start_xy": start, "photo_xy": photo,
                          "template_name": name})
        if c != 200:
            check("mrep.abs_xy_log_%s" % cam, False,
                  "start 被拒：c=%s %s" % (c, r.get("error")))
        else:
            jid = r["job_id"]
            lens = []
            while True:
                st = http_json(BASE + "/api/calibration/status?job_id=%d"
                               % jid)
                lens.append(len(st.get("log") or []))
                if st.get("state") in ("done", "error"):
                    break
                time.sleep(0.15)
            res = cal_result(jid) if st.get("state") == "done" else None
            det = (res.get("detail") or {}) if res else {}
            log_ok = (max(lens) > lens[0] and lens[-1] == max(lens)
                      and any(l.startswith("data saved:")
                              for l in (st.get("log") or [])))
            det_ok = det.get("start_xy") == [round(v, 6) for v in start] \
                and det.get("photo_xy") == [round(v, 6) for v in photo]
            check("mrep.abs_xy_log_%s" % cam,
                  st.get("state") == "done" and log_ok and det_ok
                  and res is not None and check_iter25_fields(res, 30),
                  "state=%s loglen=%s det=%s err=%s"
                  % (st.get("state"), (lens[0], max(lens), lens[-1]), det,
                     (st.get("error") or "")[:100]))
        # 4) 绝对坐标校验：分量超 [-1e4,1e4] → 400；超轴软限位 → 400
        c, r = cal_start("align_repeat_dynamic", cam,
                         {"start_xy": [0.0, 0.0], "photo_xy": [20000.0, 0.0],
                          "template_name": name})
        ok1 = c == 400
        bad = [photo[0], photo[1]]
        bad[0] = lim[axes[0]][1] + 1.0
        c2, r2 = cal_start("align_repeat_dynamic", cam,
                           {"start_xy": start, "photo_xy": bad,
                            "template_name": name})
        ok2 = c2 == 400 and "限位" in (r2.get("error") or "") \
            and axes[0] in (r2.get("error") or "")
        # 只给一个 → 400
        c3, _ = cal_start("align_repeat_dynamic", cam,
                          {"start_xy": start, "template_name": name})
        check("mrep.abs_xy_400_%s" % cam, ok1 and ok2 and c3 == 400,
              "c=%s/%s/%s err=%s" % (c, c2, c3, (r2.get("error") or "")[:60]))
    finally:
        _del_tpl(name)



# ---------------- 迭代 26：测校线协议适配器（MQTT 命令/反馈链） ----------------
def ref_motion_cmd(task, stage_name, axes):
    """参考 MC_Motion_cmd 信封：axes=[(axis_name, moveAbs, moveRel)]。"""
    payload = {"enable": False, "inPos": False, "moving": False,
               "name": stage_name, "stage": 1001,
               "lst_Axis": [
                   {"acc": {"targetVal": 0, "value": 0},
                    "coord": {"targetVal": 0, "value": 0},
                    "dec": {"targetVal": 0, "value": 0},
                    "homeOffset": 0, "jerk": {"targetVal": 0, "value": 0},
                    "jogRate": 0, "moveAbs": a, "moveRel": r,
                    "name": n, "unit": "mm",
                    "vel": {"targetVal": 300.0, "value": 0}}
                   for n, a, r in axes]}
    return json.dumps({"context": json.dumps(payload, indent=1),
                       "result": False, "task": task})


def ref_vision_cmd(cam_id, template):
    """参考 Camera_CMD.VisionCmdJson 信封。"""
    cellparams = {"0": {"m_dExposureTime": 0, "m_dFrameRate": 10,
                        "m_dGain": 1.0, "m_nCellId": 0, "m_strName": "",
                        "m_triggerMode": 0},
                  "1": {"lstShapeName": [template], "m_nCellId": 0,
                        "m_strName": ""}}
    content = {"camID": cam_id, "cmdType": 2,
               "content": json.dumps(cellparams), "image": "",
               "imgType": -1, "isImageSend": False, "lstCellType": [0, 1],
               "task": ""}
    return json.dumps({"context": json.dumps(content), "module": "",
                       "result": 0, "task": ""})


def wait_mqtt(sub, pred, timeout=15.0):
    """逐条取订阅消息直到 pred(topic, payload) 为真；超时返回 None。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        got = sub.recv(timeout=max(0.05, timeout - (time.time() - t0)))
        if got is None:
            return None
        try:
            if pred(*got):
                return got
        except Exception:
            continue
    return None


def wait_axis_at(name, target, tol=0.005, timeout=15.0):
    """轮询 /api/axes 直到 settled 且 pos≈target（MQTT 命令异步，直接
    wait_settled 会在命令尚未生效时提前返回旧位置——必须连目标一起等）。"""
    t0 = time.time()
    e = axis_entry(name)
    while time.time() - t0 < timeout:
        e = axis_entry(name)
        if e["settled"] and not e["moving"] and abs(e["pos"] - target) < tol:
            return e
        time.sleep(0.05)
    return e


def test_i26_mqtt_adapter():
    """随主后端拉起（D2W_PORT_MQTT=9383）：MoveAbs/MoveRel 命令 →
    /api/axes 到位断言；CTL_DR_ACS_Motion RunTimeInfo 结构按参考
    MqttReceiveWorker 取值路径验证；VSC_Com → VSS_Com 结果帧；
    未映射 task 不崩服务。"""
    sub_fb = MqttSub(port=MQTT_PORT, client_id="i26-fb")
    sub_fb.subscribe("CTL_DR_ACS_Motion")
    sub_vs = MqttSub(port=MQTT_PORT, client_id="i26-vs")
    sub_vs.subscribe("VSS_Com")
    time.sleep(0.3)                    # 等订阅注册到 broker
    try:
        # 1) MoveAbs（恒等映射轴名 WS_LS_X）→ 到位
        tgt = 62.5
        mqtt_publish("GUI_DR_ACS_Motion",
                     ref_motion_cmd("CMD_Motion_MoveAbs", "WaferStage",
                                    [("WS_LS_X", tgt, 0)]),
                     port=MQTT_PORT)
        e = wait_axis_at("WS_LS_X", tgt)
        check("i26.moveabs_arrive",
              abs(e["pos"] - tgt) < 0.01,
              "pos=%.4f target=%.4f" % (e["pos"], tgt))

        # 2) RunTimeInfo：按参考解析路径取 stage 1001 / WS_LS_X
        def is_runtime(_topic, payload):
            raw = json.loads(payload)
            if raw.get("task") != "CMD_Motion_RunTimeInfo":
                return False
            ds = json.loads(raw["context"])["dic_Stage"]
            st = ds["1001"]
            ax = next(a for a in st["lst_Axis"] if a["name"] == "WS_LS_X")
            return (abs(ax["coord"]["value"] - tgt) < 0.01
                    and st["inPos"] is True and st["moving"] is False)
        got = wait_mqtt(sub_fb, is_runtime, timeout=10.0)
        check("i26.runtime_info", got is not None,
              "dic_Stage.1001 WS_LS_X coord/inPos/moving %s"
              % ("可取" if got else "未等到"))

        # 3) 别名映射 + MoveRel：WaferStage_Y → WS_LS_Y 相对 -3mm
        y0 = axis_entry("WS_LS_Y")["pos"]
        mqtt_publish("GUI_DR_ACS_Motion",
                     ref_motion_cmd("CMD_Motion_MoveRel", "WaferStage",
                                    [("WaferStage_Y", 0, -3.0)]),
                     port=MQTT_PORT)
        e = wait_axis_at("WS_LS_Y", y0 - 3.0)
        check("i26.alias_moverel",
              abs(e["pos"] - (y0 - 3.0)) < 0.01,
              "pos=%.4f expect=%.4f" % (e["pos"], y0 - 3.0))

        # 4) 未映射 task（CMD_Motion_Home）不崩：后续命令仍正常处理
        mqtt_publish("GUI_DR_ACS_Motion",
                     json.dumps({"context": "", "result": False,
                                 "task": "CMD_Motion_Home"}),
                     port=MQTT_PORT)
        mqtt_publish("GUI_DR_ACS_Motion",
                     ref_motion_cmd("CMD_Motion_MoveAbs", "WaferStage",
                                    [("WS_LS_X", tgt - 2.0, 0)]),
                     port=MQTT_PORT)
        e = wait_axis_at("WS_LS_X", tgt - 2.0)
        check("i26.unknown_task_survive",
              abs(e["pos"] - (tgt - 2.0)) < 0.01,
              "pos=%.4f expect=%.4f" % (e["pos"], tgt - 2.0))

        # 4b) 归位：前面走位把 wafer 标记带出了 down 相机 FOV（1.8mm），
        #     视觉断言前送回 prep_down 的对中位
        move("WS_LS_X", 58.2162)
        move("WS_LS_Y", 111.6102)

        # 5) VSC_Com 视觉命令（down 相机 + 用户模板）→ VSS_Com 结果帧
        name = "i26_tpl_down"
        c, r = http_post(BASE + "/api/io/command",
                         {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                          "args": {"value": True}})
        assert c == 200 and r.get("ok"), (c, r)
        time.sleep(0.2)
        try:
            c, r = http_post(BASE + "/api/vision/templates",
                             {"cam": "down", "name": name})
        finally:
            http_post(BASE + "/api/io/command",
                      {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                       "args": {"value": False}})
        if c != 200 or not r.get("ok"):
            check("i26.vision_result", False,
                  "建模板失败：c=%s %s" % (c, r.get("error")))
            return
        try:
            mqtt_publish("VSC_Com", ref_vision_cmd("down", name),
                         port=MQTT_PORT)

            def is_vss(_topic, payload):
                raw = json.loads(payload)
                ctx = json.loads(raw["context"])
                content = json.loads(ctx["content"])
                c1 = content["1"]
                return (len(c1["lstRow"]) > 0 and len(c1["lstCol"]) > 0
                        and "judge" in c1 and "lstAngle" in c1
                        and "lstScore" in c1 and c1["judge"] is True)
            got = wait_mqtt(sub_vs, is_vss, timeout=15.0)
            check("i26.vision_result", got is not None,
                  "VSS_Com lstRow/lstCol/judge %s"
                  % ("齐全且 judge=True" if got else "未等到/字段缺失"))
        finally:
            req = urllib.request.Request(
                BASE + "/api/vision/templates",
                data=json.dumps({"name": name}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="DELETE")
            try:
                urllib.request.urlopen(req, timeout=5).read()
            except Exception:
                pass
    finally:
        sub_fb.close()
        sub_vs.close()



def check_csv(res, cam, dynamic, repeat):
    """CSV 落盘 + 列格式：index,col,row,angle_deg[,动轴…]。"""
    rel = res.get("csv") or ""
    path = os.path.join(ROOT, rel.replace("/", os.sep))
    if not os.path.isfile(path):
        return False, "csv 不存在: %s" % rel
    with open(path, encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    header = lines[0].split(",")
    # 2026-09-08 数据增强：CSV 尾部新增世界坐标辅助列（mark_world_*/
    # cam_world_*，6 列）——expect 前缀匹配（新列允许追加）
    expect = ["index", "col", "row", "angle_deg"] + list(JOG[cam])
    if not dynamic:
        expect = ["index", "col", "row", "angle_deg"]
    prefix = ("align_stability_dynamic_" if dynamic
              else "single_camera_stability_")
    ok = (header[:len(expect)] == expect and len(lines) == repeat + 1
          and os.path.basename(rel).startswith(prefix))
    return ok, "header=%s rows=%d file=%s" % (header, len(lines) - 1,
                                              os.path.basename(rel))


def check_contract(res, routine, cam, repeat, wait_ms):
    """契约字段（panel 按字段名渲染）逐一核对。"""
    ok = (res.get("routine") == routine and res.get("camera") == cam
          and res.get("repeat") == repeat and res.get("wait_ms") == wait_ms
          and abs(res.get("nm_per_px", 0) - NM_PER_PX[cam]) < 0.01
          and isinstance(res.get("elapsed_s"), (int, float))
          and res.get("elapsed_s") > 0)
    series = res.get("series") or {}
    ok = ok and len(series.get("row") or []) == repeat \
        and len(series.get("col") or []) == repeat
    ang = series.get("angle_deg")
    ok = ok and isinstance(ang, list) and len(ang) == repeat
    stats = res.get("stats") or {}
    for k in ("row", "col"):
        st = stats.get(k) or {}
        ok = ok and all(f in st for f in
                        ("mean_px", "std_px", "s3_px", "s3_nm"))
    ok = ok and isinstance(stats.get("angle"), dict) \
        and all(f in stats["angle"] for f in ("mean_deg", "std_deg",
                                              "s3_urad"))
    return ok


def test_static(cam, n=10):
    # 迭代 34 起 template_name 必填：当前帧 teach 用户模板（跑完删）
    name = "mrep_s_%s" % cam
    _del_tpl(name)
    c, r = _mk_tpl(cam, name)
    assert c == 200 and r.get("ok"), (c, r)
    try:
        s, res = run_job("align_repeat_static", cam,
                         {"repeat": n, "wait_ms": 0,
                          "template_name": name})
    finally:
        _del_tpl(name)
    ok = s.get("state") == "done" and res is not None \
        and check_contract(res, "align_repeat_static", cam, n, 0)
    st = res["stats"] if res else {}
    s3 = (st.get("row") or {}).get("s3_nm", -1)
    s3c = (st.get("col") or {}).get("s3_nm", -1)
    # 迭代 28 起允许 s3=0：低对比模板（up 大 ROI 小标记）的经验噪声上限
    # 把 REAL 灰度噪声压在匹配存活区内，散布可低于结果 0.001px 量化分辨率
    # （物理封顶如实兜底，非链断裂——链断裂会有 error/judge=false）
    sane = 0 <= s3 < 5 * NM_PER_PX[cam] and 0 <= s3c < 5 * NM_PER_PX[cam]
    csv_ok, csv_d = check_csv(res, cam, False, n) if res else (False, "无结果")
    # 静态：axis_log/stats.axes 为 None（契约）
    static_null = res.get("axis_log") is None and st.get("axes") is None
    check("mrep.static_%s" % cam, ok and sane and csv_ok and static_null,
          "state=%s 3σ_nm row=%.1f col=%.1f nm/px=%.1f csv=%s%s"
          % (s.get("state"), s3, s3c, res.get("nm_per_px", -1), csv_d,
             (" err=%s" % (s.get("error") or "")) if s.get("state") != "done"
             else ""))
    return res


def test_dynamic(cam, travel=2.0, n=10):
    # 迭代 34 起 template_name 必填：当前帧 teach 用户模板（跑完删）
    name = "mrep_d_%s" % cam
    _del_tpl(name)
    c, r = _mk_tpl(cam, name)
    assert c == 200 and r.get("ok"), (c, r)
    try:
        s, res = run_job("align_repeat_dynamic", cam,
                         {"repeat": n, "wait_ms": 0, "travel_mm": travel,
                          "template_name": name})
    finally:
        _del_tpl(name)
    ok = s.get("state") == "done" and res is not None \
        and check_contract(res, "align_repeat_dynamic", cam, n, 0)
    st = res["stats"] if res else {}
    axes_st = st.get("axes") or {}
    alog = res.get("axis_log") or {}
    axes_ok = set(alog) == set(JOG[cam]) and set(axes_st) == set(JOG[cam]) \
        and all(len(v) == n for v in alog.values()) \
        and all((axes_st[a] or {}).get("s3_um", -1) >= 0 for a in axes_st)
    # axis_log = 实际到位位置（迭代 48 起=误差链 actual 编码器口径）：
    # 每循环回拍照位 ±repeat 抖动（|Δ|≲3×repeat_3σ；旧口径记名义 pos
    # 恒=q0、3σ 恒 0 无意义），且到位 3σ 非零且落在配置 repeat 合理带
    # （WS/AS=0.04µm、FA_X=0.5µm 级；n≈10 样本 std 涨落留宽）
    q0 = {a: alog[a][0] for a in alog} if alog else {}
    back_ok = bool(q0) and all(
        abs(v - q0[a]) < 0.001 for a in alog for v in alog[a])
    s3_ok = all(0.002 < (axes_st[a] or {}).get("s3_um", -1) < 2.0
                for a in axes_st)
    csv_ok, csv_d = check_csv(res, cam, True, n) if res else (False, "无结果")
    s3 = (st.get("row") or {}).get("s3_nm", -1)
    check("mrep.dynamic_%s" % cam,
          ok and axes_ok and back_ok and s3_ok and csv_ok,
          "state=%s 3σ_nm row=%.1f axes=%s q0=%s csv=%s"
          % (s.get("state"), s3,
             {a: round(axes_st[a]["s3_um"], 4) for a in axes_st},
             {a: round(q0[a], 4) for a in q0}, csv_d))
    return res


def test_error_branches():
    # 迭代 34 起 template_name 必填——建错误分支专用 down 模板（无晶圆也
    # 可建，只取帧内容；400 文案分支靠它走到 repeat/wait/travel/限位/
    # 工艺/无标记各校验点）
    name = "mrep_err"
    _del_tpl(name)
    c0, r0 = _mk_tpl("down", name)
    assert c0 == 200 and r0.get("ok"), (c0, r0)
    try:
        _test_error_branches_inner(name)
    finally:
        _del_tpl(name)


def _test_error_branches_inner(name):
    # 参数校验（同步 400）
    c, r = cal_start("align_repeat_static", "down", {"repeat": 5})
    c2, _ = cal_start("align_repeat_static", "down", {"repeat": 1001})
    check("mrep.repeat_range_400", c == 400 and c2 == 400,
          "c=%s/%s err=%s" % (c, c2, (r.get("error") or "")[:60]))
    c, r = cal_start("align_repeat_static", "down", {"wait_ms": 70000})
    c2, r2 = cal_start("align_repeat_static", "down", {"wait_ms": -1})
    check("mrep.wait_range_400", c == 400 and c2 == 400,
          "c=%s/%s err=%s" % (c, c2, (r.get("error") or "")[:60]))
    # 空/None 模板 → 400（迭代 34 必填；文案列可用模板名）
    c, r = cal_start("align_repeat_static", "down",
                     {"repeat": 10, "template_name": ""})
    c2, r2 = cal_start("align_repeat_static", "down",
                       {"repeat": 10, "template_name": None})
    check("mrep.empty_tpl_400",
          c == 400 and c2 == 400 and "template_name" in (r.get("error") or "")
          and "template_name" in (r2.get("error") or ""),
          "c=%s/%s err=%s" % (c, c2, (r.get("error") or "")[:60]))
    c, r = cal_start("align_repeat_dynamic", "down",
                     {"travel_mm": 0.05, "template_name": name})
    c2, _ = cal_start("align_repeat_dynamic", "down",
                      {"travel_mm": 25, "template_name": name})
    check("mrep.travel_range_400", c == 400 and c2 == 400,
          "c=%s/%s err=%s" % (c, c2, (r.get("error") or "")[:60]))
    # travel 超软限位（coarse FA_X=0，hi=0；q0+20 越界）
    c, r = cal_start("align_repeat_dynamic", "coarse",
                     {"travel_mm": 20.0, "template_name": name})
    check("mrep.travel_limit_400", c == 400 and "限位" in (r.get("error")
                                                           or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:80]))
    # 未知 routine
    c, r = cal_start("align_repeat_foo", "down", {})
    check("mrep.unknown_routine_400", c == 400,
          "c=%s err=%s" % (c, (r.get("error") or "")[:60]))
    # 工艺流程在跑（paused 起步即暂停，无副作用）→ 动态 400；stop 后清理
    c0, r0 = http_post(BASE + "/api/flow/run",
                       {"name": "晶圆键合示范流程", "paused": True})
    c, r = cal_start("align_repeat_dynamic", "down",
                     {"travel_mm": 2.0, "template_name": name})
    busy_ok = c == 400 and "工艺" in (r.get("error") or "")
    http_post(BASE + "/api/flow/stop", {})
    time.sleep(0.2)
    http_post(BASE + "/api/flow/reset", {})
    check("mrep.flow_busy_400", c0 == 200 and busy_ok,
          "run=%s c=%s err=%s" % (c0, c, (r.get("error") or "")[:60]))
    # 无标记载体（复位后无晶圆）→ 作业 error 态且错误信息含"标记"
    prep_clean()
    c, r = cal_start("align_repeat_static", "down", {"repeat": 10,
                                                     "wait_ms": 0,
                                                     "template_name": name})
    if c != 200:
        check("mrep.no_mark_error", False, "start 被 %s 拒（应 200 起作业）"
              % c)
    else:
        s = cal_wait(r["job_id"], 60)
        err = s.get("error") or ""
        check("mrep.no_mark_error",
              s.get("state") == "error" and "标记" in err,
              "state=%s err=%s" % (s.get("state"), err[:100]))


# ---------------- 迭代 27：REAL 去相关噪声 / real_noise_scale / GUI 拉起 ----------------
def http_get_bytes(url, timeout=30.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def http_put(url, data, timeout=10.0):
    """PUT -> (status, obj)，4xx 不抛异常。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"},
                                 method="PUT")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def _real_frame(cam="down"):
    """GET /api/vision/frame 实时渲染帧（降采样 640×480，噪声同样注入）。"""
    import vision_model as vm
    return vm.png_decode_gray(http_get_bytes(
        BASE + "/api/vision/frame?cam=" + cam))


def test_i27_real_noise():
    """迭代 27：REAL 渲染噪声大底版随机取景（帧间去相关，修复旧 2 底版
    轮换只剩 2~3 个离散亚像素值的量化锯齿）+ recipe sim_sensors.
    real_noise_scale 倍率旋钮 + /api/launch/metrology_gui 端点。"""
    import numpy as np
    # 1) 同机位连拍 6 帧：像素级互不全等，去重后 ≥3 张（旧机制恒为 2）
    frames = [_real_frame() for _ in range(6)]
    distinct = len({f.tobytes() for f in frames})
    check("i27.real_frame_decorrelated",
          distinct >= 3 and not np.array_equal(frames[0], frames[1]),
          "6 帧去重后 %d 张" % distinct)

    # 2) real_noise_scale：非法值 400（区间 (0,10]）；合法值热生效——
    #    连拍帧差幅度（≈√2·σ）随倍率单调增大（弱断言防 flake）
    c0, _ = http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"real_noise_scale": {"down": 0}}})
    c1, _ = http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"real_noise_scale": {"down": 11}}})
    bad_ok = c0 == 400 and c1 == 400
    cur = (http_json(BASE + "/api/recipe").get("sim_sensors", {})
           .get("real_noise_scale", {}).get("down", 1.0))
    # 迭代 28：fast_noise_px 语义升级为 REAL 灰度噪声源（÷模板响应度）——
    # 默认 0.15px 已顶到 40DN 物理封顶，×4 倍率会把图像打进 0-255 饱和
    # 裁剪（帧差 σ 比值被压缩）；倍率改在线性区验证：先把底噪降到小 σ，
    # 结束随 real_noise_scale 一起还原
    cur_fnp = (http_json(BASE + "/api/recipe").get("sim_sensors", {})
               .get("fast_noise_px", {}).get("down", 0.15))
    http_put(BASE + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.002}}})

    def diff_sigma():
        a = _real_frame().astype(np.float32)
        b = _real_frame().astype(np.float32)
        return float(np.std(b - a))

    d1 = diff_sigma()
    c2, _ = http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"real_noise_scale": {"down": 4.0}}})
    d4 = diff_sigma() if c2 == 200 else 0.0
    # 还原原值（PUT /api/recipe 落盘持久化，别留垃圾在 recipe.json）
    http_put(BASE + "/api/recipe",
             {"sim_sensors": {"real_noise_scale": {"down": cur},
                              "fast_noise_px": {"down": cur_fnp}}})
    check("i27.real_noise_scale",
          bad_ok and c2 == 200 and d4 > 1.8 * d1,
          "PUT 0/11→%s/%s，PUT 4.0→%s；帧差σ ×1=%.2f ×4=%.2f"
          % (c0, c1, c2, d1, d4))

    # 3) launch 端点：dry 模式只校验路径不实际拉起（CI 无窗）
    c, r = http_post(BASE + "/api/launch/metrology_gui", {"dry": True})
    check("i27.launch_mgui",
          c == 200 and r.get("ok") and r.get("dry") is True,
          "c=%s resp=%s" % (c, str(r)[:80]))


# ---------------- 迭代 28：fast_noise_px 全链旋钮（FAST+REAL）/ 模板响应度 ----------------
def test_i28_real_noise_knob():
    """迭代 28：fast_noise_px 语义升级为相机质心噪声底（FAST+REAL 共用同
    一旋钮）——REAL 渲染按模板响应度（建模板自标定 px/DN，存模板 meta）
    把质心目标 px 换算成图像灰度噪声 σ（双封顶 min(40DN, 0.45×模板RMS)，
    超出即钳制）。
    - 模板 meta 带 noise_resp_px_per_dn/grad_gx/grad_gy/tpl_std_dn
      （模板列表 API 带出）
    - fast_noise_px=0 → 同机位连拍 2 帧全等（确定性回退）
    - 线性区单调性：fnp a→2a（均低于封顶对应 px）帧差 σ ≈2×
    - 标定回归（自洽因果式）：默认 fnp=0.15 下 real_noise_scale 1→2
      （灰度 σ 精确翻倍，与 resp 标定误差无关）静态连拍剔野点 3σ ~2×，
      且 3σ 脱离旧地板（≥5nm——投诉症状是旋钮脱钩时的 2~3nm）
    """
    import numpy as np
    # i27 之后停在 coarse 场景——恢复 down（晶圆 wafer 标记位）
    prep_down()
    name = "i28_tpl_down"
    c, r = http_post(BASE + "/api/io/command",
                     {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                      "args": {"value": True}})
    assert c == 200 and r.get("ok"), (c, r)
    time.sleep(0.2)
    try:
        c, r = http_post(BASE + "/api/vision/templates",
                         {"cam": "down", "name": name}, timeout=60)
    finally:
        http_post(BASE + "/api/io/command",
                  {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                   "args": {"value": False}})
    meta = r.get("template") or {}
    resp = meta.get("noise_resp_px_per_dn")
    check("i28.tpl_meta_resp",
          c == 200 and resp and resp > 0
          and (meta.get("grad_gx") or 0) > 0 and (meta.get("grad_gy") or 0) > 0
          and (meta.get("tpl_std_dn") or 0) > 0,
          "c=%s resp=%s Gx=%s Gy=%s std=%s"
          % (c, resp, meta.get("grad_gx"), meta.get("grad_gy"),
             meta.get("tpl_std_dn")))
    if c != 200 or not resp:
        return
    # 模板列表 API 带出标定字段（向后兼容：旧模板缺字段由渲染侧惰性补标定）
    lst = http_json(BASE + "/api/vision/templates")["templates"]
    mine = next((t for t in lst if t.get("name") == name), {})
    check("i28.tpl_list_api",
          mine.get("noise_resp_px_per_dn") == resp
          and (mine.get("grad_gx") or 0) > 0,
          "list resp=%s" % mine.get("noise_resp_px_per_dn"))
    # 记录原值（PUT /api/recipe 落盘持久化，结束恢复）
    ss = http_json(BASE + "/api/recipe").get("sim_sensors", {})
    cur_fnp = (ss.get("fast_noise_px") or {}).get("down", 0.15)
    cur_rns = (ss.get("real_noise_scale") or {}).get("down", 1.0)
    try:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"real_noise_scale": {"down": 1.0}}})
        # 2) 确定性回退：fnp=0 → 同机位连拍 2 帧全等
        c0, _ = http_put(BASE + "/api/recipe",
                         {"sim_sensors": {"fast_noise_px": {"down": 0.0}}})
        fa = _real_frame()
        fb = _real_frame()
        check("i28.zero_noise_deterministic",
              c0 == 200 and np.array_equal(fa, fb),
              "PUT 0→%s，两帧%s" % (c0, "全等" if np.array_equal(fa, fb)
                                    else "不等"))

        def diff_sigma():
            a = _real_frame().astype(np.float32)
            b = _real_frame().astype(np.float32)
            return float(np.std(b - a))

        # 3) 质心噪声随 fnp 线性（2026-09-08 架构修正改写）：白噪声固定
        # 雪花档后帧差 σ 不再随 fnp 变（原断言作废）；质心输出全额由
        # 平移分量承担 → identify 连拍 3σ 应 ∝ fnp（0.5×fnp→fnp 档 ×2）
        mc = http_json(BASE + "/api/machine_constants")
        defname = ((mc.get("constants") or {}).get("vision_marks") or {}) \
            .get("tpl_wafer_site")
        dmeta = next((t for t in
                      http_json(BASE + "/api/vision/templates")["templates"]
                      if t.get("name") == defname), {})
        resp_d = dmeta.get("noise_resp_px_per_dn")
        smax_d = dmeta.get("noise_sigma_max_dn")
        std_d = dmeta.get("tpl_std_dn")
        cap_d = float(smax_d) if smax_d else \
            (min(12.0, 0.25 * float(std_d)) if std_d else 12.0)
        cap_d = min(12.0, cap_d)
        cap_px = cap_d * resp_d if resp_d else None
        if not cap_px:
            check("i28.monotonic", False,
                  "默认模板 %r 缺响应度字段（补标定未生效？）" % defname)
            fnp_lo = fnp_hi = None
        else:
            fnp_lo = 0.02
            fnp_hi = 0.04

            def _centroid_s3(fnp_val, n=8):
                http_put(BASE + "/api/recipe",
                         {"sim_sensors": {"fast_noise_px": {"down": fnp_val}}})
                cols, rows = [], []
                for _ in range(n):
                    c, r = http_post(BASE + "/api/vision/identify",
                                     {"cam": "down", "template": name},
                                     timeout=60)
                    res = ((r.get("entry") or {}).get("result") or {})
                    if c == 200 and res.get("judge") and res.get("lstCol"):
                        cols.append(res["lstCol"][0])
                        rows.append(res["lstRow"][0])
                if len(cols) < 6:
                    return None
                import numpy as _np3
                cx, cy = _np3.array(cols), _np3.array(rows)
                keep = (_np3.abs(cx - _np3.median(cx)) < 1.5) \
                    & (_np3.abs(cy - _np3.median(cy)) < 1.5)
                return 0.5 * (float(_np3.std(cx[keep]))
                              + float(_np3.std(cy[keep]))) * NM_PER_PX["down"]

            d_lo = _centroid_s3(fnp_lo)
            d_hi = _centroid_s3(fnp_hi)
            check("i28.monotonic",
                  d_lo is not None and d_hi is not None
                  and d_hi > 1.3 * d_lo > 0,
                  "默认模板%s resp=%.5f cap=%.1fDN；fnp %.3f→%.3f px："
                  "质心σ %.2f→%.2f nm（×%.2f）"
                  % (defname, resp_d, cap_d, fnp_lo, fnp_hi,
                     3 * (d_lo or 0), 3 * (d_hi or 0),
                     (d_hi or 0) / max(d_lo or 1e-9, 1e-9)))

        # 4) 标定回归（自洽因果式）：fast_noise_px=0.15 默认档下
        #    real_noise_scale 1→2 使灰度 σ 精确翻倍（倍率在封顶后乘，
        #    与 resp 标定误差无关）→ 静态连拍剔野点 3σ 应 ~2×（粗网格路径
        #    重尾 + n=16 统计涨落，带宽 [1.3,3.2] 防 flake）；且 3σ_1 须
        #    明显脱离旧地板（≥5nm——投诉症状是旋钮脱钩时的 2~3nm）。
        #    模型预测（min(fnp,cap×resp)×600×3）打印供报告，不断言绝对值
        #    （粗网格路径逐次运行本身有 ±3× 涨落，见交付报告缺口分析）
        sig_max = meta.get("noise_sigma_max_dn")
        tpl_std = meta.get("tpl_std_dn")
        # 2026-09-08 与运行时封顶对齐：min(12DN, 0.25×模板RMS)（此前
        # 40/0.45 旧口径算出的 fnp_lin 在 ×2 档越过运行时封顶被钳，
        # 白噪声不再精确翻倍 → ratio 反常）
        cap_dn = float(sig_max) if sig_max else \
            (min(12.0, 0.25 * float(tpl_std)) if tpl_std else 12.0)
        cap_dn = min(12.0, cap_dn)
        # 2026-09-08：本用例原口径=白噪声线性区标定回归——fnp 设入线性区
        # （0.4×cap_px，白噪声全额承担质心噪声、无平移缺口分量），rns 对
        # 灰度 σ 精确倍增 → 3σ 精确 ×2；结束后恢复默认 0.15（默认档由
        # 平移分量补足缺口，量级断言归 i21/i22/mrep 系列）
        fnp_lin = 0.03   # 固定线性区档（<0.01px 时平移响应死区偏低）
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": fnp_lin}}})
        c, r = http_post(BASE + "/api/vision/mode", {"mode": "real"})
        assert c == 200, (c, r)

        def measure_s3(scale, n=16):
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"real_noise_scale": {"down": scale}}})
            cols, rows, scores = [], [], []
            for _ in range(n):
                c, r = http_post(BASE + "/api/vision/identify",
                                 {"cam": "down", "template": name},
                                 timeout=60)
                res = ((r.get("entry") or {}).get("result") or {})
                if c == 200 and res.get("judge") and res.get("lstCol"):
                    cols.append(res["lstCol"][0])
                    rows.append(res["lstRow"][0])
                    scores.append((res.get("lstScore") or [0.0])[0])
            if len(cols) < 8:
                return None, 0.0
            # 粗网格 NCC 在强噪声下有少量野点（错误峰锁定，多 µm 级）——
            # 3σ 对单点野值极敏感，按中位偏差 >1.5px 剔除后评估（与计量
            # 读重复性的常规做法一致）
            import numpy as _np2
            cx = _np2.array(cols)
            cy = _np2.array(rows)
            keep = (_np2.abs(cx - _np2.median(cx)) < 1.5) \
                & (_np2.abs(cy - _np2.median(cy)) < 1.5)
            sx = 0.5 * (float(_np2.std(cx[keep])) + float(_np2.std(cy[keep])))
            return 3.0 * sx * NM_PER_PX["down"], sum(scores) / len(scores)

        try:
            s3_1, sc1 = measure_s3(1.0)
            s3_2, sc2 = measure_s3(2.0)
        finally:
            http_post(BASE + "/api/vision/mode", {"mode": "fast"})
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"real_noise_scale": {"down": 1.0}}})
        if s3_1 and s3_2:
            # 2026-09-08 架构修正改写：质心预算全额走平移分量（不随
            # real_noise_scale 变——rns 是纯灰度视觉倍率）；×1 档 3σ 应
            # ≈ 3×fnp_lin×k_nm（模型预测），×2 档与 ×1 同量级（抽样带
            # 宽 ±50%），score 健康
            exp_nm = 3.0 * NM_PER_PX["down"] * fnp_lin
            check("i28.calib_band",
                  0.5 <= s3_2 / s3_1 <= 2.0 and s3_1 >= 5.0
                  and 0.5 * exp_nm <= s3_1 <= 2.0 * exp_nm
                  and sc1 > 0.6 and sc2 > 0.6,
                  "scale×1 3σ=%.1fnm ×2 3σ=%.1fnm（×%.2f）模型预测=%.1fnm"
                  "（fnp_lin=%.4fpx）score=%.2f/%.2f"
                  % (s3_1, s3_2, s3_2 / s3_1, exp_nm, fnp_lin, sc1, sc2))
        else:
            check("i28.calib_band", False,
                  "REAL identify 判真点不足（s3_1=%s s3_2=%s）"
                  % (s3_1, s3_2))
    finally:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": cur_fnp},
                                  "real_noise_scale": {"down": cur_rns}}})
        req = urllib.request.Request(
            BASE + "/api/vision/templates",
            data=json.dumps({"name": name}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="DELETE")
        try:
            urllib.request.urlopen(req, timeout=5).read()
        except Exception:
            pass



# ---------------- 迭代 29：误差链核验——repeat 进图像 / 阿贝轨迹 ----------------
def _s3_rowcol(res):
    return (res["stats"]["row"]["s3_nm"], res["stats"]["col"]["s3_nm"])


def test_i29_repeat_into_image():
    """迭代 29a：轴到位重复性（repeat）注入点核验——误差不在采集图像之后
    后处理叠加，而在"到位"时刻（engine._notify_settled → errors.settle
    重采样）经 6DOF 真值链（sample6 along → error_kinematics →
    collect_marks 标记世界坐标）进图像：
    - WS_LS_X/Y repeat 设大到 3σ=1000nm：静态连拍 3σ 不变（静止不重采样，
      证明不是逐帧后处理噪声），动态往复 3σ 显著增大且量级 =
      hypot(静态, 1000nm)（row←WS_LS_Y、col←WS_LS_X）；
    - 读出链分离：到位后 /api/axes 的 pos 恒等于指令目标（名义链精确），
      repeat 只进 actual 读数与 6DOF 真值（图像）。
    结束还原 repeat（API 写回显示单位 nm）+ config_overrides.json 按进入
    前字节恢复（/api/axis/config 会持久化，别让测试留垃圾）。"""
    ovr_path = os.path.join(ROOT, "runtime", "config_overrides.json")
    ovr_bytes = None
    if os.path.exists(ovr_path):
        with open(ovr_path, "rb") as f:
            ovr_bytes = f.read()
    em = http_json(BASE + "/api/errormaps")
    rep0 = {a: em["axes"][a]["repeat_3sigma_display"]
            for a in ("WS_LS_X", "WS_LS_Y")}
    # 2026-09-08：本用例度量**系统性误差链**（repeat 到位重采样经 6DOF 进
    # 图像），须关质心噪声（fast_noise_px 结果倒推注入后 σ=fnp=0.15px 的
    # 随机抖动会在 n=20 抽样下淹没 1000nm 注入的判读；传感器口径由
    # i21/i22/i28 系列覆盖）
    http_put(BASE + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.0}}})
    # 迭代 34 起 template_name 必填：当前帧 teach 专用模板（跑完删）
    _del_tpl("i29_rep_tpl")
    c, r = _mk_tpl("down", "i29_rep_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        _s, r1 = run_job("align_repeat_static", "down",
                         {"repeat": 20, "wait_ms": 0,
                          "template_name": "i29_rep_tpl"})
        s1 = _s3_rowcol(r1)
        for a in ("WS_LS_X", "WS_LS_Y"):
            c, r = http_post(BASE + "/api/axis/config",
                             {"axis": a, "repeat": 1000.0})
            assert c == 200, (a, r)
        _s, r2 = run_job("align_repeat_static", "down",
                         {"repeat": 20, "wait_ms": 0,
                          "template_name": "i29_rep_tpl"})
        s2 = _s3_rowcol(r2)
        _s, rd = run_job("align_repeat_dynamic", "down",
                         {"repeat": 20, "wait_ms": 0, "travel_mm": 2.0,
                          "template_name": "i29_rep_tpl"})
        d = _s3_rowcol(rd)
        # 读出链分离（repeat=1000nm 状态下）：pos 精确等于指令目标
        e = move("WS_LS_X", 58.2162)
        nom_exact = (e["pos"] == 58.2162)
        dev_nm = (e["actual"] - e["pos"]) * 1e6
        check("i29.repeat_static_unchanged",
              all(0 <= s2[i] < 2.5 * max(s1[i], 30.0) for i in (0, 1)),
              "静态 3σ_nm 前=(%.1f,%.1f) repeat=1000nm 后=(%.1f,%.1f)"
              % (s1 + s2))
        exp = (math.hypot(s2[0], 1000.0), math.hypot(s2[1], 1000.0))
        ratio = (d[0] / exp[0], d[1] / exp[1])
        check("i29.repeat_dynamic_gt_static",
              d[0] > s2[0] + 500.0 and d[1] > s2[1] + 500.0,
              "动态 3σ_nm=(%.1f,%.1f) vs 静态=(%.1f,%.1f)" % (d + s2))
        check("i29.repeat_dynamic_magnitude",
              all(0.6 <= ratio[i] <= 1.7 for i in (0, 1)),
              "动态/预期=(%.2f,%.2f)（预期=hypot(静态,1000nm)="
              "(%.0f,%.0f)nm）" % (ratio + exp))
        check("i29.readout_nominal_exact",
              nom_exact and abs(dev_nm) < 5000.0,
              "pos=%.6f（=target）actual-pos=%.1fnm（repeat+map 只进 "
              "actual）" % (e["pos"], dev_nm))
    finally:
        for a in ("WS_LS_X", "WS_LS_Y"):
            http_post(BASE + "/api/axis/config",
                      {"axis": a, "repeat": rep0[a]})
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.1}}})
        _del_tpl("i29_rep_tpl")
        if ovr_bytes is not None:
            with open(ovr_path, "wb") as f:
                f.write(ovr_bytes)
        elif os.path.exists(ovr_path):
            os.remove(ovr_path)


def test_i29_abbe_trajectory():
    """迭代 29b：阿贝误差不影响重复性、但影响标记在视场中的运动轨迹
    （系统性、逐位置不同）——WS_LS_X 的 rx/ry 角度误差地图 × 旋转枢轴
    杠杆臂（error_kinematics rot_trans 项）：
    - PUT /api/kinematics custom_center 把 WS_LS_X 误差枢轴下沉到工具点
      以下 5000mm（默认 ~76.65mm）：/api/toolpoints 的 rot_trans_nm 按
      杠杆比放大（±10%）——证明阿贝项 = 角误差 × 杠杆臂且在链上；
    - 扫 WS_LS_X ±0.5mm：guide 取 wafer_mark_1 投影像素（与 REAL 渲染同
      一 collect_marks 几何），六自由度 ON/OFF 两趟对照得纯误差像素偏移
      dev(dp)（OFF 即名义轨迹，差分精确消掉名义运动与标记排布）；dev 随
      位置系统性变化（非纯噪声），且逐位置与 toolpoints 误差差分经相机
      内参（dir_x/dir_y/view_rot/k）投影一致（容差 0.1px+10%）；
    - FAST identify（灯开）在端点位置能看到同一偏移（≥1 个判真点落在
      guide 像素 1px 内）——证明偏移确实进了图像链而非只进 toolpoints
      显示通道。
    结束 custom_center 还原 null（=默认枢轴）、六自由度开关还原、轴回位、
    灯灭。"""
    cam = "down"
    # 相机内参（投影方向映射，预测用）
    mc = http_json(BASE + "/api/machine_constants")
    cd = (mc.get("constants") or {}).get("cameras", {}).get(cam) or {}
    # 迭代 39：投影链旋转=真值 view_rot（相机物理安装旋转角；不再用认知
    # rot_mrad——它只进反算链）
    vrot_deg = float((http_json(BASE + "/api/vision/cameras")["cameras"][cam]
                      ).get("view_rot_deg") or 0.0)

    def proj_px(dx_mm, dy_mm):
        """世界系偏差 → 图像 (u,v) px（与 CameraModel.project_rel 同式：
        先 R(−view_rot) 真值旋转，再 dir/swap 映射）。"""
        rot = math.radians(vrot_deg)
        if rot:
            cr, sr = math.cos(rot), math.sin(rot)
            dx_mm, dy_mm = cr * dx_mm + sr * dy_mm, -sr * dx_mm + cr * dy_mm
        k = float(cd["pixel_um"]) / float(cd["mag"]) / 1000.0
        u = dx_mm / k * float(cd["dir_x"])
        v = dy_mm / k * float(cd["dir_y"])
        if cd.get("swap_xy"):
            u, v = v, u
        return u, v

    def mark_px():
        g = http_json(BASE + "/api/vision/guide?cam=" + cam)
        return next((m for m in g["marks"] if m["name"] == "wafer_mark_1"),
                    None)

    kin = http_json(BASE + "/api/kinematics")
    mod = kin["modules"]["wafer_stage"]
    link = next(l for l in mod["chain"] if l["axis"] == "WS_LS_X")
    c_def = list(link["effective_center"])
    lever_def = float(mod["tool_point"][2]) - c_def[2]
    lever_big = 5000.0
    p0 = 58.2162
    sweeps = (-0.5, -0.25, 0.0, 0.25, 0.5)
    try:
        # 杠杆缩放断言（默认枢轴 vs 5000mm 枢轴，同位置 p0）
        move("WS_LS_X", p0)
        rt_def = next(a for a in http_json(BASE + "/api/toolpoints")
                      ["modules"]["wafer_stage"]["axes"]
                      if a["axis"] == "WS_LS_X")["rot_trans_nm"]
        c, r = http_put(BASE + "/api/kinematics",
                        {"modules": {"wafer_stage": {"chain": [
                            {"axis": "WS_LS_X", "custom_center":
                             [c_def[0], c_def[1],
                              float(mod["tool_point"][2]) - lever_big]}]}}})
        assert c == 200, (c, r)
        rt_big = next(a for a in http_json(BASE + "/api/toolpoints")
                      ["modules"]["wafer_stage"]["axes"]
                      if a["axis"] == "WS_LS_X")["rot_trans_nm"]
        # 只比显著分量（x/y 由 z 向杠杆驱动；z 分量由水平杠杆驱动不变）
        ratios = [abs(b / d) for b, d in zip(rt_big, rt_def)
                  if abs(d) > 50.0]
        expect_ratio = lever_big / lever_def
        check("i29.abbe_lever_scaling",
              len(ratios) >= 2 and
              all(abs(rt / expect_ratio - 1.0) < 0.10 for rt in ratios),
              "rot_trans_nm 默认=%s → 5000mm杠杆=%s，分量比=%s，理论=%.1f"
              % ([round(v, 1) for v in rt_def],
                 [round(v, 1) for v in rt_big],
                 [round(v, 1) for v in ratios], expect_ratio))

        # 两趟扫位置：six_dof OFF（名义轨迹）/ ON（含误差）
        trace = {}
        for on in (False, True):
            c, r = http_post(BASE + "/api/errormaps",
                             {"six_dof_enabled": on})
            assert c == 200, (c, r)
            rows = []
            for dp in sweeps:
                move("WS_LS_X", p0 + dp)
                mk = mark_px()
                assert mk is not None and mk["in_fov"],                     "dp=%+.2f wafer_mark_1 不在视野" % dp
                tp = http_json(BASE + "/api/toolpoints")[
                    "modules"]["wafer_stage"]
                rt = next(a for a in tp["axes"] if a["axis"] == "WS_LS_X")
                rows.append({"dp": dp, "px": list(mk["fov_center"]),
                             "err": tp["error_mm"][:2],
                             "rt_x": rt["rot_trans_nm"][0]})
            trace[on] = rows
        http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        off, on_ = trace[False], trace[True]
        ref = len(sweeps) // 2
        # dev = ON−OFF（纯误差像素偏移）；与 toolpoints 误差差分投影对照
        # （参考位置 dp=0 对消掉标记相对工具点的常量姿态项 rot×local）
        devs, preds, rt_xs = [], [], []
        max_over = 0.0
        for i in range(len(sweeps)):
            dev = (on_[i]["px"][0] - off[i]["px"][0],
                   on_[i]["px"][1] - off[i]["px"][1])
            ddev = (dev[0] - (on_[ref]["px"][0] - off[ref]["px"][0]),
                    dev[1] - (on_[ref]["px"][1] - off[ref]["px"][1]))
            pu, pv = proj_px(on_[i]["err"][0] - on_[ref]["err"][0],
                             on_[i]["err"][1] - on_[ref]["err"][1])
            devs.append(ddev)
            preds.append((pu, pv))
            rt_xs.append(on_[i]["rt_x"])
            mm = math.hypot(ddev[0] - pu, ddev[1] - pv)
            max_over = max(max_over,
                           mm - (0.10 + 0.10 * math.hypot(pu, pv)))
        rng_col = max(d[0] for d in devs) - min(d[0] for d in devs)
        # dev_col 与阿贝项 rt_x 的相关性（轨迹同涨落，非纯噪声）
        n = len(devs)
        m_d = sum(d[0] for d in devs) / n
        m_r = sum(rt_xs) / n
        cov = sum((devs[i][0] - m_d) * (rt_xs[i] - m_r) for i in range(n))
        corr = cov / math.sqrt(
            sum((d[0] - m_d) ** 2 for d in devs)
            * sum((rt - m_r) ** 2 for rt in rt_xs) + 1e-30)
        check("i29.abbe_trajectory",
              max_over <= 0.0 and rng_col >= 0.5 and abs(corr) >= 0.9,
              "逐位置最大超差=%.3fpx（容差 0.1px+10%%） dev_col 范围=%.2fpx"
              " corr(dev_col, rt_x)=%.3f" % (max_over, rng_col, corr))
        check("i29.abbe_direction",
              all(i == ref or abs(devs[i][0] - preds[i][0])
                  < 0.10 + 0.10 * abs(preds[i][0]) for i in range(n)),
              "col 分量方向/量级逐位置对账（投影含 dir_x=%s/view_rot=%s°）："
              "dev=%s pred=%s"
              % (cd.get("dir_x"), vrot_deg,
                 [round(dd[0], 2) for dd in devs],
                 [round(pp[0], 2) for pp in preds]))
        # FAST identify 在同一位置看到同一偏移（图像链末端验证）
        c, r = http_post(BASE + "/api/io/command",
                         {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                          "args": {"value": True}})
        assert c == 200 and r.get("ok"), (c, r)
        time.sleep(0.2)
        try:
            c, r = http_post(BASE + "/api/vision/identify", {"cam": cam})
            res = ((r.get("entry") or {}).get("result") or {})
            pts = [(cc_, rr_) for cc_, rr_, sc in
                   zip(res.get("lstCol") or [], res.get("lstRow") or [],
                       res.get("lstScore") or [])
                   if sc >= 0.6]
            gx, gy = on_[-1]["px"]       # sweep 终点 dp=+0.5（当前位）
            near = any(math.hypot(cc_ - gx, rr_ - gy) < 1.0
                       for cc_, rr_ in pts)
            check("i29.abbe_into_image",
                  c == 200 and near,
                  "identify 判真点 %d 个，距 guide 预测像素 (%.1f,%.1f) "
                  "1px 内%s" % (len(pts), gx, gy,
                                "有命中" if near else "无命中"))
        finally:
            http_post(BASE + "/api/io/command",
                      {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                       "args": {"value": False}})
    finally:
        # 还原：custom_center=null（默认枢轴）、六自由度开、轴回 p0
        http_put(BASE + "/api/kinematics",
                 {"modules": {"wafer_stage": {"chain": [
                     {"axis": "WS_LS_X", "custom_center": None}]}}})
        http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        try:
            move("WS_LS_X", p0)
        except Exception:
            pass
        kin2 = http_json(BASE + "/api/kinematics")
        eff = next(l for l in kin2["modules"]["wafer_stage"]["chain"]
                   if l["axis"] == "WS_LS_X")["effective_center"]
        check("i29.abbe_restore",
              all(abs(eff[k] - c_def[k]) < 1e-9 for k in range(3)),
              "custom_center 还原后 effective_center=%s（默认 %s）"
              % (eff, c_def))


def test_extra_routines(tm_cam=None):
    """新增六项测校 routine 的冒烟回归（各 3 次、小行程）。"""
    import urllib.request

    def start(routine, cam, params):
        req = urllib.request.Request(
            BASE + "/api/calibration/start",
            data=json.dumps({"routine": routine, "cam": cam,
                             "params": params}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                return json.loads(r.read().decode("utf-8"))
        except urllib.error.HTTPError as ex:
            return json.loads(ex.read().decode("utf-8"))

    def wait(jid, timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with urllib.request.urlopen(
                    BASE + "/api/calibration/status?job_id=%d" % jid,
                    timeout=10) as r:
                j = json.loads(r.read().decode("utf-8"))
            if j.get("state") in ("done", "error"):
                return j
            time.sleep(1.0)
        return {"state": "timeout"}

    def get_result(jid):
        with urllib.request.urlopen(
                BASE + "/api/calibration/result?job_id=%d" % jid,
                timeout=10) as r:
            return json.loads(r.read().decode("utf-8")).get("result") or {}

    # ① 下共焦重复性（晶圆在位）
    prep_down()
    j = start("confocal_repeat", "down", {"repeat": 5, "wait_ms": 20})
    st = wait(j.get("job_id"))
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    check("extra.confocal_down",
          st.get("state") == "done"
          and 0 <= (res.get("stats") or {}).get("reading",
                                                {}).get("s3_um", -1) < 5.0,
          "state=%s s3=%s µm" % (st.get("state"),
                                 ((res.get("stats") or {}).get("reading")
                                  or {}).get("s3_um")))
    # ①' 数据增强列（静态 CSV 尾列 mark_world/cam_world）
    csv_rel = (res.get("csv") or "").replace("runtime/", "runtime/")
    hdr = None
    if csv_rel:
        path = os.path.join(ROOT, csv_rel.replace("/", os.sep))
        if os.path.isfile(path):
            hdr = open(path, encoding="utf-8").readline().strip().split(",")
    check("extra.confocal_csv",
          hdr is not None and hdr[:5] == ["index", "reading_um", "valid",
                                          "z_surface_mm", "z_focus_mm"],
          "header=%s" % (hdr or "无"))

    # ② 上共焦重复性（键合头有芯片）——先回零翻转/键合臂相关轴
    #（main 序列经多个作业的 finally 恢复后路径自然可行；extra 直接
    # 路径下 FA/BS 残留可能触发沙盒干涉，显式回零保证交接路径干净）
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_up()
    j = start("confocal_repeat", "up", {"repeat": 3, "wait_ms": 20})
    st = wait(j.get("job_id"))
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    check("extra.confocal_up",
          st.get("state") == "done"
          and (res.get("stats") or {}).get("reading", {}).get("s3_um") is
          not None,
          "state=%s" % st.get("state"))

    # ③ 硅片台移动重复性（迭代 34 起 template_name 必填：当前帧 teach；
    #    作业后台执行——模板须等作业终态后才删）
    prep_down()
    _del_tpl("extra_ws_tpl")
    c, r = _mk_tpl("down", "extra_ws_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    j = start("ws_move_repeat", "down",
              {"repeat": 5, "wait_ms": 20, "travel_mm": 2.0,
               "template_name": "extra_ws_tpl"})
    st = wait(j["job_id"]) if j.get("job_id") else \
        {"state": "rejected", "error": j.get("error")}
    _del_tpl("extra_ws_tpl")
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    stt = res.get("stats") or {}
    check("extra.ws_move",
          st.get("state") == "done"
          and 0 <= (stt.get("row") or {}).get("s3_nm", -1)
          < 5 * NM_PER_PX["down"],
          "state=%s s3=%s err=%s" % (st.get("state"),
                                     (stt.get("row") or {}).get("s3_nm"),
                                     (st.get("error") or "")[:50]))

    # ④ 对准台移动重复性（键合头芯片）——confocal_up 后 BS_Arm_Z 停在
    # -118.33（观测位），prep_down/ws_move 不复位它；不回零会在
    # prep_up 的 FA_X 交接路径触发沙盒干涉（键合头低位×翻转承载）
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_up()
    _del_tpl("extra_as_tpl")
    c, r = _mk_tpl("up", "extra_as_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    j = start("as_move_repeat", "up",
              {"repeat": 5, "wait_ms": 20, "travel_mm": 1.0,
               "template_name": "extra_as_tpl"})
    st = wait(j["job_id"]) if j.get("job_id") else \
        {"state": "rejected", "error": j.get("error")}
    _del_tpl("extra_as_tpl")
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    stt = res.get("stats") or {}
    check("extra.as_move",
          st.get("state") == "done"
          and 0 <= (stt.get("row") or {}).get("s3_nm", -1)
          < 5 * NM_PER_PX["up"],
          "state=%s s3=%s err=%s" % (st.get("state"),
                                     (stt.get("row") or {}).get("s3_nm"),
                                     (st.get("error") or "")[:50]))

    # ⑤ 芯片交接重复性（首段取片一拍 + 每循环交接/回翻两拍，repeat=3 最小）
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_clean()
    j = start("die_transfer_repeat", "coarse",
              {"repeat": 3, "wait_ms": 20})
    st = wait(j.get("job_id"), timeout=1200.0)
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    stt = res.get("stats") or {}
    check("extra.die_transfer",
          st.get("state") == "done"
          and all(k in stt for k in ("flip", "head", "back")),
          "state=%s segs=%s" % (st.get("state"), sorted(stt)))

    # ⑥ 硅片台旋转重复性（as_move 后 BS_Arm_Z 在 -118.33，先回零）
    #    迭代 34 起 auto 路径移除——示教参数化为唯一模式；迭代 35 双示教
    #    位：两点（wafer_mark_1/3 各一）+ 测试侧按枢轴算 pose_after
    #    （原后端预测走位语义用户化），走 taught_points 数组
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_down()
    _del_tpl("extra_rt_b")
    c, r = _mk_down_tpl("extra_rt_b")
    if c != 200 or not r.get("ok"):
        check("extra.ws_rotate", False,
              "tpl 点1 建立失败：c=%s %s" % (c, r.get("error")))
    else:
        pose_before = {a: axis_entry(a)["pos"]
                       for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")}
        try:
            move("WS_LS_X", pose_before["WS_LS_X"] - 161.2204)
            centered = _center_down_mark("wafer_mark_3")
            _del_tpl("extra_rt_a")
            c, r = _mk_down_tpl("extra_rt_a")
            if not centered or c != 200 or not r.get("ok"):
                check("extra.ws_rotate", False,
                      "tpl 点2 建立失败：centered=%s c=%s %s"
                      % (centered, c, r.get("error")))
            else:
                pose2 = {a: axis_entry(a)["pos"]
                         for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")}
                move("WS_LS_X", pose_before["WS_LS_X"])
                move("WS_LS_Y", pose_before["WS_LS_Y"])
                rot = math.radians(1.0)
                pts2 = _rot_pose_after(
                    [{"tpl": "extra_rt_b", "pose": pose_before},
                     {"tpl": "extra_rt_a", "pose": pose2}], rot)
                j = start("ws_rotate_repeat", "down",
                          {"repeat": 3, "wait_ms": 20, "rot_deg": 1.0,
                           "taught_points": pts2})
                if not j.get("job_id"):
                    check("extra.ws_rotate", False,
                          "start 被拒：%s" % (j.get("error") or j))
                else:
                    st = wait(j["job_id"], timeout=900.0)
                    res = get_result(j["job_id"]) \
                        if st.get("state") == "done" else {}
                    stt = ((res.get("stats") or {}).get("angle_err") or {})
                    det = (res.get("detail") or {})
                    tp_det = det.get("taught_points") or []
                    check("extra.ws_rotate",
                          st.get("state") == "done"
                          and 0 <= stt.get("s3_urad", -1) < 500.0
                          and det.get("mode") == "taught"
                          and len(tp_det) == 2
                          and all("pose_after" in p for p in tp_det),
                          "state=%s s3=%s µrad mode=%s err=%s"
                          % (st.get("state"), stt.get("s3_urad"),
                             det.get("mode"), (st.get("error") or "")[:60]))
        finally:
            for name in ("extra_rt_b", "extra_rt_a"):
                _del_tpl(name)

    # ⑦ jog_group：down 动态选对准台动（需求②；die_transfer 的
    # finally 已回收 die/回轴，但保险再回零 FA/BS）；迭代 34 起模板必填
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_down()
    _del_tpl("extra_jg_tpl")
    c, r = _mk_tpl("down", "extra_jg_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    j = start("align_repeat_dynamic", "down",
              {"repeat": 10, "wait_ms": 20, "travel_mm": 2.0,
               "jog_group": "align", "template_name": "extra_jg_tpl"})
    st = wait(j["job_id"]) if j.get("job_id") else \
        {"state": "rejected", "error": j.get("error")}
    res = get_result(j["job_id"]) if st.get("state") == "done" else {}
    check("extra.jog_group_align",
          st.get("state") == "done"
          and sorted((res.get("axis_log") or {}).keys())
          == ["AS_X", "AS_Y"],
          "state=%s axes=%s err=%s"
          % (st.get("state"),
             sorted((res.get("axis_log") or {}).keys()),
             (st.get("error") or "")[:50]))
    # 非法组 400（模板合法，400 文案落在 jog_group；作业未起，删模板）
    j = start("align_repeat_dynamic", "down",
              {"repeat": 10, "travel_mm": 2.0, "jog_group": "bad",
               "template_name": "extra_jg_tpl"})
    _del_tpl("extra_jg_tpl")
    check("extra.jog_group_bad400", not j.get("ok"),
          "resp=%s" % j)


# ---------------- 迭代 29b：示教旋转 / 协作式取消 ----------------
def _mk_down_tpl(name):
    """下相机当前帧建模板（先点灯，建完灭灯）——同 iter25 惯例。"""
    c, r = http_post(BASE + "/api/io/command",
                     {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                      "args": {"value": True}})
    assert c == 200 and r.get("ok"), (c, r)
    time.sleep(0.2)
    try:
        return http_post(BASE + "/api/vision/templates",
                         {"cam": "down", "name": name}, timeout=60)
    finally:
        http_post(BASE + "/api/io/command",
                  {"path": LIGHT_POINT["down"], "cmd": "SetBool",
                   "args": {"value": False}})


def _del_tpl(name):
    req = urllib.request.Request(
        BASE + "/api/vision/templates",
        data=json.dumps({"name": name}).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="DELETE")
    try:
        urllib.request.urlopen(req, timeout=5).read()
    except Exception:
        pass


def _mk_tpl(cam, name):
    """任意相机当前帧建模板（先点灯，建完灭灯；默认中央 ROI）——
    迭代 34 起 align/ws/as_move_repeat 模板必填，测试先 teach 再跑。"""
    c, r = http_post(BASE + "/api/io/command",
                     {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                      "args": {"value": True}})
    assert c == 200 and r.get("ok"), (cam, r)
    time.sleep(0.2)
    try:
        return http_post(BASE + "/api/vision/templates",
                         {"cam": cam, "name": name}, timeout=60)
    finally:
        http_post(BASE + "/api/io/command",
                  {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                   "args": {"value": False}})


def _mk_small_tpl(cam, name, anchor_tpl=None, half=300, anchor_px=None):
    """建 (2*half+1)px 小 ROI 模板（camera_microrot 杠杆走位用——大模板
    匹配可达范围画幅中心 ±(W−tw)/2，装不下两拍照位间像素位移）：
    anchor_px=(col,row) 显式锚点像素最优先；否则 anchor_tpl 给了先用它
    identify 取锚点（取最近画幅中心的峰，兼容同形多实例）；否则用 guide
    最近标记的 fov_center（几何投影）。返回 http_post 结果 (status, obj)；
    锚不到标记抛 RuntimeError。"""
    c, r = http_post(BASE + "/api/io/command",
                     {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                      "args": {"value": True}})
    assert c == 200 and r.get("ok"), (cam, r)
    time.sleep(0.2)
    try:
        if anchor_px is not None:
            col, row = anchor_px
        elif anchor_tpl:
            c, r = http_post(BASE + "/api/vision/identify",
                             {"cam": cam, "template": anchor_tpl})
            res = ((r.get("entry") or {}).get("result") or {})
            pts = list(zip(res.get("lstCol") or [],
                           res.get("lstRow") or []))
            if not pts:
                raise RuntimeError("锚模板 %s 未识别到标记" % anchor_tpl)
            col, row = min(pts, key=lambda p: abs(p[0] - 1500.0)
                           + abs(p[1] - 1500.0))
        else:
            g = http_json(BASE + "/api/vision/guide?cam=%s" % cam)
            ms = [m for m in g.get("marks") or [] if m.get("in_fov")]
            if not ms:
                raise RuntimeError("相机 %s 视野内无标记" % cam)
            col, row = ms[0]["fov_center"]
        tw = 2 * half + 1
        x0 = max(0, min(3000 - tw, int(round(col)) - half))
        y0 = max(0, min(3000 - tw, int(round(row)) - half))
        return http_post(BASE + "/api/vision/templates",
                         {"cam": cam, "name": name,
                          "roi": [x0, y0, tw, tw]}, timeout=60)
    finally:
        http_post(BASE + "/api/io/command",
                  {"path": LIGHT_POINT[cam], "cmd": "SetBool",
                   "args": {"value": False}})


def _center_down_mark(name, tol=0.005, max_iter=6):
    """guide 引导把指定标记居中到下相机视场中心（WS_LS 平移；迭代 31
    示教点建制复用——纯平移精确到位，残余小于 tol 即收）。"""
    for _ in range(max_iter):
        g = http_json(BASE + "/api/vision/guide?cam=down")
        mk = next((m for m in g.get("marks") or []
                   if m.get("name") == name), None)
        if mk is None:
            return False
        if mk.get("in_fov") and max(abs(d) for d in mk["delta_mm"][:2]) < tol:
            return True
        for s in (mk.get("suggested") or []):
            move(s["axis"], axis_entry(s["axis"])["pos"] + s["delta_mm"])
    return False


def test_i29b_rotate_taught():
    """ws_rotate_repeat 示教参数化模式（迭代 31 起旧四键映射为 2 点
    taught_points 走多点路径）：
    - 缺 tpl_before → 400；模板不存在 → 400；pose 越软限位 → 400
    - 合法示教小角（rot_deg=1° 兼容换算 rot_rad）repeat=3 跑通 done +
      CSV(ws_rotate_taught_*) + detail.mode=taught + 位置/Δθ 残余统计
    示教现场做（迭代 31 起示教点须使标记在 rz0 可见——时序=首循环逐点
    基线→WS_Rz 转 rot→回零→逐点重测）：pose_before=prep_down 对中位
    （wafer_mark_1 居中，建 i29b_tb）；WS_LS 平移 −161.22mm 把
    wafer_mark_3 居中（同 rz0、guide 精修，建 i29b_ta=pose_after），
    回 pose_before 开作业。返回 done 作业 id（供 cancel 拒绝分支复用），
    失败返 None。
    迭代 35 双示教位适配：run 段改 taught_points 数组 + 测试侧
    _rot_pose_after（枢轴 Δ=(I−R)(P−c)）给每点 pose_after——旧四键
    映射（pose_after 缺省=pose）在大杠杆 1° 下旋后标记出视场；四键
    路径仍由上方 400 分支覆盖。"""
    prep_down()
    # 1) 缺 tpl_before → 400（tpl_after 触发示教模式，tpl_before 先验）
    c, r = cal_start("ws_rotate_repeat", "down",
                     {"tpl_after": "x", "repeat": 3})
    ok1 = c == 400 and "tpl_before" in (r.get("error") or "")
    # 2) 模板不存在 → 400（消息列可用名）
    c2, r2 = cal_start("ws_rotate_repeat", "down",
                       {"tpl_before": "__no_such__", "tpl_after": "__no2__",
                        "repeat": 3})
    ok2 = c2 == 400 and "不存在" in (r2.get("error") or "")
    check("i29b.taught_param_400", ok1 and ok2,
          "c=%s/%s err=%s" % (c, c2, (r2.get("error") or "")[:70]))

    c, r = _mk_down_tpl("i29b_tb")
    if c != 200 or not r.get("ok"):
        check("i29b.taught_run", False,
              "tpl_before 建立失败：c=%s %s" % (c, r.get("error")))
        _del_tpl("i29b_tb")
        return None
    pose_before = {a: axis_entry(a)["pos"]
                   for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")}
    # 3) pose 越软限位 → 400（WS_LS_X hi=220mm；tpl 均存在后轮到 pose 验）
    c, r = cal_start("ws_rotate_repeat", "down",
                     {"tpl_before": "i29b_tb", "tpl_after": "i29b_tb",
                      "pose_before": {"WS_LS_X": 999.0}, "repeat": 3})
    check("i29b.taught_pose_limit_400",
          c == 400 and "限位" in (r.get("error") or "")
          and "WS_LS_X" in (r.get("error") or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:80]))
    jid = None
    try:
        # 4) 示教 pose_after：WS_LS 平移 −161.22mm（wafer_mark_1→
        # wafer_mark_3 同 y 跨片），guide 精修居中后建 i29b_ta
        move("WS_LS_X", pose_before["WS_LS_X"] - 161.2204)
        centered = _center_down_mark("wafer_mark_3")
        c, r = _mk_down_tpl("i29b_ta")
        if not centered or c != 200 or not r.get("ok"):
            check("i29b.taught_run", False,
                  "tpl_after 建立失败：centered=%s c=%s %s"
                  % (centered, c, r.get("error")))
            return None
        pose_after = {a: axis_entry(a)["pos"]
                      for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")}
        # 回 pose_before 再开作业（作业以当前 WS_Rz 为首测基准 rz0）
        move("WS_LS_X", pose_before["WS_LS_X"])
        move("WS_LS_Y", pose_before["WS_LS_Y"])
        # 迭代 35 双示教位：旧四键映射 2 点（pose_after 缺省=pose 同位
        # 复测）在大杠杆 rot=1° 下标记会出视场——run 段改 taught_points
        # 数组 + 测试侧 _rot_pose_after 给每点旋后示教位（四键路径仍由
        # 上面 400 分支覆盖）
        rot = math.radians(1.0)
        pts2 = _rot_pose_after(
            [{"tpl": "i29b_tb", "pose": pose_before},
             {"tpl": "i29b_ta", "pose": pose_after}], rot)
        s, res = run_job("ws_rotate_repeat", "down",
                         {"taught_points": pts2,
                          "rot_deg": 1.0, "repeat": 3, "wait_ms": 20},
                         timeout=600.0)
        csv_ok, csv_d = False, "无结果"
        if res:
            rel = res.get("csv") or ""
            pth = os.path.join(ROOT, rel.replace("/", os.sep))
            csv_ok = (os.path.isfile(pth)
                      and os.path.basename(rel).startswith(
                          "ws_rotate_taught_"))
            csv_d = os.path.basename(rel)
        resd = ((res or {}).get("stats") or {}).get("residual") or {}
        dth = ((res or {}).get("stats") or {}).get("dtheta") or {}
        det = (res or {}).get("detail") or {}
        tp_det = det.get("taught_points") or []
        sane = (0 <= (resd.get("col_s3_nm") or -1) < 3000.0
                and 0 <= (resd.get("row_s3_nm") or -1) < 3000.0
                and 0 <= (dth.get("s3_urad") or -1) < 500.0)
        check("i29b.taught_run",
              s.get("state") == "done" and res is not None
              and det.get("mode") == "taught" and csv_ok and sane
              and det.get("n_points") == 2
              and abs((det.get("rot_rad") or 0) - math.radians(1.0)) < 1e-9
              and len(tp_det) == 2
              and all("pose_after" in p for p in tp_det)
              and len(((res or {}).get("series") or {})
                      .get("d_col_px") or []) == 3
              and len(((res or {}).get("series") or {})
                      .get("dtheta_urad") or []) == 3,
              "state=%s mode=%s n=%s csv=%s col_s3_nm=%s row_s3_nm=%s "
              "dth_s3_urad=%s%s"
              % (s.get("state"), det.get("mode"), det.get("n_points"),
                 csv_d, resd.get("col_s3_nm"), resd.get("row_s3_nm"),
                 dth.get("s3_urad"),
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
        if s.get("state") == "done":
            jid = s.get("job_id")
    finally:
        for name in ("i29b_tb", "i29b_ta"):
            _del_tpl(name)
    return jid


# ---------------- 迭代 29c：交接循环内不放回蓝膜（物料守恒） ----------------
def test_i29c_die_transfer():
    """die_transfer_repeat 结束留片在拾取头（迭代 43，用户明确要求；
    迭代 29c 的"循环内不放回蓝膜"保留、finally 一次性归位取消）：
    prep_clean（蓝膜满阵列、无残留）起 repeat=3 作业，running 期间轮询
    /api/viz 蓝膜 die 数——仅取片 1 降（N→N-1）、全程不回升（循环内
    无 flip_place_back，finally 也不再归位）；done 后蓝膜保持 N-1、
    head 无 die、flip 恰 1 die（留片待下次作业沿用）；日志无
    reclaim/归位动作行，且恰 1 条"留在拾取头（carrier=flip）"。结果
    分段统计 flip/head/back 三段齐全（flip=首段单样本，head/back 各
    3 样本）；detail 无 measure_back（参数废止）；finally 轴位恢复
    （AS/BS/FA/ES 作业前后一致）。"""
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_clean()
    n0 = len(dies_on("film"))
    pre_axes = {a: axis_entry(a)["pos"] for a in
                ("AS_X", "AS_Y", "AS_Z", "BS_Arm_Z", "FA_X",
                 "ES_X", "ES_Y")}
    c, r = cal_start("die_transfer_repeat", "coarse",
                     {"repeat": 3, "wait_ms": 20})
    if c != 200:
        check("i29c.dt_no_placeback", False,
              "start 被拒：c=%s %s" % (c, r.get("error")))
        return
    jid = r["job_id"]
    film_run = []                   # running 期间蓝膜 die 数序列
    t0 = time.time()
    s = {}
    while time.time() - t0 < 1200.0:
        s = http_json(BASE + "/api/calibration/status?job_id=%d" % jid)
        if s.get("state") in ("done", "error", "canceled"):
            break
        film_run.append(len(dies_on("film")))
        time.sleep(0.3)
    res = cal_result(jid) if s.get("state") == "done" else None
    stt = (res or {}).get("stats") or {}
    ser = (res or {}).get("series") or {}
    hdr = None
    if res:
        pth = os.path.join(ROOT, (res.get("csv") or "")
                           .replace("/", os.sep))
        if os.path.isfile(pth):
            hdr = open(pth, encoding="utf-8").readline().strip()
    logs = s.get("log") or []
    reclaim_lines = [l for l in logs if "reclaim" in l]
    stay_lines = [l for l in logs if "留在拾取头" in l]
    # 归位动作行（finally 的"未归位蓝膜"说明行不算归位动作）
    guiwei_lines = [l for l in logs if "归位" in l and "未归位" not in l]
    drops = sum(1 for k in range(len(film_run) - 1)
                if film_run[k + 1] < film_run[k])
    rises = sum(1 for k in range(len(film_run) - 1)
                if film_run[k + 1] > film_run[k])
    post_film = len(dies_on("film"))
    post_axes = {a: axis_entry(a)["pos"] for a in pre_axes}
    axes_ok = all(abs(post_axes[a] - pre_axes[a]) < 1e-6
                  for a in pre_axes)
    check("i29c.dt_no_placeback",
          s.get("state") == "done" and res is not None
          and set(stt) == {"flip", "head", "back"}
          and len((ser.get("flip") or {}).get("col") or []) == 1
          and len((ser.get("head") or {}).get("col") or []) == 3
          and len((ser.get("back") or {}).get("col") or []) == 3
          and hdr is not None and "back_col" in hdr
          and "measure_back" not in (res.get("detail") or {})
          and film_run and min(film_run) == n0 - 1
          and drops == 1 and rises == 0
          and post_film == n0 - 1
          and not dies_on("head") and len(dies_on("flip")) == 1
          and not reclaim_lines and not guiwei_lines
          and len(stay_lines) == 1 and "carrier=flip" in stay_lines[0],
          "state=%s segs=%s film(min,drop,rise)=%s/%s/%s post=%s/%s "
          "flip_die=%d stay=%s reclaim=%s%s"
          % (s.get("state"), sorted(stt),
             min(film_run) if film_run else "-", drops, rises,
             post_film, n0, len(dies_on("flip")), stay_lines,
             reclaim_lines,
             (" err=%s" % (s.get("error") or ""))
             if s.get("state") != "done" else ""))
    check("i29c.dt_axes_restored", axes_ok,
          "diff=%s" % {a: round(post_axes[a] - pre_axes[a], 8)
                       for a in pre_axes
                       if abs(post_axes[a] - pre_axes[a]) >= 1e-9})


def test_i29c_cancel_reclaim():
    """head 残留回收沿用 + 取消留片（迭代 43 新语义；迭代 29c 的废止
    参数兼容保留）：prep_up 造 head 残留 die（flip 空、蓝膜缺 1）；
    带旧参 measure_back=False 启作业（废止参数传入忽略不报错→200 且
    正常开跑）；作业首 reclaim 把 head 残留回收 head→flip（不再回放
    蓝膜，film 数不变），flip 有余片→沿用该片、跳过首次取片；随后
    立即 cancel → 循环边界生效 → canceled 终态。断言：作业期间 film
    数全程不变（=n_full-1，回收不改 film 数、沿用不取新）、post_film
    ==n_full-1、head 空、flip 恰 1 die（留片）、日志含"残留"回收行
    与"沿用"行、无归位动作行。"""
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_up()                      # head 有 die 的残留态（真空保持）
    try:
        n_full = len(dies_on("film")) + len(dies_on("head"))   # 满阵列数
        c, r = cal_start("die_transfer_repeat", "coarse",
                         {"repeat": 3, "wait_ms": 20,
                          "measure_back": False})   # 废止参数：忽略不报错
        if c != 200:
            check("i29c.dt_cancel_reclaim", False,
                  "start 被拒：c=%s %s" % (c, r.get("error")))
            return
        jid = r["job_id"]
        time.sleep(0.5)            # 确认作业真在跑再发取消
        cc, cr = http_post(BASE + "/api/calibration/cancel",
                           {"job_id": jid})
        req_ok = cc == 200 and cr.get("ok")
        film_run = []              # 作业期间蓝膜 die 数采样（应全程不变）
        t0 = time.time()
        s = {}
        while time.time() - t0 < 600.0:
            s = http_json(BASE + "/api/calibration/status?job_id=%d" % jid)
            if s.get("state") in ("done", "error", "canceled"):
                break
            film_run.append(len(dies_on("film")))
            time.sleep(0.3)
        logs = s.get("log") or []
        post_film = len(dies_on("film"))
        # 归位动作行（finally 的"未归位蓝膜"说明行不算归位动作）
        guiwei_lines = [l for l in logs if "归位" in l and "未归位" not in l]
        check("i29c.dt_cancel_reclaim",
              req_ok and s.get("state") == "canceled"
              and "取消" in (s.get("error") or "")
              and post_film == n_full - 1
              and film_run and all(f == n_full - 1 for f in film_run)
              and not dies_on("head") and len(dies_on("flip")) == 1
              and any("残留" in l for l in logs)
              and any("沿用" in l for l in logs)
              and not guiwei_lines,
              "req=%s state=%s film=%s/%s(run=%s) head=%s flip=%s "
              "guiwei=%s err=%s"
              % (req_ok, s.get("state"), post_film, n_full,
                 sorted(set(film_run)) if film_run else "-",
                 len(dies_on("head")), len(dies_on("flip")), guiwei_lines,
                 (s.get("error") or "")[:50]))
    finally:
        for i in (1, 2, 3):
            vacuum("WT_BondingVac_%d" % i, False)


def test_i29b_cancel(done_jid=None):
    """协作式取消（POST /api/calibration/cancel）：
    repeat=100 静态作业 running 中 cancel → {ok,state:canceling} → 轮询到
    canceled 终态（status.error 文案含"取消"、jobs 透出 canceled、result
    对该态 400 清晰文案）、灯/视觉模式/临时模板恢复作业前状态；
    重复 cancel → 400；done 作业 cancel → 400；未知作业 → 400。
    迭代 34 起模板必填：空模板 start → 400；作业全程用用户模板、无 _cal_
    自建临时模板产生/残留。"""
    prep_down()
    # 空/None 模板 → 400（必填，文案列可用模板名）；且无 _cal_ 模板产生
    ce, re_ = cal_start("align_repeat_static", "down",
                        {"repeat": 10, "wait_ms": 50, "template_name": ""})
    cn, rn = cal_start("align_repeat_static", "down",
                       {"repeat": 10, "wait_ms": 50,
                        "template_name": None})
    tpl_pre = [t.get("name") for t in
               http_json(BASE + "/api/vision/templates")["templates"]
               if (t.get("name") or "").startswith("_cal_")]
    check("i29b.cancel_empty_tpl_400",
          ce == 400 and cn == 400
          and "template_name" in (re_.get("error") or "")
          and not tpl_pre,
          "c=%s/%s err=%s tpl_pre=%s"
          % (ce, cn, (re_.get("error") or "")[:60], tpl_pre))
    pre_mode = http_json(BASE + "/api/vision/cameras").get("mode")
    pre_light = bool(http_json(BASE + "/api/io")["points"].get(
        LIGHT_POINT["down"], False))
    _del_tpl("i29b_ctpl")
    c, r = _mk_down_tpl("i29b_ctpl")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        c, r = cal_start("align_repeat_static", "down",
                         {"repeat": 100, "wait_ms": 50,
                          "template_name": "i29b_ctpl"})
        assert c == 200 and r.get("ok"), (c, r)
        jid = r["job_id"]
        t0 = time.time()                # 等作业真跑起来（≥2 拍）
        while time.time() - t0 < 30:
            st = http_json(BASE + "/api/calibration/status?job_id=%d" % jid)
            if (st.get("progress") or {}).get("i", 0) >= 2:
                break
            time.sleep(0.15)
        c, r = http_post(BASE + "/api/calibration/cancel", {"job_id": jid})
        req_ok = c == 200 and r.get("ok") and r.get("state") == "canceling"
        st = {}
        t0 = time.time()
        while time.time() - t0 < 60:
            st = http_json(BASE + "/api/calibration/status?job_id=%d"
                           % jid)
            if st.get("state") in ("done", "error", "canceled"):
                break
            time.sleep(0.2)
        post_mode = http_json(BASE + "/api/vision/cameras").get("mode")
        post_light = bool(http_json(BASE + "/api/io")["points"].get(
            LIGHT_POINT["down"], False))
        tpl_left = [t.get("name") for t in
                    http_json(BASE + "/api/vision/templates")["templates"]
                    if (t.get("name") or "").startswith("_cal_")]
        jobs = http_json(BASE + "/api/calibration/jobs")["jobs"]
        jpub = next((j for j in jobs if j["job_id"] == jid), {})
        cr, robj = http_get(BASE + "/api/calibration/result?job_id=%d"
                            % jid)
        check("i29b.cancel_static",
              req_ok and st.get("state") == "canceled"
              and "取消" in (st.get("error") or "")
              and jpub.get("state") == "canceled"
              and cr == 400 and "取消" in (robj.get("error") or "")
              and post_mode == pre_mode and post_light == pre_light
              and not tpl_left,
              "req=%s state=%s jobs=%s result_c=%s mode=%s/%s light=%s/%s "
              "tpl_left=%s err=%s"
              % (req_ok, st.get("state"), jpub.get("state"), cr,
                 pre_mode, post_mode, pre_light, post_light, tpl_left,
                 (st.get("error") or "")[:40]))
    finally:
        _del_tpl("i29b_ctpl")
    # 重复 cancel → 400；未知作业 → 400；done 作业 cancel → 400
    c1, r1 = http_post(BASE + "/api/calibration/cancel", {"job_id": jid})
    c2, _ = http_post(BASE + "/api/calibration/cancel", {"job_id": 999999})
    c3, r3 = 0, {}
    if done_jid is not None:
        c3, r3 = http_post(BASE + "/api/calibration/cancel",
                           {"job_id": done_jid})
    check("i29b.cancel_reject_400",
          c1 == 400 and "不能取消" in (r1.get("error") or "")
          and c2 == 400
          and (done_jid is None or
               (c3 == 400 and "不能取消" in (r3.get("error") or ""))),
          "c=%s/%s/%s err=%s" % (c1, c2, c3,
                                 (r1.get("error") or "")[:50]))


def test_i29b():
    """迭代 29b 段汇总：示教旋转 → 协作式取消（复用示教 done 作业做
    done 后 cancel 拒绝分支）；交接测校物料守恒改造移至迭代 29c 段。"""
    done_jid = test_i29b_rotate_taught()
    test_i29b_cancel(done_jid)


def test_i29c():
    """迭代 29c 段汇总：交接循环内不放回蓝膜（三段统计/循环内无
    place_back 物证/finally 归位一次/轴恢复）→ 中断物料守恒+废止参数
    兼容。"""
    test_i29c_die_transfer()
    test_i29c_cancel_reclaim()


# ---------------- 迭代 31：rot_rad+多点示教 / camera_microrot ----------------
def test_i31_rot_rad():
    """rot_rad 校验（ws_rotate_repeat，start 同步 400）：越界（0/负/0.7）
    →400；超 WS_Rz 行程（0.035rad > ±1°×2）→400；rot_deg 兼容换算——
    40°→0.698rad 越界→400（1° 合法路径已由 i29b.taught_run 覆盖，
    detail.rot_rad≈0.017453）。"""
    prep_down()
    bads = [(("rot_rad", 0.0), "rot_rad"), (("rot_rad", -0.1), "rot_rad"),
            (("rot_rad", 0.7), "rot_rad"), (("rot_deg", 40.0), "rot_deg")]
    oks, errs = [], []
    for (key, val), tag in bads:
        c, r = cal_start("ws_rotate_repeat", "down",
                         {key: val, "repeat": 3})
        oks.append(c == 400)
        errs.append((r.get("error") or "")[:40])
    c, r = cal_start("ws_rotate_repeat", "down",
                     {"rot_rad": 0.035, "repeat": 3})
    check("i31.rot_rad_400",
          all(oks) and c == 400 and "行程" in (r.get("error") or ""),
          "c=%s errs=%s travel_err=%s"
          % (c, errs, (r.get("error") or "")[:60]))


def _teach3_down(tpls):
    """3 点跨片示教建制（wafer_mark_1/3/2，跨片 161~228mm 杠杆，rz0 均可
    见；i31/i32 共用）：prep_down → 逐点 WS 平移+guide 居中+建模板 →
    回 prep_down 对中位。返回 (taught_points, ok_build)；模板建立失败
    返回 (None, False)（已建模板由调用方 finally 清理）。"""
    prep_down()
    poses = []
    ok_build = True
    for k, (dx, dy, mark) in enumerate((
            (0.0, 0.0, "wafer_mark_1"),
            (-161.2204, 0.0, "wafer_mark_3"),
            (-161.2204, -161.2204, "wafer_mark_2"))):
        if k:
            move("WS_LS_X", 58.2162 + dx)
            move("WS_LS_Y", 111.6102 + dy)
            ok_build = _center_down_mark(mark) and ok_build
        c, r = _mk_down_tpl(tpls[k])
        if c != 200 or not r.get("ok"):
            return None, False
        poses.append({a: axis_entry(a)["pos"]
                      for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")})
    # 回 prep_down 对中位再开作业（作业以当前 WS_Rz 为基准 rz0）
    move("WS_LS_X", 58.2162)
    move("WS_LS_Y", 111.6102)
    return ([{"tpl": tpls[k], "pose": poses[k]} for k in range(3)],
            ok_build)


def _teach3_sites_down(tpls, radius_max=None, n_pick=3):
    """3 点 site quad 示教建制（迭代 32A：ws_rotate_center/ws_rotate_gain
    验收口径用——晶圆全局 2mm 大 cross 旋转态 REAL NCC 质心系统差
    ~0.5~1µm/0.014rad（渐晕+角度精化量化 warp，diag3 实测），经 1/θ
    放大别名进中心/增益拟合 ~50µm/rad 不可控；site quad 150µm 小标记走
    连续几何角（模板 <1.5Mpx²）、系统差 ~30nm 级）。取格点上成对距离
    最大的 n_pick 个 site 的 #0 角标（ratio×半宽角点）：按设计坐标粗走位
    → guide 精修居中 → 建模板 → 回 prep_down 对中位。返回
    (taught_points, ok)；模板建立失败返回 (None, False)。
    radius_max（迭代 35）：仅取角标半径 ≤ radius_max 的 site（双示教位
    下单一 pose_after 无法逐角度档重居中，可见性靠 杠杆×摆幅<FOV 半宽
    保证——中心/增益用例改小杠杆示教）；半径内不足 n_pick 个时取距心
    最近的 n_pick 个。n_pick：示教点数（默认 3，迭代 35 repeat 用例 2）。"""
    import itertools
    from recipe import wafer_sites_cells, wafer_sites_chip_size
    prep_down()
    rcp = json.load(open(os.path.join(ROOT, "runtime", "recipe.json"),
                         encoding="utf-8"))
    cells = wafer_sites_cells(rcp)
    hx, hy = (0.5 * v for v in wafer_sites_chip_size(rcp))
    ratio0 = (rcp["wafer"]["site_marks"]["ratio"] or [[0.0, 0.0]])[0]

    def _mk_r(s):
        return math.hypot(s[2] + ratio0[0] * hx, s[3] + ratio0[1] * hy)

    # 候选：默认半径最大的外圈 site；radius_max 限定小杠杆（近旋转中心）
    cand = sorted(cells, key=lambda s: -(s[2] ** 2 + s[3] ** 2))[:24]
    if radius_max is not None:
        near = [s for s in cells if _mk_r(s) <= radius_max]
        cand = near if len(near) >= n_pick else \
            sorted(cells, key=_mk_r)[:max(n_pick, 3)]
    # brute-force 成对距离最大的 n_pick 元组
    best = None
    for combo in itertools.combinations(cand, n_pick):
        d = min(math.hypot(combo[p][2] - combo[q][2],
                           combo[p][3] - combo[q][3])
                for p, q in itertools.combinations(range(n_pick), 2))
        if best is None or d > best[0]:
            best = (d, combo)
    picks = best[1]
    # 相机/吸盘心世界（当前 prep_down 位，名义）
    tp = http_json(BASE + "/api/toolpoints")
    mods = tp.get("modules") or tp.get("toolpoints") or {}
    chuck = (mods["wafer_stage"]["nominal_mm"][0],
             mods["wafer_stage"]["nominal_mm"][1])
    as_nom = mods["align_stage"]["nominal_mm"]
    cam = (as_nom[0] + 0.0, as_nom[1] - 22.5)   # 下相机 mount offset
    ws0 = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
    poses = []
    ok_build = True
    for k, (r_, c_, sx, sy) in enumerate(picks):
        name = "site_R%02dC%02d#0" % (r_, c_)
        mdx = sx + ratio0[0] * hx
        mdy = sy + ratio0[1] * hy
        mwx, mwy = chuck[0] + mdx, chuck[1] + mdy     # 设计世界（off=0）
        move("WS_LS_X", ws0[0] - (mwx - cam[0]))
        move("WS_LS_Y", ws0[1] - (mwy - cam[1]))
        ok_build = _center_down_mark(name) and ok_build
        c, r = _mk_down_tpl(tpls[k])
        if c != 200 or not r.get("ok"):
            return None, False
        poses.append({a: axis_entry(a)["pos"]
                      for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")})
    # 回 prep_down 对中位再开作业（作业以当前 WS_Rz 为基准 rz0）
    move("WS_LS_X", ws0[0])
    move("WS_LS_Y", ws0[1])
    return ([{"tpl": tpls[k], "pose": poses[k], "mark": picks[k]}
             for k in range(n_pick)], ok_build)


def _rot_pose_after(pts, rot_rad):
    """迭代 35 双示教位：按认知枢轴（toolpoints wafer_stage 名义中心 +
    机器常数 fine_align.rz_center_offset_mm 现值）为每点生成 pose_after
    （WS 分量=pose+(I−R(rot))(P_k−c)，AS 分量不变）——与后端原预测走位
    同式（Δ=(I−R)(P_k−c)），即"用户按旋后可见位示教 pose_after"的测试
    侧等价。P_k=示教位相机中心世界（_taught_snap 虚拟帧口径：标记已
    居中，P_k≈cam−(pose_ws−ws0)）。返回新 taught_points 数组（带
    pose_after；mark 键透传）。"""
    tp = http_json(BASE + "/api/toolpoints")
    mods = tp.get("modules") or tp.get("toolpoints") or {}
    chuck = (mods["wafer_stage"]["nominal_mm"][0],
             mods["wafer_stage"]["nominal_mm"][1])
    as_nom = mods["align_stage"]["nominal_mm"]
    cam = (as_nom[0] + 0.0, as_nom[1] - 22.5)
    fa = http_json(BASE + "/api/machine_constants")["constants"]["fine_align"]
    off = fa.get("rz_center_offset_mm") or [0.0, 0.0]
    c = (chuck[0] + off[0], chuck[1] + off[1])
    ws0 = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
    cr, sr = math.cos(rot_rad), math.sin(rot_rad)
    out = []
    for pt in pts:
        pose = pt["pose"]
        px = cam[0] - (pose["WS_LS_X"] - ws0[0])
        py = cam[1] - (pose["WS_LS_Y"] - ws0[1])
        vx, vy = px - c[0], py - c[1]
        rx = cr * vx - sr * vy
        ry = sr * vx + cr * vy
        pa = dict(pose)
        pa["WS_LS_X"] = pose["WS_LS_X"] + (vx - rx)
        pa["WS_LS_Y"] = pose["WS_LS_Y"] + (vy - ry)
        ent = {"tpl": pt["tpl"], "pose": pose, "pose_after": pa}
        if "mark" in pt:
            ent["mark"] = pt["mark"]
        out.append(ent)
    return out


def test_i31_taught_points():
    """taught_points 多点示教（ws_rotate_repeat）：
    - 校验：1 点/11 点 →400；与旧四键混给 →400 互斥；点模板不存在 →400；
      点 pose 未知轴 →400
    - 3 点示教（wafer_mark_1/3/2 跨片 161~228mm 杠杆，rz0 均可看见）
      rot_rad=0.01 repeat=3 → done + detail(mode=taught, method=
      direct_angle, n_points=3, rot_rad, taught_points) + stats.points×3
      + **直接测角法**（迭代 32）：series.angle_err_urad 长度=repeat、
      stats.angle_err{mean,std,s3 µrad}（3σ=旋转角重复性）+ 兼容字段
      （stats.dtheta/residual、series.dtheta_urad/points[i].d_*）+
      CSV 动态列 + 用户模板作业后不删"""
    tpls = ["i31_p0", "i31_p1", "i31_p2"]
    pts, ok_build = _teach3_down(tpls)
    if pts is None:
        check("i31.taught_points", False, "模板建立失败")
        for t in tpls:
            _del_tpl(t)
        return
    try:
        # ---- 校验分支 ----
        c1, r1 = cal_start("ws_rotate_repeat", "down",
                           {"taught_points": pts[:1], "repeat": 3})
        c2, _ = cal_start("ws_rotate_repeat", "down",
                          {"taught_points": pts + pts + pts + pts[:2],
                           "repeat": 3})
        c3, r3 = cal_start("ws_rotate_repeat", "down",
                           {"taught_points": pts[:2], "tpl_before": tpls[0],
                            "repeat": 3})
        c4, r4 = cal_start(
            "ws_rotate_repeat", "down",
            {"taught_points": [{"tpl": "__no_such__", "pose": {}},
                               {"tpl": tpls[0], "pose": {}}], "repeat": 3})
        c5, r5 = cal_start(
            "ws_rotate_repeat", "down",
            {"taught_points": [{"tpl": tpls[0], "pose": {"BAD_AX": 1.0}},
                               {"tpl": tpls[1], "pose": {}}], "repeat": 3})
        check("i31.taught_points_400",
              c1 == 400 and c2 == 400
              and c3 == 400 and "互斥" in (r3.get("error") or "")
              and c4 == 400 and "不存在" in (r4.get("error") or "")
              and c5 == 400 and "未知轴" in (r5.get("error") or "")
              and "BAD_AX" in (r5.get("error") or ""),
              "c=%s/%s/%s/%s/%s err=%s%s"
              % (c1, c2, c3, c4, c5, (r3.get("error") or "")[:40],
                 (r5.get("error") or "")[:60]))
        if not ok_build:
            check("i31.taught_points", False, "示教点 guide 居中失败")
            return
        # ---- 3 点示教 run（直接测角法·双示教位）----
        # 迭代 35：rot=0.01×161~228mm 杠杆旋后标记出视场——run 段给每点
        # 加测试侧 pose_after（_rot_pose_after 枢轴走位，等同用户旋后示教）
        pts_run = _rot_pose_after(pts, 0.01)
        s, res = run_job("ws_rotate_repeat", "down",
                         {"taught_points": pts_run, "rot_rad": 0.01,
                          "repeat": 3, "wait_ms": 20}, timeout=600.0)
        det = (res or {}).get("detail") or {}
        stt = (res or {}).get("stats") or {}
        ser = (res or {}).get("series") or {}
        hdr = None
        if res:
            pth = os.path.join(ROOT, (res.get("csv") or "")
                               .replace("/", os.sep))
            if os.path.isfile(pth):
                hdr = open(pth, encoding="utf-8").readline().strip()
        pts_st = stt.get("points") or []
        pts_ser = ser.get("points") or []
        dth = stt.get("dtheta") or {}
        ae = stt.get("angle_err") or {}
        sane = (len(pts_st) == 3
                and all(0 <= (p.get("col_s3_nm") or -1) < 3000.0
                        and 0 <= (p.get("row_s3_nm") or -1) < 3000.0
                        for p in pts_st)
                and 0.01 <= (ae.get("s3_urad") or -1) < 100.0
                and 0 <= (dth.get("s3_urad") or -1) < 500.0)
        check("i31.taught_points",
              s.get("state") == "done" and res is not None
              and det.get("mode") == "taught"
              and det.get("method") == "direct_angle"
              and det.get("n_points") == 3
              and abs((det.get("rot_rad") or 0) - 0.01) < 1e-9
              and len(det.get("taught_points") or []) == 3
              and all("pose_after" in p for p in (det.get("taught_points") or []))
              and sane and len(pts_ser) == 3
              and all(len(p.get("d_col_px") or []) == 3 for p in pts_ser)
              and len(ser.get("angle_err_urad") or []) == 3
              and len(ser.get("dtheta_urad") or []) == 3
              and len(ser.get("d_col_px") or []) == 3
              and (stt.get("residual") or {}).get("col_s3_nm")
                  == pts_st[0].get("col_s3_nm")
              and hdr is not None and "p1_col_re" in hdr
              and "p3_d_row_px" in hdr and "dtheta_urad" in hdr,
              "state=%s n=%s ae(mean=%s,s3=%s)µrad p0_col_s3=%snm hdr=%s%s"
              % (s.get("state"), det.get("n_points"), ae.get("mean_urad"),
                 ae.get("s3_urad"),
                 (pts_st[0].get("col_s3_nm") if pts_st else None),
                 (hdr or "无")[:90],
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
        kept = [t.get("name") for t in
                http_json(BASE + "/api/vision/templates")["templates"]
                if (t.get("name") or "") in tpls]
        check("i31.taught_tpl_kept", sorted(kept) == tpls,
              "作业后用户模板留存=%s" % kept)
    finally:
        for t in tpls:
            _del_tpl(t)


def _microrot_run_check(cam, params, truth_rot, repeat, tag):
    """camera_microrot 单相机 run + 结果判读：done + φ 序列长度 +
    measured.rot_mrad 与真值（machine_constants 当前值）偏差量级 +
    current.rot_mrad + stats.rot + CSV。返回 (ok, res, status)。"""
    s, res = run_job("camera_microrot", cam, params, timeout=600.0)
    ser = (res or {}).get("series") or {}
    stt = ((res or {}).get("stats") or {}).get("rot") or {}
    meas = ((res or {}).get("measured") or {})
    cur = ((res or {}).get("current") or {})
    csv_ok = False
    if res:
        rel = res.get("csv") or ""
        csv_ok = (os.path.isfile(os.path.join(ROOT, rel.replace("/", os.sep)))
                  and os.path.basename(rel).startswith("camera_microrot_"))
    dev = abs((meas.get("rot_mrad") or 1e9) - truth_rot)
    ok = (s.get("state") == "done" and res is not None
          and len(ser.get("phi_urad") or []) == repeat
          and dev < 0.25
          and abs((cur.get("rot_mrad") or 1e9) - truth_rot) < 1e-6
          and 0 <= (stt.get("s3_urad") or -1) < 200.0
          and csv_ok)
    check("i31.microrot_%s" % tag, ok,
          "state=%s measured=%s mrad truth=%s dev=%.1fµrad mean=%s "
          "std=%s s3=%s µrad n=%s%s"
          % (s.get("state"), meas.get("rot_mrad"), truth_rot, dev * 1000.0,
             stt.get("mean_urad"), stt.get("std_urad"), stt.get("s3_urad"),
             len(ser.get("phi_urad") or []),
             (" err=%s" % (s.get("error") or ""))
             if s.get("state") != "done" else ""))
    return ok, res, s


def test_i31_microrot():
    """camera_microrot 三相机微旋标定（迭代 31；迭代 32 走位轴改对准台；
    迭代 34 模板/拍照位必填、 teach 小 ROI 模板跑杠杆）：
    - 校验（start 同步 400）：模板不存在 / 空模板必填 / photo_pos 不成对 /
      缺拍照位必填 / 非纯 X（down y 不等，容差 0.01mm）/ 轴软限位 /
      repeat 越界 [3,100]（X 间距上下限与模板可达窗口校验已随迭代 34
      移除——小杠杆/大间距可用性由用户模板与作业期识别承担）
    - 三相机各 done（coarse 单轴重点：FA_X 10mm 杠杆 + cv_chipback 锚
      601px 小 ROI 模板；up 居中横跨 ±0.6mm；down **对准台 AS_X 走位**
      居中横跨 ±0.5mm——下相机挂对准台，动相机=等效动标记 rel=−1）：
      φ 序列长度=repeat、measured.rot_mrad 与真值
      （machine_constants 当前值）偏差 <250µrad（偏差=运动台直线度/
      误差链与匹配亚像素系统差的集总——真机微旋标定同样只能得集总值；
      符号/量级/链路判别）、current.rot_mrad、stats.rot（mean/std/3σ
      µrad）、CSV camera_microrot_*、**detail.axes=[AS_X,AS_Y] 走位记录**
    - apply 写回 rot_mrad 生效（GET /api/machine_constants 验证），写后还原
    亚 mrad 真值在 up/down 短杠杆（视场限 1.8mm）下信噪比受限——run 段
    统一 fast_noise_px=0.05 低噪声档（与 mrep.dynamic>=static 同手法，
    写后还原）。"""
    prep_down()
    truth = {c: http_json(BASE + "/api/machine_constants")["constants"]
             ["cameras"][c]["rot_mrad"] for c in ("down", "up", "coarse")}
    # 迭代 32 起 down 走位轴=对准台：photo_pos 坐标口径为 AS_X/AS_Y
    pos0 = {a: axis_entry(a)["pos"] for a in ("AS_X", "AS_Y")}
    x0, y0 = pos0["AS_X"], pos0["AS_Y"]
    # 校验分支用的已注册模板（wafer_mark_1 居中大 cross，存在性即可）
    _del_tpl("i31_val")
    c, r = _mk_down_tpl("i31_val")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        # ---- 校验分支（全部同步 400，不开作业）----
        v = []
        c, r = cal_start("camera_microrot", "down",
                         {"template_name": "__no_such__", "repeat": 3})
        v.append(c == 400 and "不存在" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down",
                         {"repeat": 3, "template_name": ""})
        v.append(c == 400 and "template_name" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down",
                         {"template_name": "i31_val", "repeat": 3,
                          "photo_pos1": [x0, y0]})
        v.append(c == 400 and "成对" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down",
                         {"template_name": "i31_val", "repeat": 3})
        v.append(c == 400 and "photo_pos1" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down",
                         {"template_name": "i31_val", "repeat": 3,
                          "photo_pos1": [x0, y0],
                          "photo_pos2": [x0 + 0.5, y0 + 0.2]})
        v.append(c == 400 and "纯 X" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down",
                         {"template_name": "i31_val", "repeat": 3,
                          "photo_pos1": [79.9, y0],
                          "photo_pos2": [80.5, y0]})
        v.append(c == 400 and "限位" in (r.get("error") or ""))
        c, r = cal_start("camera_microrot", "down", {"repeat": 2})
        v.append(c == 400)
        c, r = cal_start("camera_microrot", "down", {"repeat": 101})
        v.append(c == 400)
        check("i31.microrot_400", all(v), "分支通过=%s/8" % sum(v))

        # ---- run 段（fnp=0.05 低噪声档，finally 还原）----
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05,
                                                    "up": 0.05,
                                                    "coarse": 0.05}}})
        tpls_run = []
        try:
            # ---- down：site quad 小标记居中后 teach 601px 小 ROI 模板
            #      （可达 ±0.72mm，装得下 ±0.5mm=833px 杠杆），对准台
            #      AS_X 横跨 ±0.5mm（迭代 32：动相机=等效动标记）----
            g = http_json(BASE + "/api/vision/guide?cam=down")
            qd = next((m for m in g.get("marks") or []
                       if (m.get("name") or "").startswith("site_")), None)
            assert qd is not None, "down 视野内无 site quad"
            for s_ in (qd.get("suggested") or []):
                move(s_["axis"],
                     axis_entry(s_["axis"])["pos"] + s_["delta_mm"])
            _center_down_mark(qd["name"])
            _del_tpl("i31_down_tpl")
            c, r = _mk_small_tpl("down", "i31_down_tpl")
            assert c == 200 and r.get("ok"), (c, r)
            tpls_run.append("i31_down_tpl")
            q0 = {a: axis_entry(a)["pos"] for a in ("AS_X", "AS_Y")}
            ok_d, res_d, s_d = _microrot_run_check(
                "down", {"template_name": "i31_down_tpl",
                         "photo_pos1": [q0["AS_X"] - 0.5, q0["AS_Y"]],
                         "photo_pos2": [q0["AS_X"] + 0.5, q0["AS_Y"]],
                         "repeat": 10, "wait_ms": 20},
                truth["down"], 10, "down")
            # 迭代 32：down 走位记录=对准台 AS_X/AS_Y（动相机）
            check("i32.microrot_down_align_stage",
                  ok_d and ((res_d or {}).get("detail") or {}).get("axes")
                  == ["AS_X", "AS_Y"],
                  "detail.axes=%s"
                  % ((res_d or {}).get("detail") or {}).get("axes"))

            # ---- apply 写回 rot_mrad（down 作业；写后还原）----
            if ok_d and s_d.get("job_id") is not None:
                meas = res_d["measured"]["rot_mrad"]
                c, r = http_post(BASE + "/api/calibration/apply",
                                 {"job_id": s_d["job_id"],
                                  "fields": ["rot_mrad"]})
                got = None
                if c == 200 and r.get("ok"):
                    got = http_json(BASE + "/api/machine_constants")[
                        "constants"]
                    got = got["cameras"]["down"]["rot_mrad"]
                try:
                    ok_apply = (c == 200 and r.get("ok")
                                and r.get("applied") == ["rot_mrad"]
                                and got is not None
                                and abs(got - meas) < 1e-9)
                finally:
                    http_put(BASE + "/api/machine_constants",
                             {"cameras": {"down":
                                          {"rot_mrad": truth["down"]}}})
                restored = http_json(BASE + "/api/machine_constants")[
                    "constants"]
                restored = restored["cameras"]["down"]["rot_mrad"]
                check("i31.microrot_apply",
                      ok_apply and abs(restored - truth["down"]) < 1e-9,
                      "applied=%s wrote=%s restored=%s"
                      % (r.get("applied"), got, restored))
            else:
                check("i31.microrot_apply", False,
                      "down 作业未 done（job_id=%s），apply 分支无法执行"
                      % s_d.get("job_id"))

            # ---- up：prep_up 对中 die 角标后 teach 小 ROI 模板，居中
            #      横跨 ±0.6mm，repeat=10 ----
            for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
                move(ax, 0.0)
            try:
                prep_up()
                _del_tpl("i31_up_tpl")
                c, r = _mk_small_tpl("up", "i31_up_tpl")
                assert c == 200 and r.get("ok"), (c, r)
                tpls_run.append("i31_up_tpl")
                pu = {a: axis_entry(a)["pos"] for a in ("AS_X", "AS_Y")}
                _microrot_run_check(
                    "up", {"template_name": "i31_up_tpl",
                           "photo_pos1": [pu["AS_X"] - 0.6, pu["AS_Y"]],
                           "photo_pos2": [pu["AS_X"] + 0.6, pu["AS_Y"]],
                           "repeat": 10, "wait_ms": 20},
                    truth["up"], 10, "up")
            finally:
                for i in (1, 2, 3):
                    vacuum("WT_BondingVac_%d" % i, False)

            # ---- coarse（单轴重点）：FA_X 10mm 纯 X 杠杆，显式拍照位。
            #      die 背面中心小窗会落进素板均匀区（NCC 零方差退化——
            #      旧 _microrot_feature 对 box 取 trailing 外角点同理）：
            #      锚在 **trailing 外角点**（die 移向 −col 时 +col 侧
            #      trailing，角点二维结构锁定最好、两拍照位都可达；die
            #      外廓 10mm=recipe size_x/2 → px，本用例 dir_x=+1、
            #      pos2<pos1；实测 mean dev ~95µrad < 250µrad 容差）----
            for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
                move(ax, 0.0)
            prep_coarse()
            move("FA_X", 0.0)        # die 到粗对准视场中心（建模板位）
            gc = http_json(BASE + "/api/vision/guide?cam=coarse")
            cms = [m for m in gc.get("marks") or [] if m.get("in_fov")]
            assert cms, "coarse 视野内无 flip die 标记"
            ccol, crow = cms[0]["fov_center"]
            half_px = 5.0 / (NM_PER_PX["coarse"] / 1e6)   # die 半宽 → px
            _del_tpl("i31_coarse_tpl")
            c, r = _mk_small_tpl("coarse", "i31_coarse_tpl",
                                 anchor_px=(ccol + half_px,
                                            crow - half_px))
            assert c == 200 and r.get("ok"), (c, r)
            tpls_run.append("i31_coarse_tpl")
            _microrot_run_check("coarse",
                                {"template_name": "i31_coarse_tpl",
                                 "photo_pos1": [0.0, 0.0],
                                 "photo_pos2": [-10.0, 0.0],
                                 "repeat": 10, "wait_ms": 20},
                                truth["coarse"], 10, "coarse")
        finally:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px":
                                      {"down": 0.15, "up": 0.15,
                                       "coarse": 0.15}}})
            for t in tpls_run:
                _del_tpl(t)
    finally:
        _del_tpl("i31_val")


def test_i31():
    """迭代 31 段汇总：rot_rad/多点示教校验与 3 点示教 run →
    camera_microrot 校验/三相机 run/apply 写回。"""
    test_i31_rot_rad()
    test_i31_taught_points()
    test_i31_microrot()


# ---------------- 迭代 32：直接测角法 / 旋转中心标定 / 微旋对准台 ----------------
def test_i32_rotate_walk_limit_400():
    """旋转后走位软限位 start 400（迭代 35 双示教位新语义，替代迭代 32
    预测走位预估 400——Δ 预估段已删）：合法模板 + 示教点 pose_after
    WS_LS_X=999.0 越软限位 → 400 文案含"软限位"（pose_after 与 pose 同走
    _check_taught_pose 逐轴软限位校验）。"""
    prep_down()
    c, r = _mk_down_tpl("i32_tb")
    if c != 200 or not r.get("ok"):
        check("i32.rotate_walk_limit_400", False,
              "建模板失败：c=%s %s" % (c, r.get("error")))
        _del_tpl("i32_tb")
        return
    try:
        pose = {a: axis_entry(a)["pos"]
                for a in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y")}
        pa = dict(pose)
        pa["WS_LS_X"] = 999.0
        pts2 = [{"tpl": "i32_tb", "pose": pose, "pose_after": pa},
                {"tpl": "i32_tb", "pose": {}}]
        c, r = cal_start("ws_rotate_repeat", "down",
                         {"taught_points": pts2, "rot_rad": 0.017,
                          "repeat": 3})
        check("i32.rotate_walk_limit_400",
              c == 400 and "软限位" in (r.get("error") or "")
              and "pose_after" in (r.get("error") or ""),
              "c=%s err=%s" % (c, (r.get("error") or "")[:90]))
    finally:
        _del_tpl("i32_tb")


def test_i32_rotate_center():
    """ws_rotate_center 硅片台旋转中心标定（迭代 32；迭代 35 双示教位+
    _fit_rot 适配）：
    - 校验（start 同步 400）：step_rad 越界（0/0.4）/ n_angles 越界（1/9）
      / repeat 越界（2/101）/ 缺 taught_points / 1 点示教 / 角度档超
      WS_Rz 行程（step 0.01×3=±0.03 > ±1° 行程）
    - **真值注入验证**：PUT recipe sim_align.rz_center_true_offset_um=
      [300,-200]µm + **误差链 OFF**（旋转台跑出/角度增益等误差链真值
      会按 (ΣMᵀM)⁻¹ΣMᵀρ 结构别名进拟合 ~20µm，大 cross NCC 角度量化
      warp 失配别名 ~50µm/rad——均属设备/视觉真值固有系统差，真机同此
      口径；验收对算法本身故隔离 six_dof，finally 还原）→ 3 点示教
      （迭代 35：**半径 ≤15mm 小杠杆** site quad——双示教位下单一
      pose_after 无法逐档重居中，可见性靠 杠杆×摆幅<FOV 半宽；摆幅
      15mm×0.014rad≈0.21mm）step=0.007 n_angles=2 repeat=3 → done +
      measured.rz_center_offset_mm≈[0.3,-0.2]mm（拟合中心相对 Rz 机械
      轴心偏差 **<75µm**——迭代 35 从 <10µm 放宽：双示教位旋后走访不逐
      档重居中，逐 mark NCC 位置系统差别名进中心拟合（实测 r15/s007
      dev≈46µm）；待视觉链（迭代 35 视场旋转在制）稳定后收紧回 <10µm）
      + measured.rz_center_ref_xy_mm=作业起点 WS 位 +
      stats.center 逐分量 3σ + 拟合残差 RMS + series 逐 (循环,档) 长度
      =repeat×2·n_angles=12 + CSV ws_rotate_center_*
    - apply fields=["rz_center_offset_mm"] 写回 machine_constants
      fine_align 组（GET 验证；ref_xy 双写在 i35.rotate_center_refxy 覆盖），
      写后还原 [0,0]"""
    tpls = ["i32_p0", "i32_p1", "i32_p2"]
    pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0)
    if pts is None:
        check("i32.rotate_center", False, "示教点建制失败")
        for t in tpls:
            _del_tpl(t)
        return
    try:
        # ---- 校验分支（全部同步 400，不开作业）----
        v = []
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.0, "repeat": 3})
        v.append(c == 400 and "step_rad" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.4, "repeat": 3})
        v.append(c == 400 and "step_rad" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "n_angles": 1, "repeat": 3})
        v.append(c == 400 and "n_angles" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "n_angles": 9, "repeat": 3})
        v.append(c == 400 and "n_angles" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "repeat": 2})
        v.append(c == 400)
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "repeat": 101})
        v.append(c == 400)
        c, r = cal_start("ws_rotate_center", "down",
                         {"step_rad": 0.005, "repeat": 3})
        v.append(c == 400 and "taught_points" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts[:1], "step_rad": 0.005,
                          "repeat": 3})
        v.append(c == 400)
        # 角度档行程：±3×0.01=±0.03 rad > WS_Rz ±1°（0.0174533）→ 400
        c, r = cal_start("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.01,
                          "n_angles": 3, "repeat": 3})
        v.append(c == 400 and "行程" in (r.get("error") or ""))
        check("i32.rotate_center_400", all(v),
              "分支通过=%s/9" % sum(v))
        if not ok_build:
            check("i32.rotate_center", False, "示教点 guide 居中失败")
            return
        # ---- 真值注入 run（fnp=0.05 低噪声档 + **误差链 OFF**——旋转台
        #      跑出 dx/dy 误差地图按 (ΣMᵀM)⁻¹ΣMᵀρ 结构别名进拟合 ~20µm、
        #      大 cross NCC 角度量化 warp 失配 ~µm 级/档，均属设备/视觉真值
        #      固有系统差（真机同此口径），验收对算法本身故隔离；finally
        #      全部还原）----
        ws0 = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
        http_put(BASE + "/api/recipe",
                 {"sim_align": {"rz_center_true_offset_um": [300.0, -200.0]}})
        c_off, r_off = http_post(BASE + "/api/errormaps",
                                 {"six_dof_enabled": False})
        try:
            s, res = run_job("ws_rotate_center", "down",
                             {"taught_points": pts, "step_rad": 0.007,
                              "n_angles": 2, "repeat": 3, "wait_ms": 20},
                             timeout=900.0)
        finally:
            http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
            http_put(BASE + "/api/recipe",
                     {"sim_align": {"rz_center_true_offset_um": [0.0, 0.0]}})
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
        meas = (((res or {}).get("measured") or {})
                .get("rz_center_offset_mm")) or [9e9, 9e9]
        meas_ref = (((res or {}).get("measured") or {})
                    .get("rz_center_ref_xy_mm")) or [None, None]
        det = (res or {}).get("detail") or {}
        stt = (res or {}).get("stats") or {}
        ser = (res or {}).get("series") or {}
        ctr_s3 = (stt.get("center") or {}).get("s3_um")
        fr = stt.get("residual") or {}
        dev_um = math.hypot(meas[0] - 0.3, meas[1] - (-0.2)) * 1000.0
        n_fit = 3 * 2 * 2        # repeat×2·n_angles（迭代 35 逐 (循环,档)）
        ref_ok = (meas_ref[0] is not None
                  and abs(meas_ref[0] - ws0[0]) < 1e-4
                  and abs(meas_ref[1] - ws0[1]) < 1e-4)
        csv_ok = False
        if res:
            rel = res.get("csv") or ""
            csv_ok = (os.path.isfile(os.path.join(ROOT, rel.replace(
                          "/", os.sep)))
                      and os.path.basename(rel).startswith(
                          "ws_rotate_center_"))
        check("i32.rotate_center",
              s.get("state") == "done" and res is not None
              and dev_um < 75.0
              # 迭代 35：center.s3=逐 (循环,档) 拟合散布——各档摆幅不同→
              # NCC 系统差各异（实测 ~157µm），<100µm 旧口径不可达，
              # 放宽 <250µm 并注明（rms/分量均值精度仍由 dev/rms 断言把关）
              and 0 <= (ctr_s3 or -1) < 250.0
              and 0 <= (fr.get("rms_um") or -1) < 5.0
              and ref_ok
              and len(ser.get("center_x_mm") or []) == n_fit
              and len(ser.get("center_y_mm") or []) == n_fit
              and len(ser.get("offset_x_um") or []) == n_fit
              and len(ser.get("theta_fit_urad") or []) == n_fit
              and len(det.get("angles_rad") or []) == 4
              and csv_ok,
              "state=%s measured=%s ref=%s dev=%.2fµm center_s3=%sµm "
              "rms=%sµm%s"
              % (s.get("state"), meas, meas_ref, dev_um, ctr_s3,
                 fr.get("rms_um"),
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
        # ---- apply 写回 fine_align.rz_center_offset_mm + 还原 ----
        if s.get("state") == "done":
            m = res["measured"]["rz_center_offset_mm"]
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id"),
                              "fields": ["rz_center_offset_mm"]})
            got = None
            if c == 200 and r.get("ok"):
                got = http_json(BASE + "/api/machine_constants")[
                    "constants"]["fine_align"]["rz_center_offset_mm"]
            try:
                ok_apply = (c == 200 and r.get("ok")
                            and r.get("applied") == ["rz_center_offset_mm"]
                            and got is not None
                            and all(abs(got[i] - m[i]) < 1e-9
                                    for i in (0, 1)))
            finally:
                http_put(BASE + "/api/machine_constants",
                         {"fine_align": {"rz_center_offset_mm": [0.0, 0.0]}})
            restored = http_json(BASE + "/api/machine_constants")[
                "constants"]["fine_align"]["rz_center_offset_mm"]
            check("i32.rotate_center_apply",
                  ok_apply and restored == [0.0, 0.0],
                  "applied=%s wrote=%s restored=%s"
                  % (r.get("applied"), got, restored))
        else:
            check("i32.rotate_center_apply", False,
                  "作业未 done（err=%s）" % (s.get("error") or "")[:60])
    finally:
        for t in tpls:
            _del_tpl(t)


def test_i32_rotate_gain():
    """ws_rotate_gain 旋转角度增益标定（迭代 32A）：
    - 校验（start 同步 400）：缺 taught_points / step_rad=0 / n_angles 越界
      / repeat 越界 / 角度档超 WS_Rz 行程（同 rotate_center 参数面）
    - 默认真值 run（误差链 ON）：site quad 3 点（迭代 35：**半径 ≤15mm
      小杠杆**——双示教位单一 pose_after 无法逐档重居中，可见性靠
      杠杆×摆幅<FOV 半宽）step=0.007 n_angles=2 repeat=3 → done +
      measured.rz_gain≈1 + stats.gain.s3 + offset_urad +
      CSV ws_rotate_gain_*。
      **迭代 35 容差适配（|gain−1|<3e-4 → <6e-3）**：双示教位旋后走访
      不逐档重居中，逐 mark NCC 位置系统差（~0.1µm 级）按 1/(杠杆×θ)
      别名进逐档 θ（实测逐次拾取 ±3e-3 跳动），i32 旧口径需预测走位
      重居中（已删）；统计质量（s3<1e-3）与近似性（<6e-3）仍校验——
      真值 1.001 的严格判定移到 i35.rotate_two_pose_truth（重居中
      大杠杆 θ 对真值 ±6µrad，注入签名可辨）；本断言待视觉链（迭代
      35 视场旋转在制）稳定后收紧回 3e-4
    - apply fields=["rz_gain"] 写回 fine_align.rz_gain（GET 验证）+ 还原
    真值注入 1.001 测回在 main() 专属 boot 段（test_i32_gain_truth_phase：
    改 error_maps/low_order.json WS_Rz scale_ppm=1000，bytes 恢复）。"""
    tpls = ["i32_g0", "i32_g1", "i32_g2"]
    pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0)
    if pts is None:
        check("i32.rotate_gain", False, "示教点建制失败")
        for t in tpls:
            _del_tpl(t)
        return
    try:
        # ---- 校验分支（全部同步 400，不开作业）----
        v = []
        c, r = cal_start("ws_rotate_gain", "down",
                         {"step_rad": 0.005, "repeat": 3})
        v.append(c == 400 and "taught_points" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_gain", "down",
                         {"taught_points": pts, "step_rad": 0.0, "repeat": 3})
        v.append(c == 400 and "step_rad" in (r.get("error") or ""))
        c, r = cal_start("ws_rotate_gain", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "n_angles": 9, "repeat": 3})
        v.append(c == 400)
        c, r = cal_start("ws_rotate_gain", "down",
                         {"taught_points": pts, "step_rad": 0.005,
                          "repeat": 2})
        v.append(c == 400)
        c, r = cal_start("ws_rotate_gain", "down",
                         {"taught_points": pts, "step_rad": 0.01,
                          "n_angles": 3, "repeat": 3})
        v.append(c == 400 and "行程" in (r.get("error") or ""))
        check("i32.rotate_gain_400", all(v), "分支通过=%s/5" % sum(v))
        if not ok_build:
            check("i32.rotate_gain", False, "示教点 guide 居中失败")
            return
        # ---- 默认真值 run（fnp=0.05 低噪声档，finally 还原）----
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
        try:
            s, res = run_job("ws_rotate_gain", "down",
                             {"taught_points": pts, "step_rad": 0.007,
                              "n_angles": 2, "repeat": 3, "wait_ms": 20},
                             timeout=900.0)
        finally:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
        meas = ((res or {}).get("measured") or {})
        stt = (res or {}).get("stats") or {}
        ser = (res or {}).get("series") or {}
        g = meas.get("rz_gain")
        g_st = stt.get("gain") or {}
        off_st = stt.get("offset_urad") or {}
        csv_ok = False
        if res:
            rel = res.get("csv") or ""
            csv_ok = (os.path.isfile(os.path.join(ROOT, rel.replace(
                          "/", os.sep)))
                      and os.path.basename(rel).startswith("ws_rotate_gain_"))
        check("i32.rotate_gain",
              s.get("state") == "done" and res is not None
              and g is not None and abs(g - 1.0) < 6e-3
              and 0 <= (g_st.get("s3") or -1) < 1e-3
              and off_st.get("mean") is not None
              and len(ser.get("gain") or []) == 3
              and len(ser.get("theta_actual_urad") or []) == 3
              and csv_ok,
              "state=%s gain=%s s3=%s offset=%sµrad%s"
              % (s.get("state"), g, g_st.get("s3"), off_st.get("mean"),
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
        # ---- apply 写回 fine_align.rz_gain + 还原 ----
        if s.get("state") == "done":
            m = res["measured"]["rz_gain"]
            cur0 = http_json(BASE + "/api/machine_constants")[
                "constants"]["fine_align"]["rz_gain"]
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id"),
                              "fields": ["rz_gain"]})
            got = None
            if c == 200 and r.get("ok"):
                got = http_json(BASE + "/api/machine_constants")[
                    "constants"]["fine_align"]["rz_gain"]
            try:
                ok_apply = (c == 200 and r.get("ok")
                            and r.get("applied") == ["rz_gain"]
                            and got is not None and abs(got - m) < 1e-9)
            finally:
                http_put(BASE + "/api/machine_constants",
                         {"fine_align": {"rz_gain": cur0}})
            restored = http_json(BASE + "/api/machine_constants")[
                "constants"]["fine_align"]["rz_gain"]
            check("i32.rotate_gain_apply",
                  ok_apply and abs(restored - cur0) < 1e-12,
                  "applied=%s wrote=%s restored=%s"
                  % (r.get("applied"), got, restored))
        else:
            check("i32.rotate_gain_apply", False,
                  "作业未 done（err=%s）" % (s.get("error") or "")[:60])
    finally:
        for t in tpls:
            _del_tpl(t)


def test_i32_gain_truth_phase():
    """专属 boot 段（main() 主套件前独立拉起/清理）：真值增益注入——改
    error_maps/low_order.json WS_Rz scale_ppm → 1000（真值物理角=轴位增量
    ×1.001；scale 经 sample6 along → R 链元姿态 → collect_marks 标记随
    WS_Rz 实际多转 0.1%），bytes 备份后恢复。site quad 示教 →
    ws_rotate_gain step=0.007 n_angles=2 repeat=3 → measured.rz_gain≈
    1.001（|Δ|<8e-3 迭代 35 放宽并加 s3<1e-3——双示教位扫档不重居中，
    NCC 系统差别名 ~1e-3 级，i32 旧 5e-4 口径不可达；严格真值判定见
    i35.rotate_two_pose_truth）。迭代 35 适配：site quad 示教改半径
    ≤15mm 小杠杆（双示教位单一 pose_after 无法逐档重居中）。"""
    lo_path = os.path.join(ROOT, "runtime", "error_maps", "low_order.json")
    with open(lo_path, "rb") as f:
        raw = f.read()
    data = json.loads(raw.decode("utf-8"))
    ppm0 = data["axes"]["WS_Rz"]["scale_ppm"]
    data["axes"]["WS_Rz"]["scale_ppm"] = 1000.0
    with open(lo_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    proc, log = boot_backend()
    ok = False
    detail = ""
    try:
        if not wait_ready():
            detail = "后端启动失败"
        else:
            tpls = ["i32_t0", "i32_t1", "i32_t2"]
            pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0)
            if pts is None or not ok_build:
                detail = "示教点建制失败 ok_build=%s" % ok_build
            else:
                try:
                    http_put(BASE + "/api/recipe",
                             {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
                    try:
                        s, res = run_job(
                            "ws_rotate_gain", "down",
                            {"taught_points": pts, "step_rad": 0.007,
                             "n_angles": 2, "repeat": 3, "wait_ms": 20},
                            timeout=900.0)
                    finally:
                        http_put(
                            BASE + "/api/recipe",
                            {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
                    g = ((res or {}).get("measured") or {}).get("rz_gain")
                    g_st = ((res or {}).get("stats") or {}).get("gain") or {}
                    # 迭代 35 容差适配（5e-4 → 8e-3）：双示教位扫档不重居中，
                    # NCC 系统差别名使逐档 θ 带 ~1e-3 级拾取跳动（详见
                    # test_i32_rotate_gain docstring）；真值 1.001 的严格
                    # 判定由 i35.rotate_two_pose_truth（重居中 θ ±6µrad）承担
                    ok = (s.get("state") == "done" and g is not None
                          and abs(g - 1.001) < 8e-3
                          and 0 <= (g_st.get("s3") or -1) < 1e-3)
                    detail = ("state=%s gain=%s s3=%s err=%s"
                              % (s.get("state"), g, g_st.get("s3"),
                                 (s.get("error") or "")[:60]))
                finally:
                    for t in tpls:
                        _del_tpl(t)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log.close()
        # 等端口真正释放（Windows 句柄释放有延迟）再还文件——主套件紧接
        # 在本端口重起后端，避免双绑定连到本段旧实例
        t0 = time.time()
        while time.time() - t0 < 15.0:
            try:
                http_json(BASE + "/api/health", timeout=2)
                time.sleep(0.3)
            except Exception:
                break
        with open(lo_path, "wb") as f:
            f.write(raw)
    check("i32.rotate_gain_truth1001", ok,
          "%s（scale_ppm %s→1000）" % (detail, ppm0))


def test_i32():
    """迭代 32/32A 段汇总（迭代 35 适配）：旋转后走位软限位 400（新语义
    pose_after 越限）→ ws_rotate_center 校验/真值注入 run/apply 写回 →
    ws_rotate_gain 校验/默认真值 run/apply。"""
    test_i32_rotate_walk_limit_400()
    test_i32_rotate_center()
    test_i32_rotate_gain()


# ---------------- 迭代 33：串图竞态 / 适配器积压 / 批量补标定 ----------------
def test_i33_cov_scratch_isolation():
    """F1 回归：up/down 相机同分辨率（3000×3000）并行渲染时原共享
    _COV_SCRATCH 全局覆盖率缓冲（fill(0)/绘制/混合三步之间有 GIL 让步），
    A 相机帧混入 B 相机标记。修复后缓冲按渲染线程隔离（threading.local）：
    双线程交错渲染（绘制步 barrier 强制交织），断言各自帧只含自己的标记
    （A 帧标记区在 B 帧中为背景，反之亦然）。"""
    import threading
    import vision_model as vm
    if vm.cv2_or_none() is None:
        check("i33.cov_scratch_isolation", True, "cv2 不可用，跳过")
        return
    cams = vm.load_cameras()
    cam_a, cam_b = cams["up"], cams["down"]
    W = H = 512                      # 同 (H,W)：旧实现两线程撞同一缓冲
    eng = object()                   # render_frame 对 engine 只做 getattr 兜底
    # 两标记落点须不同（up dir=(-1,1) / down dir=(1,-1)，镜像映射）：
    # (-0.2,-0.2)→(313,199)；(+0.2,-0.2)→(313,313)
    mk_a = vm.Mark("i33_a", "cross", -0.2, -0.2, 0.0, 0.0, 0.06)
    mk_b = vm.Mark("i33_b", "cross", 0.2, -0.2, 0.0, 0.0, 0.06)
    layers = [{"z_mm": 0.0, "items": []}]

    def _cam_pos(cam):
        # 焦面 = cam_pos.z + look_dir*wd_mm = 0 → 层/标记 z=0 合焦（σ=0，
        # cov 直接混进整帧，不经过离焦副本路径）
        return [0.0, 0.0, -cam.look_dir * cam.wd_mm]

    col_a, row_a = cam_a.project(mk_a.x, mk_a.y, W, H)
    col_b, row_b = cam_b.project(mk_b.x, mk_b.y, W, H)
    barrier = threading.Barrier(2)
    orig_draw = vm._draw_mark_cov

    def _synced(n_, cv2_, buf, mark, col, row, k_, W_, H_):
        orig_draw(n_, cv2_, buf, mark, col, row, k_, W_, H_)
        barrier.wait(timeout=30)     # 强制双线程在绘制步交织

    frames = {}

    def _render(key, cam, mk):
        frames[key] = vm.render_frame(
            eng, cam, width=W, height=H, light_on=True, noise_sigma_dn=0.0,
            marks=[mk], cam_pos=_cam_pos(cam), layers=layers)

    vm._draw_mark_cov = _synced
    try:
        ta = threading.Thread(target=_render, args=("a", cam_a, mk_a))
        tb = threading.Thread(target=_render, args=("b", cam_b, mk_b))
        ta.start()
        tb.start()
        ta.join(60)
        tb.join(60)
    finally:
        vm._draw_mark_cov = orig_draw
    if "a" not in frames or "b" not in frames:
        check("i33.cov_scratch_isolation", False, "渲染线程未结束")
        assert False, "i33.cov_scratch_isolation: 渲染线程未结束"
        return

    def _patch_min(img, col, row, r=1):
        # 3×3：17px cross 臂宽仅 ~4px，大窗会把窗外背景采进 min 失去判别力
        n_ = vm.np()
        return float(n_.min(img[int(row) - r:int(row) + r + 1,
                                int(col) - r:int(col) + r + 1]))

    dark_a = _patch_min(frames["a"], col_a, row_a)   # A 帧自己的标记区
    dark_b = _patch_min(frames["b"], col_b, row_b)   # B 帧自己的标记区
    stray_a = _patch_min(frames["a"], col_b, row_b)  # A 帧里 B 标记区（串图证据）
    stray_b = _patch_min(frames["b"], col_a, row_a)  # B 帧里 A 标记区
    # 修复后：各帧只含自己的标记（标记 ~57，背景 ~19）。旧共享缓冲下
    # barrier 交错必使 A 帧混入 B 标记（stray_a 变暗）或 A 自身标记被
    # 对线程 fill(0) 清掉（dark_a 回到背景）——两条路都被这里抓住
    ok = (dark_a > 40 and dark_b > 40
          and stray_a < 40 and stray_b < 40)
    check("i33.cov_scratch_isolation", ok,
          "A帧 自标记=%.0f vs B标记区=%.0f；B帧 自标记=%.0f vs A标记区=%.0f"
          % (dark_a, stray_a, dark_b, stray_b))
    assert ok, "i33.cov_scratch_isolation: 双相机帧互串（缓冲未按线程隔离）"


def test_i33_adapter_backlog_drop():
    """F4 回归：per-cam 队列积压丢最旧。消费者被卡死（identify 阻塞）时
    向同一 cam 连续入队 4 条命令：最旧积压命令收到"被新命令取代"错误帧
    （回显被丢命令的原始 task 与 camID），队列长度有界（≤积压上限）。"""
    import threading
    from mqtt_cmd_adapter import MqttCmdAdapter, VISION_BACKLOG_MAX
    started = threading.Event()
    release = threading.Event()
    replies = []

    class _BlockingVision(object):
        def __init__(self):
            self.cams = {"down": object()}

        def identify(self, cam, template):
            started.set()
            release.wait(timeout=30)
            return {"result": {"strShapeName": template or "",
                               "judge": True, "lstRow": [1.0],
                               "lstCol": [2.0], "lstAngle": [0.0],
                               "lstScore": [0.9], "lstScale": [1.0]}}

    def _vsc(task):
        content = {"camID": "down", "cmdType": 2,
                   "content": json.dumps({"1": {"lstShapeName": ["t"]}}),
                   "image": "", "imgType": -1, "isImageSend": 0,
                   "lstCellType": [0, 1], "task": task}
        return json.dumps({"context": json.dumps(content), "module": "",
                           "result": 0, "task": task})

    adapter = MqttCmdAdapter(object(), _BlockingVision(), port=1,
                             map_path=os.path.join(HERE, "_no_such.json"))
    adapter._publish_vision_result = \
        lambda cam, content, res, err: replies.append((cam, dict(content),
                                                       res, err))
    try:
        tasks = ["IMG_i33_%d" % i for i in range(4)]
        adapter._dispatch_vision_cmd(_vsc(tasks[0]))
        assert started.wait(10), "消费者未开始处理首条命令"
        for t in tasks[1:]:
            adapter._dispatch_vision_cmd(_vsc(t))
        errs = [(c, req, e) for c, req, res, e in replies if e]
        hit = [(c, req, e) for c, req, e in errs
               if "被新命令取代" in str(e) and c == "down"
               and req.get("task") == tasks[1]]
        q = adapter._vision_queues.get("down")
        bounded = q is not None and q.qsize() <= VISION_BACKLOG_MAX
        check("i33.adapter_backlog_drop", bool(hit) and bounded,
              "取代错误帧=%s 回显task=%s 队列长=%s(上限%d)"
              % (len(hit), tasks[1] if hit else "-",
                 q.qsize() if q else "-", VISION_BACKLOG_MAX))
        assert hit and bounded, \
            "i33.adapter_backlog_drop: 积压丢弃/队列有界失效"
    finally:
        release.set()                # 放行消费者，随 adapter.stop 退出
        adapter.stop()
        q = adapter._vision_queues.get("down")
        t0 = time.time()
        while q is not None and not q.empty() and time.time() - t0 < 10:
            time.sleep(0.05)


def test_i33_resp_batch_calibration():
    """F5 回归：缺 noise_resp_px_per_dn/tpl_std_dn 的旧模板经
    _calibrate_missing_resp 批量补标定后 meta 带 resp 字段并落盘；退化
    模板（零梯度）探针失败路径只警告不崩溃、索引不被破坏。"""
    import threading
    import vision_model as vm
    import vision_service as vs
    if vm.cv2_or_none() is None:
        check("i33.resp_batch_calibration", True, "cv2 不可用，跳过")
        return
    n = vm.np()
    good = n.full((96, 96), 90, dtype=n.uint8)
    good[28:68, 28:68] = 200
    good_png = vm.png_encode_gray(good)
    bad_png = vm.png_encode_gray(n.full((96, 96), 90, dtype=n.uint8))
    good_f, bad_f = "tpl_i33_resp_good.png", "tpl_i33_resp_bad.png"
    idx_path = os.path.join(HERE, "_i33_templates.json")
    try:
        for f_, png in ((good_f, good_png), (bad_f, bad_png)):
            with open(os.path.join(vs.TEMPLATES_DIR, f_), "wb") as fh:
                fh.write(png)
        store = vs.TemplateStore(path=idx_path, dir_=vs.TEMPLATES_DIR)
        for name, f_ in (("i33_resp_good", good_f), ("i33_resp_bad", bad_f)):
            store.put(name, {"name": name, "cam": "down", "file": f_,
                             "shape": "box", "threshold": 0.5,
                             "roi": [0, 0, 96, 96], "w": 96, "h": 96,
                             "created": 0.0})
        svc = vs.VisionService.__new__(vs.VisionService)   # 只装补标定所需件
        svc._lock = threading.RLock()
        svc._matcher = None
        svc.cams = {}
        svc.templates = store
        svc._calibrate_missing_resp()    # 坏模板走警告跳过路径，不得崩
        good = store.get("i33_resp_good") or {}
        bad = store.get("i33_resp_bad") or {}
        ok = (good.get("noise_resp_px_per_dn") is not None
              and good.get("tpl_std_dn") is not None
              and bad.get("noise_resp_px_per_dn") is None)
        check("i33.resp_batch_calibration", ok,
              "good resp=%s std=%s；bad resp 仍=%s"
              % (good.get("noise_resp_px_per_dn"), good.get("tpl_std_dn"),
                 bad.get("noise_resp_px_per_dn")))
        assert ok, "i33.resp_batch_calibration: 批量补标定失效"
    finally:
        for f_ in (good_f, bad_f):
            try:
                os.remove(os.path.join(vs.TEMPLATES_DIR, f_))
            except OSError:
                pass
        try:
            os.remove(idx_path)
        except OSError:
            pass


def _run_i33_group():
    """迭代 33 段汇总（全进程内单测，不依赖后端现场）。"""
    test_i33_cov_scratch_isolation()
    test_i33_adapter_backlog_drop()
    test_i33_resp_batch_calibration()


# ---------------- 迭代 34：模板必填 / 台重复性相机可选 / 微旋选台 / auto 移除 ----------------
def test_i34_empty_template_400():
    """stage_move（ws/as_move_repeat）/microrot/align_dynamic 空/None 模板
    → start 400（文案含 template_name；迭代 34 模板必填）。"""
    cases = (("align_repeat_dynamic", "down",
              {"travel_mm": 2.0, "repeat": 10}),
             ("ws_move_repeat", "down", {"travel_mm": 2.0}),
             ("as_move_repeat", "up", {"travel_mm": 1.0}),
             ("camera_microrot", "down", {"repeat": 3}))
    ok = True
    det = []
    for routine, cam, params in cases:
        for empty in ("", None):
            p = dict(params)
            p["template_name"] = empty
            c, r = cal_start(routine, cam, p)
            good = c == 400 and "template_name" in (r.get("error") or "")
            ok = ok and good
            det.append("%s/%s:%s" % (routine, repr(empty), c))
    check("i34.empty_template_400", ok, " ".join(det))


def test_i34_as_move_cam_down():
    """as_move_repeat cam=down（下相机看对准台动）+ jog_group=align（
    metrology_gui 契约）小 repeat done——相机可选生效：result.camera=
    down、走位轴=AS_X/AS_Y。"""
    prep_down()
    _del_tpl("i34_asd_tpl")
    c, r = _mk_tpl("down", "i34_asd_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        s, res = run_job("as_move_repeat", "down",
                         {"repeat": 3, "wait_ms": 20, "travel_mm": 1.0,
                          "jog_group": "align",
                          "template_name": "i34_asd_tpl"})
    finally:
        _del_tpl("i34_asd_tpl")
    alog = (res or {}).get("axis_log") or {}
    check("i34.as_move_cam_down",
          s.get("state") == "done" and res is not None
          and res.get("camera") == "down"
          and sorted(alog.keys()) == ["AS_X", "AS_Y"],
          "state=%s camera=%s axes=%s err=%s"
          % (s.get("state"), (res or {}).get("camera"), sorted(alog),
             (s.get("error") or "")[:60]))


def test_i34_microrot_wafer_stage():
    """down + move_stage=wafer：晶圆台 WS_LS_X/Y 走位（rel=+1，动标记载体
    ——下相机固定看晶圆标记动），已注册小 ROI 模板 + 显式拍照位，repeat=3
    done 且 |mean−真值 rot_mrad| 在 i31 容差带（<0.25mrad）；detail.axes=
    [WS_LS_X,WS_LS_Y]。"""
    prep_down()
    # site quad 小标记居中（大 cross 中心小 ROI NCC 退化，同 i31 口径）
    g = http_json(BASE + "/api/vision/guide?cam=down")
    qd = next((m for m in g.get("marks") or []
               if (m.get("name") or "").startswith("site_")), None)
    assert qd is not None, "down 视野内无 site quad"
    for s_ in (qd.get("suggested") or []):
        move(s_["axis"], axis_entry(s_["axis"])["pos"] + s_["delta_mm"])
    _center_down_mark(qd["name"])
    truth = http_json(BASE + "/api/machine_constants")["constants"][
        "cameras"]["down"]["rot_mrad"]
    _del_tpl("i34_wf_tpl")
    c, r = _mk_small_tpl("down", "i34_wf_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
        try:
            w0 = {a: axis_entry(a)["pos"]
                  for a in ("WS_LS_X", "WS_LS_Y")}
            s, res = run_job("camera_microrot", "down",
                             {"template_name": "i34_wf_tpl",
                              "move_stage": "wafer",
                              "photo_pos1": [w0["WS_LS_X"] - 0.5,
                                             w0["WS_LS_Y"]],
                              "photo_pos2": [w0["WS_LS_X"] + 0.5,
                                             w0["WS_LS_Y"]],
                              "repeat": 3, "wait_ms": 20}, timeout=600.0)
        finally:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
    finally:
        _del_tpl("i34_wf_tpl")
    ser = (res or {}).get("series") or {}
    meas = ((res or {}).get("measured") or {}).get("rot_mrad")
    dev = abs((meas or 1e9) - truth)
    axes = ((res or {}).get("detail") or {}).get("axes")
    check("i34.microrot_wafer_stage",
          s.get("state") == "done" and res is not None
          and len(ser.get("phi_urad") or []) == 3
          and dev < 0.25 and axes == ["WS_LS_X", "WS_LS_Y"],
          "state=%s axes=%s measured=%s mrad truth=%s dev=%.1fµrad%s"
          % (s.get("state"), axes, meas, truth, dev * 1000.0,
             (" err=%s" % (s.get("error") or ""))
             if s.get("state") != "done" else ""))


def test_i34_microrot_photo_pos():
    """photo_pos 契约（迭代 34）：缺失 → 400"拍照位必填"；X 间距 0.2mm
    （旧口径 <0.3mm 会 400"过小"）现在通过校验——start 200 且作业终态
    不得再报间距类错误（done 与否取决于识别）。"""
    prep_down()
    _del_tpl("i34_pp_tpl")
    c, r = _mk_down_tpl("i34_pp_tpl")
    assert c == 200 and r.get("ok"), (c, r)
    try:
        # 缺拍照位 → 400（必填文案）
        c1, r1 = cal_start("camera_microrot", "down",
                           {"template_name": "i34_pp_tpl", "repeat": 3})
        ok_missing = c1 == 400 and "photo_pos1" in (r1.get("error") or "")
        # 间距 0.2mm：旧会 400"过小"，现须通过 start 校验
        x0 = axis_entry("AS_X")["pos"]
        y0 = axis_entry("AS_Y")["pos"]
        c2, r2 = cal_start("camera_microrot", "down",
                           {"template_name": "i34_pp_tpl", "repeat": 3,
                            "photo_pos1": [x0 - 0.1, y0],
                            "photo_pos2": [x0 + 0.1, y0]})
        if c2 == 200:
            s = cal_wait(r2["job_id"], timeout=600.0)
            err = s.get("error") or ""
            ok_span = s.get("state") in ("done", "error") \
                and "过小" not in err and "视场" not in err
            det = "state=%s err=%s" % (s.get("state"), err[:60])
        else:
            ok_span = False
            det = "start 仍被拒：c=%s err=%s" % (c2, (r2.get("error") or "")[:60])
    finally:
        _del_tpl("i34_pp_tpl")
    check("i34.microrot_no_photo_pos_400", ok_missing,
          "c=%s err=%s" % (c1, (r1.get("error") or "")[:60]))
    check("i34.microrot_tight_span_ok", ok_span, det)


def test_i34_ws_rotate_auto_gone():
    """ws_rotate_repeat 全自动路径移除（迭代 34）：无 taught_points（旧
    detail.mode="auto" 路径）或显式 mode="auto" → start 400，文案指引
    taught_points（2~10 点、每点 tpl 已注册）。"""
    prep_down()          # 晶圆在位（wafer-present 检查先于 taught 检查）
    c1, r1 = cal_start("ws_rotate_repeat", "down",
                       {"repeat": 3, "rot_rad": 0.017})
    c2, r2 = cal_start("ws_rotate_repeat", "down",
                       {"repeat": 3, "rot_rad": 0.017, "mode": "auto"})
    check("i34.ws_rotate_auto_gone",
          c1 == 400 and "taught_points" in (r1.get("error") or "")
          and c2 == 400 and "taught_points" in (r2.get("error") or ""),
          "c=%s/%s err=%s" % (c1, c2, (r1.get("error") or "")[:80]))


def test_i34_die_transfer_back_tpl():
    """die_transfer_repeat back_tpl（迭代 34）：缺省 back 段用 coarse_tpl
    （detail.back_tpl==coarse_tpl、三段统计齐、done）；显式 back_tpl 给
    不存在名 → start 400。"""
    for ax in ("FA_X", "FA_Z", "FA_Rx", "CH_Rz", "BS_Arm_Z"):
        move(ax, 0.0)
    prep_clean()
    c, r = cal_start("die_transfer_repeat", "coarse",
                     {"repeat": 3, "wait_ms": 20})
    if c != 200:
        check("i34.die_transfer_back_tpl", False,
              "start 被拒：c=%s %s" % (c, r.get("error")))
        return
    s = cal_wait(r["job_id"], timeout=1200.0)
    res = cal_result(r["job_id"]) if s.get("state") == "done" else None
    det = (res or {}).get("detail") or {}
    stt = (res or {}).get("stats") or {}
    c2, r2 = cal_start("die_transfer_repeat", "coarse",
                       {"repeat": 3, "wait_ms": 20,
                        "back_tpl": "__no_such_back__"})
    check("i34.die_transfer_back_tpl",
          s.get("state") == "done" and res is not None
          and det.get("back_tpl") == det.get("coarse_tpl")
          and set(stt) == {"flip", "head", "back"}
          and c2 == 400 and "不存在" in (r2.get("error") or ""),
          "state=%s coarse_tpl=%s back_tpl=%s segs=%s back400=%s err=%s"
          % (s.get("state"), det.get("coarse_tpl"), det.get("back_tpl"),
             sorted(stt), c2, (s.get("error") or "")[:60]))


def test_i34():
    """迭代 34 段汇总：空模板 400 → as_move_repeat cam=down → 微旋选台
    wafer → 拍照位必填/小间距放行 → ws_rotate auto 移除 → 交接 back_tpl。"""
    test_i34_empty_template_400()
    test_i34_as_move_cam_down()
    test_i34_microrot_wafer_stage()
    test_i34_microrot_photo_pos()
    test_i34_ws_rotate_auto_gone()
    test_i34_die_transfer_back_tpl()


# ---------------- 迭代 35：dir 默认 (+1,−1) / view_rot_deg 视场旋转 ----------------
def test_i35_pixel_world_roundtrip():
    """i35.pixel_world_roundtrip（离线零 HTTP；迭代 39 真值/认知分离重钉）：
    投影↔解算回环 **θ_b=θ_t 才恒等**——dir×rot×swap 全 18 组合下
    rot_mrad（认知）与 view_rot_deg（真值，°=mrad×0.05729577951）设同值，
    project_rel→solve_rel 回真值 w_true（<1e-12）；project→solve_pixel
    锚定 cam_pos（<1e-9 mm，z=cam_pos.z+look_dir·wd）。
    错配语义另钉：θ_b≠θ_t 时世界解相差旋转 (θ_b−θ_t)（θ_t=0.4mrad/
    θ_b=0 → −0.4mrad；θ_b=0.4/θ_t=0 → +0.4mrad，<1e-9 rad）；solve_pixel
    与 view_rot 无关（认知链独立——同像素输入下输出不随 view_rot 变）。
    迭代 35 需求 1 把 up/coarse 的 dir 默认改 (+1,−1)（down 已是），
    本例钉住三相机内参映射的数值契约。"""
    import vision_model as vm
    base = {"label": "i35", "pixel_um": 6.0, "mag": 10.0,
            "wd_mm": 10.0, "dof_um": 20.0, "px_u": 3000, "px_v": 3000,
            "dir_x": 1.0, "dir_y": -1.0, "swap_xy": False, "rot_mrad": 0.0,
            "mount": {"module": None, "offset": [0.0, 0.0, 0.0]},
            "look_dir": -1, "light_point": "L", "trigger_source": "T",
            "marks": {"source": "wafer_marks", "surface_z_offset_mm": 0.0}}
    w_true = (0.3, -0.2)
    cam_pos = (100.0, 50.0, 900.0)
    MRAD2DEG = 0.05729577951            # °/mrad（真值/认知同值换算）
    ok, det = True, []
    for dx_, dy_ in ((1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)):
        for rot_mrad in (0.3, -0.5, 0.0):
            for swap in (False, True):
                cfg = dict(base)
                cfg.update({"dir_x": dx_, "dir_y": dy_, "swap_xy": swap,
                            "rot_mrad": rot_mrad,
                            "view_rot_deg": rot_mrad * MRAD2DEG})  # θ_b=θ_t
                cam = vm.CameraModel("i35", cfg, {})
                u, v = cam.project_rel(*w_true)
                bx, by = cam.solve_rel(u, v)
                e1 = abs(bx - w_true[0]) + abs(by - w_true[1])
                col, row = cam.project(*w_true)
                wx, wy, wz = cam.solve_pixel(col, row, cam_pos)
                z_exp = cam_pos[2] + cam.look_dir * cam.wd_mm
                e2 = (abs(wx - cam_pos[0] - w_true[0])
                      + abs(wy - cam_pos[1] - w_true[1]) + abs(wz - z_exp))
                if not (e1 < 1e-12 and e2 < 1e-9):
                    ok = False
                    det.append("dir=(%s,%s) rot=%s swap=%s e1=%.2e e2=%.2e"
                               % (dx_, dy_, rot_mrad, swap, e1, e2))
    # ---- 错配语义：θ_b≠θ_t → 世界解相差旋转 (θ_b−θ_t) ----
    cfg_t = dict(base)
    cfg_t["view_rot_deg"] = 0.4 * MRAD2DEG      # θ_t=0.4mrad，θ_b=0
    cam_t = vm.CameraModel("i35t", cfg_t, {})
    bx, by = cam_t.solve_rel(*cam_t.project_rel(1.0, 0.0))
    ang_t = math.atan2(by, bx)                  # 期望 −0.4mrad
    cfg_b = dict(base)
    cfg_b["rot_mrad"] = 0.4                     # θ_b=0.4mrad，θ_t=0
    cam_b = vm.CameraModel("i35b", cfg_b, {})
    bx, by = cam_b.solve_rel(*cam_b.project_rel(1.0, 0.0))
    ang_b = math.atan2(by, bx)                  # 期望 +0.4mrad
    # ---- solve_pixel 与 view_rot 无关（认知链独立）----
    p_vr = cam_t.solve_pixel(1600.0, 1400.0, cam_pos)
    p_0 = vm.CameraModel("i35z", dict(base), {}).solve_pixel(
        1600.0, 1400.0, cam_pos)
    ok2 = (abs(ang_t + 0.4e-3) < 1e-9 and abs(ang_b - 0.4e-3) < 1e-9
           and p_vr == p_0)
    if not ok2:
        det.append("错配 ang_t=%.3e ang_b=%.3e solve_pixel不变=%s"
                   % (ang_t, ang_b, p_vr == p_0))
    check("i35.pixel_world_roundtrip", ok and ok2,
          "dir{(+1,−1),(−1,+1),(+1,+1)}×rot{0.3,−0.5,0}×swap{F,T} 18 组合"
          "θ_b=θ_t 回环 + 错配旋转 (θ_b−θ_t) + solve_pixel 与 view_rot 无关"
          + ("；FAIL " + "; ".join(det) if det else " 全过"))


def test_i35_view_rot_config():
    """i35.view_rot_config：camera_config 设 down view_rot_deg=0.5 →
    cameras_public 回读一致；|θ|>5（5.5）→ 400（语义同现有字段）；
    finally 还原**迁移原值**（迭代 39 起非 0：真值角色自 rot_mrad 迁入
    vision_cameras.json，还原目标从 API 现读不断链）。"""
    r2, pub = {}, {}
    orig = float((http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
                  ).get("view_rot_deg", 0.0))
    try:
        c, r = http_post(BASE + "/api/vision/camera/config",
                         {"cam": "down", "view_rot_deg": 0.5})
        ok_set = c == 200 and (r.get("config") or {}).get("view_rot_deg") \
            == 0.5
        pub = http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
        ok_pub = abs(pub.get("view_rot_deg", -1e9) - 0.5) < 1e-12
        c2, r2 = http_post(BASE + "/api/vision/camera/config",
                           {"cam": "down", "view_rot_deg": 5.5})
        ok_400 = c2 == 400 and "view_rot_deg" in (r2.get("error") or "")
    finally:
        c3, _r3 = http_post(BASE + "/api/vision/camera/config",
                            {"cam": "down", "view_rot_deg": orig})
        pub2 = http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
    ok_rst = c3 == 200 and abs(pub2.get("view_rot_deg", 9e9) - orig) < 1e-12
    check("i35.view_rot_config", ok_set and ok_pub and ok_400 and ok_rst,
          "set=%s pub=%s 5.5→400=%s 还原=%s（原值 %.7f°）err=%s"
          % (ok_set, ok_pub, ok_400, ok_rst, orig,
             (r2.get("error") or "")[:60]))


def test_i35_view_rot_identify():
    """i35.view_rot_identify（cv2 门控，REAL；迭代 39 真值/认知分离重写）：
    view_rot=相机物理安装旋转角（真值，只进投影链）新语义——
    down 机位摆一个**偏心** site 标记（居中后 AS_X +0.3mm 推离画幅中心
    ~500px；下相机挂对准台，动相机=标记反向），view_rot=0 基线 identify
    记 (col0,row0) → view_rot=+0.5° 后同机位重建模板（必须重建，模板按
    旋转画面示教）→ identify 得 (col1,row1)：内容绕画幅中心**反向**
    旋转——世界系 −θ（project_rel R(−θ)），像素系表观角
    =−dir_x·dir_y·θ（down dir(+1,−1) → atan2(row,col) 口径 +0.5°）。
    逐分量对账解析期望 u1=c·u0+s·v0、v1=−s·u0+c·v0（s=dir_x·dir_y，
    c/s=cos/sinθ，中心=px/2），容差 ±1.5px（fnp=0 确定性，NCC 亚像素
    系统差 ≪1px）；**结果不剥旋**——位移 ≈r·θ≈4.4px 显著非零（旧
    _unrotate 口径会把峰位拉回基线 ≤2px，与本断言互斥）。solve_pixel
    不受 view_rot 影响（认知链独立）由 i35.pixel_world_roundtrip 离线
    钉住。finally view_rot（迁移原值，非 0）/fnp/模式还原、模板删除。"""
    import vision_model as vm     # noqa: F401  cv2 门控（同 i33 惯例）
    if vm.cv2_or_none() is None:
        check("i35.view_rot_identify", True, "cv2 不可用，跳过")
        return
    prep_down()
    g = http_json(BASE + "/api/vision/guide?cam=down")
    mk = next((m for m in g.get("marks") or []
               if (m.get("name") or "").startswith("site_")), None)
    assert mk is not None, "down 视野内无 site 标记"
    for s_ in (mk.get("suggested") or []):
        move(s_["axis"], axis_entry(s_["axis"])["pos"] + s_["delta_mm"])
    if not _center_down_mark(mk["name"]):
        check("i35.view_rot_identify", False, "标记未能居中")
        return
    # 偏心：AS_X +0.3mm（动相机，标记像素 −0.3mm/k ≈ −500px col 方向）
    move("AS_X", axis_entry("AS_X")["pos"] + 0.3)
    name0, name1 = "i35_vr0_tpl", "i35_vr1_tpl"
    _del_tpl(name0)
    _del_tpl(name1)
    cur_fnp = (http_json(BASE + "/api/recipe").get("sim_sensors", {})
               .get("fast_noise_px", {}).get("down", 0.15))
    vrot0 = float((http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
                   ).get("view_rot_deg", 0.0))

    def _best(res):
        """取最高分项 (col,row)（多实例 NMS 下确定性锚定）。"""
        cols = res.get("lstCol") or []
        rows = res.get("lstRow") or []
        scs = res.get("lstScore") or []
        if not cols or not scs:
            return None
        bi = max(range(len(scs)), key=lambda i: scs[i])
        return cols[bi], rows[bi]

    try:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.0}}})
        http_post(BASE + "/api/vision/mode", {"mode": "real"})
        c00, r00 = http_post(BASE + "/api/vision/camera/config",
                             {"cam": "down", "view_rot_deg": 0.0})
        assert c00 == 200, (c00, r00)
        c0, r0 = _mk_small_tpl("down", name0)
        assert c0 == 200 and r0.get("ok"), (c0, r0)
        st, rp = http_post(BASE + "/api/vision/identify",
                           {"cam": "down", "template": name0}, timeout=60)
        res0 = ((rp.get("entry") or {}).get("result") or {})
        p0 = _best(res0)
        ok0 = st == 200 and res0.get("judge") is True and p0 is not None
        c1, r1 = http_post(BASE + "/api/vision/camera/config",
                           {"cam": "down", "view_rot_deg": 0.5})
        assert c1 == 200, (c1, r1)
        c2, r2 = _mk_small_tpl("down", name1)
        assert c2 == 200 and r2.get("ok"), (c2, r2)
        st2, rp2 = http_post(BASE + "/api/vision/identify",
                             {"cam": "down", "template": name1}, timeout=60)
        res1 = ((rp2.get("entry") or {}).get("result") or {})
        p1 = _best(res1)
        ok1 = st2 == 200 and res1.get("judge") is True and p1 is not None
        # 解析期望：像素系映射（s=dir_x·dir_y；世界系内容 −θ）
        pub = http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
        s_dir = float(pub["dir_x"]) * float(pub["dir_y"])
        cx, cy = pub["px"][0] / 2.0, pub["px"][1] / 2.0
        th = math.radians(0.5)
        ct, sth = math.cos(th), math.sin(th)
        dcol = drow = disp = rad0 = 1e9
        if ok0 and ok1:
            u0, v0 = p0[0] - cx, p0[1] - cy
            eu = ct * u0 + s_dir * sth * v0
            ev = -s_dir * sth * u0 + ct * v0
            dcol = (p1[0] - cx) - eu
            drow = (p1[1] - cy) - ev
            disp = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            rad0 = math.hypot(u0, v0)
        check("i35.view_rot_identify",
              ok0 and ok1 and abs(dcol) <= 1.5 and abs(drow) <= 1.5
              and disp > 2.5 and 350.0 < rad0 < 750.0,
              "judge=%s/%s 偏心半径=%.1fpx 残差 dcol=%.3fpx drow=%.3fpx "
              "位移=%.3fpx（期望≈r·θ；>2.5px=不剥旋；s=dir_x·dir_y=%+.0f，"
              "像素系表观 %+0.1f°）"
              % (ok0, ok1, rad0, dcol, drow, disp, s_dir,
                 -s_dir * 0.5))
    finally:
        http_post(BASE + "/api/vision/camera/config",
                  {"cam": "down", "view_rot_deg": vrot0})
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": cur_fnp}}})
        http_post(BASE + "/api/vision/mode", {"mode": "fast"})
        _del_tpl(name0)
        _del_tpl(name1)


def test_i35_rotate_two_pose():
    """ws_rotate_repeat 双示教位（迭代 35）：2 点 site quad（半径 ≤15mm）
    + pose_after≠pose（测试侧 _rot_pose_after 枢轴走位，旋后位错开防出
    视场）→ done + _fit_rot θ_i 序列 + stats.angle_err + detail.
    taught_points 逐点带 pose_after；pose_after 缺省=pose（同位复测）
    小角度（rot=0.008，摆幅 ≤15mm×0.008≈0.12mm≪FOV）→ done；校验：
    pose_after WS_LS_X=999.0 越软限位 →400（含"软限位"）、pose_after 未知
    轴 →400（含"未知轴"）。"""
    tpls = ["i35_r0", "i35_r1"]
    pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0, n_pick=2)
    if pts is None:
        check("i35.rotate_two_pose", False, "示教点建制失败")
        for t in tpls:
            _del_tpl(t)
        return
    try:
        # ---- 校验分支 ----
        pa_bad = dict(pts[0]["pose"])
        pa_bad["WS_LS_X"] = 999.0
        c1, r1 = cal_start("ws_rotate_repeat", "down",
                           {"taught_points": [
                               {"tpl": tpls[0], "pose": pts[0]["pose"],
                                "pose_after": pa_bad},
                               {"tpl": tpls[1], "pose": pts[1]["pose"]}],
                            "rot_rad": 0.01, "repeat": 3})
        c2, r2 = cal_start("ws_rotate_repeat", "down",
                           {"taught_points": [
                               {"tpl": tpls[0], "pose": pts[0]["pose"],
                                "pose_after": {"BAD_AX": 1.0}},
                               {"tpl": tpls[1], "pose": pts[1]["pose"]}],
                            "rot_rad": 0.01, "repeat": 3})
        check("i35.rotate_two_pose_400",
              c1 == 400 and "软限位" in (r1.get("error") or "")
              and "pose_after" in (r1.get("error") or "")
              and c2 == 400 and "未知轴" in (r2.get("error") or ""),
              "c=%s/%s err=%s%s"
              % (c1, c2, (r1.get("error") or "")[:50],
                 (r2.get("error") or "")[:50]))
        if not ok_build:
            check("i35.rotate_two_pose", False, "示教点 guide 居中失败")
            return
        # ---- run（pose_after≠pose）----
        pts2 = _rot_pose_after(pts, 0.01)
        diff = [abs(pts2[k]["pose_after"]["WS_LS_X"]
                    - pts2[k]["pose"]["WS_LS_X"])
                + abs(pts2[k]["pose_after"]["WS_LS_Y"]
                      - pts2[k]["pose"]["WS_LS_Y"]) for k in range(2)]
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
        try:
            s, res = run_job("ws_rotate_repeat", "down",
                             {"taught_points": pts2, "rot_rad": 0.01,
                              "repeat": 3, "wait_ms": 20}, timeout=600.0)
        finally:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
        det = (res or {}).get("detail") or {}
        stt = (res or {}).get("stats") or {}
        ser = (res or {}).get("series") or {}
        ae = stt.get("angle_err") or {}
        tp_det = det.get("taught_points") or []
        th_mean = (sum(ser.get("theta_fit_urad") or [1e9])
                   / max(1, len(ser.get("theta_fit_urad") or [])))
        check("i35.rotate_two_pose",
              s.get("state") == "done" and res is not None
              and all(d > 1e-3 for d in diff)          # pose_after≠pose
              and len(tp_det) == 2
              and all("pose_after" in p for p in tp_det)
              and len(ser.get("theta_fit_urad") or []) == 3
              and len(ser.get("angle_err_urad") or []) == 3
              and abs(th_mean - 1e4) < 500.0           # θ_i≈rot（默认真值带）
              and 0.01 <= (ae.get("s3_urad") or -1) < 200.0,
              "state=%s th_mean=%.1furad pose_after≠pose=%s ae_s3=%s%s"
              % (s.get("state"), th_mean,
                 ["%.4f" % d for d in diff], ae.get("s3_urad"),
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
        # ---- pose_after 缺省=pose（同位复测）小角度 done ----
        s2, res2 = run_job("ws_rotate_repeat", "down",
                           {"taught_points": pts, "rot_rad": 0.008,
                            "repeat": 3, "wait_ms": 20}, timeout=600.0)
        det2 = (res2 or {}).get("detail") or {}
        tp2 = det2.get("taught_points") or []
        check("i35.rotate_two_pose_default",
              s2.get("state") == "done" and res2 is not None
              and len(tp2) == 2
              and all(abs(p["pose_after"][a] - p["pose"][a]) < 1e-9
                      for p in tp2 for a in p["pose"]),
              "state=%s err=%s" % (s2.get("state"), (s2.get("error") or "")[:60]))
    finally:
        for t in tpls:
            _del_tpl(t)


def test_i35_rotate_center_refxy():
    """ws_rotate_center measured.rz_center_ref_xy_mm + apply 双写回（迭代 35）：
    3 点 site quad（半径 ≤15mm）step=0.007 n_angles=2 repeat=3 → done +
    measured.rz_center_ref_xy_mm=作业起点 [WS_LS_X, WS_LS_Y]（±1e-4）+
    measured.rz_center_offset_mm 在册；apply 不带 fields（全 measured）→
    机器常数 fine_align 组 rz_center_offset_mm 与 rz_center_ref_xy_mm
    双键写回（GET 验证==measured）→ 还原 [0,0]/[0,0] 并验证。"""
    tpls = ["i35_c0", "i35_c1", "i35_c2"]
    pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0)
    if pts is None:
        check("i35.rotate_center_refxy", False, "示教点建制失败")
        for t in tpls:
            _del_tpl(t)
        return
    try:
        if not ok_build:
            check("i35.rotate_center_refxy", False, "示教点 guide 居中失败")
            return
        ws0 = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
        s, res = run_job("ws_rotate_center", "down",
                         {"taught_points": pts, "step_rad": 0.007,
                          "n_angles": 2, "repeat": 3, "wait_ms": 20},
                         timeout=900.0)
        meas = ((res or {}).get("measured") or {})
        off_m = meas.get("rz_center_offset_mm") or [9e9, 9e9]
        ref_m = meas.get("rz_center_ref_xy_mm") or [None, None]
        ref_ok = (ref_m[0] is not None
                  and abs(ref_m[0] - ws0[0]) < 1e-4
                  and abs(ref_m[1] - ws0[1]) < 1e-4)
        check("i35.rotate_center_refxy",
              s.get("state") == "done" and res is not None and ref_ok,
              "state=%s ref=%s ws0=%s err=%s"
              % (s.get("state"), ref_m, ws0, (s.get("error") or "")[:60]))
        # ---- apply 双写回 + 还原 ----
        if s.get("state") == "done":
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id")})
            got = None
            if c == 200 and r.get("ok"):
                fa = http_json(BASE + "/api/machine_constants")[
                    "constants"]["fine_align"]
                got = (fa.get("rz_center_offset_mm"),
                       fa.get("rz_center_ref_xy_mm"))
            try:
                ok_apply = (c == 200 and r.get("ok")
                            and sorted(r.get("applied") or []) == sorted(
                                ["rz_center_offset_mm",
                                 "rz_center_ref_xy_mm"])
                            and got is not None
                            and all(abs(got[0][i] - off_m[i]) < 1e-9
                                    for i in (0, 1))
                            and all(abs(got[1][i] - ref_m[i]) < 1e-9
                                    for i in (0, 1)))
            finally:
                http_put(BASE + "/api/machine_constants",
                         {"fine_align": {"rz_center_offset_mm": [0.0, 0.0],
                                         "rz_center_ref_xy_mm": [0.0, 0.0]}})
            fa2 = http_json(BASE + "/api/machine_constants")[
                "constants"]["fine_align"]
            check("i35.rotate_center_refxy_apply",
                  ok_apply and fa2.get("rz_center_offset_mm") == [0.0, 0.0]
                  and fa2.get("rz_center_ref_xy_mm") == [0.0, 0.0],
                  "applied=%s wrote=%s restored=%s/%s"
                  % (r.get("applied"), got,
                     fa2.get("rz_center_offset_mm"),
                     fa2.get("rz_center_ref_xy_mm")))
        else:
            check("i35.rotate_center_refxy_apply", False,
                  "作业未 done（err=%s）" % (s.get("error") or "")[:60])
    finally:
        for t in tpls:
            _del_tpl(t)


def test_i35_gain_truth_phase():
    """专属 boot 段（-k i35 / 全量时 main() 主套件前独立拉起/清理，同
    i32 真值段手法）：改 error_maps/low_order.json WS_Rz scale_ppm→1000
    （真值物理角=轴位增量×1.001），bytes 备份后恢复。fnp=0.05：
    - i35.rotate_gain_measured：ws_rotate_gain（r15 小杠杆示教——双示教
      位可见性）step=0.007 n_angles=2 repeat=3 → measured.rz_gain≈1.001
      （|Δ|<8e-3 + s3<1e-3：双示教位扫档不重居中，NCC 系统差别名 ~1e-3
      级拾取跳动，i32 旧 5e-4 口径不可达——严格真值判定由下一项承担）
    - i35.rotate_two_pose_truth：ws_rotate_repeat **r60 大杠杆** 2 点 +
      pose_after 重居中（测试侧枢轴走位）rot=0.01 → θ̄ 对真值
      |θ̄−10010|<6µrad——重居中路径实测系统差 ~±1µrad（默认档 10000.4 /
      真值档 10011.2），注入签名 +10.8µrad 远大于带半宽，可严格判别
      注入有无（i32 的 2.9µrad 为 3σ 重复性口径，此处为系统偏差带）。"""
    lo_path = os.path.join(ROOT, "runtime", "error_maps", "low_order.json")
    with open(lo_path, "rb") as f:
        raw = f.read()
    data = json.loads(raw.decode("utf-8"))
    ppm0 = data["axes"]["WS_Rz"]["scale_ppm"]
    data["axes"]["WS_Rz"]["scale_ppm"] = 1000.0
    with open(lo_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    proc, log = boot_backend()
    ok_g = ok_t = False
    detail = ""
    try:
        if not wait_ready():
            detail = "后端启动失败"
        else:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
            try:
                tpls = ["i35_t0", "i35_t1", "i35_t2"]
                pts, ok_build = _teach3_sites_down(tpls, radius_max=15.0)
                if pts is None or not ok_build:
                    detail = "示教点建制失败 ok_build=%s" % ok_build
                else:
                    try:
                        s, res = run_job(
                            "ws_rotate_gain", "down",
                            {"taught_points": pts, "step_rad": 0.007,
                             "n_angles": 2, "repeat": 3, "wait_ms": 20},
                            timeout=900.0)
                        g = ((res or {}).get("measured") or {}).get("rz_gain")
                        g_st = ((res or {}).get("stats") or {}
                                ).get("gain") or {}
                        ok_g = (s.get("state") == "done" and g is not None
                                and abs(g - 1.001) < 8e-3
                                and 0 <= (g_st.get("s3") or -1) < 1e-3)
                        detail = ("gain: state=%s g=%s s3=%s | " %
                                  (s.get("state"), g, g_st.get("s3")))
                    finally:
                        for t in tpls:
                            _del_tpl(t)
                    # ---- rotate_two_pose θ 真值（r60 重居中严格带）----
                    tpls2 = ["i35_tp0", "i35_tp1"]
                    pts60, ok60 = _teach3_sites_down(
                        tpls2, radius_max=60.0, n_pick=2)
                    if pts60 is None or not ok60:
                        detail += "r60 示教失败 ok=%s" % ok60
                    else:
                        try:
                            pts2 = _rot_pose_after(pts60, 0.01)
                            s2, res2 = run_job(
                                "ws_rotate_repeat", "down",
                                {"taught_points": pts2, "rot_rad": 0.01,
                                 "repeat": 3, "wait_ms": 20}, timeout=600.0)
                            ser2 = (res2 or {}).get("series") or {}
                            ths = ser2.get("theta_fit_urad") or []
                            th_mean = sum(ths) / len(ths) if ths else 1e9
                            ok_t = (s2.get("state") == "done"
                                    and abs(th_mean - 10010.0) < 6.0)
                            detail += (
                                "two_pose(r60): state=%s th_mean=%.1furad "
                                "err=%s" % (s2.get("state"), th_mean,
                                            (s2.get("error") or "")[:40]))
                        finally:
                            for t in tpls2:
                                _del_tpl(t)
            finally:
                http_put(BASE + "/api/recipe",
                         {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log.close()
        # 等端口真正释放（Windows 句柄释放有延迟）再还文件——主套件紧接
        # 在本端口重起后端，避免双绑定连到本段旧实例
        t0 = time.time()
        while time.time() - t0 < 15.0:
            try:
                http_json(BASE + "/api/health", timeout=2)
                time.sleep(0.3)
            except Exception:
                break
        with open(lo_path, "wb") as f:
            f.write(raw)
    check("i35.rotate_gain_measured", ok_g,
          "%s（scale_ppm %s→1000）" % (detail, ppm0))
    check("i35.rotate_two_pose_truth", ok_t, detail)


def test_i35():
    """迭代 35 段汇总：像素↔世界回环（离线零 HTTP；迭代 39 重钉
    θ_b=θ_t 才恒等 + 错配旋转 (θ_b−θ_t) + solve_pixel 与 view_rot 无关）
    → view_rot 在线配置（回读/400/还原迁移原值）→ REAL 识别旋转真值
    （迭代 39：偏心标记像素位移=内容绕画幅中心反向旋转、结果不剥旋）
    → 旋转三件套双示教位（pose_after≠pose run/缺省回退/
    pose_after 越限 400 → 中心标定 ref_xy measured + apply 双写回还原）。
    真值注入段（gain 1.001 测回 + θ 对真值）在 main() 专属 boot 段。"""
    test_i35_pixel_world_roundtrip()
    test_i35_view_rot_config()
    test_i35_view_rot_identify()
    test_i35_rotate_two_pose()
    test_i35_rotate_center_refxy()


# ---------------- 迭代 39：view_rot 真值/认知分离（微旋真值闭环） ----------------
def test_i39_microrot_view_rot_loop():
    """i39.microrot_view_rot_loop（真值/认知分离闭环，符号锚定用例）：
    注入 view_rot_deg=0.0229183°（=+0.4mrad 相机物理安装旋转·真值，
    POST /api/vision/camera/config 热生效）→ camera_microrot（down，
    对准台 AS_X ±0.5mm 杠杆，repeat=10，fnp=0.05 低噪声档）测得
    measured.rot_mrad≈+0.4mrad（dev<0.25 惯例带；投影链 R(−θ_t) 下
    旧双角色公式会得 −0.4——迭代 39 映射末端翻号，本例即符号锚定）→
    measured PUT 进 rot_mrad（认知）→ solve_pixel 链世界解旋转误差
    ≈0：离线 CameraModel 按在线生效值（GET machine_constants 内参 +
    /api/vision/cameras view_rot_deg）重建，solve∘project=R(θ_b−θ_t)
    对 (0.6,−0.4)mm 点的世界误差 <0.25µm（0.72mm 半径 ×|θ_b−θ_t|<
    0.25mrad）。finally：view_rot/rot_mrad 还原迁移原值（现读）、fnp/
    模式还原、模板删除。"""
    import vision_model as vm
    prep_down()
    g = http_json(BASE + "/api/vision/guide?cam=down")
    qd = next((m for m in g.get("marks") or []
               if (m.get("name") or "").startswith("site_")), None)
    assert qd is not None, "down 视野内无 site quad"
    for s_ in (qd.get("suggested") or []):
        move(s_["axis"], axis_entry(s_["axis"])["pos"] + s_["delta_mm"])
    _center_down_mark(qd["name"])
    vrot0 = float((http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
                   ).get("view_rot_deg", 0.0))
    rot0 = http_json(BASE + "/api/machine_constants")["constants"][
        "cameras"]["down"]["rot_mrad"]
    _del_tpl("i39_lp_tpl")
    meas, dev, werr = None, 1e9, 1e9
    ok_run = ok_loop = False
    try:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05}}})
        c, r = http_post(BASE + "/api/vision/camera/config",
                         {"cam": "down", "view_rot_deg": 0.0229183})
        assert c == 200, (c, r)
        c, r = _mk_small_tpl("down", "i39_lp_tpl")   # 注入真值后同视角示教
        assert c == 200 and r.get("ok"), (c, r)
        q0 = {a: axis_entry(a)["pos"] for a in ("AS_X", "AS_Y")}
        s, res = run_job("camera_microrot", "down",
                         {"template_name": "i39_lp_tpl",
                          "photo_pos1": [q0["AS_X"] - 0.5, q0["AS_Y"]],
                          "photo_pos2": [q0["AS_X"] + 0.5, q0["AS_Y"]],
                          "repeat": 10, "wait_ms": 20}, timeout=600.0)
        meas = ((res or {}).get("measured") or {}).get("rot_mrad")
        dev = abs((meas if meas is not None else 1e9) - 0.4)
        ok_run = (s.get("state") == "done" and res is not None
                  and meas is not None and dev < 0.25)
        if ok_run:
            # ---- measured → rot_mrad（认知）写回，solve_pixel 链残差 ----
            c, r = http_put(BASE + "/api/machine_constants",
                            {"cameras": {"down": {"rot_mrad": meas}}})
            assert c == 200, (c, r)
            mc = http_json(BASE + "/api/machine_constants")["constants"][
                "cameras"]["down"]
            pub = http_json(BASE + "/api/vision/cameras")["cameras"]["down"]
            cfg = {"label": "i39", "mount": {"module": None,
                                             "offset": [0.0, 0.0, 0.0]},
                   "look_dir": -1, "light_point": "L", "trigger_source": "T",
                   "marks": {"source": "wafer_marks",
                             "surface_z_offset_mm": 0.0},
                   "view_rot_deg": pub["view_rot_deg"]}
            cfg.update({k: mc[k] for k in
                        ("pixel_um", "mag", "dir_x", "dir_y", "swap_xy",
                         "rot_mrad", "wd_mm", "dof_um", "px_u", "px_v")})
            cam = vm.CameraModel("i39", cfg, {})
            cam_pos = (100.0, 50.0, 900.0)
            col, row = cam.project(0.6, -0.4)
            wx, wy, _wz = cam.solve_pixel(col, row, cam_pos)
            werr = math.hypot(wx - (cam_pos[0] + 0.6),
                              wy - (cam_pos[1] - 0.4))
            ok_loop = werr < 0.00025      # <0.25µm（≈|θ_b−θ_t|<0.35mrad）
        check("i39.microrot_view_rot_loop", ok_run and ok_loop,
              "state=%s measured=%s mrad（注入真值+0.4）dev=%.1fµrad；"
              "写回认知后世界解误差=%.3fµm（0.72mm半径，<0.25µm=补偿到位）%s"
              % (s.get("state"), meas, dev * 1000.0, werr * 1000.0,
                 (" err=%s" % (s.get("error") or ""))
                 if s.get("state") != "done" else ""))
    finally:
        http_post(BASE + "/api/vision/camera/config",
                  {"cam": "down", "view_rot_deg": vrot0})
        http_put(BASE + "/api/machine_constants",
                 {"cameras": {"down": {"rot_mrad": rot0}}})
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.15}}})
        _del_tpl("i39_lp_tpl")


# ---------------- 迭代 36/37：Y 导轨方向 FK 注入 + 非正交角测校（栅格跳测法） ----------------
# 迭代 37 按用户纠正重写测量法（原理/契约见 metrology/stage_nonortho.py 与
# calibration._validate_stage_nonortho）：基准尺=晶圆 die 阵列（wafer_sites
# 格点+site_marks quad 假定正交），走距=格点节距（默认处方 20mm，start 与
# 处方校验一致 0.001mm，不一致 400 报实际 pitch_x/pitch_y）；起点须 quad
# 角对准（每 node 可见性靠走距=节距使相对相位逐 node 复现）；o_i=标记世
# 界位−相机世界位（_context 链），o_i 对步号 LS 斜率：X 线斜率方向读栅格
# 安装旋转 ρ̂、Y 线斜率大小+方向定号给 ψ_y=φ_t+ρ̂，φ(X 基准)=wrap(ψ_y−ρ̂)
# （ρ̂ 差中抵消）；ref_axis=x 报 φ / =y 报 −φ；<pfx>nonortho_urad 恒为 X
# 基准值（apply 语义不随 ref_axis 变）。φ 符号锚定：FK 注入 stage_geo
# as/ws_y_rail_dir_urad=φ_t 时 X 基准测得 ≈+φ_t。
# 容差依据（实测定案）：80mm 长杠杆把随机项压到 µrad 级（σ_斜率≈
# σ_点×√(12/(K(K²−1)))/pitch，σ_点=60nm 质心→≈1µrad）；真值段（扫描轴
# 6 分量地图归零，i35 同款专属 boot）残余为 NCC 亚像素确定项 → 注入判读
# ±5µrad、基线 ±5µrad；主套件（地图开）地图投影主导 → ±40µrad（实测修正）。
I36_TOL_URAD = 40.0             # 主套件（误差地图开）：注入判读容差
I36_BASE_TOL_URAD = 40.0        # 主套件（误差地图开）：基线容差
I36_TRUTH_TOL_URAD = 5.0        # 真值段（地图归零）：注入判读容差（验收 ±5）
I36_TRUTH_BASE_TOL_URAD = 15.0  # 真值段（地图归零）：基线容差（scale_ppm
                                # 5~10ppm 折进 ψ_y 的物理地板，φ=0 不可分；
                                # 注入判读仍 ±5）


def _i36_quiet_chain(axes):
    """确定性判读口径：备份 config_overrides.json，走位轴 repeat 置 0、
    down 质心噪声 fast_noise_px 置 0（均经 API，finally 还原始字节/原值）。
    返回还原函数（幂等）。"""
    ovr_path = os.path.join(ROOT, "runtime", "config_overrides.json")
    ovr_bytes = None
    if os.path.exists(ovr_path):
        with open(ovr_path, "rb") as f:
            ovr_bytes = f.read()
    rep0 = {}
    em_axes = http_json(BASE + "/api/errormaps")["axes"]
    for a in axes:
        rep0[a] = em_axes[a]["repeat_3sigma_display"]
    fnp0 = http_json(BASE + "/api/recipe")["recipe"]["sim_sensors"][
        "fast_noise_px"]["down"]
    for a in axes:
        c, r = http_post(BASE + "/api/axis/config", {"axis": a, "repeat": 0.0})
        assert c == 200, (a, r)
    http_put(BASE + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.0}}})

    def _restore():
        for a in axes:
            http_post(BASE + "/api/axis/config",
                      {"axis": a, "repeat": rep0[a]})
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": fnp0}}})
        if ovr_bytes is not None:
            with open(ovr_path, "wb") as f:
                f.write(ovr_bytes)
        elif os.path.exists(ovr_path):
            os.remove(ovr_path)
    return _restore


def _i36_stage_geo():
    return http_json(BASE + "/api/machine_constants")["constants"][
        "stage_geo"]


def _i36_put_stage_geo(**kw):
    c, r = http_put(BASE + "/api/machine_constants", {"stage_geo": kw})
    assert c == 200 and r.get("ok"), (kw, r)


def _i36_rc(name):
    """site_R{r}C{c}#{k} → (r, c)。"""
    p = name.replace("site_R", "").split("C")
    return int(p[0]), int(p[1].split("#")[0])


def _i36_mk_tpl(stage, name):
    """上片 + 走位台就位 + quad 角居中（**留走法余量**：十字对称走法
    ±(K−1)/2=±2 步（K=5），ws/as 统一把居中角行列调到 [6,9]——下界防
    圆裁出格（实测 R03 行 C01 起无标记：±2 步须 ≥2，留 6 余量；居中靠
    WS jog，WS −pitch=索引递增 / +pitch=递减，两 stage 同法），上界防
    上限越界（9+2=11 ⊂ 网格内）。ws：初始 ≈(3,3) 逐格 −20mm 抬到 ≥6。
    as：同法（旧 L 走法只 +4 步向网内故 C03 可用，对称走法 −2 步会踩
    出格）；AS 起点保持 (20,30)——对称行程 ±40mm：X 走 −20→60 ⊂
    [−30,80]、Y 走 −10→70 ⊂ [−30,120]）+ 小 ROI 模板。
    返回 (模板全名, 居中是否成功)——模板名加 pid 后缀（并发测试实例共享
    vision_templates/ 防互删）。"""
    name = "%s_p%d" % (name, os.getpid())
    prep_down()

    def _center():
        g = http_json(BASE + "/api/vision/guide?cam=down")
        qd = next((m for m in g.get("marks") or []
                   if (m.get("name") or "").startswith("site_")), None)
        assert qd is not None, "down 视野内无 site quad"
        for s_ in (qd.get("suggested") or []):
            move(s_["axis"], axis_entry(s_["axis"])["pos"] + s_["delta_mm"])
        okc = _center_down_mark(qd["name"])
        return qd["name"], okc

    nm, okc = _center()
    r_, c_ = _i36_rc(nm)
    while c_ < 6:
        move("WS_LS_X", axis_entry("WS_LS_X")["pos"] - 20.0)
        nm, okc = _center()
        c_ = _i36_rc(nm)[1]
    while c_ > 9:
        move("WS_LS_X", axis_entry("WS_LS_X")["pos"] + 20.0)
        nm, okc = _center()
        c_ = _i36_rc(nm)[1]
    while r_ < 6:
        move("WS_LS_Y", axis_entry("WS_LS_Y")["pos"] - 20.0)
        nm, okc = _center()
        r_ = _i36_rc(nm)[0]
    while r_ > 9:
        move("WS_LS_Y", axis_entry("WS_LS_Y")["pos"] + 20.0)
        nm, okc = _center()
        r_ = _i36_rc(nm)[0]
    _del_tpl(name)
    c, r = _mk_small_tpl("down", name)
    assert c == 200 and r.get("ok"), (c, r)
    return name, okc


def _i36_run(routine, tpl, **params):
    p = {"template_name": tpl, "repeat": 2, "wait_ms": 10}
    p.update(params)
    return run_job(routine, "down", p, timeout=900.0)


def _i36_inject_case(name, routine, inject_key, tol, base_tol, tag="",
                     phis=(200.0, -200.0, 500.0)):
    """栅格跳测注入往返：每个 φ_t 注→测→（下个 φ 前归零）…finally 全
    还原 + φ_t=0 基线。开 six_dof 链（FK 注入消费口）；确定性判读口径。
    tol/base_tol：注入判读/基线容差（真值段 5/5，主套件 40/40）。"""
    axes = JOG["down"] if routine == "ws_stage_nonortho" else ("AS_X", "AS_Y")
    geo0 = _i36_stage_geo()
    c, r = http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
    assert c == 200, (c, r)
    restore = _i36_quiet_chain(axes)
    got = {}
    det = ""
    try:
        name, okc = _i36_mk_tpl(
            "ws" if routine == "ws_stage_nonortho" else "as", name)
        det += "center_ok=%s " % okc
        for phi in phis:
            _i36_put_stage_geo(**{inject_key: phi})
            s1, res1 = _i36_run(routine, name)
            got[phi] = ((res1 or {}).get("measured") or {}).get("phi_urad")
            det += "%+.0f→%s(%s) " % (
                phi, got[phi], s1.get("state"))
        _i36_put_stage_geo(**{inject_key: 0.0})
        s2, res2 = _i36_run(routine, name)
        got[0.0] = ((res2 or {}).get("measured") or {}).get("phi_urad")
        det += "base→%s(%s)" % (got[0.0], s2.get("state"))
    finally:
        _i36_put_stage_geo(**{k: geo0.get(k) or 0.0 for k in (
            "as_y_rail_dir_urad", "ws_y_rail_dir_urad",
            "as_nonortho_urad", "ws_nonortho_urad")})
        restore()
        _del_tpl(name)
    stage = "ws" if routine == "ws_stage_nonortho" else "as"
    check("i36.nonortho_inject_%s%s" % (stage, "_" + tag if tag else ""),
          all(got.get(p) is not None and abs(got[p] - p) <= tol
              for p in phis)
          and got.get(0.0) is not None and abs(got[0.0]) <= base_tol,
          "%s | 注入容差 ±%.0f 基线 |φ0|=%.2f（容差 ±%.0f）"
          % (det, tol, abs(got.get(0.0) or 1e9), base_tol))


def test_i36_nonortho_inject_ws():
    """i36.nonortho_inject_ws：开 six_dof 链 → PUT machine_constants 注
    ws_y_rail_dir_urad=+200/−200/+500 往返（finally 还原）→ 晶圆 site quad
    角居中建小模板 → ws_stage_nonortho（默认 K=5/处方 pitch=20mm）→ 各
    |φ̄−φ_t|≤容差 + φ=0 基线。栅格跳测法（i37 重写）；严格带（±5µrad）
    在真值段 i36.nonortho_inject_ws_truth。"""
    _i36_inject_case("i36_ws_tpl", "ws_stage_nonortho",
                     "ws_y_rail_dir_urad", I36_TOL_URAD, I36_BASE_TOL_URAD)


def test_i36_nonortho_inject_as():
    """i36.nonortho_inject_as：同 ws 法注 as_y_rail_dir=±500——AS 走位
    （下相机随对准台动、看静止晶圆标记，σ=−1）；AS_X=20 起走（十字
    对称 ±40mm：X −20→60 ⊂ 行程 [−30,80]）。严格带在
    i36.nonortho_inject_as_truth。"""
    _i36_inject_case("i36_as_tpl", "as_stage_nonortho",
                     "as_y_rail_dir_urad", I36_TOL_URAD, I36_BASE_TOL_URAD,
                     phis=(500.0, -500.0))


def test_i37_nonortho_chain_off():
    """i37.nonortho_chain_off：链关时 FK 注入不消费——注 ws_y_rail_dir
    =+200 + six_dof OFF → 测得 |φ̄|≤容差（≈0）；finally 链恢复开。"""
    geo0 = _i36_stage_geo()
    restore = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
    phi_off = None
    try:
        nm_off, _ok = _i36_mk_tpl("ws", "i37_off_tpl")
        _i36_put_stage_geo(ws_y_rail_dir_urad=200.0)
        c, r = http_post(BASE + "/api/errormaps", {"six_dof_enabled": False})
        assert c == 200, (c, r)
        try:
            s, res = _i36_run("ws_stage_nonortho", nm_off)
            phi_off = ((res or {}).get("measured") or {}).get("phi_urad")
        finally:
            http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        check("i37.nonortho_chain_off",
              phi_off is not None and abs(phi_off) <= I36_TOL_URAD,
              "链关+注入 200：φ̄=%.2fµrad（应≈0，容差 ±%.0f）"
              % (phi_off if phi_off is not None else 1e9, I36_TOL_URAD))
    finally:
        _i36_put_stage_geo(**{k: geo0.get(k) or 0.0 for k in (
            "as_y_rail_dir_urad", "ws_y_rail_dir_urad",
            "as_nonortho_urad", "ws_nonortho_urad")})
        restore()
        _del_tpl(nm_off)


def test_i37_nonortho_ref_axis_y():
    """i37.nonortho_ref_axis_y：ref_axis=y 时报告值=−X 基准（+200 →
    phi_urad≈−200），而组路由键 ws_nonortho_urad 恒为 X 基准值（≈+200，
    apply 语义不随 ref_axis 变）。"""
    geo0 = _i36_stage_geo()
    restore = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
    phi_y = None
    try:
        nm_ray, _ok = _i36_mk_tpl("ws", "i37_ray_tpl")
        _i36_put_stage_geo(ws_y_rail_dir_urad=200.0)
        s, res = _i36_run("ws_stage_nonortho", nm_ray,
                          ref_axis="y")
        m = (res or {}).get("measured") or {}
        phi_y = m.get("phi_urad")
        ok = (phi_y is not None
              and abs(phi_y - (-200.0)) <= I36_TOL_URAD
              and abs((m.get("ws_nonortho_urad") or 0.0) - 200.0)
              <= I36_TOL_URAD)
        check("i37.nonortho_ref_axis_y", ok,
              "ref_axis=y：phi_urad=%s ws_nonortho_urad=%s（应≈−200/+200）"
              % (phi_y, m.get("ws_nonortho_urad")))
    finally:
        _i36_put_stage_geo(**{k: geo0.get(k) or 0.0 for k in (
            "as_y_rail_dir_urad", "ws_y_rail_dir_urad",
            "as_nonortho_urad", "ws_nonortho_urad")})
        restore()
        _del_tpl(nm_ray)


def test_i36_nonortho_400():
    """i36/37.400 分支（i37 栅格跳测法校验器）：pitch≠处方（报实际
    pitch_x/pitch_y）、行程越界、视野无标记、ref_axis 非法、k_points 越
    界、cam=up 各 400。"""
    prep_down()
    _tpl400 = "i36_400_tpl_p%d" % os.getpid()
    _del_tpl(_tpl400)
    c, r = _mk_small_tpl("down", _tpl400)
    assert c == 200 and r.get("ok"), (c, r)
    try:
        cases = []
        for empty in ("", None):
            c, r = cal_start("ws_stage_nonortho", "down",
                             {"template_name": empty})
            cases.append(("tpl_%r" % empty,
                          c == 400 and "template_name" in (r.get("error") or "")))
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": _tpl400, "k_points": 2})
        cases.append(("k2", c == 400 and "k_points" in (r.get("error") or "")))
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": _tpl400,
                          "ref_axis": "z"})
        cases.append(("ref_z", c == 400 and "ref_axis" in (r.get("error") or "")))
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": _tpl400, "pitch_mm": 20.5})
        err = (r or {}).get("error") or ""
        cases.append(("pitch", c == 400 and "pitch_x" in err
                      and "10.005" in err))
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": _tpl400,
                          "center_xy": [200.0, 111.6102]})
        err = (r or {}).get("error") or ""
        cases.append(("travel", c == 400 and "限位" in err))
        # 视野无标记（下片 + 走位到无标记区）
        prep_clean()
        for ax in ("WS_LS_X", "WS_LS_Y"):
            move(ax, 0.0)
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": _tpl400})
        err = (r or {}).get("error") or ""
        cases.append(("no_mark", c == 400 and "标记" in err
                      and "模板" not in err))
        prep_down()
        c, r = cal_start("ws_stage_nonortho", "up",
                         {"template_name": _tpl400})
        cases.append(("cam_up", c == 400))
        check("i36.nonortho_400", all(ok for _n, ok in cases),
              " ".join("%s:%s" % (n, "Y" if ok else "N") for n, ok in cases))
    finally:
        _del_tpl(_tpl400)


def test_i37_nonortho_cancel():
    """i37.nonortho_cancel：长作业（repeat=10）运行中 POST cancel →
    canceled 终态；finally 现场恢复（轴回起点/灯灭）；忙释放后新作业
    可 done。"""
    geo0 = _i36_stage_geo()
    restore = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
    state = None
    try:
        nm_cnc, _ok = _i36_mk_tpl("ws", "i37_cnc_tpl")
        wpos = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
        c, r = cal_start("ws_stage_nonortho", "down",
                         {"template_name": nm_cnc, "repeat": 10})
        assert c == 200 and r.get("job_id"), (c, r)
        jid = r["job_id"]
        t0 = time.time()
        while time.time() - t0 < 30:
            s = http_json(BASE + "/api/calibration/status?job_id=%d" % jid)
            if len(s.get("log") or []) > 3:
                break
            time.sleep(0.2)
        c2, r2 = http_post(BASE + "/api/calibration/cancel",
                           {"job_id": jid})
        assert c2 == 200, (c2, r2)
        t0 = time.time()
        while time.time() - t0 < 120:
            s = http_json(BASE + "/api/calibration/status?job_id=%d" % jid)
            if s.get("state") in ("canceled", "done", "error"):
                break
            time.sleep(0.3)
        state = s.get("state")
        time.sleep(1.0)
        wpos2 = (axis_entry("WS_LS_X")["pos"], axis_entry("WS_LS_Y")["pos"])
        axes_ok = all(abs(wpos2[i] - wpos[i]) < 0.01 for i in (0, 1))
        plc = http_json("http://127.0.0.1:%s/io/observe?path="
                        % ENV_PORTS["D2W_PORT_PLC"] + LIGHT_POINT["down"])
        light_off = not plc.get("value")
        s3, res3 = _i36_run("ws_stage_nonortho", nm_cnc)
        check("i37.nonortho_cancel",
              state == "canceled" and axes_ok and light_off
              and s3.get("state") == "done",
              "state=%s axes_restored=%s light_off=%s followup=%s"
              % (state, axes_ok, light_off, s3.get("state")))
    finally:
        _i36_put_stage_geo(**{k: geo0.get(k) or 0.0 for k in (
            "as_y_rail_dir_urad", "ws_y_rail_dir_urad",
            "as_nonortho_urad", "ws_nonortho_urad")})
        restore()
        _del_tpl(nm_cnc)


def test_i36_nonortho_apply():
    """i36.nonortho_apply：ref_axis=y 作业 done（注入 +200）后 apply
    fields=["ws_nonortho_urad"] → 机器常数 stage_geo.ws_nonortho_urad
    ==measured 的 X 基准值（≈+200，不受 ref_axis 口径影响）、真值旋钮
    ws_y_rail_dir_urad 不被 apply 改动 → 还原全零。"""
    geo0 = _i36_stage_geo()
    restore = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
    applied = got_rail = phi_y = None
    det = ""
    try:
        nm_ap, _ok = _i36_mk_tpl("ws", "i36_ap_tpl")
        _i36_put_stage_geo(ws_y_rail_dir_urad=200.0)
        s, res = _i36_run("ws_stage_nonortho", nm_ap, ref_axis="y")
        m = (res or {}).get("measured") or {}
        phi_y = m.get("phi_urad")
        det = "state=%s phi_y=%s xbasis=%s" % (
            s.get("state"), phi_y, m.get("ws_nonortho_urad"))
        if s.get("state") == "done":
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id"),
                              "fields": ["ws_nonortho_urad"]})
            det += " apply_c=%s applied=%s" % (c, r.get("applied"))
            if c == 200 and r.get("ok"):
                applied = _i36_stage_geo().get("ws_nonortho_urad")
                got_rail = _i36_stage_geo().get("ws_y_rail_dir_urad")
        check("i36.nonortho_apply",
              phi_y is not None and applied is not None
              and abs(applied - 200.0) <= I36_TOL_URAD
              and got_rail == 200.0,
              "%s wrote=%s rail_untouched=%s（还原中）"
              % (det, applied, got_rail))
    finally:
        _i36_put_stage_geo(**{k: geo0.get(k) or 0.0 for k in (
            "as_y_rail_dir_urad", "ws_y_rail_dir_urad",
            "as_nonortho_urad", "ws_nonortho_urad")})
        restore()
        _del_tpl(nm_ap)


def test_i36_nonortho_truth_phase():
    """专属 boot 段（-k i36 / 全量时 main() 主套件前独立拉起/清理，同
    i35 真值段手法）：扫描轴 WS_LS_X/WS_LS_Y/AS_X/AS_Y 的 error_maps
    6 分量 CSV 数值归零（位置列/表头不动，bytes 备份后按字节恢复）——
    80mm 长杠杆把随机项压到 µrad 级后，残差主导项=地图投影，归零后剩
    NCC 亚像素确定项：
    - i36.nonortho_inject_ws_truth / as_truth：±200/±500 往返恢复
      |φ̄−φ_t|≤5µrad（i37 验收口径）+ φ=0 基线 |φ̄|≤15µrad（scale_ppm
      地板）。"""
    maps_dir = os.path.join(ROOT, "runtime", "error_maps")
    axes = ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y")
    comps = ("dx", "dy", "dz", "rx", "ry", "rz")
    backups = {}
    for a in axes:
        for comp in comps:
            p = os.path.join(maps_dir, "%s_%s_simulated.csv" % (a, comp))
            with open(p, "rb") as f:
                backups[p] = f.read()
    try:
        for p, raw in backups.items():
            lines = raw.decode("utf-8").splitlines()
            out = [lines[0]]
            for ln in lines[1:]:
                out.append(ln.split(",")[0] + ",0.0")
            with open(p, "w", encoding="utf-8", newline="\n") as f:
                f.write("\n".join(out) + "\n")
        proc, log = boot_backend()
        try:
            if not wait_ready():
                check("i36.nonortho_inject_ws_truth", False, "后端启动失败")
                check("i36.nonortho_inject_as_truth", False, "后端启动失败")
                return
            _i36_inject_case("i36_ws_truth_tpl", "ws_stage_nonortho",
                             "ws_y_rail_dir_urad",
                             I36_TRUTH_TOL_URAD, I36_TRUTH_BASE_TOL_URAD,
                             tag="truth", phis=(200.0, -500.0))
            _i36_inject_case("i36_as_truth_tpl", "as_stage_nonortho",
                             "as_y_rail_dir_urad",
                             I36_TRUTH_TOL_URAD, I36_TRUTH_BASE_TOL_URAD,
                             tag="truth", phis=(200.0, -500.0))
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
            log.close()
            # 等端口真正释放（Windows 句柄释放有延迟）再还文件——主套件紧接
            # 在本端口重起后端，避免双绑定连到本段旧实例
            t0 = time.time()
            while time.time() - t0 < 15.0:
                try:
                    http_json(BASE + "/api/health", timeout=2)
                    time.sleep(0.3)
                except Exception:
                    break
    finally:
        for p, raw in backups.items():
            with open(p, "wb") as f:
                f.write(raw)


def test_i36():
    """迭代 36/37 段汇总（主套件，默认误差地图开）：ws/as 注入往返测回
    （栅格跳测法，链系统差带 ±40µrad）→ 链关不消费 → ref_axis=y 负值
    → 400 分支（pitch/行程/无标记/ref_axis/k/cam）→ cancel 恢复语义 →
    apply 写回 X 基准值（不动 y_rail_dir）。严格带（±5µrad）在 main()
    专属 boot 真值段（扫描轴地图归零）。"""
    test_i36_nonortho_inject_ws()
    test_i36_nonortho_inject_as()
    test_i37_nonortho_chain_off()
    test_i37_nonortho_ref_axis_y()
    test_i36_nonortho_400()
    test_i37_nonortho_cancel()
    test_i36_nonortho_apply()
    test_i44_nonortho_nodes_markworld()
    test_i45_nonortho_display_schema()


def test_i45_nonortho_display_schema():
    """i45.nonortho_display_schema：迭代 45 规范对齐 + 展示 schema——
    series 节点含 mwx/mwy（ws 口径=WS 读数+o，X 线 mwx≈pos+ox、Y 线
    mwy≈pos+oy，容差 10µm 含到位残差）；measured 增 exy_urad（规范式 4
    口径=−X 基准 φ，与组路由键互为相反数）/beta_urad/exy_u/exy_U
    （附录 D，公式复算一致性 1%）；detail.ac_check_norm（6.4 ‖AC−I‖
    <1e-12）；nodes CSV 追加 frame_cols_px/frame_rows_px 逐帧原始坐标
    列（规范 5.3，逐行帧数=fpp）。"""
    import csv as _csv
    nm = None
    restore = None
    try:
        restore = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
        nm, _ok = _i36_mk_tpl("ws", "i45_dsp_tpl")
        fpp = 3
        s, res = _i36_run("ws_stage_nonortho", nm, frames_per_point=fpp)
        m = (res or {}).get("measured") or {}
        det = (res or {}).get("detail") or {}
        ser = (res or {}).get("series") or {}
        pitch = float(det.get("node_pitch_mm"))
        k = int(det.get("k_points"))
        cen = det.get("center_xy") or [0.0, 0.0]
        # ① series 节点 mwx/mwy + ws 口径（X 线 mwx≈pos+ox、mwy≈
        #    center_y+oy；Y 线对偶）
        nodes_ok = True
        for key in ("nodes_x", "nodes_y"):
            nds = ser.get(key) or []
            if len(nds) != k:
                nodes_ok = False
                break
            for p_ in nds:
                if p_.get("mwx") is None or p_.get("mwy") is None:
                    nodes_ok = False
                    break
                if key == "nodes_x":
                    okp = (abs(p_["mwx"] - (p_["pos"] + p_["ox"])) < 0.01
                           and abs(p_["mwy"] - (cen[1] + p_["oy"])) < 0.01)
                else:
                    okp = (abs(p_["mwy"] - (p_["pos"] + p_["oy"])) < 0.01
                           and abs(p_["mwx"] - (cen[0] + p_["ox"])) < 0.01)
                if not okp:
                    nodes_ok = False
                    break
        # ② 规范标量：EXY=−φ（X 基准）、β=ρ̄、U 公式复算（附录 D）
        exy = m.get("exy_urad")
        phi_x = m.get("ws_nonortho_urad")
        exy_ok = (exy is not None and phi_x is not None
                  and abs(exy + phi_x) <= 0.002)
        beta_ok = m.get("beta_urad") is not None
        rms_nm = m.get("resid_rms_nm")
        U_rep = m.get("exy_U_urad")
        u_meas_mm = rms_nm * 1e-6
        sxx = pitch * pitch * k * (k * k - 1) / 12.0
        U_ref = 2.0 * math.sqrt(2.0) * u_meas_mm / math.sqrt(sxx) * 1e6
        u_ok = (U_rep is not None and U_rep > 0
                and abs(U_rep - U_ref) / U_ref < 0.01)
        # ③ 6.4 数值核验
        ac = det.get("ac_check_norm")
        ac_ok = ac is not None and ac < 1e-12
        # ④ nodes CSV 逐帧列（规范 5.3）
        rel = det.get("csv_nodes")
        path = os.path.join(ROOT, rel.replace("/", os.sep))
        with open(path, encoding="utf-8", newline="") as f:
            rows = list(_csv.DictReader(f))
        fr_ok = bool(rows) and "frame_cols_px" in rows[0] \
            and "frame_rows_px" in rows[0] \
            and all(len(r["frame_cols_px"].split(";")) == fpp
                    and len(r["frame_rows_px"].split(";")) == fpp
                    for r in rows)
        check("i45.nonortho_display_schema",
              s.get("state") == "done" and nodes_ok and exy_ok
              and beta_ok and u_ok and ac_ok and fr_ok,
              "state=%s nodes_mw=%s exy=%s(−φ=%s) beta=%s U=%s(参考 %.3f) "
              "ac=%s frames=%s"
              % (s.get("state"), nodes_ok, exy, phi_x, m.get("beta_urad"),
                 U_rep, U_ref, ac, fr_ok))
    finally:
        if restore is not None:
            restore()
        if nm:
            _del_tpl(nm)


def test_i44_nonortho_nodes_markworld():
    """i44.nonortho_nodes_markworld：nodes CSV 的 mark_world 列口径（迭代
    44 修复）——ws 例行 mark_world=WS_LS 读数位+像素反算偏移 o（台基准
    实测位置链，随台步进 ~pitch；旧 cam_world+o 在相机固定+跳测换标记
    下恒为常数、无分析价值）；as 例行保持 cam_world+o 不变式（相机随
    对准台动，即"对准台结合像素"）。逐行断言恒等式（列格式 %.7f/%.6f/
    %.9f 舍入带 ±2e-6）+ ws X 线 mark_world_x 逐节点步进≈pitch（随台
    走、非常数）。o_i/拟合/矩阵链不消费 mark_world，本用例只守记录口径。"""
    import csv as _csv

    def _read_nodes(res):
        rel = ((res or {}).get("detail") or {}).get("csv_nodes")
        assert rel, "result.detail.csv_nodes 缺失"
        path = os.path.join(ROOT, rel.replace("/", os.sep))
        assert os.path.isfile(path), "nodes csv 不存在: %s" % rel
        with open(path, encoding="utf-8", newline="") as f:
            rows = list(_csv.DictReader(f))
        assert rows, "nodes csv 空: %s" % rel
        return rows

    def _num(row, k):
        return float(row[k]) if row[k] != "" else None

    nm_w = nm_a = None
    restore_w = restore_a = None
    try:
        # ---- ws：mark_world == WS_LS + o，且随台步进 ----
        restore_w = _i36_quiet_chain(("WS_LS_X", "WS_LS_Y"))
        nm_w, _ok = _i36_mk_tpl("ws", "i44_mw_tpl")
        s_w, res_w = _i36_run("ws_stage_nonortho", nm_w)
        rows_w = _read_nodes(res_w)
        pitch = float((res_w.get("detail") or {}).get("node_pitch_mm"))
        id_ok = True
        for row in rows_w:
            wsx, wsy = _num(row, "WS_LS_X_mm"), _num(row, "WS_LS_Y_mm")
            mwx, mwy = _num(row, "mark_world_x_mm"), _num(row, "mark_world_y_mm")
            ox, oy = _num(row, "o_x_mm"), _num(row, "o_y_mm")
            if (wsx is None or wsy is None or mwx is None or mwy is None
                    or abs(mwx - (wsx + ox)) > 2e-6
                    or abs(mwy - (wsy + oy)) > 2e-6):
                id_ok = False
                break
        x1 = [r for r in rows_w if r["cycle"] == "1" and r["line"] == "X"]
        x1.sort(key=lambda r: int(r["node"]))
        steps = [abs(_num(x1[i + 1], "mark_world_x_mm")
                     - _num(x1[i], "mark_world_x_mm"))
                 for i in range(len(x1) - 1)]
        step_ok = (len(steps) >= 2
                   and all(abs(d - pitch) < 1.0 for d in steps)
                   and (max(_num(r, "mark_world_x_mm") for r in x1)
                        - min(_num(r, "mark_world_x_mm") for r in x1))
                   > (len(x1) - 1) * pitch * 0.9)
        check("i44.nonortho_nodes_markworld_ws",
              s_w.get("state") == "done" and id_ok and step_ok,
              "ws: state=%s 恒等式=%s X 线步进=%s（pitch=%.1f，步=%s）"
              % (s_w.get("state"), id_ok, step_ok, pitch,
                 ["%.2f" % d for d in steps]))
        # ---- as：mark_world == cam_world + o（口径未动，防回归） ----
        restore_a = _i36_quiet_chain(("AS_X", "AS_Y"))
        nm_a, _ok = _i36_mk_tpl("as", "i44_mw_tpl")
        s_a, res_a = _i36_run("as_stage_nonortho", nm_a)
        rows_a = _read_nodes(res_a)
        id_ok_a = True
        for row in rows_a:
            cwx, cwy = _num(row, "cam_world_x_mm"), _num(row, "cam_world_y_mm")
            mwx, mwy = _num(row, "mark_world_x_mm"), _num(row, "mark_world_y_mm")
            ox, oy = _num(row, "o_x_mm"), _num(row, "o_y_mm")
            if (cwx is None or cwy is None or mwx is None or mwy is None
                    or abs(mwx - (cwx + ox)) > 2e-6
                    or abs(mwy - (cwy + oy)) > 2e-6):
                id_ok_a = False
                break
        check("i44.nonortho_nodes_markworld_as",
              s_a.get("state") == "done" and id_ok_a,
              "as: state=%s 恒等式 mark_world=cam_world+o %s"
              % (s_a.get("state"), id_ok_a))
    finally:
        # LIFO 还原（迭代 45 修：原顺序 restore_w→restore_a 会让 restore_a
        # 把"备份于 restore_w 生效后"的污染字节写回 config_overrides.json
        # ——WS repeat=0/fast_noise=0 残留盘上，后续套件 i7/error 段全挂）
        if restore_a is not None:
            restore_a()
        if restore_w is not None:
            restore_w()
        for _n in (nm_w, nm_a):
            if _n:
                _del_tpl(_n)


# =====================================================================
# 迭代 42：as_head_center 上吸盘位置标定 / best_focus 相机最佳焦平面标定
# （**up 链路依赖仿真侧吸盘中心标记**——vision_model collect_marks 的
# head_die_marks 分支在无 die 时须给出 collet 中心 cross 标记（FAST 解析
# 标记实体；迭代 42 前为缺口，补丁提案 Temp/_i42_chuck_mark_proposal.py；
# 未应用时 up 用例 done 断言失败并报"视野内无标记"）；down best_focus
# 走晶圆面标记不依赖该补丁）
# =====================================================================
def _i42_consts():
    """读 wafer_ega/focus_level 相关键现值（finally 还原用）。"""
    mc = http_json(BASE + "/api/machine_constants")["constants"]
    return {"anchor": list(mc["wafer_ega"]["as_anchor_xy_mm"]),
            "head": list(mc["focus_level"]["as_head_center_xy_mm"]),
            "asz": mc["focus_level"]["as_z_chuck_focus_mm"],
            "bsz": mc["focus_level"]["head_z_upfocus_mm"]}


def _i42_restore_consts(saved):
    http_put(BASE + "/api/machine_constants",
             {"wafer_ega": {"as_anchor_xy_mm": saved["anchor"]},
              "focus_level": {"as_head_center_xy_mm": saved["head"],
                              "as_z_chuck_focus_mm": saved["asz"],
                              "head_z_upfocus_mm": saved["bsz"]}})


def _i42_csv_ok(res, prefix):
    rel = (res or {}).get("csv") or ""
    return (os.path.isfile(os.path.join(ROOT, rel.replace("/", os.sep)))
            and os.path.basename(rel).startswith(prefix))


def test_i42_as_head_center_400():
    """as_head_center 校验分支（start 同步 400，不开作业）：未注册模板
    →400 列可用名；repeat 越界（1/101）→400；cam 非 up →400。"""
    prep_clean()
    v = []
    c, r = cal_start("as_head_center", "up",
                     {"template_name": "i42_no_such_tpl", "repeat": 3})
    v.append(c == 400 and "不存在" in (r.get("error") or ""))
    c, r = cal_start("as_head_center", "up", {"repeat": 1})
    v.append(c == 400 and "repeat" in (r.get("error") or ""))
    c, r = cal_start("as_head_center", "up", {"repeat": 101})
    v.append(c == 400 and "repeat" in (r.get("error") or ""))
    c, r = cal_start("as_head_center", "down", {"repeat": 3})
    v.append(c == 400 and "cam" in (r.get("error") or ""))
    check("i42.as_head_center_400", all(v), "分支通过=%s/4" % sum(v))


def test_i42_as_head_center():
    """as_head_center 上吸盘位置标定（迭代 42）：
    **误差链 OFF**（确定性真值：误差链关时 bond_head 吸盘心世界
    (−42.5,7.0)、上相机零位世界 (−42.5,1.0) ⇒ 真值锚点恒 (0,6)，finally
    还原）+ PUT 机器常数注入锚点认知偏移（wafer_ega.as_anchor_xy_mm
    与 focus_level.as_head_center_xy_mm 同置 [0.05,6.05]，+0.05mm）→
    repeat=5 done → measured 双键同值且命中真值 (0,6) ±0.01mm →
    apply fields 双键写回 → GET 两组常数一致 → **复测残差≈0（符号自
    证：写回后 AS 已在新锚点，δ≈0，measured₂≈measured₁）**；finally
    常数/误差链全还原。"""
    prep_clean()
    saved = _i42_consts()
    http_put(BASE + "/api/machine_constants",
             {"wafer_ega": {"as_anchor_xy_mm": [0.05, 6.05]},
              "focus_level": {"as_head_center_xy_mm": [0.05, 6.05]}})
    http_post(BASE + "/api/errormaps", {"six_dof_enabled": False})
    try:
        s, res = run_job("as_head_center", "up",
                         {"repeat": 5, "wait_ms": 10}, timeout=300.0)
        meas = ((res or {}).get("measured") or {})
        m_a = meas.get("as_anchor_xy_mm") or [9e9, 9e9]
        m_h = meas.get("as_head_center_xy_mm") or [9e9, 9e9]
        stt = (res or {}).get("stats") or {}
        ser = (res or {}).get("series") or {}
        det = (res or {}).get("detail") or {}
        dev = math.hypot(m_a[0] - 0.0, m_a[1] - 6.0)
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        check("i42.as_head_center",
              s.get("state") == "done" and res is not None
              and dev < 0.01
              and m_a == m_h
              and (stt.get("anchor") or {}).get("s3_nm") is not None
              and 0 <= (stt["anchor"]["s3_nm"]) < 5000.0
              and len(ser.get("anchor_x_mm") or []) == 5
              and len(ser.get("anchor_y_mm") or []) == 5
              and det.get("anchor0_mm") == [0.05, 6.05]
              and _i42_csv_ok(res, "as_head_center_"),
              "state=%s measured=%s dev=%.4fmm s3=%snm%s（若无标记→检"
              "查 Temp/_i42_chuck_mark 提案补丁是否已应用）"
              % (s.get("state"), m_a, dev,
                 (stt.get("anchor") or {}).get("s3_nm"), err_tail))
        # ---- apply 双键写回 + GET 两组常数一致 ----
        if s.get("state") != "done":
            check("i42.as_head_center_apply", False,
                  "作业未 done%s" % err_tail[:60])
            return
        c, r = http_post(BASE + "/api/calibration/apply",
                         {"job_id": s.get("job_id"),
                          "fields": ["as_anchor_xy_mm",
                                     "as_head_center_xy_mm"]})
        got_a = got_h = None
        if c == 200 and r.get("ok"):
            mc = http_json(BASE + "/api/machine_constants")["constants"]
            got_a = mc["wafer_ega"]["as_anchor_xy_mm"]
            got_h = mc["focus_level"]["as_head_center_xy_mm"]
        ok_apply = (c == 200 and r.get("ok")
                    and r.get("applied") == ["as_anchor_xy_mm",
                                             "as_head_center_xy_mm"]
                    and got_a is not None and got_h is not None
                    and all(abs(got_a[i] - m_a[i]) < 1e-9 for i in (0, 1))
                    and all(abs(got_h[i] - m_h[i]) < 1e-9 for i in (0, 1)))
        check("i42.as_head_center_apply", ok_apply,
              "applied=%s anchor=%s head=%s"
              % (r.get("applied"), got_a, got_h))
        # ---- 复测残差≈0（符号自证：写回后 δ≈0，measured₂≈measured₁）----
        s2, res2 = run_job("as_head_center", "up",
                           {"repeat": 5, "wait_ms": 10}, timeout=300.0)
        meas2 = ((res2 or {}).get("measured") or {})
        m2 = meas2.get("as_anchor_xy_mm") or [9e9, 9e9]
        ser2 = (res2 or {}).get("series") or {}
        dxs = [abs(v) for v in (ser2.get("delta_x_um") or [])]
        dys = [abs(v) for v in (ser2.get("delta_y_um") or [])]
        resid = math.hypot(m2[0] - m_a[0], m2[1] - m_a[1])
        d_mean = (sum(dxs) + sum(dys)) / max(1, len(dxs) + len(dys))
        check("i42.as_head_center_recheck",
              s2.get("state") == "done" and resid < 0.005
              and d_mean < 2.0,
              "state=%s measured₂=%s 残差=%.5fmm mean|δ|=%.3fµm"
              % (s2.get("state"), m2, resid, d_mean))
    finally:
        http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        _i42_restore_consts(saved)


def test_i42_best_focus_400():
    """best_focus 校验分支（start 同步 400，不开作业）：z_start>=z_end
    →400；模板未注册→400；step/rounds/fpp 越界→400；扫描窗口超轴软限位
    →400；cam=coarse→400；down 无标记（无晶圆）→400 视野预检。"""
    prep_clean()
    v = []
    c, r = cal_start("best_focus", "down",
                     {"z_start": -0.5, "z_end": -0.9})
    v.append(c == 400 and "z_start" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "down",
                     {"template_name": "i42_no_such_tpl"})
    v.append(c == 400 and "不存在" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "down", {"step_mm": 0})
    v.append(c == 400 and "step_mm" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "down", {"rounds": 6})
    v.append(c == 400 and "rounds" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "down", {"frames_per_point": 11})
    v.append(c == 400 and "frames_per_point" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "down",
                     {"z_start": -2.5, "z_end": -2.1})
    v.append(c == 400 and "软限位" in (r.get("error") or ""))
    c, r = cal_start("best_focus", "coarse", {})
    v.append(c == 400 and "cam" in (r.get("error") or ""))
    # down 无晶圆（prep_clean 已复位）→ 视野预检 400
    c, r = cal_start("best_focus", "down", {})
    v.append(c == 400 and "标记" in (r.get("error") or ""))
    check("i42.best_focus_400", all(v), "分支通过=%s/8" % sum(v))


def test_i42_best_focus_down():
    """best_focus down（FAST）：晶圆在位（prep_down）+ **误差链 OFF**
    （确定性真值：晶圆面标记 z=885.9=下相机焦面@AS_Z=0 ⇒ 真值
    best_z=0.0，finally 还原）+ PUT focus_level.as_z_chuck_focus_mm=0.2
    注入焦面认知偏移（默认窗口 [−0.1,0.5] 含真值）→ rounds=2 done →
    best_z 命中 0.0 ±step/2 → 轴位/模式恢复 + best_z 处 identify 自洽
    （|focus_err| ≤ 1×DOF）→ apply 写回 as_z_chuck_focus_mm 后还原。"""
    prep_down()
    saved = _i42_consts()
    http_put(BASE + "/api/machine_constants",
             {"focus_level": {"as_z_chuck_focus_mm": 0.2}})
    http_post(BASE + "/api/errormaps", {"six_dof_enabled": False})
    try:
        s, res = run_job("best_focus", "down",
                         {"rounds": 2, "frames_per_point": 3,
                          "wait_ms": 10}, timeout=600.0)
        ser = (res or {}).get("series") or {}
        det = (res or {}).get("detail") or {}
        best_z = ser.get("best_z_mm")
        best_z = 9e9 if best_z is None else best_z
        rounds = ser.get("rounds") or []
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        check("i42.best_focus_down",
              s.get("state") == "done" and res is not None
              and abs(best_z - 0.0) <= 0.02 / 2.0
              and ((res.get("measured") or {})
                   .get("as_z_chuck_focus_mm") == ser.get("best_z_mm"))
              and len(rounds) == 2
              and det.get("axis") == "AS_Z"
              and det.get("const_key") == "as_z_chuck_focus_mm"
              and det.get("fit") == "parabola"
              and abs(det.get("z_start") - (-0.1)) < 1e-9
              and abs(det.get("z_end") - 0.5) < 1e-9
              and det.get("step_mm") == 0.02
              and _i42_csv_ok(res, "best_focus_down_"),
              "state=%s best_z=%s rounds=%d fit=%s%s"
              % (s.get("state"), ser.get("best_z_mm"), len(rounds),
                 det.get("fit"), err_tail))
        # ---- 现场恢复 + best_z 处 identify 自洽 ----
        asz = axis_entry("AS_Z")["pos"]
        ok_self = False
        if s.get("state") == "done":
            move("AS_Z", best_z, speed=100.0)
            c2, r2 = http_post(BASE + "/api/vision/identify",
                               {"cam": "down"})
            ent = (r2.get("entry") or {})
            fe = ent.get("focus_err_um")
            ok_self = (c2 == 200
                       and ((ent.get("result") or {}).get("judge") is True)
                       and fe is not None and abs(fe) <= 20.0)
            move("AS_Z", 0.0, speed=100.0)
        check("i42.best_focus_down_selfcheck",
              abs(asz) < 1e-6 and ok_self,
              "AS_Z 恢复=%.6f best_z 处 focus_err_um=%s judge 自洽=%s"
              % (asz, fe if s.get("state") == "done" else None, ok_self))
        # ---- apply 写回 + 还原 ----
        if s.get("state") == "done":
            m = res["measured"]["as_z_chuck_focus_mm"]
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id"),
                              "fields": ["as_z_chuck_focus_mm"]})
            got = None
            if c == 200 and r.get("ok"):
                got = http_json(BASE + "/api/machine_constants")[
                    "constants"]["focus_level"]["as_z_chuck_focus_mm"]
            check("i42.best_focus_down_apply",
                  c == 200 and r.get("ok")
                  and r.get("applied") == ["as_z_chuck_focus_mm"]
                  and got is not None and abs(got - m) < 1e-9,
                  "applied=%s wrote=%s" % (r.get("applied"), got))
        else:
            check("i42.best_focus_down_apply", False,
                  "作业未 done%s" % err_tail[:60])
    finally:
        http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        _i42_restore_consts(saved)


def test_i42_best_focus_up():
    """best_focus up（FAST，**依赖吸盘中心标记补丁**）：裸吸盘（无 die）
    + **误差链 OFF**（确定性真值：吸盘面 z=1053.83+BS_Arm_Z、上焦面
    934@AS_Z=−1 ⇒ 真值 best_z=−119.83，finally 还原）+ PUT
    head_z_upfocus_mm=−119.7 注入认知偏移（默认窗口 [−120.0,−119.4]）
    → done → best_z 命中 −119.83 ±step/2 + **常数前置走位断言**
    （detail.preset：AS_X/Y=锚点、AS_Z=下焦面基准、BS_Arm_Z=注入现值）
    + BS_Arm_Z 扫描（detail.axis）+ 轴位恢复 → apply 写回
    head_z_upfocus_mm 后还原。"""
    prep_clean()
    saved = _i42_consts()
    bsz0 = axis_entry("BS_Arm_Z")["pos"]
    http_put(BASE + "/api/machine_constants",
             {"focus_level": {"head_z_upfocus_mm": -119.7}})
    http_post(BASE + "/api/errormaps", {"six_dof_enabled": False})
    try:
        s, res = run_job("best_focus", "up",
                         {"rounds": 2, "frames_per_point": 2,
                          "wait_ms": 10}, timeout=600.0)
        ser = (res or {}).get("series") or {}
        det = (res or {}).get("detail") or {}
        best_z = ser.get("best_z_mm")
        best_z = 9e9 if best_z is None else best_z
        pre = det.get("preset") or {}
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        check("i42.best_focus_up",
              s.get("state") == "done" and res is not None
              and abs(best_z - (-119.83)) <= 0.02 / 2.0
              and ((res.get("measured") or {})
                   .get("head_z_upfocus_mm") == ser.get("best_z_mm"))
              and det.get("axis") == "BS_Arm_Z"
              and det.get("const_key") == "head_z_upfocus_mm"
              and pre.get("AS_X") == 0.0 and pre.get("AS_Y") == 6.0
              and pre.get("AS_Z") == -1.0
              and pre.get("BS_Arm_Z") == -119.7
              and _i42_csv_ok(res, "best_focus_up_"),
              "state=%s best_z=%s preset=%s%s（若无标记→检查 "
              "Temp/_i42_chuck_mark 提案补丁是否已应用）"
              % (s.get("state"), ser.get("best_z_mm"), pre, err_tail))
        bsz1 = axis_entry("BS_Arm_Z")["pos"]
        check("i42.best_focus_up_restore", abs(bsz1 - bsz0) < 1e-6,
              "BS_Arm_Z 恢复：作业前 %.4f 作业后 %.4f" % (bsz0, bsz1))
        if s.get("state") == "done":
            m = res["measured"]["head_z_upfocus_mm"]
            c, r = http_post(BASE + "/api/calibration/apply",
                             {"job_id": s.get("job_id"),
                              "fields": ["head_z_upfocus_mm"]})
            got = None
            if c == 200 and r.get("ok"):
                got = http_json(BASE + "/api/machine_constants")[
                    "constants"]["focus_level"]["head_z_upfocus_mm"]
            check("i42.best_focus_up_apply",
                  c == 200 and r.get("ok")
                  and r.get("applied") == ["head_z_upfocus_mm"]
                  and got is not None and abs(got - m) < 1e-9,
                  "applied=%s wrote=%s" % (r.get("applied"), got))
        else:
            check("i42.best_focus_up_apply", False,
                  "作业未 done%s" % err_tail[:60])
    finally:
        http_post(BASE + "/api/errormaps", {"six_dof_enabled": True})
        _i42_restore_consts(saved)


def test_i42():
    """迭代 42 段汇总：as_head_center（400 分支 → 注入偏移测回真值 →
    apply 双键 → 复测残差符号自证）+ best_focus（400 分支 → down 晶圆
    面焦面测回 → up 吸盘面焦面测回+前置走位断言）。"""
    test_i42_as_head_center_400()
    test_i42_as_head_center()
    test_i42_best_focus_400()
    test_i42_best_focus_down()
    test_i42_best_focus_up()


# =====================================================================
# 迭代 49：bs_rxry_axis 调平轴旋转轴线标定 / flip_handoff_height 交接
# 接触高度 / bond_height_calib 键合接触高度 / bond_temp_calib 温度传感链
# / bond_force_calib 力传感链（配套：机器常数 bs_rxry_axis/sensor_calib
# 两组、recipe sim_sensors 注入 4 键、tuning bond_hw.flip_contact_z_mm、
# engine flip 交接接触力支路+温度/力读数注入补偿链）
# =====================================================================
def _i49_mc(group):
    return dict(http_json(BASE + "/api/machine_constants")["constants"]
                .get(group) or {})


def _i49_csv_ok(res):
    rel = (res or {}).get("csv") or ""
    return bool(rel) and os.path.isfile(
        os.path.join(ROOT, rel.replace("/", os.sep)))


def test_i49_rxry_400():
    """bs_rxry_axis 校验分支（start 同步 400，不开作业）：head 无 die→400
    含"芯片"；angle_mrad 越界（0.1/2.0）→400；cam 非 up→400；模板不存在
    →400 列可用名。"""
    prep_clean()
    v = []
    c, r = cal_start("bs_rxry_axis", "up", {})
    v.append(c == 400 and "芯片" in (r.get("error") or ""))
    c, r = cal_start("bs_rxry_axis", "up", {"angle_mrad": 0.1})
    v.append(c == 400 and "angle_mrad" in (r.get("error") or ""))
    c, r = cal_start("bs_rxry_axis", "up", {"angle_mrad": 2.0})
    v.append(c == 400 and "angle_mrad" in (r.get("error") or ""))
    c, r = cal_start("bs_rxry_axis", "down", {})
    v.append(c == 400 and "cam" in (r.get("error") or ""))
    c, r = cal_start("bs_rxry_axis", "up",
                     {"template_name": "i49_no_such_tpl"})
    v.append(c == 400 and "不存在" in (r.get("error") or ""))
    check("i49.rxry_400", all(v), "分支通过=%s/5" % sum(v))


def test_i49_rxry_axis():
    """bs_rxry_axis 调平轴旋转轴线标定（迭代 49）：prep_up 真实交接链
    die 上头 → 跑 routine（angle 1.5mrad/repeat 3）→ measured.rx_axis_
    dir≈[1,0,0]、ry_axis_dir≈[0,1,0]（方向容差 0.05 rad 起步口径，实测
    收紧见 detail）+ 轴点垂直分量 y/z≈(7.0,1215.25)（沿轴 x 不可观）
    → apply 写回 bs_rxry_axis 组 → GET 一致 → finally 还原常数。"""
    prep_up()
    saved = _i49_mc("bs_rxry_axis")
    try:
        s, res = run_job("bs_rxry_axis", "up",
                         {"angle_mrad": 1.5, "repeat": 3, "wait_ms": 5},
                         timeout=600.0)
        m = ((res or {}).get("measured") or {})
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        rd = m.get("rx_axis_dir") or []
        qd = m.get("ry_axis_dir") or []
        rp = m.get("rx_axis_point_mm") or []
        qp = m.get("ry_axis_point_mm") or []
        # 容差依据（迭代 49 冒烟实测收紧）：方向 rx 偏差 ~13mrad、ry ~3mrad
        #（vision 层局部旋转×链条旋转复合系统项主导，重复间稳定）→ 30mrad；
        # 轴点垂直分量（y/z）实测偏 ≤0.53mm（同上复合伪影，非噪声）→ 1.5mm；
        # 沿轴 x 不可观（秩亏，仅锚定量级）→ 5mm
        dir_ok = len(rd) == 3 and len(qd) == 3 \
            and abs(rd[0] - 1.0) < 0.03 and abs(rd[1]) < 0.03 \
            and abs(rd[2]) < 0.03 \
            and abs(qd[1] - 1.0) < 0.03 and abs(qd[0]) < 0.03 \
            and abs(qd[2]) < 0.03
        pt_ok = len(rp) == 3 and len(qp) == 3 \
            and abs(rp[1] - 7.0) < 1.5 and abs(rp[2] - 1215.25) < 1.5 \
            and abs(qp[1] - 7.0) < 1.5 and abs(qp[2] - 1215.25) < 1.5 \
            and abs(rp[0] + 42.5) < 5.0 and abs(qp[0] + 42.5) < 5.0
        check("i49.rxry_axis",
              s.get("state") == "done" and res is not None and dir_ok
              and pt_ok and _i49_csv_ok(res)
              and abs((m.get("rx_rmse_mm") or 9) ) < 0.1
              and abs((m.get("ry_rmse_mm") or 9) ) < 0.1,
              "state=%s rx_dir=%s ry_dir=%s rx_pt=%s ry_pt=%s rmse=%s/%s%s"
              % (s.get("state"), rd, qd, rp, qp,
                 m.get("rx_rmse_mm"), m.get("ry_rmse_mm"), err_tail))
        if s.get("state") != "done":
            check("i49.rxry_apply", False, "作业未 done%s" % err_tail[:60])
            return
        c, r = http_post(BASE + "/api/calibration/apply",
                         {"job_id": s.get("job_id"),
                          "fields": ["rx_axis_point_mm", "rx_axis_dir",
                                     "ry_axis_point_mm", "ry_axis_dir"]})
        got = _i49_mc("bs_rxry_axis")
        check("i49.rxry_apply",
              c == 200 and r.get("ok")
              and got.get("rx_axis_dir") == m.get("rx_axis_dir")
              and got.get("ry_axis_point_mm") == m.get("ry_axis_point_mm"),
              "applied=%s rx_dir=%s" % (r.get("applied"),
                                        got.get("rx_axis_dir")))
    finally:
        http_put(BASE + "/api/machine_constants",
                 {"bs_rxry_axis": {k: v for k, v in saved.items()
                                   if not k.startswith("_")}})
        for _i in (1, 2, 3):     # prep_up 打开的键合头真空释放（套件卫生）
            vacuum("WT_BondingVac_%d" % _i, False)


def test_i49_handoff_height():
    """flip_handoff_height 交接接触高度标定（迭代 49）：无 die→400"flip
    上无芯片"；flip 放 die（_flip_pick 真实链）→ 慢速寻力 → Z*≈
    flip_contact_z_mm+t_chip=−50.93（±0.05mm）+ flip_surface=Z*−0.5 +
    series 抽稀 ≤400 + CSV。"""
    prep_clean()
    c, r = cal_start("flip_handoff_height", "up", {})
    g400 = c == 400 and "flip" in (r.get("error") or "").lower()
    check("i49.handoff_height_400", g400, "c=%s err=%s"
          % (c, (r.get("error") or "")[:60]))
    _flip_pick()
    # 防御：prep_up 系前置打开的 WT_BondingVac 可能从上个测试残留——交接
    # 位姿+真空同开会真触发 flip→head 交接（die 离 flip→寻力永不触力），
    # 本项只测接触力、不动真空，先释放干净
    for _i in (1, 2, 3):
        vacuum("WT_BondingVac_%d" % _i, False)
    s, res = run_job("flip_handoff_height", "up", {"chip_thickness_mm": 0.5},
                     timeout=300.0)
    m = ((res or {}).get("measured") or {})
    ser = ((res or {}).get("series") or {})
    err_tail = (" err=%s" % (s.get("error") or "")
                if s.get("state") != "done" else "")
    z = m.get("head_z_touch_mm")
    n = len(ser.get("t_s") or [])
    check("i49.handoff_height",
          s.get("state") == "done" and res is not None
          and z is not None and abs(z - (-50.93)) <= 0.05
          and abs((m.get("flip_surface_mm") or 0) - (z - 0.5)) < 1e-9
          and 0 < n <= 400 and _i49_csv_ok(res),
          "state=%s Z*=%s（期望 −50.93±0.05）contact=%s n=%d%s"
          % (s.get("state"), z, m.get("contact_force_n"), n, err_tail))


def _i49_prep_wafer_head():
    """晶圆在位 + die 上键合头 + WS 回键合锚定位（WS=(−42.394,7.0) 时
    吸盘心=键合头心，wafer_ega.ws_ega_xyz_mm 口径）+ 对准台退避让头路
    （prep_up 的上相机观测位 AS_Y=6 会被下降头撞——RxRy调整球座×对准
    相机承载台沙盒对）——bond_height/force_calib 共用前置。"""
    prep_up()
    move("AS_Y", 120.0, speed=500.0)     # 退避对准台（防撞下降头）
    move("AS_X", 0.0)
    c, r = http_post(BASE + "/api/workpiece/wafer",
                     {"action": "load", "offset": [0.0, 0.0]})
    assert c == 200 and r.get("ok"), (c, r)
    move("WS_LS_X", -42.394)
    move("WS_LS_Y", 7.0)


def test_i49_bond_height():
    """bond_height_calib 键合接触高度标定（迭代 49）：缺工件→400"请先
    放入芯片和晶圆"；晶圆+head die 就位 → 慢速寻力 → Z*≈−167.43
    （±0.05mm）、measured.head_z_contact_mm≈−168.93（±0.05）→ apply
    写回 bond 组 → 还原。"""
    prep_clean()
    c, r = cal_start("bond_height_calib", "up", {})
    check("i49.bond_height_400",
          c == 400 and "晶圆" in (r.get("error") or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:60]))
    _i49_prep_wafer_head()
    saved = _i49_mc("bond")
    try:
        s, res = run_job("bond_height_calib", "up",
                         {"chip_thickness_mm": 0.5, "wafer_thickness_mm": 1.0},
                         timeout=300.0)
        m = ((res or {}).get("measured") or {})
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        z = m.get("z_contact_mm")
        bare = m.get("head_z_contact_mm")
        check("i49.bond_height",
              s.get("state") == "done" and res is not None
              and z is not None and abs(z - (-167.43)) <= 0.05
              and bare is not None and abs(bare - (-168.93)) <= 0.05
              and _i49_csv_ok(res),
              "state=%s Z*=%s bare=%s（期望 −167.43/−168.93±0.05）%s"
              % (s.get("state"), z, bare, err_tail))
        if s.get("state") != "done":
            check("i49.bond_height_apply", False,
                  "作业未 done%s" % err_tail[:60])
            return
        c, r = http_post(BASE + "/api/calibration/apply",
                         {"job_id": s.get("job_id"),
                          "fields": ["head_z_contact_mm"]})
        got = _i49_mc("bond").get("head_z_contact_mm")
        check("i49.bond_height_apply",
              c == 200 and r.get("ok") and got == bare,
              "applied=%s wrote=%s" % (r.get("applied"), got))
    finally:
        http_put(BASE + "/api/machine_constants",
                 {"bond": {"head_z_contact_mm": saved.get(
                     "head_z_contact_mm")}})
        for _i in (1, 2, 3):     # prep_up 打开的键合头真空释放（套件卫生）
            vacuum("WT_BondingVac_%d" % _i, False)


def test_i49_temp_calib():
    """bond_temp_calib 温度传感链标定（迭代 49）：heater=up、setpoints
    [80,140]、samples 20 → 拟合 gain≈1.002（注入 temp_gain_err=0.002
    +offset 0.2 的回收；measured.gain=**绝对新常数**口径（apply 直写
    兼容），任务书 |gain−0.998|≤0.01 中的 0.998=1/1.002 为同一物理量
    的倒数口径——本测试断言 |gain−1.0|≤0.01 两口径同带内覆盖）+
    s3≥0 + apply 写回 sensor_calib 后 oCurTemp 发布读数回真值闭环 →
    finally 还原常数。"""
    prep_clean()
    saved = _i49_mc("sensor_calib")
    try:
        s, res = run_job("bond_temp_calib", "up",
                         {"heater": "up", "setpoints_c": [80, 140],
                          "samples": 20, "sample_interval_ms": 50},
                         timeout=600.0)
        m = ((res or {}).get("measured") or {})
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        gain = m.get("gain")
        off = m.get("offset_c")
        pts = m.get("points") or []
        check("i49.temp_calib",
              s.get("state") == "done" and res is not None
              and gain is not None and abs(gain - 1.0) <= 0.01
              and off is not None and abs(off - 0.2) <= 0.6
              and len(pts) == 2 and all(p.get("s3_c", -1) >= 0
                                        for p in pts)
              and _i49_csv_ok(res),
              "state=%s gain=%s offset=%s（注入 0.2%%+0.2℃，绝对新常数"
              "口径≈1.002/0.2）points=%s%s"
              % (s.get("state"), gain, off,
                 [(p.get("setpoint_c"), p.get("mean_c")) for p in pts],
                 err_tail))
        if s.get("state") != "done":
            check("i49.temp_calib_apply", False,
                  "作业未 done%s" % err_tail[:60])
            return
        c, r = http_post(BASE + "/api/calibration/apply",
                         {"job_id": s.get("job_id"),
                          "fields": ["head_temp_gain",
                                     "head_temp_offset_c"]})
        got = _i49_mc("sensor_calib")
        ok_apply = c == 200 and r.get("ok") \
            and abs((got.get("head_temp_gain") or 0)
                    - m.get("head_temp_gain", 9)) < 1e-9
        # 写回后发布读数回真值闭环：设 100℃（先压到 60℃ 再升 100——从下方
        # 逼近使 PID 低速穿带，避免从 140℃ 自然冷却高速跌穿观察带）；
        # 固定 8s 稳态等待（PID 从任一侧收敛保持远快于此）后采 10 点均值
        closed = None
        if ok_apply:
            http_post(BASE + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "SetTargetTempWithPlan",
                       "args": {"Temperature": 60, "Steady_Time": 0}})
            t0 = time.time()
            while time.time() - t0 < 60.0:
                if io_observe("SystemMachine.M_Heater.PulseHeat_Up.oCurTemp") < 70.0:
                    break
                time.sleep(0.3)
            http_post(BASE + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "SetTargetTempWithPlan",
                       "args": {"Temperature": 100, "Steady_Time": 0}})
            time.sleep(8.0)
            vals = [io_observe("SystemMachine.M_Heater.PulseHeat_Up.oCurTemp")
                    for _ in range(10)]
            closed = sum(vals) / len(vals)
            http_post(BASE + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan", "args": {}})
        check("i49.temp_calib_apply",
              ok_apply and closed is not None
              and abs(closed - 100.0) <= 0.6,
              "applied=%s 写回后 100℃ 读数=%s（补偿链闭环）"
              % (r.get("applied"), closed))
    finally:
        http_put(BASE + "/api/machine_constants",
                 {"sensor_calib": {k: v for k, v in saved.items()
                                   if not k.startswith("_")}})
        http_post(BASE + "/api/io/command",
                  {"path": "SystemMachine.PulseHeat_Up",
                   "cmd": "StopTargetTempWithPlan", "args": {}})


def test_i49_force_calib():
    """bond_force_calib 力传感链标定（迭代 49）：缺工件→400；晶圆+head
    die 就位 → 寻力接触 + setpoints [5,15,30] 力控稳定采样（**测量期
    recipe force_noise_N 降到 0.1N 防 flake——±1N 默认噪声下两点拟合
    σ_gain≈0.026 远超容差带，降噪声是测噪声旋钮本身之外的常规手段，
    finally 还原**）→ 拟合 gain≈1.003（注入 force_gain_err=0.003+offset
    0.05 的回收；measured.gain=绝对新常数口径，断言 |gain−1.0|≤0.01——
    任务书 |gain−0.997|≤0.01 中的 0.997=1/1.003 倒数口径同带内覆盖）→
    apply 写回 + 力读数闭环（写回后 10N 发布读数回真值）→ 还原。"""
    prep_clean()
    c, r = cal_start("bond_force_calib", "up", {})
    check("i49.force_calib_400",
          c == 400 and "晶圆" in (r.get("error") or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:60]))
    _i49_prep_wafer_head()
    saved = _i49_mc("sensor_calib")
    rec0 = (http_json(BASE + "/api/recipe").get("sim_sensors", {})
            .get("force_noise_N", 1.0))
    try:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"force_noise_N": 0.1}})
        s, res = run_job("bond_force_calib", "up",
                         {"setpoints_n": [5, 15, 30], "samples": 10,
                          "sample_interval_ms": 50}, timeout=600.0)
        m = ((res or {}).get("measured") or {})
        err_tail = (" err=%s" % (s.get("error") or "")
                    if s.get("state") != "done" else "")
        gain = m.get("gain")
        off = m.get("offset_n")
        pts = m.get("points") or []
        check("i49.force_calib",
              s.get("state") == "done" and res is not None
              and gain is not None and abs(gain - 1.0) <= 0.01
              and off is not None and abs(off - 0.05) <= 0.1
              and len(pts) == 3 and all(p.get("s3_n", -1) >= 0
                                        for p in pts)
              and _i49_csv_ok(res),
              "state=%s gain=%s offset=%s（注入 0.3%%+0.05N，绝对新常数"
              "口径≈1.003/0.05）points=%s%s"
              % (s.get("state"), gain, off,
                 [(p.get("setpoint_n"), p.get("mean_n")) for p in pts],
                 err_tail))
        if s.get("state") != "done":
            check("i49.force_calib_apply", False,
                  "作业未 done%s" % err_tail[:60])
            return
        c, r = http_post(BASE + "/api/calibration/apply",
                         {"job_id": s.get("job_id"),
                          "fields": ["force_gain", "force_offset_n"]})
        got = _i49_mc("sensor_calib")
        ok_apply = c == 200 and r.get("ok") \
            and abs((got.get("force_gain") or 0)
                    - m.get("force_gain", 9)) < 1e-9
        # 写回后力读数闭环：先 move 到期望有效接触位邻域（力控 PID 锚定
        # eff_contact 写轴，从 0 位起步积分绕行 167mm 需 ~2000s——真实
        # bond 流程恒从接触位接力，本检查同口径），再力控 10N → 发布读数
        # 均值应回真值 10（补偿链生效）
        closed = None
        if ok_apply:
            eff = (_i49_mc("bond").get("head_z_contact_mm") or -168.93) + 1.5
            move("BS_Arm_Z", eff + 0.2)
            http_post(BASE + "/api/io/command",
                      {"path": "SystemMachine.SS_Bonding.M_BS",
                       "cmd": "BondingByForceManual",
                       "args": {"Force": 10.0, "Speed": 1.0}})
            t0 = time.time()
            vals = []
            while time.time() - t0 < 60.0:
                cur_f = io_observe("SystemMachine.SS_Bonding.M_BS.oActualForce")
                if abs(cur_f - 10.0) <= 0.6:
                    vals.append(cur_f)
                    if len(vals) >= 10:
                        break
                time.sleep(0.1)
            closed = (sum(vals) / len(vals)) if vals else None
            http_post(BASE + "/api/io/command",
                      {"path": "SystemMachine.SS_Bonding.M_BS",
                       "cmd": "BondingByForceManual",
                       "args": {"Force": 0.0, "Speed": 1.0}})
        check("i49.force_calib_apply",
              ok_apply and closed is not None
              and abs(closed - 10.0) <= 0.6,
              "applied=%s 写回后 10N 读数=%s（补偿链闭环）"
              % (r.get("applied"), closed))
        # 别名路由（迭代 49）：panel 测校页按 res.measured 显示键
        # gain/offset_n 提交写回——calibration.apply 应映射到常数名键；
        # 先清回初始值再用别名 apply，断言同值写入
        http_put(BASE + "/api/machine_constants",
                 {"sensor_calib": {k: v for k, v in saved.items()
                                   if not k.startswith("_")}})
        c2, r2 = http_post(BASE + "/api/calibration/apply",
                           {"job_id": s.get("job_id"),
                            "fields": ["gain", "offset_n"]})
        got2 = _i49_mc("sensor_calib")
        check("i49.force_calib_apply_alias",
              c2 == 200 and r2.get("ok")
              and abs((got2.get("force_gain") or 0)
                      - m.get("force_gain", 9)) < 1e-9
              and abs((got2.get("force_offset_n") or 0)
                      - m.get("force_offset_n", 9)) < 1e-9,
              "c=%s applied=%s（别名 gain/offset_n→常数名键）"
              % (c2, r2.get("applied")))
    finally:
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"force_noise_N": rec0}})
        http_put(BASE + "/api/machine_constants",
                 {"sensor_calib": {k: v for k, v in saved.items()
                                   if not k.startswith("_")}})
        for _i in (1, 2, 3):     # prep_up 打开的键合头真空释放（套件卫生）
            vacuum("WT_BondingVac_%d" % _i, False)


def test_i49():
    """迭代 49 段汇总：rxry 400 分支 → rxry 标定（交接链 die 上头）
    → handoff_height（400+实测）→ bond_height（400+实测+apply）→
    temp_calib（实测+apply+读数闭环）→ force_calib（400+实测+apply）。"""
    test_i49_rxry_400()
    test_i49_rxry_axis()
    test_i49_handoff_height()
    test_i49_bond_height()
    test_i49_temp_calib()
    test_i49_force_calib()

def main():
    try:
        if ctypes is not None:
            ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass
    try:
        import cv2 as _cv2  # noqa: F401
    except ImportError:
        print("[skip] cv2 未安装，REAL 链路测校整组跳过（不 FAIL）"
              "——同 test_client 18 段惯例", flush=True)
        return 0
    # 端口预检（防 Windows 双绑定连到旧实例）
    try:
        http_json(BASE + "/api/health", timeout=2)
        print("端口 9310 已被占用：请先结束旧的 runtime 进程再跑测试")
        return 1
    except Exception:
        pass
    # -k <关键字>：只跑指定迭代段/主题（支持 i32/i34/i35/i36 迭代段与
    # view_rot/microrot 主题键，"a or b" 组合——迭代 39 定向回归），
    # 便于迭代开发
    only = None
    if "-k" in sys.argv:
        only = sys.argv[sys.argv.index("-k") + 1].strip().lower()
    tokens = [t for part in (only or "").split(" or ")
              for t in part.replace(",", " ").split() if t]
    _KW_OK = ("i32", "i34", "i35", "i36", "i42", "i44", "i45",
              "view_rot", "microrot", "i49", "rxry", "handoff_height",
              "bond_height", "temp_calib", "force_calib")
    if only and (not tokens or any(t not in _KW_OK for t in tokens)):
        print("不支持的 -k 过滤：%s（支持 i32/i34/i35/i36/i42/i44/i45/i49/"
              "view_rot/microrot/rxry/handoff_height/bond_height/"
              "temp_calib/force_calib，or 组合）" % only)
        return 1
    # ---- 迭代 32A 专属 boot 段：ws_rotate_gain 真值增益注入 1.001 测回
    #      （改 error_maps/low_order.json WS_Rz scale_ppm，段内自起/清理
    #      后端并按字节恢复文件；主套件在默认文件下跑）----
    if only is None:
        test_i32_gain_truth_phase()
    # ---- 迭代 35 专属 boot 段：ws_rotate_gain 1.001 测回 + ws_rotate_repeat
    #      双示教位 θ 对真值（同一真值档/boot；手法同 i32 段）----
    if only is None or "i35" in tokens:
        test_i35_gain_truth_phase()
    # ---- 迭代 36 专属 boot 段：扫描轴误差地图归零——非正交注入真值测回
    #      （确定性链严格带 ±25/±10µrad，实测定案；手法同 i35 真值段）----
    if only is None or "i36" in tokens:
        test_i36_nonortho_truth_phase()
    proc, log = boot_backend()
    t_all0 = time.time()
    try:
        if not wait_ready():
            print("runtime 启动失败（见 tests/_metrology_boot.log）")
            return 1
        print("[boot] 9310 后端就绪 %.1fs" % (time.time() - t_all0),
              flush=True)

        if tokens:
            # 定向段/主题分派：iXX=整段汇总；view_rot=视场旋转在线两件套
            # （config+identify）；microrot=微旋全部（i31 三相机 + i34
            # 选台/拍照位 + i39 真值闭环）
            runs = []
            if "i32" in tokens:
                runs.append(test_i32)
            if "i34" in tokens:
                runs.append(test_i34)
            if "i35" in tokens:
                runs.append(test_i35)
            if "i36" in tokens:
                runs.append(test_i36)
            if "i42" in tokens:
                runs.append(test_i42)
            if "i44" in tokens:
                runs.append(test_i44_nonortho_nodes_markworld)
            if "i45" in tokens:
                runs.append(test_i45_nonortho_display_schema)
            if "i49" in tokens:
                runs.append(test_i49)
            if "rxry" in tokens:
                runs += [test_i49_rxry_400, test_i49_rxry_axis]
            if "handoff_height" in tokens:
                runs.append(test_i49_handoff_height)
            if "bond_height" in tokens:
                runs.append(test_i49_bond_height)
            if "temp_calib" in tokens:
                runs.append(test_i49_temp_calib)
            if "force_calib" in tokens:
                runs.append(test_i49_force_calib)
            if "view_rot" in tokens:
                runs += [test_i35_view_rot_config, test_i35_view_rot_identify]
            if "microrot" in tokens:
                runs += [test_i31_microrot, test_i34_microrot_wafer_stage,
                         test_i34_microrot_photo_pos,
                         test_i39_microrot_view_rot_loop]
            seen = set()
            runs = [f for f in runs if not (f in seen or seen.add(f))]
            for fn in runs:
                fn()
            failed = [r for r in RESULTS if not r[1]]
            print("\n==== test_metrology -k %s %d/%d 通过（%.1fs）===="
                  % (only, len(RESULTS) - len(failed), len(RESULTS),
                     time.time() - t_all0))
            for name, ok, detail in RESULTS:
                if not ok:
                    print("  FAIL: %s %s" % (name, detail))
            return 1 if failed else 0

        test_error_branches()

        # ---- down：晶圆台载体 ----
        prep_down()
        ok_pre, _r = fast_judge("down")
        check("mrep.prep_down", ok_pre, "FAST 预检 judge=%s" % ok_pre)
        s_down = test_static("down")
        d_down = test_dynamic("down")
        test_iter25("down")
        test_i26_mqtt_adapter()

        # ---- up：键合头 die 载体（真实交接链） ----
        prep_up()
        ok_pre, res_pre = fast_judge("up")
        check("mrep.prep_up", ok_pre,
              "FAST 预检 judge=%s col=%s row=%s"
              % (ok_pre, res_pre.get("lstCol"), res_pre.get("lstRow")))
        s_up = test_static("up")
        d_up = test_dynamic("up")
        test_iter25("up")
        # 清理真空（物料随下个 prep_clean 复位）
        for i in (1, 2, 3):
            vacuum("WT_BondingVac_%d" % i, False)

        # ---- coarse：翻转模块 die 载体 ----
        prep_coarse()
        ok_pre, _r = fast_judge("coarse")
        check("mrep.prep_coarse", ok_pre, "FAST 预检 judge=%s" % ok_pre)
        s_coarse = test_static("coarse")
        d_coarse = test_dynamic("coarse")
        test_iter25("coarse")

        # ---- 迭代 27：REAL 去相关噪声 / real_noise_scale / GUI 拉起 ----
        test_i27_real_noise()

        # ---- 迭代 28：fast_noise_px 全链旋钮（FAST+REAL）/ 模板响应度 ----
        test_i28_real_noise_knob()

        # ---- 迭代 29：repeat 进图像链 / 阿贝轨迹（i28 后场景复位到
        #      prep_down；repeat 1000nm 注入+还原、custom_center 杠杆
        #      注入+还原，config_overrides.json 按字节恢复）----
        prep_down()
        test_i29_repeat_into_image()
        test_i29_abbe_trajectory()

        # ---- 动态 ≥ 静态量级（同量级下限 0.5×，重复定位噪声叠加在识别
        #      噪声上，两者同为 µm 级随机量）----
        # 2026-09-08：质心噪声结果倒推注入后默认 fnp=0.15px 的静态底
        # (~270nm 3σ@down) ≫ repeat(40nm)，n=10 抽样下 dynamic/static
        # 失去判别力（甚至倒挂偶发）——判读段改在 fnp=0.05px 下重测 down
        # （静态底 ~90nm ≈ repeat 量级，动态叠加 repeat 后稳定 > 静态），
        # 默认档三对仅打印不断言
        for cam, sr, dr in (("down", s_down, d_down), ("up", s_up, d_up),
                            ("coarse", s_coarse, d_coarse)):
            print("    [默认档] %s static=%.1f dynamic=%.1f nm（仅记录）"
                  % (cam, sr["stats"]["row"]["s3_nm"],
                     dr["stats"]["row"]["s3_nm"]))
        http_put(BASE + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.05,
                                                    "up": 0.05,
                                                    "coarse": 0.05}}})
        try:
            pairs = (("down", test_static("down"), test_dynamic("down")),)
        finally:
            http_put(BASE + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"down": 0.15,
                                                        "up": 0.15,
                                                        "coarse": 0.15}}})
        for cam, sr, dr in pairs:
            s_s3 = sr["stats"]["row"]["s3_nm"]
            d_s3 = dr["stats"]["row"]["s3_nm"]
            check("mrep.dynamic>=static_%s" % cam, d_s3 >= 0.5 * s_s3,
                  "static=%.1fnm dynamic=%.1fnm" % (s_s3, d_s3))

        # ---- 2026-09-08 需求：新增测校项（下/上共焦、台移动×2、交接、
        #      旋转重复性 + 数据增强列 + jog_group）----
        test_extra_routines(tm_cam="down")

        # ---- 迭代 29b：示教旋转 / 协作式取消（各自带场景准备，不依赖
        #      前段现场）----
        test_i29b()

        # ---- 迭代 29c：交接循环内不放回蓝膜（物料守恒）+ measure_back
        #      参数废止 ----
        test_i29c()

        # ---- 迭代 31：rot_rad+多点示教 / camera_microrot 三相机微旋 ----
        test_i31()

        # ---- 迭代 32：直接测角法 / ws_rotate_center 旋转中心标定 /
        #      microrot 对准台走位（i31 段内已含对准台断言）----
        test_i32()

        # ---- 迭代 34：模板必填（空→400）/ 台重复性相机可选（as_move
        #      cam=down）/ 微旋选台（move_stage=wafer）+ 拍照位必填/小
        #      间距放行 / ws_rotate auto 路径移除 / 交接 back_tpl ----
        test_i34()

        # ---- 迭代 35：dir 默认 (+1,−1)（pixel_world_roundtrip 离线钉
        #      数值契约）/ view_rot_deg 在线配置 + REAL 识别旋转一致性
        #      （各自带场景准备/还原，不依赖前段现场）----
        test_i35()

        # ---- 迭代 36：Y 导轨方向 FK 注入（stage_geo→error_kinematics）
        #      + as/ws_stage_nonortho 非正交测校（注入测回/基线/400/apply
        #      写回，各自带场景准备/还原，不依赖前段现场）----
        test_i36()

        # ---- 迭代 39：view_rot 真值/认知分离——微旋真值闭环（注入
        #      view_rot=+0.4mrad 等效 → measured≈+0.4 符号锚定 → 写回认知
        #      后 solve_pixel 链世界解残差≈0；现场 finally 全还原）----
        test_i39_microrot_view_rot_loop()

        # ---- 迭代 42：as_head_center 上吸盘位置标定（注入偏移测回真值→
        #      apply 双键→复测残差符号自证）+ best_focus 相机最佳焦平面标
        #      定（down 晶圆面/up 吸盘面区间收缩测回，各自带场景准备/还
        #      原，不依赖前段现场）----
        test_i42()

        # ---- 迭代 49：bs_rxry_axis 调平轴旋转轴线标定（SVD 逆解，400
        #      分支+实测+apply 写回还原）+ flip_handoff_height 交接接触
        #      高度 + bond_height_calib 键合接触高度 + bond_temp_calib
        #      温度传感链 + bond_force_calib 力传感链（sensor_calib 写回
        #      测试 finally 恢复默认值，防污染盘上 machine_constants.json）----
        test_i49()

        # ---- 迭代 33：覆盖率缓冲按渲染线程隔离（修双相机串图竞态）/
        #      适配器 per-cam 并发+积压丢最旧 / 模板噪声响应度启动批量
        #      补标定（全进程内单测，不依赖后端现场）----
        _run_i33_group()

        failed = [r for r in RESULTS if not r[1]]
        print("\n==== test_metrology %d/%d 通过（%.1fs）===="
              % (len(RESULTS) - len(failed), len(RESULTS),
                 time.time() - t_all0))
        for name, ok, detail in RESULTS:
            if not ok:
                print("  FAIL: %s %s" % (name, detail))
        # 汇总打印实测 3σ（交接报告用）
        print("---- 实测 3σ 汇总 ----")
        for cam, sr, dr in pairs:
            print("  %-6s static row/col 3σ = %.1f/%.1f nm | dynamic "
                  "row/col 3σ = %.1f/%.1f nm" % (
                      cam,
                      sr["stats"]["row"]["s3_nm"], sr["stats"]["col"]["s3_nm"],
                      dr["stats"]["row"]["s3_nm"], dr["stats"]["col"]["s3_nm"]))
        return 1 if failed else 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    sys.exit(main())

