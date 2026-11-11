# -*- coding: utf-8 -*-
"""recipe.py — 处方（recipe）核心：加载 / schema 校验 / 默认值播种 / 原子写盘。

处方 = 一次键合作业的物料与布局定义，全部参数可在线改（PUT /api/recipe，
部分字段递归合并），校验通过落盘 runtime/recipe.json 并热替换内存处方；
**改处方不动当前物料状态**，布局生效需显式 POST /api/recipe/apply
（workpieces.apply_recipe：die 按新阵列重铺满、晶圆卸载、载体清空）。

布局思路参考 d2w_sim_semiconductor_refactor（sim/config.py marks 段、
sim/flows.py mark_offsets）：晶圆标记按"半径 0.76×R 的对角线四角位"给
默认值（x=y=±0.76R/√2，保证四角物理上在片上）；
每 die 标记由 film.marks.ratio（相对 die 半宽的比例坐标列表，逐轴
位置=ratio×(size_x/2, size_y/2)）描述，die 尺寸变化时标记布局等比缩放，
per_die 控制启用前几个；wafer.site_marks 同构（ratio×site 半宽，per_site），
产品片每个 chip site 自带对准标记——视觉系统（vision_model.collect_marks）
直接消费本 schema 渲染/解析标记（film/head 上的 die 与晶圆 site 共用
同一套比例标记语义）。

**自动布局（精髓：输入芯片大小+间隔，阵列自动算出）**：
- film.dies.mode="auto"（默认）：给 size(芯片大小)/gap(间隔=街宽)/
  edge_margin(膜面边距)；膜面可用区为 film.diameter_mm 的圆，以膜面圆心
  （=film 局部原点=模块中心）为原点铺满 pitch=size+gap 网格，只保留
  四顶点都在 (diameter/2-edge_margin) 圆内的 die（与 wafer_sites 同判定，
  2026-08-24 由矩形规则改为圆形规则，旧键 film_width/film_height/origin/
  center 废除、进来报 400 指路）。默认值 size 4×4 / gap 1×1 / 膜面直径 49 /
  边距 3 → 7×7 跨度裁 4 角 = **45 颗**。
- film.dies.mode="manual"：显式 rows/cols/pitch/origin（不裁圆，对称
  wafer_sites manual；旧版键兼容——旧 recipe.json 无 mode 字段时按
  manual 迁移加载，不报错）。
- wafer_sites.mode="auto"（默认）：给 gap/edge_margin，size 默认 null=
  **跟随 film.dies 的芯片大小**（同一颗芯片）；以晶圆心为原点铺满网格，
  只保留四顶点都在 (R-edge_margin) 圆内的 site。manual 同旧版显式行列。
- 派生块只读、运行时算出、**不写盘**：GET /api/recipe 响应里注入
  film.dies_derived / wafer_sites_derived；全量坐标走
  GET /api/recipe/wafer_sites?limit=N（默认 200，防几千点撑爆轮询）。

单位约定与全线一致：距离 mm、角度 rad。
"""
import copy
import json
import math
import os
import random
import threading

MARK_SHAPES = ("cross", "box", "circle", "quad")
LAYOUT_MODES = ("auto", "manual")

# 默认晶圆标记位：半径 114mm（0.76×R150）对角线上四角 → x=y=±114/√2
# （旧值 (±114,±114) 对角 161mm>R150，物理上不在片上，2026-08-21 修正）
_WAFER_MARK_XY = round(114.0 / math.sqrt(2), 4)     # 80.6102

DEFAULT_RECIPE = {
    "_comment": "D2W 键合设备处方：晶圆/键合位阵列/蓝膜 die 布局。在线改："
                "PUT /api/recipe（部分合并）；生效到物料层：POST /api/recipe/apply。"
                "die/键合位默认 auto 布局=芯片大小+间隔自动算阵列（派生值只读不落盘）",
    "wafer": {
        "diameter_mm": 300.0,
        # 晶圆（玻璃晶圆）厚度。2026-08-31 迭代 9 起键合有效接触位按
        # 裸接触位+晶圆厚+芯片厚自动换算（engine.effective_bond_contact_mm，
        # 机械常数 bond.head_z_contact_mm 为无片裸接触位 −168.93）
        "thickness_mm": 1.0,
        "marks": [
            {"name": "wafer_mark_1", "shape": "cross",
             "x_mm": -_WAFER_MARK_XY, "y_mm": -_WAFER_MARK_XY, "size_um": 2000},
            {"name": "wafer_mark_2", "shape": "cross",
             "x_mm": _WAFER_MARK_XY, "y_mm": _WAFER_MARK_XY, "size_um": 2000},
            {"name": "wafer_mark_3", "shape": "cross",
             "x_mm": _WAFER_MARK_XY, "y_mm": -_WAFER_MARK_XY, "size_um": 2000},
            {"name": "wafer_mark_4", "shape": "cross",
             "x_mm": -_WAFER_MARK_XY, "y_mm": _WAFER_MARK_XY, "size_um": 2000},
        ],
        # 每 site 对准标记（产品片每个 chip site 自带）：结构镜像 film.marks，
        # 位置 = ratio × site 半宽（site 尺寸=wafer_sites 有效尺寸，null 时
        # 跟随 film.dies 芯片大小，与 wafer_sites_derived 一致），site 尺寸
        # 变则标记布局等比缩放；per_site 启用 ratio 前几个。默认
        # 4×quad@±0.76 → 4mm site 时 ±1.52mm（2026-09-04 迭代 13：下芯片
        # =晶圆侧改 quad 套刻标记，与 die 侧 cross 组成 quad⊕cross 套刻）
        "site_marks": {"shape": "quad", "size_um": 150,
                       "ratio": [[-0.76, -0.76], [0.76, -0.76],
                                 [-0.76, 0.76], [0.76, 0.76]],
                       "per_site": 4},
    },
    # 晶圆上的键合位阵列。auto（默认）：size 显式 10×10mm（2026-08-27 起，
    # 不再默认跟随 film.dies；面板清空仍可回填 null=跟随）、gap=0.005mm
    # （5µm 切割道街宽口径，2026-09-18 起；pitch=size+gap=10.005）、
    # edge_margin=3mm，以晶圆心为原点铺满、四顶点在 R-margin 圆内
    # 才保留；manual：显式 rows/cols/pitch/origin（默认 8×8@30mm 居中控件值）
    "wafer_sites": {
        "mode": "auto",
        "size_x_mm": 10.0, "size_y_mm": 10.0,   # null = 跟随 film.dies 芯片大小
        "gap_x_mm": 0.005, "gap_y_mm": 0.005,
        "edge_margin_mm": 3.0,
    },
    # 晶圆上片误差注入（EGA 全局对准要找回的量，2026-08-26 起；参考
    # d2w_sim_semiconductor_refactor wafer_load）：上片时晶圆心相对吸盘心
    # 的平移/旋转偏差（流程生成时刻叠加 jitter 随机落片抖动，1σ 量级）。
    # 真机上这来自机械手上片重复性；仿真里作为显式注入参数，EGA 拟合
    # 结果应找回它（自检）。
    "wafer_load": {
        "load_x_mm": 0.25,
        "load_y_mm": -0.18,
        "load_rz_rad": 0.00025,
        "jitter_xy_um": 5.0,
        "jitter_rz_urad": 30.0,
        "jitter_on": True,
        # 垂向上片注入（2026-08-27 调焦调平新增）：晶圆面相对吸盘面的
        # 面高偏移/倾斜真值（共焦晶圆面高函数与 EGA 之后测量消费；
        # 视觉景深影响 nm 级，视觉模型不动）
        "z_um": 0.0,
        "rx_urad": 0.0,
        "ry_urad": 0.0,
        # 芯片在头姿态注入（同批新增）：交接（flip↔head）完成时写入 die
        # 载体真值，共焦芯片面高函数消费
        "die_head_rx_urad": 0.0,
        "die_head_ry_urad": 0.0,
    },
    # 传感器仿真参数（2026-08-27 调焦调平迭代 2 新增）：仿真注入类参数——
    # 既非工艺处方（不描述物料/布局）也非设备机械常数（不是"认知"可标定
    # 量），是传感模型的物理噪声真值。UI 在测校页"传感器仿真"卡（与
    # wafer_load 卡同区：仿真注入参数归测校页）。
    # 2026-09-08 迭代 22：噪声逐实例拆分（相机 3 台/共焦 2 路/加热器 2 路
    # 各配各的 σ；旧标量值加载时迁移为全实例同值，见 _migrate）。
    "sim_sensors": {
        # 光谱共焦读数噪声 1σ（µm；真实共焦 nm 级，取保守值；回归/冒烟
        # 置 0 得确定性）。confocal.py read() 每次读按探头（up/down）
        # 实时取（热生效）
        "confocal_sigma_um": {"up": 0.065, "down": 0.065},
        # 温度读数噪声 ±幅值（均匀分布，按加热器名分路；2026-08-31 迭代 9
        # 自机械常数 bond 段迁入——噪声真值≠可标定设备认知）与力读数噪声
        # ±幅值（仅 1 路，保持标量）。engine 读数路径每次刷新实时取（热生效）
        "temp_noise_C": {"PulseHeat_Up": 0.5, "Heat_Dw": 0.5},
        "force_noise_N": 1.0,
        # 相机质心噪声底 1σ（px，按相机 id coarse/up/down 分路；
        # 2026-09-06 迭代 21 自 vision_model.FAST_NOISE_PX 常量迁入；
        # 2026-09-10 迭代 28 语义升级：FAST 解析链与 REAL 渲染链共用同一
        # 旋钮——FAST 侧直接作质心 σ；REAL 侧 render_frame 按模板响应度
        # （建模板自标定 px/DN）换算成图像灰度噪声，噪声在图像里）。
        # 两链每次拍照/每帧实时取（热生效）；置 0 得确定性
        "fast_noise_px": {"coarse": 0.15, "up": 0.1, "down": 0.1},
        # REAL 渲染噪声 σ 额外倍率（按相机 id 分路，(0,10]，默认 1.0；
        # 迭代 27 新增）。vision_model.render_frame 每帧热读乘到灰度 σ 上
        # （定性放大/缩小渲染噪声，非物理量，不做 nm 换算）
        "real_noise_scale": {"coarse": 1.0, "up": 1.0, "down": 1.0},
        # 传感链增益/零偏误差注入（迭代 49 新增，真值旋钮）：发布读数链
        # raw=真值×(1+gain_err)+offset_err+noise → engine 再按机器常数
        # sensor_calib 组补偿发布 (raw−offset_cal)/gain_cal。温度按加热器
        # 分路（PulseHeat_Up/Heat_Dw，同 temp_noise_C 口径），力仅 1 路
        # 标量。默认注入 0.2% 增益误差 + 0.2℃/0.05N 零偏（标定前残差
        # 量级，供 bond_temp_calib/bond_force_calib 测校演示闭环）；
        # 置 0 得理想传感链
        "temp_gain_err": {"PulseHeat_Up": 0.002, "Heat_Dw": 0.002},
        "temp_offset_C": {"PulseHeat_Up": 0.2, "Heat_Dw": 0.2},
        "force_gain_err": 0.003,
        "force_offset_N": 0.05,
    },
    # 调焦调平流程参数（2026-08-27 迭代 3）。measure_points 决定 per_site
    # /global 模式下晶圆面与芯片面的测点数；align_first 为流程 POST 参数
    # 的默认值（true=先相机 identify 对准标记再共焦测高）。
    "focus_level": {
        "measure_points": 4,
        "align_first": True,
    },
    # 逐键合位精对准流程参数（2026-08-28 迭代 6）。measure_points=双侧
    # 各测标记数（2~8，≤wafer.site_marks/film.marks 可用标记数）；
    # algorithm=拟合算法（rigid=Kabsch 3 参默认/similarity=Umeyama 4 参/
    # affine=6 参≥3 点/ransac=抗污染刚体/midpoint_2mark=双标记中点角度法
    # TORAY 式，只用前 2 标记）；confirm_*=确认环（复测-重fit-
    # 残余判定）开关/阈值/最大轮次；on_unconfirmed=超次策略（abort=报警
    # 中止/continue_warn=警告放行）。
    "fine_align": {
        "measure_points": 4,
        "algorithm": "rigid",
        "confirm_enable": True,
        "confirm_tol_xy_um": 0.15,
        "confirm_tol_rz_urad": 30.0,
        "confirm_max_iter": 3,
        "on_unconfirmed": "continue_warn",
    },
    # 对准仿真注入（2026-08-28 迭代 6）：真值类参数（非认知可标定量），
    # UI 在测校页"对准仿真"卡（与 sim_sensors 同区）。rz_center_true_
    # offset_um=WS_Rz 真值旋转枢轴相对吸盘心偏移（µm；默认 [0,0]=绕
    # 吸盘心）。认知侧补偿用机械常数 fine_align.rz_center_offset_mm
    # （标定值）——两者不等产生阿贝残余，被确认环实测抓到。
    "sim_align": {
        "rz_center_true_offset_um": [0.0, 0.0],
    },
    # 热压键合流程参数（2026-08-28 迭代 8；2026-08-31 迭代 9 两段速下压
    # 改造：approach_speed_mm_s 废除→fast/search 两段）。force_N/force_tol_N=
    # 键合力目标/到带容差；dwell_s=保压保温时长；head_temp_C=脉冲加热器目标
    # 温度；wafer_temp_C=硅片台保温温度；wafer_preheat=流程是否负责把
    # 硅片台升到 wafer_temp_C（false=只对当前目标等温确认）；
    # wafer_keep_warm=硅片台常保温（true=处方 apply/启动加载即设 Heat_Dw
    # 目标 wafer_temp_C 并常开 PID，流程等温步秒过）；head_cool_after=键合
    # 完成后脉冲加热器关（true）/保温（false）；fast_speed_mm_s=快速下压段
    # （到寻力起始高度）；search_speed_mm_s/touch_force_N=慢速寻力段速度/
    # 触力阈值；search_overtravel_mm=寻力超程余量（有效接触位以下行程
    # 上限，超过仍未触力→FAIL"寻力未接触"）。
    "bond": {
        "force_N": 50.0,
        "force_tol_N": 1.0,
        "dwell_s": 10.0,
        "head_temp_C": 300.0,
        "wafer_temp_C": 30.0,
        "wafer_preheat": True,
        "wafer_keep_warm": True,
        "head_cool_after": True,
        "fast_speed_mm_s": 20.0,
        "search_speed_mm_s": 0.2,
        "touch_force_N": 2.0,
        "search_overtravel_mm": 1.0,
    },
    # 全片键合路径规划参数（2026-09-02 迭代 11）。count=键合数量（0=全部
    # 可用位）；path_mode=site 排序（center_out 中心向外 / serpentine 蛇形
    # / nearest 最近邻）；die_order=取片顺序（nearest_pick 离取片位最近 /
    # serpentine 蛇形）；skip_bonded=跳过已键合位；stop_on_fail=单颗失败
    # 即中止（false=记 fail 继续下一颗）。pipelined（2026-09-05 迭代 15，
    # 产率模式，默认 false=串行）：当前颗 flip_handoff 完成后立即为下一
    # 颗预取（flip_pick+粗对准→待机位 fa_x_standby_mm），与当前颗调平/
    # 精对准/键合并行；预取失败记 window_missed 回退串行，不影响当前颗。
    "full_wafer": {
        "count": 0,
        "path_mode": "center_out",
        "die_order": "nearest_pick",
        "skip_bonded": True,
        "stop_on_fail": False,
        "pipelined": False,
    },
    # 键合后精度检测参数（2026-09-02 迭代 11）。enabled=手动可跑；
    # auto_after_single/auto_after_full=单片/全片键合后自动触发；
    # cool_target_C/cool_rate_cps=检测前主动降温目标温度/速率（℃/s，
    # 仿真为主动冷却，比加热快；engine 侧速率由 tuning bond_hw.
    # wafer_cool_rate_cps 供给，本键为流程期望速率声明）。
    "post_inspect": {
        "enabled": True,
        "auto_after_single": False,
        "auto_after_full": False,
        "cool_target_C": 25.0,
        "cool_rate_cps": 5.0,
    },
    "film": {
        # 蓝膜（贴膜）可用面直径：die 阵列 auto 布局在该圆内铺满（圆心 =
        # film 局部原点 = 模块中心）；蓝膜面渲染也用它（vision_model 圆盘）
        "diameter_mm": 300.0,
        # 蓝膜 die 阵列。auto（默认）：芯片大小+间隔自动布局——以膜面圆心
        # 为原点铺满 pitch=size+gap 网格，四顶点都在 (diameter/2-edge_margin)
        # 圆内才保留（与 wafer_sites 同判定）；默认派生 15×15 跨度裁圆=149 颗；
        # thickness 为描述性字段（原模型无此概念，取合理值）
        "dies": {
            "mode": "auto",
            "size_x_mm": 10.0, "size_y_mm": 10.0,
            "gap_x_mm": 10.0, "gap_y_mm": 10.0,
            "edge_margin_mm": 3.0,
            "thickness_mm": 0.5,
        },
        # 蓝膜顶针过压量（取片时顶针把 die 顶到吸盘面后再过顶 n µm，保证
        # 贴紧；过大磨损芯片表面）。process_flows.build_flip_pick_flow 消费
        "pin_overtravel_um": 20.0,
        # 每 die 标记：位置 = ratio × (size_x/2, size_y/2)（相对 die 半宽的
        # 比例坐标，参考项目 marks.ratio 思路——die 尺寸变则标记布局等比缩放）；
        # per_die 启用 ratio 前几个；shape 标记形状（box=方块/cross=十字/
        # quad=四角小方块套刻），size_um 标记尺寸（真实工艺 ~150µm）。默认
        # 4×cross@±0.76 → 10mm die 时 ±3.8mm（2026-09-04 迭代 13：上芯片
        # =die 侧改 cross，与晶圆侧 quad 组成 quad⊕cross 套刻组合）
        "marks": {"shape": "cross", "size_um": 150,
                  "ratio": [[-0.76, -0.76], [0.76, -0.76],
                            [-0.76, 0.76], [0.76, 0.76]],
                  "per_die": 4},
    },
}

_TOP_KEYS = ("wafer", "wafer_sites", "film", "wafer_load", "sim_sensors",
             "focus_level", "fine_align", "sim_align", "bond",
             "full_wafer", "post_inspect")
# GET 响应注入的派生块键名（PUT 时剥掉，不参与校验/落盘）
_DERIVED_KEYS = ("dies_derived", "wafer_sites_derived")


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _num_field(errors, path, d, key, lo, hi, integer=False):
    """数值字段校验：存在性 + 有限 + 区间；integer=True 时须为整数。"""
    if key not in d:
        errors.append("%s.%s 缺字段" % (path, key))
        return
    v = d[key]
    if integer:
        if not _is_int(v):
            errors.append("%s.%s 须为整数: %r" % (path, key, v))
            return
    elif not _is_num(v):
        errors.append("%s.%s 须为有限数值: %r" % (path, key, v))
        return
    if not (lo < v <= hi):
        errors.append("%s.%s 超出范围 (%s, %s]: %r" % (path, key, lo, hi, v))


def _unknown_keys(errors, path, d, allowed):
    for k in d:
        if k not in allowed and not str(k).startswith("_"):
            errors.append("%s 未知字段: %r" % (path, k))


def _noise_map_field(errors, path, d, key, instances, lo, hi):
    """逐实例噪声字典校验（迭代 22）：须为对象、实例键齐全、无未知键、
    各实例值均在 (lo, hi]（惯例下界 -1e-9：σ=0 合法，回归/冒烟要确定性）。"""
    if key not in d:
        errors.append("%s.%s 缺字段" % (path, key))
        return
    v = d[key]
    if not isinstance(v, dict):
        errors.append("%s.%s 须为逐实例对象 %s: %r"
                      % (path, key, list(instances), v))
        return
    _unknown_keys(errors, "%s.%s" % (path, key), v, instances)
    for inst in instances:
        _num_field(errors, "%s.%s" % (path, key), v, inst, lo, hi)


# ---------------- 派生计算（只读，不落盘） ----------------
def dies_cells(recipe):
    """film.dies 全量 die 格点 [(r, c, x, y), ...]（film 局部坐标，膜面
    圆心为原点；r/c 为行列编号，auto 以中心格起算、跨度含被裁空位）。
    auto：以圆心为原点铺满 pitch=size+gap 网格，四顶点都在
    (diameter/2-edge_margin) 圆内才保留（与 wafer_sites 同判定）；
    manual：显式 rows/cols/pitch/origin 全量矩形（不裁圆）。"""
    d = recipe["film"]["dies"]
    mode = d.get("mode", "manual")      # 旧键无 mode 时按 manual（迁移兜底）
    if mode == "manual":
        rows, cols = int(d["rows"]), int(d["cols"])
        px, py = float(d["pitch_x_mm"]), float(d["pitch_y_mm"])
        ox, oy = float(d["origin_x_mm"]), float(d["origin_y_mm"])
        return [(r, c, round(ox + c * px, 4), round(oy + r * py, 4))
                for r in range(rows) for c in range(cols)]
    sx, sy = float(d["size_x_mm"]), float(d["size_y_mm"])
    px = sx + float(d["gap_x_mm"])
    py = sy + float(d["gap_y_mm"])
    rlim = float(recipe["film"]["diameter_mm"]) / 2.0 \
        - float(d["edge_margin_mm"])
    hx, hy = sx / 2.0, sy / 2.0
    imax = int(math.floor((rlim - hx) / px))     # |x|+hx ≤ rlim 的最大列号
    jmax = int(math.floor((rlim - hy) / py))
    out = []
    for j in range(-jmax, jmax + 1):
        y = j * py
        for i in range(-imax, imax + 1):
            x = i * px
            # 四顶点都在圆内 ⇔ 最远角 (|x|+hx, |y|+hy) 在圆内
            if math.hypot(abs(x) + hx, abs(y) + hy) <= rlim + 1e-9:
                out.append((j + jmax, i + imax, round(x, 4), round(y, 4)))
    return out


def derive_dies(recipe):
    """film.dies 派生布局：{mode, rows, cols, pitch_x_mm, pitch_y_mm,
    count, effective_origin:[x,y], positions_preview(前 8 个 [x,y])}。
    auto=大小+间隔按膜面圆自动算（rows/cols=铺满跨度含被圆裁掉的空位，
    count=实际保留颗数，effective_origin 恒为膜面圆心 [0,0]）；
    manual=显式值（count=rows×cols 不裁圆）。"""
    d = recipe["film"]["dies"]
    mode = d.get("mode", "manual")      # 旧键无 mode 时按 manual（迁移兜底）
    cells = dies_cells(recipe)
    if mode == "manual":
        rows, cols = int(d["rows"]), int(d["cols"])
        px, py = float(d["pitch_x_mm"]), float(d["pitch_y_mm"])
        ox, oy = float(d["origin_x_mm"]), float(d["origin_y_mm"])
    else:
        sx, sy = float(d["size_x_mm"]), float(d["size_y_mm"])
        px = sx + float(d["gap_x_mm"])
        py = sy + float(d["gap_y_mm"])
        # 行列数 = 铺满网格的列/行跨度（含被圆裁掉的空位）
        rlim = float(recipe["film"]["diameter_mm"]) / 2.0 \
            - float(d["edge_margin_mm"])
        cols = 2 * int(math.floor((rlim - sx / 2.0) / px)) + 1
        rows = 2 * int(math.floor((rlim - sy / 2.0) / py)) + 1
        ox = oy = 0.0                   # auto 原点恒为膜面圆心
    return {"mode": mode, "rows": rows, "cols": cols,
            "pitch_x_mm": round(px, 6), "pitch_y_mm": round(py, 6),
            "count": len(cells),
            "effective_origin": [round(ox, 6), round(oy, 6)],
            "positions_preview": [[x, y] for _, _, x, y in cells[:8]]}


def _dies_chip_size(recipe):
    """film.dies 配置的芯片大小（wafer_sites auto 的默认跟随源）。"""
    d = recipe["film"]["dies"]
    return float(d["size_x_mm"]), float(d["size_y_mm"])


def wafer_sites_cells(recipe):
    """wafer_sites 全量 site 格点 [(r, c, x, y), ...]（晶圆局部坐标，圆心
    为原点；r/c 为行列编号，auto 以中心格起算、跨度含被裁空位）。"""
    s = recipe["wafer_sites"]
    mode = s.get("mode", "manual")
    if mode == "manual":
        rows, cols = int(s["rows"]), int(s["cols"])
        px, py = float(s["pitch_x_mm"]), float(s["pitch_y_mm"])
        ox, oy = float(s["origin_x_mm"]), float(s["origin_y_mm"])
        return [(r, c, round(ox + c * px, 4), round(oy + r * py, 4))
                for r in range(rows) for c in range(cols)]
    sx, sy = wafer_sites_chip_size(recipe)
    px, py = sx + float(s["gap_x_mm"]), sy + float(s["gap_y_mm"])
    rlim = float(recipe["wafer"]["diameter_mm"]) / 2.0 \
        - float(s["edge_margin_mm"])
    hx, hy = sx / 2.0, sy / 2.0
    imax = int(math.floor((rlim - hx) / px))     # |x|+hx ≤ rlim 的最大列号
    jmax = int(math.floor((rlim - hy) / py))
    out = []
    for j in range(-jmax, jmax + 1):
        y = j * py
        for i in range(-imax, imax + 1):
            x = i * px
            # 四顶点都在圆内 ⇔ 最远角 (|x|+hx, |y|+hy) 在圆内
            if math.hypot(abs(x) + hx, abs(y) + hy) <= rlim + 1e-9:
                out.append((j + jmax, i + imax, round(x, 4), round(y, 4)))
    return out


def wafer_sites_chip_size(recipe):
    """wafer_sites 有效芯片大小 (sx, sy)（size 为 null 时跟随 film.dies）。"""
    s = recipe["wafer_sites"]
    sx, sy = s.get("size_x_mm"), s.get("size_y_mm")
    fx, fy = _dies_chip_size(recipe)
    return (fx if sx is None else float(sx),
            fy if sy is None else float(sy))


def wafer_sites_positions(recipe):
    """wafer_sites 全量坐标 [[x,y],...]（晶圆局部坐标，圆心为原点）。"""
    return [[x, y] for _, _, x, y in wafer_sites_cells(recipe)]


def derive_wafer_sites(recipe):
    """wafer_sites 派生布局：{mode, rows, cols, pitch_x_mm, pitch_y_mm,
    total_sites, positions_preview(前 8 个)}（全量坐标走 wafer_sites 端点）。"""
    s = recipe["wafer_sites"]
    mode = s.get("mode", "manual")
    pos = wafer_sites_positions(recipe)
    if mode == "manual":
        rows, cols = int(s["rows"]), int(s["cols"])
        px, py = float(s["pitch_x_mm"]), float(s["pitch_y_mm"])
    else:
        sx = s.get("size_x_mm")
        sy = s.get("size_y_mm")
        fx, fy = _dies_chip_size(recipe)
        sx = fx if sx is None else float(sx)
        sy = fy if sy is None else float(sy)
        px, py = sx + float(s["gap_x_mm"]), sy + float(s["gap_y_mm"])
        # 行列数 = 铺满网格的列/行跨度（含被圆裁掉的空位）
        rlim = float(recipe["wafer"]["diameter_mm"]) / 2.0 \
            - float(s["edge_margin_mm"])
        cols = 2 * int(math.floor((rlim - sx / 2.0) / px)) + 1
        rows = 2 * int(math.floor((rlim - sy / 2.0) / py)) + 1
    return {"mode": mode, "rows": rows, "cols": cols,
            "pitch_x_mm": round(px, 6), "pitch_y_mm": round(py, 6),
            "total_sites": len(pos), "positions_preview": pos[:8]}


def wafer_load_inject_params(wl):
    """由处方 wafer_load 块计算一次上片注入真值（EGA 流程上片步与手动上片
    inject_recipe 共用，单一事实源）：load_x/y/rz 确定性值，jitter_on 时
    叠加 gauss 落片抖动（rz 限幅 ±0.01 rad）；垂向 z/rx/ry 不抖。
    返回 {offset:[x,y](mm), rz(rad), z(mm), rx(rad), ry(rad)}，
    可直接 **展开喂 workpieces.load_wafer。"""
    x = float(wl["load_x_mm"])
    y = float(wl["load_y_mm"])
    rz = float(wl["load_rz_rad"])
    if bool(wl.get("jitter_on", True)):
        x += random.gauss(0.0, float(wl["jitter_xy_um"]) / 1000.0)
        y += random.gauss(0.0, float(wl["jitter_xy_um"]) / 1000.0)
        rz += random.gauss(0.0, float(wl["jitter_rz_urad"]) / 1e6)
        rz = max(-0.01, min(0.01, rz))
    return {"offset": [x, y], "rz": rz,
            "z": float(wl.get("z_um", 0.0)) / 1000.0,
            "rx": float(wl.get("rx_urad", 0.0)) / 1e6,
            "ry": float(wl.get("ry_urad", 0.0)) / 1e6}


# ---------------- schema 校验 ----------------
# 2026-08-24 废除的 film.dies 旧键 → 报错文案指路（两模式都查）
_OLD_DIES_KEYS = {
    "film_width_mm": "film.diameter_mm",
    "film_height_mm": "film.diameter_mm",
    "center": "（膜面圆心即原点，居中开关不再需要）",
}
# auto 模式另废 origin（manual 仍保留显式 origin）
_OLD_DIES_AUTO_KEYS = ("origin_x_mm", "origin_y_mm")


def _validate_dies(errors, recipe, d):
    if not isinstance(d, dict):
        errors.append("film.dies 缺字段或须为对象")
        return
    mode = d.get("mode")
    auto_keys = ("mode", "size_x_mm", "size_y_mm", "gap_x_mm", "gap_y_mm",
                 "edge_margin_mm", "thickness_mm")
    manual_keys = ("mode", "rows", "cols", "pitch_x_mm", "pitch_y_mm",
                   "origin_x_mm", "origin_y_mm", "size_x_mm", "size_y_mm",
                   "thickness_mm")
    if mode not in LAYOUT_MODES:
        errors.append("film.dies.mode 须为 auto/manual: %r" % (mode,))
        _unknown_keys(errors, "film.dies", d,
                      tuple(sorted(set(auto_keys + manual_keys)
                                   | set(_OLD_DIES_KEYS))))
        return
    # 已废除旧键：两模式统一 400 指路（先于其他校验报出，方便迁移）
    for k, hint in _OLD_DIES_KEYS.items():
        if k in d:
            errors.append("film.dies.%s 已废除（膜面改为圆形规则），请改用 %s"
                          % (k, hint))
    # 两种模式都带芯片大小（描述 + wafer_sites auto 跟随源）与厚度
    _num_field(errors, "film.dies", d, "size_x_mm", 0.0, 100.0)
    _num_field(errors, "film.dies", d, "size_y_mm", 0.0, 100.0)
    _num_field(errors, "film.dies", d, "thickness_mm", 0.0, 10.0)
    dia = (recipe.get("film") or {}).get("diameter_mm")
    if mode == "auto":
        _unknown_keys(errors, "film.dies", d,
                      auto_keys + tuple(_OLD_DIES_KEYS)
                      + _OLD_DIES_AUTO_KEYS)
        for k in _OLD_DIES_AUTO_KEYS:
            if k in d:
                errors.append("film.dies.%s 在 auto 模式已废除（原点恒为膜面"
                              "圆心）；需显式原点请切 manual" % k)
        _num_field(errors, "film.dies", d, "gap_x_mm", -1e-9, 100.0)
        _num_field(errors, "film.dies", d, "gap_y_mm", -1e-9, 100.0)
        _num_field(errors, "film.dies", d, "edge_margin_mm", -1e-9, 250.0)
        if _is_num(dia) and _is_num(d.get("edge_margin_mm")) \
                and d["edge_margin_mm"] >= dia / 2.0:
            errors.append("film.dies.edge_margin_mm %.4g 不小于膜面半径 %.4g"
                          % (d["edge_margin_mm"], dia / 2.0))
        # 可行性：一颗 die 都排不下 → 报错
        try:
            if not errors and not dies_cells(recipe):
                errors.append("film.dies: 芯片/边距过大，膜面内排不下 1 颗 die")
        except (KeyError, TypeError, ValueError):
            pass            # 其他字段错误已在上面逐条报出
    else:
        _unknown_keys(errors, "film.dies", d,
                      manual_keys + tuple(_OLD_DIES_KEYS))
        _num_field(errors, "film.dies", d, "rows", 0, 500, integer=True)
        _num_field(errors, "film.dies", d, "cols", 0, 500, integer=True)
        _num_field(errors, "film.dies", d, "pitch_x_mm", 0.0, 100.0)
        _num_field(errors, "film.dies", d, "pitch_y_mm", 0.0, 100.0)
        _num_field(errors, "film.dies", d, "origin_x_mm", -1000.0, 1000.0)
        _num_field(errors, "film.dies", d, "origin_y_mm", -1000.0, 1000.0)
        if _is_num(dia) and _is_int(d.get("cols")) \
                and _is_num(d.get("pitch_x_mm")):
            if (d["cols"] - 1) * d["pitch_x_mm"] > dia:
                errors.append("film.dies 阵列宽 %.1fmm 超出膜面直径 %.1fmm"
                              % ((d["cols"] - 1) * d["pitch_x_mm"], dia))
        if _is_num(dia) and _is_int(d.get("rows")) \
                and _is_num(d.get("pitch_y_mm")):
            if (d["rows"] - 1) * d["pitch_y_mm"] > dia:
                errors.append("film.dies 阵列高 %.1fmm 超出膜面直径 %.1fmm"
                              % ((d["rows"] - 1) * d["pitch_y_mm"], dia))


def _validate_wafer_sites(errors, recipe):
    s = recipe.get("wafer_sites")
    if not isinstance(s, dict):
        errors.append("wafer_sites 缺字段或须为对象")
        return
    mode = s.get("mode")
    auto_keys = ("mode", "size_x_mm", "size_y_mm", "gap_x_mm", "gap_y_mm",
                 "edge_margin_mm")
    manual_keys = ("mode", "rows", "cols", "pitch_x_mm", "pitch_y_mm",
                   "origin_x_mm", "origin_y_mm")
    if mode not in LAYOUT_MODES:
        errors.append("wafer_sites.mode 须为 auto/manual: %r" % (mode,))
        _unknown_keys(errors, "wafer_sites", s,
                      tuple(sorted(set(auto_keys + manual_keys))))
        return
    dia = (recipe.get("wafer") or {}).get("diameter_mm")
    if mode == "auto":
        _unknown_keys(errors, "wafer_sites", s, auto_keys)
        for k in ("size_x_mm", "size_y_mm"):      # null = 跟随 film 芯片大小
            if k not in s:
                errors.append("wafer_sites.%s 缺字段（null=跟随 film.dies）" % k)
                continue
            v = s.get(k)
            if v is not None and not (_is_num(v) and 0.0 < v <= 100.0):
                errors.append("wafer_sites.%s 须为 null 或 (0, 100]: %r"
                              % (k, v))
        _num_field(errors, "wafer_sites", s, "gap_x_mm", -1e-9, 200.0)
        _num_field(errors, "wafer_sites", s, "gap_y_mm", -1e-9, 200.0)
        _num_field(errors, "wafer_sites", s, "edge_margin_mm", -1e-9, 250.0)
        if _is_num(dia) and _is_num(s.get("edge_margin_mm")) \
                and s["edge_margin_mm"] >= dia / 2.0:
            errors.append("wafer_sites.edge_margin_mm %.4g 不小于晶圆半径 %.4g"
                          % (s["edge_margin_mm"], dia / 2.0))
        # 可行性：一个键合位都排不下 → 报错
        try:
            if not errors and not wafer_sites_positions(recipe):
                errors.append("wafer_sites: 芯片/边距过大，晶圆内排不下 "
                              "1 个键合位")
        except (KeyError, TypeError, ValueError):
            pass            # 其他字段错误已在上面逐条报出
    else:
        _unknown_keys(errors, "wafer_sites", s, manual_keys)
        _num_field(errors, "wafer_sites", s, "rows", 0, 200, integer=True)
        _num_field(errors, "wafer_sites", s, "cols", 0, 200, integer=True)
        _num_field(errors, "wafer_sites", s, "pitch_x_mm", 0.0, 200.0)
        _num_field(errors, "wafer_sites", s, "pitch_y_mm", 0.0, 200.0)
        _num_field(errors, "wafer_sites", s, "origin_x_mm", -1000.0, 1000.0)
        _num_field(errors, "wafer_sites", s, "origin_y_mm", -1000.0, 1000.0)
        if _is_num(dia) and _is_int(s.get("cols")) \
                and _is_num(s.get("pitch_x_mm")):
            if (s["cols"] - 1) * s["pitch_x_mm"] > dia:
                errors.append("wafer_sites 阵列宽 %.1fmm 超出晶圆直径 %.1fmm"
                              % ((s["cols"] - 1) * s["pitch_x_mm"], dia))
        if _is_num(dia) and _is_int(s.get("rows")) \
                and _is_num(s.get("pitch_y_mm")):
            if (s["rows"] - 1) * s["pitch_y_mm"] > dia:
                errors.append("wafer_sites 阵列高 %.1fmm 超出晶圆直径 %.1fmm"
                              % ((s["rows"] - 1) * s["pitch_y_mm"], dia))


def validate(recipe):
    """全量 schema 校验，返回错误字符串列表（空 = 通过）。逐条定位到字段路径。"""
    errors = []
    if not isinstance(recipe, dict):
        return ["处方须为 JSON 对象"]
    _unknown_keys(errors, "recipe", recipe, _TOP_KEYS)

    # ---- wafer ----
    w = recipe.get("wafer")
    if not isinstance(w, dict):
        errors.append("wafer 缺字段或须为对象")
    else:
        _unknown_keys(errors, "wafer", w, ("diameter_mm", "thickness_mm",
                                           "marks", "site_marks"))
        _num_field(errors, "wafer", w, "diameter_mm", 0.0, 500.0)
        _num_field(errors, "wafer", w, "thickness_mm", 0.0, 10.0)
        dia = w.get("diameter_mm")
        marks = w.get("marks")
        if not isinstance(marks, list):
            errors.append("wafer.marks 缺字段或须为数组")
        else:
            names = set()
            for i, m in enumerate(marks):
                p = "wafer.marks[%d]" % i
                if not isinstance(m, dict):
                    errors.append("%s 须为对象" % p)
                    continue
                _unknown_keys(errors, p, m,
                              ("name", "shape", "x_mm", "y_mm", "size_um"))
                nm = m.get("name")
                if not (isinstance(nm, str) and nm.strip()):
                    errors.append("%s.name 须为非空字符串" % p)
                elif nm in names:
                    errors.append("%s.name 重复: %r" % (p, nm))
                else:
                    names.add(nm)
                if m.get("shape") not in MARK_SHAPES:
                    errors.append("%s.shape 非法: %r（可选 %s）"
                                  % (p, m.get("shape"), "/".join(MARK_SHAPES)))
                for k in ("x_mm", "y_mm"):
                    _num_field(errors, p, m, k, -500.0, 500.0)
                _num_field(errors, p, m, "size_um", 0.0, 50000.0)
                if _is_num(dia):
                    for k in ("x_mm", "y_mm"):
                        if _is_num(m.get(k)) and abs(m[k]) > dia:
                            errors.append("%s.%s 超出晶圆直径范围: %r"
                                          % (p, k, m[k]))
        # 每 site 对准标记（结构镜像 film.marks；ratio×site 半宽等比缩放）
        sm = w.get("site_marks")
        if not isinstance(sm, dict):
            errors.append("wafer.site_marks 缺字段或须为对象")
        else:
            _unknown_keys(errors, "wafer.site_marks", sm,
                          ("shape", "size_um", "ratio", "per_site"))
            if sm.get("shape") not in MARK_SHAPES:
                errors.append("wafer.site_marks.shape 非法: %r（可选 %s）"
                              % (sm.get("shape"), "/".join(MARK_SHAPES)))
            _num_field(errors, "wafer.site_marks", sm, "size_um", 0.0, 5000.0)
            ratio = sm.get("ratio")
            if not (isinstance(ratio, list) and ratio):
                errors.append("wafer.site_marks.ratio 缺字段或须为非空数组")
            else:
                for i, pt in enumerate(ratio):
                    if not (isinstance(pt, (list, tuple)) and len(pt) == 2
                            and all(_is_num(v) for v in pt)):
                        errors.append("wafer.site_marks.ratio[%d] 须为 [x,y]"
                                      " 数值对" % i)
                    elif not all(-1.0 <= v <= 1.0 for v in pt):
                        errors.append("wafer.site_marks.ratio[%d] 元素须在 "
                                      "[-1,1]（相对 site 半宽比例）: %r"
                                      % (i, pt))
            per = sm.get("per_site")
            if not _is_int(per):
                errors.append("wafer.site_marks.per_site 须为整数: %r" % (per,))
            elif isinstance(ratio, list) and not (0 <= per <= len(ratio)):
                errors.append("wafer.site_marks.per_site 须在 [0, "
                              "len(ratio)=%d]: %r" % (len(ratio), per))

    # ---- wafer_load（上片误差注入参数） ----
    wl = recipe.get("wafer_load")
    if not isinstance(wl, dict):
        errors.append("wafer_load 缺字段或须为对象")
    else:
        _unknown_keys(errors, "wafer_load", wl,
                      ("load_x_mm", "load_y_mm", "load_rz_rad",
                       "jitter_xy_um", "jitter_rz_urad", "jitter_on",
                       "z_um", "rx_urad", "ry_urad",
                       "die_head_rx_urad", "die_head_ry_urad"))
        _num_field(errors, "wafer_load", wl, "load_x_mm", -100.0, 100.0)
        _num_field(errors, "wafer_load", wl, "load_y_mm", -100.0, 100.0)
        _num_field(errors, "wafer_load", wl, "load_rz_rad", -0.1, 0.1)
        _num_field(errors, "wafer_load", wl, "jitter_xy_um", -1e-9, 1000.0)
        _num_field(errors, "wafer_load", wl, "jitter_rz_urad", -1e-9, 10000.0)
        _num_field(errors, "wafer_load", wl, "z_um", -1000.0, 1000.0)
        _num_field(errors, "wafer_load", wl, "rx_urad", -10000.0, 10000.0)
        _num_field(errors, "wafer_load", wl, "ry_urad", -10000.0, 10000.0)
        _num_field(errors, "wafer_load", wl, "die_head_rx_urad",
                   -10000.0, 10000.0)
        _num_field(errors, "wafer_load", wl, "die_head_ry_urad",
                   -10000.0, 10000.0)
        if not isinstance(wl.get("jitter_on"), bool):
            errors.append("wafer_load.jitter_on 须为布尔: %r"
                          % (wl.get("jitter_on"),))

    # ---- sim_sensors（传感器仿真参数：噪声真值注入，迭代 22 逐实例拆分） ----
    ss = recipe.get("sim_sensors")
    if not isinstance(ss, dict):
        errors.append("sim_sensors 缺字段或须为对象")
    else:
        _unknown_keys(errors, "sim_sensors", ss, ("confocal_sigma_um",
                                                  "temp_noise_C",
                                                  "force_noise_N",
                                                  "fast_noise_px",
                                                  "real_noise_scale",
                                                  "temp_gain_err",
                                                  "temp_offset_C",
                                                  "force_gain_err",
                                                  "force_offset_N"))
        # 光谱共焦按探头（up/down）；相机质心噪声按相机（coarse/up/down，
        # 2026-09-06 迭代 21 新增、迭代 22 逐相机拆分；迭代 28 起 FAST+REAL
        # 共用，REAL 侧按模板响应度换算灰度噪声）
        _noise_map_field(errors, "sim_sensors", ss, "confocal_sigma_um",
                         ("up", "down"), -1e-9, 10.0)
        _noise_map_field(errors, "sim_sensors", ss, "fast_noise_px",
                         ("coarse", "up", "down"), -1e-9, 2.0)
        # REAL 渲染噪声 σ 倍率（迭代 27）：倍率须为正，(0, 10]
        _noise_map_field(errors, "sim_sensors", ss, "real_noise_scale",
                         ("coarse", "up", "down"), 0.0, 10.0)
        # 温度按加热器分路（2026-08-31 迭代 9 自机械常数 bond 段迁入）；
        # 力读数噪声仅 1 路，保持标量
        _noise_map_field(errors, "sim_sensors", ss, "temp_noise_C",
                         ("PulseHeat_Up", "Heat_Dw"), -1e-9, 100.0)
        _num_field(errors, "sim_sensors", ss, "force_noise_N", -1e-9, 100.0)
        # 传感链增益/零偏误差注入（迭代 49）：温度按加热器分路（同
        # temp_noise_C 的 _noise_map_field 口径），力为标量；增益误差为
        # 相对量（±50% 放行界），零偏 ℃/N
        _noise_map_field(errors, "sim_sensors", ss, "temp_gain_err",
                         ("PulseHeat_Up", "Heat_Dw"), -0.5, 0.5)
        _noise_map_field(errors, "sim_sensors", ss, "temp_offset_C",
                         ("PulseHeat_Up", "Heat_Dw"), -50.0, 50.0)
        _num_field(errors, "sim_sensors", ss, "force_gain_err", -0.5, 0.5)
        _num_field(errors, "sim_sensors", ss, "force_offset_N", -50.0, 50.0)

    # ---- focus_level（调焦调平流程参数） ----
    fl = recipe.get("focus_level")
    if not isinstance(fl, dict):
        errors.append("focus_level 缺字段或须为对象")
    else:
        _unknown_keys(errors, "focus_level", fl,
                      ("measure_points", "align_first"))
        _num_field(errors, "focus_level", fl, "measure_points", 2, 8,
                   integer=True)
        if not isinstance(fl.get("align_first"), bool):
            errors.append("focus_level.align_first 须为布尔: %r"
                          % (fl.get("align_first"),))

    # ---- fine_align（精对准流程参数，2026-08-28 迭代 6） ----
    fa = recipe.get("fine_align")
    if not isinstance(fa, dict):
        errors.append("fine_align 缺字段或须为对象")
    else:
        _unknown_keys(errors, "fine_align", fa,
                      ("measure_points", "algorithm", "confirm_enable",
                       "confirm_tol_xy_um", "confirm_tol_rz_urad",
                       "confirm_max_iter", "on_unconfirmed"))
        _num_field(errors, "fine_align", fa, "measure_points", 1, 8,
                   integer=True)
        if fa.get("algorithm") not in ("rigid", "similarity", "affine",
                                       "ransac", "midpoint_2mark"):
            errors.append("fine_align.algorithm 非法: %r（合法：rigid/"
                          "similarity/affine/ransac/midpoint_2mark）"
                          % (fa.get("algorithm"),))
        if not isinstance(fa.get("confirm_enable"), bool):
            errors.append("fine_align.confirm_enable 须为布尔: %r"
                          % (fa.get("confirm_enable"),))
        _num_field(errors, "fine_align", fa, "confirm_tol_xy_um",
                   1e-9, 1000.0)
        _num_field(errors, "fine_align", fa, "confirm_tol_rz_urad",
                   1e-9, 100000.0)
        _num_field(errors, "fine_align", fa, "confirm_max_iter", 0, 10,
                   integer=True)
        if fa.get("on_unconfirmed") not in ("abort", "continue_warn"):
            errors.append("fine_align.on_unconfirmed 非法: %r（合法：abort/"
                          "continue_warn）" % (fa.get("on_unconfirmed"),))

    # ---- sim_align（对准仿真注入：真值类参数，2026-08-28 迭代 6） ----
    sa = recipe.get("sim_align")
    if not isinstance(sa, dict):
        errors.append("sim_align 缺字段或须为对象")
    else:
        _unknown_keys(errors, "sim_align", sa, ("rz_center_true_offset_um",))
        v = sa.get("rz_center_true_offset_um")
        if not (isinstance(v, (list, tuple)) and len(v) == 2
                and all(_is_num(x) and abs(x) <= 5000.0 for x in v)):
            errors.append("sim_align.rz_center_true_offset_um 须为二维向量"
                          "（各分量 ±5000 µm）: %r" % (v,))

    # ---- bond（热压键合流程参数，2026-08-28 迭代 8） ----
    bd = recipe.get("bond")
    if not isinstance(bd, dict):
        errors.append("bond 缺字段或须为对象")
    else:
        _unknown_keys(errors, "bond", bd,
                      ("force_N", "force_tol_N", "dwell_s", "head_temp_C",
                       "wafer_temp_C", "wafer_preheat", "wafer_keep_warm",
                       "head_cool_after", "fast_speed_mm_s",
                       "search_speed_mm_s", "touch_force_N",
                       "search_overtravel_mm"))
        _num_field(errors, "bond", bd, "force_N", -1e-9, 2000.0)
        _num_field(errors, "bond", bd, "force_tol_N", 1e-9, 100.0)
        _num_field(errors, "bond", bd, "dwell_s", -1e-9, 3600.0)
        _num_field(errors, "bond", bd, "head_temp_C", 0.0, 1000.0)
        _num_field(errors, "bond", bd, "wafer_temp_C", 0.0, 1000.0)
        _num_field(errors, "bond", bd, "fast_speed_mm_s", 1e-9, 200.0)
        _num_field(errors, "bond", bd, "search_speed_mm_s", 1e-9, 20.0)
        _num_field(errors, "bond", bd, "touch_force_N", 1e-9, 100.0)
        _num_field(errors, "bond", bd, "search_overtravel_mm", 1e-9, 50.0)
        for k in ("wafer_preheat", "wafer_keep_warm", "head_cool_after"):
            if not isinstance(bd.get(k), bool):
                errors.append("bond.%s 须为布尔: %r" % (k, bd.get(k)))

    # ---- full_wafer（全片键合路径规划参数，2026-09-02 迭代 11） ----
    fw = recipe.get("full_wafer")
    if not isinstance(fw, dict):
        errors.append("full_wafer 缺字段或须为对象")
    else:
        _unknown_keys(errors, "full_wafer", fw,
                      ("count", "path_mode", "die_order", "skip_bonded",
                       "stop_on_fail", "pipelined"))
        _num_field(errors, "full_wafer", fw, "count", -1e-9, 100000.0,
                   integer=True)
        if fw.get("path_mode") not in ("center_out", "serpentine",
                                       "nearest"):
            errors.append("full_wafer.path_mode 非法: %r（合法：center_out/"
                          "serpentine/nearest）" % (fw.get("path_mode"),))
        if fw.get("die_order") not in ("nearest_pick", "serpentine"):
            errors.append("full_wafer.die_order 非法: %r（合法：nearest_pick/"
                          "serpentine）" % (fw.get("die_order"),))
        for k in ("skip_bonded", "stop_on_fail", "pipelined"):
            if not isinstance(fw.get(k), bool):
                errors.append("full_wafer.%s 须为布尔: %r" % (k, fw.get(k)))

    # ---- post_inspect（键合后精度检测参数，2026-09-02 迭代 11） ----
    pi = recipe.get("post_inspect")
    if not isinstance(pi, dict):
        errors.append("post_inspect 缺字段或须为对象")
    else:
        _unknown_keys(errors, "post_inspect", pi,
                      ("enabled", "auto_after_single", "auto_after_full",
                       "cool_target_C", "cool_rate_cps"))
        for k in ("enabled", "auto_after_single", "auto_after_full"):
            if not isinstance(pi.get(k), bool):
                errors.append("post_inspect.%s 须为布尔: %r" % (k, pi.get(k)))
        _num_field(errors, "post_inspect", pi, "cool_target_C", -1e-9, 300.0)
        _num_field(errors, "post_inspect", pi, "cool_rate_cps", 1e-9, 100.0)

    # ---- wafer_sites / film（布局双模式） ----
    _validate_wafer_sites(errors, recipe)
    f = recipe.get("film")
    if not isinstance(f, dict):
        errors.append("film 缺字段或须为对象")
    else:
        _unknown_keys(errors, "film", f, ("diameter_mm", "dies", "marks",
                                          "pin_overtravel_um"))
        _num_field(errors, "film", f, "diameter_mm", 0.0, 1000.0)
        _num_field(errors, "film", f, "pin_overtravel_um", 0.0, 500.0)
        _validate_dies(errors, recipe, f.get("dies"))
        fm = f.get("marks")
        if not isinstance(fm, dict):
            errors.append("film.marks 缺字段或须为对象")
        else:
            _unknown_keys(errors, "film.marks", fm,
                          ("shape", "size_um", "ratio", "per_die"))
            if fm.get("shape") not in MARK_SHAPES:
                errors.append("film.marks.shape 非法: %r（可选 %s）"
                              % (fm.get("shape"), "/".join(MARK_SHAPES)))
            _num_field(errors, "film.marks", fm, "size_um", 0.0, 5000.0)
            ratio = fm.get("ratio")
            if not (isinstance(ratio, list) and ratio):
                errors.append("film.marks.ratio 缺字段或须为非空数组")
            else:
                for i, pt in enumerate(ratio):
                    if not (isinstance(pt, (list, tuple)) and len(pt) == 2
                            and all(_is_num(v) for v in pt)):
                        errors.append("film.marks.ratio[%d] 须为 [x,y] 数值对"
                                      % i)
                    elif not all(-1.0 <= v <= 1.0 for v in pt):
                        errors.append("film.marks.ratio[%d] 元素须在 [-1,1]"
                                      "（相对 die 半宽比例）: %r" % (i, pt))
            per = fm.get("per_die")
            if not _is_int(per):
                errors.append("film.marks.per_die 须为整数: %r" % (per,))
            elif isinstance(ratio, list) and not (0 <= per <= len(ratio)):
                errors.append("film.marks.per_die 须在 [0, len(ratio)=%d]: %r"
                              % (len(ratio), per))
    return errors


def _migrate(recipe):
    """旧版兼容：
    - 无 mode 字段的布局块按 manual 迁移（旧 recipe.json 直接可读）；
    - wafer.marks[] 旧键 size_mm → size_um（×1000）；
    - film.die_mark{shape:"boxes",size_mm,corner_ratio} → film.marks
      {shape:"box", size_um, ratio=四角×corner_ratio, per_die:4}；
      film.marks 整个缺失时播种默认（向后兼容兜底）。"""
    f = recipe.get("film")
    if isinstance(f, dict) and isinstance(f.get("dies"), dict) \
            and "mode" not in f["dies"]:
        f["dies"]["mode"] = "manual"
    s = recipe.get("wafer_sites")
    if isinstance(s, dict) and "mode" not in s:
        s["mode"] = "manual"
    w = recipe.get("wafer")
    if isinstance(w, dict) and isinstance(w.get("marks"), list):
        for m in w["marks"]:
            if isinstance(m, dict) and "size_mm" in m and "size_um" not in m:
                m["size_um"] = round(float(m.pop("size_mm")) * 1000.0, 3)
    if isinstance(w, dict) and "site_marks" not in w:
        # 旧文件无 site_marks（2026-08-24 新增键）→ 播种默认
        w["site_marks"] = copy.deepcopy(DEFAULT_RECIPE["wafer"]["site_marks"])
    if "wafer_load" not in recipe:
        # 旧文件无上片误差注入块（2026-08-26 EGA 流程新增键）→ 播种默认
        recipe["wafer_load"] = copy.deepcopy(DEFAULT_RECIPE["wafer_load"])
    elif isinstance(recipe["wafer_load"], dict):
        # 旧文件缺后增键（2026-08-27 调焦调平：z/rx/ry 与 die_head_rx/ry）
        # → 仅补缺失，非法值仍由 validate 报错
        for k, v in DEFAULT_RECIPE["wafer_load"].items():
            recipe["wafer_load"].setdefault(k, v)
    if "sim_sensors" not in recipe:
        # 旧文件无传感器仿真块（2026-08-27 调焦调平迭代 2 新增键）→ 播种默认
        recipe["sim_sensors"] = copy.deepcopy(DEFAULT_RECIPE["sim_sensors"])
    elif isinstance(recipe["sim_sensors"], dict):
        ss = recipe["sim_sensors"]
        for k, v in DEFAULT_RECIPE["sim_sensors"].items():
            if k not in ss:
                ss[k] = copy.deepcopy(v)
                continue
            if not isinstance(v, dict):
                continue            # 标量键（force_noise_N）无嵌套迁移
            cur = ss[k]
            if isinstance(cur, dict):
                # 逐实例键缺失补种（2026-09-08 迭代 22 逐实例拆分），
                # 非法值仍由 validate 报错
                for inst, iv in v.items():
                    cur.setdefault(inst, iv)
            else:
                # 旧标量值 → 全实例同值展开（旧 recipe.json 直接可读）
                ss[k] = {inst: cur for inst in v}
    if "focus_level" not in recipe:
        # 旧文件无调焦调平流程参数块（2026-08-27 迭代 3 新增键）→ 播种默认
        recipe["focus_level"] = copy.deepcopy(DEFAULT_RECIPE["focus_level"])
    elif isinstance(recipe["focus_level"], dict):
        for k, v in DEFAULT_RECIPE["focus_level"].items():
            recipe["focus_level"].setdefault(k, v)
    if "fine_align" not in recipe:
        # 旧文件无精对准流程参数块（2026-08-28 迭代 6 新增键）→ 播种默认
        recipe["fine_align"] = copy.deepcopy(DEFAULT_RECIPE["fine_align"])
    elif isinstance(recipe["fine_align"], dict):
        for k, v in DEFAULT_RECIPE["fine_align"].items():
            recipe["fine_align"].setdefault(k, v)
    if "sim_align" not in recipe:
        # 旧文件无对准仿真注入块（2026-08-28 迭代 6 新增键）→ 播种默认
        recipe["sim_align"] = copy.deepcopy(DEFAULT_RECIPE["sim_align"])
    elif isinstance(recipe["sim_align"], dict):
        for k, v in DEFAULT_RECIPE["sim_align"].items():
            recipe["sim_align"].setdefault(k, v)
    if "bond" not in recipe:
        # 旧文件无热压键合流程参数块（2026-08-28 迭代 8 新增键）→ 播种默认
        recipe["bond"] = copy.deepcopy(DEFAULT_RECIPE["bond"])
    elif isinstance(recipe["bond"], dict):
        # 2026-08-31 迭代 9：approach_speed_mm_s 废除（两段速下压改造，
        # 拆为 fast_speed_mm_s/search_speed_mm_s）——旧键剔除防校验拒载
        recipe["bond"].pop("approach_speed_mm_s", None)
        for k, v in DEFAULT_RECIPE["bond"].items():
            recipe["bond"].setdefault(k, v)
    if "full_wafer" not in recipe:
        # 旧文件无全片键合路径规划块（2026-09-02 迭代 11 新增键）→ 播种默认
        recipe["full_wafer"] = copy.deepcopy(DEFAULT_RECIPE["full_wafer"])
    elif isinstance(recipe["full_wafer"], dict):
        for k, v in DEFAULT_RECIPE["full_wafer"].items():
            recipe["full_wafer"].setdefault(k, v)
    if "post_inspect" not in recipe:
        # 旧文件无键合后精度检测块（2026-09-02 迭代 11 新增键）→ 播种默认
        recipe["post_inspect"] = copy.deepcopy(DEFAULT_RECIPE["post_inspect"])
    elif isinstance(recipe["post_inspect"], dict):
        for k, v in DEFAULT_RECIPE["post_inspect"].items():
            recipe["post_inspect"].setdefault(k, v)
    if isinstance(f, dict):
        if "diameter_mm" not in f:
            # 旧文件无膜面直径（2026-08-24 圆形规则新增键）→ 播种默认；
            # 仅补缺失，非法值仍由 validate 报错
            f["diameter_mm"] = DEFAULT_RECIPE["film"]["diameter_mm"]
        if "pin_overtravel_um" not in f:
            # 旧文件无顶针过压量（2026-08-24 翻转取片工艺新增键）→ 播种默认
            f["pin_overtravel_um"] = \
                DEFAULT_RECIPE["film"]["pin_overtravel_um"]
        dm = f.pop("die_mark", None)
        if "marks" not in f:
            if isinstance(dm, dict) and _is_num(dm.get("corner_ratio")):
                cr = float(dm["corner_ratio"])
                f["marks"] = {
                    "shape": "box",
                    "size_um": round(float(dm.get("size_mm", 1.0)) * 1000.0, 3),
                    "ratio": [[-cr, -cr], [cr, -cr], [-cr, cr], [cr, cr]],
                    "per_die": 4}
            else:
                f["marks"] = copy.deepcopy(DEFAULT_RECIPE["film"]["marks"])
    return recipe


def _deep_merge(dst, src):
    """递归合并：dict 逐键合并，其余（含数组）整体替换。"""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


class Recipe(object):
    """处方运行时容器：启动加载（文件缺失播种默认；校验失败拒绝拉起），
    update() 部分合并 + 全量校验 + 原子写盘 + 热替换。"""

    def __init__(self, path):
        self.path = path
        self._lock = threading.RLock()
        if not os.path.exists(path):
            self._recipe = copy.deepcopy(DEFAULT_RECIPE)
            self._save_locked()
            print("[boot] recipe.json 不存在，已播种默认处方", flush=True)
        with open(self.path, encoding="utf-8") as f:
            self._recipe = _migrate(json.load(f))
        errors = validate(self._recipe)
        if errors:
            raise ValueError("recipe.json 校验失败（%d 条）：%s"
                             % (len(errors), "；".join(errors)))

    def get(self):
        """内存处方原样（不含派生块）。"""
        with self._lock:
            return copy.deepcopy(self._recipe)

    def public(self):
        """GET /api/recipe 响应用：注入派生块（只读，不落盘）。"""
        rec = self.get()
        rec["film"]["dies_derived"] = derive_dies(rec)
        rec["wafer_sites_derived"] = derive_wafer_sites(rec)
        return rec

    def update(self, patch):
        """部分合并在线改：合并→全量校验→通过才替换+落盘。返回 errors[]（空=成功）。"""
        if not isinstance(patch, dict):
            return ["请求体须为 JSON 对象（处方部分字段）"]
        patch = copy.deepcopy(patch)
        # 客户端若整份回读再回写，剥掉 GET 注入的派生块
        patch.pop("wafer_sites_derived", None)
        if isinstance(patch.get("film"), dict):
            patch["film"].pop("dies_derived", None)
        with self._lock:
            merged = copy.deepcopy(self._recipe)
            patch = self._apply_mode_switches(merged, patch)
            merged = _migrate(_deep_merge(merged, patch))
            errors = validate(merged)
            if errors:
                return errors
            self._recipe = merged
            self._save_locked()
            return []

    def _apply_mode_switches(self, merged, patch):
        """patch 切换布局 mode 时：目标模式模板（尽量保持几何不变）为底叠 patch
        后在 merged 上**整体替换**该块（深合并会残留另一模式字段被校验拦下）；
        返回扣除已处理块后的剩余 patch。"""
        cur = self._recipe
        f = patch.get("film")
        if isinstance(f, dict) and isinstance(f.get("dies"), dict):
            pd, cd = f["dies"], cur["film"]["dies"]
            if pd.get("mode") in LAYOUT_MODES and pd["mode"] != cd.get("mode"):
                if pd["mode"] == "manual":
                    dd = derive_dies(cur)
                    # 以膜面圆心为中心铺满同跨度矩形：圆裁剪在 manual 不可
                    # 表达，切 manual 四角空位回填（默认 45→49 颗）
                    base = {"mode": "manual", "rows": dd["rows"],
                            "cols": dd["cols"], "pitch_x_mm": dd["pitch_x_mm"],
                            "pitch_y_mm": dd["pitch_y_mm"],
                            "origin_x_mm": round(-(dd["cols"] - 1)
                                                 * dd["pitch_x_mm"] / 2.0, 6),
                            "origin_y_mm": round(-(dd["rows"] - 1)
                                                 * dd["pitch_y_mm"] / 2.0, 6),
                            "size_x_mm": cd["size_x_mm"],
                            "size_y_mm": cd["size_y_mm"],
                            "thickness_mm": cd.get("thickness_mm", 0.5)}
                else:                              # 切 auto 用默认模板，保留芯片尺寸
                    base = copy.deepcopy(DEFAULT_RECIPE["film"]["dies"])
                    for k in ("size_x_mm", "size_y_mm", "thickness_mm"):
                        if k in cd:
                            base[k] = cd[k]
                merged["film"]["dies"] = _deep_merge(base, pd)
                f.pop("dies")
                if not f:
                    patch.pop("film")
        s = patch.get("wafer_sites")
        if isinstance(s, dict) and s.get("mode") in LAYOUT_MODES \
                and s["mode"] != cur["wafer_sites"].get("mode"):
            if s["mode"] == "manual":
                sd = derive_wafer_sites(cur)       # 切 manual 保持居中铺满几何
                merged["wafer_sites"] = _deep_merge(
                    {"mode": "manual", "rows": sd["rows"], "cols": sd["cols"],
                     "pitch_x_mm": sd["pitch_x_mm"],
                     "pitch_y_mm": sd["pitch_y_mm"],
                     "origin_x_mm": round(-(sd["cols"] - 1)
                                          * sd["pitch_x_mm"] / 2.0, 6),
                     "origin_y_mm": round(-(sd["rows"] - 1)
                                          * sd["pitch_y_mm"] / 2.0, 6)}, s)
            else:
                merged["wafer_sites"] = _deep_merge(
                    copy.deepcopy(DEFAULT_RECIPE["wafer_sites"]), s)
            patch.pop("wafer_sites")
        return patch

    def restore_default(self):
        """恢复默认处方并落盘（不自动 apply 到物料层）。"""
        with self._lock:
            self._recipe = copy.deepcopy(DEFAULT_RECIPE)
            self._save_locked()
            return self.get()

    def _save_locked(self):
        """原子写盘：tmp + os.replace（同 transfers.json 写法）。"""
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self._recipe, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.path)
