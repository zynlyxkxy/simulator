# -*- coding: utf-8 -*-
"""metrology/stage_nonortho.py — 运动台 Y 导轨非正交角测校（迭代 37 重写：
栅格跳测法，routine as_stage_nonortho / ws_stage_nonortho，cam 恒 down）。

迭代 36 旧法（小步距单轴扫描 + 画幅内单标记 PCA 像素轨迹）被用户否——
步距太小（0.05~2mm）、且单标记漂移法原理上把链噪声全吃进来。新法基准
尺=晶圆 die 阵列：wafer_sites 格点 + 每 site 的 site_marks（quad，recipe
site_marks ratio×site 半宽，默认 10mm site → 4 角在 ±3.8mm）**假设严格
正交**，标记只存在于 site 格点（ pitch=处方 wafer_sites 节距，auto=
size+gap、manual=pitch_x/y_mm——start 校验与用户 pitch 一致，容差
0.001mm——仿真标记只在格点上，物理约束）。

测量流程（每 repeat，**十字跳测**）：
- 中心=center_xy（缺省=作业启动时两轴当前位，须视野内见标记——
  start 校验 _check_marks；**中心须对准某 quad 角**（每 node 的可见性
  靠"走距=格点节距 → 相对相位逐 node 复现"保证，故中心角对准后各
  node 同一相对角进出 1.8mm 画幅）；
- X 线：X 轴节点目标=中心+(i−(K−1)/2)·pitch，i=0..K−1 单调升序
  （**K 须奇数**——中心±(K−1)/2 步，X/Y 两线共中心=十字；升序=
  单侧逼近，消反向间隙友好；相邻节点走距仍恒=pitch，相位复现语义
  不变），Y 轴恒中心位。每点 identify
  fpp 帧、逐帧取**距画幅中心最近的有效峰**（score≥阈值；同 site 4 角
  互距 ≥5.2mm，默认 1.8mm FOV 内唯一；小 site 多角入幅时最近峰防串）
  再取均值 → (col,row)；像素→**世界偏移 o_i=标记世界位−相机世界位**
  （_context 链 solve_pixel，含 dir/微旋反投影；画幅中心即相机光轴，
  故 o_i 是标记相对光轴的世界偏移，WS 台动=标记动、AS 台动=相机反向动，
  相对运动两者同式）；
- 回中心；Y 线同（X 恒中心位）；
- o_i 对步号 i 最小二乘 → 斜率向量 s_a（mm/步）。**o 语义与斜率几何**
  （迭代 42b 核实）：o_i=标记世界位−相机世界位是**逐节点相对光轴
  偏移**（非绝对坐标、非相对名义节点偏移）；相邻节点观测标记换为
  相邻 site 同角（走距=节距相位复现），故斜率=**台步进向量−栅格
  步向量**（弦差，非台步进本身；台 sign σ=+1 ws/−1 as 并入 s'=σ·s）：
  X 线 s_x'=p·((1+εx)−cosρ, −sinρ)，Y 线 s_y'=p·((1+εy)sinφ_t+sinρ,
  (1+εy)cosφ_t−cosρ)（p=pitch，ρ=栅格安装旋转，εx/εy=轴比例误差
  （actual=指令×(1+ε)，仿真侧 error_system scale_ppm），ê_y=
  (sinφ_t,cosφ_t)=Y 导轨真值方向，φ_t 即 FK 注入）。分解一律走
  **横向/沿轨分量**（不用斜率方向/大小整体读出——沿轨比例项会混进
  方向读出：εx>100µrad 时旧方向读出把 ρ̂ 误判 π/2；且旧方向读出对
  真实栅格旋转带弦模型 ρ/2 隐性偏差——真机带旋转上片会报幻影 φ）：
  · 栅格安装旋转 ρ 由 X 线**横向分量**读出：s_x'·ŷ=−p·sinρ →
    ρ̂=−asin(clamp(s_x'·ŷ/p))（弦模型精确；|s_x'·ŷ|<_RHO_MIN_SIN·p
    噪声地板判 0——比例项在沿轨分量，不再触发误判）；
  · Y 线**横向分量**给 ψ_y：s_y'·x̂=p·((1+εy)sinφ_t+sinρ)≈p·sin(
    φ_t+ρ)（εy·sinφ_t 二阶可忽略）→ ψ_y=asin(clamp(s_y'·x̂/p))
    （符号自带，旧"大小+方向定号"逻辑退役）；
  · **非正交角（X 基准，与机器常数 as/ws_y_rail_dir_urad 同语义）
    φ=wrap(ψ_y−ρ̂)**——ρ̂ 在差中消去（栅格整体安装旋转不影响测量）；
  · **比例误差（迭代 42b 新增）**：步进向量 v_line=s'+p·R(ρ̂)e_line
    （加回栅格步，e_line=线名义方向 x̂/ŷ）→ ε_line=|v_line|/p−1
    （弦模型下 v_x=p(1+εx)x̂、v_y=p(1+εy)ê_y 精确）。ε 含相机像元
    当量误差别名（基准尺信任像元当量，真机需干涉仪/激光尺解耦）；
    阿贝/直线度等位置相关项落残差/LUT，不进比例；
- 报告角（**偏差口径**，GUI 显示 µrad 小数值）：θ_x=−ρ̂（X 轴实际
  方向 vs 栅格），θ_y=wrap(φ−ρ̂)（与 θ_x 对称的偏差角；锚定关系
  wrap(θ_y−θ_x)=φ 与机器常数语义一致）；ref_axis="x"（默认）
  报告 φ；ref_axis="y" 报告其负值（Y 基准）。repeat 轮取均值。

φ 符号锚定（i37 测试断言依据）：FK 真值注入 machine_constants stage_geo
as/ws_y_rail_dir_urad=φ_t（error_kinematics Y 链元平移真值偏差
y·(sinφ,cosφ−1)，迭代 36 同款）时，X 基准实测 φ≈+φ_t、Y 基准≈−φ_t。

时序/现场：cancel 检查点每 repeat 首；**进度逐节点推进**（total=
repeat×2×K，文本带 循环/线/节点）；finally 恢复轴位/光源/视觉模式
（与 rotate_center 同款）。结果：measured.phi_urad（=ref_axis 口径均值）
+ **<pfx>nonortho_urad（恒为 X 基准值——apply 写回 stage_geo 的键语义
不随 ref_axis 变）** + **matrix_l2p/matrix_p2l（物理↔逻辑双向 2×2
**全矩阵**（迭代 42b）：M_l2p=[col_x|col_y]，col_line=v̄_line/pitch
=逻辑基向量的实测像（v̄=σ 修正斜率 repeat 均值 + 加回栅格步
p·R(ρ̄)e_line）——列模长=ε（scale）、列方向=线方向角，天然含比例
与非正交；**X 线方向不再强制贴逻辑 X**（旧 [[1,sinφ],[0,cosφ]]=
本全矩阵 ε=1 且 col_x 横向置零的特例）；M_p2l=inv(M_l2p)（补偿前馈），
9 位有效数字；与参考脚本 grating_and_interferometer_parallel_NO_
error_PCA.py 正向/逆向矩阵同族（其 matrix[0][0]=a/[1][1]=d≡本矩阵
列模长口径））** + **scale_x/scale_y（ε=矩阵列模长，9 位有效数字
_sig9）与 scale_ppm_x/scale_ppm_y（(ε−1)×1e6，1 位小数）** + phi_std_urad/phi_3sigma_urad/theta_x_urad/
theta_y_urad（偏差口径 wrap(φ−ρ̂)）/resid_rms_nm/resid_max_nm（o_i 点到
o=ō+s·(i−ī) 拟合线的法向残差，o_i 世界 mm×1e6=nm）/series_phi_urad；
series 另增末循环逐节点 nodes_x/nodes_y（{i,pos,col,row,ox,oy,**mwx,mwy**——
mwx/mwy=mark_world 迭代 44 口径，迭代 45 增，供 GUI 画测量十字图}）与
fit_x/fit_y（{slope:[sx,sy],mean:[mx,my]}，供 GUI 画散点+拟合线）；
**迭代 45 规范对齐（《XY运动台非正交（垂直度）校准与补偿技术规范》
Rev2.0）**：measured 增 exy_urad（规范式 4 口径：EXY=γ−90°，夹角>90°
为正——=−X 基准 φ；矩阵层面 matrix_l2p≡规范 A、matrix_p2l≡C 完全一
致，仅标量角符号口径相反）/beta_urad（=ρ̄，规范上板倾斜 β 同义，9.1
报告要素）/exy_u_urad/exy_U_urad（附录 D：Sxx=d²K(K²−1)/12、u(E_XY)
=√2·u_meas/√Sxx、U=2u，u_meas=线法向残差 RMS 保守估计——含直线度
系统项）；detail 增 ac_check_norm（6.4 数值核验 ‖A·C−I‖ 最大元，双
精度应 <1e-12）；
detail 增 ref_axis/node_pitch_mm/slope_x_mm_per_step/slope_y_mm_per_step/
rho_urad/psi_y_urad/matrix_l2p/matrix_p2l/csv_nodes；CSV 落两份
（runtime/metrology_data/）：
stage_nonortho_<ws|as>_*.csv（每 repeat 一行；迭代 42b schema 增列：
repeat,phi_urad,theta_x_urad,theta_y_urad,resid_rms_nm,**scale_x,
scale_y,scale_ppm_x,scale_ppm_y**）+
stage_nonortho_<ws|as>_nodes_*.csv（**行基准全量记录**（2026-09-16
重构，给人看）：每行一个测量节点、X/Y 分列、单位进列名——
cycle,line,node,template,AS_X_mm,AS_Y_mm,WS_LS_X_mm,WS_LS_Y_mm
（四轴读数位，_context axes_pos 快照；ws 测校时 AS 轴位也记）,
mark_col_px,mark_row_px,score（fpp 帧选峰循环内最佳峰 score 均值）,
cam_world_x_mm,cam_world_y_mm,mark_world_x_mm,mark_world_y_mm
,o_x_mm,o_y_mm,resid_x_mm,resid_y_mm
（带符号 2D 残差 r_i=o_i−(ō+s·(i−ī))，mm）,**frame_cols_px,
frame_rows_px**（迭代 45：逐帧原始坐标，规范 5.3「单帧原始坐标应保
存，不得只保存平均值」；";" 连接 fpp 帧——21 列，前缀兼容））。**mark_world 口径
（迭代 44 修，ws/as 分列）**：ws 例行=**WS_LS 读数位+像素反算偏移
o**（台基准实测位置链——真机软件"编码器读数+视觉残差"日志同口径，
随台步进 ~pitch 走、偏差即步进误差，可供外部脚本直接复算矩阵；
相机固定+栅格跳测每节点观测**不同**标记，旧 cam_world+o 口径下
该列逐节点恒为常数、无分析价值，迭代 44 前数据作废）；as 例行=
cam_world+o（相机随对准台动，cam_world 即对准台坐标+锚点常数推
出——"对准台结合像素"语义，天然逐节点变化，维持不动）。两口径
的像素反算都含 rot_mrad 控制器认知+dir/swap（solve_rel 链）。

参数校验（calibration._validate_stage_nonortho，i37 重写）：template_name
必填已注册 + ref_axis x/y + k_points **奇数** 3,5,…,21（偶数→400 明示
"十字对称跳测须奇数 K（中心±(K−1)/2 步）"）+ pitch_mm 0.1~50（默认=
处方 pitch，X/Y 都校、不一致 400 报实际值）+ frames_per_point 1~10 +
repeat 2~10 + wait_ms + center_xy 可选逐轴软限位 + 行程守卫（十字对称：
X/Y 线各自 center±(K−1)/2·pitch 两端都在走位轴软限位内）+ 流程忙守卫
+ **视野无标记 start 即 400**（_check_marks 上移 start）。i36 的 FOV
跨距/模板匹配窗口守卫已删（标记不再漂移出画幅——走距=格点节距）。

σ（台 sign）推导备忘：o_i=标记−相机；WS 台带标记动（σ=+1）、AS 台带
相机动（σ=−1），两式相差负号；斜率先乘 σ 再提取 ρ̂/ψ_y 即两台同式。
"""
import math
import os
import sys
import time

# 同 rotate_center.py 的导入定位方式（包内相对导入兜底）
try:
    from repeatability import (_check_cancel, _check_marks, _log,
                               _mean_std, _progress, _sleep_ms)
    from extra_routines import _context, _csv_write
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from repeatability import (_check_cancel, _check_marks, _log,
                               _mean_std, _progress, _sleep_ms)
    from extra_routines import _context, _csv_write

_EPS_SLOPE = 1e-9          # 斜率退化阈值（mm/步；ρ=0 理想栅格 X 线斜率恒 0）
# ρ̂ 横向分量读出的噪声门限：|s_x'·ŷ|/pitch < _RHO_MIN_SIN 时横向分量处于
# 噪声地板（理想栅格/显式 offset 上片 ρ=0——实测地图噪声 ~0.2µm/步 @20mm
# pitch；轴比例误差在沿轨分量，不进横向、不触发本门限），判 ρ=0。真实
# 上片旋转（wafer_load rz=250µrad → 5µm/步 @20mm）远在门限之上，横向
# 读出有效（弦模型精确，无旧方向读出的 ρ/2 偏差）。默认 1e-4（100µrad）。
_RHO_MIN_SIN = 1e-4


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _sig9(v):
    """9 位有效数字舍入（物理↔逻辑矩阵输出契约，GUI 消费 2×2 嵌套数组）。"""
    return float("%.9g" % v)


def _fmt(v, spec):
    """None 安全格式化（_context 子项失败置 None 时 CSV 留空不阻塞）。"""
    return (spec % v) if v is not None else ""


def _fit_slope(points2d):
    """o_i 序列 [(ox,oy)×K] 对步号 i 的最小二乘：o = ō + s·(i−ī)。
    返回 (slope[2], mean[2], resid_norm[n])——resid=各点到拟合直线
    o=ō+s·t 的法向距离 |cross(o_i−ō, ŝ)|（mm，调用方 ×1e6 得 nm）。"""
    n = len(points2d)
    mi = (n - 1) / 2.0
    sxx = sum((i - mi) ** 2 for i in range(n))
    mx = sum(p[0] for p in points2d) / n
    my = sum(p[1] for p in points2d) / n
    sxy_x = sum((i - mi) * (p[0] - mx) for i, p in enumerate(points2d))
    sxy_y = sum((i - mi) * (p[1] - my) for i, p in enumerate(points2d))
    sx, sy = sxy_x / sxx, sxy_y / sxx
    norm = math.hypot(sx, sy)
    if norm < _EPS_SLOPE:
        sn = (0.0, 0.0)
    else:
        sn = (sx / norm, sy / norm)
    resid = [abs((p[0] - mx) * sn[1] - (p[1] - my) * sn[0])
             for p in points2d]
    return (sx, sy), (mx, my), resid


def fit_nonortho(ox_seq, oy_seq, pitch, sigma=1.0):
    """单循环核心拟合（**纯函数，可独立调用**——Temp/_i42b_algebra.py
    代数自检直接 import）：两线 o_i 序列（各 [(ox,oy)×K]，o=标记世界位
    −相机世界位=**逐节点相对光轴偏移**（非绝对坐标、非相对名义节点
    偏移——相邻节点观测标记换为相邻 site 同角，o 序列斜率=台步进向量
    −栅格步向量的弦差）+ 名义 pitch（mm）+ 台 sign σ（ws=+1/as=−1）
    → dict。

    分解口径（弦模型精确；迭代 42b 起一律横向/沿轨分量，不用斜率方向/
    大小整体读出——沿轨比例项会污染方向读出，εx>100µrad 时旧方向读出
    把 ρ̂ 误判 π/2）：s'=σ·slope；X 线 s_x'=p((1+εx)−cosρ, −sinρ)、
    Y 线 s_y'=p((1+εy)sinφ_t+sinρ, (1+εy)cosφ_t−cosρ)：
    · rho=−asin(clamp(s_x'·ŷ/p))（X 线横向分量；|s_x'·ŷ|<_RHO_MIN_SIN·p
      噪声地板判 0——比例项在沿轨分量，不触发误判）；
    · psi_y=asin(clamp(s_y'·x̂/p))（Y 线横向分量≈sin(φ_t+ρ)，符号自带，
      εy·sinφ_t 二阶可忽略）；phi=wrap(ψ_y−ρ̂)（X 基准）；theta_x=−ρ̂、
      theta_y=wrap(φ−ρ̂)（偏差口径，锚定 wrap(θ_y−θ_x)=φ）；
    · 步进向量 v=s'+p·R(ρ̂)e_line（加回栅格步；e_x=x̂、e_y=ŷ）→
      eps=|v|/p−1（弦模型下 v_x=p(1+εx)x̂、v_y=p(1+εy)ê_y 精确；ε 含
      相机像元当量误差别名——基准尺信任像元当量，真机需干涉仪/激光尺
      解耦；阿贝/直线度等位置相关项落残差/LUT，不进比例）。
    返回 {slope_x/slope_y（原始斜率）、mean_x/mean_y、resid_x/resid_y
    （法向残差 mm 列表）、sx_/sy_（σ 修正后斜率）、rho/psi_y/phi/
    theta_x/theta_y（rad）、eps_x/eps_y、v_x/v_y（mm/步）}。"""
    sx, mx, res_x = _fit_slope(ox_seq)
    sy, my, res_y = _fit_slope(oy_seq)
    sxx_, sxy_ = sigma * sx[0], sigma * sx[1]
    syx_, syy_ = sigma * sy[0], sigma * sy[1]
    # ρ̂：X 线横向分量读出（s_x'·ŷ=−p·sinρ，弦模型精确；噪声地板判 0）
    if abs(sxy_) < _RHO_MIN_SIN * pitch:
        rho = 0.0
    else:
        rho = -math.asin(min(max(sxy_ / pitch, -1.0), 1.0))
    # ψ_y：Y 线横向分量读出（s_y'·x̂≈p·sin(φ_t+ρ)，符号自带）
    psi_y = math.asin(min(max(syx_ / pitch, -1.0), 1.0))
    phi_x = _wrap_pi(psi_y - rho)              # X 基准非正交角
    _cr, _sr = math.cos(rho), math.sin(rho)
    v_x = (sxx_ + pitch * _cr, sxy_ + pitch * _sr)   # 加回栅格步 p·R(ρ̂)x̂
    v_y = (syx_ - pitch * _sr, syy_ + pitch * _cr)   # 加回栅格步 p·R(ρ̂)ŷ
    return {"slope_x": sx, "slope_y": sy, "mean_x": mx, "mean_y": my,
            "resid_x": res_x, "resid_y": res_y,
            "sx_": (sxx_, sxy_), "sy_": (syx_, syy_),
            "rho": rho, "psi_y": psi_y, "phi": phi_x,
            "theta_x": -rho, "theta_y": _wrap_pi(phi_x - rho),
            "eps_x": math.hypot(*v_x) / pitch - 1.0,
            "eps_y": math.hypot(*v_y) / pitch - 1.0,
            "v_x": v_x, "v_y": v_y}


def _repeat_mat_cols(v_x, v_y, pitch):
    """逐轮矩阵列（汇总 CSV 用）：该轮步进向量 v_x/v_y（mm/步）→
    M_l2p=[v_x/p|v_y/p]（逻辑基向量实测像）与 M_p2l=inv，按行主序展开为
    8 个 9 位有效数字字符串 [l2p_00,01,10,11, p2l_00,01,10,11]。
    与作业级矩阵（均值斜率+R(ρ̄) 构建，measured.matrix_l2p）在 ρ̂ 逐轮
    抖动时仅二阶近似一致——作业级矩阵为准；det 退化兜底同主链。"""
    cx = (v_x[0] / pitch, v_x[1] / pitch)
    cy = (v_y[0] / pitch, v_y[1] / pitch)
    det = cx[0] * cy[1] - cy[0] * cx[1]
    if abs(det) > 1e-12:
        inv = ((cy[1] / det, -cy[0] / det), (-cx[1] / det, cx[0] / det))
    else:
        inv = ((1.0, 0.0), (0.0, 1.0))
    return ["%.9g" % v for v in
            (cx[0], cy[0], cx[1], cy[1],
             inv[0][0], inv[0][1], inv[1][0], inv[1][1])]


def run_stage_nonortho(svc, routine, cam_id, axes_pair, params, job=None):
    """Y 导轨非正交角测校主流程（迭代 37 栅格跳测法，十字对称走法）。参数
    校验已在 calibration._validate_stage_nonortho 完成（模板/ref_axis/K
    奇数/pitch/fpp/repeat/wait_ms/center_xy/对称行程/忙/视野标记）。现场
    自管：视觉模式/光源/轴位 finally 恢复。

    routine：ws_stage_nonortho（σ=+1，硅片台带标记动）/ as_stage_nonortho
    （σ=−1，对准台带相机动）；axes_pair=(X 轴, Y 轴)；params 规范化键：
    template_name/ref_axis/k_points/pitch_mm/frames_per_point/repeat/
    wait_ms/center_xy。"""
    from calibration import CalibError        # 懒导入避免模块环
    t0 = time.time()
    eng = svc.engine
    cam = svc.vision.cams[cam_id].model
    rt = svc.vision.cams[cam_id]
    ax_x, ax_y = axes_pair
    sigma = 1.0 if routine == "ws_stage_nonortho" else -1.0   # 台 sign
    ref_axis = params.get("ref_axis", "x")
    k_points = int(params["k_points"])
    pitch = float(params["pitch_mm"])
    fpp = int(params["frames_per_point"])
    repeat = int(params["repeat"])
    wait_ms = params["wait_ms"]
    tpl = params["template_name"]
    center = [float(params["center_xy"][0]), float(params["center_xy"][1])]
    pfx = "ws_" if routine == "ws_stage_nonortho" else "as_"
    old_mode = svc.vision.mode
    old_light = bool(svc.points.get(cam.light_point, False))
    saved = {}
    with eng.lock:
        for a in axes_pair:
            saved[a] = eng.axes[a].pos
    _check_marks(svc, cam_id, cam)            # 运行期兜底（start 已 400 拦）
    _log(svc, job, "stage nonortho(raster): routine=%s cam=%s template=%s "
         "axes=%s ref=%s K=%d pitch=%.3fmm fpp=%d repeat=%d wait_ms=%s "
         "center=(%.4f,%.4f) sigma=%+.0f"
         % (routine, cam_id, tpl, "/".join(axes_pair), ref_axis, k_points,
            pitch, fpp, repeat, wait_ms, center[0], center[1], sigma))

    def _snap_node(phase, node):
        """当前节点 fpp 帧 identify：逐帧取距画幅中心最近的有效峰，
        返回 ((col,row) 均值, (ox,oy) 世界偏移, info)——info={score
        （选峰循环内最佳峰 score 的 fpp 帧均值）, cam_world/mark_world
        （_context 前两分量；像素反算经 solve_pixel 含 rot_mrad 控制器
        认知+dir/swap。**mark_world 口径迭代 44 起 ws/as 分列**：ws=
        WS_LS 读数位+o（台基准实测位置链，随台步进；旧 cam_world+o 在
        相机固定+跳测换标记下恒为常数无意义）；as=cam_world+o（相机随
        对准台动，即对准台坐标结合像素））, axes_pos（四轴读数位
        快照，ws 测校时 AS 轴位也记）, frames（**逐帧原始 (col,row)
        列表**——规范 5.3「单帧原始坐标应保存，不得只保存平均值」，迭代
        45 起 nodes CSV 追加 frame_cols_px/frame_rows_px 两列）}，
        供行基准过程数据 CSV。"""
        cc = rc = sc = 0.0
        fcols, frows = [], []
        for _f in range(fpp):
            res = svc._identify(cam_id, tpl)["result"]
            cols = res.get("lstCol") or []
            rows = res.get("lstRow") or []
            scores = res.get("lstScore") or []
            best, bd = None, None
            for c, r, s in zip(cols, rows, scores):
                if s is None or s < cam.score_threshold:
                    continue
                d = (c - cam.px_x / 2.0) ** 2 + (r - cam.px_y / 2.0) ** 2
                if bd is None or d < bd:
                    best, bd = (c, r, s), d
            if best is None:
                raise CalibError(
                    "%s 节点#%d：视野内未找到标记（得分均低于 %.2f）"
                    % (phase, node + 1, cam.score_threshold))
            cc += best[0]
            rc += best[1]
            sc += best[2]
            fcols.append(best[0])
            frows.append(best[1])
        col, row = cc / fpp, rc / fpp
        ctx = _context(svc, cam_id, cam, col, row)
        mw = ctx.get("mark_world_mm")
        cw = ctx.get("cam_world_mm")
        if not mw or not cw:
            raise CalibError("%s 节点#%d：世界坐标反算失败（o_i 不可得）"
                             % (phase, node + 1))
        o = (mw[0] - cw[0], mw[1] - cw[1])
        # mark_world 记录口径（迭代 44）：ws=台读数+o（台基准实测位置链，
        # 随台步进 ~pitch）；as=相机链 cam_world+o（相机随台动已含台位）。
        # axes_pos 取数失败兜底保持相机链原值，不阻塞测量。
        _ap0 = ctx.get("axes_pos") or {}
        if (sigma > 0 and _ap0.get("WS_LS_X") is not None
                and _ap0.get("WS_LS_Y") is not None):
            mw_rec = (_ap0["WS_LS_X"] + o[0], _ap0["WS_LS_Y"] + o[1])
        else:
            mw_rec = (mw[0], mw[1])
        info = {"score": sc / fpp,
                "cam_world": (cw[0], cw[1]),
                "mark_world": mw_rec,
                "axes_pos": ctx.get("axes_pos") or {},
                "frames": (fcols, frows)}
        return (col, row), o, info

    def _scan_line(axis, phase, cycle):
        """单轴十字跳测 K 节点：目标=中心+(i−(K−1)/2)·pitch，i=0..K−1
        单调升序（单侧逼近，消反向间隙友好；相邻节点走距恒=pitch，相位
        复现语义不变），另一轴恒中心位；走完回中心。返回逐节点列表
        [(i, 轴位, (col,row), (ox,oy), info)]；每节点完成推一次进度。"""
        idx = 0 if axis == ax_x else 1
        half = (k_points - 1) / 2.0
        nodes = []
        for i in range(k_points):
            tgt = center[idx] + (i - half) * pitch
            svc._move_and_wait(axis, tgt)
            _sleep_ms(wait_ms)
            px, o, info = _snap_node(phase, i)
            nodes.append((i, tgt, px, o, info))
            prog[0] += 1
            _progress(job, prog[0], total,
                      "第 %d/%d 循环 %s 节点 %d/%d"
                      % (cycle, repeat, phase, i + 1, k_points))
            _log(svc, job, "%s 节点#%d/%d %s=%.4f: o=(%.6f,%.6f)mm px=(%.1f,"
                 "%.1f)" % (phase, i + 1, k_points, axis, tgt, o[0], o[1],
                            px[0], px[1]))
        svc._move_and_wait(axis, center[idx])          # 回中心
        _sleep_ms(wait_ms)
        return nodes

    phis = []                  # 报告口径（随 ref_axis）
    phis_x = []                # X 基准口径（<pfx>nonortho_urad 恒用）
    theta_xs, theta_ys = [], []
    rhos, psis = [], []
    sxs_c, sys_c = [], []      # σ 修正后逐循环斜率（矩阵列/scale 均值用）
    slope_x_last = [0.0, 0.0]
    slope_y_last = [0.0, 0.0]
    resid_all_nm = []
    resid_max_nm = 0.0
    rows = []
    node_rows = []             # 节点级过程数据（第二份 CSV）
    total = repeat * 2 * k_points      # 进度总量（每节点 1 步）
    prog = [0]                 # 已完成节点计数（_scan_line 闭包自增）
    # 末循环逐节点散点 + 拟合线（result.series，供 GUI 画散点+拟合线）
    series_nodes = {"nodes_x": [], "nodes_y": []}
    series_fits = {"fit_x": {"slope": [0.0, 0.0], "mean": [0.0, 0.0]},
                   "fit_y": {"slope": [0.0, 0.0], "mean": [0.0, 0.0]}}
    try:
        if old_mode != "real":
            svc.vision.set_mode("real")
        svc.vision._set_light(rt, True)
        svc.vision._await_light(rt, True)
        for i in range(repeat):
            _check_cancel(job, i + 1)
            svc._move_and_wait(ax_y, center[1])
            svc._move_and_wait(ax_x, center[0])
            _sleep_ms(wait_ms)
            nodes_x = _scan_line(ax_x, "X 线", i + 1)
            nodes_y = _scan_line(ax_y, "Y 线", i + 1)
            ox_seq = [nd[3] for nd in nodes_x]
            oy_seq = [nd[3] for nd in nodes_y]
            fit = fit_nonortho(ox_seq, oy_seq, pitch, sigma)
            sx, sy = fit["slope_x"], fit["slope_y"]
            mx, my = fit["mean_x"], fit["mean_y"]
            res_x, res_y = fit["resid_x"], fit["resid_y"]
            slope_x_last, slope_y_last = list(sx), list(sy)
            rho, psi_y, phi_x = fit["rho"], fit["psi_y"], fit["phi"]
            theta_x, theta_y = fit["theta_x"], fit["theta_y"]
            eps_x, eps_y = fit["eps_x"], fit["eps_y"]
            sxs_c.append(fit["sx_"])     # σ 修正斜率（矩阵列/scale 均值用）
            sys_c.append(fit["sy_"])
            # 偏差口径（GUI 显示 µrad 小数值，与 θ_x=−ρ̂ 对称）：
            # θ_y=wrap(φ−ρ̂)；锚定关系 wrap(θ_y−θ_x)=φ 与机器常数同语义
            phi_rep = _wrap_pi(theta_y - theta_x) \
                if ref_axis == "x" else _wrap_pi(theta_x - theta_y)
            phis.append(phi_rep)
            phis_x.append(phi_x)
            theta_xs.append(theta_x)
            theta_ys.append(theta_y)
            rhos.append(rho)
            psis.append(psi_y)
            res_nm = [v * 1e6 for v in (res_x + res_y)]
            rms_nm = math.sqrt(sum(v * v for v in res_nm) / len(res_nm))
            rmax_nm = max(abs(v) for v in res_nm)
            resid_all_nm.extend(res_nm)
            resid_max_nm = max(resid_max_nm, rmax_nm)
            # 节点级过程数据（行基准全量记录：每行一个测量节点、X/Y 分列、
            # 单位进列名；四轴读数位取 _context axes_pos 快照——ws 测校时
            # AS 轴位也记）：带符号 2D 残差 r_i=o_i−(ō+s·(i−ī))（mm）
            mi = (k_points - 1) / 2.0
            for _ln, _nds, _s, _m in (
                    ("X", nodes_x, sx, mx),
                    ("Y", nodes_y, sy, my)):
                for nd in _nds:
                    rxi = nd[3][0] - (_m[0] + _s[0] * (nd[0] - mi))
                    ryi = nd[3][1] - (_m[1] + _s[1] * (nd[0] - mi))
                    _if = nd[4]
                    _ap = _if.get("axes_pos") or {}
                    _cw = _if.get("cam_world") or (None, None)
                    _mw = _if.get("mark_world") or (None, None)
                    _fc, _fr = _if.get("frames") or ([], [])
                    node_rows.append([
                        i + 1, _ln, nd[0], tpl,
                        _fmt(_ap.get("AS_X"), "%.7f"),
                        _fmt(_ap.get("AS_Y"), "%.7f"),
                        _fmt(_ap.get("WS_LS_X"), "%.7f"),
                        _fmt(_ap.get("WS_LS_Y"), "%.7f"),
                        "%.3f" % nd[2][0], "%.3f" % nd[2][1],
                        _fmt(_if.get("score"), "%.4f"),
                        _fmt(_cw[0], "%.6f"), _fmt(_cw[1], "%.6f"),
                        _fmt(_mw[0], "%.6f"), _fmt(_mw[1], "%.6f"),
                        "%.9f" % nd[3][0], "%.9f" % nd[3][1],
                        "%.9f" % rxi, "%.9f" % ryi,
                        ";".join("%.3f" % v for v in _fc),
                        ";".join("%.3f" % v for v in _fr)])
            if i == repeat - 1:      # 末循环：GUI 散点 + 拟合线
                def _nd_dict(nd):
                    _mw2 = nd[4].get("mark_world") or (None, None)
                    return {"i": nd[0], "pos": round(nd[1], 6),
                            "col": round(nd[2][0], 3),
                            "row": round(nd[2][1], 3),
                            "ox": nd[3][0], "oy": nd[3][1],
                            "mwx": None if _mw2[0] is None
                            else round(_mw2[0], 6),
                            "mwy": None if _mw2[1] is None
                            else round(_mw2[1], 6)}
                series_nodes["nodes_x"] = [_nd_dict(nd) for nd in nodes_x]
                series_nodes["nodes_y"] = [_nd_dict(nd) for nd in nodes_y]
                series_fits["fit_x"] = {"slope": list(sx), "mean": list(mx)}
                series_fits["fit_y"] = {"slope": list(sy), "mean": list(my)}
            rows.append([i, "%.4f" % (phi_rep * 1e6),
                         "%.4f" % (theta_x * 1e6),
                         "%.4f" % (theta_y * 1e6), "%.4f" % rms_nm,
                         "%.9g" % (1.0 + eps_x), "%.9g" % (1.0 + eps_y),
                         "%.1f" % (eps_x * 1e6), "%.1f" % (eps_y * 1e6)]
                        + _repeat_mat_cols(fit["v_x"], fit["v_y"], pitch))
            _log(svc, job, "cycle %d: phi(%s)=%.3furad phi_x=%.3furad "
                 "rho=%.3furad psi_y=%.3furad resid_rms=%.1fnm "
                 "scale=(%+.1f,%+.1f)ppm"
                 % (i + 1, ref_axis, phi_rep * 1e6, phi_x * 1e6,
                    rho * 1e6, psi_y * 1e6, rms_nm,
                    eps_x * 1e6, eps_y * 1e6))
            _log(svc, job, "cycle time: %d / %d" % (i + 1, repeat))
    finally:
        for a in axes_pair:
            try:
                svc._move_and_wait(a, saved[a])
            except Exception:
                pass
        try:
            svc.vision._set_light(rt, old_light)
        except Exception:
            pass
        if old_mode != "real":
            try:
                svc.vision.set_mode(old_mode)
            except Exception:
                pass
    fname = _csv_write("stage_nonortho_%s" % pfx.rstrip("_"),
                       ["repeat", "phi_urad", "theta_x_urad", "theta_y_urad",
                        "resid_rms_nm", "scale_x", "scale_y",
                        "scale_ppm_x", "scale_ppm_y",
                        # 迭代 48：逐轮物理↔逻辑双向矩阵（行主序；
                        # _repeat_mat_cols 口径——作业级矩阵以 measured
                        # .matrix_l2p/p2l 为准，本列为逐轮实测像）
                        "l2p_00", "l2p_01", "l2p_10", "l2p_11",
                        "p2l_00", "p2l_01", "p2l_10", "p2l_11"], rows)
    _log(svc, job, "data saved: %s" % fname)
    fname_nd = _csv_write("stage_nonortho_%s_nodes" % pfx.rstrip("_"),
                          ["cycle", "line", "node", "template",
                           "AS_X_mm", "AS_Y_mm", "WS_LS_X_mm", "WS_LS_Y_mm",
                           "mark_col_px", "mark_row_px", "score",
                           "cam_world_x_mm", "cam_world_y_mm",
                           "mark_world_x_mm", "mark_world_y_mm",
                           "o_x_mm", "o_y_mm",
                           "resid_x_mm", "resid_y_mm",
                           # 迭代 45：逐帧原始坐标（规范 5.3「单帧原始坐标
                           # 应保存，不得只保存平均值」；";" 连接 fpp 帧）
                           "frame_cols_px", "frame_rows_px"], node_rows)
    _log(svc, job, "node data saved: %s" % fname_nd)
    if not phis:
        raise CalibError("栅格跳测无有效轮次")
    m_phi, s_phi = _mean_std(phis)
    m_x, _s = _mean_std(phis_x)
    m_tx, _s = _mean_std(theta_xs)
    m_ty, _s = _mean_std(theta_ys)
    m_rho, _s = _mean_std(rhos)
    m_psi, _s = _mean_std(psis)
    # 物理↔逻辑双向全矩阵（迭代 42b；恒 X 基准值，不随 ref_axis 变；9 位
    # 有效数字）：M_l2p=[col_x|col_y]，col_line=v̄_line/pitch（v̄=σ 修正
    # 斜率 repeat 均值 + 加回栅格步 p·R(ρ̄)e_line）=逻辑基向量的实测像
    # ——列模长=ε（scale_x/scale_y 同源）、列方向=线方向角，X 线方向不再
    # 强制贴逻辑 X（旧 [[1,sinφ],[0,cosφ]]=ε=1 且 col_x 横向置零的特例）；
    # M_p2l=inv(M_l2p)（补偿前馈）——与参考脚本 grating_and_interferometer_
    # parallel_NO_error_PCA.py 正向/逆向矩阵同族（其 matrix[0][0]=a/
    # [1][1]=d≡本矩阵列模长口径）
    m_sxx = sum(v[0] for v in sxs_c) / len(sxs_c)
    m_sxy = sum(v[1] for v in sxs_c) / len(sxs_c)
    m_syx = sum(v[0] for v in sys_c) / len(sys_c)
    m_syy = sum(v[1] for v in sys_c) / len(sys_c)
    _cr, _sr = math.cos(m_rho), math.sin(m_rho)
    col_x = ((m_sxx + pitch * _cr) / pitch, (m_sxy + pitch * _sr) / pitch)
    col_y = ((m_syx - pitch * _sr) / pitch, (m_syy + pitch * _cr) / pitch)
    sc_x = math.hypot(*col_x)      # εx=列模长（含相机像元当量误差别名）
    sc_y = math.hypot(*col_y)      # εy
    mat_l2p = [[_sig9(col_x[0]), _sig9(col_y[0])],
               [_sig9(col_x[1]), _sig9(col_y[1])]]
    _det = col_x[0] * col_y[1] - col_y[0] * col_x[1]
    if abs(_det) > 1e-12:
        mat_p2l = [[_sig9(col_y[1] / _det), _sig9(-col_y[0] / _det)],
                   [_sig9(-col_x[1] / _det), _sig9(col_x[0] / _det)]]
        # 规范 6.4 数值核验：‖A·C−I‖ 最大元（双精度 2×2 应 <1e-12；
        # 用未舍入原值计算）
        _c = ((col_y[1] / _det, -col_y[0] / _det),
              (-col_x[1] / _det, col_x[0] / _det))
        _a = ((col_x[0], col_y[0]), (col_x[1], col_y[1]))
        ac_check = _sig9(max(
            abs(_a[0][0] * _c[0][0] + _a[0][1] * _c[1][0] - 1.0),
            abs(_a[0][0] * _c[0][1] + _a[0][1] * _c[1][1]),
            abs(_a[1][0] * _c[0][0] + _a[1][1] * _c[1][0]),
            abs(_a[1][0] * _c[0][1] + _a[1][1] * _c[1][1] - 1.0)))
    else:                          # 退化兜底（物理上 det≈1，防御）
        mat_p2l = [[1.0, 0.0], [0.0, 1.0]]
        ac_check = None
    rms_nm = math.sqrt(sum(v * v for v in resid_all_nm) / len(resid_all_nm)) \
        if resid_all_nm else 0.0
    # 规范附录 D 不确定度（k=2）：u_meas 用线法向残差 RMS 估计（含直线度
    # 系统项，偏保守）；Sxx=d²·K(K²−1)/12（式 D.2）；u(E_XY)=√2·u_meas/
    # √Sxx（式 D.4）；U=2·u（式 D.6）
    if rms_nm > 0 and k_points >= 3:
        _u_meas_mm = rms_nm * 1e-6
        _sxx = pitch * pitch * k_points * (k_points * k_points - 1) / 12.0
        _u_exy = math.sqrt(2.0) * _u_meas_mm / math.sqrt(_sxx)
        exy_u_urad = round(_u_exy * 1e6, 3)
        exy_U_urad = round(2.0 * _u_exy * 1e6, 3)
    else:
        exy_u_urad = exy_U_urad = None
    rep_mean_urad = round(m_phi * 1e6, 3)
    try:
        geo = (eng.constants.get() or {}).get("stage_geo") or {}
        cur = {"%sy_rail_dir_urad" % pfx:
               float(geo.get("%sy_rail_dir_urad" % pfx) or 0.0),
               "%snonortho_urad" % pfx:
               float(geo.get("%snonortho_urad" % pfx) or 0.0)}
    except Exception:
        cur = {"%sy_rail_dir_urad" % pfx: 0.0, "%snonortho_urad" % pfx: 0.0}
    return {
        "routine": routine, "camera": cam_id,
        "repeat": repeat, "wait_ms": wait_ms,
        "measured": {
            "phi_urad": rep_mean_urad,
            # 组路由键：恒为 X 基准值（apply 写回 stage_geo 的语义键，
            # 不随 ref_axis 变）
            "%snonortho_urad" % pfx: round(m_x * 1e6, 3),
            # 物理↔逻辑双向矩阵（恒 X 基准值，不随 ref_axis 变；9 位有效
            # 数字，GUI 消费 2×2 嵌套数组）
            "matrix_l2p": mat_l2p,
            "matrix_p2l": mat_p2l,
            # 比例误差（迭代 42b）：ε=矩阵列模长（9 位有效数字）与 ppm
            # （1 位小数）；ε 含相机像元当量误差别名（真机需干涉仪/激光尺
            # 解耦）；阿贝/直线度位置相关项在残差/LUT，不进比例
            "scale_x": _sig9(sc_x),
            "scale_y": _sig9(sc_y),
            "scale_ppm_x": round((sc_x - 1.0) * 1e6, 1),
            "scale_ppm_y": round((sc_y - 1.0) * 1e6, 1),
            "phi_std_urad": round(s_phi * 1e6, 3),
            "phi_3sigma_urad": round(3.0 * s_phi * 1e6, 3),
            "theta_x_urad": round(m_tx * 1e6, 3),
            "theta_y_urad": round(m_ty * 1e6, 3),
            # 迭代 45 规范对齐（《XY运动台非正交（垂直度）校准与补偿技术
            # 规范》Rev2.0）：exy_urad=规范口径垂直度误差（式 4：EXY=γ−
            # 90°，两轴夹角>90°为正）=−X 基准 φ（矩阵层面 matrix_l2p≡
            # 规范 A、matrix_p2l≡C，仅标量角符号口径相反）；beta_urad=
            # 规范上板倾斜 β 同义量（=ρ̄ 栅格安装旋转，9.1 报告要素）；
            # exy_u/exy_U=附录 D 标准/扩展（k=2）不确定度（u_meas 用线
            # 法向残差 RMS 估计，含直线度系统项，偏保守）
            "exy_urad": round(-m_x * 1e6, 3),
            "beta_urad": round(m_rho * 1e6, 3),
            "exy_u_urad": exy_u_urad,
            "exy_U_urad": exy_U_urad,
            "resid_rms_nm": round(rms_nm, 3),
            "resid_max_nm": round(resid_max_nm, 3),
            "series_phi_urad": [round(v * 1e6, 3) for v in phis]},
        "current": cur,
        "series": {"phi_urad": [round(v * 1e6, 3) for v in phis],
                   "phi_x_urad": [round(v * 1e6, 3) for v in phis_x],
                   "theta_x_urad": [round(v * 1e6, 3) for v in theta_xs],
                   "theta_y_urad": [round(v * 1e6, 3) for v in theta_ys],
                   "nodes_x": series_nodes["nodes_x"],
                   "nodes_y": series_nodes["nodes_y"],
                   "fit_x": series_fits["fit_x"],
                   "fit_y": series_fits["fit_y"]},
        "csv": "runtime/metrology_data/" + fname, "csv_name": fname,
        "detail": {
            "ref_axis": ref_axis, "node_pitch_mm": pitch,
            "k_points": k_points, "frames_per_point": fpp,
            "axes": [ax_x, ax_y], "stage_sigma": sigma,
            "center_xy": [round(center[0], 6), round(center[1], 6)],
            "span_mm": round((k_points - 1) * pitch, 6),
            "slope_x_mm_per_step": [round(v, 9) for v in slope_x_last],
            "slope_y_mm_per_step": [round(v, 9) for v in slope_y_last],
            "rho_urad": round(m_rho * 1e6, 3),
            "psi_y_urad": round(m_psi * 1e6, 3),
            "matrix_l2p": mat_l2p,
            "matrix_p2l": mat_p2l,
            # 规范 6.4 数值核验 ‖A·C−I‖ 最大元（<1e-12 为通过）
            "ac_check_norm": ac_check,
            "csv_nodes": "runtime/metrology_data/" + fname_nd,
            "note": "栅格跳测法（迭代 37；迭代 42b 分量分解+比例）：基准"
                    "尺=晶圆 die 阵列（wafer_sites 格点+site_marks quad，"
                    "假定正交）；十字对称走法（K 奇数，中心±(K−1)/2 步，"
                    "升序单侧逼近）；走距=格点节距（start 与处方 pitch 校"
                    "验一致 0.001mm）；o_i=标记世界位−相机世界位（_context"
                    " 链，含 dir/微旋）=逐节点相对光轴偏移（相邻节点换相邻"
                    " site 同角，o 序列斜率=台步进−栅格步弦差）；分解走横"
                    "向/沿轨分量：X 线横向分量读栅格安装旋转 ρ̂=−asin("
                    "s_x'·ŷ/p)（弦模型精确，噪声地板判 0）、Y 线横向分量给"
                    " ψ_y=asin(s_y'·x̂/p)≈φ_t+ρ̂（符号自带）；φ(X 基准)="
                    "wrap(ψ_y−ρ̂)（ρ̂ 差中抵消）；θ_x=−ρ̂、θ_y=wrap(φ−ρ̂)"
                    "（偏差口径，锚定 wrap(θ_y−θ_x)=φ 与机器常数语义一致"
                    "）；ref_axis=y 报告 φ 的负值；<pfx>nonortho_urad 恒为"
                    " X 基准值；步进向量 v=s'+p·R(ρ̂)e_line（加回栅格步），"
                    "ε=|v|/p−1——ε 含相机像元当量误差别名（真机需干涉仪/"
                    "激光尺解耦），阿贝/直线度位置相关项落残差/LUT 不进比"
                    "例；matrix_l2p/matrix_p2l=物理↔逻辑双向全矩阵（[col_x"
                    "|col_y]=v̄/pitch 实测像，列模长=ε，X 线方向不再强制贴"
                    "逻辑 X，9 位有效数字，与参考脚本 grating_and_"
                    "interferometer_parallel_NO_error_PCA.py 正向/逆向矩阵"
                    "同族）；nodes CSV 的 mark_world 列口径（迭代 44 起）："
                    "ws=WS_LS 读数位+像素反算偏移 o（台基准实测位置链，随台"
                    "步进；旧 cam_world+o 在相机固定+跳测换标记下恒为常数、"
                    "无分析价值），as=cam_world+o（相机随对准台动）；迭代 45"
                    " 规范对齐（《XY运动台非正交（垂直度）校准与补偿技术规"
                    "范》Rev2.0）：measured.exy_urad=规范口径 EXY（式 4，"
                    "夹角>90°为正，=−X 基准 φ）、beta_urad=上板倾斜 β（=ρ̄"
                    "）、exy_u/exy_U=附录 D 标准/扩展（k=2）不确定度"
                    "（u_meas=线法向残差 RMS 保守估计）、detail.ac_check_"
                    "norm=6.4 数值核验 ‖AC−I‖ 最大元；nodes CSV 追加 frame_"
                    "cols_px/frame_rows_px 逐帧原始坐标列（5.3 单帧保存）",
        },
        "context": None, "elapsed_s": round(time.time() - t0, 3),
        "ts": round(time.time(), 3),
    }
