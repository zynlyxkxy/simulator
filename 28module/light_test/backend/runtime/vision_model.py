# -*- coding: utf-8 -*-
"""vision_model.py — 整机视觉系统的"物理"层：相机模型 + 标记场景生成 + 图像渲染。

坐标/图像约定（全线统一，文档见 docs/视觉系统_V1.md）：
- 世界坐标：机架右手系（X 向右、Y 向后、Z 向上，mm），与 error_kinematics 同系。
- 图像坐标：原点左上，u 向右、v 向下（工业相机通用）。结果 lstCol=u、lstRow=v。
- 远心相机：像素当量全幅恒定 k_mm = pixel_um/mag/1000（无透视）；
  投影 = 参考项目 d2w_sim_semiconductor_refactor sim/vision.py 的做法——
  先施加相机物理安装旋转 view_rot（真值，R(−θ)：相机 +θ 逆时针 →
  图像内容反向 −θ），再按 dir_x/dir_y/swap_xy 映射到 (u,v)，
  像素 = 中心 + (u,v)/k_mm。
- 焦面 Z = 相机 Z + look_dir×wd_mm（look_dir=-1 朝下看 / +1 朝上看）；
  focus_err = 标记 Z − 焦面 Z；离焦模糊 blur_ratio = |focus_err_um|/dof_um，
  渲染高斯 σ_px∝blur_ratio（封顶），blur_ratio>3 判废（参考 flows.py
  cam_measure 的景深规则）。

标记可见性映射（vision_cameras.json 每台的 marks.source）：
- down  ← wafer_marks：晶圆 marks（recipe.wafer.marks，晶圆在片才画）
  + 每 site 对准标记（recipe.wafer.site_marks：ratio×site 半宽等比缩放，
  命名 site_R{r}C{c}#{k}，z=晶圆面与全局 marks 同层；被键合 die 覆盖的
  site 不出标记（物理遮挡）；~2700 sites 按 FOV 剪枝构造）
  + 已键合到晶圆上的 die 的标记（carrier=wafer，几何=recipe.film.marks，
  随 die site/rz + WS_Rz + 模块姿态；z=晶圆面+die 厚，与晶圆 marks 分属
  不同表面层，离焦量各自独立）；
- up    ← head_die_marks：键合头吸取的 die 上的标记（die 在 head 才画）；
- coarse← flip_die_marks：翻转吸盘上 die 的**背面检出特征**（carrier=flip
  且 cos(FA_Rx)>0 即呈现位 Rx=0 才有效——粗对准相机对的是翻转模块上的
  die，不是蓝膜上的；2026-08-25 零位约定：Rx=0 吸盘面朝上=呈现位，die
  背面朝上（标记面贴吸盘朝下不可见），素板无标记 → 特征=die 外廓虚拟框（virtual，
  不画进图像，供 FAST 解析/引导）；Rx=±π 取片位 die 挂吸盘面下被本体
  遮挡；蓝膜面 ~96mm 离焦远超景深，膜上 die 亚分辨率不可见）。
die 标记几何全部来自处方 recipe.film.marks（film/head/wafer 上的 die
共用）:
位置 = ratio（相对 die 半宽的比例坐标）× (size_x/2, size_y/2)——die 尺寸
变则标记布局等比缩放；per_die 启用 ratio 前几个；shape/size_um 定形状
尺寸。标记世界位姿 = 载体模块 6DOF 位姿（error_kinematics 名义+误差，含
姿态旋转局部坐标；晶圆另加 WS_Rz 名义转角）+ 处方版图局部坐标。
无工件=空视野（下相机无晶圆→只见陶瓷面（纹路），无标记）。

渲染器（REAL 用）**分层景深**管线（2026-08-21 起，替代整帧统一模糊）：
1) 整帧铺暗背景 SCENE_VOID（载体之外的机器内部暗区）；
2) 按"回→前"顺序逐**表面层**合成（collect_layers）：每层有自己的世界 Z
   （含载体 6DOF 位姿 + recipe.wafer.thickness 等）——
   down=陶瓷吸盘面(chuck，含真空气路纹路)→晶圆面(wafer)→片上 die 面
   (wafer_die)；coarse=蓝膜面(film，在位才画、离焦雾化的纯背景，不画
   膜上 die)→翻转吸盘面(flip_chuck，91mm 方板 ±45.4 对称，任意 FA_Rx
   顶面恒 z=工具点+45.4)→翻转 die 面(flip_die，呈现位 cos(FA_Rx)>0 且
   carrier=flip 才画，素板背面无标记)；up=吸盘尖端面(collet，含真空十字
   槽纹；无 die 时上相机不再黑屏)。
   标记随各自 z 就近的表面层一起绘制（渲染时按标记 z 归属最近层：
   wafer 层的 recipe 晶圆 marks 与 wafer_die 层的 die 标记 z 不同，
   分别随层模糊）；
3) 每层独立离焦模糊：σ_layer = min(SIGMA_MAX, K_BLUR×|层Z−焦面Z|×1000/dof_um)，
   σ≤0.05 跳过（对焦层零额外开销）；否则在该层内容像素 bbox（外扩
   ceil(3σ)+2）内高斯后贴回（σ>2 降采样模糊再放回：σ≤4→1/4、σ>4→1/8，
   成本 ≤1/16）。
   遮挡语义由回→前绘制顺序天然保证（晶圆盖吸盘）；离焦层 bbox 内已合成的
   后层内容一并承受本层 σ（本机相邻面 σ 接近，无可见伪影）；
4) 乘性照明渐晕（中心亮边缘暗：1−VIGNETTE·r²/r_max²，按分辨率缓存）；
5) 相机质心噪声（迭代 28）：recipe sim_sensors.fast_noise_px[相机]（与
   FAST 链同一旋钮，每帧热读）÷ 模板响应度 resp（px/DN，建模板时现场
   实测自标定）= 灰度噪声 σ（双封顶 min(40DN, 0.45×模板RMS灰度差)，
   防噪声淹没 NCC），单位噪声大底版随机偏移视图（迭代 27 帧间去相关
   保留）×σ 叠加；再 × recipe sim_sensors.real_noise_scale[相机] 额外
   倍率（默认 1.0）。fast_noise_px=0 → 跳过（确定性回退）；无模板响应
   度回退 K_NOISE·√gain/√(exposure·光强) 旧公式。
光源灭 = 各层按比例压到近全黑 + 强噪声。陶瓷吸盘面纹路 = 静态真空气路
图案（同心环槽间距 5mm/宽 0.5mm/暗 15% + 4 条径向槽，图案常量化缓存，
每帧只画与 FOV 相交的环）。场景几何全部来自已有物料/载体状态
（workpieces + recipe + kinematics），不新造事实源。numpy 合成灰度图
+ cv2 精确画旋转矩形/多边形/圆盘/环槽；cv2/numpy 全部 lazy import——
核心运行时不依赖它们；缺 cv2 时 REAL 由 vision_service 报清晰错误，
渲染退化为纯 numpy 简画（仅预览用，不旋转、不模糊）。

FAST 不渲染大图：用解析投影器直接由几何出结果（同 schema）。

配置双源（2026-08-21 起）：本文件读 vision_cameras.json（位姿/采集）+
machine_constants.json cameras 组（内参 pixel_um/mag/dir_x/dir_y/swap_xy/
rot_mrad/wd_mm/dof_um + px_u/px_v 传感器分辨率（2026-08-24 自
vision_cameras.json 迁入并更名，机器常数单一事实源，含真实光学数据）），
load_cameras 合并校验，坏配置拒绝拉起；PUT /api/machine_constants 后
apply_constants 热生效。

视场旋转 view_rot_deg（2026-09 迭代 35 引入；迭代 39 重构为
**真值/认知分离**）：相机**物理安装旋转角**（真值，°，±5°；
vision_cameras.json defaults/每相机键，运行时可经
/api/vision/camera/config 在线改，模型内按 rad 属性 view_rot 换算）。
+θ=相机逆时针 → 世界/标记不动、图像内容绕视场中心反向（−θ）旋转。
**只进投影链**（世界→像素：project_rel 施加 R(−view_rot)，渲染/
FAST 解析/guide 同链一致），**不进反算链**；identify 结果=原始
传感器系，不再做任何剥离。机器常数 rot_mrad=**控制器认知**（同号
约定），只进反算链（solve_rel/solve_pixel 施加 R(+rot)）：θ_b=θ_t
时 solve∘project=恒等（补偿到位）；θ_b≠θ_t 时世界解相差旋转
(θ_b−θ_t)——真实设备"控制器认知不准→测量带旋转误差"的建模。
补偿二选一口径：图像去旋=物理转正 view_rot；坐标修正=写 rot_mrad。
"""
import json
import math
import os
import random
import struct
import threading
import time
import zlib

ROOT = os.path.dirname(os.path.abspath(__file__))
CAMERAS_JSON = os.path.join(ROOT, "vision_cameras.json")

_np = None
_cv2 = None
_cv2_missing = None
_tls = threading.local()


def np():
    global _np
    if _np is None:
        import numpy
        _np = numpy
    return _np


def cv2_or_none():
    """lazy import cv2；缺失返回 None（REAL 模式由上层报清晰错误）。"""
    global _cv2, _cv2_missing
    if _cv2 is None and _cv2_missing is None:
        try:
            import cv2
            _cv2 = cv2
        except ImportError:
            _cv2_missing = True
    return _cv2


def _rng():
    r = getattr(_tls, "rng", None)
    if r is None:
        r = np().random.default_rng()
        _tls.rng = r
    return r


# ---------------- 配置 ----------------
MARK_SOURCES = ("wafer_marks", "film_die_marks", "head_die_marks",
                "flip_die_marks")
CAMERA_MODES = ("continuous", "trigger")
MODULES = ("wafer_stage", "align_stage", "bond_head", "flip", "film_stage")

# 渲染常数（合成值，待真机标定）
BG_LIGHT = 165.0          # 光源开时明场基准灰度（场景各层按其比例取值）
BG_DARK = 8.0             # 光源灭时暗底
MARK_DARK = 0.35          # 标记灰度 = 背景 × 此系数（暗标记）
# 视场场景层（相对基准的比例；绘制顺序：暗背景→载体面→die 本体→标记）
SCENE_VOID = 0.12         # 视野落在载体之外（机器内部暗区）
SCENE_CARRIER = 0.72      # 载体面（晶圆台陶瓷面/蓝膜面/吸盘尖端面）
SCENE_WAFER = 0.88        # 晶圆表面（略亮于台面，圆盘边界可见）
SCENE_DIE = 0.55          # die 本体（与载体有灰度差，边缘可见）
VIGNETTE = 0.25           # 照明渐晕：边缘按 (1-V·r²/r_max²) 变暗
K_NOISE = 6.0             # 噪声系数：σ = K_NOISE·√gain/√(exposure_ms·光强)
                          # （迭代 28 起仅作无模板响应度时的回退路径）
NOISE_SIGMA_CAP_DN = 12.0  # REAL 灰度噪声 σ 物理封顶（DN；2026-09-08
                           # 40→12：用户实测雪花过强——12DN 在 ~110DN 亮面
                           # 上占 ~11%，视觉温和；缺口由平移分量承担
                           # （结果倒推，见 SCENE_SHIFT_GAIN））。快噪声
                           # 换算超出此值意味着 8-bit 图像被噪声淹没、NCC
                           # 必败——封顶保持可识别
NOISE_SNR_CAP = 0.25      # SNR 封顶：σ_gray ≤ 0.25×模板 RMS 灰度差
                           # （tpl_std_dn）——噪声超过模板自身对比度量级时
                           # NCC 信号淹没；0.45→0.25（2026-09-08 同雪花
                           # 减弱）。双封顶取小：min(12DN, 0.25×tpl_std)
SCENE_SHIFT_GAIN = 1.0    # 结果倒推平移注入的缺省增益系数（在线标定
                           # noise_scene_gain 缺失时用 1.0）：图像上标记
                           # 抖动量=√(fnp²−白噪声贡献²)/g，g 为建模板探针
                           # 实测的"NCC 匹配链对随机亚像素平移的传递增益"
                           # （实测 0.44~1.5 随模板/方向漂移，粗网格 NMS
                           # 相位依赖——除以 g 使测校输出 3σ 精确=预算）
SUBPIX_SHIFT = 16         # cv2 定点子像素坐标小数位（2026-09-08 修复）：
                           # 多边形/圆顶点按 1/65536 px 定点传入 + LINE_AA
                           # 抗锯齿——此前 astype(int32) 整像素截断把标记位
                           # 置量化到整像素网格，误差链（repeat/误差地图，
                           # 数十 nm=百分之几 px）被量化成 ±0.5px 台阶，
                           # 动态重复性 3σ 被夸大 10×且完全非高斯（实测
                           # cols 只在 1498.673/1498.174 两个离散值间跳动）
_I32_MAX = (1 << 31) - 1  # cv2 定点坐标 int32 上限：×2^16 后仅容纳
                           # ±32768 px；陶瓷面环槽这类半径 ~19 万 px 的
                           # 远景几何会溢出（实测 center=8.9e9 radius=1.3e10
                           # → Bad argument）——回退整像素绘制（shift=0 仍
                           # 带 AA）：远景背景亚像素无关紧要，近景标记不受
                           # 影响

def _shift_int(v):
    """定点子像素坐标 int；溢出 int32 → None（调用方回退整像素）。"""
    sv = int(round(v * (1 << SUBPIX_SHIFT)))
    return sv if -_I32_MAX <= sv <= _I32_MAX else None

def _circle_subpix(cv2, img, col, row, rp, gray, thickness=-1):
    """子像素圆（定点+AA）；坐标/半径溢出 int32 定点域回退整像素+AA。"""
    cx, cy, rr = _shift_int(col), _shift_int(row), _shift_int(rp)
    if cx is None or cy is None or rr is None:
        cv2.circle(img, (int(round(col)), int(round(row))),
                   int(round(rp)), gray, thickness, lineType=cv2.LINE_AA)
    else:
        cv2.circle(img, (cx, cy), rr, gray, thickness,
                   lineType=cv2.LINE_AA, shift=SUBPIX_SHIFT)

def _fillpoly_subpix(cv2, n, img, pts, gray):
    """子像素多边形（定点+AA）；任一顶点溢出 int32 定点域回退整像素+AA。
    注意：×2^16 后 ~1e10 量级，必须 float64 计算——float32 尾数 24bit
    在此量级 ULP≈0.25px，会把子像素位置重新量化掉。"""
    pp = n.array(pts, dtype=n.float64) * float(1 << SUBPIX_SHIFT)
    if n.abs(pp).max() > _I32_MAX:
        cv2.fillPoly(img, [n.round(n.array(pts, dtype=n.float32))
                           .astype(n.int32).reshape(-1, 1, 2)],
                     gray, lineType=cv2.LINE_AA)
    else:
        cv2.fillPoly(img, [n.rint(pp).astype(n.int32).reshape(-1, 1, 2)],
                     gray, lineType=cv2.LINE_AA, shift=SUBPIX_SHIFT)


# ---------------- 标记精灵光栅（2026-09-08 亚像素修复） ----------------
# cv2.fillPoly/circle 的 shift 只平移边界、填充边不做抗锯齿（中心采样二值
# 边）——标记位置仍被量化到整像素网格（实测 col 台阶 ~0.7px=430nm），
# repeat/误差地图的数十 nm 分量完全不可见，动态重复性被量化成 ±0.5px
# 台阶（测校"误差链没进来"的最终根源）。改为：标记形状按无旋转姿态
# 光栅到 _MARK_SS× 超采样覆盖率母版（按 形状+像素尺寸 缓存），每帧用
# cv2.remap 双线性采样（旋转+缩放+分数平移全部连续）得亚像素覆盖率，
# 与背景按覆盖率混合——位置响应连续（阶梯测试 0→1µm 线性，增益≈1）。
_MARK_SS = 4
_SPRITE_CACHE = {}
# 迭代 33 串图竞态修复：原全局 (H,W)→buf 字典被所有渲染线程共享——
# up/down 相机同分辨率（3000×3000）撞同一缓冲，fill(0)/绘制/混合三步
# 之间的 time.sleep(0) 让出 GIL，对面线程即把自己的标记画进同一缓冲，
# A 相机帧混入 B 相机标记。改为 threading.local：每个渲染线程一份
# (H,W)→buf 字典，跨帧复用语义保留，4 桶上限改为**每线程** 4 桶。
_COV_SCRATCH = threading.local()


def _cov_buffer(n, W, H):
    """标记覆盖率累积缓冲（复用；每帧首次取用时清零——上帧内容已消费）。
    缓冲表按渲染线程隔离（threading.local，见 _COV_SCRATCH 注释）：
    同分辨率双相机并行渲染不再共享内存。"""
    d = getattr(_COV_SCRATCH, "bufs", None)
    if d is None:
        d = {}
        _COV_SCRATCH.bufs = d
    buf = d.get((H, W))
    if buf is None:
        buf = n.zeros((H, W), dtype=n.float32)
        if len(d) >= 4:
            d.clear()
        d[(H, W)] = buf
    return buf


def _mark_master(mark, size_px, k):
    """无旋转标记覆盖率母版（[0,1] float32，_MARK_SS× 超采样）。
    母版二值填充即可：remap 双线性采样把 1/SS px 宽的二值边变成连续
    带限覆盖率，亚像素相位信息由插值保真。按 (shape, size_px 量化)
    缓存；上界 16 个防膨胀。返回 (master, half, scale)——scale=母版
    每目标像素的采样步长（=SS）。"""
    key = (mark.shape, round(size_px, 2))
    hit = _SPRITE_CACHE.get(key)
    if hit is not None:
        return hit
    n, cv2 = np(), cv2_or_none()
    scale = float(_MARK_SS)
    # 图元以局部 mm 表达，目标半径 size_px/2；母版半幅 = 内容 + 高斯预滤波
    # 3σ（0.6 目标px×SS×3）+ 2 目标px 护边——余量不足时模糊的数组边界
    # 反射会污染边缘斜坡（实测引入 ~0.3px 相位锯齿）
    half = int(math.ceil(size_px / 2.0 * scale)) + 5 * int(scale)
    m = n.zeros((2 * half, 2 * half), dtype=n.float32)
    mmpx = size_px / mark.size_mm * scale     # 局部 mm → 母版 px
    c = half
    for prim in shape_primitives(mark):
        if prim[0] == "rect":
            _, rcx, rcy, rw, rh = prim
            x0, y0 = c + (rcx - rw / 2.0) * mmpx, c + (rcy - rh / 2.0) * mmpx
            x1, y1 = c + (rcx + rw / 2.0) * mmpx, c + (rcy + rh / 2.0) * mmpx
            pp = n.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
                         dtype=n.float32)
            cv2.fillPoly(m, [pp.reshape(-1, 1, 2).astype(n.int32)], 1.0)
        elif prim[0] == "ring":
            _, rcx, rcy, r_out, r_in = prim
            cc = (int(round(c + rcx * mmpx)), int(round(c + rcy * mmpx)))
            cv2.circle(m, cc, int(round(r_out * mmpx)), 1.0, -1)
            cv2.circle(m, cc, int(round(r_in * mmpx)), 0.0, -1)
    # 母版高斯预滤波（σ≈0.6 目标像素，边缘扩展 ~1.2px——真实远心光学
    # 量级）：二值母版边缘在 SS 网格上，bilinear 降采样对齐网格相位呈
    # 锯齿偏差；预滤波后采样在平滑斜坡上插值，相位偏差 <0.001px
    # （隔离验证：SS=4 σ=0.6 双边缘质心平移最大偏差 0.0005px）
    m = cv2.GaussianBlur(m, (0, 0), 0.6 * _MARK_SS)
    if len(_SPRITE_CACHE) >= 16:
        _SPRITE_CACHE.clear()
    entry = (m, half, scale)
    _SPRITE_CACHE[key] = entry
    return entry


def _draw_mark_cov(n, cv2, cov_buf, mark, col, row, k, W, H):
    """把一个标记的亚像素覆盖率 warpAffine 进 cov_buf（0..1；多标记重叠
    区取并集）。cv2 缺失时调用方走 _draw_prim 降级。
    warpAffine 单次调用完成旋转+缩放+分数平移（连续采样）且释放 GIL：
    物理线程 HIGHEST 忙等下 numpy 逐条小操作会被稀释 160×（实测
    1.4→227ms/标记），cv2 大块调用不受影响。
    **2026-09-21 修复（REAL 键合后检测 quad 全灭根因）**：旧实现直接
    `dst=cov_buf[y0:y1, x0:x1]`——warpAffine 是**整窗替换**而非累积；
    两标记绘制窗（±size/2+3）重叠时，后画标记把先画标记的覆盖率整窗
    清零。键合后检测 IR 套刻对（site quad 与 die cross 相距 ~21px≪
    窗径 128px）必现：先画的 quad 只剩 cross 窗口外的右缘细条，NCC
    无峰 → pi_measure"双组标记不齐"。改 warp 进暂存后 `cv2.max` 并入
    （覆盖率并集；不同物理标记本就不同像素重叠，max 语义正确）。"""
    size_px = mark.size_mm / k
    master, half, scale = _mark_master(mark, size_px, k)
    c_, s_ = math.cos(mark.yaw), math.sin(mark.yaw)
    ext = size_px / 2.0 + 3.0
    x0 = max(0, int(math.floor(col - ext)))
    x1 = min(W, int(math.ceil(col + ext)) + 1)
    y0 = max(0, int(math.floor(row - ext)))
    y1 = min(H, int(math.ceil(row + ext)) + 1)
    if x1 <= x0 or y1 <= y0:
        return
    cs, ss = c_ * scale, s_ * scale
    # WARP_INVERSE_MAP：M 直接作为 目标像素→母版坐标 的映射（缺省会把 M
    # 当正向变换内部求逆——方向相反）；且 warpAffine 以**窗口局部坐标**
    # 求值，目标位 (col,row) 须先减窗口原点 (x0,y0)
    lc, lr = col - x0, row - y0
    mtx = n.array([[cs, ss, half - (cs * lc + ss * lr)],
                   [-ss, cs, half - (-ss * lc + cs * lr)]], dtype=n.float32)
    warped = cv2.warpAffine(master, mtx, (x1 - x0, y1 - y0),
                            flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    # 窗口重叠区并集累积（不整窗替换，见 docstring 2026-09-21 修复）
    cv2.max(cov_buf[y0:y1, x0:x1], warped, dst=cov_buf[y0:y1, x0:x1])
LIGHT_LEVEL_OFF = 0.03    # 光源灭时的等效残余光强
K_BLUR = 0.9              # 高斯模糊 σ_px = K_BLUR × blur_ratio（封顶 SIGMA_MAX）
SIGMA_MAX = 8.0
BLUR_REJECT = 3.0         # blur_ratio 超过此值判废（参考 flows.py cam_measure）
ARM_RATIO = 0.25          # cross 臂宽 = size × 0.25
CIRCLE_INNER = 0.6        # circle 环内径 = 外径 × 0.6
FAST_NOISE_PX = 0.1       # FAST 解析投影质心噪声（px；2026-09-08
                           # 用户设定 up/down 默认 0.1，coarse 单独 0.15）
FAST_BASE_SCORE = 0.96    # FAST 分数基准（blur 与抖动下压）
# 陶瓷吸盘面真空气路纹路（静态图案，本表即"按分辨率缓存"的图案定义：
# 吸盘局部系几何，世界位姿每帧仅平移+偏航，见 _chuck_groove_items）
CHUCK_GROOVE_STEP = 5.0                       # 同心环槽间距 mm
CHUCK_GROOVE_W = 0.5                          # 槽宽 mm
CHUCK_GROOVE_GRAY = 0.85                      # 槽灰度 = 载体面 × 0.85（暗 15%）
CHUCK_GROOVE_RADII = tuple(range(5, 201, 5))  # 环槽半径 5..200mm
CHUCK_SLOT_ANGLES = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)  # 4 径向槽
CHUCK_SLOT_R0 = 0.0                           # 径向槽内端半径 mm（自槽心起）
CHUCK_SLOT_R1 = 200.0                         # 径向槽外端半径 mm
# 翻转吸盘面（diechuck 91mm 方板，工具点=AABB 中心）：半宽 45.4mm，
# ±45.4 上下对称（FA_Rx=0/π 朝上面恒 z=工具点+45.4）；板面纹路 = 淡
# 真空槽（中心环 + 十字槽，参考 diechuck_cross/ring 沙盒件，别过度）
FLIP_CHUCK_HALF = 45.4                      # 方板半宽 mm（AABB 90.8 见方）
FLIP_CHUCK_GRAY = 0.9                       # 纹路灰度 = 载体面 × 0.9（淡）
FLIP_CHUCK_RING_R = 30.0                    # 中心环槽半径 mm
FLIP_CHUCK_GROOVE_W = 1.0                   # 槽宽 mm
FLIP_CHUCK_CROSS_ARM = 40.0                 # 十字槽臂长（半长）mm
# 键合头吸盘底面（collet）：AABB 54.6 mm 见方、中心 (-42.5,7.0)（2026-08-27
# 迭代 4 零位纠错后）。
COLLET_HALF = 27.28                         # 吸盘面半宽 mm
COLLET_CROSS_GRAY = 0.85                    # 槽灰度 = 载体面 × 0.85（暗 15%）
# 真空气道纹路（2026-09-21 迭代 54 真实化，取代旧"一条 50.6×2.56mm 通边大
# 十字槽"——气道直通边缘在物理上不成立：真空槽必须由密封缘包住否则漏气；
# 同日按用户实物反馈再改：中心不是实心台柱，真机拾取吸盘中心是真空主孔+
# 宽带十字凹槽分配道）。
# 形态：中心真空圆孔（主抽气口）+ 宽臂十字凹槽（主孔→内环槽互联）+ 两圈
# 环形分配槽 + 窄径向槽，槽全部内缩于密封缘；主轴走向延续 3D 模型
# bondhead_cross_h/v 的十字设计语言。head_chuck_cross 标定标记（150µm）
# 在孔底（mark_gray=base×0.35 << 槽底 base×0.72×0.85，对比度保持可识别，
# 物理解释=标记刻在真空孔/槽底部）。
COLLET_SEAL_MM = 3.0                        # 密封缘宽度（气道不出此缘）
COLLET_HOLE_R = 1.5                         # 中心真空主孔半径 mm
COLLET_HOLE_GRAY = 0.70                     # 主孔灰度系数（深孔吸光，比浅槽暗）
COLLET_XGROOVE_W = 2.0                      # 中心十字凹槽带宽 mm（宽臂）
COLLET_XGROOVE_GRAY = 0.78                  # 十字凹槽灰度系数（介于孔与浅槽间）
COLLET_RING_R = (6.0, 16.0)                 # 环形分配槽半径 mm
COLLET_GROOVE_W = 0.6                       # 环槽宽 mm
COLLET_SPOKE_W = 0.5                        # 径向槽宽 mm（窄槽，防小半径重叠）
COLLET_SPOKES = 8                           # 径向槽条数（含原十字主轴）
# 各相机标记的默认宿主表面层名（信息性；渲染按标记 z 就近归属层——
# 下相机 recipe 晶圆 marks 在 wafer 层、已键合 die 标记在 wafer_die 层，
# z 不同、离焦量不同，分别随层模糊）
MARK_HOST_LAYER = {"wafer_marks": "wafer", "film_die_marks": "film_die",
                   "head_die_marks": "collet", "flip_die_marks": "flip_die"}


def validate_config(cfg):
    """本文件（位姿/采集）schema 校验 -> 错误明细列表（空=合法）。
    内参字段（pixel_um/mag/dir_x/dir_y/swap_xy/rot_mrad/wd_mm/dof_um +
    px_u/px_v 分辨率）不在本文件——由 machine_constants.py 校验并在
    load_cameras 合并时交叉核对；本文件出现 px_x/px_y（旧键）同样拒绝。"""
    errors = []
    if not isinstance(cfg, dict) or not isinstance(cfg.get("cameras"), dict):
        return ["顶层须含 cameras 字典"]
    dfl = cfg.get("defaults", {})
    if not isinstance(dfl, dict):
        errors.append("defaults 须为对象")
        dfl = {}
    for key in ("px_x", "px_y"):
        if key in dfl:
            errors.append("defaults.%s 已迁至 machine_constants.json（更名 "
                          "px_u/px_v，内参单一事实源），本文件不再接受" % key)
    for key in ("exposure_ms", "gain", "score_threshold"):
        v = dfl.get(key)
        if v is not None and (not isinstance(v, (int, float))
                              or isinstance(v, bool) or not v > 0):
            errors.append("defaults.%s 须为正数" % key)
    if dfl.get("mode", "trigger") not in CAMERA_MODES:
        errors.append("defaults.mode 只能是 %s" % (CAMERA_MODES,))
    vr = dfl.get("view_rot_deg")
    if vr is not None and (not isinstance(vr, (int, float))
                           or isinstance(vr, bool)
                           or not math.isfinite(float(vr))
                           or abs(float(vr)) > 5.0):
        errors.append("defaults.view_rot_deg 须为 |θ|≤5° 的有限数（相机物理"
                      "安装旋转角·真值，逆时针为正，图像内容反向旋转）: %r"
                      % vr)
    cams = cfg["cameras"]
    if not cams:
        errors.append("cameras 不能为空")
    for cid, c in cams.items():
        pre = "相机 %s" % cid
        if not isinstance(c, dict):
            errors.append("%s 须为对象" % pre)
            continue
        mount = c.get("mount")
        if not isinstance(mount, dict):
            errors.append("%s.mount 须为对象" % pre)
        else:
            mod = mount.get("module")
            if mod is not None and mod not in MODULES:
                errors.append("%s.mount.module 未知模块 %r（null=固连机架）"
                              % (pre, mod))
            off = mount.get("offset")
            if not (isinstance(off, (list, tuple)) and len(off) == 3 and
                    all(isinstance(x, (int, float)) and not isinstance(x, bool)
                        and math.isfinite(x) for x in off)):
                errors.append("%s.mount.offset 须为三维有限数向量" % pre)
        if c.get("look_dir") not in (-1, 1):
            errors.append("%s.look_dir 只能是 -1（朝下）/ +1（朝上）" % pre)
        if not c.get("light_point") or not isinstance(c.get("light_point"), str):
            errors.append("%s.light_point 必填（点位路径字符串）" % pre)
        if not c.get("trigger_source"):
            errors.append("%s.trigger_source 必填" % pre)
        marks = c.get("marks")
        if not isinstance(marks, dict) or marks.get("source") not in MARK_SOURCES:
            errors.append("%s.marks.source 只能是 %s" % (pre, MARK_SOURCES))
        else:
            zo = marks.get("surface_z_offset_mm", 0.0)
            if not isinstance(zo, (int, float)) or not math.isfinite(zo):
                errors.append("%s.marks.surface_z_offset_mm 须为有限数" % pre)
        for key in ("exposure_ms", "gain"):
            v = c.get(key, dfl.get(key, 1.0))
            if not isinstance(v, (int, float)) or isinstance(v, bool) or v <= 0:
                errors.append("%s.%s 须为正数" % (pre, key))
        st = c.get("score_threshold", dfl.get("score_threshold", 0.6))
        if not isinstance(st, (int, float)) or not (0.0 < st <= 1.0):
            errors.append("%s.score_threshold 须在 (0,1]" % pre)
        if c.get("mode", dfl.get("mode", "trigger")) not in CAMERA_MODES:
            errors.append("%s.mode 只能是 %s" % (pre, CAMERA_MODES))
        vr = c.get("view_rot_deg", dfl.get("view_rot_deg"))
        if vr is not None and (not isinstance(vr, (int, float))
                               or isinstance(vr, bool)
                               or not math.isfinite(float(vr))
                               or abs(float(vr)) > 5.0):
            errors.append("%s.view_rot_deg 须为 |θ|≤5° 的有限数（相机物理安"
                          "装旋转角·真值，与机器常数 rot_mrad（控制器认知）"
                          "配套，相等=补偿到位）: %r"
                          % (pre, vr))
        for key in ("pixel_um", "mag", "dir_x", "dir_y", "swap_xy",
                    "rot_mrad", "wd_mm", "dof_um", "px_x", "px_y"):
            if key in c:
                errors.append("%s.%s 已迁至 machine_constants.json（内参单一"
                              "事实源，px 更名 px_u/px_v），本文件不再接受"
                              % (pre, key))
    return errors


class CameraModel(object):
    """单台相机：配置 + 解析投影（参考 sim/vision.py 的 Camera）。"""

    def __init__(self, cam_id, cfg, defaults):
        self.id = cam_id
        self.label = cfg.get("label", cam_id)
        self.mount_module = cfg["mount"].get("module")
        self.mount_offset = [float(v) for v in cfg["mount"]["offset"]]
        self.look_dir = int(cfg["look_dir"])
        self.wd_mm = float(cfg["wd_mm"])
        self.dof_um = float(cfg["dof_um"])
        self.px_x = int(cfg["px_u"])         # 分辨率=机器常数（merge 进来）
        self.px_y = int(cfg["px_v"])
        self.pixel_um = float(cfg["pixel_um"])
        self.mag = float(cfg["mag"])
        self.dir_x = float(cfg["dir_x"])
        self.dir_y = float(cfg["dir_y"])
        self.swap_xy = bool(cfg.get("swap_xy", False))
        # rot_mrad=控制器认知（机器常数，迭代 39 起为纯认知：只进反算链
        # solve_rel/solve_pixel，不进投影链）
        self.rot = float(cfg.get("rot_mrad", 0.0)) * 1e-3     # rad
        self.light_point = cfg["light_point"]
        self.trigger_source = cfg["trigger_source"]
        self.mark_source = cfg["marks"]["source"]
        self.surface_z_off = float(cfg["marks"].get("surface_z_offset_mm", 0.0))
        # 运行时默认（可被 /api/vision/camera/config 在线改）
        self.exposure_ms = float(cfg.get("exposure_ms",
                                         defaults.get("exposure_ms", 8.0)))
        self.gain = float(cfg.get("gain", defaults.get("gain", 1.0)))
        self.score_threshold = float(cfg.get(
            "score_threshold", defaults.get("score_threshold", 0.6)))
        self.mode = cfg.get("mode", defaults.get("mode", "trigger"))
        # 视场旋转（迭代 35 引入，迭代 39 重构，°）：相机物理安装旋转角
        # （真值，逆时针为正，图像内容反向旋转；只进投影链 project_rel，
        # 与机器常数 rot_mrad（控制器认知）配套，相等=补偿到位）。
        # 采集域配置（defaults/每相机）；rad 换算见 view_rot property
        # （camera_config 热改 view_rot_deg 即时生效）
        self.view_rot_deg = float(cfg.get(
            "view_rot_deg", defaults.get("view_rot_deg", 0.0)))

    @property
    def view_rot(self):
        """相机物理安装旋转角（rad，真值）= view_rot_deg 的弧度换算；
        property 保证 /api/vision/camera/config 热改 view_rot_deg 后
        投影链即时跟随（不经 apply_constants）。"""
        return math.radians(self.view_rot_deg)

    @property
    def k_mm(self):
        """像素当量 mm/px = 像元尺寸/放大倍率（远心：全幅恒定）。"""
        return self.pixel_um / self.mag / 1000.0

    @property
    def fov_mm(self):
        return self.px_x * self.k_mm, self.px_y * self.k_mm

    def project_rel(self, dx_mm, dy_mm, k=None):
        """空间偏差（标记相对光轴，世界系 mm）→ 图像系偏移 (u,v) px
        （相对画幅中心）。先施加相机物理安装旋转 R(−view_rot)（真值：
        相机 +θ 逆时针 → 图像内容 −θ），再做方向/互换映射。控制器认知
        rot_mrad 不进投影链（迭代 39 真值/认知分离）。"""
        if self.view_rot:
            cr, sr = math.cos(self.view_rot), math.sin(self.view_rot)
            dx_mm, dy_mm = (cr * dx_mm + sr * dy_mm,
                            -sr * dx_mm + cr * dy_mm)
        k = k or self.k_mm
        u = dx_mm / k * self.dir_x
        v = dy_mm / k * self.dir_y
        if self.swap_xy:
            u, v = v, u
        return u, v

    def project(self, dx_mm, dy_mm, w=None, h=None):
        """→ 绝对像素坐标 (col, row)。w/h 缺省为全分辨率。"""
        w = w or self.px_x
        h = h or self.px_y
        k = self.k_mm * (self.px_x / float(w))
        u, v = self.project_rel(dx_mm, dy_mm, k)
        return w / 2.0 + u, h / 2.0 + v

    def in_fov(self, col, row, margin=0.0, w=None, h=None):
        w = w or self.px_x
        h = h or self.px_y
        return (-margin <= col <= w + margin) and (-margin <= row <= h + margin)

    def solve_rel(self, u, v, k=None):
        """图像系偏差 (u,v) px（相对画幅中心）→ 空间偏差（世界系 mm，焦面
        平面）。先逆方向/互换映射，再施加控制器认知的安装微旋 R(+rot)
        （机器常数 rot_mrad，迭代 39 起为纯认知、只进反算链；与投影链
        真值 view_rot 同号约定——θ_b=θ_t 时 solve∘project=恒等（补偿
        到位），θ_b≠θ_t 时世界解相差旋转 (θ_b−θ_t)：认知不准则测量带
        旋转误差，符合真实。换算口径参考
        d2w_sim_semiconductor_refactor sim/vision.py Camera.solve——
        用处方标定参数反算，标错则结果错）。"""
        k = k or self.k_mm
        u, v = float(u), float(v)
        if self.swap_xy:
            u, v = v, u
        dx = u * k * self.dir_x
        dy = v * k * self.dir_y
        if self.rot:
            cr, sr = math.cos(self.rot), math.sin(self.rot)
            dx, dy = cr * dx - sr * dy, sr * dx + cr * dy
        return dx, dy

    def solve_pixel(self, col, row, cam_pos, w=None, h=None):
        """绝对像素 (col,row) → 焦面平面上世界坐标 (x,y,z)。cam_pos=相机
        光轴世界坐标（camera_pose 结果）；焦面 z = cam_pos.z + look_dir×wd。
        粗对准"拍照→芯片世界坐标"的反投影通道（像素当量 k、方向性
        dir_x/dir_y/swap_xy、安装微旋 rot（控制器认知）全部按处方
        反算——认知与投影链真值 view_rot 相等时反投影精确）。"""
        w = w or self.px_x
        h = h or self.px_y
        k = self.k_mm * (self.px_x / float(w))
        dx, dy = self.solve_rel(col - w / 2.0, row - h / 2.0, k)
        z = cam_pos[2] + self.look_dir * self.wd_mm
        return cam_pos[0] + dx, cam_pos[1] + dy, z


def load_cameras(path=CAMERAS_JSON, constants=None):
    """加载相机配置：vision_cameras.json（位姿/采集）与 machine_constants.json
    的 cameras 组（内参，单一事实源）**合并**后构造 CameraModel；任一侧
    schema 校验失败抛 ValueError（启动拒绝拉起）。constants 可传
    MachineConstants 实例（热重载时复用同一对象）。"""
    from machine_constants import MachineConstants
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    errors = validate_config(cfg)
    if errors:
        raise ValueError("vision_cameras.json 校验失败（%d 条）：\n  %s"
                         % (len(errors), "\n  ".join(errors)))
    if constants is None:
        constants = MachineConstants()
    const = constants.get()["cameras"]
    for cid in cfg["cameras"]:
        if cid not in const:
            raise ValueError("机器常数 cameras 组缺相机 %r 的内参" % cid)
    for cid in const:
        if cid not in cfg["cameras"]:
            raise ValueError("机器常数 cameras 组有未知相机 %r"
                             "（vision_cameras.json 无此相机）" % cid)
    defaults = cfg.get("defaults", {})
    out = {}
    for cid, c in cfg["cameras"].items():
        merged = dict(c)
        merged.update({k: v for k, v in const[cid].items()
                       if not k.startswith("_")})
        out[cid] = CameraModel(cid, merged, defaults)
    return out


def apply_constants(cam_models, constants):
    """内参热生效：PUT /api/machine_constants 成功后，把新内参写回现有
    CameraModel（k_mm/fov 为 property 自动跟随；曝光/gain/mode 等采集
    运行态不动）。相机集合一致性在 load/update 校验链上已保证。
    rot_mrad 为控制器认知（迭代 39 起只进反算链），在此热更；真值
    view_rot_deg 的热更走 /api/vision/camera/config 既有路径（view_rot
    为 property 即时跟随），不经本函数。"""
    const = constants.get()["cameras"]
    for cid, cam in cam_models.items():
        c = const.get(cid)
        if c is None:
            continue
        cam.pixel_um = float(c["pixel_um"])
        cam.mag = float(c["mag"])
        cam.dir_x = float(c["dir_x"])
        cam.dir_y = float(c["dir_y"])
        cam.swap_xy = bool(c["swap_xy"])
        cam.rot = float(c["rot_mrad"]) * 1e-3
        cam.wd_mm = float(c["wd_mm"])
        cam.dof_um = float(c["dof_um"])
        cam.px_x = int(c["px_u"])
        cam.px_y = int(c["px_v"])


# ---------------- 场景生成（标记世界位姿） ----------------
def _rotvec_apply(p, r):
    """按旋转矢量 r（rad）把点 p 绕原点旋转（|r|≈0 原样返回）。
    与 error_kinematics._rotvec_apply 同式（c=原点）。"""
    theta = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    if theta < 1e-15:
        return [p[0], p[1], p[2]]
    k = [r[0] / theta, r[1] / theta, r[2] / theta]
    x, y, z = p
    ct, st = math.cos(theta), math.sin(theta)
    d = k[0] * x + k[1] * y + k[2] * z
    return [x * ct + (k[1] * z - k[2] * y) * st + k[0] * d * (1 - ct),
            y * ct + (k[2] * x - k[0] * z) * st + k[1] * d * (1 - ct),
            z * ct + (k[0] * y - k[1] * x) * st + k[2] * d * (1 - ct)]


def wafer_rz_pivot(recipe):
    """WS_Rz **真值**旋转枢轴相对吸盘心的偏移（局部系 mm，(px,py)）：
    测校注入 recipe sim_align.rz_center_true_offset_um（µm）/1000；
    缺省 [0,0]=绕吸盘心（阿贝效应不注入）。认知侧补偿用的是机械常数
    fine_align.rz_center_offset_mm（标定值）——两者不等时补偿残余被
    精对准确认环实测抓到（真实测校行为）。"""
    sa = (recipe or {}).get("sim_align") or {}
    off = sa.get("rz_center_true_offset_um") or [0.0, 0.0]
    return (float(off[0]) / 1000.0, float(off[1]) / 1000.0)


def _rotvec_apply_pivot(p, r, pivot):
    """绕局部系枢轴 pivot=(px,py) 旋转：q + R(p−q)。pivot≈0 时与
    _rotvec_apply 完全等价（默认路径零开销零差异）。"""
    if not pivot or (abs(pivot[0]) < 1e-15 and abs(pivot[1]) < 1e-15):
        return _rotvec_apply(p, r)
    w = _rotvec_apply([p[0] - pivot[0], p[1] - pivot[1], p[2]], r)
    return [w[0] + pivot[0], w[1] + pivot[1], w[2]]


def _module_pose(poses, module):
    """从 kinematics.compute() 结果取模块位姿：pos=名义+误差，rot=姿态误差。"""
    m = poses["modules"][module]
    pos = [m["nominal_mm"][k] + m["error_mm"][k] for k in range(3)]
    return pos, list(m["error_rad"])


class Mark(object):
    __slots__ = ("name", "shape", "x", "y", "z", "yaw", "size_mm",
                 "blur_ratio", "focus_err_um", "virtual")

    def __init__(self, name, shape, x, y, z, yaw, size_mm, virtual=False):
        self.name = name
        self.shape = shape
        self.x, self.y, self.z = x, y, z
        self.yaw = yaw                    # 世界系 yaw（rad）
        self.size_mm = size_mm
        self.blur_ratio = 0.0
        self.focus_err_um = 0.0
        # virtual=True：解析检出特征（不画进图像）——如芯片背面轮廓检出，
        # 图像里背面就是素板无标记，该特征只供 FAST 解析/引导使用
        self.virtual = virtual


def camera_pose(engine, cam, poses=None, positions=None):
    """相机中心世界坐标 + 相机 yaw（物理安装旋转 view_rot，真值——相机
    的真实世界朝向；迭代 39 起不再用控制器认知 rot）。
    mount.module=None → 固连机架（offset 即世界坐标）；否则 = 模块 6DOF 位姿
    （名义+误差）+ 随模块姿态旋转的 offset。"""
    if cam.mount_module is None:
        return list(cam.mount_offset), cam.view_rot
    if poses is None or positions is None:
        positions, poses = _engine_poses(engine)
    pos, rot = _module_pose(poses, cam.mount_module)
    # 对准台名义姿态（AS_Rx/Ry 轴值，小角）并入
    if cam.mount_module == "align_stage":
        rot = [rot[0] + positions.get("AS_Rx", 0.0),
               rot[1] + positions.get("AS_Ry", 0.0), rot[2]]
    off = _rotvec_apply(cam.mount_offset, rot)
    return [pos[k] + off[k] for k in range(3)], cam.view_rot + rot[2]


def _engine_poses(engine):
    """一致读取：轴位快照 + 6DOF 模块位姿（无 kinematics 时名义姿态）。"""
    with engine.lock:
        positions = {n: ax.pos for n, ax in engine.axes.items()}
    if engine.kinematics is not None:
        return positions, engine.kinematics.compute(positions)
    # 无 6DOF 层时退化：直接给零误差（一般不发生，main 总会注入）
    raise RuntimeError("vision 需要 engine.kinematics（6DOF 模块位姿来源）")


def collect_marks(engine, cam, positions=None, poses=None, ir_overlay=False):
    """按相机 marks.source 收集当前可见标记（世界位姿列表）。
    工件存在性决定标记是否存在：晶圆在片 / die 在 film 格 / die 在 head。
    标记几何全部来自处方：wafer.marks（全局，位置+size_um）、
    wafer.site_marks（每 site，ratio×site 半宽，FOV 剪枝）与 film.marks
    （ratio×die 半宽，等比缩放；film/flip/head/wafer 上的 die 共用）。

    ir_overlay=True（2026-09-05 迭代 15，仅 wafer_marks 源=下相机有效）：
    键合后检测红外套刻通道——IR 穿透一个芯片厚度成像，与可见光通道
    两点差异：①被键合 die 覆盖的 site 对准标记（quad）**恢复可见**
    （可见光下物理遮挡）；②wafer_die 层 die 角标（cross）的 z 按晶圆
    面计（IR 假设：焦面=晶圆面（下芯片表面）时，晶圆面 quad 与 die
    角标 cross 在同一画幅内都算合焦——远心投影 z 不影响 (x,y)，clamp
    只改离焦量），一张图形成 quad⊕cross 套刻图。可见光通道
    （EGA/调平/精对准）传 False，行为不变。"""
    if positions is None or poses is None:
        positions, poses = _engine_poses(engine)
    wp = engine.workpieces
    recipe = engine.recipe.get() if engine.recipe is not None else None
    marks = []
    if cam.mark_source == "wafer_marks":
        if wp is None or recipe is None or not wp.wafer["present"]:
            return []
        center, rot = _module_pose(poses, "wafer_stage")
        rz = positions.get("WS_Rz", 0.0)
        rot = [rot[0], rot[1], rot[2] + rz]
        # WS_Rz 真值旋转枢轴（迭代 6）：默认 [0,0]=绕吸盘心（与历史行为
        # 一致）；测校注入 sim_align.rz_center_true_offset_um 时绕
        # 吸盘心+真值偏移旋转——阿贝效应可仿真（confocal 侧同步）
        pivot = wafer_rz_pivot(recipe)
        wthk = float(recipe["wafer"]["thickness_mm"])
        z_off = cam.surface_z_off + wthk
        with wp._lock:
            off = list(wp.wafer["offset"])
            wrz = float(wp.wafer.get("rz", 0.0))   # 上片旋转误差（rad）
            dies = [dict(d) for d in wp.dies if d["carrier"] == "wafer"]

        def _load_rot(x, y):
            """晶圆设计系 → 吸盘系：绕晶圆心旋 wrz（上片 Rz 误差）。"""
            if not wrz:
                return x, y
            c, s = math.cos(wrz), math.sin(wrz)
            return c * x - s * y, s * x + c * y

        for mk in recipe["wafer"]["marks"]:
            mx, my = _load_rot(float(mk["x_mm"]), float(mk["y_mm"]))
            local = [off[0] + mx, off[1] + my, z_off]
            w = _rotvec_apply_pivot(local, rot, pivot)
            marks.append(Mark(mk["name"], mk["shape"],
                              center[0] + w[0], center[1] + w[1],
                              center[2] + w[2], rz + wrz,
                              float(mk["size_um"]) / 1000.0))
        # 每 site 对准标记 site_R{r}C{c}#{k}（产品片每 chip site 自带）：
        # 位置 = site 中心 + ratio×site 半宽，z=晶圆面（与全局 marks 同层
        # 同 z），yaw=WS_Rz+模块姿态。默认处方 ~2700 sites × 4 ≈ 1.1 万
        # 标记，按 site 中心与相机距离先剪枝（FOV 半对角+site 半对角+余量
        # 1mm）再构造 Mark——全量构造每帧几十 ms，会把 cameras/guide 轮询
        # 拖慢
        sm = recipe["wafer"].get("site_marks")
        if sm:
            from recipe import wafer_sites_cells, wafer_sites_chip_size
            sratio = sm.get("ratio") or []
            sper = min(max(0, int(sm.get("per_site", len(sratio)))),
                       len(sratio))
            if sper:
                shx, shy = (0.5 * v for v in wafer_sites_chip_size(recipe))
                ssize = float(sm.get("size_um", 150)) / 1000.0
                sshape = sm.get("shape", "box")
                cam_xy, _yaw = camera_pose(engine, cam, poses, positions)
                fx2, fy2 = cam.fov_mm
                reach = 0.5 * math.hypot(fx2, fy2) \
                    + math.hypot(shx, shy) + 1.0
                # 已被键合 die 覆盖的 site 不出标记（die 物理盖住 site 标记）
                # ——ir_overlay 红外套刻通道例外：IR 穿透 die，被覆盖 site 的
                # quad 标记恢复可见（迭代 15）
                covered = [] if ir_overlay else \
                    [(d["site"] or [0.0, 0.0]) for d in dies]
                for r_, c_, sx_, sy_ in wafer_sites_cells(recipe):
                    csx, csy = _load_rot(sx_, sy_)
                    if any(math.hypot(csx - cs[0], csy - cs[1])
                           < min(shx, shy) for cs in covered):
                        continue
                    w = _rotvec_apply_pivot([off[0] + csx, off[1] + csy, 0.0],
                                            rot, pivot)
                    wx, wy = center[0] + w[0], center[1] + w[1]
                    if abs(wx - cam_xy[0]) > reach \
                            or abs(wy - cam_xy[1]) > reach:
                        continue
                    for k in range(sper):
                        lx = float(sratio[k][0]) * shx
                        ly = float(sratio[k][1]) * shy
                        mxx, myy = _load_rot(sx_ + lx, sy_ + ly)
                        w = _rotvec_apply_pivot(
                            [off[0] + mxx, off[1] + myy, z_off],
                            rot, pivot)
                        marks.append(Mark(
                            "site_R%02dC%02d#%d" % (r_, c_, k), sshape,
                            center[0] + w[0], center[1] + w[1],
                            center[2] + w[2], rz + wrz, ssize))
        # 已键合到晶圆上的 die 的标记（die_id#k）：几何同 film.marks，
        # 随 die site/rz + WS_Rz + 模块姿态；标记面 = 晶圆面 + die 厚
        # （z 高于 recipe 晶圆 marks，渲染按 z 归属 wafer_die 层单独随层）
        # die 物理落在晶圆真实 site 上：整颗 die（位+向）随上片旋转 wrz
        # ——site 心过 _load_rot，且 die 自身角标分布也要带 wrz（die_rz
        # 含 wrz，只转心不转姿态等价于标记没随片，迭代 11b 根因）；再加
        # 键合物理残差 die["resid"]（bond_mark 落笔的 fine_align
        # phys_resid，设计系 mm/rad）。
        if dies:
            dthk = float(recipe["film"]["dies"].get("thickness_mm", 0.5))
            # ir_overlay 红外套刻：die 角标 z 按晶圆面计（IR 假设——焦面=
            # 晶圆面时 quad/cross 同画幅都合焦；远心投影 z 不影响 (x,y)）
            z_die = z_off if ir_overlay else z_off + dthk
            for d in dies:
                site = d["site"] or [0.0, 0.0]
                drz = float(d.get("rz", 0.0))
                res = d.get("resid") or [0.0, 0.0, 0.0]
                csx, csy = _load_rot(site[0] + res[0], site[1] + res[1])
                drz_eff = wrz + drz + float(res[2])
                marks.extend(_die_marks(
                    d["id"], [off[0] + csx, off[1] + csy],
                    rz + drz_eff, center, rot, z_die, recipe,
                    die_rz=drz_eff, pivot=pivot))
    elif cam.mark_source == "flip_die_marks":
        if wp is None or recipe is None:
            return []
        # 位姿约定（2026-08-25 整机零位修正）：FA_Rx=0 吸盘面朝上=呈现位，
        # die 贴朝上面且**背面朝上**（标记面贴吸盘朝下不可见）；FA_Rx=±π 取片位，die
        # 挂吸盘面下被本体遮挡。cos(FA_Rx)≤0（朝下/侧立）→ 相机只见吸盘
        rx = float(positions.get("FA_Rx", 0.0))
        if math.cos(rx) <= 0.0:
            return []
        rz = float(positions.get("CH_Rz", 0.0))
        center, rot = _module_pose(poses, "flip")
        xf = _flip_transform(center, rot, rx, rz)
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "flip"]
        hx, hy = _die_half_size(recipe)
        for d in dies:
            site = d["site"] or [0.0, 0.0]
            drz = float(d.get("rz", 0.0))
            w = xf([site[0], site[1], cam.surface_z_off])
            # 背面检出特征（虚拟，不画进图像）：素板背面无标记，粗对准
            # 靠 die 本体轮廓检出——特征=背面中心，尺寸=die 外廓
            vx = xf([site[0] + math.cos(drz), site[1] + math.sin(drz),
                     cam.surface_z_off])
            yaw = math.atan2(vx[1] - w[1], vx[0] - w[0])
            marks.append(Mark("%s#back" % d["id"], "box",
                              w[0], w[1], w[2], yaw,
                              2.0 * max(hx, hy), virtual=True))
    elif cam.mark_source == "film_die_marks":
        if wp is None or recipe is None or not wp.film["present"]:
            return []                       # 蓝膜不在位 → 无 die 标记
        center, rot = _module_pose(poses, "film_stage")
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "film"]
        for d in dies:
            marks.extend(_die_marks(d["id"], d["film_pos"], 0.0,
                                    center, rot, cam.surface_z_off,
                                    recipe))
    elif cam.mark_source == "head_die_marks":
        if wp is None or recipe is None:
            return []
        center, rot = _module_pose(poses, "bond_head")
        # 键合头名义姿态（BS_CAL_Rx/Ry 轴值）并入
        rot = [rot[0] + positions.get("BS_CAL_Rx", 0.0),
               rot[1] + positions.get("BS_CAL_Ry", 0.0), rot[2]]
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "head"]
        # 标记在 die 下表面（贴晶圆侧）= 吸盘尖端面 − die 厚
        # （2026-08-27 修复：原按 surface_z_off=0 把标记放在吸盘尖端面
        # ——那是 die 顶面，up 相机对 die 底面进焦时标记恒离焦 t_d
        # （默认 500µm）判废，align_first 先对准暴露）
        t_d = float(recipe["film"]["dies"]["thickness_mm"])
        if not dies:
            # 吸盘中心 cross 标记（迭代 42 as_head_center/best_focus(up)
            # 测校实体）：无 die 时恒在——键合头吸盘尖端面（collet）真空
            # 十字气道的交叉中心（物理实体；场景层 collet 已渲染
            # 2.56mm 宽槽纹，150µm 模板无法匹配槽纹尺度——FAST 解析标
            # 记实体按 tpl_head_chuck 同尺度 150µm cross 补出，REAL 由
            # 渲染链按标记绘制同尺度 cross，两模式均可识别）。有 die 时
            # 吸盘心被 die 物理覆盖，不出标记（与 wafer site 标记被已
            # 键合 die 覆盖同口径）。
            cz = center[2] + _rotvec_apply(
                [0.0, 0.0, cam.surface_z_off], rot)[2]
            marks.append(Mark("head_chuck_cross", "cross",
                              center[0], center[1], cz, rot[2], 0.15))
        for d in dies:
            marks.extend(_die_marks(d["id"], d["site"] or [0.0, 0.0], 0.0,
                                    center, rot, cam.surface_z_off - t_d,
                                    recipe))
    return marks


def _die_half_size(recipe):
    """芯片半宽 (hx, hy) mm（film.dies 的芯片大小；wafer_sites 跟随同一尺寸）。"""
    dsz = recipe["film"]["dies"]
    return (0.5 * float(dsz.get("size_x_mm", 4.0)),
            0.5 * float(dsz.get("size_y_mm", 4.0)))


def _flip_transform(center, rot, rx, rz):
    """翻转模块局部（相对工具点）→ 世界的变换闭包。链序 FA_X→FA_Z→FA_Rx→
    CH_Rz：先 Rx(FA_Rx) 再 Rz(CH_Rz)，最后叠模块 error_rad（小角 rotvec
    求和，同 _module_pose 风格）；平移 = 模块工具点（名义+误差）。
    吸盘面局部 z=±45.4（工具点=91mm 方板 AABB 中心 z=937；呈现位 Rx=0
    吸盘面朝上 z=982.4，取片位 Rx=−π 吸盘面朝下 z=891.6=chuck_face_z0）。
    die 背面（检出面/粗对准可见面）= +45.4+die 厚（呈现位 z=982.9 进焦），
    surface_z_offset_mm 按 +45.9 配置（2026-08-25 整机零位约定）。"""
    def xf(p):
        w = _rotvec_apply(p, [rx, 0.0, 0.0])
        w = _rotvec_apply(w, [0.0, 0.0, rz])
        w = _rotvec_apply(w, rot)
        return [center[0] + w[0], center[1] + w[1], center[2] + w[2]]
    return xf


def _die_marks(die_id, die_pos, die_yaw, center, rot, z_off, recipe,
               die_rz=0.0, pivot=None):
    """一颗 die 的标记列表：film.marks 的 ratio 前 per_die 个 × die 半宽
    （die 尺寸变则标记位置等比缩放，与参考项目 marks.ratio 同语义）。
    die_rz = die 自身在载体平面内的转角（rad）：ratio 偏移先随 die 转，
    再加载体局部位。pivot=WS_Rz 真值旋转枢轴（仅 wafer 载体侧传，
    迭代 6）。"""
    fm = recipe["film"]["marks"]
    hx, hy = _die_half_size(recipe)
    ratio = fm.get("ratio") or []
    per = min(max(0, int(fm.get("per_die", len(ratio)))), len(ratio))
    size_mm = float(fm.get("size_um", 150)) / 1000.0
    shape = fm.get("shape", "box")
    out = []
    cd, sd = math.cos(die_rz), math.sin(die_rz)
    for i in range(per):
        rx, ry = float(ratio[i][0]), float(ratio[i][1])
        lx, ly = rx * hx, ry * hy
        if die_rz:
            lx, ly = cd * lx - sd * ly, sd * lx + cd * ly
        local = [die_pos[0] + lx, die_pos[1] + ly, z_off]
        w = _rotvec_apply_pivot(local, rot, pivot)
        out.append(Mark("%s#%d" % (die_id, i), shape,
                        center[0] + w[0], center[1] + w[1],
                        center[2] + w[2], die_yaw, size_mm))
    return out


# ---------------- 场景层（视场内载体/工件表面，世界系） ----------------
def _rect_corners(cx, cy, hx, hy, yaw):
    """世界系矩形四角点（局部半宽 hx/hy，绕中心 yaw rad）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * lx - s * ly, cy + s * lx + c * ly)
            for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def _film_face_disc(recipe, center):
    """蓝膜面圆盘（世界系）：直径=film.diameter_mm，圆心=模块中心
    （film 局部原点=膜面圆心；圆旋转不变，无需叠姿态）。"""
    return ("disc", SCENE_CARRIER, center[0], center[1],
            0.5 * float(recipe["film"]["diameter_mm"]))


def _chuck_groove_items(center, rot, cam, cam_pos):
    """陶瓷吸盘面真空气路纹路（静态图案：同心环槽 + 4 条径向槽）。
    图案是吸盘局部系常数（模块顶部 CHUCK_GROOVE_* 定义，即"静态缓存"）；
    同心环旋转不变 → 世界位姿只需模块中心平移，径向槽加 WS_Rz/姿态偏航。
    每帧只产出与 FOV 相交的环（|r − dist(槽心,相机)| ≤ FOV 半对角+槽宽）。"""
    g = SCENE_CARRIER * CHUCK_GROOVE_GRAY
    cx, cy = center[0], center[1]
    dist = math.hypot(cam_pos[0] - cx, cam_pos[1] - cy)
    fx, fy = cam.fov_mm
    reach = 0.5 * math.hypot(fx, fy) + CHUCK_GROOVE_W
    items = []
    for r in CHUCK_GROOVE_RADII:
        if abs(r - dist) <= reach:
            items.append(("ring", g, cx, cy, float(r), CHUCK_GROOVE_W))
    yaw = rot[2]
    hw = 0.5 * CHUCK_GROOVE_W
    for a in CHUCK_SLOT_ANGLES:
        dx, dy = math.cos(a + yaw), math.sin(a + yaw)
        nx, ny = -dy, dx
        pts = [(cx + dx * CHUCK_SLOT_R0 + nx * hw,
                cy + dy * CHUCK_SLOT_R0 + ny * hw),
               (cx + dx * CHUCK_SLOT_R1 + nx * hw,
                cy + dy * CHUCK_SLOT_R1 + ny * hw),
               (cx + dx * CHUCK_SLOT_R1 - nx * hw,
                cy + dy * CHUCK_SLOT_R1 - ny * hw),
               (cx + dx * CHUCK_SLOT_R0 - nx * hw,
                cy + dy * CHUCK_SLOT_R0 - ny * hw)]
        items.append(("poly", g, pts))
    return items


def collect_layers(engine, cam, positions=None, poses=None, cam_pos=None,
                   ir_overlay=False):
    """视场**分层表面**（回→前有序）：[{name, z_mm, items}]，每层一个世界 Z
    （含载体 6DOF 位姿、recipe.wafer.thickness_mm 等），items 为世界系绘制项
    （("fill",g) / ("disc",g,cx,cy,r) / ("poly",g,[(x,y)...]) /
    ("ring",g,cx,cy,r,w)，g = 相对基准灰度比例）。几何全部来自已有物料/载体
    状态（workpieces + recipe + kinematics），不新造事实源；超出 FOV 的项由
    绘制侧裁剪。无工件 = 只剩载体面层/暗背景；蓝膜不在位 = coarse 无层。
    ir_overlay=True（迭代 15，仅 wafer_marks 源）：红外套刻通道——wafer_die
    层 z 按晶圆面计（IR 假设：焦面=晶圆面时 die 面层与晶圆面同焦，REAL
    渲染两层都清晰）。"""
    if positions is None or poses is None:
        positions, poses = _engine_poses(engine)
    if cam_pos is None:
        cam_pos, _yaw = camera_pose(engine, cam, poses, positions)
    wp = engine.workpieces
    recipe = engine.recipe.get() if engine.recipe is not None else None
    if cam.mark_source == "wafer_marks":
        center, rot = _module_pose(poses, "wafer_stage")
        rot = [rot[0], rot[1], rot[2] + positions.get("WS_Rz", 0.0)]
        # 陶瓷吸盘面（机器部件、恒在且远大于 FOV）→ 整帧铺满 + 气路纹路
        chuck_z = center[2] + _rotvec_apply(
            [0.0, 0.0, cam.surface_z_off], rot)[2]
        layers = [{"name": "chuck", "z_mm": chuck_z,
                   "items": [("fill", SCENE_CARRIER)]
                            + _chuck_groove_items(center, rot, cam, cam_pos)}]
        if wp is None or recipe is None or not wp.wafer["present"]:
            return layers
        with wp._lock:
            off = list(wp.wafer["offset"])
            wrz = float(wp.wafer.get("rz", 0.0))
            dies = [dict(d) for d in wp.dies if d["carrier"] == "wafer"]
        thk = float(recipe["wafer"]["thickness_mm"])
        wc = _rotvec_apply([off[0], off[1], 0.0], rot)
        wafer_z = center[2] + _rotvec_apply(
            [off[0], off[1], cam.surface_z_off + thk], rot)[2]
        layers.append({"name": "wafer", "z_mm": wafer_z,
                       "items": [("disc", SCENE_WAFER, center[0] + wc[0],
                                  center[1] + wc[1],
                                  0.5 * float(recipe["wafer"]["diameter_mm"]))]})
        if dies:
            hx, hy = _die_half_size(recipe)
            ditems = []
            # 与 collect_marks wafer_die 标记同源：site 设计上片旋转 wrz +
            # 键合物理残差 die["resid"]（迭代 11b，勿再只画 off+site 名义位）
            cw_, sw_ = math.cos(wrz), math.sin(wrz)
            for d in dies:                  # 已键合到晶圆上的 die（随片旋转）
                if ir_overlay:
                    # IR 穿透 die：本体不遮光（2026-09-05 迭代 17 实测 REAL
                    # 红外帧根因——die 本体后画盖住 wafer 层 quad 与按 z 并列
                    # 时宿主判到 wafer 层的 cross，两个套刻标记全被遮没，
                    # NCC 无峰）。只跳本体，die 角标 cross 仍随 wafer_die
                    # 层绘制（标记绘制在层 items 之后，不受影响）
                    continue
                site = d["site"] or [0.0, 0.0]
                res = d.get("resid") or [0.0, 0.0, 0.0]
                ex_, ey_ = site[0] + res[0], site[1] + res[1]
                csx, csy = (cw_ * ex_ - sw_ * ey_, sw_ * ex_ + cw_ * ey_)
                dc = _rotvec_apply([off[0] + csx, off[1] + csy, 0.0], rot)
                ditems.append(("poly", SCENE_DIE, _rect_corners(
                    center[0] + dc[0], center[1] + dc[1], hx, hy,
                    rot[2] + wrz + float(d.get("rz", 0.0)) + float(res[2]))))
            dthk = float(recipe["film"]["dies"].get("thickness_mm", 0.5))
            layers.append({"name": "wafer_die",
                           "z_mm": wafer_z if ir_overlay else wafer_z + dthk,
                           "items": ditems})
        return layers
    if cam.mark_source == "flip_die_marks":
        # 粗对准相机：对翻转模块上的 die。回→前：蓝膜背景层（在位才画膜面
        # 圆盘；膜面 z=886.9 距焦面 ~96mm 远超景深，σ 封顶成均匀亮雾——
        # 膜上 die 亚分辨率不可分辨，不画）→ 翻转吸盘方板 → 翻转 die 层。
        layers = []
        if wp is not None and recipe is not None and wp.film["present"]:
            fcenter, _frot = _module_pose(poses, "film_stage")
            layers.append({"name": "film", "z_mm": fcenter[2],
                           "items": [_film_face_disc(recipe, fcenter)]})
        if wp is None or recipe is None:
            return layers
        center, rot = _module_pose(poses, "flip")
        rx = float(positions.get("FA_Rx", 0.0))
        rz = float(positions.get("CH_Rz", 0.0))
        yaw = rz + rot[2]
        # 翻转吸盘层：91mm 方板（±45.4 对称，任意 FA_Rx 朝上面恒
        # z=工具点+45.4），板面淡真空槽纹路（中心环 + 十字槽）
        g = SCENE_CARRIER * FLIP_CHUCK_GRAY
        cx, cy = center[0], center[1]
        items = [("poly", SCENE_CARRIER,
                  _rect_corners(cx, cy, FLIP_CHUCK_HALF, FLIP_CHUCK_HALF,
                                yaw)),
                 ("ring", g, cx, cy, FLIP_CHUCK_RING_R, FLIP_CHUCK_GROOVE_W),
                 ("poly", g, _rect_corners(cx, cy, FLIP_CHUCK_CROSS_ARM,
                                           0.5 * FLIP_CHUCK_GROOVE_W, yaw)),
                 ("poly", g, _rect_corners(cx, cy, 0.5 * FLIP_CHUCK_GROOVE_W,
                                           FLIP_CHUCK_CROSS_ARM, yaw))]
        layers.append({"name": "flip_chuck", "z_mm": center[2] + 45.4,
                       "items": items})
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "flip"]
        if dies and math.cos(rx) > 0.0:
            # 呈现位（Rx=0 附近，吸盘面朝上）：die 贴朝上面，**背面朝上**
            # （标记面贴吸盘朝下不可见）→ 只画素板本体，无标记
            hx, hy = _die_half_size(recipe)
            xf = _flip_transform(center, rot, rx, rz)
            ditems = []
            for d in dies:
                site = d["site"] or [0.0, 0.0]
                drz = float(d.get("rz", 0.0))
                cd, sd = math.cos(drz), math.sin(drz)
                pts = []
                for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                    wx, wy = cd * lx - sd * ly, sd * lx + cd * ly
                    w = xf([site[0] + wx, site[1] + wy, cam.surface_z_off])
                    pts.append((w[0], w[1]))
                ditems.append(("poly", SCENE_DIE, pts))
            layers.append({"name": "flip_die",
                           "z_mm": xf([0.0, 0.0, cam.surface_z_off])[2],
                           "items": ditems})
        return layers
    if cam.mark_source == "film_die_marks":
        if wp is None or recipe is None or not wp.film["present"]:
            return []                       # 蓝膜不在位 → 只剩暗背景
        center, rot = _module_pose(poses, "film_stage")
        # 蓝膜面（在位才画）先铺，再叠 film 上的 die 本体
        layers = [{"name": "film", "z_mm": center[2],
                   "items": [_film_face_disc(recipe, center)]}]
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "film"]
        if dies:
            hx, hy = _die_half_size(recipe)
            ditems = []
            for d in dies:
                dc = _rotvec_apply([d["film_pos"][0], d["film_pos"][1], 0.0],
                                   rot)
                ditems.append(("poly", SCENE_DIE,
                               _rect_corners(center[0] + dc[0],
                                             center[1] + dc[1],
                                             hx, hy, 0.0)))  # yaw 同标记（≈0）
            # die 面 = 蓝膜面 + die 厚（与 marks surface_z_offset_mm 一致）
            layers.append({"name": "film_die",
                           "z_mm": center[2] + cam.surface_z_off,
                           "items": ditems})
        return layers
    if cam.mark_source == "head_die_marks":
        if wp is None or recipe is None:
            return []
        center, rot = _module_pose(poses, "bond_head")
        # 键合头名义姿态（BS_CAL_Rx/Ry 轴值）并入（同 collect_marks）
        rot = [rot[0] + positions.get("BS_CAL_Rx", 0.0),
               rot[1] + positions.get("BS_CAL_Ry", 0.0), rot[2]]
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "head"]
        cx, cy = center[0], center[1]
        yaw = rot[2]
        # 吸盘底面层：恒在（无 die 也画），fill 铺底 + 真空气道网
        # （迭代 54：中心真空主孔 + 宽臂十字凹槽 + 密封缘内两圈环槽
        # +8 窄径向槽，取代通边大十字与实心台柱）。
        # 纹路按 BS_CAL_Rx/Ry 偏航旋转（与吸盘姿态一致；环旋转不变）。
        g = SCENE_CARRIER * COLLET_CROSS_GRAY
        collet_items = [
            ("poly", SCENE_CARRIER,
             _rect_corners(cx, cy, COLLET_HALF, COLLET_HALF, yaw))]
        # 中心真空主孔（圆孔，主抽气口；深孔比浅槽更暗）
        collet_items.append(("disc", SCENE_CARRIER * COLLET_HOLE_GRAY,
                             cx, cy, COLLET_HOLE_R))
        # 宽臂十字凹槽：主孔缘 → 内环槽（互联分配道），随 yaw 转
        gx = SCENE_CARRIER * COLLET_XGROOVE_GRAY
        x0 = COLLET_HOLE_R
        x1 = COLLET_RING_R[0] + 0.5 * COLLET_GROOVE_W
        for i in range(4):
            a = yaw + i * (math.pi / 2.0)
            xm = 0.5 * (x0 + x1)
            collet_items.append(("poly", gx, _rect_corners(
                cx + math.cos(a) * xm, cy + math.sin(a) * xm,
                0.5 * (x1 - x0), 0.5 * COLLET_XGROOVE_W, a)))
        for rr in COLLET_RING_R:
            collet_items.append(("ring", g, cx, cy, rr, COLLET_GROOVE_W))
        r0 = COLLET_RING_R[0] - 0.3               # 径向槽内端（自内环槽起）
        r1 = COLLET_RING_R[-1] + 1.0              # 径向槽外端（过外环交联）
        for i in range(COLLET_SPOKES):
            a = yaw + i * (2.0 * math.pi / COLLET_SPOKES)
            rm = 0.5 * (r0 + r1)
            collet_items.append(("poly", g, _rect_corners(
                cx + math.cos(a) * rm, cy + math.sin(a) * rm,
                0.5 * (r1 - r0), 0.5 * COLLET_SPOKE_W, a)))
        collet_z = center[2] + _rotvec_apply(
            [0.0, 0.0, cam.surface_z_off], rot)[2]
        layers = [{"name": "collet", "z_mm": collet_z,
                   "items": collet_items}]
        if dies:
            # die 下表面（标记/本体可见面）= 吸盘尖端面 − die 厚（同 collect_marks
            # 2026-08-27 修复）；die 本体盖在吸盘槽纹之上（相机朝上看，die
            # 面 z 更小=更靠近相机，按回→前顺序后画）。
            hx, hy = _die_half_size(recipe)
            die_items = []
            t_d = float(recipe["film"]["dies"]["thickness_mm"])
            for d in dies:
                site = d["site"] or [0.0, 0.0]
                dc = _rotvec_apply([site[0], site[1], 0.0], rot)
                dcx, dcy = center[0] + dc[0], center[1] + dc[1]
                die_items.append(("poly", SCENE_DIE,
                                  _rect_corners(dcx, dcy, hx, hy, 0.0)))
            layers.append({"name": "head_die",
                           "z_mm": center[2] + cam.surface_z_off - t_d,
                           "items": die_items})
        return layers
    return []


def collect_scene(engine, cam, positions=None, poses=None):
    """扁平场景项列表（旧接口，= collect_layers 各层 items 顺序拼接）。"""
    return [item for layer in collect_layers(engine, cam, positions, poses)
            for item in layer["items"]]


def _layer_px_bbox(cam, items, cam_pos, W, H, k, marks_px=None):
    """层内容在渲染分辨率下的像素 bbox (x0,y0,x1,y1)（已裁剪到画幅）；
    无任何可见内容返回 None。"fill" 恒为整幅。marks_px=该层宿主的标记
    投影像素 [(col,row,margin)]（2026-09-05 迭代 17：IR 层 items 可为空
    ——die 本体不遮光，但层上 cross 标记仍须参与模糊 bbox）。"""
    if not items and not marks_px:
        return None
    x0 = y0 = float("inf")
    x1 = y1 = float("-inf")
    for item in items:
        kind = item[0]
        if kind == "fill":
            return (0, 0, W, H)
        if kind in ("disc", "ring"):
            cx, cy, r = item[2], item[3], item[4]
            wmm = item[5] if kind == "ring" else 0.0
            col, row = cam.project(cx - cam_pos[0], cy - cam_pos[1], W, H)
            rp = (r + wmm) / k
            x0, x1 = min(x0, col - rp), max(x1, col + rp)
            y0, y1 = min(y0, row - rp), max(y1, row + rp)
        else:                                   # poly
            for px, py in item[2]:
                col, row = cam.project(px - cam_pos[0], py - cam_pos[1], W, H)
                x0, x1 = min(x0, col), max(x1, col)
                y0, y1 = min(y0, row), max(y1, row)
    for col, row, mg in (marks_px or []):
        x0, x1 = min(x0, col - mg), max(x1, col + mg)
        y0, y1 = min(y0, row - mg), max(y1, row + mg)
    x0, x1 = max(0, int(math.floor(x0))), min(W, int(math.ceil(x1)))
    y0, y1 = max(0, int(math.floor(y0))), min(H, int(math.ceil(y1)))
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1, y1)


def surfaces_public(cam, layers, cam_pos):
    """分层景深状态（cameras_public / identify entry 的 surfaces 字段）：
    每可见表面层 {name, z_mm, defocus_um, in_dof}（FOV 内出现才算可见，
    按层几何像素 bbox 与画幅相交判定）。"""
    focus_z = cam_pos[2] + cam.look_dir * cam.wd_mm
    out = []
    for layer in layers:
        if _layer_px_bbox(cam, layer["items"], cam_pos,
                          cam.px_x, cam.px_y, cam.k_mm) is None:
            continue
        defocus = (layer["z_mm"] - focus_z) * 1000.0
        out.append({"name": layer["name"], "z_mm": round(layer["z_mm"], 4),
                    "defocus_um": round(defocus, 1),
                    "in_dof": bool(abs(defocus) <= cam.dof_um)})
    return out


def apply_focus(cam, cam_z, marks):
    """给每个标记算 focus_err / blur_ratio。返回焦面 Z。"""
    focus_z = cam_z + cam.look_dir * cam.wd_mm
    for m in marks:
        err_mm = m.z - focus_z
        m.focus_err_um = err_mm * 1000.0
        m.blur_ratio = abs(m.focus_err_um) / cam.dof_um
    return focus_z


# ---------------- 形状几何（mark 局部 mm 多边形） ----------------
def shape_primitives(mark):
    """标记 → 图元列表（mark 局部坐标 mm，未加 yaw）：
    ("rect", cx, cy, w, h) / ("ring", cx, cy, r_out, r_in)。"""
    s = mark.size_mm
    if mark.shape == "cross":
        a = s * ARM_RATIO
        return [("rect", 0.0, 0.0, s, a), ("rect", 0.0, 0.0, a, s)]
    if mark.shape == "box":
        return [("rect", 0.0, 0.0, s, s)]
    if mark.shape == "circle":
        r = s * 0.5
        return [("ring", 0.0, 0.0, r, r * CIRCLE_INNER)]
    if mark.shape == "quad":
        # 套刻（overlay）标记（2026-09-04 迭代 13）：150µm 见方边界内四角
        # 各一个小方块（边长 0.2s，s=150µm 时 30µm），外缘贴边界；
        # FAST 结果点=几何中心（entry_points 统一中心点）
        q = s * 0.2
        off = 0.5 * (s - q)
        return [("rect", -off, -off, q, q), ("rect", off, -off, q, q),
                ("rect", -off, off, q, q), ("rect", off, off, q, q)]
    return []


def entry_points(mark):
    """FAST 结果点（mark 局部 mm）：每个标记一个中心点（box/cross/circle/
    quad——quad 四角小方块几何中心即标记中心）。"""
    return [(0.0, 0.0)]


# ---------------- 渲染器 ----------------
_VIGNETTE_CACHE = {}
# 单位法线噪声底版（2026-09-05 迭代 17：9MP 下 n.random.normal 全幅生成
# ~150ms 是 render 大头之一；按 (W,H) 缓存底版，内参/分辨率不变时零重建）。
# 迭代 27 起底版为 2 张 (H+512)×(W+512) 大版，每帧随机偏移 (dy,dx)∈
# [0,512]² 取 (H,W) **numpy 视图**（零逐帧生成、零拷贝）——旧版 2 张整幅
# 底版逐帧轮换，同机位连拍只剩 2~3 个离散亚像素值，测校静态重复性 3σ 呈
# 2~3nm 量化锯齿（不真实）；大版随机取景后帧间去相关（实测 NCC 亚像素散布
# 回到 ~0.03px 量级）。迭代 28：底版存 **N(0,1) 单位噪声**（与 σ 解耦——
# 换噪声参数不换桶不重建，每帧 img += view×σ）。内存：9MP 相机每分辨率
# 2×~49MB float32，桶数由 _NOISE_CACHE 防御性清空兜底
_NOISE_CACHE = {}
_NOISE_POOL = 2
_NOISE_PAD = 512                    # 大底版四向外扩 px（随机取景范围）
_NOISE_LOCK = threading.Lock()      # 桶创建/轮换（多相机渲染线程并发）
_OFFSET_RNG = random.Random()       # 取景偏移源（免播种：只需帧间去相关）


def _noise_unit(n, W, H):
    """按 (W,H) 缓存的 N(0,1) **单位**噪声底版；返回 (H,W) 视图，调用方
    乘 σ 后叠加（img += view×σ）。迭代 27 机制保留：(H+512)×(W+512) 大版
    每帧随机偏移取景（帧间去相关），双底版轮换保持相邻帧不落同一张。视图
    非连续切片，numpy 乘加兼容。"""
    key = (W, H)
    with _NOISE_LOCK:
        ent = _NOISE_CACHE.get(key)
        if ent is None:
            ent = {"pool": [_rng().normal(0.0, 1.0,
                                          (H + _NOISE_PAD, W + _NOISE_PAD))
                            .astype(n.float32)
                            for _ in range(_NOISE_POOL)], "cur": 0}
            if len(_NOISE_CACHE) >= 6:      # 分辨率组合有限，防御性清空
                _NOISE_CACHE.clear()
            _NOISE_CACHE[key] = ent
        ent["cur"] = (ent["cur"] + 1) % _NOISE_POOL
        tile = ent["pool"][ent["cur"]]
        dy = _OFFSET_RNG.randrange(_NOISE_PAD + 1)
        dx = _OFFSET_RNG.randrange(_NOISE_PAD + 1)
    return tile[dy:dy + H, dx:dx + W]


def _vignette(n, W, H):
    """照明渐晕因子阵 (H,W) float32：1 − VIGNETTE·r²/r_max²（中心 1.0，
    角落 1−VIGNETTE）。按分辨率缓存（mgrid 一次性开销）。"""
    key = (W, H)
    f = _VIGNETTE_CACHE.get(key)
    if f is None:
        yy, xx = n.mgrid[0:H, 0:W].astype(n.float32)
        cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
        r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (cx * cx + cy * cy + 1e-9)
        f = (1.0 - VIGNETTE * n.clip(r2, 0.0, 1.0)).astype(n.float32)
        if len(_VIGNETTE_CACHE) >= 8:       # 分辨率种类有限，防御性清空
            _VIGNETTE_CACHE.clear()
        _VIGNETTE_CACHE[key] = f
    return f


def real_noise_scale(engine, cam_id):
    """recipe sim_sensors.real_noise_scale 热读（迭代 27：REAL 渲染噪声 σ
    倍率，按相机 id 分路，(0,10] 默认 1.0——定性放大/缩小渲染噪声，
    非物理量；缺省/异常回退 1.0，照 vision_service fast_noise_px 先例）。"""
    rec = engine.recipe.get() if getattr(engine, "recipe", None) else None
    if not rec:
        return 1.0
    v = (rec.get("sim_sensors") or {}).get("real_noise_scale")
    if isinstance(v, dict):
        v = v.get(cam_id, 1.0)
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 1.0
    return v if v > 0.0 else 1.0


def fast_noise_px(engine, cam_id):
    """recipe sim_sensors.fast_noise_px 热读（px，默认 0.15；2026-09-10
    迭代 28 语义升级：**相机质心噪声底**，FAST 解析链与 REAL 渲染链共用
    同一旋钮——REAL 侧由 render_frame 按模板响应度换算成图像灰度 σ
    （噪声在图像里，不再是识别后加数值误差）；FAST 侧由 vision_service
    直接用作质心 σ）。缺省/异常回退 0.15。"""
    rec = engine.recipe.get() if getattr(engine, "recipe", None) else None
    dfl = 0.15 if cam_id == "coarse" else 0.1
    if not rec:
        return dfl
    v = (rec.get("sim_sensors") or {}).get("fast_noise_px")
    if isinstance(v, dict):
        v = v.get(cam_id, dfl)
    try:
        v = float(v)
    except (TypeError, ValueError):
        return dfl
    return v if v >= 0.0 else dfl


_RESP_FALLBACK_LOGGED = set()


def _vision_noise_resp(engine, cam_id):
    """REAL 渲染噪声换算的模板参考 → (resp px/DN, cap DN, scene_gain_ref)
    或 None：鸭嘴访问 vision_service.template_noise_resp（避免 import 环——
    vision_service 依赖本模块）。无视觉服务/无模板/标定不可用 → None
    （render_frame 回退 K_NOISE 旧公式）。scene_gain_ref（2026-09-08 结果
    倒推）：NCC 匹配链对随机亚像素平移的传递增益（随幅度非线性，探针
    两点标定线性模型 g(σ)=a+bσ 的 (a, b) 二元组），None=未标定（渲染侧
    用缺省 a=1, b=0）。"""
    vs = getattr(engine, "vision", None)
    fn = getattr(vs, "template_noise_resp", None) if vs is not None else None
    if fn is None:
        return None
    try:
        return fn(cam_id)
    except Exception:
        return None


def _unpack_noise_ref(ref):
    """噪声参考元组解包 → (resp, cap, scene_gain_ref)；兼容 2/3 元组。
    scene_gain_ref 为 (a, b) 或 None。"""
    if ref is None:
        return None, None, None
    if len(ref) >= 3:
        return ref[0], ref[1], ref[2]
    return ref[0], ref[1], None


def render_frame(engine, cam, width=None, height=None, light_on=True,
                 exposure_ms=None, gain=None, marks=None, cam_pos=None,
                 layers=None, positions=None, poses=None, noise_resp=None,
                 noise_sigma_dn=None, scene_shift_px=None):
    """按当前机器状态合成灰度图 -> numpy uint8 (H,W)。

    分层景深管线（顺序见模块 docstring）：暗背景铺底 → 回→前逐表面层
    （collect_layers；载体面→工件本体，标记随宿主层一起绘制）→ 每层按
    各自离焦量区域高斯模糊（σ≤0.05 的对焦层零额外开销）→ 渐晕 → 噪声。
    width/height 缺省全分辨率；预览可降采样（如 640×480，像素当量等比变粗，
    画面内容不变——FAST 的 live 预览也走这里，画面始终真实）。
    light_on=False → 全场景按比例压到近全黑（光源灭=暗图，匹配必败——
    噪声仍按当前配置叠加，暗图低灰度下相对显著）。marks/cam_pos/layers
    可由调用方按同一快照算好传入
    （REAL 全链路一致性）；缺省时本函数自取一次快照统一计算。
    noise_resp（迭代 28）：本次识别模板的噪声参考 (resp px/DN,
    tpl_std_dn)（REAL 识别链按触发模板名解析传入）；None=用该相机
    vision_marks 默认模板（鸭嘴经 engine.vision 查），仍无则回退
    K_NOISE 旧公式（记一次日志）。
    noise_sigma_dn（迭代 28，建模板现场标定用）：显式指定灰度噪声 σ
    （DN），绕过 fnp/resp 换算与 real_noise_scale 倍率——标定试验需要
    已知注入量。
    """
    n = np()
    cv2 = cv2_or_none()
    W = int(width or cam.px_x)
    H = int(height or cam.px_y)
    exposure = float(exposure_ms if exposure_ms is not None else cam.exposure_ms)
    gain = float(gain if gain is not None else cam.gain)
    if cam_pos is None:
        positions, poses = _engine_poses(engine)
        cam_pos, _yaw = camera_pose(engine, cam, poses, positions)
        marks = collect_marks(engine, cam, positions, poses)
        apply_focus(cam, cam_pos[2], marks)
        layers = collect_layers(engine, cam, positions, poses, cam_pos)
    elif layers is None:
        layers = collect_layers(engine, cam, positions, poses, cam_pos)
    marks = marks or []
    k = cam.k_mm * (cam.px_x / float(W))       # 渲染分辨率下的 mm/px
    base = BG_LIGHT if light_on else BG_DARK
    base_g = base * min(gain, 8.0)             # 场景各层灰度 = 比例 × base_g
    img = n.full((H, W), SCENE_VOID * base_g, dtype=n.float32)
    mark_gray = base * MARK_DARK
    focus_z = cam_pos[2] + cam.look_dir * cam.wd_mm
    # 标记随 z 就近的表面层一起绘制/模糊（同一相机的标记可分属不同层：
    # 下相机 recipe 晶圆 marks 在 wafer 层、已键合 die 标记在 wafer_die
    # 层，z 不同、离焦量不同）
    mark_host = {}
    if marks and layers:
        for i, m in enumerate(marks):
            if m.virtual:               # 虚拟特征不画进图像
                continue
            mark_host[i] = min(range(len(layers)),
                               key=lambda j: abs(layers[j]["z_mm"] - m.z))

    for li, layer in enumerate(layers or []):
        # 本层离焦量 → 高斯 σ（远心景深模型；σ≤0.05 视为对焦，直接画进整帧）
        dz_um = abs((layer["z_mm"] - focus_z) * 1000.0)
        blur = dz_um / cam.dof_um
        sigma = min(SIGMA_MAX, K_BLUR * blur)
        # 离焦对比度衰减（2026-09-08）：几何离焦超出景深后层内容**细节
        # 对比度**按 DOF/|dz| 滚降（1mm 离焦≈50×DOF 时标记应不可见——
        # 用户实测投诉"焦面移 1mm 仍高分识别"）；景深内（blur≤1）不衰
        # 减（MTF 名义全通带）。衰减方向=向本层亮基底收缩（细节消失、
        # 亮面保持——均匀大面的亮度无"对比度"可言，不随离焦变暗）
        att = 1.0 if blur <= 1.0 else 1.0 / blur
        tgt = img
        if sigma > 0.05 and cv2 is not None:
            tgt = img.copy()                 # 离焦层：离屏合成后区域模糊贴回
        for item in layer["items"]:
            _draw_scene_item(n, cv2, tgt, cam, item, cam_pos, W, H, k, base_g)
            time.sleep(0)   # 绘制在视觉线程，别压制 1kHz 物理线程
        marks_px = []
        cov_buf = None               # 本层标记覆盖率累积（出层一次性混合）
        for i, m in enumerate(marks):      # 标记随各自 z 归属层（同层同模糊）
            if mark_host.get(i) != li:
                continue                   # 虚拟特征无宿主层，天然跳过不画
            margin = m.size_mm / k + 4
            col, row = cam.project(m.x - cam_pos[0], m.y - cam_pos[1], W, H)
            if not cam.in_fov(col, row, margin, W, H):
                continue
            marks_px.append((col, row, margin))
            # 亚像素精灵绘制（2026-09-08）：误差链的 repeat/误差地图分量
            # （数十 nm=百分之几 px）只有在亚像素连续的渲染下才能进入
            # 测量——整像素 fillPoly 曾把测量量化成 ~0.7px 台阶。覆盖率
            # 先累积到 cov_buf、每层只做一次混合：物理线程 HIGHEST 忙等
            # 下每个 GIL 竞争点代价 ~3ms，逐标记混合会把渲染拖慢 3×
            if cv2 is not None:
                if cov_buf is None:
                    cov_buf = _cov_buffer(n, W, H)
                    cov_buf.fill(0.0)   # 逐层清零（上帧/上层内容已消费）
                _draw_mark_cov(n, cv2, cov_buf, m, col, row, k, W, H)
            else:
                for prim in shape_primitives(m):
                    _draw_prim(n, cv2, tgt, cam, m, prim, cam_pos, W, H, k,
                               mark_gray)
            time.sleep(0)   # 标记间让出 GIL：别压制 1kHz 物理线程
        if cov_buf is not None:
            # tgt·(1-cov) + mark_gray·cov（全 cv2 运算，3 次调用）
            d = cv2.subtract(float(mark_gray), tgt)
            cv2.multiply(d, cov_buf, dst=d)
            cv2.add(tgt, d, dst=tgt)
        if tgt is not img:
            _blur_layer_region(n, cv2, img, tgt, cam, layer["items"],
                               cam_pos, W, H, k, sigma, marks_px=marks_px,
                               att=att, base_g=base_g)
            time.sleep(0)
    # 照明渐晕（乘性：中心亮边缘暗，标记/场景一起衰减）
    img *= _vignette(n, W, H)
    time.sleep(0)
    # 相机质心噪声（2026-09-10 迭代 28）：recipe sim_sensors.
    # fast_noise_px[相机]（与 FAST 解析链同一旋钮，测校页"传感器仿真"卡，
    # 每帧热读）÷ 模板响应度 resp（px/DN，建模板时现场实测自标定存
    # meta）= 图像灰度噪声 σ_gray——**噪声在图像里**（不是识别后加数值
    # 误差），REAL 链静态重复性量级与面板配置直接对得上。简化模型：不做
    # 散粒/读出分解（旋钮语义=质心噪声底）。无响应度（未建模板/无 cv2
    # 标定）回退 K_NOISE 曝光/增益旧公式并记一次日志。fast_noise_px=0
    # → σ=0 跳过叠加（确定性回退，与是否有模板无关）。
    fnp = fast_noise_px(engine, cam.id)
    sig_shift_col = sig_shift_row = 0.0   # 全局亚像素平移 σ（col/row 各向异性）
    if noise_sigma_dn is not None:
        # 建模板现场标定（迭代 28）：显式 σ 注入，跳过 fnp/resp 换算与
        # real_noise_scale 倍率——试验需要已知注入量
        sig = float(noise_sigma_dn)
    elif fnp <= 0.0:
        sig = 0.0
    else:
        ref = noise_resp
        if ref is None:
            ref = _vision_noise_resp(engine, cam.id)
        resp, cap, _sg_ref = _unpack_noise_ref(ref)
        sg_col = sg_row = (1.0, 0.0)   # 平移增益模型 g(σ)=a+bσ（col/row 各向异性）
        if _sg_ref is not None:
            try:
                if _sg_ref[0]:
                    sg_col = (float(_sg_ref[0][0]), float(_sg_ref[0][1]))
                if _sg_ref[1]:
                    sg_row = (float(_sg_ref[1][0]), float(_sg_ref[1][1]))
            except (TypeError, ValueError, IndexError):
                pass
        rns = real_noise_scale(engine, cam.id)
        if resp is not None and resp > 1e-9:
            # 2026-09-08 架构修正（用户预算对不上 + 雪花过强两项投诉的
            # 根因）：NCC 匹配对纯白噪声的质心响应实测远小于 resp 标定
            # （40DN 口径的 resp 被粗网格亚像素伪影地板污染，低 σ 白噪
            # 声线性区响应近零——实测 fnp 线性区输出仅 3.5nm vs 预测
            # 13.8nm），白噪声无法可靠承担质心噪声预算。改为：
            # ①白噪声固定雪花视觉档（不随 fnp 换算，σ=min(6DN, 0.2×
            # tpl_std)——用户投诉 40DN 雪花过强）；②质心噪声预算全额
            # 走全局亚像素平移分量（实测 0.05/0.1px 输出=预算 ±16%），
            # 即图像上标记抖动 σ=fnp（物理=曝光窗内相机/场景微振动，
            # 参考仿真"机械振动"误差源）。resp/cap 仍读出供探针/测试
            # 打印。real_noise_scale 只倍乘灰度 σ（迭代 27 原语义）
            # 雪花视觉档：σ=min(6DN, 0.2×tpl_std)（cap=min(12,0.25×std)
            # → std=cap/0.25；无 cap 时 30DN 兜底）
            tpl_std_dn = (cap / NOISE_SNR_CAP) if cap else 30.0
            sig = min(6.0, 0.2 * tpl_std_dn)
            sig_shift_col = sig_shift_row = fnp
        else:
            light_level = 1.0 if light_on else LIGHT_LEVEL_OFF
            sig = K_NOISE * math.sqrt(gain) / math.sqrt(
                max(1e-6, exposure * light_level))
            # 无响应度回退路径：白噪声量不可知 → 振动分量承担全部目标
            sig_shift_col = sig_shift_row = fnp
            if cam.id not in _RESP_FALLBACK_LOGGED:
                _RESP_FALLBACK_LOGGED.add(cam.id)
                print("[vision_model] 相机 %s 无模板响应度，REAL 噪声回退 "
                      "K_NOISE 旧公式（建模板/启动预热后自动切换）" % cam.id,
                      flush=True)
    # 迭代 27：× recipe real_noise_scale[相机] 热读额外倍率（默认 1.0；
    # 倍率在封顶之后乘——用户显式超调不受物理封顶限制；noise_sigma_dn
    # 标定覆盖路径不乘）
    if noise_sigma_dn is None:
        sig *= real_noise_scale(engine, cam.id)
    if sig > 1e-6:
        img += _noise_unit(n, W, H) * sig
    # 全局亚像素平移（曝光窗振动）：cv2 warpAffine 释放 GIL，BORDER_REPLICATE
    # 避免卷边（平移 σ 通常 ≪1px，边缘拉伸不可见）；模板示教/探针
    # （noise_sigma_dn 显式）与 fnp=0 不注入——确定性回退保持逐字节全等；
    # scene_shift_px 显式给定（平移增益标定试验）时无条件注入。
    # col/row 各向异性注入（sig_shift_col/row 由探针按轴标定的增益模型
    # 分别解出）；回退路径（无 resp）两轴同 σ
    if scene_shift_px is not None:
        dx, dy = float(scene_shift_px[0]), float(scene_shift_px[1])
    elif sig_shift_col > 1e-6 or sig_shift_row > 1e-6:
        rng = _rng()
        dx = float(rng.normal(0.0, sig_shift_col))
        dy = float(rng.normal(0.0, sig_shift_row))
    else:
        dx = dy = 0.0
    if abs(dx) > 1e-6 or abs(dy) > 1e-6:
        mtx = n.array([[1.0, 0.0, dx], [0.0, 1.0, dy]], dtype=n.float32)
        img = cv2.warpAffine(img, mtx, (W, H),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                             borderMode=cv2.BORDER_REPLICATE)
        time.sleep(0)         # cv2 大块调用释放 GIL，仍让出物理线程
    # 视场旋转（迭代 39 重构）：相机物理安装旋转 view_rot 已在投影链
    # （project_rel R(−θ)，真值）施加——渲染内容随标记投影自然反向旋转，
    # 边缘内容真实（不再是旧出图级 warpAffine 的 BORDER_REPLICATE 假
    # 边缘），此处不再做任何整图旋转；结果侧亦不剥旋（原始传感器系）。
    n.clip(img, 0, 255, out=img)          # 原位裁剪省一次 36MB 分配
    return img.astype(n.uint8)


def _blur_layer_region(n, cv2, img, tgt, cam, items, cam_pos, W, H, k, sigma,
                       marks_px=None, att=None, base_g=None):
    """离焦层区域模糊：取层内容像素 bbox（外扩 ceil(3σ)+2 容纳模糊扩散），
    在 tgt 的该裁剪区高斯后整体贴回 img。σ>2 时降采样模糊再放大
    （σ≤4→1/4、σ>4→1/8，σ'=σ/ds≤1 视觉无差，成本 ≤1/16——对焦工况下
    chuck 恒离焦 1mm，全幅 σ=8 直接模糊太贵，perf 红线所迫）。bbox 内
    已合成的后层内容一并承受本层 σ（本机相邻面 σ 接近，无可见伪影）。
    marks_px：该层宿主标记的投影像素点（IR 层 items 为空时 bbox 仍有内容，
    迭代 17）。
    att/base_g（2026-09-08 离焦对比度衰减）：景深外层内容**细节对比度**
    按 att=DOF/|dz| 滚降（1mm 离焦标记应不可见——只糊不衰减时 NCC 仍
    高分识别，不真实）。衰减方向=向本层亮基底灰度（fill 项灰度，无 fill
    时 SCENE_CARRIER×base_g）收缩——细节/标记消失、均匀亮面保持（亮度
    无对比度可言，不随离焦变暗——chuck 恒离焦 1mm 的常规对焦工况下裸
    吸盘面仍是亮面）。"""
    bbox = _layer_px_bbox(cam, items, cam_pos, W, H, k, marks_px=marks_px)
    if bbox is None:
        return
    pad = int(math.ceil(3 * sigma)) + 2
    x0 = max(0, bbox[0] - pad)
    y0 = max(0, bbox[1] - pad)
    x1 = min(W, bbox[2] + pad)
    y1 = min(H, bbox[3] + pad)
    if x1 <= x0 or y1 <= y0:
        return
    crop = tgt[y0:y1, x0:x1]
    if sigma > 2.0:
        # σ 越大降采样越狠：σ>4 走 1/8（σ'=σ/8≤1，成本 ~1/64），否则 1/4。
        # 对焦工况 chuck 恒离焦 1mm(σ=8)，全幅 σ=8 直接模糊太贵（perf 红线）
        ds = 8 if sigma > 4.0 else 4
        sw = max(1, (x1 - x0) // ds)
        sh = max(1, (y1 - y0) // ds)
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA)
        ksz = int(math.ceil(sigma / float(ds) * 6)) | 1
        small = cv2.GaussianBlur(small, (max(ksz, 3), max(ksz, 3)),
                                 sigma / float(ds))
        crop = cv2.resize(small, (x1 - x0, y1 - y0),
                          interpolation=cv2.INTER_LINEAR)
    else:
        ksz = int(math.ceil(sigma * 6)) | 1
        crop = cv2.GaussianBlur(crop, (max(ksz, 3), max(ksz, 3)), sigma)
    if att is not None and att < 0.999:
        # 向本层亮基底收缩：att·模糊层 + (1-att)·基底灰度
        fill_g = None
        for item in items:
            if item[0] == "fill":
                fill_g = float(item[1]) * (base_g if base_g else 1.0)
                break
        if fill_g is None:
            fill_g = SCENE_CARRIER * (base_g if base_g else 1.0)
        crop = cv2.addWeighted(crop, float(att), fill_g, 1.0 - float(att), 0.0)
    img[y0:y1, x0:x1] = crop




def _draw_scene_item(n, cv2, img, cam, item, cam_pos, W, H, k, base_g):
    """画一个场景层（世界系几何 → 投影裁剪 → 灰度填充）。
    cv2 在场走精确圆盘/多边形；缺失时 numpy 简画（圆盘=mgrid 掩膜、
    多边形=轴对齐 bbox 近似——仅预览降级路径，REAL 不允许无 cv2）。"""
    kind = item[0]
    gray = float(item[1] * base_g)
    if kind == "fill":
        img[:, :] = gray
        return
    if kind == "disc":
        _, _, cx, cy, r = item
        col, row = cam.project(cx - cam_pos[0], cy - cam_pos[1], W, H)
        rp = r / k
        if col + rp < 0 or col - rp > W or row + rp < 0 or row - rp > H:
            return                              # bbox 快速剔除
        if cv2 is not None:
            # 定点子像素 + AA：消除整像素截断量化（远景几何自动回退）
            _circle_subpix(cv2, img, col, row, rp, gray)
        else:
            x0, x1 = max(0, int(col - rp)), min(W, int(col + rp) + 1)
            y0, y1 = max(0, int(row - rp)), min(H, int(row + rp) + 1)
            if x1 > x0 and y1 > y0:
                yy, xx = n.mgrid[y0:y1, x0:x1]
                mask = (xx - col) ** 2 + (yy - row) ** 2 <= rp * rp
                img[y0:y1, x0:x1][mask] = gray
        return
    if kind == "ring":
        # 环槽描边（陶瓷面气路纹路）：("ring", g, cx, cy, r, 槽宽mm)
        _, _, cx, cy, r, wmm = item
        col, row = cam.project(cx - cam_pos[0], cy - cam_pos[1], W, H)
        ro = (r + 0.5 * wmm) / k
        ri = max(0.0, (r - 0.5 * wmm) / k)
        if col + ro < 0 or col - ro > W or row + ro < 0 or row - ro > H:
            return                              # bbox 快速剔除
        if cv2 is not None:
            _circle_subpix(cv2, img, col, row, 0.5 * (ro + ri), gray,
                           thickness=max(1, int(round(ro - ri))))
        else:
            x0, x1 = max(0, int(col - ro)), min(W, int(col + ro) + 1)
            y0, y1 = max(0, int(row - ro)), min(H, int(row + ro) + 1)
            if x1 > x0 and y1 > y0:
                yy, xx = n.mgrid[y0:y1, x0:x1]
                d2 = (xx - col) ** 2 + (yy - row) ** 2
                mask = (d2 <= ro * ro) & (d2 >= ri * ri)
                img[y0:y1, x0:x1][mask] = gray
        return
    if kind == "poly":
        pts = [cam.project(px - cam_pos[0], py - cam_pos[1], W, H)
               for px, py in item[2]]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if max(xs) < 0 or min(xs) > W or max(ys) < 0 or min(ys) > H:
            return                              # bbox 快速剔除
        if cv2 is not None:
            # 定点子像素 + AA：顶点 ×2^16（fillPoly 的 int32 顶点按
            # shift 解释为定点数，亚像素位置得以连续表达）
            _fillpoly_subpix(cv2, n, img,
                             [[p[0], p[1]] for p in pts], gray)
        else:
            x0, x1 = int(max(0, min(xs))), int(min(W - 1, max(xs)))
            y0, y1 = int(max(0, min(ys))), int(min(H - 1, max(ys)))
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = gray


def _draw_prim(n, cv2, img, cam, mark, prim, cam_pos, W, H, k, gray):
    """画一个图元。cv2 在场走旋转多边形精确画；缺失时 numpy 轴对齐简画
    （仅预览降级路径，REAL 不允许无 cv2）。"""
    kind = prim[0]
    cyaw = math.cos(mark.yaw), math.sin(mark.yaw)

    def to_px(lx, ly):
        # mark 局部 →（yaw 旋转）→ 世界偏移 → 投影像素
        wx = cyaw[0] * lx - cyaw[1] * ly
        wy = cyaw[1] * lx + cyaw[0] * ly
        return cam.project(mark.x - cam_pos[0] + wx,
                           mark.y - cam_pos[1] + wy, W, H)

    if kind == "rect":
        _, rcx, rcy, rw, rh = prim
        pts = [to_px(rcx + sx * rw / 2, rcy + sy * rh / 2)
               for sx in (-1, 1) for sy in ((-1, 1) if sx < 0 else (1, -1))]
        if cv2 is not None:
            # 定点子像素 + AA（quad 标记=4 个角 rect，整像素截断曾把
            # 质心量化成 ±0.5px 台阶——误差链修复的关键一处）
            _fillpoly_subpix(cv2, n, img,
                             [[p[0], p[1]] for p in pts], float(gray))
        else:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x0, x1 = int(max(0, min(xs))), int(min(W - 1, max(xs)))
            y0, y1 = int(max(0, min(ys))), int(min(H - 1, max(ys)))
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = gray
    elif kind == "ring":
        _, rcx, rcy, r_out, r_in = prim
        c = to_px(rcx, rcy)
        ro = r_out / k
        ri = r_in / k
        if cv2 is not None:
            _circle_subpix(cv2, img, c[0], c[1], ro, float(gray))
            _circle_subpix(cv2, img, c[0], c[1], ri, float(img[0, 0]))
        else:
            H, W_ = img.shape
            x0, x1 = max(0, int(c[0] - ro)), min(W_, int(c[0] + ro) + 1)
            y0, y1 = max(0, int(c[1] - ro)), min(H, int(c[1] + ro) + 1)
            if x1 > x0 and y1 > y0:
                yy, xx = n.mgrid[y0:y1, x0:x1]
                d2 = (xx - c[0]) ** 2 + (yy - c[1]) ** 2
                mask = (d2 <= ro * ro) & (d2 >= ri * ri)
                img[y0:y1, x0:x1][mask] = gray


# ---------------- FAST 解析投影器 ----------------
def analytic_result(cam, marks, cam_pos, light_on, noise_px=None):
    """FAST 模式：不渲染、不走 TCP——由几何直接合成结果点（含噪声/景深/
    光源判定），输出与 REAL 同 schema 的结果 dict。
    noise_px=None 回退模块常量 FAST_NOISE_PX（2026-09-06 迭代 21：处方
    sim_sensors.fast_noise_px 热读传入）。"""
    if noise_px is None:
        noise_px = FAST_NOISE_PX
    entries = []
    for m in marks:
        for (lx, ly) in entry_points(m):
            cyaw = (math.cos(m.yaw), math.sin(m.yaw))
            wx = cyaw[0] * lx - cyaw[1] * ly
            wy = cyaw[1] * lx + cyaw[0] * ly
            dx = m.x + wx - cam_pos[0]
            dy = m.y + wy - cam_pos[1]
            col, row = cam.project(dx, dy)
            in_fov = cam.in_fov(col, row)
            blur = m.blur_ratio
            # 噪声：离焦超 1×DOF 后平方放大、封顶 25×（参考 flows.py）
            npx = noise_px
            if blur > 1.0:
                npx += min(25.0, (blur - 1.0) ** 2) * max(0.02, noise_px)
            col += float(_rng().normal(0, npx))
            row += float(_rng().normal(0, npx))
            if light_on:
                score = FAST_BASE_SCORE - 0.15 * min(blur, BLUR_REJECT) \
                    + float(_rng().uniform(-0.01, 0.01))
                score = max(0.0, min(1.0, score))
            else:
                score = 0.05 + abs(float(_rng().normal(0, 0.02)))
            # 相对建模板角度 = 特征投影后 +x 轴的图像角（deg）
            ax = cam.project_rel(math.cos(m.yaw), math.sin(m.yaw))
            ang = math.degrees(math.atan2(ax[1], ax[0]))
            ok = bool(in_fov and blur <= BLUR_REJECT and light_on)
            entries.append({"col": col, "row": row, "angle": ang,
                            "score": score, "scale": 1.0, "judge": ok,
                            "in_fov": in_fov, "blur": blur, "mark": m.name})
    # 形状标签取第一个在 FOV 内的标记（下相机 recipe 晶圆 marks 与已键合
    # die 标记可混在列表，FOV 外的种属不该冒名）；都不在 FOV 回退 marks[0]
    shape = ""
    for m, e in zip(marks, entries):
        if e["in_fov"]:
            shape = m.shape
            break
    if not shape and marks:
        shape = marks[0].shape
    return build_result(shape, entries), entries


def build_result(shape_name, entries):
    """组装结果 JSON（七字段契约 + 2026-09-02 迭代 11 起追加第 8 字段
    lstName，lst* 等长数组；judge=任一有效）。lstName=各条目来源标记名
    （FAST=scene 标记名如 site_R01C02#0 / R03C04#2；REAL=模板名）——
    键合后检测按名筛 die 标记（die_id#k）消费，旧消费者忽略即可。"""
    seen = [e for e in entries if e.get("in_fov", True)]
    return {
        "strShapeName": shape_name,
        "judge": bool(any(e["judge"] for e in seen)) if seen else False,
        "lstCol": [round(e["col"], 3) for e in seen],
        "lstRow": [round(e["row"], 3) for e in seen],
        "lstAngle": [round(e["angle"], 4) for e in seen],
        "lstScore": [round(e["score"], 4) for e in seen],
        "lstScale": [round(e["scale"], 4) for e in seen],
        "lstName": [str(e.get("mark") or "") for e in seen],
    }


# ---------------- 标准库 PNG 编码（灰度 8bit，zlib level 1 求快） ----------------
def png_encode_gray(img):
    """numpy uint8 (H,W) → PNG bytes。纯标准库（zlib），压缩等级 1。"""
    n = np()
    if img.dtype != n.uint8 or img.ndim != 2:
        raise ValueError("png_encode_gray 需要 uint8 (H,W)")
    H, W = img.shape
    raw = b"".join(b"\x00" + img[i].tobytes() for i in range(H))
    comp = zlib.compress(raw, 1)

    def chunk(tag, data):
        return struct.pack("!I", len(data)) + tag + data \
            + struct.pack("!I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    ihdr = struct.pack("!IIBBBBB", W, H, 8, 0, 0, 0, 0)   # 8bit 灰度
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", comp) + chunk(b"IEND", b""))


def png_size(png_bytes):
    """从 PNG IHDR 读 (W, H)（不解码，TCP 接收侧登记用）。"""
    if png_bytes[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    return struct.unpack("!II", png_bytes[16:24])


def png_decode_gray(png_bytes):
    """png_encode_gray 的逆：PNG bytes → numpy uint8 (H,W)。
    只认本模块编码约定（8bit 灰度、非隔行、逐行 filter-0）——采图/模板
    落盘产物恒定满足；不符合约定抛 ValueError（消息指路）。"""
    n = np()
    if png_bytes[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, w, h, idat = 8, 0, 0, bytearray()
    while pos + 8 <= len(png_bytes):
        ln = struct.unpack("!I", png_bytes[pos:pos + 4])[0]
        tag = png_bytes[pos + 4:pos + 8]
        data = png_bytes[pos + 8:pos + 8 + ln]
        if tag == b"IHDR":
            w, h = struct.unpack("!II", data[:8])
            if data[8] != 8 or data[9] != 0 or data[12] != 0:
                raise ValueError("只支持 8bit 灰度非隔行 PNG"
                                 "（png_encode_gray 约定）")
        elif tag == b"IDAT":
            idat += data
        pos += 12 + ln
    if not w or not h:
        raise ValueError("PNG 缺 IHDR")
    raw = zlib.decompress(bytes(idat))
    if len(raw) != h * (w + 1):
        raise ValueError("PNG 数据长度不符（%d != %d）"
                         % (len(raw), h * (w + 1)))
    img = n.empty((h, w), dtype=n.uint8)
    for r in range(h):
        if raw[r * (w + 1)] != 0:
            raise ValueError("非 filter-0 行（不符合 png_encode_gray 约定）")
        img[r] = n.frombuffer(raw[r * (w + 1) + 1:(r + 1) * (w + 1)],
                              dtype=n.uint8)
    return img
