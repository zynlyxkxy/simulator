# -*- coding: utf-8 -*-
"""fine_align.py — 逐键合位精对准（fine align）双侧图案拟合算法。

纯标准库实现（复用 ega._solve3/_solve4 高斯消元），与参考项目
d2w_sim_semiconductor fine_alignment.py 的算法注册表语义对齐：

  设计点集 P（相对 site/die 中心的偏移）→ 实测点集 Q（公共叠合帧）
  rigid      Kabsch 3 参（tx, ty, θ）——默认，无缩放自由度，最稳健
  similarity Umeyama 4 参（+scale）
  affine     6 参（≥3 点；n=2 欠定，强制拒绝——参考项目教训：六参数
             在 n=2 时把旋转劈给正交项，θ 不可信）
  ransac     抗污染刚体：两点最小集全枚举（n≤8 → C(n,2)≤28 组），
             内点阈值 2µm，取最大内点集后对内点重拟合 rigid
  midpoint_2mark 双标记中点角度法（TORAY 式）：取两侧各自前 2 个标记，
             中点 Cm/Pm 定平移、连线角 CA/PA 定旋转：
             θ = (PA_m−CA_m) − (PA_d−CA_d)、t = 中点差经 R(θ) 折算；
             n>2 只用前 2 点（结果 note 注明），n=2 与 rigid 精确解同值

统一返回 dict：
  tx, ty      平移（mm）
  theta       旋转（rad；统一用 θ=atan2(M10−M01, M00+M11) 稳健提取，
              M 为变换的 2×2 线性部分）
  scale       等比缩放（rigid 恒 1.0）
  rms         残差均方根（mm）
  max_resid   最大点残差（mm）
  residuals   逐点残差列表（mm）
  n           参与拟合点数（ransac=内点数）

单位与全线一致：距离 mm，角度 rad。
"""
import math

import ega

ALGORITHMS = ("rigid", "similarity", "affine", "ransac", "midpoint_2mark")
RANSAC_INLIER_TOL_MM = 0.002     # ransac 内点阈值（2µm，精对准测量噪声量级之上）


def _centroid(pts):
    n = float(len(pts))
    return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n)


def _check_pair(design, measured):
    if len(design) != len(measured):
        raise ValueError("设计/实测点数不一致：%d vs %d"
                         % (len(design), len(measured)))
    if len(design) < 2:
        raise ValueError("精对准拟合需要至少 2 个点，当前 %d 个" % len(design))


def _rigid_params(design, measured):
    """Kabsch 2D 刚体：返回 (tx, ty, theta)。n≥2（n=2 精确解）。"""
    dx, dy = _centroid(design)
    mx, my = _centroid(measured)
    # M = Σ (q−q̄)(p−p̄)^T（2×2 互协方差，未归一化）
    m00 = m01 = m10 = m11 = 0.0
    for (px, py), (qx, qy) in zip(design, measured):
        ax, ay = px - dx, py - dy
        bx, by = qx - mx, qy - my
        m00 += bx * ax
        m01 += bx * ay
        m10 += by * ax
        m11 += by * ay
    theta = math.atan2(m10 - m01, m00 + m11)
    c, s = math.cos(theta), math.sin(theta)
    tx = mx - (c * dx - s * dy)
    ty = my - (s * dx + c * dy)
    return tx, ty, theta


def _similarity_params(design, measured):
    """Umeyama 2D 相似变换：返回 (tx, ty, theta, scale)。"""
    tx, ty, theta = _rigid_params(design, measured)
    dx, dy = _centroid(design)
    mx, my = _centroid(measured)
    c, s = math.cos(theta), math.sin(theta)
    num = den = 0.0
    for (px, py), (qx, qy) in zip(design, measured):
        ax, ay = px - dx, py - dy
        bx, by = qx - mx, qy - my
        rx, ry = c * ax - s * ay, s * ax + c * ay
        num += bx * rx + by * ry
        den += rx * rx + ry * ry
    scale = num / den if den > 1e-18 else 1.0
    # 缩放吸收进平移：q = s·R·p + t
    tx = mx - scale * (c * dx - s * dy)
    ty = my - scale * (s * dx + c * dy)
    return tx, ty, theta, scale


def _affine_params(design, measured):
    """六参数仿射（ega 同式）：返回 (a11, a12, a21, a22, tx, ty)。"""
    n = len(design)
    sx_xx = sx_yx = sX = 0.0
    sy_xy = sy_yy = sY = 0.0
    cxx = cxy = cx = cyy = cy = 0.0
    for (x, y), (X, Y) in zip(design, measured):
        cxx += x * x
        cxy += x * y
        cx += x
        cyy += y * y
        cy += y
        sx_xx += x * X
        sx_yx += y * X
        sX += X
        sy_xy += x * Y
        sy_yy += y * Y
        sY += Y
    C = [[cxx, cxy, cx], [cxy, cyy, cy], [cx, cy, float(n)]]
    px = ega._solve3([row[:] for row in C], [sx_xx, sx_yx, sX])
    py = ega._solve3([row[:] for row in C], [sy_xy, sy_yy, sY])
    a11, a12, tx = px
    a21, a22, ty = py
    return a11, a12, a21, a22, tx, ty


def _build_result(tx, ty, theta, scale, design, measured, apply_fn):
    """统一结果组装 + 残差统计。apply_fn(p) -> 拟合后预测点。"""
    residuals = []
    for p, q in zip(design, measured):
        Xp, Yp = apply_fn(p)
        residuals.append(math.hypot(Xp - q[0], Yp - q[1]))
    n = len(residuals)
    rms = math.sqrt(sum(r * r for r in residuals) / n) if n else 0.0
    return {"tx": float(tx), "ty": float(ty), "theta": float(theta),
            "scale": float(scale), "rms": float(rms),
            "max_resid": max(residuals) if residuals else 0.0,
            "residuals": residuals, "n": n}


def fit_rigid(design, measured):
    """Kabsch 3 参刚体拟合（默认算法）。n≥2。"""
    _check_pair(design, measured)
    tx, ty, theta = _rigid_params(design, measured)
    c, s = math.cos(theta), math.sin(theta)
    return _build_result(tx, ty, theta, 1.0, design, measured,
                         lambda p: (c * p[0] - s * p[1] + tx,
                                    s * p[0] + c * p[1] + ty))


def fit_similarity(design, measured):
    """Umeyama 4 参相似变换拟合。n≥2。"""
    _check_pair(design, measured)
    tx, ty, theta, scale = _similarity_params(design, measured)
    c, s = math.cos(theta), math.sin(theta)
    return _build_result(tx, ty, theta, scale, design, measured,
                         lambda p: (scale * (c * p[0] - s * p[1]) + tx,
                                    scale * (s * p[0] + c * p[1]) + ty))


def fit_affine(design, measured):
    """六参数仿射拟合。**n≥3**——n=2 欠定（旋转劈给正交项），强制拒绝。"""
    _check_pair(design, measured)
    if len(design) < 3:
        raise ValueError("affine 拟合需要至少 3 个点（n=%d 欠定）" % len(design))
    a11, a12, a21, a22, tx, ty = _affine_params(design, measured)
    theta = math.atan2(a21 - a12, a11 + a22)
    scale = 0.5 * (a11 + a22)
    return _build_result(tx, ty, theta, scale, design, measured,
                         lambda p: (a11 * p[0] + a12 * p[1] + tx,
                                    a21 * p[0] + a22 * p[1] + ty))


def fit_ransac(design, measured, tol=RANSAC_INLIER_TOL_MM):
    """抗污染刚体拟合：两点最小集全枚举 + 内点阈值 tol（默认 2µm），
    最大内点集重拟合 rigid。n≥2；全为离群点时退化为最大内点子集
    （至少 2 点，总能拟合）。"""
    _check_pair(design, measured)
    n = len(design)
    if n < 3:
        return fit_rigid(design, measured)
    best_inliers = None
    for i in range(n):
        for j in range(i + 1, n):
            try:
                tx, ty, theta = _rigid_params([design[i], design[j]],
                                              [measured[i], measured[j]])
            except Exception:
                continue
            c, s = math.cos(theta), math.sin(theta)
            inliers = []
            for k in range(n):
                Xp = c * design[k][0] - s * design[k][1] + tx
                Yp = s * design[k][0] + c * design[k][1] + ty
                if math.hypot(Xp - measured[k][0],
                              Yp - measured[k][1]) <= tol:
                    inliers.append(k)
            if best_inliers is None or len(inliers) > len(best_inliers):
                best_inliers = inliers
    if not best_inliers:
        best_inliers = list(range(n))
    d_in = [design[k] for k in best_inliers]
    m_in = [measured[k] for k in best_inliers]
    res = fit_rigid(d_in, m_in)
    res["inliers"] = list(best_inliers)
    res["n_total"] = n
    return res


def fit_midpoint_2mark(design, measured):
    """双标记中点角度法（TORAY 式，按图公式）：取两侧各自前 2 个标记，
    中点 Cm=(C1+C2)/2、连线角 CA=atan2(C2y−C1y, C2x−C1x)（Pm/PA 同理）；
    θ = A_meas − A_design（diA/dmA 差值形式），平移 t = 中点差经 R(θ)
    折算（dX=dmx−dix、dY 同式）。n≥2；n>2 只用前 2 点并在结果 note
    注明；n=2 时与 rigid 精确解同值（中点/连线角即 2 点 Kabsch）。
    两设计点重合（基线退化）→ 拒绝。"""
    _check_pair(design, measured)
    d = design[:2]
    m = measured[:2]
    if math.hypot(d[1][0] - d[0][0], d[1][1] - d[0][1]) < 1e-12:
        raise ValueError("midpoint_2mark 两设计标记重合，基线退化无法定角")
    cm_d = ((d[0][0] + d[1][0]) / 2.0, (d[0][1] + d[1][1]) / 2.0)
    cm_m = ((m[0][0] + m[1][0]) / 2.0, (m[0][1] + m[1][1]) / 2.0)
    ca_d = math.atan2(d[1][1] - d[0][1], d[1][0] - d[0][0])
    ca_m = math.atan2(m[1][1] - m[0][1], m[1][0] - m[0][0])
    theta = ca_m - ca_d
    c, s = math.cos(theta), math.sin(theta)
    tx = cm_m[0] - (c * cm_d[0] - s * cm_d[1])
    ty = cm_m[1] - (s * cm_d[0] + c * cm_d[1])
    res = _build_result(tx, ty, theta, 1.0, design, measured,
                        lambda p: (c * p[0] - s * p[1] + tx,
                                   s * p[0] + c * p[1] + ty))
    res["n"] = 2                    # 参与拟合的点数（中点/角度只用前 2 点）
    if len(design) > 2:
        res["note"] = ("midpoint_2mark 只用前 2 个标记（共 %d 个，余者仅"
                       "计入残差统计）" % len(design))
        res["n_total"] = len(design)
    return res


def fit(algorithm, design, measured):
    """算法注册表分发：algorithm ∈ ALGORITHMS。"""
    if algorithm == "rigid":
        return fit_rigid(design, measured)
    if algorithm == "similarity":
        return fit_similarity(design, measured)
    if algorithm == "affine":
        return fit_affine(design, measured)
    if algorithm == "ransac":
        return fit_ransac(design, measured)
    if algorithm == "midpoint_2mark":
        return fit_midpoint_2mark(design, measured)
    raise ValueError("未知精对准算法 %r（合法：%s）"
                     % (algorithm, "/".join(ALGORITHMS)))


def compose_rigid(fit_d, fit_w):
    """Δ = T_d ∘ T_w⁻¹ 的刚体参数（两侧拟合皆为小角刚体/相似时成立）：
    晶圆侧当前姿态 T_w、die 侧姿态 T_d（同一公共帧），返回晶圆达到
    die 叠合所需的运动 (tx, ty, theta)。"""
    th = fit_d["theta"] - fit_w["theta"]
    c, s = math.cos(th), math.sin(th)
    # t_Δ = t_d − R_Δ·t_w
    tx = fit_d["tx"] - (c * fit_w["tx"] - s * fit_w["ty"])
    ty = fit_d["ty"] - (s * fit_w["tx"] + c * fit_w["ty"])
    return {"tx": float(tx), "ty": float(ty), "theta": float(th)}


def relative_delta(fit_d, fit_w):
    """die↔site 相对残差的**参考点无关**形式（2026-09-07 迭代 20）：
    tx,ty = t_d − t_w（两侧设计原点经各自拟合的像之差——fit 平移
    t = T(设计原点)，Kabsch 刚体下同 = 两侧点云质心像之差）；theta =
    θ_d − θ_w（同 compose_rigid）。

    与 compose_rigid 的区别：compose 的 t_Δ = t_d − R_Δ·t_w 是复合变换
    在**帧原点**处的平移——θ_Δ≠0 且点云质心距帧原点 |F| 很远时含
    (I−R_Δ)·F ≈ θ_Δ×J(F) 的参考原点耦合项（本机真值点云在世界系
    ~43mm 处，θ 残余 10µrad × 43mm ≈ 0.4µm 假平移散布——迭代 20 前
    phys_resid/truth 的"框偏"主因；测量侧点云被重建到头心相对帧
    （近原点），不受影响）。真值通道（_fa_truth_resid / bond_commit）
    与键合后检测多点对组间偏差（世界系点云）必须用本函数。"""
    return {"tx": float(fit_d["tx"] - fit_w["tx"]),
            "ty": float(fit_d["ty"] - fit_w["ty"]),
            "theta": float(fit_d["theta"] - fit_w["theta"])}
