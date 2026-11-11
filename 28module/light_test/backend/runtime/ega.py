# -*- coding: utf-8 -*-
"""ega.py — 晶圆全局对准（Enhanced Global Alignment，EGA）六参数仿射拟合。

纯标准库实现，与参考项目 d2w_sim_semiconductor_refactor/sim/vision.py
`fit_similarity` 的语义对齐：

  名义设计坐标 (x, y) → 实测坐标 (X, Y)
  X = a11*x + a12*y + tx
  Y = a21*x + a22*y + ty

小角度分解：
  θ    = (a21 - a12) / 2          # 平均旋转角（rad）
  sx   = a11 - 1.0                # X 向比例误差（ppm 量级）
  sy   = a22 - 1.0                # Y 向比例误差
  ortho = (a12 + a21) / 2         # 非正交项（rad）

n ≥ 3：六参数仿射（过约束最小二乘）
n = 2：四参数相似变换（小角度线性化）
n < 2：无法拟合，返回 None

用法：
  fit = fit_similarity(nominal=[(x1,y1), ...], measured=[(X1,Y1), ...])
  X, Y = apply_map(fit, x, y)

单位与全线一致：距离 mm，角度 rad。
"""
import math


def _solve3(A, b):
    """高斯消元解 3×3 线性方程组 A·x = b。A 被原地破坏；返回 x 列表。"""
    # 增广
    M = [A[i][:] + [b[i]] for i in range(3)]
    # 列主元消去
    for col in range(3):
        piv, maxv = col, abs(M[col][col])
        for row in range(col + 1, 3):
            if abs(M[row][col]) > maxv:
                piv, maxv = row, abs(M[row][col])
        if maxv < 1e-15:
            raise ValueError("奇异矩阵")
        if piv != col:
            M[col], M[piv] = M[piv], M[col]
        # 归一化主元行
        d = M[col][col]
        for j in range(col, 4):
            M[col][j] /= d
        # 消去下方和上方
        for row in range(3):
            if row == col:
                continue
            factor = M[row][col]
            if abs(factor) < 1e-18:
                continue
            for j in range(col, 4):
                M[row][j] -= factor * M[col][j]
    return [M[i][3] for i in range(3)]


def fit_similarity(nominal, measured):
    """名义/实测点组 → 六参数仿射拟合结果 dict。

    Args:
        nominal:  [(x,y), ...]  设计名义坐标
        measured: [(X,Y), ...]  实测坐标（含上片偏差、随机抖动、视觉噪声）

    Returns:
        dict with keys:
            tx, ty, theta, sx, sy, ortho, resid_3s, n,
            a: [a11, a12, a21, a22]
        或 n < 2 时返回 None。
    """
    n = len(nominal)
    if n < 2:
        return None
    if n == 2:
        return _fit_similarity_4param(nominal, measured)
    return _fit_affine_6param(nominal, measured)


def _fit_affine_6param(nominal, measured):
    """n ≥ 3：分别对 X、Y 建立法方程 C·p = rhs，其中 p = [a1,a2,tx] 或 [a3,a4,ty]。"""
    n = len(nominal)
    sx_xx = sx_yx = sX = 0.0
    sy_xy = sy_yy = sY = 0.0
    cxx = cxy = cx = cyy = cy = 0.0
    for (x, y), (X, Y) in zip(nominal, measured):
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
    px = _solve3(C, [sx_xx, sx_yx, sX])
    py = _solve3(C, [sy_xy, sy_yy, sY])
    a11, a12, tx = px
    a21, a22, ty = py
    return _build_result(a11, a12, a21, a22, tx, ty, nominal, measured)


def _fit_similarity_4param(nominal, measured):
    """n = 2：四参数相似变换（小角度线性化）。

    模型：X ≈ (1+a)*x - b*y + tx
         Y ≈ b*x + (1+a)*y + ty
    其中 b = θ（rad），a = 等比比例误差。
    """
    # 联立 4 未知 [a, b, tx, ty]：X 方程行 [x,-y,1,0]；Y 方程行 [y,x,0,1]
    ATA = [[0.0] * 4 for _ in range(4)]
    ATX = [0.0] * 4
    for (x, y), (X, Y) in zip(nominal, measured):
        rows = [[x, -y, 1.0, 0.0], [y, x, 0.0, 1.0]]
        vals = [X, Y]
        for row, v in zip(rows, vals):
            for k in range(4):
                ATX[k] += row[k] * v
                for l in range(4):
                    ATA[k][l] += row[k] * row[l]
    p = _solve4(ATA, ATX)
    a, bb, tx, ty = p
    a11, a12, a21, a22 = 1.0 + a, -bb, bb, 1.0 + a
    return _build_result(a11, a12, a21, a22, tx, ty, nominal, measured)


def _solve4(A, b):
    """高斯消元解 4×4。"""
    M = [A[i][:] + [b[i]] for i in range(4)]
    for col in range(4):
        piv, maxv = col, abs(M[col][col])
        for row in range(col + 1, 4):
            if abs(M[row][col]) > maxv:
                piv, maxv = row, abs(M[row][col])
        if maxv < 1e-15:
            raise ValueError("奇异矩阵")
        if piv != col:
            M[col], M[piv] = M[piv], M[col]
        d = M[col][col]
        for j in range(col, 5):
            M[col][j] /= d
        for row in range(4):
            if row == col:
                continue
            factor = M[row][col]
            if abs(factor) < 1e-18:
                continue
            for j in range(col, 5):
                M[row][j] -= factor * M[col][j]
    return [M[i][4] for i in range(4)]


def _build_result(a11, a12, a21, a22, tx, ty, nominal, measured):
    """由 a11..a22, tx, ty 构造标准结果 dict（含残差统计）。"""
    theta = 0.5 * (a21 - a12)
    sx = a11 - 1.0
    sy = a22 - 1.0
    ortho = 0.5 * (a12 + a21)
    # 残差
    sq = []
    for (x, y), (X, Y) in zip(nominal, measured):
        Xp = a11 * x + a12 * y + tx
        Yp = a21 * x + a22 * y + ty
        sq.append((Xp - X) ** 2 + (Yp - Y) ** 2)
    n = len(sq)
    rms = math.sqrt(sum(sq) / n) if n else 0.0
    resid_3s = 3.0 * rms
    return {
        "tx": float(tx),
        "ty": float(ty),
        "theta": float(theta),
        "sx": float(sx),
        "sy": float(sy),
        "ortho": float(ortho),
        "resid_3s": float(resid_3s),
        "rms": float(rms),
        "n": n,
        "a": [float(a11), float(a12), float(a21), float(a22)],
    }


def apply_map(ega, x, y):
    """用 EGA 拟合结果把名义 (x, y) 映射到实测/整机坐标系。"""
    a11, a12, a21, a22 = ega["a"]
    return (a11 * x + a12 * y + ega["tx"],
            a21 * x + a22 * y + ega["ty"])


def select_measure_points(cells, n_pts=9, radius_ratio=0.88):
    """从 site 格点中挑选适合 EGA 测量的点位。

    策略：在最大可用半径的 radius_ratio 处沿圆周均匀取点，每个角度扇区
    选最近的实际 site；这样保证覆盖晶圆边缘、分布均匀、降低拟合病态。

    Args:
        cells: [(r, c, x, y), ...]（如 wafer_sites_cells 输出）
        n_pts: 期望测量点数（默认 9，最少 3）
        radius_ratio: 选点目标圆半径 = max_r * radius_ratio

    Returns:
        [(x, y), ...] 选中点的名义坐标；若 cells 不足 3 个则返回全部。
    """
    pts = [(x, y) for _, _, x, y in cells]
    if len(pts) < 3:
        return pts
    max_r = max(math.hypot(x, y) for x, y in pts)
    target_r = max_r * radius_ratio
    used = set()
    out = []
    for i in range(n_pts):
        ang = i * 2.0 * math.pi / n_pts + 0.4
        tx, ty = target_r * math.cos(ang), target_r * math.sin(ang)
        best = None
        best_i = -1
        best_d = float("inf")
        pool = [j for j in range(len(pts)) if j not in used] or list(range(len(pts)))
        for j in pool:
            x, y = pts[j]
            d = (x - tx) ** 2 + (y - ty) ** 2
            if d < best_d:
                best_d, best_i, best = d, j, (x, y)
        used.add(best_i)
        out.append(best)
    return out
