#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W XY运动台非正交校准仿真 — 一阶M法 / LUT法 合并版
参考文档: SDS §6.1 (XY运动轴非正交校准附录), SSPS §3.2.5/3.2.6,
          STPS FR11/FR12 (LUT) 与 对准台/硅片台非正交校准, SSDS 序号10/11
算法来源: non_orthogonal_model_PCA.py (PCA扫描线拟合, 已剔除干涉仪相关部分,
          测量端为高精度对准相机 + 标准标记硅片)

仿真闭环 (算法在环):
  真值误差场 → 采集过程 → 标定算法 → 补偿 → 验证残差
  ───────────────────────────────────────────
  真值: A = [[1+εx, sinφ],[0, (1+εy)cosφ]] (一阶量: 非正交角φ + 比例εx/εy)
        + 高阶场 e(u): X/Y导轨直线度弓形 + 偏摆(yaw)耦合  ← M无法补偿, LUT可以
        + 上片倾斜θp (标定算法必须先去除, 否则会被当成非正交)
  采集: 运动台带标准硅片走X线/Y线(一阶法)与网格(LUT法), 相机逐点对准,
        记录"对准成功时的指令坐标" = A⁻¹·R(θp)·名义 + 高阶 + 测量噪声
  一阶法: PCA拟合两条扫描线方向 → 以X为基准去上片倾斜 → 解 φ估计, a估计, d估计 → 补偿矩阵M_hat
  LUT法: 网格刚体拟合去上片倾斜 → 节点误差表 → 双线性插值补偿
  验证: 同一验证网格上比较 未校准/M_hat校准/LUT校准 的残差 (各自刚体去除后, 公平比较)

验收判据 (无固定Spec, 由模型推导):
  φ估计 标定一致性区间 [MC P2.5, P97.5] (对应STPS"多次测量标准差足够小")
  校准后验证残差RMS区间, 与解析误差传播交叉校验:
    σ_φ估计 = sqrt(2)·σ_меas / (pitch·sqrt(K(K²−1)/12))   (直线斜率方差公式, 双线RSS)
    LUT残差 ≈ RSS(0.667·σ_meas [插值烘入噪声], 截断误差[场曲率×网格步距²/8])
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure

import simcore
from simgui import BaseSimApp, launch, FONT


# ===================================================================
#  几何工具 (源自上传脚本, 清理版)
# ===================================================================

def fit_line_pca(coords):
    """PCA/SVD 拟合点云主方向, 返回单位向量 (符号与行走方向一致).
    源自 non_orthogonal_model_PCA.fit_line_angle_pca, 去除干涉仪部分."""
    c = coords - coords.mean(axis=0)
    _, _, vh = np.linalg.svd(c, full_matrices=False)
    d = vh[0]
    if np.dot(d, coords[-1] - coords[0]) < 0:
        d = -d
    return d


def rigid_fit(P, Q):
    """2D Kabsch: 求刚体变换 Q ≈ R·P + t (无反射). 返回 (theta, R, t)."""
    Pm, Qm = P.mean(axis=0), Q.mean(axis=0)
    H = (P - Pm).T @ (Q - Qm)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = Qm - R @ Pm
    return float(np.arctan2(R[1, 0], R[0, 0])), R, t


def rigid_residual(nominal, achieved):
    """刚体去除后的残差 (mm): 整体旋转/平移不是运动台几何误差, 公平比较前先扣除."""
    _, R, t = rigid_fit(nominal, achieved)
    return achieved - (nominal @ R.T + t)


def bilinear(xs, ys, table, q):
    """规则网格 (xs[G], ys[G], table[G,G,2]) 上的双线性插值, q:(N,2). 越界夹取."""
    qx = np.clip(q[:, 0], xs[0], xs[-1] - 1e-12)
    qy = np.clip(q[:, 1], ys[0], ys[-1] - 1e-12)
    hx, hy = xs[1] - xs[0], ys[1] - ys[0]
    i = np.clip(((qx - xs[0]) / hx).astype(int), 0, len(xs) - 2)
    j = np.clip(((qy - ys[0]) / hy).astype(int), 0, len(ys) - 2)
    s = (qx - xs[i]) / hx
    t = (qy - ys[j]) / hy
    s, t = s[:, None], t[:, None]
    return ((1 - s) * (1 - t) * table[i, j] + s * (1 - t) * table[i + 1, j]
            + (1 - s) * t * table[i, j + 1] + s * t * table[i + 1, j + 1])


# ===================================================================
#  标定仿真模型 (算法在环)
# ===================================================================

class OrthoCalibModel:
    """单位约定: 位置 mm, 误差显示 nm (内部换算 1 nm = 1e-6 mm; 1 urad×mm = 1 nm)."""

    # ---------- 真值 ----------
    @staticmethod
    def truth(p):
        phi = p["phi_true_urad"] * 1e-6
        A = np.array([[1 + p["scale_x_ppm"] * 1e-6, np.sin(phi)],
                      [0.0, (1 + p["scale_y_ppm"] * 1e-6) * np.cos(phi)]])
        span = (max(3, int(p["marks_per_line"])) - 1) * p["mark_pitch_mm"]
        half = span / 2.0

        def e_high(u):
            """高阶误差场 (nm): u(N,2) mm → e(N,2) nm."""
            ux, uy = u[:, 0], u[:, 1]
            ey = p["bow_x_pv_nm"] / 2 * np.sin(
                p["bow_x_arch"] * np.pi * (ux + half) / max(span, 1e-9))
            ex = p["bow_y_pv_nm"] / 2 * np.sin(
                p["bow_y_arch"] * np.pi * (uy + half) / max(span, 1e-9))
            psi = p["yaw_pv_urad"] / 2 * np.sin(np.pi * (ux + half) / max(span, 1e-9))
            ex = ex - psi * uy          # urad × mm = nm
            return np.column_stack([ex, ey])

        return A, e_high, span

    @staticmethod
    def forward(A, e_high, u):
        return u @ A.T + e_high(u) * 1e-6

    @staticmethod
    def inverse(A, e_high, T):
        """求对准成功时的指令坐标: forward(u)=T 的不动点迭代."""
        Ainv = np.linalg.inv(A)
        u = T @ Ainv.T
        for _ in range(3):
            u = (T - e_high(u) * 1e-6) @ Ainv.T
        return u

    # ---------- 采集 ----------
    @staticmethod
    def collect(p, rng):
        A, e_high, span = OrthoCalibModel.truth(p)
        K = max(3, int(p["marks_per_line"]))
        G = max(3, int(p["lut_grid"]))
        m = max(1, int(p["frames_per_mark"]))
        sigma_mm = np.sqrt(p["cam_sigma_nm"] ** 2 + p["stage_rep_nm"] ** 2) / np.sqrt(m) * 1e-6
        half = span / 2.0
        th_p = p["plate_tilt_urad"] * 1e-6
        Rp = np.array([[np.cos(th_p), -np.sin(th_p)], [np.sin(th_p), np.cos(th_p)]])
        t0 = np.array([p.get("plate_off_x_mm", 0.0) or 0.0,
                       p.get("plate_off_y_mm", 0.0) or 0.0])

        def measure(nominal):
            T = nominal @ Rp.T + t0
            u = OrthoCalibModel.inverse(A, e_high, T)
            return u + rng.normal(0, sigma_mm, u.shape)

        ax_ = np.linspace(-half, half, K)
        x_nom = np.column_stack([ax_, np.zeros(K)])
        y_nom = np.column_stack([np.zeros(K), ax_])
        gax = np.linspace(-half, half, G)
        gx, gy = np.meshgrid(gax, gax, indexing="ij")
        g_nom = np.column_stack([gx.ravel(), gy.ravel()])
        return {"A": A, "e_high": e_high, "span": span, "sigma_mm": sigma_mm,
                "x_nom": x_nom, "x_rec": measure(x_nom),
                "y_nom": y_nom, "y_rec": measure(y_nom),
                "g_nom": g_nom, "g_rec": measure(g_nom),
                "gax": gax, "G": G, "K": K}

    # ---------- 一阶M法 (PCA扫描线, 以X为基准去上片倾斜) ----------
    @staticmethod
    def calibrate_M(d):
        x_nom, x_rec = d["x_nom"], d["x_rec"]
        y_nom, y_rec = d["y_nom"], d["y_rec"]
        dir_x = fit_line_pca(x_rec)
        dir_y = fit_line_pca(y_rec)
        # 比例: 名义弧长 vs 投影坐标的回归斜率 (用全部点, 比首尾差更稳健)
        sx_nom = x_nom[:, 0] - x_nom[:, 0].mean()
        sy_nom = y_nom[:, 1] - y_nom[:, 1].mean()
        px = (x_rec - x_rec.mean(axis=0)) @ dir_x
        py = (y_rec - y_rec.mean(axis=0)) @ dir_y
        sx = float(np.polyfit(sx_nom, px, 1)[0])
        sy = float(np.polyfit(sy_nom, py, 1)[0])
        D = np.column_stack([dir_x * sx, dir_y * sy])      # 名义→指令 (含上片倾斜)
        # 精确去上片倾斜 (RQ式分解): D = A⁻¹·R(θp) → A = R(θp)·D⁻¹.
        # 在 SDS"X为基准, c≈0"约定下, θp 由强制 A[1,0]=0 精确解出 —
        # 避免小角近似留下的 ~φ·θp·行程 二阶耦合残差.
        B = np.linalg.inv(D)                               # = R⁻¹·A
        th_p = float(np.arctan2(-B[1, 0], B[0, 0]))        # sinθ·B00+cosθ·B10=0
        c, s = np.cos(th_p), np.sin(th_p)
        M = np.array([[c, -s], [s, c]]) @ B                # ≈ A (M[1,0]≈0)
        C = np.linalg.inv(M)                               # 补偿: 指令 u = C·期望
        th_x = th_p
        phi_hat = float(np.arctan2(M[0, 1], M[1, 1]))
        # 扫描线法向残差 (nm) → 弓形诊断
        res_x = ((x_rec - x_rec.mean(axis=0))
                 - np.outer(px, dir_x)) @ np.array([-dir_x[1], dir_x[0]]) * 1e6
        res_y = ((y_rec - y_rec.mean(axis=0))
                 - np.outer(py, dir_y)) @ np.array([-dir_y[1], dir_y[0]]) * 1e6
        return {"M": M, "C": C, "phi_hat_urad": phi_hat * 1e6,
                "a_ppm": (M[0, 0] - 1) * 1e6,
                "d_ppm": (float(np.hypot(M[0, 1], M[1, 1])) - 1) * 1e6,
                "plate_tilt_urad": th_x * 1e6,
                "line_s_x": sx_nom, "line_res_x": res_x,
                "line_s_y": sy_nom, "line_res_y": res_y}

    # ---------- LUT法 (网格刚体去倾斜 + 节点误差表) ----------
    @staticmethod
    def build_lut(d):
        g_nom, g_rec, G = d["g_nom"], d["g_rec"], d["G"]
        th, R, t = rigid_fit(g_nom, g_rec)
        corr = (g_rec - (g_nom @ R.T + t)).reshape(G, G, 2)   # mm, 规则名义网格上
        return {"theta_urad": th * 1e6, "R": R, "t": t,
                "corr": corr, "gax": d["gax"]}

    # ---------- 验证: 三种策略在同一验证网格上的残差 (各自刚体去除) ----------
    @staticmethod
    def validate(p, d, calM, lut):
        A, e_high = d["A"], d["e_high"]
        V = max(5, int(p["val_grid"]))
        margin = (d["gax"][1] - d["gax"][0]) * 0.5
        vax = np.linspace(d["gax"][0] + margin, d["gax"][-1] - margin, V)
        vx, vy = np.meshgrid(vax, vax, indexing="ij")
        q = np.column_stack([vx.ravel(), vy.ravel()])         # 期望位移 (基准系)

        u_un = q
        u_M = q @ calM["C"].T
        u_L = q @ lut["R"].T + lut["t"] + bilinear(lut["gax"], lut["gax"], lut["corr"], q)

        out = {"q": q, "V": V, "vax": vax}
        for tag, u in (("uncal", u_un), ("M", u_M), ("LUT", u_L)):
            ach = OrthoCalibModel.forward(A, e_high, u)
            res = rigid_residual(q, ach) * 1e6                # nm
            mag = np.linalg.norm(res, axis=1)
            out[tag] = {"res": res, "mag": mag,
                        "rms": float(np.sqrt(np.mean(mag ** 2))),
                        "max": float(np.max(mag))}
        return out

    # ---------- 单次完整流程 ----------
    @staticmethod
    def run(p, seed=None):
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))
        d = OrthoCalibModel.collect(p, rng)
        calM = OrthoCalibModel.calibrate_M(d)
        lut = OrthoCalibModel.build_lut(d)
        val = OrthoCalibModel.validate(p, d, calM, lut)
        return {"params": dict(p), "data": d, "calM": calM, "lut": lut, "val": val}

    # ---------- 解析误差传播 ----------
    @staticmethod
    def analytical(p):
        K = max(3, int(p["marks_per_line"]))
        m = max(1, int(p["frames_per_mark"]))
        pitch = p["mark_pitch_mm"]
        sigma_nm = np.sqrt(p["cam_sigma_nm"] ** 2 + p["stage_rep_nm"] ** 2) / np.sqrt(m)
        Sxx = pitch ** 2 * K * (K ** 2 - 1) / 12.0            # mm², Σ(s-s̄)²
        s_theta = sigma_nm * 1e-6 / np.sqrt(Sxx)              # rad, 单线斜率
        s_phi = np.sqrt(2) * s_theta * 1e6                    # urad, 双线RSS
        s_scale = sigma_nm * 1e-6 / np.sqrt(Sxx) * 1e6        # ppm (同公式, 平行分量)
        span = (K - 1) * pitch
        # LUT: 烘入噪声 + 双线性截断. 口径与MC一致: 二维模长RMS = sqrt(E[ex²+ey²])
        G = max(3, int(p["lut_grid"]))
        h = span / (G - 1)
        baked = np.sqrt(2.0) * np.sqrt(4.0 / 9.0) * sigma_nm   # 双轴模长: √2 × 0.667σ
        trunc_terms = []
        for pv, narch in ((p["bow_x_pv_nm"], p["bow_x_arch"]),
                          (p["bow_y_pv_nm"], p["bow_y_arch"])):
            curv = pv / 2 * (narch * np.pi / max(span, 1e-9)) ** 2   # nm/mm² (峰值曲率)
            trunc_terms.append(curv * h ** 2 / 8.0)                  # 单胞最大截断
        # 0.4: 正弦场的经验形状系数 (胞内分布×全场曲率分布×刚体吸收, 量级参考)
        trunc = 0.4 * float(np.sqrt(np.sum(np.square(trunc_terms))))
        return {"sigma_phi_urad": s_phi, "three_sigma_phi_urad": 3 * s_phi,
                "sigma_scale_ppm": s_scale, "sigma_meas_nm": sigma_nm,
                "lut_baked_nm": baked, "lut_trunc_nm": trunc,
                "lut_rms_pred_nm": float(np.hypot(baked, trunc)),
                "lut_pitch_mm": h, "span_mm": span, "K": K, "G": G}

    # ---------- 蒙特卡洛 ----------
    @staticmethod
    def monte_carlo(p, runs=200, seed0=20000):
        phi = np.empty(runs); a = np.empty(runs); dd = np.empty(runs)
        rM = np.empty(runs); rL = np.empty(runs); xL = np.empty(runs)
        for i in range(runs):
            r = OrthoCalibModel.run(p, seed=seed0 + i)
            phi[i] = r["calM"]["phi_hat_urad"]
            a[i] = r["calM"]["a_ppm"]; dd[i] = r["calM"]["d_ppm"]
            rM[i] = r["val"]["M"]["rms"]; rL[i] = r["val"]["LUT"]["rms"]
            xL[i] = r["val"]["LUT"]["max"]
        def stats(v):
            return {"mean": float(np.mean(v)), "std": float(np.std(v, ddof=1)),
                    "p2_5": float(np.percentile(v, 2.5)),
                    "p97_5": float(np.percentile(v, 97.5)),
                    "arr": v}
        return {"runs": runs, "phi": stats(phi), "a": stats(a), "d": stats(dd),
                "rms_M": stats(rM), "rms_LUT": stats(rL), "max_LUT": stats(xL)}

    # ---------- 诊断 ----------
    @staticmethod
    def diagnose(p, R, ana, mc):
        f = []
        calM, val = R["calM"], R["val"]
        # 1) 扫描线弓形显著性 (二次项 t 检验)
        for lbl, s, res in (("X", calM["line_s_x"], calM["line_res_x"]),
                            ("Y", calM["line_s_y"], calM["line_res_y"])):
            if len(s) >= 5:
                A2 = np.column_stack([s ** 2, s, np.ones_like(s)])
                coef, *_ = np.linalg.lstsq(A2, res, rcond=None)
                fit = A2 @ coef
                dof = len(s) - 3
                se = np.sqrt(np.sum((res - fit) ** 2) / max(dof, 1)
                             / np.sum((s ** 2 - np.mean(s ** 2)) ** 2))
                tq = abs(coef[0]) / se if se > 0 else 0
                bow_pv = float(np.ptp(fit))
                if tq > 4 and bow_pv > 3 * ana["sigma_meas_nm"]:
                    f.append(("warn", f"{lbl}扫描线弓形显著 (PV≈{bow_pv:.0f} nm)",
                              f"法向残差二次项 t={tq:.1f}, 直线度误差超出噪声水平",
                              "一阶M法无法补偿弓形 → 该分量将全部留在M法残差中, 建议采用LUT补偿"))
        # 2) M vs LUT
        if val["M"]["rms"] > 2.0 * val["LUT"]["rms"]:
            f.append(("warn", "高阶误差主导",
                      f"M法残差RMS={val['M']['rms']:.1f} nm ≫ LUT残差RMS={val['LUT']['rms']:.1f} nm",
                      "直线度/偏摆等高阶分量显著, 一阶非正交校准不足, 投产建议使用LUT"))
        else:
            f.append(("ok", "一阶量主导",
                      f"M法残差RMS={val['M']['rms']:.1f} nm 与 LUT残差RMS={val['LUT']['rms']:.1f} nm 接近",
                      "非正交+比例为主, M法已基本足够; LUT收益有限"))
        # 3) LUT 限制因素
        if ana["lut_baked_nm"] > 1.5 * ana["lut_trunc_nm"]:
            f.append(("warn", "LUT精度受测量噪声限制",
                      f"烘入噪声≈{ana['lut_baked_nm']:.1f} nm > 截断误差≈{ana['lut_trunc_nm']:.1f} nm",
                      "加密网格无益; 优先增加每点采集帧数 m 或降低单点噪声"))
        elif ana["lut_trunc_nm"] > 1.5 * ana["lut_baked_nm"]:
            f.append(("warn", "LUT精度受网格截断限制",
                      f"截断误差≈{ana['lut_trunc_nm']:.1f} nm > 烘入噪声≈{ana['lut_baked_nm']:.1f} nm",
                      f"建议加密LUT网格 (当前步距 {ana['lut_pitch_mm']:.1f} mm)"))
        # 4) 上片倾斜
        if abs(calM["plate_tilt_urad"]) > 2000:
            f.append(("warn", f"上片倾斜偏大 ({calM['plate_tilt_urad']:.0f} urad)",
                      "倾斜过大时小角近似与视场捕获均受影响",
                      "建议预对准后重新上片 (<1 mrad), 再执行标定"))
        # 5) φ估计 一致性与系统偏差 (仿真真值视角)
        bias = mc["phi"]["mean"] - p["phi_true_urad"]
        if abs(bias) > 2 * max(mc["phi"]["std"], 1e-12):
            f.append(("warn", f"一阶法 φ估计 存在系统偏差 ({bias:+.2f} urad)",
                      "高阶场(弓形/偏摆)使扫描线整体微倾, 一阶拟合将其部分吸收为非正交角; "
                      "真机上不可直接观测 (多次标定仍一致), 等效位置误差 ≈ 偏差×半行程",
                      "若不可接受 → 改用LUT (对高阶场免疫), 或增加扫描线条数/对称布线平均弓形"))
        f.append(("info", "φ估计 标定一致性",
                  f"MC std={mc['phi']['std']:.3f} urad vs 解析 1σ={ana['sigma_phi_urad']:.3f} urad "
                  f"(比值 {mc['phi']['std']/max(ana['sigma_phi_urad'],1e-12):.2f}; 含高阶场引入的系统分量)",
                  "若实测多次标定的离散明显大于该区间 → 排查标记质量/对准得分/温漂"))
        return f


# ===================================================================
#  GUI
# ===================================================================

class OrthoCalibApp(BaseSimApp):
    title = "D2W - XY运动台非正交校准仿真 (一阶M法 / LUT法)"
    subtitle = "XY非正交校准 算法在环  |  Stage Orthogonality: First-order M vs LUT"
    report_name = "ortho_calib"
    model = OrthoCalibModel

    # ---------------- schema ----------------
    def schema(self):
        span_fn = lambda p: f"{(max(3, int(p['marks_per_line'])) - 1) * p['mark_pitch_mm']:.0f}"
        lutp_fn = lambda p: (f"{(max(3, int(p['marks_per_line'])) - 1) * p['mark_pitch_mm'] / (max(3, int(p['lut_grid'])) - 1):.1f}")
        return [
            ("section", "采集方案"),
            ("entry", "marks_per_line", "扫描线标记数 K", 15, "个"),
            ("entry", "mark_pitch_mm", "标记节距", 10.0, "mm"),
            ("derived", "_span", "工作行程 (自动)", "mm", span_fn,
             ["marks_per_line", "mark_pitch_mm"]),
            ("entry", "lut_grid", "LUT 网格 G×G", 9, "G"),
            ("derived", "_lutp", "LUT 网格步距 (自动)", "mm", lutp_fn,
             ["marks_per_line", "mark_pitch_mm", "lut_grid"]),
            ("entry", "frames_per_mark", "每点采集帧数", 3, "帧"),
            ("entry", "val_grid", "验证网格 V×V", 21, "V"),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 200, "次"),
            ("note", "采集流程: 运动台带标准标记硅片, 相机固定, 逐点对准并记录指令坐标. "
                     "一阶法用X/Y两条扫描线, LUT法用G×G网格 (同一片同一相机)"),

            ("section", "真值: 一阶量 (待标定对象)"),
            ("entry", "phi_true_urad", "非正交角 φ (真值)", 100.0, "urad"),
            ("entry", "scale_x_ppm", "X 比例误差", 20.0, "ppm"),
            ("entry", "scale_y_ppm", "Y 比例误差", -15.0, "ppm"),
            ("entry", "plate_tilt_urad", "上片倾斜 θp", 500.0, "urad"),
            ("note", "上片倾斜是干扰量: 算法以X扫描线为基准去除. "
                     "若不去除会被当成非正交 — 可设 θp≫φ 验证算法稳健性"),

            ("section", "真值: 高阶量 (M法盲区, LUT可补偿)"),
            ("entry", "bow_x_pv_nm", "X导轨直线度弓形 PV", 150.0, "nm"),
            ("entry", "bow_x_arch", "X 弓形拱数", 1.0, "拱"),
            ("entry", "bow_y_pv_nm", "Y导轨直线度弓形 PV", 120.0, "nm"),
            ("entry", "bow_y_arch", "Y 弓形拱数", 1.0, "拱"),
            ("entry", "yaw_pv_urad", "偏摆(yaw)沿程 PV", 1.0, "urad"),
            ("note", "yaw耦合: X滑台偏摆ψ(x)使搭载的Y轴整体微转 → ex≈−ψ(x)·y, "
                     "属典型阿贝/姿态误差, 一阶矩阵无法表达"),

            ("section", "测量噪声"),
            ("entry", "cam_sigma_nm", "相机单点对准 1σ", 8.0, "nm"),
            ("entry", "stage_rep_nm", "运动台到位重复性 1σ", 25.0, "nm"),
            ("note", "单点有效噪声 σ_meas = RSS(相机, 到位)/√帧数. "
                     "验收区间在[验收参考]页由MC推导并与解析误差传播交叉校验"),
        ]

    # ---------------- run pipeline (覆写: 标定类流程) ----------------
    def _run(self):
        from tkinter import messagebox
        try:
            p = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        R = OrthoCalibModel.run(p)
        ana = OrthoCalibModel.analytical(p)
        mc = OrthoCalibModel.monte_carlo(p, runs=max(50, int(p.get("mc_runs") or 200)))
        diag = OrthoCalibModel.diagnose(p, R, ana, mc)
        self.R = {"result": R, "ana": ana, "mc": mc, "diag": diag}
        self._status_calib()
        self._render_all()

    def _status_calib(self):
        R, ana, mc = self.R["result"], self.R["ana"], self.R["mc"]
        phi = R["calM"]["phi_hat_urad"]
        in_ci = mc["phi"]["p2_5"] <= phi <= mc["phi"]["p97_5"]
        rL = R["val"]["LUT"]["rms"]; rM = R["val"]["M"]["rms"]
        ok = in_ci and rL <= mc["rms_LUT"]["p97_5"] * 1.0001
        bg = self.C["pass_bg"] if ok else self.C["ref_bg"]
        fg = self.C["pass_fg"] if ok else self.C["warn"]
        tag = "正常·位于验收区间" if ok else "超出验收区间 → 见诊断页"
        self.sfr.config(bg=bg)
        self.sl.config(bg=bg, fg=fg, font=(FONT, 11, "bold"), text=(
            f"[{tag}]  φ估计={phi:.2f} urad (区间[{mc['phi']['p2_5']:.2f},{mc['phi']['p97_5']:.2f}])"
            f"   残差RMS: M法={rM:.1f}  LUT={rL:.1f} nm"
            f" (LUT上限{mc['rms_LUT']['p97_5']:.1f})  | 运行#{self.sim_n}"))

    # ---------------- tabs (整体覆写) ----------------
    def standard_tabs(self):
        return [("误差场与采样", self._tab_field),
                ("一阶校准 (M法)", self._tab_M),
                ("LUT 校准", self._tab_lut),
                ("方法对比", self._tab_compare),
                ("诊断与建议", self._tab_diag_c),
                ("验收参考", self._tab_accept_c)]

    def _tab_field(self, tab, RR):
        R = RR["result"]; d = R["data"]
        q = d["g_nom"]
        A, e_high = d["A"], d["e_high"]
        err = (OrthoCalibModel.forward(A, e_high, q) - q) * 1e6   # nm (含一阶+高阶)
        fig = Figure(figsize=(10, 6), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, "真值误差场 (指令→实际, 含一阶+高阶)", "X (mm)", "Y (mm)")
        ax.quiver(q[:, 0], q[:, 1], err[:, 0], err[:, 1],
                  np.linalg.norm(err, axis=1), cmap="viridis",
                  angles="xy", width=0.004)
        ax.set_aspect("equal")
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "采样布置", "X (mm)", "Y (mm)")
        ax2.scatter(d["g_nom"][:, 0], d["g_nom"][:, 1], s=14, c="#c8cdd4",
                    label=f"LUT网格 {d['G']}×{d['G']}")
        ax2.scatter(d["x_nom"][:, 0], d["x_nom"][:, 1], s=26, c=self.C["cx"],
                    label=f"X扫描线 K={d['K']}")
        ax2.scatter(d["y_nom"][:, 0], d["y_nom"][:, 1], s=26, c=self.C["cy"],
                    label=f"Y扫描线 K={d['K']}")
        ax2.set_aspect("equal"); ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_M(self, tab, RR):
        R = RR["result"]; cal = R["calM"]; p = R["params"]
        fig = Figure(figsize=(10, 6), dpi=100, facecolor=self.C["bg"])
        for i, (lbl, s, res, col) in enumerate(
                [("X", cal["line_s_x"], cal["line_res_x"], self.C["cx"]),
                 ("Y", cal["line_s_y"], cal["line_res_y"], self.C["cy"])]):
            ax = fig.add_subplot(2, 1, i + 1)
            self._ax(ax, f"{lbl}扫描线法向残差 (去拟合直线后) — 弓形/直线度可见",
                     "沿线位置 (mm)", "法向残差 (nm)")
            ax.plot(s, res, "o-", color=col, ms=4, lw=0.9)
            ax.axhline(0, color=self.C["dim"], lw=0.6)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        M = cal["M"]
        txt = (f"标定输出:  φ估计 = {cal['phi_hat_urad']:.3f} urad  (真值 {p['phi_true_urad']:.1f},"
               f" 偏差 {cal['phi_hat_urad']-p['phi_true_urad']:+.3f})    "
               f"a估计 = {cal['a_ppm']:+.1f} ppm (真值 {p['scale_x_ppm']:+.1f})    "
               f"d估计 = {cal['d_ppm']:+.1f} ppm (真值 {p['scale_y_ppm']:+.1f})\n"
               f"上片倾斜估计 θp估计 = {cal['plate_tilt_urad']:.1f} urad (真值 {p['plate_tilt_urad']:.1f})"
               f"    补偿矩阵 M_hat = [[{M[0,0]:.8f}, {M[0,1]:.2e}], [{M[1,0]:.2e}, {M[1,1]:.8f}]]"
               f"   (指令侧使用 M_hat⁻¹)")
        tk.Label(tab, text=txt, bg=self.C["card"], fg=self.C["text"], font=("Consolas", 9),
                 justify="left", anchor="w", padx=10, pady=8).pack(fill="x", padx=8, pady=(0, 8))

    def _tab_lut(self, tab, RR):
        R = RR["result"]; lut = R["lut"]; val = R["val"]
        corr = lut["corr"] * 1e6   # nm
        V = val["V"]
        fig = Figure(figsize=(11, 4.2), dpi=100, facecolor=self.C["bg"])
        ext = [lut["gax"][0], lut["gax"][-1], lut["gax"][0], lut["gax"][-1]]
        for i, (z, ttl) in enumerate([(corr[:, :, 0], "LUT 节点修正量 X (nm)"),
                                      (corr[:, :, 1], "LUT 节点修正量 Y (nm)")]):
            ax = fig.add_subplot(1, 3, i + 1)
            self._ax(ax, ttl, "X (mm)", "Y (mm)")
            im = ax.imshow(z.T, origin="lower", extent=ext, cmap="RdBu_r", aspect="equal")
            fig.colorbar(im, ax=ax, shrink=0.8)
        ax3 = fig.add_subplot(133)
        self._ax(ax3, f"LUT补偿后残差 |e| (nm)  RMS={val['LUT']['rms']:.1f}",
                 "X (mm)", "Y (mm)")
        im = ax3.imshow(val["LUT"]["mag"].reshape(V, V).T, origin="lower",
                        extent=[val["vax"][0], val["vax"][-1]] * 2,
                        cmap="viridis", aspect="equal")
        fig.colorbar(im, ax=ax3, shrink=0.8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        ana = RR["ana"]
        tk.Label(tab, text=(
            f"刚体去除: 上片倾斜 θ估计 = {lut['theta_urad']:.1f} urad   |   "
            f"解析预测 LUT残差 ≈ RSS(烘入噪声 {ana['lut_baked_nm']:.1f}, "
            f"截断 {ana['lut_trunc_nm']:.1f}) = {ana['lut_rms_pred_nm']:.1f} nm   "
            f"(实际 {val['LUT']['rms']:.1f} nm)"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x", padx=8, pady=(0, 8))

    def _tab_compare(self, tab, RR):
        R = RR["result"]; val = R["val"]; V = val["V"]
        fig = Figure(figsize=(11, 4.6), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(131)
        self._ax(ax, "验证残差对比 (刚体去除后)", "", "nm")
        tags = ["uncal", "M", "LUT"]; names = ["未校准", "一阶M法", "LUT法"]
        rms = [val[t]["rms"] for t in tags]; mx = [val[t]["max"] for t in tags]
        xp = np.arange(3)
        ax.bar(xp - 0.18, rms, 0.36, color=self.C["cx"], label="RMS")
        ax.bar(xp + 0.18, mx, 0.36, color=self.C["cy"], alpha=0.6, label="Max")
        for x0, v in zip(xp, rms):
            ax.text(x0 - 0.18, v, f"{v:.0f}", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(xp); ax.set_xticklabels(names, fontsize=9)
        ax.set_yscale("log"); ax.legend(fontsize=8)
        vext = [val["vax"][0], val["vax"][-1]] * 2
        vmax = max(val["M"]["mag"].max(), val["LUT"]["mag"].max())
        for i, (t, ttl) in enumerate([("M", "M法残差 |e| (nm)"), ("LUT", "LUT残差 |e| (nm)")]):
            ax2 = fig.add_subplot(1, 3, i + 2)
            self._ax(ax2, f"{ttl}  RMS={val[t]['rms']:.1f}", "X (mm)", "Y (mm)")
            im = ax2.imshow(val[t]["mag"].reshape(V, V).T, origin="lower", extent=vext,
                            cmap="viridis", vmin=0, vmax=vmax, aspect="equal")
            fig.colorbar(im, ax=ax2, shrink=0.8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_diag_c(self, tab, RR):
        from tkinter import ttk
        wrap = tk.Frame(tab, bg=self.C["bg"]); wrap.pack(fill="both", expand=True)
        txt = tk.Text(wrap, bg=self.C["panel"], fg=self.C["text"], font=(FONT, 10),
                      relief="flat", wrap="word", padx=14, pady=12, spacing3=4)
        sb = ttk.Scrollbar(wrap, command=txt.yview); txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y"); txt.pack(fill="both", expand=True, padx=8, pady=8)
        for tag, fgk, bold in (("h", "accent", True), ("warn", "fail_fg", True),
                               ("ok", "pass_fg", True), ("info", "accent2", True),
                               ("dim", "dim", False)):
            txt.tag_config(tag, foreground=self.C[fgk],
                           font=(FONT, 11 if tag == "h" else 10, "bold" if bold else "normal"))
        txt.insert("end", "标定诊断与工程建议\n", "h")
        R = RR["result"]
        txt.insert("end", f"φ估计 = {R['calM']['phi_hat_urad']:.3f} urad   "
                          f"残差RMS: 未校准 {R['val']['uncal']['rms']:.1f} / "
                          f"M法 {R['val']['M']['rms']:.1f} / LUT {R['val']['LUT']['rms']:.1f} nm\n\n")
        mark = {"warn": "⚠ ", "ok": "✓ ", "info": "ℹ "}
        for sev, t, de, adv in RR["diag"]:
            txt.insert("end", mark.get(sev, "") + t + "\n", sev)
            txt.insert("end", f"   现象: {de}\n", "dim")
            txt.insert("end", f"   建议: {adv}\n\n")
        txt.config(state="disabled")

    def _tab_accept_c(self, tab, RR):
        ana, mc = RR["ana"], RR["mc"]; p = RR["result"]["params"]
        info = tk.Frame(tab, bg=self.C["ref_bg"]); info.pack(fill="x", padx=10, pady=(10, 4))
        tk.Label(info, text=(
            f"建议验收区间 (由当前误差模型MC推导, N={mc['runs']}):\n"
            f"   非正交角 φ估计 ∈ [{mc['phi']['p2_5']:.2f}, {mc['phi']['p97_5']:.2f}] urad "
            f"(标定一致性 3σ = {3*mc['phi']['std']:.2f} urad; 解析噪声极限 3σ = {ana['three_sigma_phi_urad']:.2f} urad)\n"
            f"   比例 a估计 ∈ [{mc['a']['p2_5']:.1f}, {mc['a']['p97_5']:.1f}] ppm,  "
            f"d估计 ∈ [{mc['d']['p2_5']:.1f}, {mc['d']['p97_5']:.1f}] ppm\n"
            f"   LUT校准后残差 RMS ≤ {mc['rms_LUT']['p97_5']:.1f} nm (P97.5), "
            f"Max ≤ {mc['max_LUT']['p97_5']:.1f} nm;  M法残差 RMS ≤ {mc['rms_M']['p97_5']:.1f} nm\n"
            f"判读: 实测标定值落于区间内且多次标定std≤区间宽度 → 正常; 超出 → 按诊断页排查"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(11, 4), dpi=100, facecolor=self.C["bg"])
        specs = [("phi", "φ估计 (urad)", self.C["cx"], p["phi_true_urad"], "真值"),
                 ("rms_M", "M法残差RMS (nm)", self.C["cy"], None, None),
                 ("rms_LUT", "LUT残差RMS (nm)", "#1a7f37", ana["lut_rms_pred_nm"], "解析预测")]
        for i, (k, ttl, col, ref, refl) in enumerate(specs):
            ax = fig.add_subplot(1, 3, i + 1)
            self._ax(ax, f"MC分布: {ttl}", ttl, "频次")
            s = mc[k]
            ax.hist(s["arr"], bins=26, color=col, alpha=0.7, edgecolor="white", lw=0.4)
            ax.axvspan(s["p2_5"], s["p97_5"], alpha=0.10, color=self.C["ref"])
            for q in ("p2_5", "p97_5"):
                ax.axvline(s[q], color="#e5a00d", lw=1.2, ls="--")
            if ref is not None:
                ax.axvline(ref, color=self.C["ref"], lw=2, ls="-.", label=f"{refl}={ref:.2f}")
                ax.legend(fontsize=7)
        fig.tight_layout(pad=2.2)
        self._embed(fig, tab)

    # ---------------- export ----------------
    def _export(self):
        from tkinter import filedialog, messagebox
        from datetime import datetime
        if not self.R:
            messagebox.showinfo("提示", "请先运行仿真"); return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Text", "*.txt")],
            initialfile=f"{self.report_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        R, ana, mc = self.R["result"], self.R["ana"], self.R["mc"]
        p = R["params"]; cal = R["calM"]; val = R["val"]
        L = ["=" * 76, f"  {self.title}", "=" * 76,
             f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   simcore v{simcore.__version__}",
             f"  Layout: K={ana['K']} pitch={p['mark_pitch_mm']} mm span={ana['span_mm']:.0f} mm, "
             f"LUT {ana['G']}x{ana['G']} (h={ana['lut_pitch_mm']:.1f} mm), frames/mark={int(p['frames_per_mark'])}",
             "", "-" * 76, "  CALIBRATION RESULT", "-" * 76,
             f"  phi_hat = {cal['phi_hat_urad']:.3f} urad   (truth {p['phi_true_urad']:.1f}, "
             f"err {cal['phi_hat_urad']-p['phi_true_urad']:+.3f})",
             f"  a_hat   = {cal['a_ppm']:+.2f} ppm (truth {p['scale_x_ppm']:+.1f})   "
             f"d_hat = {cal['d_ppm']:+.2f} ppm (truth {p['scale_y_ppm']:+.1f})",
             f"  plate tilt: est {cal['plate_tilt_urad']:.1f} / truth {p['plate_tilt_urad']:.1f} urad",
             f"  M_hat = [[{cal['M'][0,0]:.8f}, {cal['M'][0,1]:.3e}], "
             f"[{cal['M'][1,0]:.3e}, {cal['M'][1,1]:.8f}]]",
             "", "-" * 76, "  VALIDATION RESIDUAL (rigid-removed, nm)", "-" * 76,
             f"  uncalibrated: RMS={val['uncal']['rms']:.1f}  Max={val['uncal']['max']:.1f}",
             f"  M method:     RMS={val['M']['rms']:.1f}  Max={val['M']['max']:.1f}",
             f"  LUT method:   RMS={val['LUT']['rms']:.1f}  Max={val['LUT']['max']:.1f}",
             "", "-" * 76, "  ACCEPTANCE REFERENCE (MC, N=%d)" % mc["runs"], "-" * 76,
             f"  phi_hat interval: [{mc['phi']['p2_5']:.2f}, {mc['phi']['p97_5']:.2f}] urad  "
             f"(repeatability 3sigma={3*mc['phi']['std']:.2f}; analytic noise-floor 3sigma={ana['three_sigma_phi_urad']:.2f})",
             f"  a interval: [{mc['a']['p2_5']:.1f}, {mc['a']['p97_5']:.1f}] ppm   "
             f"d interval: [{mc['d']['p2_5']:.1f}, {mc['d']['p97_5']:.1f}] ppm",
             f"  LUT residual RMS <= {mc['rms_LUT']['p97_5']:.1f} nm (P97.5), "
             f"Max <= {mc['max_LUT']['p97_5']:.1f} nm",
             f"  M residual RMS <= {mc['rms_M']['p97_5']:.1f} nm (P97.5)",
             f"  LUT analytic prediction: RSS(baked {ana['lut_baked_nm']:.1f}, "
             f"trunc {ana['lut_trunc_nm']:.1f}) = {ana['lut_rms_pred_nm']:.1f} nm",
             "", "-" * 76, "  DIAGNOSIS", "-" * 76]
        for sev, t, de, adv in self.R["diag"]:
            L += [f"  [{sev.upper()}] {t}", f"      现象: {de}", f"      建议: {adv}"]
        L += ["", "-" * 76, "  PARAMETERS", "-" * 76]
        for k, v in p.items():
            L.append(f"  {k}: {v}")
        L += ["", "=" * 76]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(L))
        messagebox.showinfo("OK", f"已保存:\n{path}")

    def _imp_csv(self):
        from tkinter import messagebox
        messagebox.showinfo("说明",
            "标定类项目的实测数据格式: type,nom_x_mm,nom_y_mm,rec_x_mm,rec_y_mm\n"
            "type: X/Y(扫描线) 或 G(网格)。该功能将在确认实测数据格式后接入\n"
            "(同一套 calibrate_M / build_lut 算法可直接处理实测数据)。")


if __name__ == "__main__":
    launch(OrthoCalibApp)
