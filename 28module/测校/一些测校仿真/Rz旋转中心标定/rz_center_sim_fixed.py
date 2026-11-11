#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W Rz旋转测校套件仿真 — 旋转中心 / 角度精度(指令vs实测) / 角度重复性 /
                          角度增益(线性+非线性谐波) 四合一
参考文档: SDS §4.3 | 算法来源: find_rotation_center.py (2D SVD, 忠实移植)
角度单位: 全部弧度 (rad)

前三/四项共享同一次全周扫描数据集 (与真机最经济流程一致). 四个测校项:

  1. 旋转中心标定 (find_rotation_center 2D SVD): 标记阵列绕中心旋转, 逐配对
     SVD求中心 Ĉ=(I−R̂)⁻¹T̂. 核心: 中心误差∝1/sin(θ/2), 小转角剧烈放大,
     必须用大角. 多配对按 sin²(Δθ/2) 方差加权 (大角对主导).

  2. 角度精度 (指令角 vs 实测角): 每个角位用 SVD 从标记云解出实测转角 θ̂,
     与指令角 θ_cmd 对比. 实测角 = SVD旋转矩阵的 atan2(R₁₀,R₀₀).
     残差 = θ̂ − θ_cmd, 暴露增益误差与谐波.

  3. 角度重复性: 同一指令角 N 次循环, 实测角 3σ (编码器+伺服重复性 +
     SVD测量噪声 + 热漂移随机游走时间基). 等效切向位移 = 3σ × 工作半径.

  4. 角度增益标定 (线性 + 非线性):
     - 线性: θ̂ = g·θ_cmd 的斜率 g (一阶增益/标度因子).
     - 非线性: 码盘偏心/安装产生一阶/二阶谐波角误差 A1·sin(θ+φ1)+A2·sin(2θ+φ2).
       关键: sinθ 与 θ 在任何弧段都不正交 (整周相关 k=3/π²≈0.304) →
       纯线性拟合的增益被谐波污染, 偏置 = A1·cos(θ0+φ1)·k(a),
       k(a)=3(sin a−a cos a)/a³ (a=半弧长, 整周亦不为零!).
       必须用 [θ, sinθ, cosθ, sin2θ, cos2θ] 联合拟合得无偏增益 + 谐波 → 角度LUT.

验收判据 (无固定Spec, MC推导 + 解析交叉校验):
  中心残差 / 角度精度RMS / 增益(线性vs联合两口径) / 谐波幅值 /
  补偿后残差 / 重复性3σ / 等效切向位移
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib import cm

import simcore
from simgui import BaseSimApp, launch, FONT


# ===================================================================
#  算法 (忠实移植自 find_rotation_center.py)
# ===================================================================

def find_rotation_center(p, p_prime):
    """2D SVD刚体配准求旋转中心. 返回 (C, theta_rad). 与上传脚本逐步一致."""
    p = np.asarray(p, float)
    p_prime = np.asarray(p_prime, float)
    p_mean = p.mean(axis=0)
    p_prime_mean = p_prime.mean(axis=0)
    q = p - p_mean
    q_prime = p_prime - p_prime_mean
    H = q.T @ q_prime
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    T = p_prime_mean - R @ p_mean
    C = np.linalg.inv(np.eye(2) - R) @ T
    theta = float(np.arctan2(R[1, 0], R[0, 0]))
    return C, theta


def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


# ===================================================================
#  模型
# ===================================================================

class RzSuiteModel:
    """单位: 位置 mm (残差 nm/um), 角度 rad (残差 urad), 噪声 nm.
    中心/角度精度/增益 共享一次全周扫描; 重复性单独采集."""

    # ---------- 真值: 实际转角 (增益 + 谐波) ----------
    @staticmethod
    def angle_truth(p, th_cmd):
        """指令角(rad) → 实际机械角(rad): 增益 + 一阶/二阶谐波(码盘偏心/安装)."""
        g = 1 + p["gain_err_ppm"] * 1e-6
        a1 = p["harm1_urad"] * 1e-6
        a2 = p["harm2_urad"] * 1e-6
        f1 = p["harm1_phase_rad"]
        f2 = p["harm2_phase_rad"]
        return g * th_cmd + a1 * np.sin(th_cmd + f1) + a2 * np.sin(2 * th_cmd + f2)

    @staticmethod
    def true_center(p):
        return np.array([p["center_x_mm"], p["center_y_mm"]])

    @staticmethod
    def _center_at(p, ang, rng):
        """该角位真实中心 = 名义 + 同步径向跳动(一阶) + NRRO."""
        C0 = RzSuiteModel.true_center(p)
        rs = p["runout_sync_nm"] * 1e-6
        ph = p["runout_phase_rad"]
        c = C0 + rs * np.array([np.cos(ang + ph), np.sin(ang + ph)])
        if rng is not None and p["nrro_nm"] > 0:
            c = c + rng.normal(0, p["nrro_nm"] * 1e-6, 2)
        return c

    @staticmethod
    def marks(p):
        G = max(2, int(p["marks_grid"]))
        s = p["marks_span_mm"] / 2.0
        C = RzSuiteModel.true_center(p)
        off = np.array([p["marks_offset_x_mm"], p["marks_offset_y_mm"]])
        ax_ = np.linspace(-s, s, G)
        gx, gy = np.meshgrid(ax_, ax_, indexing="ij")
        return C + off + np.column_stack([gx.ravel(), gy.ravel()])

    @staticmethod
    def _noise(p, rng, n):
        m = max(1, int(p["frames_per_point"]))
        sxy = np.sqrt(p["cam_sigma_nm"] ** 2 / m + p["stage_rep_nm"] ** 2) * 1e-6
        return rng.normal(0, sxy, (n, 2)), sxy * 1e6

    # ---------- 全周扫描采集 (中心/角度精度/增益 共享) ----------
    @staticmethod
    def sweep(p, rng):
        K = max(3, int(p["num_angles"]))
        arc = p["sweep_arc_rad"]
        th_cmd = np.linspace(0.0, arc, K)             # 站0=零位
        th_act = RzSuiteModel.angle_truth(p, th_cmd)
        P0 = RzSuiteModel.marks(p)
        clouds, sxy = [], 0.0
        for k in range(K):
            ck = RzSuiteModel._center_at(p, th_cmd[k], rng)
            Pk = (rot2d(th_act[k]) @ (P0 - ck).T).T + ck
            e, sxy = RzSuiteModel._noise(p, rng, len(P0))
            clouds.append(Pk + e)
        return {"th_cmd": th_cmd, "th_act": th_act, "clouds": clouds,
                "P0": P0, "sxy_nm": sxy}

    # ---------- 1) 旋转中心 (方差加权多配对) ----------
    @staticmethod
    def calib_center(p, sw):
        th_cmd, clouds = sw["th_cmd"], sw["clouds"]
        cands, wts, rmses, pairwise = [], [], [], []
        gate = min(np.deg2rad(2),
                   0.5 * abs(th_cmd[-1] - th_cmd[0]))
        for k in range(1, len(clouds)):
            dth = th_cmd[k] - th_cmd[0]
            sh = abs(np.sin(dth / 2))
            if sh < abs(np.sin(gate / 2)):
                continue
            C_h, th_h = find_rotation_center(clouds[0], clouds[k])
            cands.append(C_h); wts.append(sh ** 2)
            pairwise.append((th_cmd[k], C_h))
            R_ = rot2d(th_h)
            fit = (R_ @ (clouds[0] - C_h).T).T + C_h
            rmses.append(np.sqrt(np.mean(np.sum((fit - clouds[k]) ** 2, axis=1))))
        C_true = RzSuiteModel.true_center(p)
        if cands:
            w = np.array(wts); w /= w.sum()
            C_hat = (np.array(cands) * w[:, None]).sum(axis=0)
        else:
            C_hat = np.full(2, np.nan)
        err = (C_hat - C_true) * 1e6
        pw_C = np.array([c for _, c in pairwise]) if pairwise else np.zeros((0, 2))
        return {"C_true": C_true, "C_hat": C_hat, "err_nm": err,
                "err_radial_nm": float(np.linalg.norm(err)),
                "pw_ang": np.array([a for a, _ in pairwise]), "pw_C": pw_C,
                "rmse_nm": float(np.mean(rmses) * 1e6) if rmses else 0.0,
                "scatter_nm": pw_C.std(axis=0) * 1e6 if len(pw_C) > 2
                else np.zeros(2)}

    # ---------- 2) 角度精度: 各角位 SVD 实测角 vs 指令角 ----------
    @staticmethod
    def measure_angles(p, sw):
        th_cmd, clouds = sw["th_cmd"], sw["clouds"]
        raw = np.zeros(len(clouds), dtype=float)
        for k in range(1, len(clouds)):
            _, raw[k] = find_rotation_center(clouds[0], clouds[k])

        # atan2 只返回 [-pi, pi]. 直接 np.unwrap 在角位稀疏或跨整周端点时
        # 可能选错 2pi 分支。利用指令角作为先验，为每个点选择最近的等价角。
        th_rel_cmd = th_cmd - th_cmd[0]
        th_meas = raw.copy()
        for k in range(1, len(raw)):
            th_meas[k] += 2 * np.pi * np.round(
                (th_rel_cmd[k] - th_meas[k]) / (2 * np.pi))
        return th_cmd, th_meas

    @staticmethod
    def _gain_design(th_cmd, order):
        """相对零位的物理设计矩阵。

        SVD测到的是 theta(k)-theta(0)，所以谐波项也必须减去零位值；
        不能再放一个独立常数项，否则会引入多余自由度并加重病态。
        """
        t0 = float(th_cmd[0])
        rel = th_cmd - t0
        cols = [rel]
        if order >= 1:
            cols += [np.sin(th_cmd) - np.sin(t0),
                     np.cos(th_cmd) - np.cos(t0)]
        if order >= 2:
            cols += [np.sin(2 * th_cmd) - np.sin(2 * t0),
                     np.cos(2 * th_cmd) - np.cos(2 * t0)]
        return np.column_stack(cols)

    # ---------- 4) 角度增益: 可观测性自适应拟合 ----------
    @staticmethod
    def calib_gain(p, sw):
        th_cmd, th_meas = RzSuiteModel.measure_angles(p, sw)
        th_rel = th_cmd - th_cmd[0]

        # 纯线性口径：相对零位模型必须过原点。
        den = float(th_rel @ th_rel)
        slope_lin = float((th_rel @ th_meas) / den) if den > 0 else 1.0
        pred_lin = slope_lin * th_rel
        res_lin = th_meas - pred_lin

        # 候选模型：一阶谐波、二阶谐波。小弧段时 theta/sin/cos
        # 高度相关，盲目解满模型会把亚微弧度噪声放大为 10^4~10^7 ppm。
        candidates = {}
        for order in (1, 2):
            A = RzSuiteModel._gain_design(th_cmd, order)
            coef, *_ = np.linalg.lstsq(A, th_meas, rcond=None)
            rank = int(np.linalg.matrix_rank(A))
            cond = float(np.linalg.cond(A))
            candidates[order] = (A, coef, rank, cond)

        arc = float(np.ptp(th_cmd))
        n = len(th_cmd)
        A1, c1, rank1, cond1 = candidates[1]
        A2m, c2, rank2, cond2 = candidates[2]

        # 只在信息充分时使用全二阶联合拟合；否则自动降阶。
        # 阈值与诊断建议一致：完整增益/谐波分离至少约150度且角位充足。
        if n >= 8 and arc >= np.deg2rad(150) and rank2 == 5 and cond2 <= 1e3:
            order, A, coef, cond = 2, A2m, c2, cond2
            mode = "联合拟合(H1+H2)"
            warning = ""
        elif n >= 6 and arc >= np.deg2rad(45) and rank1 == 3 and cond1 <= 1e3:
            order, A, coef, cond = 1, A1, c1, cond1
            mode = "降阶拟合(H1)"
            warning = ("扫描弧段/角位不足以稳定分离二阶谐波，已自动降为"
                       "增益+一阶谐波模型")
        else:
            order = 0
            A = th_rel[:, None]
            coef = np.array([slope_lin])
            cond = 1.0
            mode = "线性回退"
            warning = ("扫描弧段过小或角位过少，谐波与增益不可辨识；"
                       "已禁止病态联合解，建议弧段>=150度、K>=8")

        pred = A @ coef
        res_joint = th_meas - pred
        h1s = float(coef[1]) if order >= 1 else 0.0
        h1c = float(coef[2]) if order >= 1 else 0.0
        h2s = float(coef[3]) if order >= 2 else 0.0
        h2c = float(coef[4]) if order >= 2 else 0.0

        # 保留原始满模型值用于诊断，但不再把病态结果作为主输出。
        raw_joint_ppm = (float(c2[0]) - 1) * 1e6
        return {"th_cmd": th_cmd, "th_meas": th_meas, "th_rel": th_rel,
                "gain_lin_ppm": (slope_lin - 1) * 1e6,
                "gain_joint_ppm": (float(coef[0]) - 1) * 1e6,
                "gain_joint_raw_ppm": raw_joint_ppm,
                "A1_urad": float(np.hypot(h1s, h1c)) * 1e6,
                "phase1_rad": float(np.arctan2(h1c, h1s)) if order >= 1 else 0.0,
                "A2_urad": float(np.hypot(h2s, h2c)) * 1e6,
                "phase2_rad": float(np.arctan2(h2c, h2s)) if order >= 2 else 0.0,
                "res_lin_urad": res_lin * 1e6, "res_joint_urad": res_joint * 1e6,
                "rms_lin_urad": float(np.sqrt(np.mean(res_lin ** 2))) * 1e6,
                "rms_joint_urad": float(np.sqrt(np.mean(res_joint ** 2))) * 1e6,
                "fit_order": order, "fit_mode": mode,
                "fit_condition": cond, "full_fit_condition": cond2,
                "joint_valid": order == 2, "fit_warning": warning}

    # ---------- 3) 角度重复性 ----------
    @staticmethod
    def repeatability(p, rng):
        N = max(10, int(p["rep_cycles"]))
        m = max(1, int(p["frames_per_point"]))
        M = max(3, int(p["marks_grid"]) ** 2)
        r = p["marks_span_mm"] / 2.0
        sxy = np.sqrt(p["cam_sigma_nm"] ** 2 / m + p["stage_rep_nm"] ** 2) * 1e-6
        s = p["marks_span_mm"] / 2.0
        ax_ = np.linspace(-s, s, max(2, int(p["marks_grid"])))
        Sxx = float(M / max(2, int(p["marks_grid"]))
                    * np.sum((ax_ - ax_.mean()) ** 2))
        s_meas = sxy * np.sqrt(2.0) / np.sqrt(2 * Sxx)             # rad, SVD角噪声
        s_rz = p["rz_rep_urad"] * 1e-6
        step = (p["thermal_rw_urad_sqrt_min"] * 1e-6
                * np.sqrt(max(p["rep_cycle_time_s"], 1e-9) / 60.0))
        drift = np.cumsum(rng.normal(0, step, N)) if step > 0 else np.zeros(N)
        drift -= drift.mean()
        th_err = rng.normal(0, s_rz, N) + rng.normal(0, s_meas, N) + drift
        s3 = 3 * float(np.std(th_err, ddof=1)) * 1e6              # urad
        tan_nm = th_err * 1e6 * r                                 # 切向 nm
        return {"th_err_urad": th_err * 1e6, "s3_urad": s3,
                "tan_nm": tan_nm, "s3_tan_nm": 3 * float(np.std(tan_nm, ddof=1)),
                "sigma_meas_urad": s_meas * 1e6, "radius_mm": r}

    # ---------- 完整流程 ----------
    @staticmethod
    def run(p, seed=None):
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))

        # 原代码虽然提供 repeats 参数，实际只采了一套扫描，导致仿真噪声
        # 与 analytical() 中的 1/sqrt(R) 不一致。这里按角位平均 R 套独立扫描。
        Rn = max(1, int(p.get("repeats", 1)))
        sweeps = [RzSuiteModel.sweep(p, rng) for _ in range(Rn)]
        sw = dict(sweeps[0])
        if Rn > 1:
            sw["clouds"] = [np.mean([x["clouds"][k] for x in sweeps], axis=0)
                            for k in range(len(sweeps[0]["clouds"]))]
            sw["sxy_nm"] = sweeps[0]["sxy_nm"] / np.sqrt(Rn)
        sw["repeats_used"] = Rn

        return {"params": dict(p), "sweep": sw,
                "center": RzSuiteModel.calib_center(p, sw),
                "gain": RzSuiteModel.calib_gain(p, sw),
                "rep": RzSuiteModel.repeatability(p, rng)}

    # ---------- 解析 ----------
    @staticmethod
    def analytical(p):
        G = max(2, int(p["marks_grid"]))
        M = G * G
        m = max(1, int(p["frames_per_point"]))
        R = max(1, int(p["repeats"]))
        K = max(3, int(p["num_angles"]))
        sxy = np.sqrt(p["cam_sigma_nm"] ** 2 / m + p["stage_rep_nm"] ** 2)
        arc = max(p["sweep_arc_rad"], 1e-9)
        s = p["marks_span_mm"] / 2.0
        ax_ = np.linspace(-s, s, G)
        Sxx = float(M / G * np.sum((ax_ - ax_.mean()) ** 2))
        # 中心: 全周端点等价于0度，不能用 arc 本身计算放大因子。
        # 使用实际采样角中最大的 |sin(dtheta/2)|；多配对按逆方差合成。
        th_grid = np.linspace(0.0, arc, K)
        sh = np.abs(np.sin((th_grid[1:] - th_grid[0]) / 2.0))
        sh = sh[sh >= abs(np.sin(min(np.deg2rad(2), 0.5 * arc) / 2.0))]
        sh_max = float(np.max(sh)) if len(sh) else 1e-12
        amp = 1.0 / (2.0 * sh_max)
        base_c = sxy * np.sqrt(2.0 / M) / 2.0
        sig_c_pair = base_c / sh_max
        sig_c = base_c / np.sqrt(max(float(np.sum(sh ** 2)), 1e-24)) / np.sqrt(R)
        # 角度: 单配对 SVD 角噪声 √2σ/√(2Sxx); 全周拟合增益
        sig_ang_pair = sxy * 1e-6 * np.sqrt(2.0) / np.sqrt(2 * Sxx)   # rad
        Sqq = float(np.sum((np.linspace(0, arc, K)
                            - np.mean(np.linspace(0, arc, K))) ** 2))
        sig_gain = sig_ang_pair / np.sqrt(Sqq) / np.sqrt(R)          # rad/rad
        # 谐波污染偏置 (纯线性增益)
        a = arc / 2.0
        k_a = 3 * (np.sin(a) - a * np.cos(a)) / a ** 3 if a > 1e-6 else 1.0
        th0 = arc / 2.0
        bias_g = p["harm1_urad"] * np.cos(th0 + p["harm1_phase_rad"]) * k_a
        sig_A1 = sig_ang_pair * np.sqrt(2.0 / K) * 1e6              # urad
        # 重复性
        s_meas_u = sig_ang_pair * 1e6
        n = max(10, int(p["rep_cycles"]))
        step_u = p["thermal_rw_urad_sqrt_min"] * np.sqrt(
            max(p["rep_cycle_time_s"], 1e-9) / 60.0)
        s_drift = step_u * np.sqrt((n + 1) / 6.0)
        s_rep = np.sqrt(p["rz_rep_urad"] ** 2 + s_meas_u ** 2 + s_drift ** 2)
        return {"sigma_xy_nm": sxy, "M": M, "amp_factor": amp,
                "sig_center_pair_nm": sig_c_pair, "sig_center_nm": sig_c,
                "three_sigma_center_nm": 3 * sig_c,
                "sig_angle_pair_urad": sig_ang_pair * 1e6,
                "sig_gain_ppm": sig_gain * 1e6, "bias_gain_lin_ppm": bias_g,
                "k_arc": k_a, "sig_A1_urad": sig_A1,
                "three_sigma_rep_urad": 3 * s_rep,
                "three_sigma_tan_nm": 3 * s_rep * 1e-6 * (p["marks_span_mm"] / 2.0)
                * 1e6, "arc_rad": arc, "Sxx_mm2": Sxx}

    # ---------- 蒙特卡洛 ----------
    @staticmethod
    def monte_carlo(p, runs=200, seed0=60000):
        ce = np.empty(runs); gl = np.empty(runs); gj = np.empty(runs)
        a1 = np.empty(runs); rj = np.empty(runs); ra = np.empty(runs)
        r3 = np.empty(runs); t3 = np.empty(runs); rm = np.empty(runs)
        for i in range(runs):
            r = RzSuiteModel.run(p, seed=seed0 + i)
            ce[i] = r["center"]["err_radial_nm"]; rm[i] = r["center"]["rmse_nm"]
            g = r["gain"]
            gl[i] = g["gain_lin_ppm"]; gj[i] = g["gain_joint_ppm"]
            a1[i] = g["A1_urad"]; rj[i] = g["rms_joint_urad"]
            ra[i] = g["rms_lin_urad"]
            r3[i] = r["rep"]["s3_urad"]; t3[i] = r["rep"]["s3_tan_nm"]
        def st(v):
            return {"mean": float(np.mean(v)), "std": float(np.std(v, ddof=1)),
                    "p2_5": float(np.percentile(v, 2.5)),
                    "p97_5": float(np.percentile(v, 97.5)), "arr": v}
        return {"runs": runs, "center_err": st(ce), "center_rmse": st(rm),
                "gain_lin": st(gl), "gain_joint": st(gj), "A1": st(a1),
                "rms_joint": st(rj), "rms_lin": st(ra),
                "rep3s": st(r3), "tan3s": st(t3)}

    # ---------- 诊断 ----------
    @staticmethod
    def diagnose(p, R, ana, mc):
        f = []
        amp = ana["amp_factor"]
        f.append(("info", "① 旋转中心: 小转角放大 (本项核心矛盾)",
                  f"中心误差 ≈ σxy·√(2/M)/(2sin(θ/2)); 弧段 "
                  f"{np.rad2deg(ana['arc_rad']):.0f}° → 放大 {amp:.1f}×, "
                  f"单对中心噪声 {ana['sig_center_pair_nm']:.0f} nm",
                  "中心标定必须用大角 (与增益标定相反). 增大弧段比增标记数更有效"))
        if p["sweep_arc_rad"] < np.deg2rad(30):
            f.append(("warn", f"扫描弧段过小 ({np.rad2deg(p['sweep_arc_rad']):.0f}°)",
                      f"中心放大 {amp:.0f}×, 估计被噪声主导",
                      "建议弧段≥90° (≈1.57 rad)"))
        # 增益拟合可观测性
        gg = R["gain"]
        if not gg.get("joint_valid", True):
            f.append(("warn", f"④ 已启用{gg.get('fit_mode', '降阶拟合')}",
                      f"满模型条件数={gg.get('full_fit_condition', float('nan')):.2e}; "
                      f"原始病态解={gg.get('gain_joint_raw_ppm', float('nan')):+.1f} ppm, "
                      f"主输出={gg['gain_joint_ppm']:+.1f} ppm",
                      gg.get("fit_warning", "增大扫描弧段和角位数")))
        # 增益谐波污染
        db = mc["gain_lin"]["mean"] - mc["gain_joint"]["mean"]
        if abs(db) > 2 * ana["sig_gain_ppm"]:
            f.append(("warn", f"④ 纯线性增益被谐波污染 (偏置≈{db:+.1f} ppm)",
                      f"sinθ与θ不正交 (k(a)={ana['k_arc']:.3f}, 整周仍≈0.304); "
                      f"解析偏置 {ana['bias_gain_lin_ppm']:+.1f} ppm",
                      "增益必须与谐波联合拟合, 纯线性口径不作验收量"))
        else:
            f.append(("ok", "④ 增益两口径一致",
                      f"线性 {mc['gain_lin']['mean']:+.1f} vs 联合 "
                      f"{mc['gain_joint']['mean']:+.1f} ppm",
                      "当前谐波/弧段组合下污染不显著, 仍建议联合拟合为标准流程"))
        if mc["A1"]["mean"] > 3 * ana["sig_A1_urad"]:
            f.append(("info", f"④ 一阶谐波显著 (A1≈{mc['A1']['mean']:.1f} urad)",
                      f"典型来源码盘安装偏心 e≈A1×码盘半径 "
                      f"(如{mc['A1']['mean']:.0f}urad×40mm≈{mc['A1']['mean']*0.04:.1f}um)",
                      "联合拟合补偿后残差从 "
                      f"{mc['rms_lin']['mean']:.2f}→{mc['rms_joint']['mean']:.2f} urad, "
                      "输出角度LUT"))
        # 谐波可观测性 (弧段跨度)
        if p["sweep_arc_rad"] < np.deg2rad(150):
            f.append(("warn", "④ 谐波/增益可观测性不足 (弧段偏小)",
                      f"弧段 {np.rad2deg(p['sweep_arc_rad']):.0f}° < 150° 时 "
                      f"sinθ/cosθ 在拟合区间内变化不足, 联合拟合的增益与谐波"
                      f"强相关, MC区间显著展宽",
                      "增益/谐波标定建议弧段≥180°, 最佳全周(2π); "
                      "本套件共享一次扫描, 取全周可同时满足中心与谐波两项"))
        # 重复性
        f.append(("info", "③ 角度重复性 → 切向位移",
                  f"角度3σ={mc['rep3s']['mean']:.2f} urad 在 r={p['marks_span_mm']/2:.0f} mm "
                  f"处等效切向 {mc['tan3s']['mean']:.0f} nm",
                  "该值进入键合对位误差链; 缩小工作半径或提高编码器分辨率可降切向分量"))
        if mc["rep3s"]["mean"] > 1.3 * ana["three_sigma_rep_urad"]:
            f.append(("warn", "③ 重复性劣于解析RSS",
                      f"MC {mc['rep3s']['mean']:.2f} vs 解析 "
                      f"{ana['three_sigma_rep_urad']:.2f} urad",
                      "检查热漂移时间基与循环周期设置"))
        # RMSE质量
        rc = R["center"]
        if rc["rmse_nm"] > 5 * ana["sigma_xy_nm"]:
            f.append(("warn", f"① 配准RMSE偏高 ({rc['rmse_nm']:.0f} nm)",
                      "远超测量噪声 → 非刚体/标记误识别/夹持滑移",
                      "核对标记识别与真空吸附 (脚本原始质量判据)"))
        # 同步跳动
        if p["runout_sync_nm"] > 2 * ana["sig_center_pair_nm"]:
            sc = rc["scatter_nm"]
            f.append(("info", "① 同步径向跳动呈圆轨迹",
                      f"逐配对中心散布≈({sc[0]:.0f},{sc[1]:.0f}) nm vs 噪声极限 "
                      f"{ana['sig_center_pair_nm']:.0f} nm",
                      "同步分量可按角度索引LUT补偿; NRRO为不可补偿底噪"))
        return f


# ===================================================================
#  GUI
# ===================================================================

class RzSuiteApp(BaseSimApp):
    title = "D2W - Rz旋转测校套件仿真 (中心/角度精度/重复性/增益)"
    subtitle = ("Rz套件: 旋转中心 · 指令vs实测角 · 角度重复性 · "
                "角度增益(线性+非线性谐波)  |  角度单位: 弧度")
    report_name = "rz_suite"
    model = RzSuiteModel

    def schema(self):
        amp_fn = lambda p: f"{1/(2*np.sin(max(p['sweep_arc_rad'],1e-3)/2)):.0f}"
        sxy_fn = lambda p: f"{np.sqrt(p['cam_sigma_nm']**2/max(1,int(p['frames_per_point']))+p['stage_rep_nm']**2):.0f}"
        deg_fn = lambda p: f"{np.rad2deg(p['sweep_arc_rad']):.1f}"
        return [
            ("section", "采集方案 (中心/角度精度/增益 共享一次扫描)"),
            ("entry", "marks_grid", "标记网格 G×G", 3, "G"),
            ("entry", "marks_span_mm", "标记展布 (边长)", 40.0, "mm"),
            ("entry", "marks_offset_x_mm", "标记区相对中心偏置 X", 0.0, "mm"),
            ("entry", "marks_offset_y_mm", "标记区相对中心偏置 Y", 0.0, "mm"),
            ("entry", "num_angles", "扫描角位数 K (含零位)", 12, "个"),
            ("entry", "sweep_arc_rad", "扫描弧段", 6.2832, "rad"),
            ("derived", "_deg", "扫描弧段 (换算)", "deg", deg_fn, ["sweep_arc_rad"]),
            ("derived", "_amp", "中心误差放大因子 (自动)", "×", amp_fn,
             ["sweep_arc_rad"]),
            ("entry", "frames_per_point", "每点采集帧数 m", 3, "帧"),
            ("entry", "repeats", "整套流程重复次数 R", 3, "次"),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 200, "次"),
            ("note", "推荐全周 2π≈6.2832 rad: 中心标定要大配对角(∝1/sin(θ/2)), 增益/谐波分离要大角度跨度, 全周同时满足. 部分弧段下谐波可观测性骤降 "
                     "(诊断会提示). π/2≈1.5708, π≈3.1416, 2π≈6.2832"),

            ("section", "真实旋转中心 (已知真值)"),
            ("entry", "center_x_mm", "中心 X", 5.0, "mm"),
            ("entry", "center_y_mm", "中心 Y", 3.0, "mm"),

            ("section", "真值: 角度增益与谐波 (待标定)"),
            ("entry", "gain_err_ppm", "角度增益误差 (一阶标度)", 200.0, "ppm"),
            ("entry", "harm1_urad", "一阶谐波 A1 (码盘偏心)", 50.0, "urad"),
            ("entry", "harm1_phase_rad", "一阶谐波相位 φ1", 0.5, "rad"),
            ("entry", "harm2_urad", "二阶谐波 A2 (安装/椭圆)", 5.0, "urad"),
            ("entry", "harm2_phase_rad", "二阶谐波相位 φ2", 0.0, "rad"),
            ("note", "非线性来源: 码盘安装偏心→一阶谐波 (e≈A1×码盘半径), "
                     "栅距不均/椭圆→二阶. 纯线性拟合的增益会被一阶谐波污染 "
                     "(sinθ与θ不正交), 必须联合拟合"),

            ("section", "角度重复性"),
            ("entry", "rep_cycles", "重复性循环数 N", 30, "次"),
            ("entry", "rep_cycle_time_s", "单循环周期", 5.0, "s"),
            ("entry", "rz_rep_urad", "编码器+伺服重复性 1σ", 0.5, "urad"),
            ("entry", "thermal_rw_urad_sqrt_min", "热漂移扩散系数", 0.3, "urad/√min"),

            ("section", "轴系与测量噪声"),
            ("entry", "cam_sigma_nm", "相机单帧对准 1σ", 8.0, "nm"),
            ("entry", "stage_rep_nm", "台到位重复性 1σ", 25.0, "nm"),
            ("derived", "_sxy", "XY点噪声 σxy (自动)", "nm", sxy_fn,
             ["cam_sigma_nm", "stage_rep_nm", "frames_per_point"]),
            ("entry", "runout_sync_nm", "同步径向跳动 (一阶)", 200.0, "nm"),
            ("entry", "runout_phase_rad", "同步跳动相位", 0.0, "rad"),
            ("entry", "nrro_nm", "非同步跳动 NRRO 1σ", 30.0, "nm"),
            ("note", "同步跳动使逐配对中心呈圆轨迹 (可索引补偿), NRRO为底噪. "
                     "验收区间见[验收参考]页"),
        ]

    # ---------------- run ----------------
    def _run(self):
        from tkinter import messagebox
        try:
            p = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        R = RzSuiteModel.run(p)
        ana = RzSuiteModel.analytical(p)
        mc = RzSuiteModel.monte_carlo(p, runs=max(50, int(p.get("mc_runs") or 200)))
        diag = RzSuiteModel.diagnose(p, R, ana, mc)
        self.R = {"result": R, "ana": ana, "mc": mc, "diag": diag}
        self._status_calib()
        self._render_all()

    def _status_calib(self):
        R, mc = self.R["result"], self.R["mc"]
        rc, g = R["center"], R["gain"]
        ok = (rc["err_radial_nm"] <= mc["center_err"]["p97_5"] * 1.0001
              and mc["gain_joint"]["p2_5"] <= g["gain_joint_ppm"]
              <= mc["gain_joint"]["p97_5"])
        bg = self.C["pass_bg"] if ok else self.C["ref_bg"]
        fg = self.C["pass_fg"] if ok else self.C["warn"]
        tag = "正常·位于验收区间" if ok else "超出验收区间 → 见诊断页"
        self.sfr.config(bg=bg)
        self.sl.config(bg=bg, fg=fg, font=(FONT, 11, "bold"), text=(
            f"[{tag}]  中心残差={rc['err_radial_nm']:.0f} nm  "
            f"增益({g.get('fit_mode', '联合')})={g['gain_joint_ppm']:+.0f} ppm  A1={g['A1_urad']:.1f} urad  "
            f"重复3σ={R['rep']['s3_urad']:.2f} urad  | 运行#{self.sim_n}"))

    # ---------------- tabs ----------------
    def standard_tabs(self):
        return [("旋转中心", self._tab_center),
                ("角度精度", self._tab_angle),
                ("角度增益/谐波", self._tab_gain),
                ("角度重复性", self._tab_rep),
                ("诊断与建议", self._tab_diag_c),
                ("验收参考", self._tab_accept_c)]

    def _tab_center(self, tab, RR):
        R = RR["result"]; rc = R["center"]; sw = R["sweep"]; p = R["params"]
        ana = RR["ana"]; mc = RR["mc"]
        fig = Figure(figsize=(10.8, 5.2), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, "标记云各角位叠加 + 中心解算", "X (mm)", "Y (mm)")
        K = len(sw["clouds"])
        for k, cl in enumerate(sw["clouds"]):
            ax.scatter(cl[:, 0], cl[:, 1], s=18,
                       color=cm.viridis(k / max(K - 1, 1)), alpha=0.8)
        ax.scatter(*rc["C_true"], marker="+", s=150, color=self.C["fail_fg"],
                   zorder=6, label="真实中心")
        ax.scatter(*rc["C_hat"], marker="x", s=120, color=self.C["accent"],
                   zorder=6, label=f"解算 (残差 {rc['err_radial_nm']:.0f} nm)")
        ax.legend(fontsize=8); ax.set_aspect("equal")
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "逐配对中心 (相对真值) — 同步跳动呈圆轨迹",
                 "ΔX (nm)", "ΔY (nm)")
        if len(rc["pw_C"]):
            d = (rc["pw_C"] - rc["C_true"]) * 1e6
            ang = np.rad2deg(rc["pw_ang"])
            o = np.argsort(rc["pw_ang"])
            sc = ax2.scatter(d[o, 0], d[o, 1], c=ang[o], cmap="twilight", s=60,
                             edgecolor="white", linewidth=0.5, zorder=4)
            fig.colorbar(sc, ax=ax2, label="配对角位 (deg)", shrink=0.85)
            ax2.plot(d[o, 0], d[o, 1], color=self.C["dim"], lw=0.6, alpha=0.5)
        ax2.scatter(*(rc["err_nm"]), marker="x", s=110, color=self.C["accent"],
                    zorder=5, label="加权解")
        ax2.scatter(0, 0, marker="+", s=140, color=self.C["fail_fg"], zorder=5,
                    label="真值")
        ax2.legend(fontsize=8); ax2.set_aspect("equal")
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"① 旋转中心: 真值 ({p['center_x_mm']:.3f}, {p['center_y_mm']:.3f}) → "
            f"解算 ({rc['C_hat'][0]:.4f}, {rc['C_hat'][1]:.4f}) mm, 残差 "
            f"{rc['err_radial_nm']:.0f} nm (≤{mc['center_err']['p97_5']:.0f})   |   "
            f"放大因子 {ana['amp_factor']:.1f}× (弧段 "
            f"{np.rad2deg(ana['arc_rad']):.0f}°)   逐配对散布 "
            f"({rc['scatter_nm'][0]:.0f},{rc['scatter_nm'][1]:.0f}) nm   "
            f"RMSE={rc['rmse_nm']:.0f} nm"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_angle(self, tab, RR):
        R = RR["result"]; g = R["gain"]; ana = RR["ana"]; p = R["params"]
        fig = Figure(figsize=(10.8, 5.2), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, "指令角 vs 实测角 (SVD算法解算)", "指令角 (rad)",
                 "实测角 (rad)")
        ax.plot(g["th_rel"], g["th_meas"], "o", color=self.C["cx"], ms=7,
                label="实测 (SVD)", zorder=3)
        lim = [g["th_rel"].min(), g["th_rel"].max()]
        ax.plot(lim, lim, color=self.C["dim"], lw=1.0, ls=":",
                label="理想 y=x", zorder=2)
        ax.legend(fontsize=8)
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "角度误差: 实测 - 指令 (含增益+谐波)", "指令角 (rad)",
                 "误差 (urad)")
        dev = (g["th_meas"] - g["th_rel"]) * 1e6
        ax2.plot(g["th_rel"], dev, "o-", color=self.C["cx"], ms=6, lw=0.9,
                 label="实测−指令")
        # 拆出线性分量
        ax2.plot(g["th_rel"], g["gain_lin_ppm"] * 1e-6 * g["th_rel"] * 1e6
                 + (dev - g["res_lin_urad"] - g["gain_lin_ppm"] * 1e-6
                    * g["th_rel"] * 1e6).mean(), color=self.C["fail_fg"],
                 lw=1.3, ls="--", label=f"线性增益 {g['gain_lin_ppm']:+.0f} ppm")
        ax2.axhline(0, color=self.C["dim"], lw=0.6)
        ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        rms_acc = float(np.sqrt(np.mean(dev ** 2)))
        tk.Label(tab, text=(
            f"② 角度精度: 实测角由 SVD 旋转矩阵 atan2(R₁₀,R₀₀) 解出. "
            f"指令-实测误差 RMS = {rms_acc:.1f} urad (含增益 "
            f"{p['gain_err_ppm']:+.0f} ppm + 谐波)   |   单配对角度噪声 1σ = "
            f"{ana['sig_angle_pair_urad']:.2f} urad   |   "
            f"该误差经[角度增益/谐波]页分解为可补偿(增益/谐波)与随机(噪声)部分"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_gain(self, tab, RR):
        R = RR["result"]; g = R["gain"]; ana = RR["ana"]; mc = RR["mc"]
        p = R["params"]
        fig = Figure(figsize=(10.8, 5.4), dpi=100, facecolor=self.C["bg"])
        # 左上: 线性残差 + 谐波模型 + 补偿后
        ax = fig.add_subplot(121)
        self._ax(ax, "线性拟合残差 → 谐波模型 → 联合补偿后", "指令角 (rad)",
                 "残差 (urad)")
        ax.plot(g["th_rel"], g["res_lin_urad"], "o", color=self.C["cx"], ms=6,
                label=f"线性残差 RMS={g['rms_lin_urad']:.2f}")
        o = np.argsort(g["th_rel"])
        ax.plot(g["th_rel"][o], (g["res_lin_urad"] - g["res_joint_urad"])[o],
                color=self.C["accent2"], lw=1.6,
                label=f"谐波模型 A1={g['A1_urad']:.1f} A2={g['A2_urad']:.1f} urad")
        ax.plot(g["th_rel"], g["res_joint_urad"], "s", color=self.C["pass_fg"],
                ms=4, label=f"联合补偿后 RMS={g['rms_joint_urad']:.2f}")
        ax.axhline(0, color=self.C["dim"], lw=0.6); ax.legend(fontsize=8)
        # 右: 两口径增益 MC分布
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "增益两口径 MC分布 (自动抑制病态拟合)", "增益误差 (ppm)",
                 "频次")
        bins = np.histogram_bin_edges(
            np.concatenate([mc["gain_lin"]["arr"], mc["gain_joint"]["arr"]]),
            bins=30)
        ax2.hist(mc["gain_lin"]["arr"], bins=bins, color=self.C["fail_fg"],
                 alpha=0.5,
                 label=f"纯线性 (偏置{mc['gain_lin']['mean']-p['gain_err_ppm']:+.1f})")
        ax2.hist(mc["gain_joint"]["arr"], bins=bins, color=self.C["pass_fg"],
                 alpha=0.6,
                 label=f"推荐拟合 (偏置{mc['gain_joint']['mean']-p['gain_err_ppm']:+.1f})")
        ax2.axvline(p["gain_err_ppm"], color=self.C["ref"], lw=1.8, ls="-.",
                    label="真值")
        ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"④ 增益({g.get('fit_mode', '联合')}) = {g['gain_joint_ppm']:+.1f} ppm (真值 "
            f"{p['gain_err_ppm']:+.0f})   A1 = {g['A1_urad']:.2f} urad @ "
            f"{g['phase1_rad']:.6f} rad (真值 {p['harm1_urad']:.0f}@"
            f"{p['harm1_phase_rad']:.6f})   A2 = {g['A2_urad']:.2f} (真值 "
            f"{p['harm2_urad']:.0f})  条件数={g.get('fit_condition', float('nan')):.2e}\n纯线性偏置 = A1·cos(θ0+φ1)·k(a) = "
            f"{ana['bias_gain_lin_ppm']:+.1f} ppm (k(a)={ana['k_arc']:.6f}; 整周 "
            f"k=3/π²≈0.304 仍不为零) → 增益必须联合拟合, 纯线性口径不作验收量. "
            f"补偿后残差 {g['rms_joint_urad']:.2f} urad → 角度LUT"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_rep(self, tab, RR):
        R = RR["result"]; rp = R["rep"]; p = R["params"]; mc = RR["mc"]
        ana = RR["ana"]
        fig = Figure(figsize=(10.8, 5.2), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, f"角度误差序列 (3σ={rp['s3_urad']:.2f} urad)", "循环序号",
                 "角度误差 (urad)")
        n = len(rp["th_err_urad"])
        ax.plot(np.arange(1, n + 1), rp["th_err_urad"], "o-", color=self.C["cx"],
                ms=3.5, lw=0.9)
        ax.axhspan(-rp["s3_urad"], rp["s3_urad"], alpha=0.08, color=self.C["cx"])
        for sgn in (1, -1):
            ax.axhline(sgn * rp["s3_urad"], color=self.C["cx"], lw=0.9, ls=":")
        ax.axhline(0, color=self.C["dim"], lw=0.6)
        ax2 = fig.add_subplot(122)
        self._ax(ax2, f"r={rp['radius_mm']:.0f} mm 处等效切向位移 "
                      f"(3σ={rp['s3_tan_nm']:.0f} nm)", "循环序号", "切向 (nm)")
        ax2.plot(np.arange(1, n + 1), rp["tan_nm"], "s-", color=self.C["cy"],
                 ms=3.5, lw=0.9)
        ax2.axhline(rp["s3_tan_nm"] / 3, color=self.C["cy"], lw=1.0, ls="--",
                    label=f"1σ ≈ {rp['s3_tan_nm']/3:.0f} nm")
        ax2.axhline(0, color=self.C["dim"], lw=0.6); ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"③ 重复性分量(1σ): 编码器+伺服 {p['rz_rep_urad']:.2f} ⊕ SVD测量 "
            f"{rp['sigma_meas_urad']:.3f} ⊕ 热漂移(时间基) → 解析RSS 3σ = "
            f"{ana['three_sigma_rep_urad']:.2f} urad (MC {mc['rep3s']['mean']:.2f})"
            f"   |   等效切向 3σ = {rp['s3_tan_nm']:.0f} nm @ r="
            f"{rp['radius_mm']:.0f} mm"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_diag_c(self, tab, RR):
        from tkinter import ttk
        wrap = tk.Frame(tab, bg=self.C["bg"]); wrap.pack(fill="both", expand=True)
        txt = tk.Text(wrap, bg=self.C["panel"], fg=self.C["text"], font=(FONT, 10),
                      relief="flat", wrap="word", padx=14, pady=12, spacing3=4)
        sb = ttk.Scrollbar(wrap, command=txt.yview); txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y"); txt.pack(fill="both", expand=True, padx=8, pady=8)
        for tag, fgk in (("h", "accent"), ("warn", "fail_fg"), ("ok", "pass_fg"),
                         ("info", "accent2"), ("dim", "dim")):
            txt.tag_config(tag, foreground=self.C[fgk],
                           font=(FONT, 11 if tag == "h" else 10,
                                 "normal" if tag == "dim" else "bold"))
        txt.insert("end", "Rz套件诊断 — 四项测校的影响因素\n\n", "h")
        mark = {"warn": "⚠ ", "ok": "✓ ", "info": "ℹ "}
        for sev, t, de, adv in RR["diag"]:
            txt.insert("end", mark.get(sev, "") + t + "\n", sev)
            txt.insert("end", f"   现象: {de}\n", "dim")
            txt.insert("end", f"   建议: {adv}\n\n")
        txt.config(state="disabled")

    def _tab_accept_c(self, tab, RR):
        ana, mc = RR["ana"], RR["mc"]; p = RR["result"]["params"]
        info = tk.Frame(tab, bg=self.C["ref_bg"]); info.pack(fill="x", padx=10,
                                                             pady=(10, 4))
        tk.Label(info, text=(
            f"建议验收区间 (由当前误差模型MC推导, N={mc['runs']}):\n"
            f"   ① 中心径向残差 ≤ {mc['center_err']['p97_5']:.0f} nm (P97.5; "
            f"解析1σ={ana['sig_center_nm']:.0f}), RMSE ≤ "
            f"{mc['center_rmse']['p97_5']:.0f} nm\n"
            f"   ④ 增益(推荐拟合) ∈ [{mc['gain_joint']['p2_5']:.1f}, "
            f"{mc['gain_joint']['p97_5']:.1f}] ppm (解析噪声极限3σ="
            f"{3*ana['sig_gain_ppm']:.1f}); A1 ∈ [{mc['A1']['p2_5']:.1f}, "
            f"{mc['A1']['p97_5']:.1f}] urad; 补偿后残差 ≤ "
            f"{mc['rms_joint']['p97_5']:.2f} urad\n"
            f"   ③ 重复性 3σ ∈ [{mc['rep3s']['p2_5']:.2f}, "
            f"{mc['rep3s']['p97_5']:.2f}] urad (解析{ana['three_sigma_rep_urad']:.2f}); "
            f"等效切向 3σ ≤ {mc['tan3s']['p97_5']:.0f} nm\n"
            f"判读: 实测落于区间→正常; 纯线性增益不作验收口径(谐波污染); "
            f"中心RMSE异常大→非刚体/标记误识别"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(10.8, 5.4), dpi=100, facecolor=self.C["bg"])
        specs = [("center_err", "①中心残差 (nm)", self.C["cx"], None),
                 ("gain_joint", "④增益·推荐 (ppm)", self.C["pass_fg"],
                  p["gain_err_ppm"]),
                 ("A1", "④A1 (urad)", self.C["accent2"], p["harm1_urad"]),
                 ("rms_joint", "④补偿后残差 (urad)", "#0a7ea4", None),
                 ("rep3s", "③重复性3σ (urad)", self.C["cy"], None),
                 ("tan3s", "③等效切向3σ (nm)", "#b45309", None)]
        for i, (k, ttl, col, ref) in enumerate(specs):
            ax = fig.add_subplot(2, 3, i + 1)
            self._ax(ax, ttl, "", "")
            s = mc[k]
            ax.hist(s["arr"], bins=22, color=col, alpha=0.7, edgecolor="white",
                    lw=0.4)
            ax.axvspan(s["p2_5"], s["p97_5"], alpha=0.10, color=self.C["ref"])
            for q in ("p2_5", "p97_5"):
                ax.axvline(s[q], color="#e5a00d", lw=1.0, ls="--")
            if ref is not None:
                ax.axvline(ref, color=self.C["ref"], lw=1.6, ls="-.")
            ax.tick_params(labelsize=7)
        fig.tight_layout(pad=1.8)
        self._embed(fig, tab)

    # ---------------- export ----------------
    def _export(self):
        from tkinter import filedialog, messagebox
        from datetime import datetime
        if not self.R:
            messagebox.showinfo("提示", "请先运行仿真"); return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Text", "*.txt")],
            initialfile=f"{self.report_name}_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        R, ana, mc = self.R["result"], self.R["ana"], self.R["mc"]
        p = R["params"]; rc = R["center"]; g = R["gain"]; rp = R["rep"]
        L = ["=" * 76, f"  {self.title}", "=" * 76,
             f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   "
             f"simcore v{simcore.__version__}   (angles in radians)",
             f"  Layout: {int(p['marks_grid'])}x{int(p['marks_grid'])} marks span "
             f"{p['marks_span_mm']} mm; K={int(p['num_angles'])} angles / arc "
             f"{p['sweep_arc_rad']:.4f} rad ({np.rad2deg(p['sweep_arc_rad']):.1f} deg); "
             f"R={int(p['repeats'])}",
             f"  Noise: sigma_xy={ana['sigma_xy_nm']:.0f} nm; runout sync "
             f"{p['runout_sync_nm']:.0f}/NRRO {p['nrro_nm']:.0f} nm",
             "", "-" * 76, "  1) ROTATION CENTER", "-" * 76,
             f"  true ({rc['C_true'][0]:.4f}, {rc['C_true'][1]:.4f}) -> solved "
             f"({rc['C_hat'][0]:.4f}, {rc['C_hat'][1]:.4f}) mm",
             f"  radial residual = {rc['err_radial_nm']:.1f} nm; amplification "
             f"{ana['amp_factor']:.1f}x; RMSE = {rc['rmse_nm']:.0f} nm; "
             f"scatter ({rc['scatter_nm'][0]:.0f}, {rc['scatter_nm'][1]:.0f}) nm",
             "", "-" * 76, "  2) ANGLE ACCURACY (commanded vs measured)", "-" * 76,
             f"  angle error RMS (meas-cmd) = "
             f"{float(np.sqrt(np.mean((g['th_meas']-g['th_rel'])**2)))*1e6:.1f} urad "
             f"(SVD-measured per angle)",
             "", "-" * 76, "  3) ANGLE REPEATABILITY", "-" * 76,
             f"  3sigma = {rp['s3_urad']:.3f} urad (analytic "
             f"{ana['three_sigma_rep_urad']:.3f}); tangential 3sigma "
             f"{rp['s3_tan_nm']:.0f} nm @ r={rp['radius_mm']:.0f} mm",
             "", "-" * 76, "  4) ANGLE GAIN & HARMONICS", "-" * 76,
             f"  gain linear-only = {g['gain_lin_ppm']:+.2f} ppm [contaminated; "
             f"analytic bias {ana['bias_gain_lin_ppm']:+.1f}, k(a)={ana['k_arc']:.3f}]",
             f"  gain recommended [{g.get('fit_mode', 'joint')}] = {g['gain_joint_ppm']:+.2f} ppm (truth "
             f"{p['gain_err_ppm']:+.0f})",
             f"  A1 = {g['A1_urad']:.2f} urad @ {g['phase1_rad']:.6f} rad (truth "
             f"{p['harm1_urad']:.0f}@{p['harm1_phase_rad']:.6f}); A2 = "
             f"{g['A2_urad']:.2f} (truth {p['harm2_urad']:.0f})",
             f"  residual RMS linear {g['rms_lin_urad']:.2f} -> compensated "
             f"{g['rms_joint_urad']:.2f} urad (angle LUT)",
             "", "-" * 76, f"  ACCEPTANCE (MC N={mc['runs']})", "-" * 76,
             f"  center <= {mc['center_err']['p97_5']:.0f} nm; gain(joint) "
             f"[{mc['gain_joint']['p2_5']:.1f}, {mc['gain_joint']['p97_5']:.1f}] ppm; "
             f"A1 [{mc['A1']['p2_5']:.1f}, {mc['A1']['p97_5']:.1f}] urad",
             f"  compensated RMS <= {mc['rms_joint']['p97_5']:.2f} urad; rep 3sigma "
             f"[{mc['rep3s']['p2_5']:.2f}, {mc['rep3s']['p97_5']:.2f}] urad; tan 3sigma "
             f"<= {mc['tan3s']['p97_5']:.0f} nm",
             f"  NOTE: linear-only gain NOT an acceptance quantity (harmonic "
             f"contamination)",
             "", "-" * 76, "  DIAGNOSIS", "-" * 76]
        for sev, t, de, adv in self.R["diag"]:
            L += [f"  [{sev.upper()}] {t}", f"      现象: {de}",
                  f"      建议: {adv}"]
        L += ["", "-" * 76, "  PARAMETERS", "-" * 76]
        for k, v in p.items():
            L.append(f"  {k}: {v}")
        L += ["", "=" * 76]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(L))
        messagebox.showinfo("OK", f"已保存:\n{path}")

    def _imp_csv(self):
        """实测数据: angle_rad,mark_id,x_mm,y_mm
        各角位标记坐标 (mark_id一致, 行序对应同一标记). 最小角位为零位配对,
        跑 find_rotation_center 求中心 + 各角位实测角 + 增益联合拟合."""
        from tkinter import filedialog, messagebox, ttk
        import csv
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"),
                                                     ("All", "*.*")])
        if not path:
            return
        try:
            recs = []
            with open(path, "r", encoding="utf-8-sig") as fh:
                for row in csv.reader(fh):
                    cells = [c.strip() for c in row if c.strip() != ""]
                    if len(cells) < 4:
                        continue
                    try:
                        recs.append([float(c) for c in cells[:4]])
                    except ValueError:
                        continue
            if len(recs) < 4:
                raise ValueError("数据不足 (每行: angle_rad,mark_id,x_mm,y_mm)")
            recs = np.array(recs)
            angles = np.unique(recs[:, 0])
            mids0 = np.sort(recs[recs[:, 0] == angles[0]][:, 1])
            clouds = {}
            for a in angles:
                sel = recs[recs[:, 0] == a]
                sel = sel[np.argsort(sel[:, 1])]
                if not np.array_equal(np.sort(sel[:, 1]), mids0):
                    raise ValueError(f"角位 {a} 标记ID与零位不一致")
                clouds[a] = sel[:, 2:4]
            P0 = clouds[angles[0]]
            lines = [f"文件: {path}", f"角位数 K={len(angles)}, 标记数 "
                     f"M={len(mids0)} (角度单位 rad)", "=" * 70]
            cands, wts, th_meas = [], [], [0.0]
            for a in angles[1:]:
                dth = a - angles[0]
                C_h, th_h = find_rotation_center(P0, clouds[a])
                th_meas.append(th_h)
                sh = abs(np.sin(dth / 2))
                if sh >= np.sin(np.deg2rad(2)):
                    cands.append(C_h); wts.append(sh ** 2)
            if cands:
                w = np.array(wts); w /= w.sum()
                C = (np.array(cands) * w[:, None]).sum(axis=0)
                sc = np.array(cands).std(axis=0) * 1e6
                lines.append(f"[① 中心] 加权解 Ĉ = ({C[0]:.4f}, {C[1]:.4f}) mm, "
                             f"逐配对散布 ({sc[0]:.0f}, {sc[1]:.0f}) nm")
            # 与仿真路径共用同一套角度分支选择和可观测性自适应拟合，
            # 避免实测CSV在小弧段下再次输出病态的超大增益。
            sw_csv = {"th_cmd": angles,
                      "clouds": [clouds[a] for a in angles]}
            gcsv = RzSuiteModel.calib_gain({}, sw_csv)
            th_meas = gcsv["th_meas"]
            th_rel = gcsv["th_rel"]
            lines.append(f"[② 角度精度] 实测-指令 RMS = "
                         f"{float(np.sqrt(np.mean((th_meas-th_rel)**2)))*1e6:.1f} urad")
            lines += [f"[④ 增益] 纯线性 = {gcsv['gain_lin_ppm']:+.1f} ppm "
                      f"(谐波污染, 参考)",
                      f"         {gcsv['fit_mode']} = {gcsv['gain_joint_ppm']:+.1f} ppm  "
                      f"A1={gcsv['A1_urad']:.1f} urad  A2={gcsv['A2_urad']:.1f} urad  "
                      f"残差RMS={gcsv['rms_joint_urad']:.2f} urad  "
                      f"cond={gcsv['fit_condition']:.2e}"]
            if gcsv["fit_warning"]:
                lines.append("         提示: " + gcsv["fit_warning"])
            if self.R:
                mc = self.R["mc"]
                lines.append(f"[验收对比] 仿真区间: 中心≤"
                             f"{mc['center_err']['p97_5']:.0f}nm, 增益(推荐拟合)["
                             f"{mc['gain_joint']['p2_5']:.0f},"
                             f"{mc['gain_joint']['p97_5']:.0f}]ppm")
            else:
                lines.append("提示: 先运行仿真可获得验收区间对比。")
            win = tk.Toplevel(self.root)
            win.title("实测Rz套件数据分析"); win.geometry("780x440")
            win.configure(bg=self.C["bg"])
            txt = tk.Text(win, bg=self.C["panel"], fg=self.C["text"],
                          font=("Consolas", 10), relief="flat", wrap="word",
                          padx=14, pady=12)
            sb = ttk.Scrollbar(win, command=txt.yview)
            txt.configure(yscrollcommand=sb.set)
            sb.pack(side="right", fill="y")
            txt.pack(fill="both", expand=True, padx=8, pady=8)
            txt.insert("end", "\n".join(lines) + "\n")
            txt.config(state="disabled")
        except Exception as e:
            messagebox.showerror("导入失败",
                f"{e}\n\n格式: angle_rad,mark_id,x_mm,y_mm (表头可选)\n"
                f"各角位标记各占一行; 标记ID一致, 行序对应同一标记")


if __name__ == "__main__":
    launch(RzSuiteApp)
