#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W 相机微旋标定仿真 — 两点法 / 九点法 合并版 (FR10/FR11) · v2
参考文档: SDS §4.2, SSPS §3.1.5/3.1.6, STPS FR10/FR11

v2 改进 (按评审意见):
  1. 弃用ppm: 像素尺度偏差统一以 nm/mm 表示 — "每移动1mm, 测量值偏差多少nm"
     (数值上 1 nm/mm ≡ 1 ppm, 但物理含义直白). 畸变以"视场角部位移(nm)"表示.
  2. 光学链显式建模并与移动距离联动:
       等效像素尺寸 = 像元尺寸 / 放大倍率
       视场 = 传感器分辨率 × 等效像素尺寸
       扫描基线 L = 占比 × 视场宽    (改放大倍率 → 基线/角度噪声自动联动)
       相机噪声 = 亚像素定位精度(px) × 等效像素尺寸
     测量管线按真实链路: 物理位移 →(实际尺度)→ 像素 →(+亚像素噪声)→
     (名义尺度换算)→ 测量值, 尺度偏差即"实际/名义尺度差"自然进入.
  3. 重画两点法/九点法图: 视场内原理示意 (偏差放大标注、角度扇形、
     坐标系旋转示意)、分解结果条形图.

方法:
  两点法: 台带标记沿台X横穿视场, φ = −atan(Δy/Δx); 对台非正交免疫;
          扫描偏离光轴且不对称时畸变偏置 bias ≈ −2·kd·x0·y0/r².
  九点法: 3×3网格仿射映射H + RQ式分解 (X为基准), 附带产出剪切项(≈台非正交)
          与像素尺度偏差(nm/mm).
  解析噪声极限: σφ₂=√2·σ_pt/(L√R);  σφ₉=σ_pt/(g·√6·√R);  σ_pt=√(σc²/m+σs²)
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, FancyArrowPatch, Arc, Circle

import simcore
from simgui import BaseSimApp, launch, FONT


def rq_angle(H):
    """RQ式分解 (X为基准): 求 α 使 Rot(−α)·H 为上三角. 返回 (α, U)."""
    a = float(np.arctan2(H[1, 0], H[0, 0]))
    c, s = np.cos(-a), np.sin(-a)
    return a, np.array([[c, -s], [s, c]]) @ H


def nice_exa(raw):
    """偏差放大倍数取 1/2/5×10^k 的整洁值."""
    if raw <= 1:
        return 1
    e = 10 ** np.floor(np.log10(raw))
    for m in (1, 2, 5, 10):
        if m * e >= raw:
            return int(m * e)
    return int(10 * e)


# ===================================================================
#  模型
# ===================================================================

class MicroRotModel:
    """单位: 视场内位置 um, 噪声 nm, 角度 urad. 光学链: 像元/放大倍率→像素→物理."""

    # ---------- 光学几何 (联动核心) ----------
    @staticmethod
    def optics(p):
        pix = p["sensor_pixel_size"] / p["lens_magnification"]      # um/px
        half_x = p["sensor_w_px"] / 2.0 * pix
        half_y = p["sensor_h_px"] / 2.0 * pix
        r_fov = float(np.hypot(half_x, half_y))                     # 对角半径
        L = 2 * half_x * np.clip(p["span_frac"], 0.05, 0.98)        # 扫描基线
        g = np.clip(p["span_frac"], 0.05, 0.98) * min(half_x, half_y)
        cam_nm = p["subpixel_acc_px"] * pix * 1e3                   # 单帧噪声
        return {"pix_um": pix, "half_x": half_x, "half_y": half_y,
                "r_fov": r_fov, "L": L, "g": g, "cam_nm": cam_nm}

    @staticmethod
    def truth(p):
        phi = p["phi_true_urad"] * 1e-6
        Rc = np.array([[np.cos(-phi), -np.sin(-phi)],
                       [np.sin(-phi), np.cos(-phi)]])               # 台系→相机系
        A = np.array([[1.0, p["stage_nonorth_urad"] * 1e-6],
                      [0.0, 1.0]])                                   # 台指令→台实际
        # 像素尺度偏差 nm/mm ≡ 1e-6 相对偏差
        K = np.diag([1 + p["scale_err_x_nm_mm"] * 1e-6,
                     1 + p["scale_err_y_nm_mm"] * 1e-6])
        return phi, Rc, A, K

    @staticmethod
    def measure_seq(p, u_cmds, rng):
        """逐点测量 (真实光学管线):
        台指令 → 台实际 → 相机系物理 → 畸变 → ×实际尺度→像素 → +亚像素噪声
        → ×名义尺度 → 测量值(um). 含逐走位热漂移随机游走."""
        _, Rc, A, K = MicroRotModel.truth(p)
        opt = MicroRotModel.optics(p)
        m = max(1, int(p["frames_per_point"]))
        s_stage = p["stage_rep_nm"] * 1e-3                           # um
        s_px = p["subpixel_acc_px"] / np.sqrt(m)                     # px (帧平均)
        kd = p["dist_edge_nm"] * 1e-3 / max(opt["r_fov"], 1e-9)      # 无量纲
        step = (p["thermal_rw_nm_sqrt_min"]
                * np.sqrt(max(p["move_time_s"], 1e-9) / 60.0) * 1e-3)
        N = len(u_cmds)
        drift = np.cumsum(rng.normal(0, step, (N, 2)), axis=0) if step > 0 \
            else np.zeros((N, 2))
        u_act = u_cmds @ A.T + rng.normal(0, s_stage, (N, 2)) + drift
        w = u_act @ Rc.T                                             # 相机系物理(um)
        r2 = np.sum(w ** 2, axis=1, keepdims=True)
        w = w * (1 + kd * r2 / opt["r_fov"] ** 2)                    # 径向畸变
        w_px = (w @ K.T) / opt["pix_um"]                             # 实际尺度→像素
        w_px = w_px + rng.normal(0, s_px, w_px.shape)                # 亚像素噪声
        return w_px * opt["pix_um"]                                  # 名义尺度换算

    # ---------- 采集轨迹 ----------
    @staticmethod
    def trajectories(p):
        opt = MicroRotModel.optics(p)
        P = max(2, int(p["line_points"]))
        x0, y0 = p["scan_x_center_um"], p["scan_y_offset_um"]
        line = np.column_stack([np.linspace(x0 - opt["L"] / 2,
                                            x0 + opt["L"] / 2, P),
                                np.full(P, y0)])
        g = opt["g"]
        gx, gy = np.meshgrid([-g, 0, g], [-g, 0, g], indexing="ij")
        return line, np.column_stack([gx.ravel(), gy.ravel()]), opt

    # ---------- 两点/线法 ----------
    @staticmethod
    def calib_2pt(u_cmds, w_meas):
        if len(u_cmds) == 2:
            d = w_meas[1] - w_meas[0]
            return -float(np.arctan2(d[1], d[0])) * 1e6, d
        s = u_cmds[:, 0] - u_cmds[:, 0].mean()
        kx = float(np.polyfit(s, w_meas[:, 0], 1)[0])
        ky = float(np.polyfit(s, w_meas[:, 1], 1)[0])
        return -float(np.arctan2(ky, kx)) * 1e6, w_meas[-1] - w_meas[0]

    # ---------- 九点法 ----------
    @staticmethod
    def calib_9pt(u_cmds, w_meas):
        Adsg = np.column_stack([u_cmds, np.ones(len(u_cmds))])
        cx, *_ = np.linalg.lstsq(Adsg, w_meas[:, 0], rcond=None)
        cy, *_ = np.linalg.lstsq(Adsg, w_meas[:, 1], rcond=None)
        H = np.array([[cx[0], cx[1]], [cy[0], cy[1]]])
        alpha, U = rq_angle(H)
        resid = (w_meas - Adsg @ np.column_stack([cx, cy])) * 1e3   # nm
        return {"phi_urad": -alpha * 1e6, "H": H,
                "scale_x_nm_mm": (U[0, 0] - 1) * 1e6,
                "scale_y_nm_mm": (U[1, 1] - 1) * 1e6,
                "shear_urad": float(np.arctan2(U[0, 1], U[1, 1])) * 1e6,
                "resid_nm": resid}

    # ---------- 单次完整流程 (各法重复R次取均值) ----------
    @staticmethod
    def run(p, seed=None):
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))
        line, grid9, opt = MicroRotModel.trajectories(p)
        R = max(1, int(p["repeats"]))
        phi2s, phi9s, nines = [], [], []
        last = {}
        for _ in range(R):
            w_line = MicroRotModel.measure_seq(p, line, rng)
            phi2, d2 = MicroRotModel.calib_2pt(line, w_line)
            w_grid = MicroRotModel.measure_seq(p, grid9, rng)
            n9 = MicroRotModel.calib_9pt(grid9, w_grid)
            phi2s.append(phi2); phi9s.append(n9["phi_urad"]); nines.append(n9)
            last = {"w_line": w_line, "d2": d2, "w_grid": w_grid, "n9": n9}
        phi2s, phi9s = np.array(phi2s), np.array(phi9s)
        return {"params": dict(p), "line": line, "grid9": grid9, "opt": opt,
                "phi2": float(np.mean(phi2s)), "phi2_reps": phi2s,
                "phi9": float(np.mean(phi9s)), "phi9_reps": phi9s,
                "scale_x_nm_mm": float(np.mean([n["scale_x_nm_mm"] for n in nines])),
                "scale_y_nm_mm": float(np.mean([n["scale_y_nm_mm"] for n in nines])),
                "shear_urad": float(np.mean([n["shear_urad"] for n in nines])),
                **last}

    # ---------- 解析误差传播 ----------
    @staticmethod
    def analytical(p):
        opt = MicroRotModel.optics(p)
        m = max(1, int(p["frames_per_point"]))
        R = max(1, int(p["repeats"]))
        P = max(2, int(p["line_points"]))
        s_pt = np.sqrt(opt["cam_nm"] ** 2 / m + p["stage_rep_nm"] ** 2)   # nm
        s_pt_um = s_pt * 1e-3
        if P == 2:
            s2 = np.sqrt(2) * s_pt_um / opt["L"]
        else:
            pitch = opt["L"] / (P - 1)
            s2 = s_pt_um / (pitch * np.sqrt(P * (P ** 2 - 1) / 12.0))
        s9 = s_pt_um / (opt["g"] * np.sqrt(6.0))
        s2 /= np.sqrt(R); s9 /= np.sqrt(R)
        kd = p["dist_edge_nm"] * 1e-3 / max(opt["r_fov"], 1e-9)
        bias2 = -2 * kd * p["scan_x_center_um"] * p["scan_y_offset_um"] \
            / opt["r_fov"] ** 2
        return {"sigma_pt_nm": s_pt, "cam_nm": opt["cam_nm"],
                "sigma_phi2_urad": s2 * 1e6, "three_sigma_phi2_urad": 3e6 * s2,
                "sigma_phi9_urad": s9 * 1e6, "three_sigma_phi9_urad": 3e6 * s9,
                "bias2_dist_urad": bias2 * 1e6,
                "moves_2pt": P * R, "moves_9pt": 9 * R,
                "L_um": opt["L"], "g_um": opt["g"], "r_fov": opt["r_fov"],
                "pix_um": opt["pix_um"], "P": P, "R": R}

    # ---------- 蒙特卡洛 ----------
    @staticmethod
    def monte_carlo(p, runs=300, seed0=30000):
        e2 = np.empty(runs); e9 = np.empty(runs)
        p2 = np.empty(runs); p9 = np.empty(runs)
        sh = np.empty(runs); sx = np.empty(runs); sy = np.empty(runs)
        for i in range(runs):
            r = MicroRotModel.run(p, seed=seed0 + i)
            p2[i], p9[i] = r["phi2"], r["phi9"]
            e2[i] = r["phi2"] - p["phi_true_urad"]
            e9[i] = r["phi9"] - p["phi_true_urad"]
            sh[i] = r["shear_urad"]
            sx[i] = r["scale_x_nm_mm"]; sy[i] = r["scale_y_nm_mm"]
        def st(v):
            return {"mean": float(np.mean(v)), "std": float(np.std(v, ddof=1)),
                    "p2_5": float(np.percentile(v, 2.5)),
                    "p97_5": float(np.percentile(v, 97.5)), "arr": v}
        return {"runs": runs, "phi2": st(p2), "phi9": st(p9),
                "err2": st(e2), "err9": st(e9), "delta": st(p2 - p9),
                "shear": st(sh), "scale_x": st(sx), "scale_y": st(sy)}

    # ---------- 诊断 ----------
    @staticmethod
    def diagnose(p, R, ana, mc):
        f = []
        ratio = (ana["sigma_phi2_urad"] / max(ana["sigma_phi9_urad"], 1e-12)) ** 2
        f.append(("info", "基线放大效应与方法效率",
                  f"σ_pt={ana['sigma_pt_nm']:.1f} nm 经 L={ana['L_um']:.0f} um 基线放大: "
                  f"两点法 1σ={ana['sigma_phi2_urad']:.1f}, 九点法 "
                  f"1σ={ana['sigma_phi9_urad']:.1f} urad",
                  f"两点法需 ≈{ratio:.1f} 倍重复次数才达到九点法同等一致性 "
                  f"(走位数 {ana['moves_2pt']} vs {ana['moves_9pt']}/组); "
                  f"提高放大倍率会缩小视场→缩短基线→角度噪声变差 (光学链联动)"))
        if abs(ana["bias2_dist_urad"]) > 0.5 * ana["sigma_phi2_urad"]:
            f.append(("warn", f"两点法畸变偏置显著 ({ana['bias2_dist_urad']:+.1f} urad)",
                      f"扫描中心偏离光轴 x0={p['scan_x_center_um']:.0f} um 且 "
                      f"线偏离 y0={p['scan_y_offset_um']:.0f} um, 经角部畸变 "
                      f"{p['dist_edge_nm']:.0f} nm 耦合 (≈−2·kd·x0·y0/r²)",
                      "将扫描线对称居中于光轴 (x0→0 或 y0→0) 可严格消除; 或改用九点法"))
        for tag, ekey, akey in (("两点法", "err2", "sigma_phi2_urad"),
                                ("九点法", "err9", "sigma_phi9_urad")):
            r_ = mc[ekey]["std"] / max(ana[akey], 1e-12)
            if r_ > 1.3:
                f.append(("warn", f"{tag}一致性劣于解析噪声极限 (×{r_:.2f})",
                          "逐走位热漂移(随机游走)在采集序列内积累",
                          "缩短单步走位时间, 或两组方向交替采集对消漂移"))
        d = mc["delta"]
        lim = 2 * np.hypot(ana["sigma_phi2_urad"], ana["sigma_phi9_urad"])
        if abs(d["mean"]) > lim:
            f.append(("warn", f"两法系统性互差 ({d['mean']:+.1f} urad)",
                      "超出噪声可解释范围, 通常来自两点法畸变偏置",
                      "消除偏置后复测; 以九点法为准"))
        else:
            f.append(("ok", "两法一致",
                      f"互差均值 {d['mean']:+.2f} urad, 区间 "
                      f"[{d['p2_5']:.1f}, {d['p97_5']:.1f}] urad",
                      "现场无真值时, 可将该互差区间作为自洽性验收判据"))
        f.append(("info", "九点法附加标定量 (交叉验证)",
                  f"剪切项 = {mc['shear']['mean']:+.1f} urad (台非正交真值 "
                  f"{p['stage_nonorth_urad']:+.1f}); 像素尺度偏差 X "
                  f"{mc['scale_x']['mean']:+.1f} / Y {mc['scale_y']['mean']:+.1f} nm/mm "
                  f"(真值 {p['scale_err_x_nm_mm']:+.0f}/{p['scale_err_y_nm_mm']:+.0f})",
                  "尺度偏差即'每移动1mm测量偏差多少nm', 可直接校核 像元尺寸/放大倍率 名义值"))
        rmax = ana["r_fov"]
        eq2 = 3 * mc["err2"]["std"] * rmax * 1e-3
        eq9 = 3 * mc["err9"]["std"] * rmax * 1e-3
        f.append(("info", "补偿后等效残余耦合 (标记位于视场角部)",
                  f"两点法 3σ×r_fov ≈ {eq2:.1f} nm, 九点法 ≈ {eq9:.1f} nm",
                  "应远小于相机重复性预算, 否则增加重复次数R或每点帧数m"))
        return f


# ===================================================================
#  GUI
# ===================================================================

class MicroRotApp(BaseSimApp):
    title = "D2W - 相机微旋标定仿真 (两点法 / 九点法, FR10/FR11)"
    subtitle = "相机微旋标定 算法在环  |  Camera Micro-Rotation: 2-Point vs 9-Point"
    report_name = "microrot_calib"
    model = MicroRotModel

    def schema(self):
        pix_fn = lambda p: f"{p['sensor_pixel_size']/p['lens_magnification']:.4f}"
        fov_fn = lambda p: (lambda o: f"{2*o['half_x']:.0f} × {2*o['half_y']:.0f}")(
            MicroRotModel.optics(p))
        L_fn = lambda p: f"{MicroRotModel.optics(p)['L']:.0f}"
        cam_fn = lambda p: f"{MicroRotModel.optics(p)['cam_nm']:.1f}"
        mv_fn = lambda p: (f"{max(2,int(p['line_points']))*max(1,int(p['repeats']))}"
                           f" / {9*max(1,int(p['repeats']))}")
        OPT_DEPS = ["sensor_pixel_size", "lens_magnification",
                    "sensor_w_px", "sensor_h_px", "span_frac", "subpixel_acc_px"]
        return [
            ("section", "光学系统 (与移动距离联动)"),
            ("entry", "sensor_pixel_size", "传感器像元尺寸", 3.45, "um"),
            ("entry", "lens_magnification", "镜头放大倍率", 6.3, "x"),
            ("derived", "_pix", "等效像素尺寸 (自动)", "um/px", pix_fn,
             ["sensor_pixel_size", "lens_magnification"]),
            ("entry", "sensor_w_px", "传感器宽度", 2448, "px"),
            ("entry", "sensor_h_px", "传感器高度", 2048, "px"),
            ("derived", "_fov", "视场 宽×高 (自动)", "um", fov_fn, OPT_DEPS),
            ("entry", "subpixel_acc_px", "亚像素定位精度", 0.015, "px"),
            ("derived", "_cam", "相机单帧噪声 (自动)", "nm", cam_fn, OPT_DEPS),
            ("note", "联动链: 像元/放大倍率→等效像素尺寸→视场→扫描基线; "
                     "相机噪声=亚像素精度×等效像素尺寸. 改放大倍率即可观察基线与"
                     "角度噪声的此消彼长"),

            ("section", "采集方案"),
            ("entry", "span_frac", "轨迹占视场比例", 0.8, "-"),
            ("derived", "_L", "两点法基线 L (自动)", "um", L_fn, OPT_DEPS),
            ("entry", "line_points", "两点法采样点数 P", 2, "点"),
            ("note", "P=2 为纯两点法; P>2 自动升级直线拟合, 对比页可量化加点收益"),
            ("entry", "repeats", "重复标定次数 R", 3, "次"),
            ("entry", "frames_per_point", "每点采集帧数 m", 3, "帧"),
            ("entry", "move_time_s", "单步走位时间", 1.5, "s"),
            ("derived", "_mv", "走位数 两点/九点 (自动)", "次", mv_fn,
             ["line_points", "repeats"]),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 300, "次"),

            ("section", "真值: 待标定量与干扰量"),
            ("entry", "phi_true_urad", "微旋角 φ (真值)", 2000.0, "urad"),
            ("entry", "stage_nonorth_urad", "台XY非正交 (干扰)", 100.0, "urad"),
            ("entry", "scale_err_x_nm_mm", "像素尺度偏差 X", 50.0, "nm/mm"),
            ("entry", "scale_err_y_nm_mm", "像素尺度偏差 Y", -30.0, "nm/mm"),
            ("note", "尺度偏差 nm/mm = 每移动1mm测量值偏差多少nm "
                     "(来源: 实际像元尺寸/放大倍率与名义值的差异). "
                     "两点法只走台X → 对台非正交免疫"),

            ("section", "扫描几何与畸变 (两点法偏置源)"),
            ("entry", "scan_x_center_um", "扫描中心偏离光轴 x0", 0.0, "um"),
            ("entry", "scan_y_offset_um", "扫描线偏离光轴 y0", 0.0, "um"),
            ("entry", "dist_edge_nm", "畸变位移 @视场角部", 30.0, "nm"),
            ("note", "两点法畸变偏置 ≈ −2·kd·x0·y0/r² — 对称居中扫描时严格为零. "
                     "可设 x0=y0=100 观察偏置出现"),

            ("section", "测量噪声与漂移"),
            ("entry", "stage_rep_nm", "台到位重复性 1σ", 25.0, "nm"),
            ("entry", "thermal_rw_nm_sqrt_min", "热漂移扩散系数", 8.0, "nm/√min"),
            ("note", "视场内基线仅百μm级 → nm级点噪声放大为百μrad级角度噪声, "
                     "这是该标定需要多帧×多次重复的根本原因. 验收区间见[验收参考]页"),
        ]

    # ---------------- run pipeline ----------------
    def _run(self):
        from tkinter import messagebox
        try:
            p = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        R = MicroRotModel.run(p)
        ana = MicroRotModel.analytical(p)
        mc = MicroRotModel.monte_carlo(p, runs=max(50, int(p.get("mc_runs") or 300)))
        diag = MicroRotModel.diagnose(p, R, ana, mc)
        self.R = {"result": R, "ana": ana, "mc": mc, "diag": diag}
        self._status_calib()
        self._render_all()

    def _status_calib(self):
        R, mc = self.R["result"], self.R["mc"]
        in2 = mc["phi2"]["p2_5"] <= R["phi2"] <= mc["phi2"]["p97_5"]
        in9 = mc["phi9"]["p2_5"] <= R["phi9"] <= mc["phi9"]["p97_5"]
        ok = in2 and in9
        bg = self.C["pass_bg"] if ok else self.C["ref_bg"]
        fg = self.C["pass_fg"] if ok else self.C["warn"]
        tag = "正常·位于验收区间" if ok else "超出验收区间 → 见诊断页"
        self.sfr.config(bg=bg)
        self.sl.config(bg=bg, fg=fg, font=(FONT, 11, "bold"), text=(
            f"[{tag}]  φ两点={R['phi2']:.1f}  φ九点={R['phi9']:.1f} urad "
            f"(真值{R['params']['phi_true_urad']:.0f})  一致性3σ: "
            f"{3*mc['err2']['std']:.1f} / {3*mc['err9']['std']:.1f} urad  "
            f"| 运行#{self.sim_n}"))

    # ---------------- 视场绘制助手 ----------------
    def _draw_fov(self, ax, opt, title):
        self._ax(ax, title, "相机 X (um)", "相机 Y (um)")
        hx, hy = opt["half_x"], opt["half_y"]
        ax.add_patch(Rectangle((-hx, -hy), 2 * hx, 2 * hy, facecolor="#eef3f9",
                               edgecolor=self.C["border"], lw=1.2, zorder=0))
        ax.axhline(0, color=self.C["dim"], lw=0.5, ls=":", zorder=1)
        ax.axvline(0, color=self.C["dim"], lw=0.5, ls=":", zorder=1)
        ax.plot(0, 0, "+", color=self.C["fail_fg"], ms=12, mew=1.6, zorder=3)
        ax.annotate("光轴", (0, 0), textcoords="offset points", xytext=(6, 6),
                    fontsize=8, color=self.C["fail_fg"])
        ax.set_xlim(-hx * 1.18, hx * 1.18)
        ax.set_ylim(-hy * 1.25, hy * 1.25)
        ax.set_aspect("equal")

    # ---------------- tabs ----------------
    def standard_tabs(self):
        return [("轨迹与视场", self._tab_traj),
                ("两点法", self._tab_2pt),
                ("九点法", self._tab_9pt),
                ("方法对比", self._tab_cmp),
                ("诊断与建议", self._tab_diag_c),
                ("验收参考", self._tab_accept_c)]

    def _tab_traj(self, tab, RR):
        R = RR["result"]; opt = R["opt"]; p = R["params"]
        fig = Figure(figsize=(10.8, 5.4), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._draw_fov(ax, opt, "视场与采集轨迹 (由传感器/放大倍率推导)")
        ax.plot(R["line"][:, 0], R["line"][:, 1], "o-", color=self.C["cx"],
                ms=7, lw=1.2, label=f"两点/线法 P={len(R['line'])}, "
                                    f"L={opt['L']:.0f} um", zorder=4)
        ax.scatter(R["grid9"][:, 0], R["grid9"][:, 1], s=52, marker="s",
                   facecolors="none", edgecolors=self.C["cy"], linewidths=1.6,
                   label=f"九点法 3×3, g={opt['g']:.0f} um", zorder=4)
        ax.legend(fontsize=8, loc="upper right")
        ax.annotate(f"视场 {2*opt['half_x']:.0f}×{2*opt['half_y']:.0f} um\n"
                    f"等效像素 {opt['pix_um']:.4f} um/px",
                    (-opt["half_x"] * 0.96, -opt["half_y"] * 1.0),
                    fontsize=8, color=self.C["dim"], va="bottom")
        ax2 = fig.add_subplot(122)
        self._draw_fov(ax2, opt, f"径向畸变场 (角部位移 {p['dist_edge_nm']:.0f} nm)")
        gg_x = np.linspace(-opt["half_x"], opt["half_x"], 11)
        gg_y = np.linspace(-opt["half_y"], opt["half_y"], 9)
        qx, qy = np.meshgrid(gg_x, gg_y)
        q = np.column_stack([qx.ravel(), qy.ravel()])
        kd = p["dist_edge_nm"] * 1e-3 / opt["r_fov"]
        dv = kd * (np.sum(q ** 2, axis=1, keepdims=True) / opt["r_fov"] ** 2) \
            * q * 1e3
        ax2.quiver(q[:, 0], q[:, 1], dv[:, 0], dv[:, 1],
                   np.linalg.norm(dv, axis=1), cmap="viridis", width=0.004)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_2pt(self, tab, RR):
        R = RR["result"]; ana = RR["ana"]; p = R["params"]
        opt = R["opt"]; mc = RR["mc"]
        w = R["w_line"]; u = R["line"]
        fig = Figure(figsize=(11.0, 5.6), dpi=100, facecolor=self.C["bg"])
        # ---- 左: 原理示意 (Δy放大) ----
        ax = fig.add_subplot(121)
        dy_um = w[-1, 1] - w[0, 1]
        exa = nice_exa(0.30 * opt["half_y"] / max(abs(dy_um), 1e-9))
        self._draw_fov(ax, opt, f"两点法原理示意  (Y向偏差放大 ×{exa})")
        xs, xe = w[0, 0], w[-1, 0]
        y0v = w[0, 1]
        ax.plot([xs, xe], [y0v, y0v], color=self.C["dim"], lw=1.2, ls="--",
                zorder=3, label="相机X方向 (平行参考线)")
        ye = y0v + dy_um * exa
        ax.plot([xs, xe], [y0v, ye], color=self.C["cx"], lw=2.0, zorder=4,
                label="标记实际轨迹 (台X方向)")
        ax.plot(xs, y0v, "o", color="#1a7f37", ms=12, zorder=5)
        ax.annotate("起点 (视场左)", (xs, y0v), textcoords="offset points",
                    xytext=(8, -16), fontsize=9, color="#1a7f37")
        ax.plot(xe, ye, "o", color=self.C["cx"], ms=12, zorder=5)
        ax.annotate("终点 (视场右)", (xe, ye), textcoords="offset points",
                    xytext=(-66, 10), fontsize=9, color=self.C["cx"])
        ax.add_patch(FancyArrowPatch((xe, y0v), (xe, ye), arrowstyle="-|>",
                                     mutation_scale=16, color=self.C["fail_fg"],
                                     lw=1.8, zorder=5))
        ax.annotate(f"Δy = {dy_um*1e3:.0f} nm\n(图中放大×{exa})",
                    (xe, (y0v + ye) / 2), textcoords="offset points",
                    xytext=(10, 0), fontsize=9, color=self.C["fail_fg"],
                    fontweight="bold")
        arc_r = opt["L"] * 0.30
        ang = np.degrees(np.arctan2(ye - y0v, xe - xs))
        ax.add_patch(Arc((xs, y0v), 2 * arc_r, 2 * arc_r, angle=0,
                         theta1=min(0, ang), theta2=max(0, ang),
                         color=self.C["accent2"], lw=1.8, zorder=4))
        ax.annotate(f"φ = -atan(Δy/Δx)\n  = {R['phi2_reps'][-1]:.1f} urad (末次)",
                    (xs + arc_r * 1.05, y0v + np.sign(ang) * arc_r * 0.30),
                    fontsize=9, color=self.C["accent2"], fontweight="bold")
        ax.legend(fontsize=8, loc="upper left")
        # ---- 右: R次重复 + 解析3σ带 ----
        ax2 = fig.add_subplot(122)
        self._ax(ax2, f"R={len(R['phi2_reps'])} 次重复的 φ估计", "重复序号",
                 "φ估计 (urad)")
        idx = np.arange(1, len(R["phi2_reps"]) + 1)
        tru = p["phi_true_urad"]
        band = ana["sigma_phi2_urad"] * np.sqrt(max(1, int(p["repeats"])))
        ax2.axhspan(tru - 3 * band, tru + 3 * band, alpha=0.10,
                    color=self.C["cx"], label="解析单次 ±3σ")
        ax2.axhline(tru, color=self.C["ref"], lw=1.6, ls=":",
                    label=f"真值 {tru:.0f}")
        ax2.plot(idx, R["phi2_reps"], "o", color=self.C["cx"], ms=8, zorder=4,
                 label="单次估计")
        ax2.axhline(R["phi2"], color=self.C["accent"], lw=1.8, ls="-.",
                    label=f"R次均值 {R['phi2']:.1f}")
        ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"φ估计(两点法, R次均值) = {R['phi2']:.2f} urad   |   一致性3σ(MC) = "
            f"{3*mc['err2']['std']:.1f} urad, 解析噪声极限 3σ = "
            f"{ana['three_sigma_phi2_urad']:.1f}   |   畸变偏置(解析) = "
            f"{ana['bias2_dist_urad']:+.2f} urad   |   基线 L = {ana['L_um']:.0f} um "
            f"(由放大倍率联动)   |   走位数 = {ana['moves_2pt']}"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_9pt(self, tab, RR):
        R = RR["result"]; ana = RR["ana"]; p = R["params"]
        opt = R["opt"]; mc = RR["mc"]
        u, w, n9 = R["grid9"], R["w_grid"], R["n9"]
        fig = Figure(figsize=(11.0, 5.6), dpi=100, facecolor=self.C["bg"])
        # ---- 左: 指令网格 vs 测得 (放大) + 坐标系旋转示意 ----
        ax = fig.add_subplot(121)
        dev = w - u
        exa = nice_exa(0.22 * opt["g"] / max(np.max(np.abs(dev)), 1e-9))
        self._draw_fov(ax, opt, f"九点法: 指令网格 vs 测得  (偏差放大 ×{exa})")
        ax.scatter(u[:, 0], u[:, 1], s=70, marker="s", facecolors="none",
                   edgecolors=self.C["dim"], linewidths=1.4, zorder=3,
                   label="台指令位置")
        wm = u + dev * exa
        ax.scatter(wm[:, 0], wm[:, 1], s=46, color=self.C["cy"], zorder=4,
                   label="相机测得 (偏差已放大)")
        for a_, b_ in zip(u, wm):
            ax.add_patch(FancyArrowPatch(a_, b_, arrowstyle="-|>",
                                         mutation_scale=10, color=self.C["cy"],
                                         lw=1.0, alpha=0.8, zorder=4))
        # 坐标系旋转示意 (角度放大)
        ax_len = opt["g"] * 0.55
        phi_show = R["phi9"] * 1e-6 * exa
        ax.add_patch(FancyArrowPatch((0, 0), (ax_len, 0), arrowstyle="-|>",
                                     mutation_scale=14, color=self.C["dim"],
                                     lw=1.6, zorder=5))
        ax.annotate("相机 x", (ax_len, 0), textcoords="offset points",
                    xytext=(4, -2), fontsize=9, color=self.C["dim"])
        ex_, ey_ = ax_len * np.cos(phi_show), ax_len * np.sin(phi_show)
        ax.add_patch(FancyArrowPatch((0, 0), (ex_, ey_), arrowstyle="-|>",
                                     mutation_scale=14, color=self.C["accent2"],
                                     lw=1.8, zorder=5))
        ax.annotate(f"台X方向\n(φ放大×{exa})", (ex_, ey_),
                    textcoords="offset points", xytext=(4, 4), fontsize=9,
                    color=self.C["accent2"], fontweight="bold")
        ax.legend(fontsize=8, loc="upper left")
        # ---- 右: 分解结果 — 画偏差(估计-真值), 绝对值入行标签, 量级可比 ----
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "H 分解结果: 估计偏差 (估计 - 真值)", "偏差", "")
        rows = [("微旋角 φ", R["phi9"], p["phi_true_urad"], "urad", self.C["cy"]),
                ("剪切≈台非正交", R["shear_urad"],
                 p["stage_nonorth_urad"], "urad", self.C["accent2"]),
                ("尺度偏差X", R["scale_x_nm_mm"],
                 p["scale_err_x_nm_mm"], "nm/mm", self.C["cx"]),
                ("尺度偏差Y", R["scale_y_nm_mm"],
                 p["scale_err_y_nm_mm"], "nm/mm", "#0a7ea4")]
        yp = np.arange(len(rows))[::-1]
        labels = []
        for y_, (lbl, est, tru, unit, col) in zip(yp, rows):
            dev_ = est - tru
            ax2.barh(y_, dev_, height=0.52, color=col, alpha=0.8)
            ax2.text(dev_, y_, f" {dev_:+.1f} {unit}",
                     fontsize=9, color=col, fontweight="bold",
                     va="center", ha="left" if dev_ >= 0 else "right")
            labels.append(f"{lbl}\n估计 {est:+.1f} (真值 {tru:+.0f}) {unit}")
        ax2.set_yticks(yp)
        ax2.set_yticklabels(labels, fontsize=8.5)
        ax2.axvline(0, color=self.C["dim"], lw=1.0)
        lim = max(abs(r[1] - r[2]) for r in rows) * 1.6 + 1e-9
        ax2.set_xlim(-lim, lim)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        H = n9["H"]
        tk.Label(tab, text=(
            f"φ估计(九点法, R次均值) = {R['phi9']:.2f} urad   |   一致性3σ(MC) = "
            f"{3*mc['err9']['std']:.1f} urad, 解析噪声极限 3σ = "
            f"{ana['three_sigma_phi9_urad']:.1f}   |   拟合残差 RMS = "
            f"{np.sqrt(np.mean(n9['resid_nm']**2)):.1f} nm   |   "
            f"H = [[{H[0,0]:.7f}, {H[0,1]:+.3e}], [{H[1,0]:+.3e}, {H[1,1]:.7f}]]"),
            bg=self.C["card"], fg=self.C["text"], font=("Consolas", 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_cmp(self, tab, RR):
        mc, ana = RR["mc"], RR["ana"]
        rmax = ana["r_fov"]
        fig = Figure(figsize=(10.8, 5.0), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, f"MC分布: φ估计误差 (N={mc['runs']})",
                 "φ估计 - 真值 (urad)", "频次")
        bins = np.histogram_bin_edges(
            np.concatenate([mc["err2"]["arr"], mc["err9"]["arr"]]), bins=34)
        ax.hist(mc["err2"]["arr"], bins=bins, color=self.C["cx"], alpha=0.55,
                label=f"两点法 3σ={3*mc['err2']['std']:.1f}")
        ax.hist(mc["err9"]["arr"], bins=bins, color=self.C["cy"], alpha=0.55,
                label=f"九点法 3σ={3*mc['err9']['std']:.1f}")
        ax.axvline(0, color=self.C["dim"], lw=0.8)
        ax.legend(fontsize=8)
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "一致性与偏置对比", "", "urad / nm")
        names = ["3σ一致性\n(urad)", "|系统偏置|\n(urad)",
                 "等效残余耦合\n3σ×r_fov (nm)"]
        v2 = [3 * mc["err2"]["std"], abs(mc["err2"]["mean"]),
              3 * mc["err2"]["std"] * rmax * 1e-3]
        v9 = [3 * mc["err9"]["std"], abs(mc["err9"]["mean"]),
              3 * mc["err9"]["std"] * rmax * 1e-3]
        xp = np.arange(3)
        ax2.bar(xp - 0.18, v2, 0.36, color=self.C["cx"], label="两点法")
        ax2.bar(xp + 0.18, v9, 0.36, color=self.C["cy"], label="九点法")
        for x0, (a, b) in enumerate(zip(v2, v9)):
            ax2.text(x0 - 0.18, a, f"{a:.1f}", ha="center", va="bottom", fontsize=8)
            ax2.text(x0 + 0.18, b, f"{b:.1f}", ha="center", va="bottom", fontsize=8)
        ax2.set_xticks(xp); ax2.set_xticklabels(names, fontsize=8)
        ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"解析噪声极限 1σ: 两点 {ana['sigma_phi2_urad']:.1f} / 九点 "
            f"{ana['sigma_phi9_urad']:.1f} urad (MC实测 {mc['err2']['std']:.1f} / "
            f"{mc['err9']['std']:.1f})   |   两法互差区间 "
            f"[{mc['delta']['p2_5']:.1f}, {mc['delta']['p97_5']:.1f}] urad   |   "
            f"走位数 {ana['moves_2pt']} vs {ana['moves_9pt']}"),
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
        txt.insert("end", "标定诊断与工程建议\n\n", "h")
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
            f"建议验收区间 (由当前误差模型MC推导, N={mc['runs']}, "
            f"R={ana['R']}次重复均值口径):\n"
            f"   两点法 φ估计 ∈ [{mc['phi2']['p2_5']:.1f}, "
            f"{mc['phi2']['p97_5']:.1f}] urad   (一致性 3σ = "
            f"{3*mc['err2']['std']:.1f}; 解析噪声极限 3σ = "
            f"{ana['three_sigma_phi2_urad']:.1f})\n"
            f"   九点法 φ估计 ∈ [{mc['phi9']['p2_5']:.1f}, "
            f"{mc['phi9']['p97_5']:.1f}] urad   (一致性 3σ = "
            f"{3*mc['err9']['std']:.1f}; 解析噪声极限 3σ = "
            f"{ana['three_sigma_phi9_urad']:.1f})\n"
            f"   两法互差 ∈ [{mc['delta']['p2_5']:.1f}, "
            f"{mc['delta']['p97_5']:.1f}] urad (现场无真值时的自洽性判据)\n"
            f"   九点法附加: 剪切 ∈ [{mc['shear']['p2_5']:.1f}, "
            f"{mc['shear']['p97_5']:.1f}] urad, 尺度偏差X ∈ "
            f"[{mc['scale_x']['p2_5']:.1f}, {mc['scale_x']['p97_5']:.1f}] nm/mm\n"
            f"判读: 实测标定值落于区间内且多次标定std≤一致性3σ/3 → 正常; "
            f"超出 → 按诊断页排查"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(10.8, 3.9), dpi=100, facecolor=self.C["bg"])
        specs = [("phi2", "两点法 φ估计 (urad)", self.C["cx"]),
                 ("phi9", "九点法 φ估计 (urad)", self.C["cy"]),
                 ("delta", "两法互差 (urad)", "#1a7f37")]
        for i, (k, ttl, col) in enumerate(specs):
            ax = fig.add_subplot(1, 3, i + 1)
            self._ax(ax, f"MC分布: {ttl}", ttl, "频次")
            s = mc[k]
            ax.hist(s["arr"], bins=26, color=col, alpha=0.7, edgecolor="white",
                    lw=0.4)
            ax.axvspan(s["p2_5"], s["p97_5"], alpha=0.10, color=self.C["ref"])
            for q in ("p2_5", "p97_5"):
                ax.axvline(s[q], color="#e5a00d", lw=1.2, ls="--")
            if k in ("phi2", "phi9"):
                ax.axvline(p["phi_true_urad"], color=self.C["ref"], lw=1.8,
                           ls="-.", label=f"真值={p['phi_true_urad']:.0f}")
                ax.legend(fontsize=7)
            else:
                ax.axvline(0, color=self.C["dim"], lw=0.8)
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
            initialfile=f"{self.report_name}_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        R, ana, mc = self.R["result"], self.R["ana"], self.R["mc"]
        p = R["params"]
        L = ["=" * 76, f"  {self.title}", "=" * 76,
             f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   "
             f"simcore v{simcore.__version__}",
             f"  Optics: pixel {p['sensor_pixel_size']} um / "
             f"{p['lens_magnification']}x = {ana['pix_um']:.4f} um/px, "
             f"FOV diag r={ana['r_fov']:.0f} um, cam noise "
             f"{ana['cam_nm']:.1f} nm/frame",
             f"  Layout: L={ana['L_um']:.0f} um, P={ana['P']}, "
             f"g={ana['g_um']:.0f} um, m={int(p['frames_per_point'])}, "
             f"R={ana['R']}, sigma_pt={ana['sigma_pt_nm']:.1f} nm",
             "", "-" * 76, "  CALIBRATION RESULT (R-rep mean)", "-" * 76,
             f"  2-point: phi = {R['phi2']:.2f} urad  (truth "
             f"{p['phi_true_urad']:.1f}, err {R['phi2']-p['phi_true_urad']:+.2f})",
             f"  9-point: phi = {R['phi9']:.2f} urad  (truth "
             f"{p['phi_true_urad']:.1f}, err {R['phi9']-p['phi_true_urad']:+.2f})",
             f"  9-point extras: shear={R['shear_urad']:+.1f} urad "
             f"(stage nonorth truth {p['stage_nonorth_urad']:+.1f}), "
             f"scale X={R['scale_x_nm_mm']:+.1f} / Y={R['scale_y_nm_mm']:+.1f} "
             f"nm/mm (truth {p['scale_err_x_nm_mm']:+.0f}/"
             f"{p['scale_err_y_nm_mm']:+.0f})",
             "", "-" * 76, f"  ACCEPTANCE REFERENCE (MC, N={mc['runs']})", "-" * 76,
             f"  2pt phi interval: [{mc['phi2']['p2_5']:.1f}, "
             f"{mc['phi2']['p97_5']:.1f}] urad  3sigma={3*mc['err2']['std']:.1f} "
             f"(analytic {ana['three_sigma_phi2_urad']:.1f})",
             f"  9pt phi interval: [{mc['phi9']['p2_5']:.1f}, "
             f"{mc['phi9']['p97_5']:.1f}] urad  3sigma={3*mc['err9']['std']:.1f} "
             f"(analytic {ana['three_sigma_phi9_urad']:.1f})",
             f"  method delta:     [{mc['delta']['p2_5']:.1f}, "
             f"{mc['delta']['p97_5']:.1f}] urad",
             f"  2pt distortion bias (analytic): "
             f"{ana['bias2_dist_urad']:+.2f} urad",
             f"  equivalent residual coupling 3sigma x r_fov: "
             f"2pt {3*mc['err2']['std']*ana['r_fov']*1e-3:.1f} / "
             f"9pt {3*mc['err9']['std']*ana['r_fov']*1e-3:.1f} nm",
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
        """实测数据: type,cmd_x_um,cmd_y_um,meas_x_um,meas_y_um (type=L/G).
        跑同一套 calib_2pt / calib_9pt 并与仿真验收区间对比."""
        from tkinter import filedialog, messagebox, ttk
        import csv
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"),
                                                     ("All", "*.*")])
        if not path:
            return
        try:
            rows = []
            with open(path, "r", encoding="utf-8-sig") as fh:
                for row in csv.reader(fh):
                    cells = [c.strip() for c in row if c.strip() != ""]
                    if len(cells) >= 5 and cells[0].upper() in ("L", "G"):
                        rows.append((cells[0].upper(),
                                     *[float(c) for c in cells[1:5]]))
            if not rows:
                raise ValueError("未找到有效数据行")
            lines = [f"文件: {path}", "=" * 70]
            phi2 = phi9 = None
            sel = [r for r in rows if r[0] == "L"]
            if len(sel) >= 2:
                u = np.array([[r[1], r[2]] for r in sel])
                w = np.array([[r[3], r[4]] for r in sel])
                o = np.argsort(u[:, 0]); u, w = u[o], w[o]
                phi2, _ = MicroRotModel.calib_2pt(u, w)
                lines.append(f"[两点/线法] P={len(u)}  φ估计 = {phi2:.2f} urad")
            sel = [r for r in rows if r[0] == "G"]
            if len(sel) >= 6:
                u = np.array([[r[1], r[2]] for r in sel])
                w = np.array([[r[3], r[4]] for r in sel])
                n9 = MicroRotModel.calib_9pt(u, w)
                phi9 = n9["phi_urad"]
                lines += [f"[九点法] 点数={len(u)}  φ估计 = {phi9:.2f} urad",
                          f"  剪切 = {n9['shear_urad']:+.1f} urad  尺度偏差 X "
                          f"{n9['scale_x_nm_mm']:+.1f} / Y "
                          f"{n9['scale_y_nm_mm']:+.1f} nm/mm",
                          f"  拟合残差 RMS = "
                          f"{np.sqrt(np.mean(n9['resid_nm']**2)):.1f} nm"]
            if phi2 is not None and phi9 is not None:
                lines.append(f"[一致性] 两法互差 = {phi2-phi9:+.2f} urad")
            if self.R:
                mc = self.R["mc"]
                for tag, v, key in (("两点法", phi2, "phi2"),
                                    ("九点法", phi9, "phi9")):
                    if v is not None:
                        lo, hi = mc[key]["p2_5"], mc[key]["p97_5"]
                        lines.append(f"[验收对比] {tag} φ={v:.1f} vs 仿真区间 "
                                     f"[{lo:.1f}, {hi:.1f}] urad → "
                                     f"{'位于区间内' if lo <= v <= hi else '超出区间'}")
            else:
                lines.append("提示: 先运行仿真可获得验收区间对比。")
            win = tk.Toplevel(self.root)
            win.title("实测标定数据分析"); win.geometry("700x380")
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
                f"{e}\n\n格式: type,cmd_x_um,cmd_y_um,meas_x_um,meas_y_um\n"
                f"type: L=线扫描点, G=九点网格点; 坐标单位um; 表头可选")


if __name__ == "__main__":
    launch(MicroRotApp)
