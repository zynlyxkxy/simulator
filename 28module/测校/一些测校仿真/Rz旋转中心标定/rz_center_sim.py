#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W Rz旋转中心标定仿真 — 2D SVD法 (正向生成 / SVD逆解 / 真值对比残差)
参考文档: SDS §4.3 | 算法来源: find_rotation_center.py (2D SVD, 忠实移植)

仿真闭环 (与真实测校流程一一对应):
  ① 输入真实旋转中心 C=(cx,cy) mm (已知真值, 仿真用它做正向旋转)
  ② 正向: 标记阵列零位真实位置 P, 绕真实中心C旋转角度 θ(rad)
     → 旋转后真实位置 P' = R(θ)·(P−C) + C
  ③ 测量: P、P' 各叠加相机+运动台测量噪声 (σ_xy)
  ④ 逆解: SVD刚体配准 → 旋转矩阵R̂、平移T̂ →
     旋转中心 Ĉ = (I−R̂)⁻¹·T̂,  角度 θ̂ = atan2(R̂₁₀, R̂₀₀)
  ⑤ 残差 = 解算中心 − 真实中心 (XY偏差与径向距离), 即该项测校残差.

影响测校精度的因素 (已识别并代入):
  · 小转角剧烈放大 (本项核心): 中心误差 ≈ σ_xy/(2·sin(θ/2))/√M ∝ 1/θ.
    θ=0.001 rad → 放大 ≈1000×; 40nm噪声 → 数十μm中心误差. 中心标定必须用大角!
  · 配对策略: 单对(两角位)受该对噪声主导; 多角位都对零位配对再平均 → /√(对数)
  · 同步径向跳动: 使逐配对中心估计随角度呈圆轨迹 (轴承同步误差指纹, 可索引补偿)
  · NRRO(非同步跳动): 不可补偿底噪
  · 标记展布: 角度估计精度 ∝ 展布; 中心横向精度受标记几何分布影响

验收判据 (无固定Spec, MC推导 + 解析交叉校验):
  中心残差区间 (径向/XY分量) / 角度残差区间 / RMSE区间 / 逐配对散布(同步跳动可见性)
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure

import simcore
from simgui import BaseSimApp, launch, FONT


# ===================================================================
#  算法 (忠实移植自 find_rotation_center.py)
# ===================================================================

def find_rotation_center(p, p_prime):
    """2D SVD刚体配准求旋转中心. 返回 (C, theta). 与上传脚本逐步一致."""
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

class RotCenterModel:
    """单位: 位置 mm (残差显示 nm/um), 角度 rad (显示 rad/urad), 噪声 nm."""

    # ---------- 真实中心 + 标记 ----------
    @staticmethod
    def true_center(p):
        return np.array([p["center_x_mm"], p["center_y_mm"]])

    @staticmethod
    def marks(p):
        """标记阵列: G×G 网格, 以真实中心为参考布置 (展布=边长)."""
        G = max(2, int(p["marks_grid"]))
        s = p["marks_span_mm"] / 2.0
        C = RotCenterModel.true_center(p)
        ax_ = np.linspace(-s, s, G)
        gx, gy = np.meshgrid(ax_, ax_, indexing="ij")
        off = np.array([p["marks_offset_x_mm"], p["marks_offset_y_mm"]])
        return C + off + np.column_stack([gx.ravel(), gy.ravel()])

    @staticmethod
    def _noise(p, rng, n):
        m = max(1, int(p["frames_per_point"]))
        sxy = np.sqrt(p["cam_sigma_nm"] ** 2 / m + p["stage_rep_nm"] ** 2) * 1e-6
        return rng.normal(0, sxy, (n, 2)), sxy * 1e6

    @staticmethod
    def _center_at(p, ang, rng):
        """该角位下的真实中心 (名义 + 同步径向跳动 + NRRO)."""
        C0 = RotCenterModel.true_center(p)
        rs = p["runout_sync_nm"] * 1e-6
        ph = np.deg2rad(p["runout_phase_deg"])
        c = C0 + rs * np.array([np.cos(ang + ph), np.sin(ang + ph)])
        if rng is not None and p["nrro_nm"] > 0:
            c = c + rng.normal(0, p["nrro_nm"] * 1e-6, 2)
        return c

    # ---------- 一次完整标定 (R次重复取均值) ----------
    @staticmethod
    def calib(p, rng):
        K = max(2, int(p["num_angles"]))
        arc = np.deg2rad(p["sweep_arc_deg"])
        angles = np.linspace(0.0, arc, K)             # 站0=零位, 其余为各角位
        P0 = RotCenterModel.marks(p)
        reps = max(1, int(p["repeats"]))
        Cs, ths, rmses, pairwise = [], [], [], []
        last = {}
        for _ in range(reps):
            # 零位测量
            c0 = RotCenterModel._center_at(p, angles[0], rng)
            eP, sxy = RotCenterModel._noise(p, rng, len(P0))
            Pm = (R0 := rot2d(angles[0])) @ (P0 - c0).T
            Pm = (Pm.T + c0) + eP                     # 零位实测
            cands = []
            wts = []
            clouds = [Pm]
            gate = min(np.sin(np.deg2rad(2)),
                       0.5 * abs(np.sin((angles[-1] - angles[0]) / 2)))
            for k in range(1, K):
                ck = RotCenterModel._center_at(p, angles[k], rng)
                Pk = (rot2d(angles[k]) @ (P0 - ck).T).T + ck
                eQ, _ = RotCenterModel._noise(p, rng, len(P0))
                Qm = Pk + eQ
                clouds.append(Qm)
                sh = abs(np.sin((angles[k] - angles[0]) / 2))
                if sh < gate:                         # 病态对剔除 (相对门槛)
                    continue
                C_h, th_h = find_rotation_center(Pm, Qm)
                cands.append(C_h)
                wts.append(sh ** 2)                   # 方差∝1/sin² → 权重∝sin²
                pairwise.append((angles[k], C_h))
                # RMSE 质量指标
                R_ = rot2d(th_h)
                fit = (R_ @ (Pm - C_h).T).T + C_h
                rmses.append(np.sqrt(np.mean(np.sum((fit - Qm) ** 2, axis=1))))
                ths.append(th_h / max(angles[k] - angles[0], 1e-12))
            if cands:
                w = np.array(wts); w = w / w.sum()
                Cs.append((np.array(cands) * w[:, None]).sum(axis=0))
            last = {"clouds": clouds, "angles": angles}
        Cs = np.array(Cs)
        C_hat = Cs.mean(axis=0) if len(Cs) else np.full(2, np.nan)
        C_true = RotCenterModel.true_center(p)
        err = (C_hat - C_true) * 1e6                  # nm
        pw_ang = np.array([a for a, _ in pairwise])
        pw_C = np.array([c for _, c in pairwise]) if pairwise else np.zeros((0, 2))
        return {"C_true": C_true, "C_hat": C_hat,
                "err_nm": err, "err_radial_nm": float(np.linalg.norm(err)),
                "Cs_reps": Cs, "pw_ang": pw_ang, "pw_C": pw_C,
                "theta_gain": float(np.mean(ths)) if ths else 1.0,
                "rmse_nm": float(np.mean(rmses) * 1e6) if rmses else 0.0,
                "scatter_nm": pw_C.std(axis=0) * 1e6 if len(pw_C) > 2
                else np.zeros(2),
                "angles": angles, "P0": P0, "sxy_nm": sxy, **last}

    @staticmethod
    def run(p, seed=None):
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))
        return {"params": dict(p), "rc": RotCenterModel.calib(p, rng)}

    # ---------- 解析 ----------
    @staticmethod
    def analytical(p):
        G = max(2, int(p["marks_grid"]))
        M = G * G
        m = max(1, int(p["frames_per_point"]))
        R = max(1, int(p["repeats"]))
        K = max(2, int(p["num_angles"]))
        sxy = np.sqrt(p["cam_sigma_nm"] ** 2 / m + p["stage_rep_nm"] ** 2)
        arc = np.deg2rad(p["sweep_arc_deg"])
        th_max = max(arc, 1e-9)
        # 单对中心噪声 (前后各一次测量→√2, 质心→/√M, 配准映射→1/(2sin(Δθ/2)))
        amp = 1.0 / (2 * np.sin(th_max / 2))
        sig_c_pair = sxy * np.sqrt(2.0 / M) * amp                 # nm, 最大角对
        # 多角位最优策略 = 方差加权 (≈ 仅最大角对主导, 中间小角对权重低);
        # 仅R次重复独立 → 1/√R. 朴素等权平均反而更差 (见诊断), 故不用 √(K-1).
        sig_c = sig_c_pair / np.sqrt(R)
        n_pairs = max(1, K - 1)
        # 角度噪声
        s = p["marks_span_mm"] / 2.0
        ax_ = np.linspace(-s, s, G)
        Sxx = float(M / G * np.sum((ax_ - ax_.mean()) ** 2))
        sig_theta = sxy * 1e-6 * np.sqrt(2.0) / np.sqrt(2 * Sxx) / np.sqrt(n_pairs * R)
        return {"sigma_xy_nm": sxy, "M": M, "amp_factor": amp,
                "sig_center_pair_nm": sig_c_pair, "sig_center_nm": sig_c,
                "three_sigma_center_nm": 3 * sig_c,
                "sig_theta_urad": sig_theta * 1e6, "n_pairs": n_pairs,
                "th_max_rad": th_max, "Sxx_mm2": Sxx}

    # ---------- 蒙特卡洛 ----------
    @staticmethod
    def monte_carlo(p, runs=200, seed0=60000):
        ce = np.empty(runs); ex = np.empty(runs); ey = np.empty(runs)
        rm = np.empty(runs); tg = np.empty(runs)
        for i in range(runs):
            r = RotCenterModel.run(p, seed=seed0 + i)["rc"]
            ce[i] = r["err_radial_nm"]
            ex[i] = r["err_nm"][0]; ey[i] = r["err_nm"][1]
            rm[i] = r["rmse_nm"]; tg[i] = r["theta_gain"]
        def st(v):
            return {"mean": float(np.mean(v)), "std": float(np.std(v, ddof=1)),
                    "p2_5": float(np.percentile(v, 2.5)),
                    "p97_5": float(np.percentile(v, 97.5)), "arr": v}
        return {"runs": runs, "center_err": st(ce), "err_x": st(ex),
                "err_y": st(ey), "rmse": st(rm), "theta_gain": st(tg)}

    # ---------- 诊断 ----------
    @staticmethod
    def diagnose(p, R, ana, mc):
        f = []
        rc = R["rc"]
        amp = ana["amp_factor"]
        f.append(("info", "影响因素①: 小转角放大 (本项核心矛盾)",
                  f"中心误差 ≈ σxy·√(2/M)/(2sin(Δθ/2)); 当前最大配对角 "
                  f"{np.rad2deg(ana['th_max_rad']):.1f}° → 放大因子 {amp:.1f}×. "
                  f"σxy={ana['sigma_xy_nm']:.0f} nm → 单对中心噪声 "
                  f"{ana['sig_center_pair_nm']:.0f} nm",
                  "中心标定必须用大转角 (与增益标定相反!). 若行程允许, "
                  "增大扫描弧段比增加标记数更有效 (∝1/sin(θ/2) vs ∝1/√M)"))
        if p["sweep_arc_deg"] < 30:
            f.append(("warn", f"扫描弧段过小 ({p['sweep_arc_deg']:.0f}°)",
                      f"放大因子高达 {amp:.0f}×, 中心估计被噪声主导",
                      "建议扫描弧段 ≥90°; 受机械行程限制时至少用满可用行程"))
        # 配对策略
        f.append(("info", "影响因素②: 多角位配对 (方差加权)",
                  f"各配对按 sin²(Δθ/2) 加权 — 大角对主导, 中间小角对权重低. "
                  f"注意: 朴素等权平均反而更差 (小角对放大噪声拖累均值). "
                  f"独立改善仅来自重复 R: 1/√{int(p['repeats'])}",
                  "提升精度优先增大总弧段, 而非堆中间角位; 中间角位主要用于"
                  "暴露同步跳动(逐配对圆轨迹), 对中心精度本身贡献有限"))
        # 同步跳动
        if p["runout_sync_nm"] > 2 * ana["sig_center_pair_nm"]:
            sc = rc["scatter_nm"]
            f.append(("info", "影响因素③: 同步径向跳动呈圆轨迹",
                      f"逐配对中心散布 ≈ ({sc[0]:.0f}, {sc[1]:.0f}) nm vs 噪声极限 "
                      f"{ana['sig_center_pair_nm']:.0f} nm — 圆形轨迹是轴承同步误差指纹",
                      "同步分量可按角度索引建LUT补偿; NRRO为不可补偿底噪"))
        # RMSE 质量
        if rc["rmse_nm"] > 5 * ana["sigma_xy_nm"]:
            f.append(("warn", f"配准RMSE偏高 ({rc['rmse_nm']:.0f} nm)",
                      "远超测量噪声 → 可能非刚体运动/标记误识别/夹持滑移",
                      "核对标记识别与真空吸附 (脚本原始质量判据)"))
        else:
            f.append(("ok", "配准质量正常",
                      f"RMSE={rc['rmse_nm']:.0f} nm ≈ 测量噪声水平 "
                      f"({ana['sigma_xy_nm']:.0f} nm)",
                      "刚体假设成立, 标定可信"))
        # 验收
        f.append(("info", "中心残差→等效影响",
                  f"径向残差P97.5 = {mc['center_err']['p97_5']:.0f} nm. "
                  f"旋转中心偏差直接传入Rz对位: 工作半径r处附加切向误差 ≈ "
                  f"残差 (与r无关, 为刚性平移)",
                  "该值进入键合对位误差链, 与上游预算核对"))
        return f


# ===================================================================
#  GUI
# ===================================================================

class RotCenterApp(BaseSimApp):
    title = "D2W - Rz旋转中心标定仿真 (2D SVD法)"
    subtitle = ("Rz旋转中心标定: ①零位测标记 → ②绕真实中心转θ → ③重测 → "
                "④SVD逆解中心 → ⑤与真值对比 = 测校残差")
    report_name = "rot_center"
    model = RotCenterModel

    def schema(self):
        amp_fn = lambda p: f"{1/(2*np.sin(np.deg2rad(max(p['sweep_arc_deg'],0.01))/2)):.0f}"
        sxy_fn = lambda p: f"{np.sqrt(p['cam_sigma_nm']**2/max(1,int(p['frames_per_point']))+p['stage_rep_nm']**2):.0f}"
        return [
            ("section", "① 标记阵列 (零位先测一遍平面位置)"),
            ("entry", "marks_grid", "标记网格 G×G", 3, "G"),
            ("entry", "marks_span_mm", "标记展布 (边长)", 40.0, "mm"),
            ("entry", "marks_offset_x_mm", "标记区相对中心偏置 X", 0.0, "mm"),
            ("entry", "marks_offset_y_mm", "标记区相对中心偏置 Y", 0.0, "mm"),
            ("note", "标记由相机+运动台测出平面坐标 (x,y). 标记区可偏离中心 "
                     "(贴近实际芯片位置)"),

            ("section", "② 旋转方案 (角度单位: rad/deg)"),
            ("entry", "num_angles", "测量角位数 K (含零位)", 5, "个"),
            ("entry", "sweep_arc_deg", "总扫描弧段", 90.0, "deg"),
            ("derived", "_amp", "中心误差放大因子 (自动)", "×", amp_fn,
             ["sweep_arc_deg"]),
            ("entry", "frames_per_point", "每点采集帧数 m", 3, "帧"),
            ("entry", "repeats", "整套流程重复次数 R", 3, "次"),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 200, "次"),
            ("note", "核心: 中心误差 ∝ 1/sin(θ/2) — 转角越小放大越剧烈. "
                     "中心标定必须用大角 (与角度增益标定相反)"),

            ("section", "真实旋转中心 (已知真值, 仿真用它做正向旋转)"),
            ("entry", "center_x_mm", "中心 X", 5.0, "mm"),
            ("entry", "center_y_mm", "中心 Y", 3.0, "mm"),
            ("note", "正向: P' = R(θ)·(P−C) + C; 逆解 Ĉ=(I−R̂)⁻¹T̂ 后与真值比残差"),

            ("section", "③ 轴系与测量噪声 (影响测校精度的因素)"),
            ("entry", "cam_sigma_nm", "相机单帧对准 1σ", 8.0, "nm"),
            ("entry", "stage_rep_nm", "台到位重复性 1σ", 25.0, "nm"),
            ("derived", "_sxy", "XY点噪声 σxy (自动)", "nm", sxy_fn,
             ["cam_sigma_nm", "stage_rep_nm", "frames_per_point"]),
            ("entry", "runout_sync_nm", "同步径向跳动 (一阶)", 200.0, "nm"),
            ("entry", "runout_phase_deg", "同步跳动相位", 0.0, "deg"),
            ("entry", "nrro_nm", "非同步跳动 NRRO 1σ", 30.0, "nm"),
            ("note", "同步跳动使逐配对中心估计呈圆轨迹 (可索引补偿), "
                     "NRRO为不可补偿底噪. 详见[逐配对中心]页"),
        ]

    # ---------------- run ----------------
    def _run(self):
        from tkinter import messagebox
        try:
            p = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        R = RotCenterModel.run(p)
        ana = RotCenterModel.analytical(p)
        mc = RotCenterModel.monte_carlo(p, runs=max(50, int(p.get("mc_runs") or 200)))
        diag = RotCenterModel.diagnose(p, R, ana, mc)
        self.R = {"result": R, "ana": ana, "mc": mc, "diag": diag}
        self._status_calib()
        self._render_all()

    def _status_calib(self):
        R, mc = self.R["result"], self.R["mc"]
        rc = R["rc"]
        ok = rc["err_radial_nm"] <= mc["center_err"]["p97_5"] * 1.0001
        bg = self.C["pass_bg"] if ok else self.C["ref_bg"]
        fg = self.C["pass_fg"] if ok else self.C["warn"]
        tag = "正常·位于验收区间" if ok else "超出验收区间 → 见诊断页"
        self.sfr.config(bg=bg)
        self.sl.config(bg=bg, fg=fg, font=(FONT, 11, "bold"), text=(
            f"[{tag}]  中心残差 = {rc['err_radial_nm']:.0f} nm "
            f"(径向, ≤{mc['center_err']['p97_5']:.0f})   "
            f"Ĉ=({rc['C_hat'][0]:.4f}, {rc['C_hat'][1]:.4f}) mm   "
            f"RMSE={rc['rmse_nm']:.0f} nm  | 运行#{self.sim_n}"))

    # ---------------- tabs ----------------
    def standard_tabs(self):
        return [("流程与布置", self._tab_layout),
                ("中心解算", self._tab_center),
                ("逐配对中心", self._tab_pairwise),
                ("转角灵敏度", self._tab_sens),
                ("诊断与建议", self._tab_diag_c),
                ("验收参考", self._tab_accept_c)]

    def _tab_layout(self, tab, RR):
        R = RR["result"]; rc = R["rc"]; p = R["params"]
        head = tk.Frame(tab, bg=self.C["ref_bg"]); head.pack(fill="x", padx=10,
                                                             pady=(8, 2))
        tk.Label(head, text=(
            "仿真闭环 = 真实测校流程:  ① 零位测标记平面位置 P (相机+台)  →  "
            "② 绕[真实旋转中心]转角度θ: P' = R(θ)·(P−C) + C  →  ③ 重测得 P' "
            "(前后均含噪声)  →  ④ SVD逆解: R̂,T̂ → 中心 Ĉ=(I−R̂)⁻¹T̂  →  "
            "⑤ 残差 = Ĉ − C (径向距离) = 该项测校残差"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", wraplength=1020, padx=10, pady=6).pack(
            fill="x")
        fig = Figure(figsize=(10.8, 5.0), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(111)
        self._ax(ax, "标记阵列绕中心旋转 (各角位叠加, 颜色=角度)",
                 "X (mm)", "Y (mm)")
        clouds, angles = rc["clouds"], rc["angles"]
        from matplotlib import cm
        K = len(clouds)
        for k, cl in enumerate(clouds):
            ax.scatter(cl[:, 0], cl[:, 1], s=24,
                       color=cm.viridis(k / max(K - 1, 1)),
                       label=f"{np.rad2deg(angles[k]):.0f}°" if K <= 6 else None)
        ax.scatter(*rc["C_true"], marker="+", s=160, color=self.C["fail_fg"],
                   zorder=6, label="真实中心")
        ax.scatter(*rc["C_hat"], marker="x", s=130, color=self.C["accent"],
                   zorder=6, label=f"解算中心 (残差 {rc['err_radial_nm']:.0f} nm)")
        if K <= 6:
            ax.legend(fontsize=8, loc="best")
        else:
            ax.legend(fontsize=8, loc="best",
                      handles=[ax.collections[-2], ax.collections[-1]])
        ax.set_aspect("equal")
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"标记: {int(p['marks_grid'])}×{int(p['marks_grid'])} 网格, 展布 "
            f"{p['marks_span_mm']:.0f} mm   |   角位 {int(p['num_angles'])} 个 / "
            f"弧段 {p['sweep_arc_deg']:.0f}°   |   σxy={rc['sxy_nm']:.0f} nm   |   "
            f"真值中心 ({p['center_x_mm']:.3f}, {p['center_y_mm']:.3f}) → 解算 "
            f"({rc['C_hat'][0]:.4f}, {rc['C_hat'][1]:.4f}) mm"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_center(self, tab, RR):
        R = RR["result"]; rc = R["rc"]; ana = RR["ana"]; mc = RR["mc"]
        fig = Figure(figsize=(10.8, 5.2), dpi=100, facecolor=self.C["bg"])
        # 左: 残差放大视图 (中心附近)
        ax = fig.add_subplot(121)
        self._ax(ax, "解算中心 vs 真值 (放大视图)", "X − C真 (nm)", "Y − C真 (nm)")
        if len(rc["Cs_reps"]):
            d = (rc["Cs_reps"] - rc["C_true"]) * 1e6
            ax.scatter(d[:, 0], d[:, 1], s=40, color=self.C["dim"], alpha=0.7,
                       label="各次重复解")
        e = rc["err_nm"]
        ax.scatter(*e, marker="x", s=130, color=self.C["accent"], zorder=5,
                   label=f"均值解 (残差 {rc['err_radial_nm']:.0f} nm)")
        ax.scatter(0, 0, marker="+", s=160, color=self.C["fail_fg"], zorder=5,
                   label="真值")
        th = np.linspace(0, 2 * np.pi, 100)
        for ns, ls in ((1, "--"), (3, ":")):
            r3 = ns * mc["center_err"]["std"] if False else None
        r1 = ana["sig_center_nm"]
        ax.plot(r1 * np.cos(th), r1 * np.sin(th), color=self.C["cx"], lw=1.0,
                ls="--", label=f"解析1σ={r1:.0f} nm")
        ax.plot(3 * r1 * np.cos(th), 3 * r1 * np.sin(th), color=self.C["cx"],
                lw=1.0, ls=":", label="解析3σ")
        ax.legend(fontsize=8); ax.set_aspect("equal")
        # 右: 残差/RMSE MC分布
        ax2 = fig.add_subplot(122)
        self._ax(ax2, f"中心径向残差 MC分布 (N={mc['runs']})",
                 "径向残差 (nm)", "频次")
        s = mc["center_err"]
        ax2.hist(s["arr"], bins=26, color=self.C["cx"], alpha=0.7,
                 edgecolor="white", lw=0.4)
        ax2.axvline(s["p97_5"], color="#e5a00d", lw=1.4, ls="--",
                    label=f"P97.5={s['p97_5']:.0f} nm")
        ax2.axvline(rc["err_radial_nm"], color=self.C["accent"], lw=1.6,
                    ls="-.", label=f"本次={rc['err_radial_nm']:.0f}")
        ax2.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            f"⑤ 测校残差: 径向 {rc['err_radial_nm']:.0f} nm, 分量 "
            f"({rc['err_nm'][0]:+.0f}, {rc['err_nm'][1]:+.0f}) nm   |   "
            f"解析: 单对 {ana['sig_center_pair_nm']:.0f} → {ana['n_pairs']}对×"
            f"{int(R['params']['repeats'])}重复 → 1σ={ana['sig_center_nm']:.0f} nm   |   "
            f"角度增益估计 {rc['theta_gain']:.6f} (理想1.0)   RMSE={rc['rmse_nm']:.0f} nm"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_pairwise(self, tab, RR):
        R = RR["result"]; rc = R["rc"]; p = R["params"]; ana = RR["ana"]
        fig = Figure(figsize=(10.8, 5.2), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(111)
        self._ax(ax, "逐配对中心估计 (相对真值) — 同步跳动呈圆轨迹",
                 "ΔX (nm)", "ΔY (nm)")
        if len(rc["pw_C"]):
            d = (rc["pw_C"] - rc["C_true"]) * 1e6
            ang = np.rad2deg(rc["pw_ang"])
            order = np.argsort(rc["pw_ang"])
            sc = ax.scatter(d[order, 0], d[order, 1], c=ang[order],
                            cmap="twilight", s=70, edgecolor="white",
                            linewidth=0.6, zorder=4)
            fig.colorbar(sc, ax=ax, label="配对角位 (deg)", shrink=0.85)
            ax.plot(d[order, 0], d[order, 1], color=self.C["dim"], lw=0.7,
                    alpha=0.5, zorder=3)
            # 拟合同步跳动圆
            if len(d) >= 3 and p["runout_sync_nm"] > 0:
                A = np.column_stack([d[:, 0], d[:, 1], np.ones(len(d))])
                b = d[:, 0] ** 2 + d[:, 1] ** 2
                sol, *_ = np.linalg.lstsq(A, b, rcond=None)
                cx_, cy_ = sol[0] / 2, sol[1] / 2
                rr = np.sqrt(sol[2] + cx_ ** 2 + cy_ ** 2)
                th = np.linspace(0, 2 * np.pi, 100)
                ax.plot(cx_ + rr * np.cos(th), cy_ + rr * np.sin(th),
                        color=self.C["accent2"], lw=1.4, ls="--",
                        label=f"拟合同步跳动圆 (R≈{rr:.0f} nm)")
        e = rc["err_nm"]
        ax.scatter(*e, marker="x", s=130, color=self.C["accent"], zorder=5,
                   label="均值解")
        ax.scatter(0, 0, marker="+", s=160, color=self.C["fail_fg"], zorder=5,
                   label="真值")
        ax.legend(fontsize=8); ax.set_aspect("equal")
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        sc = rc["scatter_nm"]
        tk.Label(tab, text=(
            f"逐配对中心散布 = ({sc[0]:.0f}, {sc[1]:.0f}) nm   |   "
            f"同步径向跳动真值 {p['runout_sync_nm']:.0f} nm (相位 "
            f"{p['runout_phase_deg']:.0f}°), NRRO {p['nrro_nm']:.0f} nm   |   "
            f"单对噪声极限 {ana['sig_center_pair_nm']:.0f} nm   →   "
            f"圆轨迹半径≈同步跳动幅值, 可按角度索引补偿; 圆外散布≈噪声+NRRO"),
            bg=self.C["card"], fg=self.C["text"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_sens(self, tab, RR):
        p = RR["result"]["params"]
        fig = Figure(figsize=(10.8, 5.0), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(111)
        self._ax(ax, "转角灵敏度: 中心残差 1σ vs 扫描弧段 (1/sin(θ/2) 律)",
                 "扫描弧段 (deg)", "中心残差 1σ (nm)")
        arcs = np.array([1, 2, 5, 10, 20, 30, 45, 60, 90, 120, 150, 180])
        sig = []
        for a in arcs:
            pp = dict(p, sweep_arc_deg=float(a))
            sig.append(RotCenterModel.analytical(pp)["sig_center_nm"])
        ax.loglog(arcs, sig, "o-", color=self.C["cx"], ms=7, lw=1.4,
                  label="解析 1σ")
        cur = p["sweep_arc_deg"]
        cur_s = RotCenterModel.analytical(p)["sig_center_nm"]
        ax.scatter([cur], [cur_s], s=160, color=self.C["accent"], zorder=5,
                   marker="*", label=f"当前 {cur:.0f}° → {cur_s:.0f} nm")
        ax.axhline(RR["ana"]["sigma_xy_nm"], color=self.C["dim"], lw=1.0,
                   ls=":", label=f"σxy={RR['ana']['sigma_xy_nm']:.0f} nm (噪声基底)")
        ax.legend(fontsize=9); ax.grid(True, which="both", alpha=0.3)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        tk.Label(tab, text=(
            "结论: 中心残差随弧段以 1/sin(θ/2) 下降 — 小角段斜率接近 1/θ (极陡), "
            "大角段趋缓. 从1°增到90°精度提升约百倍; 90°之后收益递减. "
            "工程上取机械行程允许的最大弧段, 通常90°~180°即足够; "
            "继续靠增标记数(∝1/√M)提升远不如增大转角高效"),
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
        txt.insert("end", "Rz旋转中心标定诊断 — 影响测校精度的因素\n\n", "h")
        mark = {"warn": "⚠ ", "ok": "✓ ", "info": "ℹ "}
        for sev, t, de, adv in RR["diag"]:
            txt.insert("end", mark.get(sev, "") + t + "\n", sev)
            txt.insert("end", f"   现象: {de}\n", "dim")
            txt.insert("end", f"   建议: {adv}\n\n")
        txt.config(state="disabled")

    def _tab_accept_c(self, tab, RR):
        ana, mc = RR["ana"], RR["mc"]
        info = tk.Frame(tab, bg=self.C["ref_bg"]); info.pack(fill="x", padx=10,
                                                             pady=(10, 4))
        tk.Label(info, text=(
            f"建议验收区间 (测校残差 = 解算中心−真实中心, MC N={mc['runs']}):\n"
            f"   中心径向残差 ≤ {mc['center_err']['p97_5']:.0f} nm (P97.5; "
            f"解析1σ={ana['sig_center_nm']:.0f} nm, 3σ={ana['three_sigma_center_nm']:.0f})\n"
            f"   分量: X残差 ∈ [{mc['err_x']['p2_5']:.0f}, {mc['err_x']['p97_5']:.0f}] nm, "
            f"Y残差 ∈ [{mc['err_y']['p2_5']:.0f}, {mc['err_y']['p97_5']:.0f}] nm\n"
            f"   角度增益 ∈ [{mc['theta_gain']['p2_5']:.6f}, "
            f"{mc['theta_gain']['p97_5']:.6f}] (理想1.0);  "
            f"RMSE ≤ {mc['rmse']['p97_5']:.0f} nm (P97.5)\n"
            f"判读: 仿真残差可直接对真值; 真机无真值 → 以多次标定的解算中心互差"
            f"(一致性)对应判读, 互差3σ应≤区间宽度; RMSE异常→非刚体/标记误识别"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(10.8, 4.0), dpi=100, facecolor=self.C["bg"])
        specs = [("center_err", "中心径向残差 (nm)", self.C["cx"], False),
                 ("err_x", "X残差 (nm)", self.C["cy"], True),
                 ("rmse", "RMSE (nm)", "#1a7f37", False)]
        for i, (k, ttl, col, zero) in enumerate(specs):
            ax = fig.add_subplot(1, 3, i + 1)
            self._ax(ax, f"MC分布: {ttl}", ttl, "频次")
            s = mc[k]
            ax.hist(s["arr"], bins=24, color=col, alpha=0.7, edgecolor="white",
                    lw=0.4)
            if zero:
                ax.axvspan(s["p2_5"], s["p97_5"], alpha=0.10, color=self.C["ref"])
                for q in ("p2_5", "p97_5"):
                    ax.axvline(s[q], color="#e5a00d", lw=1.1, ls="--")
                ax.axvline(0, color=self.C["dim"], lw=0.8)
            else:
                ax.axvline(s["p97_5"], color="#e5a00d", lw=1.3, ls="--",
                           label=f"P97.5={s['p97_5']:.0f}")
                ax.legend(fontsize=7)
        fig.tight_layout(pad=2.0)
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
        p = R["params"]; rc = R["rc"]
        L = ["=" * 76, f"  {self.title}", "=" * 76,
             f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   "
             f"simcore v{simcore.__version__}",
             f"  Flow: measure P -> rotate theta about TRUE center -> measure P' "
             f"-> SVD inverse -> residual vs truth",
             f"  Layout: {int(p['marks_grid'])}x{int(p['marks_grid'])} marks, "
             f"span {p['marks_span_mm']} mm; K={int(p['num_angles'])} angles / "
             f"{p['sweep_arc_deg']:.0f} deg; m={int(p['frames_per_point'])}, "
             f"R={int(p['repeats'])}",
             f"  Noise: sigma_xy={ana['sigma_xy_nm']:.0f} nm; sync runout "
             f"{p['runout_sync_nm']:.0f} nm; NRRO {p['nrro_nm']:.0f} nm",
             "", "-" * 76, "  RESULT", "-" * 76,
             f"  true center   = ({rc['C_true'][0]:.4f}, {rc['C_true'][1]:.4f}) mm",
             f"  solved center = ({rc['C_hat'][0]:.4f}, {rc['C_hat'][1]:.4f}) mm",
             f"  RESIDUAL: radial = {rc['err_radial_nm']:.1f} nm, components "
             f"({rc['err_nm'][0]:+.1f}, {rc['err_nm'][1]:+.1f}) nm",
             f"  theta gain = {rc['theta_gain']:.6f}; RMSE = {rc['rmse_nm']:.0f} nm; "
             f"per-pair scatter ({rc['scatter_nm'][0]:.0f}, "
             f"{rc['scatter_nm'][1]:.0f}) nm",
             "", "-" * 76, "  ERROR PROPAGATION", "-" * 76,
             f"  amplification 1/(2 sin(theta/2)) = {ana['amp_factor']:.1f}x "
             f"(max pair angle {np.rad2deg(ana['th_max_rad']):.1f} deg)",
             f"  single-pair center noise = {ana['sig_center_pair_nm']:.0f} nm; "
             f"{ana['n_pairs']} pairs x {int(p['repeats'])} rep -> 1sigma "
             f"{ana['sig_center_nm']:.0f} nm",
             "", "-" * 76, f"  ACCEPTANCE REFERENCE (MC, N={mc['runs']})", "-" * 76,
             f"  center radial residual <= {mc['center_err']['p97_5']:.0f} nm (P97.5)",
             f"  X residual [{mc['err_x']['p2_5']:.0f}, {mc['err_x']['p97_5']:.0f}] nm; "
             f"Y [{mc['err_y']['p2_5']:.0f}, {mc['err_y']['p97_5']:.0f}] nm",
             f"  RMSE <= {mc['rmse']['p97_5']:.0f} nm",
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
        """实测数据: angle_deg,mark_id,x_mm,y_mm
        各角位的标记坐标 (mark_id一致, 各角位行序对应同一标记). 以最小角位为零位,
        其余各角位与之配对跑 find_rotation_center."""
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
                raise ValueError("有效数据不足 (每行: angle_deg,mark_id,x_mm,y_mm)")
            recs = np.array(recs)
            angles = np.unique(recs[:, 0])
            mids0 = np.sort(recs[recs[:, 0] == angles[0]][:, 1])
            clouds = {}
            for a in angles:
                sel = recs[recs[:, 0] == a]
                sel = sel[np.argsort(sel[:, 1])]
                if not np.array_equal(np.sort(sel[:, 1]), mids0):
                    raise ValueError(f"角位 {a} 的标记ID与零位不一致")
                clouds[a] = sel[:, 2:4]
            P0 = clouds[angles[0]]
            lines = [f"文件: {path}", f"角位数 K={len(angles)}, 标记数 "
                     f"M={len(mids0)}", "=" * 70]
            cands, pw = [], []
            for a in angles[1:]:
                if abs(np.sin(np.deg2rad(a - angles[0]) / 2)) < np.sin(np.deg2rad(2)):
                    continue
                C_h, th_h = find_rotation_center(P0, clouds[a])
                cands.append(C_h); pw.append((a, C_h, np.rad2deg(th_h)))
            if not cands:
                raise ValueError("无良态配对 (各角位与零位夹角过小)")
            cands = np.array(cands)
            C = cands.mean(axis=0)
            lines += [f"[中心] 解算 Ĉ = ({C[0]:.4f}, {C[1]:.4f}) mm",
                      f"  逐配对散布 = ({cands.std(0)[0]*1e6:.0f}, "
                      f"{cands.std(0)[1]*1e6:.0f}) nm (含同步跳动特征)",
                      f"  良态配对数 = {len(cands)}"]
            lines.append("  逐配对: " + ", ".join(
                f"{a:.0f}°→({c[0]:.4f},{c[1]:.4f})" for a, c, _ in pw[:6]))
            if self.R:
                mc = self.R["mc"]
                lines.append(f"[验收对比] 仿真区间: 中心径向残差 ≤ "
                             f"{mc['center_err']['p97_5']:.0f} nm. "
                             f"真机无真值时以逐配对散布/多次标定互差判读")
            else:
                lines.append("提示: 先运行仿真可获得验收区间对比。")
            win = tk.Toplevel(self.root)
            win.title("实测Rz旋转中心数据分析"); win.geometry("760x420")
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
                f"{e}\n\n格式: angle_deg,mark_id,x_mm,y_mm (表头可选)\n"
                f"各角位的所有标记各占一行; 各角位标记ID需一致, 行序对应同一标记")


if __name__ == "__main__":
    launch(RotCenterApp)
