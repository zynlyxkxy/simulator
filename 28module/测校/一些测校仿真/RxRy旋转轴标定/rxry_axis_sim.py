#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W Rx/Ry旋转轴标定仿真 v2 — 正向Rodrigues生成 / SVD逆解 / 真值对比残差
参考文档: SDS §4.4 | 算法来源: Rx_Ry旋转轴标定.py (3D SVD, 忠实移植)

仿真闭环 (与真实测校流程一一对应):
  ① 输入真实轴线的空间表示: 轴上一点 C=(x,y,z) mm + 方向向量 k=(kx,ky,kz)
     (理想Rx为(1,0,0), 理想Ry为(0,1,0); 分量偏差即轴线安装误差)
  ② 正向: 标记阵列零位真实位置 P, 按 Rodrigues 旋转矩阵绕真实轴线转 Θ(rad)
     → 旋转后真实位置 Q = C + R(k,Θ)·(P−C)
  ③ 测量: P、Q 各叠加测量噪声 — XY由相机+运动台 (σxy), Z由光谱共焦 (σz)
  ④ 逆解: SVD刚体变换 → 旋转矩阵R̂、平移t̂ → 轴方向k̂ (Rodrigues逆) +
     轴上点Ĉ ((I−R̂)C=t̂ 最小二乘)
  ⑤ 残差 = 解算轴 − 真实轴:
       方向残差 = k̂与k的空间夹角 (urad)
       位置残差 = 两轴线在标记区中心截面的垂直偏距 (um)
     这就是该项测校的标定残差, MC给出其验收区间.

影响测校精度的因素 (已识别并代入):
  · 噪声各向异性: σz(共焦200nm) ≫ σxy(26nm) → 方向残差中"面内偏摆"分量
    与位置残差中"横向"分量由共焦决定, 比相机决定的分量差 ~√2·σz/σxy 倍
  · 小转角放大: 全部残差 ∝ 1/Θ (Θ=0.035 rad 时 ≈29×)
  · 轴高分量含 Θ估计误差×力臂(标记面−轴) 耦合 (Θ估计也来自共焦)
  · 沿轴方向位置不可观 (秩亏), 残差按垂直分量评估
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure

import simcore
from simgui import BaseSimApp, launch, FONT


# ===================================================================
#  算法 (忠实移植自 Rx_Ry旋转轴标定.py)
# ===================================================================

def rotvec_from_R(R):
    """Rodrigues逆: 旋转矩阵 → (单位轴, 角度rad)."""
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    sin_t = np.linalg.norm(w) / 2.0
    cos_t = (np.trace(R) - 1.0) / 2.0
    angle = float(np.arctan2(sin_t, cos_t))
    if angle < 1e-9:
        raise ValueError("旋转角度过小, 无法准确计算旋转轴")
    return w / (2.0 * sin_t), angle


def estimate_rotation_axis_svd(points_before, points_after):
    """SVD求刚体变换并提取旋转轴 (与上传脚本逐步对应).
    返回 (axis_point, axis_direction, angle_rad, rmse)."""
    P = np.asarray(points_before, float)
    Q = np.asarray(points_after, float)
    if len(P) != len(Q):
        raise ValueError("前后点对数量不一致")
    cP, cQ = P.mean(axis=0), Q.mean(axis=0)
    H = (P - cP).T @ (Q - cQ)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    R = V @ U.T
    if np.linalg.det(R) < 0:
        V[:, 2] *= -1
        R = V @ U.T
    t = cQ - R @ cP
    rmse = float(np.sqrt(np.mean(np.sum((P @ R.T + t - Q) ** 2, axis=1))))
    axis_dir, angle = rotvec_from_R(R)
    axis_point, *_ = np.linalg.lstsq(np.eye(3) - R, t, rcond=None)
    return axis_point, axis_dir, angle, rmse


def rodrigues(k, theta):
    """正向Rodrigues: 绕单位轴k转theta(rad)的旋转矩阵."""
    k = np.asarray(k, float); k = k / np.linalg.norm(k)
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * Kx + (1 - np.cos(theta)) * (Kx @ Kx)


def axis_section(C, k, which):
    """轴线与标记区中心截面 (Rx: x=0面 / Ry: y=0面) 的交点 → 截面内坐标对."""
    i = 0 if which == "rx" else 1
    s0 = -C[i] / k[i]
    pt = C + s0 * k
    return pt[[1, 2]] if which == "rx" else pt[[0, 2]]


# ===================================================================
#  模型
# ===================================================================

class RxRyAxisModel:
    """单位: 位置 mm, 角度 rad (残差显示 urad/um), 噪声 nm.
    Rx与Ry独立采集独立标定."""

    # ---------- ① 真实轴线 (用户输入的空间表示) ----------
    @staticmethod
    def true_axis(p, which):
        if which == "rx":
            C = np.array([p["rx_px_mm"], p["rx_py_mm"], p["rx_pz_mm"]])
            k = np.array([p["rx_kx"], p["rx_ky"], p["rx_kz"]])
        else:
            C = np.array([p["ry_px_mm"], p["ry_py_mm"], p["ry_pz_mm"]])
            k = np.array([p["ry_kx"], p["ry_ky"], p["ry_kz"]])
        n = np.linalg.norm(k)
        if n < 1e-9:
            raise ValueError(f"{which.upper()} 方向向量为零")
        return k / n, C

    @staticmethod
    def marks(p):
        """标记阵列: G×G 网格, 共面 z=h."""
        G = max(2, int(p["marks_grid"]))
        s = p["marks_span_mm"] / 2.0
        ax_ = np.linspace(-s, s, G)
        gx, gy = np.meshgrid(ax_, ax_, indexing="ij")
        return np.column_stack([gx.ravel(), gy.ravel(),
                                np.full(gx.size, p["wafer_z_mm"])])

    @staticmethod
    def _noise(p, rng, n):
        """③ 测量噪声 (mm): XY=相机+台合成, Z=光谱共焦."""
        e = np.empty((n, 3))
        e[:, 0] = rng.normal(0, p["sigma_xy_nm"] * 1e-6, n)
        e[:, 1] = rng.normal(0, p["sigma_xy_nm"] * 1e-6, n)
        e[:, 2] = rng.normal(0, p["sigma_z_nm"] * 1e-6, n)
        return e

    # ---------- 单轴一次标定 (R次重复取均值) ----------
    @staticmethod
    def calib_axis(p, which, rng):
        k_t, C_t = RxRyAxisModel.true_axis(p, which)
        theta = p["theta_rx_rad" if which == "rx" else "theta_ry_rad"]
        Rk = rodrigues(k_t, theta)                      # ② 正向Rodrigues
        P0 = RxRyAxisModel.marks(p)
        Q0 = C_t + (P0 - C_t) @ Rk.T
        reps = max(1, int(p["repeats"]))
        ks, Cs, angs, rmses = [], [], [], []
        last = {}
        for _ in range(reps):
            Pm = P0 + RxRyAxisModel._noise(p, rng, len(P0))   # ③ 测量
            Qm = Q0 + RxRyAxisModel._noise(p, rng, len(P0))
            C_h, k_h, ang, rmse = estimate_rotation_axis_svd(Pm, Qm)  # ④ 逆解
            if np.dot(k_h, k_t) < 0:
                k_h = -k_h
            ks.append(k_h); Cs.append(C_h)
            angs.append(abs(ang)); rmses.append(rmse)
            last = {"Pm": Pm, "Qm": Qm}
        k_h = np.mean(ks, axis=0); k_h /= np.linalg.norm(k_h)
        # ⑤ 残差: 解算轴 vs 真实轴
        dir_err = float(np.arccos(np.clip(np.dot(k_h, k_t), -1, 1))) * 1e6  # urad
        sec_t = axis_section(C_t, k_t, which)
        secs = np.array([axis_section(C, kk, which) for C, kk in zip(Cs, ks)])
        sec_h = secs.mean(axis=0)
        d_sec = (sec_h - sec_t) * 1e3                                       # um
        pos_err = float(np.linalg.norm(d_sec))
        # 分量分解 (各向异性分析用)
        if which == "rx":
            psi_e = (np.arctan2(k_h[1], k_h[0])
                     - np.arctan2(k_t[1], k_t[0])) * 1e6
        else:
            psi_e = (np.arctan2(k_h[0], k_h[1])
                     - np.arctan2(k_t[0], k_t[1])) * 1e6
        bet_e = (np.arcsin(np.clip(k_h[2], -1, 1))
                 - np.arcsin(np.clip(k_t[2], -1, 1))) * 1e6
        return {"which": which, "theta_rad": theta,
                "k_true": k_t, "C_true": C_t, "k_hat": k_h,
                "C_hat": np.mean(Cs, axis=0),
                "dir_err_urad": dir_err, "pos_err_um": pos_err,
                "comp": np.array([psi_e, bet_e, d_sec[0], d_sec[1]]),
                "sec_true_mm": sec_t, "sec_hat_mm": sec_h,
                "angle_meas_rad": float(np.mean(angs)),
                "rmse_nm": float(np.mean(rmses) * 1e6),
                "P0": P0, "Q0_true": Q0, **last}

    @staticmethod
    def run(p, seed=None):
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))
        return {"params": dict(p),
                "rx": RxRyAxisModel.calib_axis(p, "rx", rng),
                "ry": RxRyAxisModel.calib_axis(p, "ry", rng)}

    # ---------- 解析误差传播 ----------
    @staticmethod
    def analytical(p):
        G = max(2, int(p["marks_grid"]))
        M = G * G
        R = max(1, int(p["repeats"]))
        sxy, sz = p["sigma_xy_nm"], p["sigma_z_nm"]
        s = p["marks_span_mm"] / 2.0
        ax_ = np.linspace(-s, s, G)
        Sxx = float(M / G * np.sum((ax_ - ax_.mean()) ** 2))
        out = {"sigma_xy_nm": sxy, "sigma_z_nm": sz, "M": M, "Sxx_mm2": Sxx}
        for which in ("rx", "ry"):
            th = max(p["theta_rx_rad" if which == "rx" else "theta_ry_rad"], 1e-9)
            f = 1.0 / (th * np.sqrt(R))
            sig_yaw = f * np.sqrt(2.0) * sz * 1e-6 / np.sqrt(Sxx) * 1e6   # urad
            sig_pitch = f * sxy * 1e-6 / np.sqrt(Sxx) * 1e6
            sig_lat = f * np.sqrt(2.0) * sz * 1e-6 / np.sqrt(M) * 1e3     # um
            pz = p["rx_pz_mm" if which == "rx" else "ry_pz_mm"]
            lever = abs(p["wafer_z_mm"] - pz)
            sig_th = np.sqrt(2.0) * sz * 1e-6 / np.sqrt(Sxx)
            sig_z = f * np.sqrt(2.0 * (sxy * 1e-6) ** 2 / M
                                + (lever * sig_th) ** 2) * 1e3
            out[which] = {"sig_yaw_urad": sig_yaw, "sig_pitch_urad": sig_pitch,
                          "sig_lat_um": sig_lat, "sig_z_um": sig_z,
                          "sig_dir_urad": float(np.hypot(sig_yaw, sig_pitch)),
                          "sig_pos_um": float(np.hypot(sig_lat, sig_z)),
                          "lever_mm": lever, "theta_rad": th,
                          "amp_1_over_theta": 1.0 / th}
        return out

    # ---------- 蒙特卡洛 ----------
    @staticmethod
    def monte_carlo(p, runs=200, seed0=50000):
        keys = ("yaw", "pitch", "lat", "z")
        acc = {w: {k: np.empty(runs) for k in keys} for w in ("rx", "ry")}
        de = {w: np.empty(runs) for w in ("rx", "ry")}
        pe = {w: np.empty(runs) for w in ("rx", "ry")}
        rm = {w: np.empty(runs) for w in ("rx", "ry")}
        for i in range(runs):
            r = RxRyAxisModel.run(p, seed=seed0 + i)
            for w in ("rx", "ry"):
                for j, k in enumerate(keys):
                    acc[w][k][i] = r[w]["comp"][j]
                de[w][i] = r[w]["dir_err_urad"]
                pe[w][i] = r[w]["pos_err_um"]
                rm[w][i] = r[w]["rmse_nm"]
        def st(v):
            return {"mean": float(np.mean(v)), "std": float(np.std(v, ddof=1)),
                    "p2_5": float(np.percentile(v, 2.5)),
                    "p97_5": float(np.percentile(v, 97.5)), "arr": v}
        return {"runs": runs,
                **{w: {**{k: st(acc[w][k]) for k in keys},
                       "dir": st(de[w]), "pos": st(pe[w]), "rmse": st(rm[w])}
                   for w in ("rx", "ry")}}

    # ---------- 诊断 ----------
    @staticmethod
    def diagnose(p, R, ana, mc):
        f = []
        ratio = ana["sigma_z_nm"] / max(ana["sigma_xy_nm"], 1e-9)
        f.append(("info", "影响因素①: 测量噪声各向异性 (共焦 vs 相机)",
                  f"σz(共焦)={ana['sigma_z_nm']:.0f} nm vs σxy(相机+台)="
                  f"{ana['sigma_xy_nm']:.0f} nm ≈ {ratio:.1f}×. "
                  f"方向残差中'面内偏摆'与位置残差中'横向'分量由共焦决定",
                  "提升精度的第一杠杆是Z测量: 共焦多次平均/更高档传感器/"
                  "改善标记面反射一致性"))
        for w, nm in (("rx", "Rx"), ("ry", "Ry")):
            th = R[w]["theta_rad"]
            if th < 0.026:
                f.append(("warn", f"影响因素②: {nm} 标定转角过小 "
                                  f"(Θ={th:.4f} rad)",
                          f"全部残差按 1/Θ 放大, 当前 ≈{1/th:.0f}×",
                          "在机械行程允许范围内尽量取大Θ; Θ加倍残差减半"))
            r_ = mc[w]["dir"]["mean"] / max(ana[w]["sig_dir_urad"] * 1.25, 1e-12)
            if r_ > 1.4:
                f.append(("warn", f"{nm}: 方向残差显著大于噪声极限 (×{r_:.2f})",
                          "通常为面形/夹持变形等非高斯成分或非刚体运动 "
                          "(RMSE同时升高)",
                          "核对RMSE与真空吸附状态; 排除标记误识别"))
        for w, nm in (("rx", "Rx"), ("ry", "Ry")):
            f.append(("info", f"影响因素③: {nm} 轴高残差的Θ耦合",
                      f"力臂(标记面−轴高)={ana[w]['lever_mm']:.1f} mm: Θ估计误差"
                      f"(来自共焦斜率)经力臂放大进入轴高分量",
                      "标记面尽量靠近轴高度可减小该耦合"))
        r_wk = p["marks_span_mm"] / 2.0
        for w, nm in (("rx", "Rx"), ("ry", "Ry")):
            eq = (mc[w]["dir"]["p97_5"] * 1e-6 * 1e-3 * r_wk * 1e9 * 1e-3
                  + mc[w]["pos"]["p97_5"])                      # um, 1mrad工作角下
            f.append(("info", f"{nm}: 残差对调平的影响量级",
                      f"方向残差P97.5={mc[w]['dir']['p97_5']:.1f} urad, "
                      f"位置残差P97.5={mc[w]['pos']['p97_5']:.2f} um "
                      f"(评估半径取标记半展布 {r_wk:.0f} mm)",
                      "按实际工作转角×半径换算到XY/Z误差链核对预算"))
        return f


# ===================================================================
#  GUI
# ===================================================================

class RxRyAxisApp(BaseSimApp):
    title = "D2W - Rx/Ry旋转轴标定仿真 (正向Rodrigues / SVD逆解 / 真值对比残差)"
    subtitle = ("Rx/Ry轴标定: ①零位测标记 → ②绕真实轴转Θ → ③重测 → "
                "④SVD逆解轴线 → ⑤与真实轴对比 = 测校残差")
    report_name = "rxry_axis"
    model = RxRyAxisModel

    def schema(self):
        amp_fn = lambda p: (f"{1/max(p['theta_rx_rad'],1e-9):.0f} / "
                            f"{1/max(p['theta_ry_rad'],1e-9):.0f}")
        return [
            ("section", "① 标记阵列 (零位先测一遍空间位置)"),
            ("entry", "marks_grid", "标记网格 G×G", 3, "G"),
            ("entry", "marks_span_mm", "标记展布 (边长)", 20.0, "mm"),
            ("entry", "wafer_z_mm", "标记面高度 z", 5.0, "mm"),
            ("note", "XY由相机+运动台测量, Z由光谱共焦测量 — 每个标记得到一个"
                     "三维空间坐标 (x,y,z)"),

            ("section", "② 旋转指令 (角度单位: rad)"),
            ("entry", "theta_rx_rad", "Rx 标定转角 Θ", 0.035, "rad"),
            ("entry", "theta_ry_rad", "Ry 标定转角 Θ", 0.035, "rad"),
            ("derived", "_amp", "1/Θ 残差放大倍数 Rx/Ry (自动)", "×", amp_fn,
             ["theta_rx_rad", "theta_ry_rad"]),
            ("entry", "repeats", "整套流程重复次数 R", 3, "次"),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 200, "次"),
            ("note", "残差 ∝ 1/Θ: 转角越小放大越狠, 行程允许时尽量取大"),

            ("section", "真实 Rx 轴线 (已知真值, 仿真用它做正向Rodrigues)"),
            ("entry", "rx_px_mm", "轴上一点 x", 0.0, "mm"),
            ("entry", "rx_py_mm", "轴上一点 y", 0.08, "mm"),
            ("entry", "rx_pz_mm", "轴上一点 z", -0.05, "mm"),
            ("entry", "rx_kx", "方向向量 kx", 1.0, ""),
            ("entry", "rx_ky", "方向向量 ky", 0.0003, ""),
            ("entry", "rx_kz", "方向向量 kz", -0.0002, ""),
            ("note", "理想Rx方向为(1,0,0). ky/kz的小分量即轴线安装偏差 "
                     "(0.0003≈300urad面内偏摆); 向量自动归一化"),

            ("section", "真实 Ry 轴线 (已知真值)"),
            ("entry", "ry_px_mm", "轴上一点 x", -0.06, "mm"),
            ("entry", "ry_py_mm", "轴上一点 y", 0.0, "mm"),
            ("entry", "ry_pz_mm", "轴上一点 z", 0.04, "mm"),
            ("entry", "ry_kx", "方向向量 kx", -0.00025, ""),
            ("entry", "ry_ky", "方向向量 ky", 1.0, ""),
            ("entry", "ry_kz", "方向向量 kz", 0.00015, ""),
            ("note", "理想Ry方向为(0,1,0)"),

            ("section", "③ 测量噪声 (影响测校精度的因素)"),
            ("entry", "sigma_xy_nm", "XY 单点测量 1σ (相机+台)", 26.0, "nm"),
            ("entry", "sigma_z_nm", "Z 单点测量 1σ (光谱共焦)", 200.0, "nm"),
            ("note", "σz≫σxy 的各向异性是本项标定的核心: 旋转信号主要在Z向, "
                     "恰落在最差测量轴. 各分量受哪个噪声决定见[噪声各向异性]页"),
        ]

    # ---------------- run ----------------
    def _run(self):
        from tkinter import messagebox
        try:
            p = self.collect()
            RxRyAxisModel.true_axis(p, "rx")
            RxRyAxisModel.true_axis(p, "ry")
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        R = RxRyAxisModel.run(p)
        ana = RxRyAxisModel.analytical(p)
        mc = RxRyAxisModel.monte_carlo(p, runs=max(50, int(p.get("mc_runs") or 200)))
        diag = RxRyAxisModel.diagnose(p, R, ana, mc)
        self.R = {"result": R, "ana": ana, "mc": mc, "diag": diag}
        self._status_calib()
        self._render_all()

    def _status_calib(self):
        R, mc = self.R["result"], self.R["mc"]
        ok = all(R[w]["dir_err_urad"] <= mc[w]["dir"]["p97_5"] * 1.0001
                 and R[w]["pos_err_um"] <= mc[w]["pos"]["p97_5"] * 1.0001
                 for w in ("rx", "ry"))
        bg = self.C["pass_bg"] if ok else self.C["ref_bg"]
        fg = self.C["pass_fg"] if ok else self.C["warn"]
        tag = "正常·位于验收区间" if ok else "超出验收区间 → 见诊断页"
        rx, ry = R["rx"], R["ry"]
        self.sfr.config(bg=bg)
        self.sl.config(bg=bg, fg=fg, font=(FONT, 11, "bold"), text=(
            f"[{tag}]  测校残差 — Rx: 方向{rx['dir_err_urad']:.1f} urad / "
            f"位置{rx['pos_err_um']:.2f} um   Ry: 方向{ry['dir_err_urad']:.1f} "
            f"urad / 位置{ry['pos_err_um']:.2f} um  | 运行#{self.sim_n}"))

    # ---------------- tabs ----------------
    def standard_tabs(self):
        return [("流程与布置", self._tab_layout),
                ("Rx 轴标定", lambda t, R: self._tab_axis(t, R, "rx")),
                ("Ry 轴标定", lambda t, R: self._tab_axis(t, R, "ry")),
                ("噪声各向异性", self._tab_aniso),
                ("诊断与建议", self._tab_diag_c),
                ("验收参考", self._tab_accept_c)]

    def _tab_layout(self, tab, RR):
        R = RR["result"]; p = R["params"]
        head = tk.Frame(tab, bg=self.C["ref_bg"]); head.pack(fill="x", padx=10,
                                                             pady=(8, 2))
        tk.Label(head, text=(
            "仿真闭环 = 真实测校流程:  ① 零位测标记空间位置 P (XY相机 / Z共焦)"
            "  →  ② 绕[真实轴线](用户输入的 点+方向向量)按Rodrigues矩阵转Θ, "
            "生成旋转后真实位置  →  ③ 重测得 Q (前后均含噪声)  →  "
            "④ SVD逆解: R̂,t̂ → 轴方向k̂(Rodrigues逆) + 轴上点Ĉ((I-R̂)C=t̂)  →  "
            "⑤ 残差 = 解算轴-真实轴 (空间夹角 + 截面垂直偏距) = 该项测校残差"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", wraplength=1020,
            padx=10, pady=6).pack(fill="x")
        fig = Figure(figsize=(10.8, 5.0), dpi=100, facecolor=self.C["bg"])
        for i, (w, nm, col) in enumerate((("rx", "Rx", self.C["cx"]),
                                          ("ry", "Ry", self.C["cy"]))):
            d = R[w]
            ax = fig.add_subplot(1, 2, i + 1, projection="3d")
            P0, Q0 = d["P0"], d["Q0_true"]
            dev = Q0 - P0
            exa = max(1, int(round(0.3 * p["marks_span_mm"]
                                   / max(np.max(np.abs(dev)), 1e-9))))
            Qv = P0 + dev * exa
            ax.set_title(f"{nm}: 旋转前后标记位置 (Θ={d['theta_rad']:.4f} rad, "
                         f"位移放大×{exa})",
                         color=self.C["text"], fontsize=10, fontweight="bold")
            ax.scatter(P0[:, 0], P0[:, 1], P0[:, 2], s=34, color=self.C["dim"],
                       depthshade=False, label="零位 P")
            ax.scatter(Qv[:, 0], Qv[:, 1], Qv[:, 2], s=34, color=col,
                       depthshade=False, label=f"旋转后 Q (位移×{exa})")
            for a_, b_ in zip(P0, Qv):
                ax.plot([a_[0], b_[0]], [a_[1], b_[1]], [a_[2], b_[2]],
                        color=col, lw=0.9, alpha=0.7)
            k_t, C_t = d["k_true"], d["C_true"]
            s_ = p["marks_span_mm"] * 0.75
            ln = np.array([C_t - s_ * k_t, C_t + s_ * k_t])
            ax.plot(ln[:, 0], ln[:, 1], ln[:, 2], color=self.C["fail_fg"],
                    lw=2.2, label=f"真实{nm}轴线")
            ax.set_xlabel("X (mm)", fontsize=8)
            ax.set_ylabel("Y (mm)", fontsize=8)
            ax.set_zlabel("Z (mm)", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.legend(fontsize=7, loc="upper left")
        fig.tight_layout(pad=1.6)
        self._embed(fig, tab)

    def _tab_axis(self, tab, RR, which):
        R = RR["result"]; ana = RR["ana"]; mc = RR["mc"]
        d = R[which]; nm = which.upper()
        lat_lbl = "y" if which == "rx" else "x"
        fig = Figure(figsize=(11.0, 5.4), dpi=100, facecolor=self.C["bg"])
        # 左: 测量信号 — 共焦读到的ΔZ vs 杠杆坐标
        ax = fig.add_subplot(121)
        P0 = d["P0"]
        dz = (d["Qm"][:, 2] - d["Pm"][:, 2]) * 1e3                  # um
        lever = P0[:, 1] if which == "rx" else P0[:, 0]
        self._ax(ax, f"{nm}: 测量信号 — 共焦ΔZ vs 杠杆坐标 (ΔZ≈Θ×力臂)",
                 f"{lat_lbl} (mm)", "ΔZ (um)")
        ax.plot(lever, dz, "o", color=self.C["cx"], ms=7,
                label="实测 (含σz噪声)")
        kk = np.polyfit(lever, dz, 1)
        xx = np.array([lever.min(), lever.max()])
        ax.plot(xx, np.polyval(kk, xx), color=self.C["fail_fg"], lw=1.5,
                ls="--", label=f"斜率 ≈ Θ = {abs(kk[0])*1e-3:.5f} rad")
        x0 = -kk[1] / kk[0] if abs(kk[0]) > 1e-12 else 0.0
        ax.axvline(x0, color=self.C["accent2"], lw=1.2, ls=":",
                   label=f"零交点 ≈ 轴{lat_lbl}位置 = {x0:.4f} mm")
        ax.axhline(0, color=self.C["dim"], lw=0.6)
        ax.legend(fontsize=8)
        # 右: 残差分量 归一化 (误差/3σ)
        ax2 = fig.add_subplot(122)
        self._ax(ax2, f"{nm} 残差分量 (归一化: 误差/3σ, ±1带内=正常)",
                 "误差 / 3σ", "")
        rows = [("方向: 面内偏摆", d["comp"][0], 3 * mc[which]["yaw"]["std"],
                 "urad", "#b45309", "共焦Z决定"),
                ("方向: 俯仰", d["comp"][1], 3 * mc[which]["pitch"]["std"],
                 "urad", self.C["cx"], "相机XY决定"),
                (f"位置: {lat_lbl}向", d["comp"][2], 3 * mc[which]["lat"]["std"],
                 "um", "#b45309", "共焦Z决定"),
                ("位置: z向", d["comp"][3], 3 * mc[which]["z"]["std"],
                 "um", self.C["cx"], "相机+Θ耦合")]
        yp = np.arange(len(rows))[::-1]
        ax2.axvspan(-1, 1, alpha=0.10, color=self.C["pass_fg"], zorder=1)
        for sgn in (1, -1):
            ax2.axvline(sgn, color=self.C["pass_fg"], lw=1.0, ls="--", alpha=0.6)
        labels = []
        for y_, (lbl, err, s3, unit, col, src) in zip(yp, rows):
            nrm = err / max(s3, 1e-12)
            ax2.barh(y_, nrm, height=0.46, color=col, alpha=0.85, zorder=3)
            ax2.text(nrm, y_ + 0.32, f"{err:+.2f} {unit}", fontsize=9,
                     color=col, ha="center", fontweight="bold")
            labels.append(f"{lbl} [{src}]\n3σ={s3:.2f} {unit}")
        ax2.set_yticks(yp); ax2.set_yticklabels(labels, fontsize=8.5)
        ax2.axvline(0, color=self.C["dim"], lw=1.0)
        ax2.set_xlim(-1.7, 1.7)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        kt, kh = d["k_true"], d["k_hat"]
        st, sh = d["sec_true_mm"], d["sec_hat_mm"]
        a = ana[which]
        tk.Label(tab, text=(
            f"⑤ 测校残差: 方向夹角 = {d['dir_err_urad']:.2f} urad "
            f"(解析1σ {a['sig_dir_urad']:.1f}),  截面垂直偏距 = "
            f"{d['pos_err_um']:.3f} um (解析1σ {a['sig_pos_um']:.2f})   |   "
            f"测得转角 {d['angle_meas_rad']:.6f} rad (指令 "
            f"{d['theta_rad']:.4f})   RMSE={d['rmse_nm']:.0f} nm\n"
            f"真实轴: k=({kt[0]:+.6f},{kt[1]:+.6f},{kt[2]:+.6f}),  截面点"
            f"({lat_lbl},z)=({st[0]:+.4f},{st[1]:+.4f}) mm     解算轴: "
            f"k=({kh[0]:+.6f},{kh[1]:+.6f},{kh[2]:+.6f}),  截面点="
            f"({sh[0]:+.4f},{sh[1]:+.4f}) mm"),
            bg=self.C["card"], fg=self.C["text"], font=("Consolas", 9),
            justify="left", anchor="w", padx=10, pady=8).pack(
            fill="x", padx=8, pady=(0, 8))

    def _tab_aniso(self, tab, RR):
        ana, mc = RR["ana"], RR["mc"]
        fig = Figure(figsize=(10.8, 5.0), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(121)
        self._ax(ax, "残差分量按数据来源分裂 (3σ, 对数轴)", "", "3σ")
        names = ["面内偏摆\n(urad)", "俯仰\n(urad)", "横向位置\n(um)",
                 "轴高z\n(um)"]
        cz = "#b45309"; cc = self.C["cx"]
        cols = [cz, cc, cz, cc]
        for i, w in enumerate(("rx", "ry")):
            vals = [3 * mc[w][k]["std"] for k in ("yaw", "pitch", "lat", "z")]
            xp = np.arange(4) + (i - 0.5) * 0.36
            ax.bar(xp, vals, 0.34, color=cols,
                   alpha=0.95 if w == "rx" else 0.45,
                   edgecolor="white", lw=0.5)
            for x_, v in zip(xp, vals):
                ax.text(x_, v, f"{v:.2f}", ha="center", va="bottom",
                        fontsize=7.5)
        ax.set_xticks(np.arange(4)); ax.set_xticklabels(names, fontsize=8.5)
        ax.set_yscale("log")
        from matplotlib.patches import Patch
        ax.legend(handles=[Patch(color=cz, label="共焦Z决定 (σz=200nm级)"),
                           Patch(color=cc, label="相机XY决定 (σxy=26nm级)"),
                           Patch(color="#999", alpha=0.95, label="深色=Rx"),
                           Patch(color="#999", alpha=0.45, label="浅色=Ry")],
                  fontsize=8)
        ax2 = fig.add_subplot(122)
        self._ax(ax2, "解析 vs MC 交叉校验 (1σ比值, 理想=1)", "", "MC/解析")
        labels2, vals2, cols2 = [], [], []
        for w in ("rx", "ry"):
            for k, akey, col in (("yaw", "sig_yaw_urad", cz),
                                 ("pitch", "sig_pitch_urad", cc),
                                 ("lat", "sig_lat_um", cz),
                                 ("z", "sig_z_um", cc)):
                labels2.append(f"{w.upper()}\n{k}")
                vals2.append(mc[w][k]["std"] / max(ana[w][akey], 1e-12))
                cols2.append(col)
        xp = np.arange(len(vals2))
        ax2.bar(xp, vals2, 0.62, color=cols2, alpha=0.8)
        ax2.axhline(1.0, color=self.C["pass_fg"], lw=1.4, ls="--")
        ax2.set_xticks(xp); ax2.set_xticklabels(labels2, fontsize=7.5)
        ax2.set_ylim(0, max(vals2) * 1.3)
        for x_, v in zip(xp, vals2):
            ax2.text(x_, v, f"{v:.2f}", ha="center", va="bottom", fontsize=7.5)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)
        rat = ana["sigma_z_nm"] / max(ana["sigma_xy_nm"], 1e-9)
        tk.Label(tab, text=(
            f"σz/σxy = {ana['sigma_z_nm']:.0f}/{ana['sigma_xy_nm']:.0f} ≈ "
            f"{rat:.1f}× → 共焦决定的分量比相机决定的分量差约 √2×该比值 "
            f"(俯仰有双相机观测通道, 方差减半). 提升路径: 改善Z测量(第一杠杆)、"
            f"增大Θ(全分量∝1/Θ)、增大标记展布(方向分量)、增加重复次数R(全分量∝1/√R)"),
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
        txt.insert("end", "Rx/Ry轴标定诊断 — 影响测校精度的因素\n\n", "h")
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
        def line(w):
            nm = w.upper()
            return (f"   {nm}: 方向残差 ≤ {mc[w]['dir']['p97_5']:.1f} urad "
                    f"(P97.5; 解析1σ={ana[w]['sig_dir_urad']:.1f}),  "
                    f"位置残差 ≤ {mc[w]['pos']['p97_5']:.2f} um "
                    f"(解析1σ={ana[w]['sig_pos_um']:.2f}),  "
                    f"RMSE ≤ {mc[w]['rmse']['p97_5']:.0f} nm")
        tk.Label(info, text=(
            f"建议验收区间 (测校残差 = 解算轴−真实轴, MC N={mc['runs']}):\n"
            + line("rx") + "\n" + line("ry") + "\n"
            f"判读: 仿真中残差可直接对真值; 真机无真值 → 以多次标定的解算轴"
            f"互差(一致性)对应判读, 互差3σ应≤上述区间宽度; "
            f"RMSE异常大 → 非刚体运动/夹持滑移/标记误识别 (脚本原始判据 >10um报警)"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(10.8, 5.6), dpi=100, facecolor=self.C["bg"])
        specs = []
        for w in ("rx", "ry"):
            nm = w.upper()
            specs += [(w, "dir", f"{nm} 方向残差 (urad)", "#b45309"),
                      (w, "pos", f"{nm} 位置残差 (um)", self.C["cx"]),
                      (w, "rmse", f"{nm} RMSE (nm)", self.C["cy"])]
        for i, (w, k, ttl, col) in enumerate(specs):
            ax = fig.add_subplot(2, 3, i + 1)
            self._ax(ax, ttl, "", "")
            s = mc[w][k]
            ax.hist(s["arr"], bins=22, color=col, alpha=0.7,
                    edgecolor="white", lw=0.4)
            ax.axvline(s["p97_5"], color="#e5a00d", lw=1.3, ls="--",
                       label=f"P97.5={s['p97_5']:.2f}")
            ax.legend(fontsize=7)
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
        p = R["params"]
        L = ["=" * 76, f"  {self.title}", "=" * 76,
             f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   "
             f"simcore v{simcore.__version__}",
             f"  Flow: measure P -> rotate Theta (Rodrigues about TRUE axis) -> "
             f"measure Q -> SVD inverse -> residual vs truth",
             f"  Layout: {int(p['marks_grid'])}x{int(p['marks_grid'])} marks, "
             f"span {p['marks_span_mm']} mm, z={p['wafer_z_mm']} mm; "
             f"Theta Rx/Ry = {p['theta_rx_rad']}/{p['theta_ry_rad']} rad; "
             f"R={int(p['repeats'])}",
             f"  Noise: sigma_xy={p['sigma_xy_nm']:.0f} nm, "
             f"sigma_z={p['sigma_z_nm']:.0f} nm (confocal)"]
        for w in ("rx", "ry"):
            d = R[w]; nm = w.upper()
            kt, kh = d["k_true"], d["k_hat"]
            L += ["", "-" * 76, f"  {nm} AXIS", "-" * 76,
                  f"  true axis:   k=({kt[0]:+.6f},{kt[1]:+.6f},{kt[2]:+.6f})  "
                  f"point=({d['C_true'][0]:+.4f},{d['C_true'][1]:+.4f},"
                  f"{d['C_true'][2]:+.4f}) mm",
                  f"  solved axis: k=({kh[0]:+.6f},{kh[1]:+.6f},{kh[2]:+.6f})  "
                  f"section({'y,z' if w=='rx' else 'x,z'})="
                  f"({d['sec_hat_mm'][0]:+.4f},{d['sec_hat_mm'][1]:+.4f}) mm",
                  f"  RESIDUAL: direction = {d['dir_err_urad']:.2f} urad, "
                  f"position = {d['pos_err_um']:.3f} um;  measured angle "
                  f"{d['angle_meas_rad']:.6f} rad (cmd {d['theta_rad']:.4f});  "
                  f"RMSE = {d['rmse_nm']:.0f} nm",
                  f"  components (err / 3sigma): yaw {d['comp'][0]:+.2f}/"
                  f"{3*mc[w]['yaw']['std']:.2f} urad, pitch {d['comp'][1]:+.2f}/"
                  f"{3*mc[w]['pitch']['std']:.2f} urad, lat {d['comp'][2]:+.3f}/"
                  f"{3*mc[w]['lat']['std']:.2f} um, z {d['comp'][3]:+.3f}/"
                  f"{3*mc[w]['z']['std']:.2f} um"]
        L += ["", "-" * 76, f"  ACCEPTANCE REFERENCE (MC, N={mc['runs']})",
              "-" * 76]
        for w in ("rx", "ry"):
            L.append(f"  {w.upper()}: direction <= {mc[w]['dir']['p97_5']:.1f} "
                     f"urad, position <= {mc[w]['pos']['p97_5']:.2f} um, "
                     f"RMSE <= {mc[w]['rmse']['p97_5']:.0f} nm "
                     f"(analytic 1sigma: dir {ana[w]['sig_dir_urad']:.1f} urad, "
                     f"pos {ana[w]['sig_pos_um']:.2f} um)")
        L += ["", "-" * 76, "  DIAGNOSIS", "-" * 76]
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
        """实测数据: axis,phase,x_mm,y_mm,z_mm (axis=RX/RY, phase=0零位/1旋转后,
        两相行序对应同一标记). 跑同一套 estimate_rotation_axis_svd."""
        from tkinter import filedialog, messagebox, ttk
        import csv
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"),
                                                     ("All", "*.*")])
        if not path:
            return
        try:
            recs = {"RX": {0: [], 1: []}, "RY": {0: [], 1: []}}
            with open(path, "r", encoding="utf-8-sig") as fh:
                for row in csv.reader(fh):
                    cells = [c.strip() for c in row if c.strip() != ""]
                    if len(cells) < 5 or cells[0].upper() not in ("RX", "RY"):
                        continue
                    recs[cells[0].upper()][int(float(cells[1]))].append(
                        [float(c) for c in cells[2:5]])
            lines = [f"文件: {path}", "=" * 70]
            any_ok = False
            for w in ("RX", "RY"):
                P, Q = recs[w][0], recs[w][1]
                if len(P) >= 3 and len(P) == len(Q):
                    C_h, k_h, ang, rmse = estimate_rotation_axis_svd(P, Q)
                    any_ok = True
                    sec = axis_section(np.asarray(C_h), np.asarray(k_h),
                                       w.lower())
                    lines += [f"[{w}] M={len(P)}, 测得转角 = {abs(ang):.6f} rad",
                              f"  解算轴方向 k = ({k_h[0]:+.6f}, {k_h[1]:+.6f}, "
                              f"{k_h[2]:+.6f})",
                              f"  截面交点 ({'y,z' if w=='RX' else 'x,z'}) = "
                              f"({sec[0]:+.4f}, {sec[1]:+.4f}) mm",
                              f"  RMSE = {rmse*1e6:.0f} nm "
                              f"{'(>10um, 警告: 可能非刚体运动)' if rmse > 0.01 else ''}"]
            if not any_ok:
                raise ValueError("未找到成对的 RX/RY 数据 (每轴需 phase=0 与 "
                                 "phase=1 各≥3行, 行序对应同一标记)")
            if self.R:
                mc = self.R["mc"]
                lines.append(f"[验收对比] 仿真区间: Rx 方向≤"
                             f"{mc['rx']['dir']['p97_5']:.1f} urad / 位置≤"
                             f"{mc['rx']['pos']['p97_5']:.2f} um / RMSE≤"
                             f"{mc['rx']['rmse']['p97_5']:.0f} nm; Ry 同列. "
                             f"真机以多次标定互差判读")
            else:
                lines.append("提示: 先运行仿真可获得验收区间对比。")
            win = tk.Toplevel(self.root)
            win.title("实测Rx/Ry轴标定数据分析"); win.geometry("760x420")
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
                f"{e}\n\n格式: axis,phase,x_mm,y_mm,z_mm (表头可选)\n"
                f"axis=RX/RY; phase=0零位 / 1旋转后; 两相行序对应同一标记")


if __name__ == "__main__":
    launch(RxRyAxisApp)
