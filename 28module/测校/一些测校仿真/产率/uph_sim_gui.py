#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W 整机产率(UPH)仿真 GUI — v1
离散事件 + 运动学节拍. 每个模块的位置/速度/加速度/jerk可设,
prep/bond路径由动作序列组装, 对准/计算等操作时间可定义,
A/B双套并行流水求 T_eff=max(T_prep,T_bond) 与 UPH=3600/T_eff.

参考: 架构.md / 全流程串联.md / 交互模型.html
运行: PYTHONPATH=. xvfb-run -a python3 uph_sim_gui.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from machine import (D2WMachine, DEFAULT_POSITIONS, DEFAULT_Z,
                     DEFAULT_KINEMATICS, DEFAULT_OP_TIMES)
from motion import sample_profile
from simgui import BaseSimApp, launch, FONT

try:
    import matplotlib.font_manager as fm
    for f in fm.findSystemFonts():
        if "wqy" in f.lower() or "zenhei" in f.lower():
            fm.fontManager.addfont(f)
            break
    matplotlib.rcParams["font.family"] = FONT
    matplotlib.rcParams["axes.unicode_minus"] = False
except Exception:
    pass

C = {
    "bg": "#f4f6fa", "panel": "#ffffff", "card": "#eef2f8", "border": "#c9d3e0",
    "text": "#1a2332", "dim": "#6b7688", "accent": "#2E6FB5", "accent2": "#8a5cf6",
    "prep": "#2E9B6F", "bond": "#c9752b", "move": "#2E6FB5", "op": "#c94f4f",
    "ok": "#1a7f37", "warn": "#c0392b", "ref_bg": "#fff6e6", "ref": "#8a6d1f",
}


class UPHApp:
    def __init__(self, root):
        self.root = root
        root.title("D2W 整机产率(UPH)仿真 — 运动学节拍 · 双套流水")
        root.configure(bg=C["bg"])
        root.geometry("1480x920")
        self.m = D2WMachine()
        self.entries = {}
        self.R = None
        self._build()
        self._run()

    # ================= 布局 =================
    def _build(self):
        top = tk.Frame(self.root, bg=C["accent"], height=48)
        top.pack(fill="x"); top.pack_propagate(False)
        tk.Label(top, text="D2W 整机产率 (UPH) 仿真", bg=C["accent"],
                 fg="white", font=(FONT, 15, "bold")).pack(side="left", padx=16)
        tk.Label(top, text="运动学节拍 (位置→速度/加速度/jerk曲线) · A/B双套并行流水",
                 bg=C["accent"], fg="#cfe0f5", font=(FONT, 10)).pack(side="left")

        self.status = tk.Label(self.root, text="", bg=C["ref_bg"], fg=C["ref"],
                               font=(FONT, 11, "bold"), anchor="w", padx=14, pady=7)
        self.status.pack(fill="x")

        body = tk.Frame(self.root, bg=C["bg"]); body.pack(fill="both", expand=True)
        # 左: 参数面板 (可滚动)
        left = tk.Frame(body, bg=C["panel"], width=430); left.pack(side="left",
                                                                   fill="y")
        left.pack_propagate(False)
        self._build_param_panel(left)
        # 右: 结果标签页
        right = tk.Frame(body, bg=C["bg"]); right.pack(side="left", fill="both",
                                                       expand=True)
        self.nb = ttk.Notebook(right); self.nb.pack(fill="both", expand=True,
                                                    padx=6, pady=6)
        self.tabs = {}
        for name in ("路径节拍", "并行流水", "运动剖面", "敏感性分析", "节拍明细"):
            f = tk.Frame(self.nb, bg=C["bg"]); self.nb.add(f, text=name)
            self.tabs[name] = f

    def _build_param_panel(self, parent):
        bar = tk.Frame(parent, bg=C["panel"]); bar.pack(fill="x", padx=8, pady=6)
        tk.Button(bar, text="▶ 运行仿真", bg=C["accent"], fg="white",
                  font=(FONT, 11, "bold"), relief="flat", command=self._run,
                  padx=12, pady=4).pack(side="left")
        tk.Button(bar, text="导出报告", bg=C["card"], fg=C["text"],
                  font=(FONT, 10), relief="flat", command=self._export,
                  padx=8, pady=4).pack(side="left", padx=4)

        canvas = tk.Canvas(parent, bg=C["panel"], highlightthickness=0)
        sb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        self.scroll = tk.Frame(canvas, bg=C["panel"])
        self.scroll.bind("<Configure>",
                         lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scroll, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        self._populate_params(self.scroll)

    def _section(self, parent, title):
        f = tk.Frame(parent, bg=C["accent"]); f.pack(fill="x", pady=(10, 2), padx=6)
        tk.Label(f, text=title, bg=C["accent"], fg="white",
                 font=(FONT, 10, "bold"), anchor="w", padx=8, pady=3).pack(fill="x")

    def _row(self, parent, key, label, val, unit=""):
        f = tk.Frame(parent, bg=C["panel"]); f.pack(fill="x", padx=8, pady=1)
        tk.Label(f, text=label, bg=C["panel"], fg=C["text"], font=(FONT, 9),
                 width=20, anchor="w").pack(side="left")
        e = tk.Entry(f, width=9, font=("Consolas", 9), justify="right")
        e.insert(0, str(val)); e.pack(side="left", padx=2)
        tk.Label(f, text=unit, bg=C["panel"], fg=C["dim"], font=(FONT, 8),
                 width=7, anchor="w").pack(side="left")
        self.entries[key] = e

    def _populate_params(self, p):
        # 处方与全局
        self._section(p, "处方 / 全局")
        f = tk.Frame(p, bg=C["panel"]); f.pack(fill="x", padx=8, pady=2)
        tk.Label(f, text="键合模式", bg=C["panel"], fg=C["text"], font=(FONT, 9),
                 width=20, anchor="w").pack(side="left")
        self.mode_var = tk.StringVar(value=self.m.p["recipe::mode"])
        ttk.Combobox(f, textvariable=self.mode_var, values=["fd", "fu"],
                     width=7, state="readonly").pack(side="left", padx=2)
        tk.Label(f, text="fd/fu", bg=C["panel"], fg=C["dim"],
                 font=(FONT, 8)).pack(side="left")

        f2 = tk.Frame(p, bg=C["panel"]); f2.pack(fill="x", padx=8, pady=2)
        tk.Label(f2, text="运动剖面", bg=C["panel"], fg=C["text"], font=(FONT, 9),
                 width=20, anchor="w").pack(side="left")
        self.prof_var = tk.StringVar(value=self.m.p["profile"])
        ttk.Combobox(f2, textvariable=self.prof_var,
                     values=["scurve", "trapezoidal"], width=10,
                     state="readonly").pack(side="left", padx=2)

        # 开关
        self.sw = {}
        for key, lbl in (("recipe::dip", "沾胶 Dipping"),
                         ("recipe::level", "调焦调平"),
                         ("recipe::confirm", "对准确认"),
                         ("coupled_xy", "XY协调插补")):
            fr = tk.Frame(p, bg=C["panel"]); fr.pack(fill="x", padx=8, pady=0)
            v = tk.BooleanVar(value=bool(self.m.p[key]))
            tk.Checkbutton(fr, text=lbl, variable=v, bg=C["panel"], fg=C["text"],
                           font=(FONT, 9), anchor="w",
                           selectcolor=C["card"]).pack(side="left")
            self.sw[key] = v

        self._row(p, "recipe::align_marks", "精对准标记数", 3, "2-4")
        self._row(p, "recipe::dwell_s", "保压时间", 3.0, "s")
        self._row(p, "recipe::N", "本批键合数 N", 25, "颗")
        self._row(p, "recipe::film_dies", "蓝膜余量", 25, "片")
        self._row(p, "addr_dist_mm", "硅片台寻址平均行程", 120.0, "mm")

        # 模块空间位置
        self._section(p, "模块空间位置 (mm)")
        for k, (x, y) in DEFAULT_POSITIONS.items():
            self._row(p, f"pos::{k}::x", f"{k} · X", x, "mm")
            self._row(p, f"pos::{k}::y", f"{k} · Y", y, "mm")
        self._section(p, "Z向关键高度 (mm)")
        for k, z in DEFAULT_Z.items():
            self._row(p, f"z::{k}", k, z, "mm")

        # 运动参数
        self._section(p, "各轴运动参数 (v / a / jerk)")
        for k, (v, a, j, u) in DEFAULT_KINEMATICS.items():
            fr = tk.Frame(p, bg=C["card"]); fr.pack(fill="x", padx=6, pady=1)
            tk.Label(fr, text=k, bg=C["card"], fg=C["accent"],
                     font=(FONT, 9, "bold"), anchor="w").pack(fill="x", padx=4)
            self._row(p, f"kin::{k}::v", f"  速度 v", v, f"{u}/s")
            self._row(p, f"kin::{k}::a", f"  加速度 a", a, f"{u}/s²")
            self._row(p, f"kin::{k}::j", f"  加加速度 j", j, f"{u}/s³")

        # 操作时间
        self._section(p, "操作/计算时间 (s)")
        for k, t in DEFAULT_OP_TIMES.items():
            self._row(p, f"op::{k}", k, t, "s")

    # ================= 采集参数 =================
    def _collect(self):
        p = dict(self.m.p)
        for key, e in self.entries.items():
            try:
                val = float(e.get())
                if key in ("recipe::align_marks", "recipe::N",
                           "recipe::film_dies"):
                    val = int(val)
                p[key] = val
            except ValueError:
                pass
        p["recipe::mode"] = self.mode_var.get()
        p["profile"] = self.prof_var.get()
        for key, v in self.sw.items():
            p[key] = bool(v.get())
        return p

    # ================= 运行 =================
    def _run(self):
        p = self._collect()
        self.m = D2WMachine(p)
        addr = p.get("addr_dist_mm", 120.0)
        self.addr = addr
        self.R = self.m.pipeline(addr_dist_mm=addr)
        r = self.R
        bn = "键合(bond)" if r["bottleneck"] == "bond" else "取片(prep)"
        self.status.config(text=(
            f"T_prep={r['t_prep']:.2f}s   T_bond={r['t_bond']:.2f}s   "
            f"T_eff={r['t_eff']:.2f}s (瓶颈: {bn})   ▶ UPH(双套)={r['uph']:.0f} "
            f"颗/时  (串行 {r['uph_serial']:.0f}, 提升×{r['throughput_ratio']:.2f})   "
            f"本批{r['n_bond']}颗≈{r['total_time_s']/60:.1f}min"))
        self._render()

    def _render(self):
        for f in self.tabs.values():
            for w in f.winfo_children():
                w.destroy()
        self._tab_paths()
        self._tab_pipeline()
        self._tab_profile()
        self._tab_sensitivity()
        self._tab_detail()

    def _embed(self, fig, tab):
        c = FigureCanvasTkAgg(fig, tab); c.draw()
        c.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

    # ---------- 路径节拍 (甘特) ----------
    def _tab_paths(self):
        tab = self.tabs["路径节拍"]
        bd = self.m.breakdown(self.addr)
        fig = Figure(figsize=(11, 6), dpi=100, facecolor=C["bg"])
        for idx, (seg, acts, tot) in enumerate(
                (("PREP 段 (不占硅片台)", bd["prep"], self.R["t_prep"]),
                 ("BOND 段 (占硅片台)", bd["bond"], self.R["t_bond"]))):
            ax = fig.add_subplot(2, 1, idx + 1)
            ax.set_title(f"{seg}   总耗时 {tot:.2f} s", fontsize=11,
                         fontweight="bold", color=C["text"])
            t0 = 0.0
            for name, kind, t, _ in acts:
                col = C["move"] if kind == "move" else C["op"]
                ax.barh(0, t, left=t0, height=0.6, color=col,
                        edgecolor="white", linewidth=0.5)
                if t > tot * 0.03:
                    ax.text(t0 + t / 2, 0, name, ha="center", va="center",
                            fontsize=6.5, color="white", rotation=0)
                t0 += t
            ax.set_xlim(0, tot * 1.02); ax.set_ylim(-0.5, 0.5)
            ax.set_yticks([]); ax.set_xlabel("时间 (s)", fontsize=9)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.spines["left"].set_visible(False)
        from matplotlib.patches import Patch
        fig.legend(handles=[Patch(color=C["move"], label="运动 (运动学解算)"),
                            Patch(color=C["op"], label="操作/计算 (固定时间)")],
                   loc="upper right", fontsize=9, ncol=2)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    # ---------- 并行流水 ----------
    def _tab_pipeline(self):
        tab = self.tabs["并行流水"]
        r = self.R
        fig = Figure(figsize=(11, 6), dpi=100, facecolor=C["bg"])
        ax = fig.add_subplot(111)
        tp, tb, te = r["t_prep"], r["t_bond"], r["t_eff"]
        n_show = min(5, r["N"])
        # 三行: 硅片台(共享bond) / A套 / B套
        rows = {"硅片台(共享)": 2, "A套": 1, "B套": 0}
        for lbl, y in rows.items():
            ax.text(-0.3, y, lbl, ha="right", va="center", fontsize=10,
                    fontweight="bold", color=C["text"])
        # prep(0,A)
        t = 0.0
        ax.barh(1, tp, left=t, height=0.6, color=C["prep"], alpha=0.85,
                edgecolor="white")
        ax.text(t + tp / 2, 1, "prep0·A", ha="center", va="center", fontsize=8,
                color="white")
        t += tp
        for i in range(n_show):
            setb = "A" if i % 2 == 0 else "B"
            setp = "B" if i % 2 == 0 else "A"
            yb = 1 if setb == "A" else 0
            yp = 1 if setp == "A" else 0
            # bond(i) 在硅片台行 + 对应套行
            ax.barh(2, tb, left=t, height=0.6, color=C["bond"], alpha=0.85,
                    edgecolor="white")
            ax.text(t + tb / 2, 2, f"bond{i}·{setb}", ha="center", va="center",
                    fontsize=8, color="white")
            # prep(i+1) 并行于另一套
            if i < n_show - 1:
                ax.barh(yp, tp, left=t, height=0.6, color=C["prep"], alpha=0.85,
                        edgecolor="white")
                ax.text(t + tp / 2, yp, f"prep{i+1}·{setp}", ha="center",
                        va="center", fontsize=8, color="white")
            t += te
        ax.axvline(tp, color=C["dim"], ls=":", lw=1)
        ax.set_ylim(-0.6, 2.6); ax.set_xlim(-0.05 * t, t * 1.02)
        ax.set_yticks([]); ax.set_xlabel("时间 (s)", fontsize=10)
        ax.set_title(f"A/B双套并行流水 (前{n_show}颗)   稳态节拍 T_eff = "
                     f"max(T_prep {tp:.1f}, T_bond {tb:.1f}) = {te:.2f} s   →   "
                     f"UPH = 3600/{te:.2f} = {r['uph']:.0f}",
                     fontsize=11, fontweight="bold", color=C["text"])
        ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
        from matplotlib.patches import Patch
        ax.legend(handles=[Patch(color=C["prep"], label="prep段(取片/翻转/沾胶)"),
                           Patch(color=C["bond"], label="bond段(寻址/对准/键合)")],
                  loc="lower right", fontsize=9)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    # ---------- 运动剖面 ----------
    def _tab_profile(self):
        tab = self.tabs["运动剖面"]
        bar = tk.Frame(tab, bg=C["bg"]); bar.pack(fill="x", padx=8, pady=6)
        tk.Label(bar, text="选择轴查看运动剖面:", bg=C["bg"], fg=C["text"],
                 font=(FONT, 10)).pack(side="left")
        self.prof_axis = tk.StringVar(value="键合臂X")
        cb = ttk.Combobox(bar, textvariable=self.prof_axis,
                          values=list(DEFAULT_KINEMATICS.keys()), width=14,
                          state="readonly"); cb.pack(side="left", padx=6)
        tk.Label(bar, text="行程距离:", bg=C["bg"], fg=C["text"],
                 font=(FONT, 10)).pack(side="left")
        self.prof_dist = tk.Entry(bar, width=8, font=("Consolas", 10))
        self.prof_dist.insert(0, "200"); self.prof_dist.pack(side="left", padx=4)
        holder = tk.Frame(tab, bg=C["bg"]); holder.pack(fill="both", expand=True)

        def draw(*_):
            for w in holder.winfo_children():
                w.destroy()
            ax_name = self.prof_axis.get()
            try:
                D = float(self.prof_dist.get())
            except ValueError:
                D = 200.0
            v, a, j = self.m.kin(ax_name)
            prof = self.m.prof()
            sc = sample_profile(D, v, a, j, prof)
            tr = sample_profile(D, v, a, j, "trapezoidal")
            fig = Figure(figsize=(11, 6), dpi=100, facecolor=C["bg"])
            for i, (key, lbl, unit) in enumerate(
                    (("pos", "位置", "mm"), ("vel", "速度", "mm/s"),
                     ("acc", "加速度", "mm/s²"))):
                ax = fig.add_subplot(3, 1, i + 1)
                ax.plot(sc["t"], sc[key], color=C["accent"], lw=1.8,
                        label=f"{prof} (T={sc['T']:.3f}s)")
                ax.set_ylabel(f"{lbl}\n({unit})", fontsize=9)
                ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="best")
                if i == 0:
                    ax.set_title(f"{ax_name} 运动剖面  行程 {D:.0f}  "
                                 f"v={v} a={a} j={j}", fontsize=11,
                                 fontweight="bold", color=C["text"])
                if i == 2:
                    ax.set_xlabel("时间 (s)", fontsize=9)
            fig.tight_layout(pad=1.5)
            c = FigureCanvasTkAgg(fig, holder); c.draw()
            c.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=4)

        cb.bind("<<ComboboxSelected>>", draw)
        self.prof_dist.bind("<Return>", draw)
        draw()

    # ---------- 敏感性 ----------
    def _tab_sensitivity(self):
        tab = self.tabs["敏感性分析"]
        fig = Figure(figsize=(11, 6), dpi=100, facecolor=C["bg"])
        base = dict(self.m.p)
        # 1) 保压时间 → UPH
        ax = fig.add_subplot(221)
        dwells = [1, 2, 3, 4, 5, 6, 8, 10]
        uphs = []
        for d in dwells:
            mm = D2WMachine(dict(base, **{"recipe::dwell_s": float(d)}))
            uphs.append(mm.pipeline(addr_dist_mm=self.addr)["uph"])
        ax.plot(dwells, uphs, "o-", color=C["bond"], lw=1.6)
        ax.set_xlabel("保压时间 (s)", fontsize=9); ax.set_ylabel("UPH", fontsize=9)
        ax.set_title("保压时间 → UPH", fontsize=10, fontweight="bold")
        ax.grid(alpha=0.3)
        # 2) 硅片台XY速度 → UPH
        ax2 = fig.add_subplot(222)
        speeds = [50, 100, 150, 200, 300, 400, 600]
        u2 = []
        for s in speeds:
            mm = D2WMachine(dict(base, **{"kin::硅片台XY::v": float(s)}))
            u2.append(mm.pipeline(addr_dist_mm=self.addr)["uph"])
        ax2.plot(speeds, u2, "s-", color=C["accent"], lw=1.6)
        ax2.set_xlabel("硅片台XY速度 (mm/s)", fontsize=9)
        ax2.set_ylabel("UPH", fontsize=9)
        ax2.set_title("硅片台速度 → UPH", fontsize=10, fontweight="bold")
        ax2.grid(alpha=0.3)
        # 3) 对准标记数 → UPH
        ax3 = fig.add_subplot(223)
        marks = [2, 3, 4]
        u3 = []
        for mk in marks:
            mm = D2WMachine(dict(base, **{"recipe::align_marks": mk}))
            u3.append(mm.pipeline(addr_dist_mm=self.addr)["uph"])
        ax3.bar([str(x) for x in marks], u3, color=C["accent2"], alpha=0.8)
        for i, u in enumerate(u3):
            ax3.text(i, u, f"{u:.0f}", ha="center", va="bottom", fontsize=9)
        ax3.set_xlabel("精对准标记数", fontsize=9); ax3.set_ylabel("UPH", fontsize=9)
        ax3.set_title("对准标记数 → UPH", fontsize=10, fontweight="bold")
        # 4) 开关组合 → UPH
        ax4 = fig.add_subplot(224)
        combos = [("全开", {"recipe::dip": True, "recipe::level": True,
                            "recipe::confirm": True}),
                  ("默认", {"recipe::dip": True, "recipe::level": True,
                            "recipe::confirm": False}),
                  ("免调平", {"recipe::dip": True, "recipe::level": False,
                              "recipe::confirm": False}),
                  ("免沾胶", {"recipe::dip": False, "recipe::level": True,
                              "recipe::confirm": False}),
                  ("最简", {"recipe::dip": False, "recipe::level": False,
                            "recipe::confirm": False})]
        labels = [c[0] for c in combos]
        u4 = []
        for _, ov in combos:
            mm = D2WMachine(dict(base, **ov))
            u4.append(mm.pipeline(addr_dist_mm=self.addr)["uph"])
        ax4.barh(labels[::-1], u4[::-1], color=C["prep"], alpha=0.8)
        ax4.set_xlim(0, max(u4) * 1.18)
        for i, u in enumerate(u4[::-1]):
            ax4.text(u, i, f" {u:.0f}", va="center", fontsize=9)
        ax4.set_xlabel("UPH", fontsize=9)
        ax4.set_title("工序开关 → UPH", fontsize=10, fontweight="bold")
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    # ---------- 节拍明细 ----------
    def _tab_detail(self):
        tab = self.tabs["节拍明细"]
        wrap = tk.Frame(tab, bg=C["bg"]); wrap.pack(fill="both", expand=True)
        txt = tk.Text(wrap, bg=C["panel"], fg=C["text"], font=("Consolas", 10),
                      relief="flat", wrap="none", padx=12, pady=10)
        sb = ttk.Scrollbar(wrap, command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y"); txt.pack(fill="both", expand=True,
                                                  padx=6, pady=6)
        r = self.R
        L = []
        L.append("="*64)
        L.append("  D2W 产率仿真 — 节拍明细")
        L.append("="*64)
        L.append(f"  运动剖面: {self.m.prof()}   XY协调插补: {self.m.p['coupled_xy']}")
        L.append(f"  键合模式: {self.m.p['recipe::mode']}   保压: "
                 f"{self.m.p['recipe::dwell_s']}s   对准标记: "
                 f"{self.m.p['recipe::align_marks']}")
        L.append("")
        bd = self.m.breakdown(self.addr)
        L.append("-"*64)
        L.append(f"  PREP 段  (总 {r['t_prep']:.3f} s = 运动 {bd['prep_move']:.2f} "
                 f"+ 操作 {bd['prep_op']:.2f})")
        L.append("-"*64)
        for name, kind, t, _ in bd["prep"]:
            L.append(f"    [{kind:4s}] {name:22s} {t*1000:8.1f} ms")
        L.append("")
        L.append("-"*64)
        L.append(f"  BOND 段  (总 {r['t_bond']:.3f} s = 运动 {bd['bond_move']:.2f} "
                 f"+ 操作 {bd['bond_op']:.2f})")
        L.append("-"*64)
        for name, kind, t, _ in bd["bond"]:
            L.append(f"    [{kind:4s}] {name:22s} {t*1000:8.1f} ms")
        L.append("")
        L.append("="*64)
        L.append("  产率结果")
        L.append("="*64)
        L.append(f"  T_prep        = {r['t_prep']:.3f} s")
        L.append(f"  T_bond        = {r['t_bond']:.3f} s")
        L.append(f"  T_eff         = max(T_prep, T_bond) = {r['t_eff']:.3f} s  "
                 f"(瓶颈: {r['bottleneck']})")
        L.append(f"  UPH (双套流水) = 3600 / {r['t_eff']:.3f} = {r['uph']:.1f} 颗/时")
        L.append(f"  UPH (串行)     = 3600 / (Tp+Tb) = {r['uph_serial']:.1f} 颗/时")
        L.append(f"  流水提升        = ×{r['throughput_ratio']:.3f}")
        L.append(f"  本批 {r['n_bond']} 颗总耗时 = {r['total_time_s']:.1f} s "
                 f"= {r['total_time_s']/60:.2f} min")
        txt.insert("1.0", "\n".join(L))
        txt.config(state="disabled")

    # ---------- 导出 ----------
    def _export(self):
        if not self.R:
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                            filetypes=[("Text", "*.txt")])
        if not path:
            return
        bd = self.m.breakdown(self.addr)
        r = self.R
        with open(path, "w", encoding="utf-8") as f:
            f.write("D2W 产率仿真报告\n")
            f.write(f"T_prep={r['t_prep']:.3f}s T_bond={r['t_bond']:.3f}s "
                    f"T_eff={r['t_eff']:.3f}s\n")
            f.write(f"UPH(双套)={r['uph']:.1f} UPH(串行)={r['uph_serial']:.1f} "
                    f"提升x{r['throughput_ratio']:.3f}\n\n")
            for seg, acts in (("PREP", bd["prep"]), ("BOND", bd["bond"])):
                f.write(f"[{seg}]\n")
                for name, kind, t, _ in acts:
                    f.write(f"  [{kind}] {name}: {t*1000:.1f} ms\n")
                f.write("\n")
        messagebox.showinfo("导出", f"已保存:\n{path}")


if __name__ == "__main__":
    root = tk.Tk()
    app = UPHApp(root)
    root.mainloop()
