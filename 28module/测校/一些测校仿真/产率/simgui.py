#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simgui.py — D2W 测校仿真通用GUI框架
====================================
参数面板由 Schema 声明自动生成; 时序/XY散点/误差分解/诊断/验收参考 五个
标准标签页对所有测校项通用. 新测校项的GUI = Schema + 少量定制Hook.

Schema 条目格式:
  ("section", "分组标题")
  ("entry",  key, 标签, 默认值, 单位)
  ("radio",  key, 标签, 默认值, [(value, text), ...])
  ("check",  key, 标签, 默认bool)
  ("note",   提示文字)
  ("derived", key, 标签, 单位, fn(params)->str, [触发刷新的key, ...])
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import numpy as np
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib import font_manager as _fm

import simcore

# 中文字体兜底
for _f in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei"):
    if any(_f.lower() in f.name.lower() for f in _fm.fontManager.ttflist):
        matplotlib.rcParams["font.sans-serif"] = [_f, "DejaVu Sans"]
        break
matplotlib.rcParams["axes.unicode_minus"] = False

FONT = "Microsoft YaHei UI"


class BaseSimApp:
    """通用仿真App. 子类需提供:
        title, subtitle           — 窗口标题
        schema()                  — 参数Schema (见模块docstring)
        model                     — 提供 context(p)/sources(p,ctx) 的对象
        collect_extra(p)          — 可选: 对收集到的参数做派生/校验
        extra_tabs()              — 可选: [(名称, builder(parent, R)), ...]
        report_header_lines(R)    — 可选: 报告头附加行
    """

    C = {
        "bg": "#f0f2f5", "panel": "#ffffff", "card": "#f8f9fb",
        "border": "#d0d7de", "accent": "#0969da", "accent2": "#7c3aed",
        "pass_fg": "#1a7f37", "pass_bg": "#dafbe1",
        "fail_fg": "#cf222e", "fail_bg": "#ffebe9",
        "text": "#1f2328", "dim": "#656d76", "entry_bg": "#ffffff",
        "btn": "#0969da", "btn_h": "#0550ae",
        "cx": "#0969da", "cy": "#7c3aed", "grid": "#e8ecef",
        "ref": "#b45309", "ref_bg": "#fef3c7", "warn": "#9a6700",
    }
    title = "D2W 测校仿真"
    subtitle = ""
    report_name = "sim"

    def __init__(self, root):
        self.root = root
        root.title(self.title)
        root.geometry("1500x940"); root.minsize(1200, 700)
        root.configure(bg=self.C["bg"])
        self.vars = {}
        self.R = None          # {"result","ana","mc","diag"}
        self.measured = None   # 导入的实测序列 (x, y)
        self.sim_n = 0
        self._styles(); self._layout()

    # ───────────────────────── layout ─────────────────────────
    def _styles(self):
        s = ttk.Style(); s.theme_use("clam")
        s.configure("TNotebook", background=self.C["bg"])
        s.configure("TNotebook.Tab", background=self.C["card"],
                    foreground=self.C["text"], font=(FONT, 9), padding=[14, 5])
        s.map("TNotebook.Tab", background=[("selected", self.C["panel"])],
              foreground=[("selected", self.C["accent"])])

    def _layout(self):
        hdr = tk.Frame(self.root, bg=self.C["panel"], height=50)
        hdr.pack(fill="x"); hdr.pack_propagate(False)
        tk.Frame(hdr, bg=self.C["accent"], width=4).pack(side="left", fill="y")
        tk.Label(hdr, text="  D2W 测校仿真", font=(FONT, 14, "bold"),
                 bg=self.C["panel"], fg=self.C["text"]).pack(side="left")
        tk.Label(hdr, text=self.subtitle, font=(FONT, 9),
                 bg=self.C["panel"], fg=self.C["dim"]).pack(side="left", padx=12)
        tk.Frame(self.root, bg=self.C["border"], height=1).pack(fill="x")

        pw = tk.PanedWindow(self.root, orient="horizontal", bg=self.C["bg"],
                            sashwidth=5, sashrelief="flat")
        pw.pack(fill="both", expand=True)

        lf = tk.Frame(pw, bg=self.C["panel"], width=410)
        lf.pack_propagate(False); pw.add(lf, width=410, minsize=370)
        cv = tk.Canvas(lf, bg=self.C["panel"], highlightthickness=0, bd=0)
        sb = ttk.Scrollbar(lf, orient="vertical", command=cv.yview)
        sf = tk.Frame(cv, bg=self.C["panel"])
        sf.bind("<Configure>", lambda e: cv.configure(scrollregion=cv.bbox("all")))
        cv.create_window((0, 0), window=sf, anchor="nw")
        cv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y"); cv.pack(side="left", fill="both", expand=True)
        cv.bind_all("<MouseWheel>", lambda e: cv.yview_scroll(-int(e.delta / 120), "units"))
        self._build_params(sf)

        rf = tk.Frame(pw, bg=self.C["bg"]); pw.add(rf, minsize=620)
        self.sfr = tk.Frame(rf, bg=self.C["card"], height=46)
        self.sfr.pack(fill="x", padx=6, pady=(6, 2)); self.sfr.pack_propagate(False)
        self.sl = tk.Label(self.sfr, text='点击 "运行仿真" 开始',
                           bg=self.C["card"], fg=self.C["dim"], font=(FONT, 11))
        self.sl.pack(expand=True)
        self.nb = ttk.Notebook(rf)
        self.nb.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        names = ["  时序图  ", "  XY散点  ", "  误差分解  ", "  诊断与建议  ", "  验收参考  "]
        self.tabs = []
        for nm in names + [f"  {n}  " for n, _ in self.extra_tabs()]:
            t = tk.Frame(self.nb, bg=self.C["bg"])
            self.nb.add(t, text=nm); self.tabs.append(t)

    # ───────────────────── schema-driven panel ─────────────────────
    def _build_params(self, p):
        self._derived = []
        self._cond = []          # [(inner_frame, var_key, accepted_values)]

        def host(parent, when):
            """when=(key, value或值元组): 包一层容器, 内容可随条件折叠且保持顺序."""
            if when is None:
                return parent
            wrap = tk.Frame(parent, bg=self.C["panel"]); wrap.pack(fill="x")
            inner = tk.Frame(wrap, bg=self.C["panel"]); inner.pack(fill="x")
            vals = when[1] if isinstance(when[1], (tuple, list, set)) else (when[1],)
            self._cond.append((inner, when[0], tuple(vals)))
            return inner

        for item in self.schema():
            when = None
            if isinstance(item[-1], dict) and "when" in item[-1]:
                when = item[-1]["when"]; item = item[:-1]
            kind = item[0]
            tgt = host(p, when)
            if kind == "section":
                f = tk.Frame(tgt, bg=self.C["panel"]); f.pack(fill="x", padx=12, pady=(12, 4))
                tk.Frame(f, bg=self.C["accent"], width=3, height=14).pack(side="left", padx=(0, 8))
                tk.Label(f, text=item[1], bg=self.C["panel"], fg=self.C["accent"],
                         font=(FONT, 10, "bold")).pack(side="left")
            elif kind == "entry":
                _, key, label, default, unit = item
                f = tk.Frame(tgt, bg=self.C["panel"]); f.pack(fill="x", padx=12, pady=2)
                tk.Label(f, text=label, bg=self.C["panel"], fg=self.C["dim"],
                         font=(FONT, 9), width=24, anchor="w").pack(side="left")
                v = tk.StringVar(value=str(default)); self.vars[key] = v
                tk.Entry(f, textvariable=v, bg=self.C["entry_bg"], fg=self.C["text"],
                         insertbackground=self.C["accent"], font=("Consolas", 10),
                         relief="groove", width=10, justify="right").pack(side="left", padx=4)
                if unit:
                    tk.Label(f, text=unit, bg=self.C["panel"], fg=self.C["dim"],
                             font=(FONT, 8)).pack(side="left")
            elif kind == "radio":
                _, key, label, default, opts = item
                f = tk.Frame(tgt, bg=self.C["panel"]); f.pack(fill="x", padx=12, pady=2)
                tk.Label(f, text=label, bg=self.C["panel"], fg=self.C["dim"],
                         font=(FONT, 9), width=24, anchor="w").pack(side="left")
                v = tk.StringVar(value=default); self.vars[key] = v
                for val, txt in opts:
                    tk.Radiobutton(f, text=txt, variable=v, value=val,
                                   bg=self.C["panel"], fg=self.C["text"],
                                   selectcolor=self.C["card"], font=(FONT, 9)).pack(side="left", padx=2)
            elif kind == "check":
                _, key, label, default = item
                f = tk.Frame(tgt, bg=self.C["panel"]); f.pack(fill="x", padx=12, pady=(6, 2))
                v = tk.BooleanVar(value=default); self.vars[key] = v
                tk.Checkbutton(f, text=label, variable=v, bg=self.C["panel"],
                               fg=self.C["accent"], selectcolor=self.C["card"],
                               font=(FONT, 9, "bold")).pack(side="left")
            elif kind == "note":
                tk.Label(tgt, text="  " + item[1], bg=self.C["panel"], fg=self.C["dim"],
                         font=(FONT, 8), justify="left", anchor="w",
                         wraplength=370).pack(fill="x", padx=12, pady=(0, 4))
            elif kind == "derived":
                _, key, label, unit, fn, deps = item
                f = tk.Frame(tgt, bg=self.C["panel"]); f.pack(fill="x", padx=12, pady=2)
                tk.Label(f, text=label, bg=self.C["panel"], fg=self.C["dim"],
                         font=(FONT, 9), width=24, anchor="w").pack(side="left")
                lbl = tk.Label(f, text="---", bg=self.C["card"], fg=self.C["accent"],
                               font=("Consolas", 10, "bold"), relief="groove", padx=6, pady=1)
                lbl.pack(side="left", padx=4)
                if unit:
                    tk.Label(f, text=unit, bg=self.C["panel"], fg=self.C["dim"],
                             font=(FONT, 8)).pack(side="left")
                self._derived.append((lbl, fn))
                for dk in deps:
                    if dk in self.vars:
                        self.vars[dk].trace_add("write", lambda *_: self._refresh_derived())

        # 条件可见性: 跟踪所有条件Key, 切换时折叠/展开 (容器占位保持顺序)
        for ck in {c[1] for c in self._cond}:
            if ck in self.vars:
                self.vars[ck].trace_add("write", lambda *_: self._apply_visibility())
        self._apply_visibility()
        self._refresh_derived()

        tk.Frame(p, bg=self.C["border"], height=1).pack(fill="x", padx=12, pady=4)
        bf = tk.Frame(p, bg=self.C["panel"]); bf.pack(fill="x", padx=12, pady=6)
        rb = tk.Button(bf, text="  ▶  运行仿真  ", command=self._run, bg=self.C["btn"],
                       fg="white", font=(FONT, 12, "bold"), relief="flat", cursor="hand2",
                       padx=20, pady=8, activebackground=self.C["btn_h"], activeforeground="white")
        rb.pack(fill="x", pady=(0, 8))
        rb.bind("<Enter>", lambda e: rb.config(bg=self.C["btn_h"]))
        rb.bind("<Leave>", lambda e: rb.config(bg=self.C["btn"]))
        br = tk.Frame(bf, bg=self.C["panel"]); br.pack(fill="x")
        btns = [("导出报告", self._export), ("导出参数", self._exp_p),
                ("导入参数", self._imp_p), ("导入实测CSV", self._imp_csv)]
        for txt, cmd in btns:
            b = tk.Button(br, text=f" {txt} ", command=cmd, bg=self.C["card"],
                          fg=self.C["text"], font=(FONT, 9), relief="groove",
                          cursor="hand2", padx=4, pady=3)
            b.pack(side="left", expand=True, fill="x", padx=2)

    def _apply_visibility(self):
        for inner, key, vals in getattr(self, "_cond", []):
            if key not in self.vars:
                continue
            if str(self.vars[key].get()) in [str(v) for v in vals]:
                inner.pack(fill="x")
            else:
                inner.pack_forget()

    def _refresh_derived(self):
        if not hasattr(self, "_derived"):
            return
        try:
            p = self.collect()
        except Exception:
            return
        for lbl, fn in self._derived:
            try:
                lbl.config(text=fn(p))
            except Exception:
                lbl.config(text="---")

    # ───────────────────── param collection ─────────────────────
    def _gf(self, k, d=0.0):
        v = self.vars[k].get().strip()
        if v == "":
            return None if d is None else d
        try:
            return float(v)
        except ValueError:
            return d

    def _gi(self, k, d=0):
        v = self.vars[k].get().strip()
        if v == "":
            return None if d is None else d
        try:
            return int(float(v))
        except ValueError:
            return d

    def collect(self):
        """按Schema自动收集. entry→float(可空), check→bool, radio→str.
        子类可覆写 collect_extra() 做派生/约束."""
        p = {}
        for item in self.schema():
            kind = item[0]
            if kind == "entry":
                key = item[1]
                if key in ("num_frames", "num_cycles", "num_samples", "mc_runs", "seed"):
                    p[key] = self._gi(key, None if key == "seed" else int(float(item[3] or 0)))
                else:
                    p[key] = self._gf(key, float(item[3]) if str(item[3]).strip() != "" else 0.0)
            elif kind == "radio":
                p[item[1]] = self.vars[item[1]].get()
            elif kind == "check":
                p[item[1]] = bool(self.vars[item[1]].get())
        self.collect_extra(p)
        return p

    def collect_extra(self, p):
        pass

    def extra_tabs(self):
        return []

    def report_header_lines(self, R):
        return []

    # ───────────────────── plotting helpers ─────────────────────
    def _ax(self, ax, title="", xl="", yl=""):
        ax.set_facecolor("#ffffff")
        if title:
            ax.set_title(title, color=self.C["text"], fontsize=10, pad=8, fontweight="bold")
        if xl:
            ax.set_xlabel(xl, color=self.C["dim"], fontsize=9)
        if yl:
            ax.set_ylabel(yl, color=self.C["dim"], fontsize=9)
        ax.tick_params(colors=self.C["dim"], labelsize=8)
        for sp in ax.spines.values():
            sp.set_color(self.C["border"])
        ax.grid(True, color=self.C["grid"], alpha=0.7, linewidth=0.5)

    def _embed(self, fig, tab):
        c = FigureCanvasTkAgg(fig, tab); c.draw()
        tb = NavigationToolbar2Tk(c, tab); tb.update()
        tb.pack(side="bottom", fill="x")
        c.get_tk_widget().pack(fill="both", expand=True)

    def _clear(self, tab):
        for w in tab.winfo_children():
            w.destroy()

    # ───────────────────── run ─────────────────────
    def _run(self):
        try:
            p = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        self.sim_n += 1
        eng = simcore.Engine(self.model, p)
        result = eng.run()
        ana = eng.analytical()
        mc = eng.monte_carlo(max(50, p.get("mc_runs") or 500))
        key = "sp" if result["detrend"] else "sr"
        st = result[key]
        diag = simcore.diagnose_series(
            result["proc" if result["detrend"] else "raw"]["x"],
            result["proc" if result["detrend"] else "raw"]["y"],
            dt_s=result["ctx"].dt_s, analytical=ana, spec=result["spec"])
        self.R = {"result": result, "ana": ana, "mc": mc, "diag": diag, "stats": st}
        self._update_status()
        self._render_all()

    def _update_status(self):
        R = self.R; st = R["stats"]; r = R["result"]
        mc = R["mc"]; spec = r["spec"]
        in_ci = (mc["s3x_p2_5"] <= st["s3x"] <= mc["s3x_p97_5"] and
                 mc["s3y_p2_5"] <= st["s3y"] <= mc["s3y_p97_5"])
        if spec:
            ok = r["pass_p"] if r["detrend"] else r["pass_r"]
            if ok and in_ci:
                bg, fg, tag = self.C["pass_bg"], self.C["pass_fg"], "PASS·正常"
            elif ok:
                bg, fg, tag = self.C["ref_bg"], self.C["warn"], "PASS·超出参考区间, 建议关注"
            else:
                bg, fg, tag = self.C["fail_bg"], self.C["fail_fg"], "FAIL"
            tail = f"Spec ±{spec:.0f} nm"
        else:
            if in_ci:
                bg, fg, tag = self.C["pass_bg"], self.C["pass_fg"], "正常·位于验收区间"
            else:
                bg, fg, tag = self.C["ref_bg"], self.C["warn"], "超出验收区间 → 见诊断页"
            tail = (f"验收区间 X[{mc['s3x_p2_5']:.0f},{mc['s3x_p97_5']:.0f}] "
                    f"Y[{mc['s3y_p2_5']:.0f},{mc['s3y_p97_5']:.0f}] nm")
        self.sfr.config(bg=bg); self.sl.config(
            bg=bg, fg=fg, font=(FONT, 11, "bold"),
            text=(f"[{tag}]   3σX={st['s3x']:.1f} nm   3σY={st['s3y']:.1f} nm   "
                  f"({tail} | 解析 X={R['ana']['three_sigma_x']:.1f} "
                  f"Y={R['ana']['three_sigma_y']:.1f} | 运行#{self.sim_n})"))

    # ───────────────────── standard tabs ─────────────────────
    def _render_all(self):
        R = self.R
        builders = [self._tab_timeseries, self._tab_scatter, self._tab_budget,
                    self._tab_diag, self._tab_accept]
        for i, b in enumerate(builders):
            self._clear(self.tabs[i]); b(self.tabs[i], R)
        for j, (_, b) in enumerate(self.extra_tabs()):
            self._clear(self.tabs[5 + j]); b(self.tabs[5 + j], R)

    def _tab_timeseries(self, tab, R):
        r = R["result"]; key = "proc" if r["detrend"] else "raw"
        x, y, fr = r[key]["x"], r[key]["y"], r["frames"]
        st = R["stats"]; spec = r["spec"]
        fig = Figure(figsize=(10, 6), dpi=100, facecolor=self.C["bg"])
        for i, (a, lbl, col, s3) in enumerate(
                [(x, "X", self.C["cx"], st["s3x"]), (y, "Y", self.C["cy"], st["s3y"])]):
            ax = fig.add_subplot(2, 1, i + 1)
            self._ax(ax, f"{lbl} 方向误差时序  (3σ={s3:.1f} nm)", "帧/循环序号", "误差 (nm)")
            ax.plot(fr, a, color=col, lw=1.0, marker="o", ms=2.5, alpha=0.85)
            ax.axhline(0, color=self.C["dim"], lw=0.6)
            for sgn in (1, -1):
                ax.axhline(sgn * s3 / 3 * 3, color=col, lw=0.8, ls=":", alpha=0.6)
                if spec:
                    ax.axhline(sgn * spec, color=self.C["fail_fg"], lw=1.0, ls="--", alpha=0.6)
            if self.measured is not None and len(self.measured[0]) > 1:
                mx = self.measured[0] if lbl == "X" else self.measured[1]
                ax.plot(np.arange(1, len(mx) + 1), mx, color="#444", lw=0.9,
                        ls="--", alpha=0.7, label="实测导入")
                ax.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_scatter(self, tab, R):
        r = R["result"]; key = "proc" if r["detrend"] else "raw"
        x, y = r[key]["x"], r[key]["y"]; st = R["stats"]; spec = r["spec"]
        fig = Figure(figsize=(7, 6), dpi=100, facecolor=self.C["bg"])
        ax = fig.add_subplot(111)
        self._ax(ax, "XY 误差散点 (含3σ椭圆)", "X (nm)", "Y (nm)")
        sc = ax.scatter(x, y, c=np.arange(len(x)), cmap="viridis", s=22,
                        alpha=0.85, edgecolor="white", linewidth=0.4)
        fig.colorbar(sc, ax=ax, label="序号")
        from matplotlib.patches import Ellipse, Rectangle
        ax.add_patch(Ellipse((np.mean(x), np.mean(y)), 2 * st["s3x"], 2 * st["s3y"],
                             fill=False, color=self.C["accent"], lw=1.5, ls="-."))
        if spec:
            ax.add_patch(Rectangle((-spec, -spec), 2 * spec, 2 * spec, fill=False,
                                   color=self.C["fail_fg"], lw=1.2, ls="--"))
        lim = max(np.max(np.abs(x)), np.max(np.abs(y)), spec or 0) * 1.2
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_budget(self, tab, R):
        r, ana = R["result"], R["ana"]
        names = [c["name"] for c in ana["components"]]
        sx = [c["sigma_x"] for c in ana["components"]]
        sy = [c["sigma_y"] for c in ana["components"]]
        rms_x = [r["budget"][n]["rms_x"] for n in names]
        rms_y = [r["budget"][n]["rms_y"] for n in names]
        fig = Figure(figsize=(10, 6), dpi=100, facecolor=self.C["bg"])
        for i, (ana_v, run_v, lbl, col) in enumerate(
                [(sx, rms_x, "X", self.C["cx"]), (sy, rms_y, "Y", self.C["cy"])]):
            ax = fig.add_subplot(1, 2, i + 1)
            self._ax(ax, f"{lbl} 方向误差分解 (σ, nm)", "nm", "")
            yp = np.arange(len(names))
            ax.barh(yp + 0.18, ana_v, height=0.34, color=col, alpha=0.85, label="解析σ")
            ax.barh(yp - 0.18, run_v, height=0.34, color=col, alpha=0.35, label="单次RMS")
            ax.set_yticks(yp); ax.set_yticklabels(names, fontsize=8)
            ax.invert_yaxis(); ax.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def _tab_diag(self, tab, R):
        d = R["diag"]
        wrap = tk.Frame(tab, bg=self.C["bg"]); wrap.pack(fill="both", expand=True)
        txt = tk.Text(wrap, bg=self.C["panel"], fg=self.C["text"], font=(FONT, 10),
                      relief="flat", wrap="word", padx=14, pady=12, spacing3=4)
        sb = ttk.Scrollbar(wrap, command=txt.yview); txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y"); txt.pack(fill="both", expand=True, padx=8, pady=8)
        txt.tag_config("h", foreground=self.C["accent"], font=(FONT, 11, "bold"))
        txt.tag_config("warn", foreground=self.C["fail_fg"], font=(FONT, 10, "bold"))
        txt.tag_config("ok", foreground=self.C["pass_fg"], font=(FONT, 10, "bold"))
        txt.tag_config("dim", foreground=self.C["dim"], font=(FONT, 9))
        txt.tag_config("mono", font=("Consolas", 9))
        txt.insert("end", "误差特征识别\n", "h")
        txt.insert("end", d["summary"] + "\n\n", "mono")
        for sev, t, de, adv in d["findings"]:
            txt.insert("end", ("⚠ " if sev == "warn" else "✓ ") + t + "\n", sev if sev in ("warn", "ok") else "dim")
            txt.insert("end", f"   现象: {de}\n", "dim")
            txt.insert("end", f"   建议: {adv}\n\n")
        if d["ranking"]:
            txt.insert("end", "\n方差贡献排名 (解析模型, 超标排查顺序)\n", "h")
            for nm, grp, fx, fy in d["ranking"]:
                bar = "█" * int(round(max(fx, fy) * 24))
                txt.insert("end", f" {nm:<18s} X:{fx*100:5.1f}%  Y:{fy*100:5.1f}%  {bar}\n", "mono")
        bias = R["result"]["bias"]
        if abs(bias["x"]) > 1e-9 or abs(bias["y"]) > 1e-9:
            txt.insert("end", "\n系统偏置 (准确度项, 不计入3σ)\n", "h")
            txt.insert("end", f" Bias X = {bias['x']:.2f} nm,  Bias Y = {bias['y']:.2f} nm\n", "mono")
            txt.insert("end", " 来源: 微旋残差×离心距 r、畸变等. 影响绝对定位准确度, "
                              "由相应标定项(微旋标定/LUT)补偿.\n", "dim")
        txt.config(state="disabled")

    def _tab_accept(self, tab, R):
        ana, mc, r = R["ana"], R["mc"], R["result"]; spec = r["spec"]
        info = tk.Frame(tab, bg=self.C["ref_bg"])
        info.pack(fill="x", padx=10, pady=(10, 4))
        if spec:
            head = (f"验收判读: 实测3σ落于MC 95%CI内 → 正常; 超CI但<Spec(±{spec:.0f}nm) → 关注并按诊断页排查; "
                    f">Spec → FAIL")
            if "pass_prob" in mc:
                head += f" | 仿真通过概率 {mc['pass_prob']*100:.1f}%"
        else:
            head = (f"建议验收区间 (由当前误差模型推导, 供验收判据使用):\n"
                    f"   3σX ∈ [{mc['s3x_p2_5']:.1f}, {mc['s3x_p97_5']:.1f}] nm      "
                    f"3σY ∈ [{mc['s3y_p2_5']:.1f}, {mc['s3y_p97_5']:.1f}] nm      "
                    f"(建议验收上限取P97.5, 工程裕量可上浮10~20%)\n"
                    f"判读: 实测3σ落于区间内 → 设备状态正常; 低于下限 → 核查测量有效性; "
                    f"高于上限 → 按诊断页排查")
        tk.Label(info, text=(
            head + f"\n注意: n={ana['n']} 时3σ点估计自身相对波动约"
            f"{100/np.sqrt(2*(ana['n']-1)):.0f}% (1σ), 区间判读比点值判读更可靠"),
            bg=self.C["ref_bg"], fg=self.C["ref"], font=(FONT, 9),
            justify="left", anchor="w", padx=10, pady=8).pack(fill="x")
        fig = Figure(figsize=(10, 4), dpi=100, facecolor=self.C["bg"])
        for i, ax_lbl in enumerate(("x", "y")):
            ax = fig.add_subplot(1, 2, i + 1)
            col = self.C["cx"] if ax_lbl == "x" else self.C["cy"]
            self._ax(ax, f"MC分布: 3σ{ax_lbl.upper()} ({mc['mc_runs']}次)",
                     f"3σ{ax_lbl.upper()} (nm)", "频次")
            arr = mc[f"s3{ax_lbl}"]
            ax.hist(arr, bins=30, color=col, alpha=0.7, edgecolor="white", lw=0.5)
            ax.axvline(ana[f"three_sigma_{ax_lbl}"], color=self.C["ref"], lw=2, ls="-.",
                       label=f"解析={ana[f'three_sigma_{ax_lbl}']:.1f}")
            for q in ("p2_5", "p97_5"):
                ax.axvline(mc[f"s3{ax_lbl}_{q}"], color="#e5a00d", lw=1.2, ls="--")
            ax.axvspan(mc[f"s3{ax_lbl}_p2_5"], mc[f"s3{ax_lbl}_p97_5"],
                       alpha=0.08, color=self.C["ref"])
            if spec:
                ax.axvline(spec, color=self.C["fail_fg"], lw=1.5, ls="--",
                           alpha=0.7, label=f"Spec={spec:.0f}")
            ax.legend(fontsize=7)
        fig.tight_layout(pad=2.5)
        self._embed(fig, tab)
        det = tk.Frame(tab, bg=self.C["bg"]); det.pack(fill="x", padx=10, pady=(0, 10))
        tk.Label(det, text="解析模型各分量 σ (单轴)", bg=self.C["bg"], fg=self.C["accent2"],
                 font=(FONT, 10, "bold")).pack(anchor="w", pady=(4, 4))
        rows = [(c["name"], f"{c['sigma_x']:.2f} / {c['sigma_y']:.2f}", c["group"])
                for c in ana["components"]]
        rows += [("合成 σ (RSS)", f"{ana['sigma_total_x']:.2f} / {ana['sigma_total_y']:.2f}", ""),
                 ("合成 3σ", f"{ana['three_sigma_x']:.2f} / {ana['three_sigma_y']:.2f}", "")]
        for lbl, val, note in rows:
            f = tk.Frame(det, bg=self.C["bg"]); f.pack(fill="x", padx=4, pady=1)
            bold = "bold" if lbl.startswith("合成") else "normal"
            fg = self.C["accent"] if lbl.startswith("合成") else self.C["text"]
            tk.Label(f, text=lbl, bg=self.C["bg"], fg=fg, font=(FONT, 9, bold),
                     width=24, anchor="w").pack(side="left")
            tk.Label(f, text=val + "  nm", bg=self.C["bg"], fg=fg,
                     font=("Consolas", 10, bold), width=20, anchor="e").pack(side="left")
            tk.Label(f, text=note, bg=self.C["bg"], fg=self.C["dim"],
                     font=(FONT, 8), anchor="w").pack(side="left", padx=(8, 0))

    # ───────────────────── import / export ─────────────────────
    def _export(self):
        if not self.R:
            messagebox.showinfo("提示", "请先运行仿真"); return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Text", "*.txt")],
            initialfile=f"{self.report_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if not path:
            return
        rep = simcore.build_report(self.title, self.R["result"], self.R["ana"],
                                   self.R["mc"], self.R["diag"],
                                   extra_lines=self.report_header_lines(self.R))
        with open(path, "w", encoding="utf-8") as f:
            f.write(rep)
        messagebox.showinfo("OK", f"已保存:\n{path}")

    def _exp_p(self):
        try:
            pr = self.collect()
        except Exception as e:
            messagebox.showerror("参数错误", str(e)); return
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")],
                                            initialfile="sim_params.json")
        if path:
            cl = {k: (int(v) if isinstance(v, (np.integer,)) else
                      float(v) if isinstance(v, (np.floating,)) else v)
                  for k, v in pr.items()}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(cl, f, indent=2, ensure_ascii=False)
            messagebox.showinfo("OK", f"已保存:\n{path}")

    def _imp_p(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                pr = json.load(f)
            for k, v in pr.items():
                if k in self.vars:
                    if isinstance(self.vars[k], tk.BooleanVar):
                        self.vars[k].set(bool(v))
                    else:
                        self.vars[k].set("" if v is None else str(v))
            messagebox.showinfo("OK", "参数已载入")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _imp_csv(self):
        """导入实测序列 (idx,x,y 或 x,y; 单位nm), 叠加显示并跑同一套诊断."""
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not path:
            return
        try:
            x, y = simcore.load_measured_csv(path)
            if len(x) < 5:
                raise ValueError("有效数据少于5行")
            self.measured = (x, y)
            dt = None
            if self.R:
                dt = self.R["result"]["ctx"].dt_s
            ana = self.R["ana"] if self.R else None
            spec = self.R["result"]["spec"] if self.R else None
            diag = simcore.diagnose_series(x, y, dt_s=dt, analytical=ana, spec=spec)
            msg = [f"实测数据 n={len(x)}",
                   f"3σX={3*np.std(x, ddof=1):.1f} nm  3σY={3*np.std(y, ddof=1):.1f} nm", ""]
            if self.R:
                mc = self.R["mc"]
                px = float(np.mean(mc["s3x"] <= 3 * np.std(x, ddof=1))) * 100
                py = float(np.mean(mc["s3y"] <= 3 * np.std(y, ddof=1))) * 100
                msg.append(f"在当前仿真MC分布中的分位: X≈P{px:.0f}, Y≈P{py:.0f}")
                msg.append("(P2.5~P97.5 之间为正常)")
                msg.append("")
            for sev, t, de, adv in diag["findings"]:
                msg.append(("⚠ " if sev == "warn" else "✓ ") + t)
                msg.append("   建议: " + adv)
            messagebox.showinfo("实测数据诊断", "\n".join(msg))
            if self.R:
                self._render_all()
        except Exception as e:
            messagebox.showerror("导入失败", str(e))


def launch(app_cls):
    root = tk.Tk()
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    app_cls(root)
    root.mainloop()
