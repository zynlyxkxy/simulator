#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D2W 相机对准重复性仿真 — 静态/动态合并版 (FR1/FR2/FR5/FR6)
参考文档: SDS §4.1.1, SSPS §3.1.1~3.1.4, STPS FR1/FR2/FR5/FR6, SSDS 序号5/6/12/13

一个GUI覆盖四个测校项:
  静态模式 (FR1/FR2): 对准台静止, 连续采集 n 帧;
  动态模式 (FR5/FR6): 对准台往返运动 n 次, 每次到位后触发拍照,
                      在静态误差源之上叠加运动台误差源.
切换模式时, 与当前模式无关的参数自动折叠.

验收判据: 不设固定Spec. 由当前误差模型推导"建议验收区间":
  [MC P2.5, MC P97.5] (与解析RSS交叉校验), 建议验收上限取P97.5,
  实测3σ落于区间内→正常; 高于上限→按诊断页排查; 低于下限→核查测量有效性.

误差模型:
  静态:  σ² = σ_cam² + σ_quant²(r) + σ_thermal²(t) + σ_drift²(t) + σ_vib² + σ_light²
  动态:  σ_动态² = σ_静态² + σ_到位² + σ_导轨²(行程) + σ_间隙²(工况) + σ_稳定残余²
  时间基: 漂移类参数以 nm/min 计经采样周期换算 (静态帧周期ms级, 动态循环周期s级,
          故同样漂移系数下动态漂移贡献显著更大 — 框架自动体现).
  标记离心 r: 边缘像差退化亚像素精度(入3σ); 微旋残差×r、畸变 → 系统偏置(准确度项).
"""

import numpy as np
from simcore import (SimContext, GaussianNoise, UniformQuantization,
                     RandomWalkDrift, LinearDrift, Vibration, ConstantBias,
                     Backlash, RailStraightness, SettlingResidual)
from simgui import BaseSimApp, launch, FONT


# ===================================================================
#  模型: 静态/动态共用一套相机误差源, 动态叠加运动台误差源
# ===================================================================

class CameraRepeatModel:

    @staticmethod
    def pixel_size_um(p):
        return p["sensor_pixel_size"] / p["lens_magnification"]

    @staticmethod
    def context(p):
        n = max(30, int(p["num_samples"]))
        if p["test_mode"] == "static":
            dt = p["frame_period_ms"] / 1000.0
        else:
            dt = max(p["cycle_time_s"], 0.01)
        return SimContext(n=n, dt_s=dt,
                          pixel_size_um=CameraRepeatModel.pixel_size_um(p))

    @staticmethod
    def sources(p, ctx):
        ps_nm = ctx.pixel_size_nm
        dyn = p["test_mode"] == "dynamic"

        # ---- 相机/标记 公共分量 ----
        r = float(np.hypot(p["mark_offset_x"], p["mark_offset_y"]))
        r_fov = max(p["fov_half_um"], 1e-9)
        subpix_eff = p["subpixel_accuracy"] * (1.0 + p["edge_degrade_k"] * (r / r_fov) ** 2)
        phi = p["microrot_residual_urad"] * 1e-6
        kd = p["distortion_ppm"] * 1e-6
        srcs = [
            GaussianNoise("相机离线重复性", p["camera_offline_3sigma"] / 3.0,
                          group="相机/算法"),
            UniformQuantization("亚像素量化", 0.5 * subpix_eff * ps_nm,
                                group="相机/算法"),
            GaussianNoise("光源噪声",
                          p["light_noise_pct"] * 0.01 * ps_nm * p["light_couple_k"],
                          group="照明"),
            Vibration("机械振动", p["vib_rms_nm"],
                      sine_amp_nm=p["vib_sine_amp_nm"],
                      sine_freq_hz=p["vib_sine_freq_hz"], group="振动"),
            RandomWalkDrift("热漂移(随机游走)", p["thermal_rw_nm_sqrt_min"],
                            group="环境"),
            LinearDrift("线性漂移(预热)", p["drift_x_nm_min"], p["drift_y_nm_min"],
                        group="环境"),
            ConstantBias("微旋残差×离心距",
                         -phi * p["mark_offset_y"] * 1000.0,
                         phi * p["mark_offset_x"] * 1000.0),
            ConstantBias("径向畸变",
                         kd * p["mark_offset_x"] * 1000.0,
                         kd * p["mark_offset_y"] * 1000.0),
        ]

        # ---- 动态: 叠加运动台分量 ----
        if dyn:
            srcs += [
                GaussianNoise("台到位重复性",
                              p["stage_pos_rep_x"], p["stage_pos_rep_y"],
                              group="运动台"),
                RailStraightness("导轨直线度耦合",
                                 p["rail_x_nm_100mm"], p["rail_y_nm_100mm"],
                                 p["travel_x_mm"], p["travel_y_mm"], group="运动台"),
                Backlash("反向间隙", p["backlash_x_nm"], p["backlash_y_nm"],
                         mode=p["approach_mode"], group="运动台"),
                SettlingResidual("稳定残余振荡", p["settle_amp0_nm"],
                                 p["settle_time_s"], p["settle_tau_s"],
                                 group="运动台"),
            ]
        return srcs


# ===================================================================
#  GUI
# ===================================================================

DYN = {"when": ("test_mode", "dynamic")}
STA = {"when": ("test_mode", "static")}


class CameraRepeatApp(BaseSimApp):
    title = "D2W - 相机对准重复性仿真 (静态FR1/FR2 · 动态FR5/FR6)"
    subtitle = "相机对准重复性 静/动态合并版  |  Camera Alignment Repeatability"
    report_name = "cam_repeat"
    model = CameraRepeatModel

    def schema(self):
        px_fn = lambda p: f"{CameraRepeatModel.pixel_size_um(p):.4f}"

        def dur_fn(p):
            n = max(30, int(p["num_samples"]))
            if p["test_mode"] == "static":
                return f"{n * p['frame_period_ms'] / 1000.0:.1f} s"
            return f"{n * p['cycle_time_s'] / 60.0:.1f} min"

        return [
            ("section", "测试配置"),
            ("radio", "test_mode", "测试模式", "static",
             [("static", "静态 (FR1/FR2)"), ("dynamic", "动态 (FR5/FR6)")]),
            ("radio", "camera_type", "相机类型", "upper",
             [("upper", "上精对准相机"), ("lower", "下精对准相机")]),
            ("entry", "num_samples", "采集帧数/循环数 (≥30)", 50, "次"),
            ("entry", "frame_period_ms", "帧周期", 50.0, "ms", STA),
            ("entry", "cycle_time_s", "单循环周期", 4.0, "s", DYN),
            ("derived", "_dur", "测试总时长 (自动)", "", dur_fn,
             ["num_samples", "frame_period_ms", "cycle_time_s", "test_mode"]),
            ("entry", "seed", "随机种子 (空=随机)", "", ""),
            ("entry", "mc_runs", "蒙特卡洛次数", 500, "次"),
            ("note", "验收判据由模型推导: 运行后在状态栏与[验收参考]页给出建议验收区间 "
                     "[MC P2.5, P97.5], 并与解析RSS交叉校验"),

            ("section", "运动路径与运动台误差 (动态)", DYN),
            ("entry", "travel_x_mm", "X 向行程", 100.0, "mm", DYN),
            ("entry", "travel_y_mm", "Y 向行程", 50.0, "mm", DYN),
            ("radio", "approach_mode", "到位方式", "bidirectional",
             [("bidirectional", "双向到位"), ("unidirectional", "定向逼近")], DYN),
            ("note", "双向到位: 间隙以±h/2交替进入3σ; 定向逼近: 间隙退化为常值偏置(准确度项), "
                     "重复性仅余间隙抖动 — 用本开关量化定向逼近策略的收益", DYN),
            ("entry", "stage_pos_rep_x", "到位重复性 X (1σ)", 25.0, "nm", DYN),
            ("entry", "stage_pos_rep_y", "到位重复性 Y (1σ)", 25.0, "nm", DYN),
            ("entry", "rail_x_nm_100mm", "X导轨直线度→Y耦合", 20.0, "nm/100mm", DYN),
            ("entry", "rail_y_nm_100mm", "Y导轨直线度→X耦合", 20.0, "nm/100mm", DYN),
            ("entry", "backlash_x_nm", "反向间隙 X", 20.0, "nm", DYN),
            ("entry", "backlash_y_nm", "反向间隙 Y", 20.0, "nm", DYN),
            ("entry", "settle_amp0_nm", "到位初始振荡幅值", 100.0, "nm", DYN),
            ("entry", "settle_time_s", "稳定等待时间", 0.3, "s", DYN),
            ("entry", "settle_tau_s", "振荡衰减时间常数", 0.08, "s", DYN),

            ("section", "光学系统与标记"),
            ("entry", "sensor_pixel_size", "传感器像元尺寸", 3.45, "um"),
            ("entry", "lens_magnification", "镜头放大倍率", 6.3, "x"),
            ("derived", "_px", "等效像素尺寸 (自动)", "um/px", px_fn,
             ["sensor_pixel_size", "lens_magnification"]),
            ("entry", "camera_offline_3sigma", "相机离线 3σ", 15.0, "nm"),
            ("entry", "subpixel_accuracy", "亚像素定位精度", 0.01, "px"),
            ("entry", "mark_offset_x", "标记偏离视场中心 X", 0.0, "um"),
            ("entry", "mark_offset_y", "标记偏离视场中心 Y", 0.0, "um"),
            ("entry", "fov_half_um", "视场半径 (半幅)", 150.0, "um"),
            ("entry", "edge_degrade_k", "边缘退化系数 k_edge", 0.5, "-"),
            ("entry", "microrot_residual_urad", "微旋残差 (标定后)", 2.0, "urad"),
            ("entry", "distortion_ppm", "径向畸变残差", 5.0, "ppm"),
            ("note", "标记离心 (a)按边缘像差退化亚像素精度→入3σ; "
                     "(b)经微旋残差/畸变产生系统偏置→准确度项, 诊断页单独报告"),

            ("section", "照明"),
            ("entry", "light_noise_pct", "光源强度噪声", 0.5, "%"),
            ("entry", "light_couple_k", "光强→定位耦合系数", 0.3, "-"),
            ("note", "k为光强波动到定位误差的经验耦合系数, 建议用实测数据反标"),

            ("section", "环境因素 (时间基)"),
            ("entry", "thermal_rw_nm_sqrt_min", "热漂移扩散系数", 8.0, "nm/√min"),
            ("entry", "drift_x_nm_min", "X 线性漂移速率", 0.0, "nm/min"),
            ("entry", "drift_y_nm_min", "Y 线性漂移速率", 0.0, "nm/min"),
            ("entry", "vib_rms_nm", "机械振动 (随机RMS)", 8.0, "nm"),
            ("entry", "vib_sine_amp_nm", "周期振动幅值", 0.0, "nm"),
            ("entry", "vib_sine_freq_hz", "周期振动频率", 0.0, "Hz"),
            ("check", "remove_slope", "启用线性去趋势 (评估纯随机分量)", False),
            ("note", "漂移以时间为基底: 动态循环周期(秒级)≫静态帧周期(毫秒级), "
                     "同样漂移系数下动态漂移贡献显著更大 — 这是动态指标天然宽于静态的原因之一"),
        ]

    # ---- 动态专属: 方向/间隙分析页 (静态模式下显示说明) ----
    def extra_tabs(self):
        return [("方向分析", self._tab_direction)]

    def _tab_direction(self, tab, R):
        import tkinter as tk
        from matplotlib.figure import Figure
        r = R["result"]
        if r["params"]["test_mode"] != "dynamic":
            tk.Label(tab, text="方向分析仅适用于动态模式 (往返运动的奇/偶循环分组,\n"
                               "用于识别双向到位的反向间隙特征)。\n\n"
                               "请将测试模式切换为 [动态 (FR5/FR6)] 后重新运行。",
                     bg=self.C["bg"], fg=self.C["dim"], font=(FONT, 11),
                     justify="center").pack(expand=True)
            return
        key = "proc" if r["detrend"] else "raw"
        x, y = r[key]["x"], r[key]["y"]
        n = len(x)
        odd_idx = np.arange(1, n, 2); even_idx = np.arange(0, n, 2)
        fig = Figure(figsize=(10, 6), dpi=100, facecolor=self.C["bg"])
        for i, (a, lbl, col) in enumerate([(x, "X", self.C["cx"]), (y, "Y", self.C["cy"])]):
            ax = fig.add_subplot(2, 1, i + 1)
            mo, me = np.mean(a[odd_idx]), np.mean(a[even_idx])
            self._ax(ax, f"{lbl}: 奇/偶循环分组 (双向到位的间隙特征)  "
                         f"|Δ均值|={abs(mo - me):.1f} nm", "循环序号", "误差 (nm)")
            ax.plot(even_idx + 1, a[even_idx], "o-", color=col, ms=3, lw=0.8,
                    alpha=0.85, label=f"正向到位 均值={me:.1f}")
            ax.plot(odd_idx + 1, a[odd_idx], "s--", color="#e5a00d", ms=3, lw=0.8,
                    alpha=0.85, label=f"反向到位 均值={mo:.1f}")
            ax.axhline(me, color=col, lw=0.8, ls=":", alpha=0.6)
            ax.axhline(mo, color="#e5a00d", lw=0.8, ls=":", alpha=0.6)
            ax.legend(fontsize=8)
        fig.tight_layout(pad=2.0)
        self._embed(fig, tab)

    def report_header_lines(self, R):
        p = R["result"]["params"]
        mode = "Static (FR1/FR2)" if p["test_mode"] == "static" else "Dynamic (FR5/FR6)"
        lines = [f"Mode: {mode}   Camera: {'Upper' if p['camera_type'] == 'upper' else 'Lower'}",
                 f"Sensor: {p['sensor_pixel_size']} um / {p['lens_magnification']}x = "
                 f"{CameraRepeatModel.pixel_size_um(p):.4f} um/px"]
        if p["test_mode"] == "dynamic":
            lines.append(f"Approach: {'双向到位' if p['approach_mode'] == 'bidirectional' else '定向逼近'}, "
                         f"Travel X={p['travel_x_mm']} mm Y={p['travel_y_mm']} mm, "
                         f"cycle={p['cycle_time_s']} s")
        return lines


if __name__ == "__main__":
    launch(CameraRepeatApp)
