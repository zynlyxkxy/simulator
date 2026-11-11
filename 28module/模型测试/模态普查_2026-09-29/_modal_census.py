# -*- coding: utf-8 -*-
"""整机模态普查实验（迭代 57，9010 实机后端）：
工艺流程执行期间，逐模态（21 轴 SISO + 9 整机 MIMO）响应观测 +
全轴激励扫描补盲区 + scope FFT 频谱自洽性校验 → CSV/图/报告。

相位：P0 准备（模态 ON/物料/基线）→ P1 单颗链（chain）全程采样 →
P2 逐轴 jog 激励扫描 → P3 scope FFT（WS 阶跃 + BS 阶跃两事件窗）→
P4 还原。产出到 模型测试/模态普查_2026-09-29/。"""
import json
import math
import os
import time
import urllib.request

BASE = "http://127.0.0.1:9010"
OUT = os.path.join("模型测试", "模态普查_2026-09-29")
SAMPLE_HZ = 10.0

AXES = ["WS_LS_X", "WS_LS_Y", "WS_Rz", "E_EPIN_Z_1", "E_EPIN_Z_2",
        "E_EPIN_Z_3", "BS_Arm_Z", "BS_CAL_Rx", "BS_CAL_Ry", "AS_X", "AS_Y",
        "AS_Z", "AS_Rx", "AS_Ry", "FA_X", "FA_Z", "FA_Rx", "CH_Rz",
        "ES_X", "ES_Y", "ES_PunchZ"]
ROTARY = {"WS_Rz", "BS_CAL_Rx", "BS_CAL_Ry", "AS_Rx", "AS_Ry", "FA_Rx",
          "CH_Rz"}
MODES = ["granite_sway_x", "granite_sway_y", "gantry_yaw", "gantry_sway_y",
         "head_z_bounce", "align_bridge", "frame_film", "wafer_stage_rock",
         "flip_arm_sway"]


def http(path, payload=None, timeout=30):
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def move(axis, target, speed=None, tol=0.005, timeout=40.0):
    req = {"axis": axis, "target": target}
    if speed:
        req["speed"] = speed
    http("/api/axis/move", req)
    t0 = time.time()
    while time.time() - t0 < timeout:
        a = next(x for x in http("/api/axes")["axes"] if x["name"] == axis)
        if abs(a["pos"] - target) <= tol and abs(a.get("vel", 0.0)) < 0.01:
            return
        time.sleep(0.15)
    raise RuntimeError("settle 超时 %s → %s" % (axis, target))


def modal_state():
    return http("/api/modal/state")


def amp_of(st, name, is_mode):
    e = (st["modes"] if is_mode else st["axes"])[name]
    return e["amp"]          # 原生单位（mm/rad）


def main():
    os.makedirs(OUT, exist_ok=True)
    log = open(os.path.join(OUT, "run.log"), "w", encoding="utf-8")

    def pr(*a):
        print(*a)
        log.write(" ".join(str(x) for x in a) + "\n")
        log.flush()

    h0 = http("/api/health")
    pr("health tick=%.1f push=%.1f" % (h0["tick_hz"], h0["push_hz"]))
    m0 = http("/api/modal")
    pr("modal 参数：floor=%s axes=%d modes=%d"
       % (m0["params"]["floor"]["level"], len(m0["params"]["axes"]),
          len(m0["params"]["modes"])))
    st_ax = http("/api/axes")
    pos0 = {a["name"]: a["pos"] for a in st_ax["axes"]}
    amp_lim = {a["name"]: a["limits"] for a in st_ax["axes"]}
    vmax = {a["name"]: a["config"]["vmax"] for a in st_ax["axes"]}
    amax = {a["name"]: a["config"]["amax"] for a in st_ax["axes"]}
    fn_ax = {k: v["fn_hz"] for k, v in m0["params"]["axes"].items()}
    zeta_ax = {k: v["zeta"] for k, v in m0["params"]["axes"].items()}
    gain_ax = {k: v.get("input_gain", 0.05) for k, v in
               m0["params"]["axes"].items()}
    fn_md = {k: v["fn_hz"] for k, v in m0["params"]["modes"].items()}
    zeta_md = {k: v["zeta"] for k, v in m0["params"]["modes"].items()}
    inputs_md = {k: dict(v.get("inputs") or {}) for k, v in
                 m0["params"]["modes"].items()}
    contact_md = {k: bool(v.get("contact")) for k, v in
                  m0["params"]["modes"].items()}
    outputs_md = {k: dict(v.get("outputs") or {}) for k, v in
                  m0["params"]["modes"].items()}

    # ---- P0：物料 + 模态 ON + 视觉 FAST ----
    http("/api/vision/mode", {"mode": "fast"})
    # 物料强制全新（链是状态感知跳过——上一轮的成品会让本轮全 skip）：
    # 先卸后上，保证 P1 的链每个阶段真跑
    for path in ("/api/workpiece/wafer", "/api/workpiece/film"):
        try:
            http(path, {"action": "unload"})
        except Exception:
            pass
    http("/api/workpiece/wafer", {"action": "load", "offset": [0.0, 0.0],
                                  "rz": 0.0})
    http("/api/workpiece/film", {"action": "load"})
    pr("物料：晶圆+蓝膜全新上片")
    # EGA 直跑一遍强制刷新（EGA 态不随上片失效——残留对准会让链内
    # fine_align judge 失败；链里 EGA 显示 skip 属正常）
    http("/api/process/wafer_ega", {})
    # 先等它真起来再等它收（异步启动竞态：首poll可能落在 active 置位前）
    t0 = time.time()
    while time.time() - t0 < 10.0:
        if http("/api/flow/state").get("active"):
            break
        time.sleep(0.2)
    t0 = time.time()
    while time.time() - t0 < 180.0:
        if not http("/api/flow/state").get("active"):
            break
        time.sleep(0.3)
    pr("EGA 直跑完成")
    http("/api/modal", {"enabled": True})
    pr("模态 ON（floor=typical、contact_pulse on、热软化 on）")

    timeline = []            # (sim_t, stage, {axes amp}, {modes amp})
    try:
        # ---- 地板基线（全静止 12s） ----
        pr("P0 地板基线采样 12s…")
        t0 = time.time()
        while time.time() - t0 < 12.0:
            h = http("/api/health")
            st = modal_state()
            timeline.append((h["sim_time"], "__floor__", st))
            time.sleep(1.0 / SAMPLE_HZ)

        # ---- P1：单颗链全程采样 ----
        pr("P1 启动单颗链（EGA→取片→交接→调焦调平→精对准）…")
        http("/api/process/chain", {})
        t0 = time.time()
        last_cur = None
        while True:
            h = http("/api/health")
            fl = http("/api/flow/state")
            ch = fl.get("chain") or {}
            cur = ch.get("current") or ""
            if cur != last_cur:
                pr("   [链] 阶段 → %s（sim_t=%.1f）" % (cur, h["sim_time"]))
                last_cur = cur
            st = modal_state()
            timeline.append((h["sim_time"], cur, st))
            if not ch.get("active"):
                pr("   [链] 结束 stages=%s error=%s"
                   % (ch.get("stages"), ch.get("error")))
                # bond 兜底：judge 偶发失败时直跑 fine_align→bond 补齐
                # （接触脉冲是本实验关键激励，必须真打一次）
                if (ch.get("stages") or {}).get("bond") == "fail":
                    pr("   [链] bond 兜底：直跑 fine_align→bond")
                    for step in ("/api/process/fine_align",
                                 "/api/process/bond"):
                        try:
                            http(step, {})
                        except Exception as ex:
                            pr("   [兜底] %s 启动失败: %s" % (step, ex))
                            break
                        tw = time.time()
                        while time.time() - tw < 10.0:
                            if http("/api/flow/state").get("active"):
                                break
                            time.sleep(0.2)
                        tw = time.time()
                        while time.time() - tw < 180.0:
                            st2 = http("/api/flow/state")
                            timeline.append((http("/api/health")["sim_time"],
                                             "bond_retry", modal_state()))
                            if not st2.get("active"):
                                break
                            time.sleep(0.2)
                break
            if time.time() - t0 > 600:
                pr("   [链] 超时 600s 中止观察")
                break
            time.sleep(1.0 / SAMPLE_HZ)
        time.sleep(3.0)      # 链后余振采样
        for _ in range(int(3 * SAMPLE_HZ)):
            h = http("/api/health")
            timeline.append((h["sim_time"], "__settle__", modal_state()))
            time.sleep(1.0 / SAMPLE_HZ)

        # ---- P3a：bond 接触后 scope 波形已被链内覆盖（链在 1× 跑），
        #      另起两个合成激励事件抓 FFT ----
        pr("P3 scope 事件采集…")
        scope_sets = {}
        # 事件 A：WS_LS_X 快速往返（激励 granite_sway_x + WS_LS_X 轴模态）
        move("WS_LS_X", pos0.get("WS_LS_X", 0.0) + 2.0,
             speed=vmax["WS_LS_X"])
        time.sleep(0.9)                      # 让 8s 环窗含激励段
        move("WS_LS_X", pos0.get("WS_LS_X", 0.0), speed=vmax["WS_LS_X"])
        time.sleep(0.9)
        sc = http("/api/modal/scope?ch=0&n=8000")
        sc2 = http("/api/modal/scope?ch=1&n=8000")
        scope_sets["ws_step"] = (sc, sc2)
        time.sleep(8.5)                       # 排空窗
        # 事件 B：BS_Arm_Z 阶跃（激励 head_z_bounce + BS_Arm_Z 轴模态）
        move("BS_Arm_Z", -5.0, speed=vmax["BS_Arm_Z"])
        time.sleep(0.9)
        move("BS_Arm_Z", pos0.get("BS_Arm_Z", 0.0), speed=vmax["BS_Arm_Z"])
        time.sleep(0.9)
        sc3 = http("/api/modal/scope?ch=2&n=8000")
        sc4 = http("/api/modal/scope?ch=3&n=8000")
        scope_sets["bs_step"] = (sc3, sc4)
        time.sleep(8.5)

        # ---- P2：逐轴 jog 激励扫描（移动全程 20Hz 采样覆盖加速段+余振） ----
        pr("P2 全轴激励扫描（21 轴逐轴 ±小行程往返）…")
        scan = {}            # axis -> [(sim_t, amp)]
        for ax in AXES:
            p0 = pos0.get(ax, 0.0)
            lo, hi = amp_lim[ax]
            d = 0.002 if ax in ROTARY else 1.0
            d = min(d, (hi - lo) * 0.1)       # 行程 10% 以内
            tgt1 = max(lo + 0.001, min(hi - 0.001, p0 + d))
            v = max(0.5, vmax[ax] * 0.5)      # 半速（加速度激励足够大）
            rec = []
            for tgt in (tgt1, p0):
                http("/api/axis/move", {"axis": ax, "target": tgt,
                                        "speed": v})
                t0 = time.time()
                while time.time() - t0 < 1.2:
                    h = http("/api/health")
                    st = modal_state()
                    rec.append((h["sim_time"], amp_of(st, ax, False)))
                    time.sleep(0.05)
                # 等位（防叠下一步）
                tw = time.time()
                while time.time() - tw < 15.0:
                    a = next(x for x in http("/api/axes")["axes"]
                             if x["name"] == ax)
                    if abs(a["pos"] - tgt) <= 0.005 \
                            and abs(a.get("vel", 0.0)) < 0.01:
                        break
                    time.sleep(0.1)
            # 余振尾
            for _ in range(3):
                h = http("/api/health")
                st = modal_state()
                rec.append((h["sim_time"], amp_of(st, ax, False)))
                time.sleep(0.2)
            scan[ax] = rec
            pr("   轴 %s: jog ±%.4f 峰值 amp=%.3f %s"
               % (ax, d, max(r[1] for r in rec) * 1e6,
                  "µrad" if ax in ROTARY else "µm"))
    finally:
        # ---- P4 还原 ----
        http("/api/modal", {"enabled": False})
        http("/api/vision/mode", {"mode": "real"})
        for ax in ("BS_Arm_Z", "AS_Z", "AS_Y", "AS_X", "WS_LS_Y", "WS_LS_X",
                   "WS_Rz", "FA_X", "FA_Z", "FA_Rx", "CH_Rz",
                   "E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3",
                   "ES_X", "ES_Y", "ES_PunchZ", "BS_CAL_Rx", "BS_CAL_Ry"):
            try:
                move(ax, pos0.get(ax, 0.0))
            except Exception as ex:
                pr("还原 %s 失败: %s" % (ax, ex))
        pr("已还原（模态 OFF/视觉 real/轴位）")

    # ================= 分析 =================
    pr("分析中…")
    # 地板基线（模态）：__floor__ 窗各 mode rms 均值
    floor_base = {m: 0.0 for m in MODES}
    fl = [s for (_t, ph, s) in timeline if ph == "__floor__"]
    if fl:
        for m in MODES:
            floor_base[m] = sum(s["modes"][m]["rms_mm"] for s in fl) / len(fl)
    # 链阶段切段统计（每模态/轴：全程 rms 均值、峰值 amp、主导阶段）
    stats = {}
    chain_rows = [(t, ph, s) for (t, ph, s) in timeline
                  if ph not in ("__floor__",)]
    for m in MODES:
        rms_all = [s["modes"][m]["rms_mm"] for (_t, _p, s) in chain_rows]
        amp_all = [s["modes"][m]["amp_mm"] for (_t, _p, s) in chain_rows]
        # 主导阶段：分阶段 rms 均值最大者
        by_ph = {}
        for (_t, ph, s) in chain_rows:
            by_ph.setdefault(ph, []).append(s["modes"][m]["rms_mm"])
        dom = max(by_ph.items(), key=lambda kv: sum(kv[1]) / len(kv[1]))[0] \
            if by_ph else ""
        stats[m] = {"rms": (sum(rms_all) / len(rms_all)) if rms_all else 0.0,
                    "peak": max(amp_all) if amp_all else 0.0,
                    "dom_phase": dom,
                    "floor_rms": floor_base[m]}
    ax_stats = {}
    for a in AXES:
        rms_all = [s["axes"][a]["rms"] for (_t, _p, s) in chain_rows]
        amp_all = [s["axes"][a]["amp"] for (_t, _p, s) in chain_rows]
        jog_peak = max((r[1] for r in scan.get(a, [])), default=0.0)
        ax_stats[a] = {
            "rms": (sum(rms_all) / len(rms_all)) if rms_all else 0.0,
            "peak": max(amp_all) if amp_all else 0.0,
            "jog_peak": jog_peak,
        }

    # ---- CSV ----
    import csv as _csv
    with open(os.path.join(OUT, "modal_census.csv"), "w", newline="",
              encoding="utf-8") as f:
        w = _csv.writer(f)
        w.writerow(["kind", "name", "fn_hz", "zeta", "unit",
                    "chain_rms", "chain_peak", "jog_peak",
                    "floor_rms", "dom_phase", "excitation",
                    "outputs", "量级判定"])
        for a in AXES:
            s = ax_stats[a]
            k = 1e6 if a in ROTARY else 1e3    # rad→µrad / mm→µm
            w.writerow(["axis", a, fn_ax[a], zeta_ax[a],
                        "urad" if a in ROTARY else "um",
                        s["rms"] * k, s["peak"] * k,
                        s["jog_peak"] * k, "", "", "自身加速度激励", "",
                        ""])
        for m in MODES:
            s = stats[m]
            w.writerow(["mode", m, fn_md[m], zeta_md[m], "um",
                        s["rms"] * 1e3, s["peak"] * 1e3, "",
                        s["floor_rms"] * 1e3, s["dom_phase"],
                        "/".join(inputs_md[m].keys())
                        + ("+contact" if contact_md[m] else ""),
                        "/".join(outputs_md[m].keys()), ""])
    pr("CSV 写出 modal_census.csv")

    # ---- 图 1：链时间线（6 代表模态 amp） ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    show = ["granite_sway_x", "gantry_yaw", "head_z_bounce", "align_bridge",
            "wafer_stage_rock", "flip_arm_sway"]
    fig, ax = plt.subplots(figsize=(10, 4.6))
    tt = [(t - timeline[0][0]) for (t, _p, _s) in timeline]
    for m in show:
        ax.plot(tt, [s["modes"][m]["amp_mm"] * 1e3
                     for (_t, _p, s) in timeline], lw=0.8, label=m)
    # 阶段边界
    last = None
    for (t, ph, _s) in timeline:
        if ph != last and ph not in ("__floor__", "__settle__", ""):
            ax.axvline(t - timeline[0][0], color="gray", lw=0.4, alpha=0.5)
            ax.text(t - timeline[0][0], ax.get_ylim()[1] * 0.9, ph,
                    rotation=90, fontsize=6, va="top")
            last = ph
    ax.set_yscale("log")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("mode amp (um, log)")
    ax.set_title("machine modes during single-die chain (floor=typical)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "modal_timeline.png"), dpi=130)

    # ---- 图 2：普查条形图（轴 jog 峰值 + 模态链峰值） ----
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6))
    x = np.arange(len(AXES))
    ax1.bar(x - 0.2, [ax_stats[a]["jog_peak"] * (1e6 if a in ROTARY else 1e6)
                      for a in AXES], width=0.4, label="jog peak")
    ax1.bar(x + 0.2, [ax_stats[a]["peak"] * (1e6 if a in ROTARY else 1e6)
                      for a in AXES], width=0.4, label="chain peak")
    ax1.set_xticks(x)
    ax1.set_xticklabels(AXES, rotation=90, fontsize=6)
    ax1.set_yscale("log")
    ax1.set_ylabel("amp (nm or nrad, log)")
    ax1.set_title("axis SISO modes")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3, which="both", axis="y")
    x2 = np.arange(len(MODES))
    ax2.bar(x2 - 0.2, [stats[m]["peak"] * 1e3 for m in MODES],
            width=0.4, label="chain peak (um)")
    ax2.bar(x2 + 0.2, [max(stats[m]["floor_rms"], 1e-9) * 1e3 for m in MODES],
            width=0.4, label="floor rms (um)")
    ax2.set_xticks(x2)
    ax2.set_xticklabels(MODES, rotation=90, fontsize=6)
    ax2.set_yscale("log")
    ax2.set_title("machine MIMO modes")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3, which="both", axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "modal_census_bar.png"), dpi=130)

    # ---- 图 3：scope FFT ----
    fig, axes_ = plt.subplots(4, 2, figsize=(10, 10))
    fft_rows = []
    for row, (tag, pair) in enumerate([("ws_step", scope_sets["ws_step"]),
                                       ("bs_step", scope_sets["bs_step"])]):
        for j, sc in enumerate(pair):
            data = [v for v in sc.get("data", []) if v is not None]
            r = row * 2 + j
            axw = axes_[r][0]
            axf = axes_[r][1]
            if not data:
                continue
            t_ax = [i * sc["dt"] for i in range(len(data))]
            axw.plot(t_ax, data, lw=0.5)
            axw.set_title("%s  %s" % (tag, sc["label"]), fontsize=8)
            axw.grid(alpha=0.3)
            arr = np.asarray(data, dtype=float)
            arr = arr - arr.mean()
            win = np.hanning(len(arr))
            sp = np.abs(np.fft.rfft(arr * win))
            fr = np.fft.rfftfreq(len(arr), d=sc["dt"])
            band = (fr >= 5) & (fr <= 200)
            pk = int(np.argmax(sp[band]))
            f_peak = float(fr[band][pk])
            axf.plot(fr[band], sp[band], lw=0.8)
            axf.axvline(f_peak, color="r", ls="--", lw=0.8,
                        label="peak %.1fHz" % f_peak)
            # 配置 fn 参考线
            lbl = sc["label"]
            cfg_fn = None
            if lbl.startswith("axis:"):
                cfg_fn = fn_ax.get(lbl[5:])
            elif lbl.startswith("mode:"):
                cfg_fn = fn_md.get(lbl[5:])
            if cfg_fn:
                axf.axvline(cfg_fn, color="g", ls=":", lw=1.0,
                            label="cfg fn %.1fHz" % cfg_fn)
                fft_rows.append((lbl, cfg_fn, f_peak))
            axf.legend(fontsize=7)
            axf.set_xlabel("Hz")
            axf.grid(alpha=0.3)
    fig.suptitle("scope waveforms + FFT (1kHz x 8s)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "modal_scope_fft.png"), dpi=130)
    pr("图三件已出（timeline/census_bar/scope_fft）")
    for lbl, cfg, pk in fft_rows:
        pr("   FFT 自洽：%s cfg=%.1fHz peak=%.1fHz" % (lbl, cfg, pk))

    # ---- 报告辅助数据落盘（REPORT.md 由主代理写） ----
    with open(os.path.join(OUT, "_analysis.json"), "w",
              encoding="utf-8") as f:
        json.dump({
            "floor_rms_um": m0["params"]["floor"]["level"],
            "floor_baseline_modes_um": {m: floor_base[m] * 1e3
                                        for m in MODES},
            "axes": {a: {"fn": fn_ax[a], "zeta": zeta_ax[a],
                         "rotary": a in ROTARY,
                         "chain_rms": ax_stats[a]["rms"] * 1e6,
                         "chain_peak": ax_stats[a]["peak"] * 1e6,
                         "jog_peak": ax_stats[a]["jog_peak"] * 1e6}
                     for a in AXES},
            "modes": {m: {"fn": fn_md[m], "zeta": zeta_md[m],
                          "chain_rms_um": stats[m]["rms"] * 1e3,
                          "chain_peak_um": stats[m]["peak"] * 1e3,
                          "floor_rms_um": floor_base[m] * 1e3,
                          "dom_phase": stats[m]["dom_phase"],
                          "inputs": inputs_md[m],
                          "contact": contact_md[m],
                          "outputs": {k: v for k, v in
                                      outputs_md[m].items()}}
                     for m in MODES},
            "fft_rows": fft_rows,
        }, f, ensure_ascii=False, indent=1)
    pr("分析数据 _analysis.json 落盘")
    h1 = http("/api/health")
    pr("结束 health tick=%.1f push=%.1f" % (h1["tick_hz"], h1["push_hz"]))
    log.close()


if __name__ == "__main__":
    main()
