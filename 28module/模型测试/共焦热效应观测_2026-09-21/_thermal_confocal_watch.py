# -*- coding: utf-8 -*-
"""光谱共焦热效应观测（9010 实机后端）：下共焦打在晶圆面，
头 300℃ + 晶圆吸盘 110℃，持续采样读数/z_surface/z_focus/节点温度。
输出 Temp/_thermal_confocal_watch.csv + .png；结束全还原。"""
import json
import math
import time
import urllib.request

BASE = "http://127.0.0.1:9010"
SAMPLES = 60                 # 60 拍 × 2.5s 墙钟 ≈ 2.5min 墙钟 ≈ 20min sim
PERIOD_S = 2.5
SPEED = 8.0


def http(path, payload=None, timeout=30):
    if payload is None:
        with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def put_thermal(patch):
    data = json.dumps(patch).encode("utf-8")
    req = urllib.request.Request(BASE + "/api/thermal", data=data,
                                 method="PUT",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode("utf-8"))


def main():
    tj0 = http("/api/thermal")
    wave0 = ((tj0.get("params") or {}).get("env") or {}).get("wave", "typical")
    st_ax = http("/api/axes")
    pos0 = {a["name"]: a["pos"] for a in st_ax["axes"]}
    try:
        http("/api/workpiece/wafer", {"action": "load",
                                      "offset": [0.0, 0.0], "rz": 0.0})
        print("晶圆新上片（0 偏置）")
    except Exception:
        print("晶圆已在位，沿用")
    http("/api/axis/move", {"axis": "AS_X", "target": 0.0, "speed": 200.0})
    http("/api/axis/move", {"axis": "AS_Y", "target": 6.0, "speed": 200.0})
    # AS_Z=0：焦面落在晶圆面（885.9=吸盘面 884.9+片厚 1.0），共焦在量程内
    # （AS_Z=-1 焦面在裸吸盘面，晶圆面 +1mm 超量程读数 None）
    http("/api/axis/move", {"axis": "AS_Z", "target": 0.0, "speed": 50.0})
    http("/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    time.sleep(2.5)

    put_thermal({"env": {"wave": "off"}})
    http("/api/thermal", {"enabled": True})
    http("/api/io/command", {"path": "SystemMachine.PulseHeat_Up",
                             "cmd": "SetTargetTempWithPlan",
                             "args": {"Temperature": 300.0, "Steady_Time": 0}})
    http("/api/io/command", {"path": "SystemMachine.Heat_Dw",
                             "cmd": "SetTargetTempWithPlan",
                             "args": {"Temperature": 110.0, "Steady_Time": 0}})
    http("/api/speed", {"scale": SPEED})

    rows = []
    t_start = time.time()
    try:
        for k in range(SAMPLES):
            t_loop = time.time()
            rd = http("/api/confocal/read?probe=down")
            th = http("/api/thermal/state")
            nd = th["nodes"]
            h = http("/api/health")
            rows.append({
                "k": k, "t_wall": round(time.time() - t_start, 2),
                "sim_t": round(h.get("sim_time", 0.0), 2),
                "reading_um": rd.get("reading_um"),
                "valid": rd.get("valid"),
                "z_surface": rd.get("z_surface"),
                "z_focus": rd.get("z_focus"),
                "T_head": nd["bond_head"]["t_c"],
                "T_chuck": nd["wafer_chuck"]["t_c"],
                "T_optics": nd["align_optics"]["t_c"],
                "T_align": nd["align_stage"]["t_c"],
                "T_granite_up": nd["granite_up"]["t_c"],
            })
            if k % 6 == 0:
                print("k=%2d sim=%7.1fs read=%s z_s=%s z_f=%s T盘=%5.1f T镜=%6.3f"
                      % (k, rows[-1]["sim_t"], rd.get("reading_um"),
                         rd.get("z_surface"), rd.get("z_focus"),
                         nd["wafer_chuck"]["t_c"], nd["align_optics"]["t_c"]))
            dt = PERIOD_S - (time.time() - t_loop)
            if dt > 0:
                time.sleep(dt)
    finally:
        http("/api/speed", {"scale": 1.0})
        http("/api/io/command",
             {"path": "SystemMachine.PulseHeat_Up",
              "cmd": "SetTargetTempWithPlan",
              "args": {"Temperature": 25.0, "Steady_Time": 0}})
        http("/api/io/command",
             {"path": "SystemMachine.Heat_Dw",
              "cmd": "SetTargetTempWithPlan",
              "args": {"Temperature": 30.0, "Steady_Time": 0}})
        http("/api/thermal", {"enabled": False})
        put_thermal({"env": {"wave": wave0}})
        for ax in ("BS_Arm_Z", "AS_Z", "AS_Y", "AS_X"):
            http("/api/axis/move", {"axis": ax,
                                    "target": pos0.get(ax, 0.0),
                                    "speed": 200.0})
        print("已还原")

    csv_path = "Temp/_thermal_confocal_watch.csv"
    keys = list(rows[0].keys())
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r[k]) for k in keys) + "\n")
    print("CSV:", csv_path, "rows=%d" % len(rows))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = rows[0]["sim_t"]
    X = [(r["sim_t"] - t0) / 60.0 for r in rows]
    rdg = [r["reading_um"] if r["reading_um"] is not None else float("nan")
           for r in rows]
    base = next((v for v in rdg if not math.isnan(v)), 0.0)
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.plot(X, [v - base for v in rdg], ".-", ms=4, lw=0.9,
            label="confocal reading - baseline (um)")
    ax.set_xlabel("sim time (min)")
    ax.set_ylabel("reading drift (um)")
    ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=8)
    ax.set_title("down confocal on wafer surface: head 300C + chuck 110C",
                 fontsize=10)
    ax2 = ax.twinx()
    ax2.plot(X, [r["T_chuck"] for r in rows], "m-", alpha=0.4,
             label="T_chuck")
    ax2.plot(X, [r["T_head"] for r in rows], "r-", alpha=0.35,
             label="T_head")
    ax2.plot(X, [r["T_optics"] for r in rows], "c-", alpha=0.5,
             label="T_optics")
    ax2.set_ylabel("T (C)"); ax2.legend(loc="lower right", fontsize=7)
    fig.tight_layout()
    png_path = "Temp/_thermal_confocal_watch.png"
    fig.savefig(png_path, dpi=130)
    print("PNG:", png_path)

    zs = [r["z_surface"] for r in rows if r["z_surface"] is not None]
    zf = [r["z_focus"] for r in rows if r["z_focus"] is not None]
    vals = [v for v in rdg if not math.isnan(v)]
    print("== 摘要 ==")
    if vals:
        print("reading min=%.4f max=%.4f um (span %.4f)"
              % (min(vals), max(vals), max(vals) - min(vals)))
    else:
        print("reading 全程 None（超量程/无有效读数）")
    if zs and zf:
        print("z_surface 首末: %.6f → %.6f (Δ %.1f nm)"
              % (zs[0], zs[-1], (zs[-1] - zs[0]) * 1e6))
        print("z_focus   首末: %.6f → %.6f (Δ %.1f nm)"
              % (zf[0], zf[-1], (zf[-1] - zf[0]) * 1e6))


if __name__ == "__main__":
    main()
