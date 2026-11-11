# -*- coding: utf-8 -*-
"""双相机热漂移观测（迭代 55 辐射+热变形链整机演示，9010 实机后端）：
上相机看吸盘 head_chuck_cross 标记、下相机看晶圆 site quad 标记，
脉冲加热头 300℃ + 晶圆吸盘 110℃，持续采样两相机视场中标记像素偏移。

要点：
- FAST 模式（truth 链与 REAL 同一——热变形经 6DOF 注入真值，两模式一致；
  采样快、噪声 0.1px=60nm ≪ 漂移信号）。
- 上相机重聚焦：头 300℃ 热伸长 ~1mm（head_z_growth）会使标记失焦，
  每拍按 thermal.state deform.bond_head.t_mm[2] 抬 BS_Arm_Z 补偿
  （真机标准操作；只动头 Z，不影响下相机，X/Y 漂移测量不受影响）。
- 结束还原：速度/温度/热开关/视觉模式/轴位/晶圆（若本次新上片）。
输出：Temp/_thermal_drift_watch.csv + .png（漂移-温度趋势）。"""
import json
import math
import time
import urllib.request

BASE = "http://127.0.0.1:9010"
SAMPLES = 90                 # 90 拍 × 2.5s 墙钟 ≈ 3.75min 墙钟 ≈ 30min sim
PERIOD_S = 2.5
SPEED = 8.0
K_NM_PX = 600.0              # up/down 相机 0.6µm/px

UP_CAM, UP_TPL = "up", "tpl_head_chuck"
DN_CAM, DN_TPL = "down", "cv_quad"
WS_ANCHOR = (-46.197, 10.799)  # site_R14C14#1 quad 标记在下相机视场中心
                               # （EGA 锚位 (-42.394,7.0) 最近标记在 ±3.8mm
                               # FOV 外，探针实测此位 col/row=1500,1500）
AS_ANCHOR = (0.0, 6.0, -1.0)  # 上相机对准吸盘心
BS_UPFOCUS = -119.83         # 上焦面基准（裸吸盘面 @AS_Z=-1）


def http(path, payload=None, timeout=30):
    if payload is None:
        with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def move(axis, target, speed=200.0, tol=0.005, timeout=30.0):
    http("/api/axis/move", {"axis": axis, "target": target, "speed": speed})
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = http("/api/axes")
        a = next(x for x in st["axes"] if x["name"] == axis)
        if abs(a["pos"] - target) <= tol and abs(a.get("vel", 0.0)) < 0.01:
            return
        time.sleep(0.2)
    raise RuntimeError("wait_settled 超时: %s → %s" % (axis, target))


def shot(cam, tpl):
    """识别 → (col,row,score,judge,sim_t)；失败返回 None 五元。"""
    try:
        e = http("/api/vision/identify", {"cam": cam, "template": tpl},
                 timeout=40)
    except Exception as ex:
        print("   [%s] identify 异常: %s" % (cam, ex))
        return None
    res = (e.get("entry") or {}).get("result") or {}
    cols = res.get("lstCol") or []
    if not cols:
        print("   [%s] 无结果条目（judge=%s error=%s）"
              % (cam, res.get("judge"), (e.get("entry") or {}).get("error")))
        return None
    scores = res.get("lstScore") or [0.0] * len(cols)
    i = max(range(len(cols)), key=lambda k: scores[k])
    return (cols[i], res["lstRow"][i], scores[i],
            bool(res.get("judge")), (e.get("entry") or {}).get("sim_t"))


def main():
    # ---- 记录初始状态（晶圆先探活：已在位则沿用不上片） ----
    loaded_here = False
    try:
        http("/api/workpiece/wafer", {"action": "load",
                                      "offset": [0.0, 0.0], "rz": 0.0})
        loaded_here = True
    except Exception:
        print("晶圆已在位，沿用（不上片）")
    st_ax = http("/api/axes")
    pos0 = {a["name"]: a["pos"] for a in st_ax["axes"]}
    tj0 = http("/api/thermal")
    wave0 = ((tj0.get("params") or {}).get("env") or {}).get("wave", "typical")
    http("/api/vision/mode", {"mode": "fast"})
    print("初始：wafer loaded_here=%s" % loaded_here)

    # ---- 机位 ----
    move("WS_LS_X", WS_ANCHOR[0], speed=300.0)
    move("WS_LS_Y", WS_ANCHOR[1], speed=300.0)
    move("AS_X", AS_ANCHOR[0]); move("AS_Y", AS_ANCHOR[1])
    move("AS_Z", AS_ANCHOR[2], speed=50.0)
    move("BS_Arm_Z", BS_UPFOCUS, speed=100.0)

    up0 = shot(UP_CAM, UP_TPL)
    dn0 = shot(DN_CAM, DN_TPL)
    if up0 is None or dn0 is None:
        raise RuntimeError("基线识别失败 up=%s down=%s" % (up0, dn0))
    print("基线：up col=%.1f row=%.1f score=%.3f | down col=%.1f row=%.1f "
          "score=%.3f" % (up0[0], up0[1], up0[2], dn0[0], dn0[1], dn0[2]))

    # ---- 加热：热仿真 ON + 头 300 + 吸盘 110 + 8× ----
    http("/api/thermal", None)  # 探活
    import urllib.request as _u
    def put_thermal(patch):
        data = json.dumps(patch).encode("utf-8")
        req = _u.Request(BASE + "/api/thermal", data=data, method="PUT",
                         headers={"Content-Type": "application/json"})
        with _u.urlopen(req, timeout=30) as r:
            return json.loads(r.read().decode("utf-8"))
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
            th = http("/api/thermal/state")
            nd = th["nodes"]
            dz_bh = th["deform"]["bond_head"]["t_mm"][2]
            # 上相机重聚焦：头 Z 热伸长用头 Z 轴补回（dz<0 伸长向下→抬头）
            bs_tgt = BS_UPFOCUS - dz_bh
            cur_bs = next(a for a in http("/api/axes")["axes"]
                          if a["name"] == "BS_Arm_Z")
            if abs(cur_bs["pos"] - bs_tgt) > 0.02:
                move("BS_Arm_Z", bs_tgt, speed=50.0, tol=0.005)
            up = shot(UP_CAM, UP_TPL)
            dn = shot(DN_CAM, DN_TPL)
            sim_t = (up or dn or (0, 0, 0, 0, None))[4]
            rows.append({
                "k": k, "t_wall": round(time.time() - t_start, 2),
                "sim_t": sim_t,
                "T_head": nd["bond_head"]["t_c"],
                "T_chuck": nd["wafer_chuck"]["t_c"],
                "T_optics": nd["align_optics"]["t_c"],
                "T_beam_a": nd["bond_beam_a"]["t_c"],
                "T_arm": nd["bond_arm"]["t_c"],
                "dz_head_nm": dz_bh * 1e6,
                "up_col": up[0] if up else "", "up_row": up[1] if up else "",
                "up_score": up[2] if up else "",
                "dn_col": dn[0] if dn else "", "dn_row": dn[1] if dn else "",
                "dn_score": dn[2] if dn else "",
            })
            if k % 6 == 0:
                print("k=%2d sim_t=%7.1fs T头=%5.1f T盘=%5.1f T镜=%6.3f "
                      "up=%s down=%s"
                      % (k, sim_t or -1, nd["bond_head"]["t_c"],
                         nd["wafer_chuck"]["t_c"], nd["align_optics"]["t_c"],
                         ("%.1f,%.1f" % (up[0], up[1])) if up else "LOST",
                         ("%.1f,%.1f" % (dn[0], dn[1])) if dn else "LOST"))
            dt = PERIOD_S - (time.time() - t_loop)
            if dt > 0:
                time.sleep(dt)
    finally:
        # ---- 还原 ----
        http("/api/speed", {"scale": 1.0})
        http("/api/io/command",
             {"path": "SystemMachine.PulseHeat_Up", "cmd": "SetTargetTempWithPlan",
              "args": {"Temperature": 25.0, "Steady_Time": 0}})
        http("/api/io/command",
             {"path": "SystemMachine.Heat_Dw", "cmd": "SetTargetTempWithPlan",
              "args": {"Temperature": 30.0, "Steady_Time": 0}})
        http("/api/thermal", {"enabled": False})
        put_thermal({"env": {"wave": wave0}})
        http("/api/vision/mode", {"mode": "real"})
        for ax in ("BS_Arm_Z", "AS_Z", "AS_Y", "AS_X", "WS_LS_Y", "WS_LS_X"):
            try:
                move(ax, pos0.get(ax, 0.0), speed=100.0)
            except Exception as ex:
                print("还原 %s 失败: %s" % (ax, ex))
        if loaded_here:
            http("/api/workpiece/wafer", {"action": "unload"})
        print("已还原（速度/温度/热开关/视觉模式/轴位%s）"
              % ("/晶圆" if loaded_here else ""))

    # ---- CSV ----
    csv_path = "Temp/_thermal_drift_watch.csv"
    keys = list(rows[0].keys())
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r[k]) for k in keys) + "\n")
    print("CSV:", csv_path, "rows=%d" % len(rows))

    # ---- 趋势图 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ts = [r["sim_t"] for r in rows if r["sim_t"] is not None]
    t0 = ts[0] if ts else 0.0
    X = [(r["sim_t"] - t0) / 60.0 for r in rows]
    up_dx = [(r["up_col"] - up0[0]) * K_NM_PX / 1000.0
             if r["up_col"] != "" else float("nan") for r in rows]
    up_dy = [(r["up_row"] - up0[1]) * K_NM_PX / 1000.0
             if r["up_row"] != "" else float("nan") for r in rows]
    dn_dx = [(r["dn_col"] - dn0[0]) * K_NM_PX / 1000.0
             if r["dn_col"] != "" else float("nan") for r in rows]
    dn_dy = [(r["dn_row"] - dn0[1]) * K_NM_PX / 1000.0
             if r["dn_row"] != "" else float("nan") for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for ax, dx, dy, name in ((axes[0], up_dx, up_dy,
                              "up cam - head chuck mark"),
                             (axes[1], dn_dx, dn_dy,
                              "down cam - wafer site mark")):
        ax.plot(X, dx, ".-", ms=3, lw=0.8, label="dX (um)")
        ax.plot(X, dy, ".-", ms=3, lw=0.8, label="dY (um)")
        ax.set_ylabel("mark drift (um)")
        ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=8)
        ax.set_title(name, fontsize=10)
        ax2 = ax.twinx()
        ax2.plot(X, [r["T_head"] for r in rows], "r-", alpha=0.35, lw=1,
                 label="T_head")
        ax2.plot(X, [r["T_chuck"] for r in rows], "m-", alpha=0.35, lw=1,
                 label="T_chuck")
        ax2.plot(X, [r["T_optics"] for r in rows], "c-", alpha=0.5, lw=1,
                 label="T_optics")
        ax2.set_ylabel("T (C)"); ax2.legend(loc="upper right", fontsize=7)
    axes[1].set_xlabel("sim time (min)")
    fig.suptitle("thermal drift watch: head 300C + chuck 110C "
                 "(FAST, up-cam refocused)", fontsize=11)
    fig.tight_layout()
    png_path = "Temp/_thermal_drift_watch.png"
    fig.savefig(png_path, dpi=130)
    print("PNG:", png_path)

    # ---- 摘要 ----
    def _mm(v):
        vv = [x for x in v if not math.isnan(x)]
        return (min(vv), max(vv)) if vv else (float("nan"),) * 2
    print("== 摘要（相对基线，全窗） ==")
    print("up  dX [%.2f, %.2f] µm  dY [%.2f, %.2f] µm"
          % (_mm(up_dx) + _mm(up_dy)))
    print("down dX [%.2f, %.2f] µm  dY [%.2f, %.2f] µm"
          % (_mm(dn_dx) + _mm(dn_dy)))


if __name__ == "__main__":
    main()
