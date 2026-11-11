# -*- coding: utf-8 -*-
"""_warp_analysis_73.py — 晶圆翘曲开关影响分析采集（迭代 73 验证延伸，
2026-10-15）。连 9310 独立后端实例（不影响 9010 面板演示），翘曲 OFF/ON
对照实验，产出落盘 模型测试/：

  相位 A 共焦直径扫描：WS 载物晶圆相对下共焦探头步进 ±100mm（步 20mm，
  22 点），OFF/ON 两趟——dz 测量抛物线拟合 bow 回收率（独立测量口径，
  非同源断言）；OFF 基线平直度=零影响证据。
  相位 B EGA 对准对照：FAST/共焦噪声置 0（确定性）后 wafer_ega ×OFF/ON
  各两跑——ega_result 逐字段对比（XY 对准免疫的设计验证：应逐位一致）。
  相位 C 调焦调平对照（Z 向对准，共焦消费链）：flip_pick→flip_handoff→
  wafer_ega→focus_level(global)，OFF/ON 各一跑——fl_result/fl_verify
  对比（曲率不可被拟平面吸收→残余增大，补偿坡度=局部梯度）。
  相位 D 视觉离焦剂量响应：换片 bow∈{0,30,80}µm 逐次 identify 下相机
  晶圆标记——blur_ratio ≈ |dz|/DOF 线性验证。
  相位 E 接触位读数：state.bond_dz_um OFF/ON（键合点注入量观测）。

噪声全部置 0（sim_sensors confocal_sigma_um/fast_noise_px）→ OFF/ON 差
异=纯翘曲效应，EGA 应逐位一致。收尾还原处方/翘曲参数/工件。
"""
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:9310"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTDIR = os.path.join(ROOT, "模型测试")
TS = time.strftime("%Y%m%d_%H%M%S")

WARP = {"bow_um": 80.0, "saddle_um": 0.0, "ang_deg": 0.0}   # 分析用穹顶片
WARP_PARAMS = {"bow_um": 80.0, "saddle_um": 0.0,
               "saddle_ang_deg": 0.0, "jitter_pct": 0.0}


def http(method, path, body=None, timeout=30.0):
    req = urllib.request.Request(BASE + path, method=method)
    data = None
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, data, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        # 4xx/5xx 不抛：返回 (code, body)，由调用方按语境判（flow/stop
        # 空闲时 409 属正常）
        try:
            return e.code, json.loads(e.read().decode("utf-8"))
        except Exception:
            return e.code, {"ok": False, "error": str(e)}


def get(path):
    return http("GET", path)[1]


def post(path, body=None):
    c, r = http("POST", path, body or {})
    return c, r


def put(path, body):
    return http("PUT", path, body)


def warp(en):
    c, r = post("/api/warpage", {"enabled": bool(en)})
    assert c == 200 and r.get("enabled") == bool(en), (c, r)


def move(axis, target, speed=300.0):
    c, r = post("/api/axis/move", {"axis": axis, "target": target,
                                   "speed": speed})
    assert c == 200, (axis, target, r)
    t0 = time.time()
    while time.time() - t0 < 30.0:
        st = get("/api/axes")
        a = [x for x in st["axes"] if x["name"] == axis][0]
        if a.get("in_position") and not a.get("moving"):
            return
        time.sleep(0.05)
    raise RuntimeError("move 超时 %s→%s" % (axis, target))


def run_flow(path, body, name, timeout=600.0):
    """启动流程并等到 done/aborted；返回 flow state（aborted 也返回——
    验证类流程判废即结果）。"""
    post("/api/flow/stop", {})
    time.sleep(0.2)
    post("/api/flow/reset", {})
    c, r = post(path, body)
    assert c == 200, (name, c, r)
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = get("/api/flow/state")
        if st.get("state") in ("done", "aborted"):
            return st
        time.sleep(0.3)
    raise RuntimeError("流程超时 %s" % name)


def cf_z():
    return get("/api/confocal/read?probe=down").get("z_surface")


def wstate():
    return get("/api/warpage/state")


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    rows = {"scan": [], "ega": [], "fl": [], "blur": [], "bond": []}

    # ---------- 0. 确定性起点 ----------
    print("[setup] recipe 默认 + 噪声置 0 + modal/thermal OFF + 翘曲参数")
    post("/api/recipe/restore_default", {})
    post("/api/modal", {"enabled": False})
    post("/api/thermal", {"enabled": False})
    warp(False)
    put("/api/warpage", dict(WARP_PARAMS))
    rcp = get("/api/recipe")["recipe"]
    ss = rcp.get("sim_sensors") or {}
    fnp = ss.get("fast_noise_px") or {}
    zero_cams = {k: 0.0 for k in fnp} if isinstance(fnp, dict) else {}
    put("/api/recipe", {"sim_sensors": {
        "confocal_sigma_um": 0.0, "fast_noise_px": zero_cams},
        "wafer_load": {"jitter_on": False, "load_x_mm": 0.0,
                       "load_y_mm": 0.0, "load_rz_rad": 0.0}})
    post("/api/workpiece/reset", {})

    # ---------- A. 共焦直径扫描（OFF/ON 两趟） ----------
    print("[A] 共焦直径扫描 ±100mm（22 点 ×2 趟）")
    post("/api/workpiece/wafer", {"action": "load", "warp": dict(WARP)})
    xs = [float(v) for v in range(-100, 101, 20)]
    for mode in (False, True):
        warp(mode)
        for x in xs:
            move("WS_LS_X", x)
            zs = cf_z()
            pd = (wstate().get("probe_down") or {})
            rows["scan"].append({
                "mode": "ON" if mode else "OFF",
                "ws_x_mm": x, "probe_x_mm": pd.get("x_mm"),
                "probe_y_mm": pd.get("y_mm"),
                "z_surface_mm": zs,
                "state_probe_dz_um": pd.get("dz_um")})
        move("WS_LS_X", 0.0)

    # 抛物线拟合（独立回收 bow）：dz(ws_x) = bow·(1−((u0−x)²+v0²)/R²)
    import numpy as np
    off = {r["ws_x_mm"]: r["z_surface_mm"] for r in rows["scan"]
           if r["mode"] == "OFF"}
    meas = [(r["ws_x_mm"], (r["z_surface_mm"] - off[r["ws_x_mm"]]) * 1000.0)
            for r in rows["scan"] if r["mode"] == "ON"]
    px = np.array([m[0] for m in meas])
    py = np.array([m[1] for m in meas])
    a2, a1, a0 = np.polyfit(px, py, 2)
    R = float(rcp["wafer"]["diameter_mm"]) / 2.0
    bow_fit = -a2 * R * R
    vertex_x = -a1 / (2.0 * a2)
    flat_off = max(off.values()) - min(off.values())
    resid = py - (a2 * px * px + a1 * px + a0)
    scan_fit = {"bow_fit_um": float(bow_fit), "bow_inject_um": WARP["bow_um"],
                "bow_recover_pct": float(bow_fit / WARP["bow_um"] * 100.0),
                "vertex_ws_x_mm": float(vertex_x),
                "fit_resid_max_um": float(abs(resid).max()),
                "off_flatness_um": float(flat_off * 1000.0)}

    # ---------- B. EGA 对准对照（确定性 → 逐位一致预期） ----------
    print("[B] EGA 对准对照 ×2+2")
    for mode in (False, True):
        warp(mode)
        for rep in (1, 2):
            post("/api/workpiece/reset", {})
            st = run_flow("/api/process/wafer_ega", {"with_load": True},
                          "ega", timeout=300.0)
            ega = st.get("ega_result") or {}
            rows["ega"].append({"mode": "ON" if mode else "OFF", "rep": rep,
                                "state": st.get("state"), **ega})
            print("   ega %s#%d state=%s n=%s" %
                  ("ON" if mode else "OFF", rep, st.get("state"),
                   ega.get("n")))
    warp(False)

    # ---------- C. 调焦调平对照（flip_pick→handoff→ega→fl global） ----------
    print("[C] 调焦调平对照（全链 ×2，8× 提速）")
    post("/api/speed", {"scale": 8.0})
    for mode in (False, True):
        warp(mode)
        post("/api/workpiece/reset", {})
        run_flow("/api/process/flip_pick", {}, "flip_pick", timeout=300.0)
        run_flow("/api/process/flip_handoff", {}, "flip_handoff",
                 timeout=300.0)
        run_flow("/api/process/wafer_ega", {"with_load": True}, "ega",
                 timeout=300.0)
        st = run_flow("/api/process/focus_level", {"mode": "global"},
                      "focus_level", timeout=300.0)
        rows["fl"].append({"mode": "ON" if mode else "OFF",
                           "state": st.get("state"),
                           "fl_result": st.get("fl_result"),
                           "fl_verify": st.get("fl_verify")})
        print("   fl %s state=%s verify=%s"
              % ("ON" if mode else "OFF", st.get("state"),
                 json.dumps(st.get("fl_verify"), ensure_ascii=False)[:160]))
    post("/api/speed", {"scale": 1.0})
    warp(False)

    # ---------- D. 视觉离焦剂量响应（换片 bow 0/30/80） ----------
    print("[D] 视觉离焦剂量响应（下相机晶圆标记 blur）")
    for bow in (0.0, 30.0, 80.0):
        post("/api/workpiece/reset", {})
        post("/api/workpiece/wafer",
             {"action": "load",
              "warp": {"bow_um": bow, "saddle_um": 0.0, "ang_deg": 0.0}})
        rec = {"bow_um": bow}
        for mode in (False, True):
            warp(mode)
            r = post("/api/vision/identify",
                     {"cam": "down", "template": "cv_quad"})[1]
            ent = r.get("entry") or {}
            rec["blur_ON" if mode else "blur_OFF"] = ent.get("blur_ratio")
            rec["found_ON" if mode else "found_OFF"] = bool(
                r.get("ok") and ent)
        pd = (wstate().get("probe_down") or {})
        rec["probe_dz_um_ON"] = pd.get("dz_um")
        rows["blur"].append(rec)
        print("   bow=%g blur OFF=%s ON=%s" %
              (bow, rec.get("blur_OFF"), rec.get("blur_ON")))
    warp(False)

    # ---------- E. 键合点注入量观测 ----------
    for mode in (False, True):
        warp(mode)
        rows["bond"].append({"mode": "ON" if mode else "OFF",
                             "bond_dz_um": wstate().get("bond_dz_um")})
    warp(False)

    # ---------- 落盘 ----------
    print("[save] 落盘 模型测试/")
    scan_csv = os.path.join(OUTDIR, "warpage_confocal_scan_%s.csv" % TS)
    with open(scan_csv, "w", encoding="utf-8-sig", newline="") as f:
        hdr = ["mode", "ws_x_mm", "probe_x_mm", "probe_y_mm",
               "z_surface_mm", "state_probe_dz_um"]
        f.write(",".join(hdr) + "\n")
        for r in rows["scan"]:
            f.write(",".join(str(r.get(h)) for h in hdr) + "\n")

    ega_keys = sorted({k for r in rows["ega"]
                       for k in r if k not in ("mode", "rep", "state")})
    ega_csv = os.path.join(OUTDIR, "warpage_ega_%s.csv" % TS)
    with open(ega_csv, "w", encoding="utf-8-sig", newline="") as f:
        f.write(",".join(["mode", "rep", "state"] + ega_keys) + "\n")
        for r in rows["ega"]:
            f.write(",".join(str(r.get(k))
                             for k in ["mode", "rep", "state"] + ega_keys)
                    + "\n")

    fl_csv = os.path.join(OUTDIR, "warpage_fl_%s.csv" % TS)
    with open(fl_csv, "w", encoding="utf-8-sig", newline="") as f:
        f.write("mode,state,fl_result,fl_verify\n")
        for r in rows["fl"]:
            f.write("%s,%s,%s,%s\n" % (
                r["mode"], r["state"],
                json.dumps(r["fl_result"], ensure_ascii=False),
                json.dumps(r["fl_verify"], ensure_ascii=False)))

    blur_csv = os.path.join(OUTDIR, "warpage_blur_%s.csv" % TS)
    with open(blur_csv, "w", encoding="utf-8-sig", newline="") as f:
        f.write("bow_um,blur_OFF,blur_ON,found_OFF,found_ON,"
                "probe_dz_um_ON\n")
        for r in rows["blur"]:
            f.write("%s,%s,%s,%s,%s,%s\n" % (
                r["bow_um"], r.get("blur_OFF"), r.get("blur_ON"),
                r.get("found_OFF"), r.get("found_ON"),
                r.get("probe_dz_um_ON")))

    bond_csv = os.path.join(OUTDIR, "warpage_bond_dz_%s.csv" % TS)
    with open(bond_csv, "w", encoding="utf-8-sig", newline="") as f:
        f.write("mode,bond_dz_um\n")
        for r in rows["bond"]:
            f.write("%s,%s\n" % (r["mode"], r["bond_dz_um"]))

    # PNG：抛物线拟合对照
    png = os.path.join(OUTDIR, "warpage_scan_parity_%s.png" % TS)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False
        fig, ax = plt.subplots(figsize=(8, 4.5), dpi=130)
        ax.plot(px, py, "o", color="#3a7bd5", label="measured dz (ON-OFF)")
        xf = np.linspace(-100, 100, 200)
        ax.plot(xf, a2 * xf * xf + a1 * xf + a0, "-", color="#d5803a",
                label="parabola fit: bow=%.2fµm (inject 80µm)" % bow_fit)
        ax.set_xlabel("WS_LS_X (mm)")
        ax.set_ylabel("dz (µm)")
        ax.set_title("Warpage confocal scan parity (R=%.0fmm, "
                     "fit resid max %.3fµm)" % (R, abs(resid).max()))
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(png)
    except Exception as e:
        png = None
        print("   [warn] PNG 生成失败：%s" % e)

    summary = {"ts": TS, "scan_fit": scan_fit,
               "ega_rows": rows["ega"], "fl_rows": [
                   {"mode": r["mode"], "state": r["state"],
                    "fl_result": r["fl_result"], "fl_verify": r["fl_verify"]}
                   for r in rows["fl"]],
               "blur_rows": rows["blur"], "bond_rows": rows["bond"],
               "files": {"scan_csv": scan_csv, "ega_csv": ega_csv,
                         "fl_csv": fl_csv, "blur_csv": blur_csv,
                         "bond_csv": bond_csv, "png": png}}
    sj = os.path.join(OUTDIR, "warpage_analysis_%s.json" % TS)
    with open(sj, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

    # ---------- 收尾还原 ----------
    post("/api/recipe/restore_default", {})
    put("/api/warpage", {"bow_um": 30.0, "saddle_um": 12.0,
                         "saddle_ang_deg": 0.0, "jitter_pct": 20.0})
    warp(False)
    post("/api/workpiece/reset", {})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z", "BS_Arm_Z"):
        try:
            move(ax, 0.0)
        except Exception:
            pass
    print("[done] %s" % sj)
    return summary


if __name__ == "__main__":
    s = main()
    print(json.dumps(s["scan_fit"], ensure_ascii=False))
