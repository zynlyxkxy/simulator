# -*- coding: utf-8 -*-
"""接触位热漂移活体复现/取证：力-位移扫描反推 eff_contact（冷 vs 吸盘热）。"""
import json
import time
import urllib.request

BASE = "http://127.0.0.1:9010"


def http(path, payload=None, method=None, timeout=30):
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(BASE + path, data=data,
                                 method=method or ("POST" if data else "GET"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def force_avg(n=8, dt=0.05):
    s = 0.0
    for _ in range(n):
        s += float((http("/api/io").get("points") or {}).get(
            "SystemMachine.SS_Bonding.M_BS.oActualForce", 0.0) or 0.0)
        time.sleep(dt)
    return s / n


def move(ax, tgt, speed=20.0):
    http("/api/axis/move", {"axis": ax, "target": tgt, "speed": speed})
    t0 = time.time()
    while time.time() - t0 < 20:
        a = next(x for x in http("/api/axes")["axes"] if x["name"] == ax)
        if abs(a["pos"] - tgt) < 0.003 and abs(a.get("vel", 0.0)) < 0.01:
            return
        time.sleep(0.15)
    raise RuntimeError("settle 超时 %s" % ax)


def onset_scan(tag):
    """对准台退避后从 −167.38 逐步下移，读力 → 线性拟合 x 截距=eff_contact。
    （沙盒约束：对准台在锚点时头降键合行程会撞相机承载台 409）"""
    move("AS_X", 0.0, speed=100.0)
    move("AS_Y", 120.0, speed=100.0)         # as_safe（限位内全行程退避）
    pts = []
    for p in (-167.38, -167.40, -167.42, -167.44, -167.46):
        move("BS_Arm_Z", p)
        time.sleep(0.2)
        f = force_avg()
        pts.append((p, f))
        print("   [%s] pos=%.3f f=%+.3f" % (tag, p, f))
    move("BS_Arm_Z", 0.0, speed=100.0)       # 先抬头再回对准台
    move("AS_X", 0.0, speed=100.0)
    move("AS_Y", 6.0, speed=100.0)
    # 用 force>0.05 的点做线性拟合；不足则直接报告
    hot = [(p, f) for p, f in pts if f > 0.05]
    if len(hot) >= 2:
        (p1, f1), (p2, f2) = hot[0], hot[-1]
        k = (f2 - f1) / (p1 - p2)          # f 随 pos 减小而增大 → k>0
        x0 = p1 - (f1 - 0.0) / k if abs(k) > 1e-9 else None
        print("   [%s] 拟合 k=%.1fN/mm eff_contact≈%.4f" % (tag, k, x0))
        return x0
    print("   [%s] 力未有效建立（>0.05N 点数=%d）" % (tag, len(hot)))
    return None


http("/api/thermal", {"enabled": True}, method="POST")
http("/api/thermal", {"env": {"wave": "off"}}, method="PUT")
try:
    http("/api/workpiece/wafer", {"action": "load", "offset": [0.0, 0.0],
                                  "rz": 0.0})
except Exception:
    pass
move("AS_X", 0.0); move("AS_Y", 6.0); move("AS_Z", 0.0)
# 头等冷
http("/api/speed", {"scale": 8.0})
for _ in range(60):
    if http("/api/thermal/state")["nodes"]["bond_head"]["t_c"] < 26.5:
        break
    time.sleep(0.5)
http("/api/speed", {"scale": 1.0})

rd0 = http("/api/confocal/read?probe=down")
print("冷态 z_s=%.6f read=%.3f" % (rd0["z_surface"], rd0["reading_um"]))
x_cold = onset_scan("冷")

# 吸盘 →110℃
http("/api/io/command", {"path": "SystemMachine.Heat_Dw",
                         "cmd": "SetTargetTempWithPlan",
                         "args": {"Temperature": 110.0, "Steady_Time": 0}})
http("/api/speed", {"scale": 8.0})
for _ in range(120):
    if http("/api/thermal/state")["nodes"]["wafer_chuck"]["t_c"] >= 105.0:
        break
    time.sleep(0.5)
http("/api/speed", {"scale": 1.0})
rd1 = http("/api/confocal/read?probe=down")
th1 = http("/api/thermal/state")
print("热态 z_s=%.6f read=%.3f deform_z=%.1fµm T=%.1f"
      % (rd1["z_surface"], rd1["reading_um"],
         th1["deform"]["wafer_stage"]["t_mm"][2] * 1000.0,
         th1["nodes"]["wafer_chuck"]["t_c"]))
x_hot = onset_scan("热")

if x_cold is not None and x_hot is not None:
    print("==> eff_contact 冷 %.4f → 热 %.4f，Δ=%+.1fµm"
          % (x_cold, x_hot, (x_hot - x_cold) * 1000.0))
else:
    print("==> 无法反推接触位（力未建立）")
