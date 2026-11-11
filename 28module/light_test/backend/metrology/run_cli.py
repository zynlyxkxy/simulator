#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_cli.py — 相机对准重复性测校命令行入口（连已启动的后端 HTTP API）。

用法：
  python backend/metrology/run_cli.py --cam up --mode static --repeat 50 --wait 10
  python backend/metrology/run_cli.py --cam down --mode dynamic --repeat 50 --wait 10 --travel 5

流程：POST {base}/api/calibration/start {routine, cam, params} → 轮询
GET /api/calibration/status?job_id= 至 done/error → GET result → 打印统计
摘要（CSV 已由后端落盘 runtime/metrology_data/）。退出码 0=done，非 0=失败。

注意：--base 默认 9010（WS 网关 HTTP 端口，/api/calibration/* 所在）；
9101 是 ACS TCP 行 JSON 口，不是 HTTP，别填。
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

DEFAULT_BASE = "http://127.0.0.1:9010"


def _http_json(url, data=None, timeout=10.0):
    if data is not None:
        req = urllib.request.Request(
            url, data=json.dumps(data).encode("utf-8"),
            headers={"Content-Type": "application/json"})
    else:
        req = url
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8"))
        except Exception:
            return e.code, {"ok": False, "error": "HTTP %d" % e.code}


def _fmt_px(st, k_nm):
    return ("mean %.4fpx  std %.4fpx  3σ %.4fpx = %.1fnm"
            % (st["mean_px"], st["std_px"], st["s3_px"], st["s3_nm"]))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="相机对准重复性测校（静态/动态），连已启动的后端 HTTP API")
    ap.add_argument("--cam", required=True, choices=("up", "down", "coarse"),
                    help="相机：up=上精对准(键合头 die) / down=下精对准(晶圆) "
                         "/ coarse=粗对准(翻转模块 die)")
    ap.add_argument("--mode", required=True, choices=("static", "dynamic"),
                    help="static=静止连拍；dynamic=载体轴退 travel 再回拍照位"
                         "往复，每循环拍照")
    ap.add_argument("--repeat", type=int, default=50, help="拍照次数 [10,1000]")
    ap.add_argument("--wait", type=float, default=10.0,
                    help="每次拍照前等待 ms [0,60000]")
    ap.add_argument("--travel", type=float, default=5.0,
                    help="dynamic 回退行程 mm [0.1,20]（static 忽略）")
    ap.add_argument("--base", default=DEFAULT_BASE,
                    help="后端 HTTP API 根（默认 %s）" % DEFAULT_BASE)
    ap.add_argument("--timeout", type=float, default=900.0,
                    help="作业轮询上限 s（默认 900）")
    args = ap.parse_args(argv)

    routine = "align_repeat_" + args.mode
    params = {"repeat": args.repeat, "wait_ms": args.wait}
    if args.mode == "dynamic":
        params["travel_mm"] = args.travel
    base = args.base.rstrip("/")
    print("[cli] %s %s cam=%s repeat=%d wait=%sms%s -> %s"
          % (routine, "POST", args.cam, args.repeat, args.wait,
             (" travel=%smm" % args.travel) if args.mode == "dynamic" else "",
             base))
    c, r = _http_json(base + "/api/calibration/start",
                      {"routine": routine, "cam": args.cam, "params": params})
    if c != 200 or not r.get("ok"):
        print("[cli] 启动失败 HTTP %d: %s" % (c, r.get("error", r)))
        return 1
    jid = r["job_id"]
    print("[cli] job_id=%d 运行中..." % jid)
    t0 = time.time()
    while True:
        c, s = _http_json(base + "/api/calibration/status?job_id=%d" % jid)
        if c != 200:
            print("[cli] status 查询失败 HTTP %d: %s" % (c, s.get("error", s)))
            return 1
        st = s.get("state")
        pg = s.get("progress") or {}
        print("\r[cli] %s %s/%s %s   " % (st, pg.get("i", "?"),
                                          pg.get("n", "?"), pg.get("note", "")),
              end="", flush=True)
        if st in ("done", "error"):
            print()
            break
        if time.time() - t0 > args.timeout:
            print("\n[cli] 轮询超时（%gs）" % args.timeout)
            return 1
        time.sleep(0.2)
    if st == "error":
        print("[cli] 作业失败：%s" % s.get("error"))
        return 1
    c, r = _http_json(base + "/api/calibration/result?job_id=%d" % jid)
    if c != 200 or not r.get("ok"):
        print("[cli] result 查询失败 HTTP %d: %s" % (c, r.get("error", r)))
        return 1
    res = r["result"]
    k = res["nm_per_px"]
    stats = res["stats"]
    print("=" * 60)
    print("routine=%s camera=%s repeat=%d wait_ms=%s nm_per_px=%.2f"
          % (res["routine"], res["camera"], res["repeat"], res["wait_ms"], k))
    print("row : %s" % _fmt_px(stats["row"], k))
    print("col : %s" % _fmt_px(stats["col"], k))
    if stats.get("angle"):
        a = stats["angle"]
        print("ang : mean %.6fdeg  std %.6fdeg  3σ %.1fµrad"
              % (a["mean_deg"], a["std_deg"], a["s3_urad"]))
    if stats.get("axes"):
        for ax, st_ in stats["axes"].items():
            print("axis %-10s: mean %.6fmm  std %.7fmm  3σ %.2fµm"
                  % (ax, st_["mean_mm"], st_["std_mm"], st_["s3_um"]))
    print("csv : %s" % res["csv"])
    print("elapsed: %.1fs" % res["elapsed_s"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
