#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_regression.py — 定时/日常回归 runner（迭代 50，纯标准库）。

用法：python backend/tests/run_regression.py [任意 test_client 参数...]
      空参 = quick 套件；--full 全量；--segs a,b / -k "..." 等原样透传。

做的事：
  1. 预检 9010：被占（多半是残留 runtime）→ 中文报错退 1；
  2. 自动补 --out Temp/regression_logs/<suite>_yyyymmdd_HHMMSS.json，
     subprocess 跑 env/python/python.exe backend/tests/test_client.py，
     stdout 直接落同名 .log（文件句柄，不走管道），退出码原样继承；
  3. 跑完写/更新 Temp/regression_logs/latest.json：
     {ts, suite, passed, total, wall_s, exit_code, fails, log, json}
     （passed/total/fails 解析自 --out 的结果 JSON；套件中途崩溃导致
     结果 JSON 缺失时 passed/total/wall_s 记 null，exit_code 仍照实）；
  4. 日志轮转：regression_logs/ 只留最近 30 份（按文件名时间戳排序，
     log/json 成对删；latest.json 不参与轮转）。
"""
import json
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BACKEND = os.path.dirname(HERE)
ROOT = os.path.dirname(BACKEND)
ENV_PY = os.path.join(ROOT, "env", "python", "python.exe")
LOG_DIR = os.path.join(ROOT, "Temp", "regression_logs")
KEEP = 30                                # 日志保留份数（log/json 成对计 1 份）
WS_PORT = 9010


def port_occupied(port):
    """TCP 能连上即视为被占（不限于本仿真进程，宁严勿宽）。"""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


def rotate_logs():
    """只留最近 KEEP 份（文件名 <suite>_yyyymmdd_HHMMSS 字典序=时序）。"""
    try:
        names = os.listdir(LOG_DIR)
    except OSError:
        return
    stems = set()
    for fn in names:
        if re.match(r"^(quick|full)_\d{8}_\d{6}\.(log|json)$", fn):
            stems.add(fn.rsplit(".", 1)[0])
    stems = sorted(stems)
    for st in stems[:-KEEP] if len(stems) > KEEP else []:
        for ext in (".log", ".json"):
            try:
                os.remove(os.path.join(LOG_DIR, st + ext))
            except OSError:
                pass


def main():
    args = sys.argv[1:]
    suite = "full" if "--full" in args else "quick"
    if port_occupied(WS_PORT):
        print("[错误] 端口 %d 已被占用（多半是残留 runtime 进程）。请先 "
              "netstat -ano | findstr :%d 查到 PID、验明是本工作区 python "
              "后 taskkill /PID <pid> /F，再重跑。" % (WS_PORT, WS_PORT))
        return 1
    # --out 由本 runner 统一管理：透传里的 --out 剥掉，防 latest.json 失配
    passthrough = []
    skip_next = False
    for a in args:
        if skip_next:
            skip_next = False
            continue
        if a == "--out":
            skip_next = True
            print("[run_regression] 忽略透传的 --out（产物路径由 runner 统一管理）")
            continue
        passthrough.append(a)

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_json = os.path.join(LOG_DIR, "%s_%s.json" % (suite, stamp))
    log_path = os.path.join(LOG_DIR, "%s_%s.log" % (suite, stamp))
    py = ENV_PY if os.path.exists(ENV_PY) else sys.executable
    env = dict(os.environ)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    cmd = [py, os.path.join(BACKEND, "tests", "test_client.py")] \
        + passthrough + ["--out", out_json]
    print("[run_regression] suite=%s log=%s" % (suite, log_path))
    with open(log_path, "wb") as lf:      # stdout 直落文件句柄，不走管道
        rc = subprocess.run(cmd, cwd=BACKEND, env=env,
                            stdout=lf, stderr=subprocess.STDOUT).returncode

    passed = total = None
    wall_s = None
    fails = []
    if os.path.exists(out_json):
        try:
            with open(out_json, encoding="utf-8") as f:
                rep = json.load(f)
            results = rep.get("results") or []
            total = len(results)
            fails = [{"name": n, "detail": d}
                     for n, ok, d in results if not ok]
            passed = total - len(fails)
            wall_s = rep.get("wall_s")
        except (ValueError, TypeError, KeyError) as e:
            print("[run_regression] 结果 JSON 解析失败（latest 记空）：%s" % e)
    latest = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"),
              "suite": suite, "passed": passed, "total": total,
              "wall_s": wall_s, "exit_code": rc, "fails": fails,
              "log": log_path, "json": out_json}
    with open(os.path.join(LOG_DIR, "latest.json"), "w",
              encoding="utf-8") as f:
        json.dump(latest, f, ensure_ascii=False, indent=1)
    rotate_logs()

    print("[run_regression] 完成：suite=%s exit=%d 通过=%s/%s wall=%s"
          % (suite, rc, passed, total,
             ("%.1fs" % wall_s) if wall_s is not None else "?"))
    for fl in fails:
        print("  FAIL: %s %s" % (fl["name"], fl["detail"]))
    print("[run_regression] log=%s json=%s" % (log_path, out_json))
    return rc                              # 退出码原样继承 test_client


if __name__ == "__main__":
    sys.exit(main())
