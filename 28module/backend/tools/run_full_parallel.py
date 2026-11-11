#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_full_parallel.py — quick 套件并行分片回归编排器（迭代 50 复活，纯标准库）。

迭代 19 在 test_client.py 埋好了 --bare/--segs/--skip-segs/--out/--shard-id
分片参数与 tests/bare_harness.py 裸引擎 harness（机制见其头部 docstring），
但编排器本体一直缺失（test_client.py:137、:11222、:11494 三处注释引用本
文件）——本文件补齐这条断链。

用法：
  python backend/tools/run_full_parallel.py                 # quick 拆 3 片（默认）
  python backend/tools/run_full_parallel.py --shards 4      # 4 片
  python backend/tools/run_full_parallel.py --segs 3.plc,15.recipe
      # 显式段集：仍按 bare/全栈分类分片（全栈段自动单列一片，不 bare）
  python backend/tools/run_full_parallel.py --concurrent
      # bare 相与全栈片也并发（默认分相执行：先 bare 各片并发、再全栈片，
      # 防裸引擎全速 tick 线程抢 CPU 污染全栈片 1000Hz/500Hz 墙钟红线）
  python backend/tools/run_full_parallel.py --full
      # 全量口径（full-only 段未做 bare 验证，一律归全栈片；quick 是实测路径）

分片机制：
- AST 静态解析 test_client.py 的 segs 段表（运行时真值，免手抄漂移）；
- 段分三类：WS 段（段表 args 非空，bare 无 WS 客户端）与 FULLSTACK_SEGS
  （端口/REAL/墙钟/9200 帧通道段，迭代 50 逐段实测裁定，清单见下）归全栈片；
  其余归 bare 片（与全栈逐段一致性实测见 docs/回归基建_V1.md §5）；
- bare 段按段表顺序切连续区间，19.x 同族段（fl/fa→wafer_ega 等有顺序依赖，
  test_client.py:11435-11437 注释）强制同片，各片段数尽量均衡；
- 每片 subprocess 跑 test_client.py（bare 片加 --bare --shard-id i，MQTT
  派生 11883+i、tempdir per-shard 隔离），--out Temp/_par_shard<i>.json，
  stdout 直落 Temp/_par_shard<i>.log（文件句柄，不走管道）；
- 聚合打印总表（各片通过数/墙钟、FAIL 明细、总墙钟 vs 串行参考），
  任一 FAIL 或片进程非零退出 → 本编排器退 1。
"""
import ast
import itertools
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BACKEND = os.path.dirname(HERE)
ROOT = os.path.dirname(BACKEND)
TEST_CLIENT = os.path.join(BACKEND, "tests", "test_client.py")
ENV_PY = os.path.join(ROOT, "env", "python", "python.exe")
TMP = os.path.join(ROOT, "Temp")

# 全栈段裁定清单（迭代 50 实测；WS 段由段表 args 自动判定，不在此列）：
#   3.5.plc_scan    PLC 扫描周期墙钟 ms 语义
#   4.buffer        ACS TCP 9101（ACSClient 真 socket）
#   11.fault        端口层故障注入（HTTP 挂起/MCS 断线拒连/health 端口字段）
#   11.5.acsn       acs_probe 子进程 + 9101
#   17.vision       MQTT 1883 broker 自测 + REAL(cv2) + perf 红线
#   18.mc_calib     REAL 测校链（cv2）
#   19.13.iter8b    MQTT 1883 墙钟（段内硬编码 1883，per-shard 隔离吃不到）
#   19.22.iter17    REAL 冒烟 + VisionClient 1883
#   19.26.iter22    read_clean_health 5s 聚合窗（裸引擎 tick 无墙钟节拍语义）
#   19.27.iter23a   REAL 连拍（cv2）
#   19.28.iter23b   health 窗 perf（同上）
#   19.24.iter20    段内直 urllib 打 9010 /api/vision/capture/file 取 PNG
#                   二进制（绕过 http_* shim，裸引擎无 HTTP 服务；实测 bare
#                   下 ConnectionRefused/误答并发全栈后端）
#   19.29.nonortho  测校栅格走视觉 TCP 9200 帧通道（VisionService 不 start
#                   无此通道；实测 bare 报 "9200 帧处理超时"）
#   19.30.pi_real   REAL 键合后检测端到端（迭代 53；REAL 需真视觉服务）
#   19.31.real_chains REAL 工艺链端到端（迭代 54；同上）
#   19.32~35        翘曲/正压与通大气/误差全关/IO 搜力均含真实 HTTP 场景
FULLSTACK_SEGS = frozenset({
    "3.5.plc_scan", "4.buffer", "11.fault", "11.5.acsn", "17.vision",
    "18.mc_calib", "19.13.iter8b", "19.22.iter17", "19.24.iter20",
    "19.26.iter22", "19.27.iter23a", "19.28.iter23b", "19.29.nonortho",
    "19.30.pi_real", "19.31.real_chains", "19.32.iter73", "19.33.iter76",
    "19.34.iter77", "19.35.iter78",
})

# 串行 quick 参考墙钟（迭代 50 实测，见 docs/回归基建_V1.md §5；None=不打印对比）
SERIAL_QUICK_REF_S = 646.7


def parse_segs():
    """AST 解析 test_client.py main() 里的 segs 段表 -> [(name, has_args, suite)]。"""
    with open(TEST_CLIENT, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == "segs"
                   for t in node.targets):
            continue
        out = []
        for elt in node.value.elts:
            name = ast.literal_eval(elt.elts[0])
            args = elt.elts[2]
            has_args = isinstance(args, ast.Tuple) and len(args.elts) > 0
            suite = ast.literal_eval(elt.elts[4])
            out.append((name, has_args, suite))
        return out
    raise RuntimeError("test_client.py 里找不到 segs 段表（结构变了？）")


def port_occupied(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


def split_bare(bare_names, k):
    """bare 段列表（段表顺序）切 k 个连续区间；19.x 同族强制同片。
    返回 [ [段名...], ... ]（不足 k 个可切点时返回更少片）。"""
    fam = [n.startswith("19.") for n in bare_names]
    # 允许在 i 前切 <=> 左右不都在 19.x 族内
    can_cut = [not (fam[i - 1] and fam[i]) for i in range(1, len(bare_names))]
    cut_positions = [i for i in range(1, len(bare_names)) if can_cut[i - 1]]
    k = max(1, min(k, len(cut_positions) + 1))
    if k <= 1 or len(bare_names) <= 1:
        return [list(bare_names)]
    best = None
    for cuts in itertools.combinations(cut_positions, k - 1):
        bounds = [0] + list(cuts) + [len(bare_names)]
        sizes = [bounds[i + 1] - bounds[i] for i in range(k)]
        score = max(sizes) - min(sizes)
        if best is None or score < best[0]:
            best = (score, cuts)
    bounds = [0] + list(best[1]) + [len(bare_names)]
    return [bare_names[bounds[i]:bounds[i + 1]] for i in range(k)]


def main():
    args = sys.argv[1:]
    full = "--full" in args
    concurrent = "--concurrent" in args

    def _arg(name, default=None):
        return args[args.index(name) + 1] if name in args else default
    n_shards = int(_arg("--shards", "3"))
    segs_only = set((_arg("--segs") or "").split(",")) - {""} \
        if "--segs" in args else None

    table = parse_segs()
    bare_names, fs_names = [], []
    for name, has_args, suite in table:
        if segs_only is not None and name not in segs_only:
            continue
        if suite == "full" and not full:
            continue                     # quick 口径：full-only 段不跑
        if suite == "full":              # --full：full-only 段未做 bare 验证
            fs_names.append(name)
        elif has_args or name in FULLSTACK_SEGS:
            fs_names.append(name)
        else:
            bare_names.append(name)
    if segs_only is not None:
        unknown = segs_only - {n for n, _, _ in table}
        if unknown:
            print("[错误] --segs 含段表没有的段名：%s" % sorted(unknown))
            return 1
    if not bare_names and not fs_names:
        print("[错误] 没有可跑的段")
        return 1

    k_bare = max(1, n_shards - (1 if fs_names else 0)) if bare_names else 0
    bare_shards = split_bare(bare_names, k_bare) if bare_names else []
    shards = [{"kind": "bare", "segs": s, "id": i}
              for i, s in enumerate(bare_shards)]
    if fs_names:
        shards.append({"kind": "fullstack", "segs": fs_names,
                       "id": len(bare_shards)})

    # 端口预检：全栈片要拉起真后端，六端口必须空闲；bare 片查 11883+i
    if fs_names:
        busy = [p for p in (9010, 9101, 9102, 9190, 1883, 9200)
                if port_occupied(p)]
        if busy:
            print("[错误] 全栈片所需端口被占用：%s。请 netstat -ano 查 PID、"
                  "验明是本工作区 python 后 taskkill /PID <pid> /F。" % busy)
            return 1
    for sh in shards:
        if sh["kind"] == "bare" and port_occupied(11883 + sh["id"]):
            print("[错误] bare 片 MQTT 派生端口 %d 被占用"
                  % (11883 + sh["id"]))
            return 1

    py = ENV_PY if os.path.exists(ENV_PY) else sys.executable
    env = dict(os.environ)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    print("[par] 分片方案：%s" % "; ".join(
        "片%d(%s, %d 段)" % (sh["id"], sh["kind"], len(sh["segs"]))
        for sh in shards))
    for sh in shards:
        print("[par]   片%d(%s): %s" % (sh["id"], sh["kind"],
                                       ",".join(sh["segs"])))

    def launch(sh):
        cmd = [py, TEST_CLIENT]
        if full:
            cmd.append("--full")
        if sh["kind"] == "bare":
            cmd += ["--bare", "--shard-id", str(sh["id"])]
        cmd += ["--segs", ",".join(sh["segs"]),
                "--out", os.path.join(TMP, "_par_shard%d.json" % sh["id"])]
        lf = open(os.path.join(TMP, "_par_shard%d.log" % sh["id"]), "wb")
        t0 = time.time()
        p = subprocess.Popen(cmd, cwd=BACKEND, env=env,
                             stdout=lf, stderr=subprocess.STDOUT)
        sh["_lf"], sh["_t0"], sh["_p"] = lf, t0, p
        print("[par] 片%d(%s) 启动 pid=%d" % (sh["id"], sh["kind"], p.pid),
              flush=True)

    def wait_all(group):
        for sh in group:
            sh["_rc"] = sh["_p"].wait()
            sh["_lf"].close()
            print("[par] 片%d(%s) 退出 rc=%d (%.1fs)"
                  % (sh["id"], sh["kind"], sh["_rc"],
                     time.time() - sh["_t0"]), flush=True)

    t_all = time.time()
    if concurrent:
        for sh in shards:
            launch(sh)
        wait_all(shards)
    else:
        bare_group = [sh for sh in shards if sh["kind"] == "bare"]
        fs_group = [sh for sh in shards if sh["kind"] == "fullstack"]
        for sh in bare_group:
            launch(sh)
        wait_all(bare_group)
        for sh in fs_group:
            launch(sh)
        wait_all(fs_group)
    wall_all = time.time() - t_all

    # ---- 聚合 ----
    total_pass = total_n = 0
    fails = []
    broken = []
    print("\n==== 并行回归聚合 ====")
    for sh in shards:
        out = os.path.join(TMP, "_par_shard%d.json" % sh["id"])
        if not os.path.exists(out):
            broken.append(sh)
            print("片%d(%s, %d 段)：结果 JSON 缺失（rc=%d，片中途崩溃，"
                  "见 Temp/_par_shard%d.log）"
                  % (sh["id"], sh["kind"], len(sh["segs"]), sh["_rc"],
                     sh["id"]))
            continue
        with open(out, encoding="utf-8") as f:
            rep = json.load(f)
        results = rep.get("results") or []
        n_fail = len([1 for _n, ok, _d in results if not ok])
        total_n += len(results)
        total_pass += len(results) - n_fail
        fails += [(sh["id"], n, d) for n, ok, d in results if not ok]
        print("片%d(%s, %2d 段)：%d/%d 通过，墙钟 %.1fs"
              % (sh["id"], sh["kind"], len(sh["segs"]),
                 len(results) - n_fail, len(results),
                 rep.get("wall_s") or 0.0))
    print("----")
    print("总通过 %d/%d（%d 片，总墙钟 %.1fs）"
          % (total_pass, total_n, len(shards), wall_all))
    if SERIAL_QUICK_REF_S:
        print("串行参考 %.0fs → 加速比 %.2fx"
              % (SERIAL_QUICK_REF_S, SERIAL_QUICK_REF_S / max(wall_all, 1e-9)))
    for sid, n, d in fails:
        print("  FAIL[片%d]: %s %s" % (sid, n, d))
    if broken:
        print("  崩溃片：%s（日志 Temp/_par_shard<id>.log 末尾有栈）"
              % [sh["id"] for sh in broken])
    bad_rc = [sh["id"] for sh in shards if sh["_rc"] != 0]
    if fs_names:
        busy = [p for p in (9010, 9101, 9102, 9190, 1883, 9200)
                if port_occupied(p)]
        if busy:
            # 诊断告警（不改退出码）：本片自己的后端由 test_client finally
            # 负责清理；仍被占=有孤儿进程，按 docs/回归基建_V1.md §6 清理
            print("[par][警告] 全栈片跑完端口仍被占：%s（疑似残留后端孤儿，"
                  "按 回归基建_V1.md §6 查杀）" % busy)
    ok = not fails and not broken and not bad_rc
    print("结论：%s" % ("全部通过" if ok else "有 FAIL/异常，见上"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
