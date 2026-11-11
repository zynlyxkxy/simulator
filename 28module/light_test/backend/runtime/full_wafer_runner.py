# -*- coding: utf-8 -*-
"""full_wafer_runner.py — 全片键合执行器（2026-09-02 迭代 11 第二棒）。

入口 POST /api/process/full_wafer（ws_gateway）。启动时按处方
recipe.full_wafer 规划路径并生成**显式 die↔site 配对表**：

  - site 侧：recipe.wafer_sites_cells() → skip_bonded 过滤已键合位
    （workpieces.site_bonded）→ path_mode 排序（center_out=按 |x|+|y|
    升序中心向外 / serpentine=按行蛇形 / nearest=最近邻链，从离中心
    最近位出发）→ count>0 截断；
  - die 侧：film 上 die 按 die_order 排序（nearest_pick=离取片位距离
    升序 / serpentine=按行列蛇形）；
  - 两侧等长截断后一一配对（die 耗尽优雅截短）。

执行：晶圆在位且 EGA 未完成则先跑一次 wafer_ega；然后逐对跑
flip_pick(die)→flip_handoff→focus_level(site)→fine_align(site)→bond
（复用 process_flows 现有 builder 与 FlowEngine.run_steps，已键合守卫
天然防重）。每 die 用 bond_results.collect_stage/finalize_record 落
记录（buf 每 die 新起，与单片 chain 同一收集器）。site 失败按处方
stop_on_fail（默认 false=记 fail 继续下一颗）。结束后按处方
post_inspect.auto_after_full 自动触发键合后检测。

迭代 15（2026-09-05）：
- 规划三值：_plan 返回 (pairs, plan_info)——可用 site 数/膜上 die 数/
  配对数（含处方派生对照，膜布局陈旧一眼定位），启动响应带出+流程
  首条日志；
- 产率计时：每 die 记 t_start（取片开始）/t_end（键合结束，sim_t）+
  五阶段分阶段耗时，存进 bond_results 记录 timing 与 run 级
  throughput_add（UPH=成功颗数/净流水时间；EGA/上片/闲置不计入）；
- 产率模式 pipelined（处方 full_wafer.pipelined，默认 false 串行）：
  当前颗 flip_handoff 完成（芯片已上键合头）→ 立即为下一颗在**第二
  FlowEngine 实例**上执行 flip_pick+粗对准（取片交接链不碰键合头/
  硅片台/对准台资源，触发窗口=交接完成后即可，参照参考项目 backend
  安全相位），完成后携片到待机位（机械常数 flip_handoff.
  fa_x_standby_mm，初值待真机标定）等待；当前颗 bond 完成→立即
  flip_handoff 下一颗。预取失败记 window_missed 回退串行、不影响
  当前颗；预取重叠时间 overlap_s 单列记账（参照 backend overlap_ms）。

状态 status() 由 ws_gateway 合入 /api/flow/state 的 st["full_wafer"]：
{active, progress:[i,n], pairs:[{die, site:{row,col}}...],
 current:{i,die,site,stage,t_start}|null, results:[{die,site,ok,error?,
 prefetch_error?,recovered?}...], error, plan_info, window_missed,
 overlap_s, run_t_first}。
FlowEngine stop 中止当前阶段流程 → 该 site 记 fail（abort 视为
失败），stop_on_fail=false 时仍继续后续配对（人工 stop 语义=跳过当前
颗；要整链中止用 stop_on_fail=true 或 estop）。

迭代 17 WP-C（2026-09-06）主控智能恢复：
- 单对失败且 stop_on_fail=false → 自动放回残片：die 在 head →
  build_head_to_flip_flow + build_flip_place_back_flow 放回蓝膜
  （result 条目加 recovered:true，恢复流程首步 log 留证）；die 在
  flip → 直接 flip_place_back；恢复失败 → 整批 abort。
- start(clear_head=True)：head/flip 有残片时立即返回 clearing:true，
  runner 线程内异步放回蓝膜后再规划启动（不堵 HTTP 响应）；不带
  clear_head 时直接 409（错误文案提示 clear_head 选项，ws_gateway
  附 clear_head_option:true）。
  已知限制：pipelined 模式下预取已把下一颗放上 flip 时，head_to_flip
  会撞"翻转吸盘已有芯片"守卫 → 恢复失败整批 abort（符合语义）。
"""
import math
import threading
import time

import process_flows
from recipe import wafer_sites_cells
from workpieces import site_bonded

STAGES = ("flip_pick", "flip_handoff", "focus_level", "fine_align", "bond")


# ---------------- 路径规划（纯函数，可单测） ----------------
def plan_sites(cells, bonded_fn, path_mode, count):
    """site 排序。cells=[(r,c,x,y)]；bonded_fn(x,y)->bool（None=不过滤）。
    path_mode ∈ center_out/serpentine/nearest；count>0 截断。返回
    [(r,c,x,y),...]。"""
    avail = [c for c in cells
             if bonded_fn is None or not bonded_fn(c[2], c[3])]
    if path_mode == "center_out":
        avail.sort(key=lambda c: (abs(c[2]) + abs(c[3]), c[0], c[1]))
    elif path_mode == "serpentine":
        rows = {}
        for c in avail:
            rows.setdefault(c[0], []).append(c)
        out = []
        for i, r in enumerate(sorted(rows)):
            row = sorted(rows[r], key=lambda c: c[1])
            if i % 2:
                row.reverse()
            out.extend(row)
        avail = out
    elif path_mode == "nearest":
        # 最近邻链：从离中心最近位出发，逐步取离上一位最近的未访问位
        remaining = list(avail)
        out = []
        if remaining:
            cur = min(remaining,
                      key=lambda c: (math.hypot(c[2], c[3]), c[0], c[1]))
            while remaining:
                remaining.remove(cur)
                out.append(cur)
                if remaining:
                    cur = min(remaining,
                              key=lambda c: ((c[2] - out[-1][2]) ** 2
                                             + (c[3] - out[-1][3]) ** 2,
                                             c[0], c[1]))
        avail = out
    else:
        raise ValueError("path_mode 非法：%r（合法：center_out/serpentine/"
                         "nearest）" % (path_mode,))
    if count and count > 0:
        avail = avail[:int(count)]
    return avail


def order_dies(dies, die_order, pick_xy=None, tool_xy=None):
    """film die 排序。dies=[{id, film_pos:[x,y]}]。
    nearest_pick=离取片位距离升序（die 世界 ≈ 工具点 + film_pos，同
    build_flip_pick_flow 缺省选片口径；pick_xy/tool_xy 缺省退化为离膜面
    圆心距离）；serpentine=按行列蛇形（行升序，奇行反向）。"""
    dies = list(dies)
    if die_order == "nearest_pick":
        if pick_xy is not None and tool_xy is not None:
            ox = float(pick_xy[0]) - float(tool_xy[0])
            oy = float(pick_xy[1]) - float(tool_xy[1])
        else:
            ox = oy = 0.0
        return sorted(dies, key=lambda d:
                      ((d["film_pos"][0] - ox) ** 2
                       + (d["film_pos"][1] - oy) ** 2, d["id"]))
    if die_order == "serpentine":
        rows = {}
        for d in dies:
            r = int(d["id"][1:3])          # id 形如 R%02dC%02d
            rows.setdefault(r, []).append(d)
        out = []
        for i, r in enumerate(sorted(rows)):
            row = sorted(rows[r], key=lambda d: d["id"])
            if i % 2:
                row.reverse()
            out.extend(row)
        return out
    raise ValueError("die_order 非法：%r（合法：nearest_pick/serpentine）"
                     % (die_order,))


class FullWaferRunner(object):
    def __init__(self, eng):
        self.eng = eng
        self._lock = threading.Lock()
        self._thread = None
        self._state = self._blank()
        self._stop_req = False       # full_reset 请求停链（迭代 13）
        self._flow2 = None           # 预取专用第二流程引擎（迭代 15，懒创建）

    @staticmethod
    def _blank():
        return {"active": False, "progress": [0, 0], "pairs": [],
                "current": None, "results": [], "error": None,
                "plan_info": None, "window_missed": 0, "overlap_s": 0.0,
                "run_t_first": None}   # 本轮首颗取片起点 sim_t（迭代 17 live）

    # ---------------- 控制接口 ----------------
    def start(self, clear_head=False):
        """规划 + 启动全片键合。已在跑/单片链在跑 → FlowStateError（409
        语义）；规划期守卫失败 → ValueError（400 语义）。
        clear_head（迭代 17 WP-C 主控智能恢复）：启动时 head/flip 上有
        残片 → 不带 clear_head 直接 FlowStateError（提示带 clear_head=
        true）；带 clear_head=true → **异步**清残片（放回流程分钟级耗时，
        不能堵 HTTP 响应）：立即返回 clearing:true，runner 线程内先
        head_to_flip/flip_place_back 放回蓝膜再规划启动，放回/规划失败
        记 error 收批（status 可见）。"""
        from flow_engine import FlowStateError
        with self._lock:
            # 运行中重复启动的 409 优先于残片提示（运行中 head 正常持
            # die，不能被误判成"残片"）
            if self._state["active"]:
                raise FlowStateError("全片键合已在运行（%d/%d）"
                                     % tuple(self._state["progress"]))
            ch = getattr(self.eng, "chain", None)
            if ch is not None and ch.status().get("active"):
                raise FlowStateError("单片键合链在运行中，不能启动全片键合")
        head_die, flip_die = self._carriers()
        clear_first = (head_die is not None or flip_die is not None)
        if clear_first and not clear_head:
            raise FlowStateError(
                "键合头%s翻转吸盘%s上有残片（%s），不能规划启动——先手"
                "动放回（吸回翻转头+放回蓝膜），或请求带 "
                "clear_head=true 自动放回后启动"
                % ("" if head_die is None else "、",
                   "" if flip_die is None else "、",
                   "/".join(x for x in (head_die, flip_die) if x)))
        with self._lock:
            if self._state["active"]:
                raise FlowStateError("全片键合已在运行（%d/%d）"
                                     % tuple(self._state["progress"]))
            if clear_first:
                self._state = self._blank()
                self._state["active"] = True
                self._stop_req = False
                self._state["current"] = {"i": -1, "die": None, "site": None,
                                          "stage": "clear_head"}
                self._thread = threading.Thread(
                    target=self._run_with_clear,
                    args=(head_die is not None,), name="full_wafer",
                    daemon=True)
                self._thread.start()
                return {"full_wafer": True, "clearing": True,
                        "pairs": None, "plan": [],
                        "sites_avail": None, "dies_on_film": None,
                        "dies_expected": None}
            pairs, plan_info = self._plan()  # 规划期守卫（400 族）
            self._state = self._blank()
            self._state["active"] = True
            self._stop_req = False
            self._state["progress"] = [0, len(pairs)]
            self._state["plan_info"] = dict(plan_info)
            self._state["pairs"] = [{"die": p["die"],
                                     "site": {"row": p["site"][0],
                                              "col": p["site"][1]}}
                                    for p in pairs]
            self._thread = threading.Thread(
                target=self._run, args=(pairs, plan_info), name="full_wafer",
                daemon=True)
            self._thread.start()
        return {"full_wafer": True, "pairs": len(pairs),
                "plan": self._state["pairs"],
                "sites_avail": plan_info["sites_avail"],
                "dies_on_film": plan_info["dies_on_film"],
                "dies_expected": plan_info["dies_expected"],
                "dies_consumed": plan_info["dies_consumed"],
                "film_layout_mismatch": plan_info["film_layout_mismatch"]}

    def _run_with_clear(self, head_first):
        """clear_head 启动路径：runner 线程内先放回残片再规划+执行。"""
        try:
            self._clear_carriers(head_first)
            pairs, plan_info = self._plan()
            with self._lock:
                self._state["progress"] = [0, len(pairs)]
                self._state["plan_info"] = dict(plan_info)
                self._state["pairs"] = [{"die": p["die"],
                                         "site": {"row": p["site"][0],
                                                  "col": p["site"][1]}}
                                        for p in pairs]
                self._state["current"] = None
        except Exception as e:
            self._set(current=None, error="clear_head 恢复/规划失败：%s" % e,
                      active=False)
            return
        self._run(pairs, plan_info)

    def status(self):
        with self._lock:
            return {"active": self._state["active"],
                    "progress": list(self._state["progress"]),
                    "pairs": [dict(p) for p in self._state["pairs"]],
                    "current": (dict(self._state["current"])
                                if self._state["current"] else None),
                    "results": [dict(r) for r in self._state["results"]],
                    "error": self._state["error"],
                    "plan_info": (dict(self._state["plan_info"])
                                  if self._state.get("plan_info") else None),
                    "window_missed": self._state.get("window_missed", 0),
                    "overlap_s": round(self._state.get("overlap_s", 0.0), 3),
                    "run_t_first": self._state.get("run_t_first")}

    # ---------------- 一键复位（迭代 13，/api/full_reset） ----------------
    def request_stop(self):
        """请求停链：当前 site 的阶段流程由 FlowEngine.stop 中止记 fail，
        配对/阶段循环在下一边界退出（stop_on_fail 语义不变，人工停优先）。
        预取线程（flow2，迭代 15）同样请求停止。"""
        self._stop_req = True
        if self._flow2 is not None:
            try:
                self._flow2.stop()
            except Exception:
                pass

    def _state_overlap(self, ov):
        with self._lock:
            self._state["overlap_s"] = round(
                self._state.get("overlap_s", 0.0) + ov, 3)

    def _state_window_missed(self):
        with self._lock:
            self._state["window_missed"] = \
                self._state.get("window_missed", 0) + 1

    def reset(self):
        """状态清零（_state=_blank()）；须在 active=False 后调。"""
        with self._lock:
            self._state = self._blank()
            self._stop_req = False

    # ---------------- 规划 ----------------
    def _plan(self):
        """规划配对表。返回 (pairs, plan_info)；plan_info={sites_total,
        sites_avail, dies_on_film, pairs}——启动响应/首条日志的「可用 site
        数/膜上 die 数/配对数」三值（迭代 15：24 颗问题排查落地）。"""
        eng = self.eng
        if eng.recipe is None or eng.workpieces is None \
                or eng.constants is None:
            raise RuntimeError("full_wafer 需要 engine.recipe/workpieces/"
                               "constants")
        wp = eng.workpieces
        with wp._lock:
            wafer_present = bool(wp.wafer["present"])
            film_present = bool(wp.film["present"])
            dies = [dict(d) for d in wp.dies if d["carrier"] == "film"]
        if not wafer_present:
            raise ValueError("晶圆不在位（先上片：/api/process/wafer_ega）")
        if not film_present or not dies:
            raise ValueError("蓝膜不在位或膜上无可用 die")
        rec = eng.recipe.get()
        fw = rec.get("full_wafer") or {}
        cells = wafer_sites_cells(rec)
        if not cells:
            raise ValueError("处方无晶圆键合位（wafer_sites 为空）")
        bonded_fn = (lambda x, y: site_bonded(wp, x, y)) \
            if bool(fw.get("skip_bonded", True)) else None
        sites_all = plan_sites(cells, bonded_fn,
                               fw.get("path_mode", "center_out"), 0)
        cnt = int(fw.get("count", 0))
        sites = sites_all[:cnt] if cnt > 0 else sites_all
        if not sites:
            raise ValueError("无可用键合位（全部已键合/处方为空）")
        mc = eng.constants.get()["flip_pick"]
        kp = eng.kinematics.config_public()
        tool_xy = kp["modules"]["film_stage"]["tool_point"][:2]
        dies = order_dies(dies, fw.get("die_order", "nearest_pick"),
                          mc["pick_point_xy_mm"], tool_xy)
        n = min(len(sites), len(dies))
        pairs = [{"die": dies[i]["id"], "site": sites[i]}
                 for i in range(n)]
        # 膜上 die 布局签名 ≠ 当前处方派生 ⇒ 膜是旧处方上膜时铺的（迭代 15
        # 根因，迭代 23 改签名判据——键合消耗导致的少于处方不再误报）
        from recipe import derive_dies
        lay = wp.layout_public()
        info = {"sites_total": len(cells), "sites_avail": len(sites_all),
                "dies_on_film": len(dies),
                "dies_expected": int(derive_dies(rec)["count"]),
                "dies_consumed": lay["dies_consumed"],
                "film_layout_mismatch": lay["film_layout_mismatch"],
                "pairs": n}
        return pairs, info

    # ---------------- 内部 ----------------
    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def _result(self, rec):
        with self._lock:
            self._state["results"].append(rec)

    # ---------------- 残片恢复（迭代 17 WP-C 主控智能恢复） ----------------
    def _carriers(self):
        """当前 head/flip 上的 die id（无则 None）。"""
        try:
            with self.eng.workpieces._lock:
                head = next((d["id"] for d in self.eng.workpieces.dies
                             if d["carrier"] == "head"), None)
                flip = next((d["id"] for d in self.eng.workpieces.dies
                             if d["carrier"] == "flip"), None)
            return head, flip
        except Exception:
            return None, None

    def _die_carrier(self, die_id):
        try:
            with self.eng.workpieces._lock:
                for d in self.eng.workpieces.dies:
                    if d["id"] == die_id:
                        return d["carrier"]
        except Exception:
            pass
        return None

    def _run_recovery_flow(self, build, tag, log_msg):
        """跑一个恢复流程（带 log 首步留证），等终态；未完成返回错误串。"""
        fname, steps = build(self.eng)
        steps = [{"op": "log", "msg": log_msg,
                  "desc": "残片恢复（迭代 17 主控智能恢复）"}] + steps
        self.eng.flow.run_steps(steps, "全片键合:残片恢复:%s" % tag)
        st = self._wait_flow()
        if st != "done":
            return "%s 未完成（flow state=%s）" % (tag, st)
        return None

    def _recover_die(self, die_id, carrier):
        """失败后把残片放回蓝膜：die 在 head → 先吸回翻转头再放回；
        die 在 flip → 直接放回。返回 (ok, error)。"""
        try:
            if carrier == "head":
                err = self._run_recovery_flow(
                    process_flows.build_head_to_flip_flow,
                    "head_to_flip",
                    "残片恢复：die %s 在键合头，先吸回翻转拾取头" % die_id)
                if err:
                    return False, err
            err = self._run_recovery_flow(
                process_flows.build_flip_place_back_flow,
                "flip_place_back",
                "残片恢复：die %s 放回蓝膜原槽位" % die_id)
            if err:
                return False, err
            return True, None
        except Exception as e:
            return False, "恢复流程异常：%s" % e

    def _clear_carriers(self, head_first):
        """清残片（clear_head 启动路径/_run_with_clear 调用）：head 有
        die 先吸回翻转头，再把翻转吸盘上的 die 放回蓝膜；失败抛
        RuntimeError（调用侧记 error 收批）。"""
        head_die, flip_die = self._carriers()
        if head_first and head_die is not None:
            err = self._run_recovery_flow(
                process_flows.build_head_to_flip_flow,
                "head_to_flip",
                "启动前清残片：die %s 在键合头，先吸回翻转拾取头"
                % head_die)
            if err:
                raise RuntimeError("clear_head 恢复失败：%s" % err)
        _, flip_die = self._carriers()
        if flip_die is not None:
            err = self._run_recovery_flow(
                process_flows.build_flip_place_back_flow,
                "flip_place_back",
                "启动前清残片：die %s 放回蓝膜原槽位" % flip_die)
            if err:
                raise RuntimeError("clear_head 恢复失败：%s" % err)

    def _build(self, name, site, die_id):
        if name == "flip_pick":
            return process_flows.build_flip_pick_flow(self.eng, die_id)
        if name == "flip_handoff":
            return process_flows.build_flip_handoff_flow(self.eng)
        if name == "focus_level":
            return process_flows.build_focus_level_flow(
                self.eng, mode="per_site", site=site)
        if name == "fine_align":
            return process_flows.build_fine_align_flow(self.eng, site=site)
        if name == "bond":
            return process_flows.build_bond_flow(self.eng, site=site)
        raise KeyError("未知阶段 %r" % name)

    def _wait_flow(self, timeout=900.0):
        """等当前 FlowEngine 流程终态；返回 state 字符串。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._stop_req:
                return "stopped"
            st = self.eng.flow.state_public()["state"]
            if st in ("done", "aborted"):
                return st
            time.sleep(0.2)
        return "timeout"

    # ---------------- 产率计时/流水线辅助（迭代 15） ----------------
    def _sim_t(self):
        with self.eng.lock:
            return self.eng.sim_time

    def _get_flow2(self):
        """预取专用第二 FlowEngine 实例（懒创建）：主 eng.flow 跑当前颗的
        调平/精对准/键合，预取链（取片+粗对准+待机走位）走本实例——两链
        轴系不相交（FA/ES vs AS/WS/BS），碰撞检查在引擎层兜底。"""
        if self._flow2 is None:
            from flow_engine import FlowEngine
            self._flow2 = FlowEngine(self.eng, self.eng.flow.points,
                                     self.eng.flow.flows_path)
        return self._flow2

    def _wait_flow2(self, timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self._flow2.state_public()["state"]
            if st in ("done", "aborted"):
                return st
            time.sleep(0.2)
        return "timeout"

    def _start_prefetch(self, die_id):
        """为下一颗 die 启动预取线程：flip_pick(die)（含粗对准）→ 携片到
        待机位（机械常数 flip_handoff.fa_x_standby_mm，待真机标定）。
        触发窗口=当前颗 flip_handoff 完成后（取片交接链不碰硅片台/对准
        台/键合头资源；参照参考项目 backend 安全相位窗口）。预取异常不
        影响当前颗：只记 pf["error"]，轮到该 die 时回退串行取片并记
        window_missed。返回预取状态 dict。"""
        pf = {"die": die_id, "ok": False, "error": None,
              "t_start": None, "t_end": None, "thread": None}

        def _work():
            try:
                eng = self.eng
                fname, steps = process_flows.build_flip_pick_flow(
                    eng, die_id)
                fh = eng.constants.get()["flip_handoff"]
                fp = eng.constants.get()["flip_pick"]
                standby = float(fh.get("fa_x_standby_mm", -1050.0))
                steps = steps + [
                    {"op": "move", "axis": "FA_Z",
                     "target": float(fp["fa_z_safe_mm"]),
                     "desc": "预取收尾：翻转 Z 回安全位"},
                    {"op": "move", "axis": "FA_X", "target": standby,
                     "desc": "携片到待机位 fa_x_standby_mm=%.2f（等当前颗"
                             "键合结束；初值待真机标定）" % standby},
                    {"op": "log",
                     "msg": "预取完成：die %s 已在待机位（FA_X=%.2f）"
                            % (die_id, standby),
                     "desc": "预取完成"}]
                pf["t_start"] = round(self._sim_t(), 3)
                flow2 = self._get_flow2()
                flow2.run_steps(steps, "全片键合:预取:%s" % fname)
                st = self._wait_flow2()
                pf["t_end"] = round(self._sim_t(), 3)
                if st == "done":
                    pf["ok"] = True
                else:
                    # 失败明细就地留证（flow2 日志/失败步骤不进主状态）
                    try:
                        s2 = flow2.state_public()
                        fails = [x for x in (s2.get("steps") or [])
                                 if x.get("status") == "fail"]
                        det = (fails[0].get("info") if fails else "") or ""
                        if not det:
                            logs = s2.get("log") or []
                            det = (logs[-1].get("msg") if logs else "") or ""
                    except Exception:
                        det = ""
                    pf["error"] = "预取流程未完成（flow2 state=%s）%s" % (
                        st, ("：" + det[:160]) if det else "")
            except Exception as e:
                pf["t_end"] = round(self._sim_t(), 3)
                pf["error"] = "预取失败：%s" % e

        th = threading.Thread(target=_work, name="fw-prefetch", daemon=True)
        pf["thread"] = th
        th.start()
        return pf

    @staticmethod
    def _join_pf(pf):
        th = pf.get("thread")
        if th is not None:
            th.join()

    def _run(self, pairs, plan_info):
        eng = self.eng
        rec = eng.recipe.get()
        fw = rec.get("full_wafer") or {}
        stop_on_fail = bool(fw.get("stop_on_fail", False))
        pipelined = bool(fw.get("pipelined", False))
        n = len(pairs)
        # 首条日志（迭代 15）：可用 site 数/膜上 die 数/配对数三值——插到
        # 第一个执行的流程（EGA 或首个阶段）开头，24 颗类问题一眼定位
        plan_log = [{
            "op": "log",
            "msg": ("全片键合规划：可用 site %d 个（处方派生 %d）/ 膜上 die "
                    "%d 颗（处方派生 %d）/ 配对 %d 对%s%s%s"
                    % (plan_info["sites_avail"], plan_info["sites_total"],
                       plan_info["dies_on_film"], plan_info["dies_expected"],
                       plan_info["pairs"],
                       "；产率模式 pipelined（预取流水线）" if pipelined
                       else "",
                       "；⚠ 膜上 die 布局与当前处方不符（膜是旧处方上膜时铺"
                       "的），请重新上膜"
                       if plan_info["film_layout_mismatch"] else "",
                       "；膜上 die 已消耗 %d 颗（键合/取片消耗，非布局不"
                       "符），配对按剩余 die 数截短为 %d 对"
                       % (plan_info["dies_consumed"], plan_info["pairs"])
                       if not plan_info["film_layout_mismatch"]
                       and plan_info["dies_consumed"] > 0
                       and plan_info["pairs"] < plan_info["sites_avail"]
                       else "")),
            "desc": "全片规划摘要（可用 site/膜上 die/配对数三值，迭代 15；"
                    "迭代 23 起陈膜判定改铺膜签名，消耗单独说明）"}]
        # 产率记账（run 级）：成功/失败/净流水/分阶段耗时/overlap
        tp = {"ok": 0, "fail": 0, "t_first": None, "t_last": None,
              "stage_sums": {}, "overlap_s": 0.0}
        pf = None                     # 下一颗 die 的预取状态
        prev_bond_end = None          # 上一颗 bond 结束 sim_t（overlap 口径）
        try:
            # EGA 只做一次（晶圆在位 + EGA 完成则跳过）
            if getattr(eng, "ega_result", None) is None:
                self._set(current={"i": -1, "die": None, "site": None,
                                   "stage": "wafer_ega"})
                try:
                    fname, steps = process_flows.build_wafer_ega_flow(eng)
                    eng.flow.run_steps(plan_log + steps, "全片键合:%s" % fname)
                    plan_log = []
                except Exception as e:
                    self._set(error="EGA 启动失败：%s" % e)
                    return
                if self._wait_flow() != "done":
                    self._set(error="EGA 未完成（flow state≠done）")
                    return
            for i, pair in enumerate(pairs):
                if self._stop_req:       # full_reset 停链（迭代 13）
                    self._set(error="一键复位停链")
                    return
                die_id = pair["die"]
                r, c, sx, sy = pair["site"]
                site = [r, c]
                buf = {"site": site, "die_id": die_id, "chain": {},
                       "timing": {"stages": {}}}
                ok, err = True, None
                prefetch_err = None
                for stage in STAGES:
                    if self._stop_req:
                        ok, err = False, "一键复位停链"
                        break
                    cur_new = {"i": i, "die": die_id, "site": site,
                               "stage": stage}
                    # t_start 随 current 逐阶段携带（迭代 17 live 当前颗已
                    # 耗时数据源；否则只在 flip_pick 起点挂一次会被下阶段
                    # 覆盖丢）
                    if "t_start" in buf["timing"]:
                        cur_new["t_start"] = buf["timing"]["t_start"]
                    self._set(current=cur_new)
                    t0 = self._sim_t()
                    use_pf = (stage == "flip_pick" and pipelined
                              and pf is not None and pf.get("die") == die_id)
                    if use_pf:
                        self._join_pf(pf)      # 等预取收尾（正常早已完成）
                    # live 块数据源（迭代 17）：t_start=当前颗起点（预取命中
                    # 时=预取启动 sim_t）——阶段一开始就记，current 全程带
                    # t_start；本轮首颗起点挂 run_t_first
                    if "t_start" not in buf["timing"]:
                        buf["timing"]["t_start"] = round(
                            pf["t_start"] if use_pf and pf.get("t_start")
                            else t0, 3)
                        cur = dict(self._state.get("current") or {})
                        if cur:
                            cur["t_start"] = buf["timing"]["t_start"]
                            self._set(current=cur)
                        if self._state.get("run_t_first") is None:
                            self._set(run_t_first=buf["timing"]["t_start"])
                    if use_pf and pf["ok"]:
                        # 预取命中：跳过取片（die 已在待机位），耗时=预取
                        # 时长；overlap=预取与上一颗后段并行的时间
                        buf["timing"]["stages"]["flip_pick"] = round(
                            (pf["t_end"] or t0) - pf["t_start"], 3)
                        if prev_bond_end is not None \
                                and pf["t_start"] is not None:
                            ov = min(pf["t_end"], prev_bond_end) \
                                - pf["t_start"]
                            if ov > 0:
                                tp["overlap_s"] += ov
                                self._state_overlap(ov)
                    else:
                        if use_pf:
                            # 预取失败/未完成：记 window_missed 回退串行
                            # （不影响当前颗——当前颗键合早已完成）
                            self._state_window_missed()
                            prefetch_err = pf.get("error") or "预取未命中"
                            # 预取可能已把 die 拾到 flip 才 abort（半成品），
                            # 直接串行取片会撞"翻转吸盘已有芯片"——先放回
                            # 蓝膜恢复干净起点再走串行
                            try:
                                with eng.workpieces._lock:
                                    on_flip = any(
                                        d["id"] == die_id
                                        and d["carrier"] == "flip"
                                        for d in eng.workpieces.dies)
                            except Exception:
                                on_flip = False
                            if on_flip:
                                try:
                                    fn2, st2 = (process_flows.
                                                build_flip_place_back_flow(
                                                    eng))
                                    eng.flow.run_steps(
                                        st2, "全片键合:预取回退:%s" % fn2)
                                    if self._wait_flow() != "done":
                                        ok, err = False, ("预取回退（放回蓝"
                                                          "膜）未完成")
                                except Exception as e:
                                    ok, err = False, ("预取回退（放回蓝膜）"
                                                      "失败：%s" % e)
                                if not ok:
                                    break
                        try:
                            fname, steps = self._build(stage, site, die_id)
                            if plan_log:        # EGA 跳过时规划摘要插首阶段
                                steps = plan_log + steps
                                plan_log = []
                            eng.flow.run_steps(steps, "全片键合:%s" % fname)
                        except Exception as e:
                            ok, err = False, "阶段 %s 启动失败：%s" % (stage, e)
                        else:
                            st = self._wait_flow()
                            if st != "done":
                                ok, err = False, ("阶段 %s 未完成（flow "
                                                  "state=%s）" % (stage, st))
                        t1 = self._sim_t()
                        buf["timing"]["stages"][stage] = round(t1 - t0, 3)
                    # 结果采集（与单片 chain 同一收集器；采集异常不阻断）
                    br = getattr(eng, "bond_results", None)
                    if br is not None:
                        try:
                            import bond_results as _br
                            _br.collect_stage(eng, stage, site, buf)
                        except Exception:
                            pass
                    if not ok:
                        break
                    # ---- 产率模式：当前颗 flip_handoff 完成（芯片已上键合
                    # 头）→ 立即为下一颗预取（取片+粗对准+到待机位；不占用
                    # 键合头/硅片台/对准台资源，窗口=交接完成后即可）----
                    if stage == "flip_handoff" and pipelined \
                            and i + 1 < n and not self._stop_req:
                        pf = self._start_prefetch(pairs[i + 1]["die"])
                    if stage == "bond":
                        buf["timing"]["t_end"] = round(self._sim_t(), 3)
                        prev_bond_end = buf["timing"]["t_end"]
                if ok:
                    br = getattr(eng, "bond_results", None)
                    if br is not None:
                        try:
                            import bond_results as _br
                            br.add(_br.finalize_record(eng, site, buf))
                        except Exception as e:
                            print("[full_wafer] bond_results 采集异常"
                                  "（已忽略）：%s" % e, flush=True)
                    rec2 = {"die": die_id,
                            "site": {"row": r, "col": c}, "ok": True}
                    if prefetch_err:
                        rec2["prefetch_error"] = prefetch_err
                    self._result(rec2)
                    # 产率记账：成功颗
                    tp["ok"] += 1
                    tm = buf["timing"]
                    if tp["t_first"] is None:
                        tp["t_first"] = tm.get("t_start")
                    tp["t_last"] = tm.get("t_end", tp["t_last"])
                    for k, v in (tm.get("stages") or {}).items():
                        tp["stage_sums"][k] = tp["stage_sums"].get(k, 0.0) + v
                else:
                    rec2 = {"die": die_id,
                            "site": {"row": r, "col": c},
                            "ok": False, "error": err}
                    if prefetch_err:
                        rec2["prefetch_error"] = prefetch_err
                    # 迭代 17 WP-C 主控智能恢复：stop_on_fail=false 时单对
                    # 失败后自动放回残片（head→吸回翻转头+放回蓝膜；flip→
                    # 直接放回），不放回会让下一对 flip_handoff 守卫 400 连
                    # 锁失败；恢复失败 → 整批 abort
                    if not stop_on_fail:
                        carrier = self._die_carrier(die_id)
                        if carrier in ("head", "flip"):
                            okr, rerr = self._recover_die(die_id, carrier)
                            rec2["recovered"] = okr
                            if not okr:
                                rec2["recovery_error"] = rerr
                                self._result(rec2)
                                tp["fail"] += 1
                                self._set(error="残片恢复失败（die %s 在 "
                                          "%s）：%s" % (die_id, carrier,
                                                        rerr))
                                break
                    self._result(rec2)
                    tp["fail"] += 1
                    if stop_on_fail:
                        self._set(error="stop_on_fail：%s" % err)
                        break
                self._set(progress=[i + 1, n])
        finally:
            # 收尾前等预取线程退出（stop/异常路径；预取异常已就地记账）
            if pf is not None and pf.get("thread") is not None:
                try:
                    pf["thread"].join(timeout=5.0)
                except Exception:
                    pass
            # 产率记账（迭代 15）：净流水=首颗取片开始→末颗键合结束
            # （EGA/上片/闲置不计入）；UPH=3600×ok/net
            try:
                br = getattr(eng, "bond_results", None)
                if br is not None and (tp["ok"] or tp["fail"]):
                    net = ((tp["t_last"] - tp["t_first"])
                           if tp["t_first"] is not None
                           and tp["t_last"] is not None else 0.0)
                    br.throughput_add(tp["ok"], tp["fail"],
                                      max(0.0, net), tp["stage_sums"],
                                      tp["overlap_s"],
                                      stage_n=tp["ok"])
            except Exception:
                pass
            # 全片结束后按处方 auto_after_full 自动触发键合后检测
            try:
                any_ok = any(r2.get("ok") for r2 in self._state["results"])
                if self._state["error"] is None and any_ok:
                    process_flows.maybe_auto_post_inspect(eng, "full")
            except Exception:
                pass
            self._set(active=False, current=None)
