# -*- coding: utf-8 -*-
"""chain_runner.py — 工艺一键串联（迭代 7 需求 6；迭代 8 加键合阶段）。

按序执行六阶段工艺链：wafer_ega → flip_pick → flip_handoff →
focus_level(per_site) → fine_align → bond。每阶段用 process_flows 对应
builder 生成步骤、FlowEngine 跑、等完成再下一阶段；任一阶段失败 →
链 abort 并记录失败阶段。

状态感知跳过（每阶段开始前按当前物料/测量状态判定）：
  - wafer_ega：晶圆在位且 EGA 已完成 → 跳过；
  - flip_pick：head 有 die 或 flip 有 die → 跳过；
  - flip_handoff：head 有 die → 跳过；
  - focus_level / fine_align：每次都跑（幂等设计）；
  - 本 site 已键合（die carrier=wafer 且 site==该位）→ flip_pick 起的
    工艺阶段全部幂等跳过（重跑精对准会测到已键合 die 遮挡 site 标记，
    物理无效）；
  - bond：未键合时总跑；head 无 die → build_bond_flow 守卫报错 →
    链 abort（没东西可键合是异常，不是可跳过态）。

链状态（active/current/stages/error/t_start/recovery）由 status() 暴露，
ws_gateway 合入/api/flow/state（state_public）。t_start=首个实跑阶段起点
sim_t（迭代 17 产率实时化 live 块数据源）；recovery=abort 后残片恢复
选项（迭代 17 WP-C：head 有 die→[return_film, continue_bond]，flip 有
die→[return_film]，其余 null）。FlowEngine stop 中止当前阶段流程 →
链随之中断（阶段 aborted 视为失败）。
"""
import threading
import time

import process_flows
import workpieces

STAGES = ("wafer_ega", "flip_pick", "flip_handoff", "focus_level",
          "fine_align", "bond")


class ChainRunner(object):
    def __init__(self, eng):
        self.eng = eng
        self._lock = threading.Lock()
        self._thread = None
        self._state = self._blank()
        self._stop_req = False       # full_reset 请求停链（迭代 13）

    @staticmethod
    def _blank(site=None):
        return {"active": False, "current": None,
                "site": list(site) if site is not None else None,
                "stages": {n: "wait" for n in STAGES},   # wait/run/done/fail/skip
                "error": None,
                "t_start": None}      # 首个实跑阶段起点 sim_t（迭代 17 产率实时化）

    # ---------------- 控制接口 ----------------
    def start(self, site=None):
        """启动串联链（site=None → 各 builder 缺省中心位）。已在跑 →
        FlowStateError（与 FlowEngine 409 语义一致）。"""
        from flow_engine import FlowStateError
        with self._lock:
            if self._state["active"]:
                raise FlowStateError("一键串联已在运行（阶段 %s）"
                                     % (self._state["current"],))
            self._state = self._blank(site)
            self._state["active"] = True
            self._stop_req = False
            self._thread = threading.Thread(
                target=self._run, args=(site,), name="chain", daemon=True)
            self._thread.start()
        return {"chain": True, "site": list(site) if site is not None else None,
                "stages": list(STAGES)}

    def status(self):
        with self._lock:
            return {"active": self._state["active"],
                    "current": self._state["current"],
                    "site": list(self._state["site"])
                    if self._state["site"] is not None else None,
                    "stages": dict(self._state["stages"]),
                    "error": self._state["error"],
                    "t_start": self._state.get("t_start"),
                    "recovery": self._recovery_info()}

    def _recovery_info(self):
        """abort 后残片恢复提示（迭代 17 WP-C 主控智能恢复）：非运行态且
        有 error 时按当前物料给恢复选项——head 有 die → 放回蓝膜/继续键合
        本 site 两项；flip 有 die → 仅放回蓝膜；无残片 → None。"""
        if self._state["active"] or not self._state["error"]:
            return None
        try:
            wp = self.eng.workpieces
            with wp._lock:
                head = next((d for d in wp.dies if d["carrier"] == "head"),
                            None)
                flip = next((d for d in wp.dies if d["carrier"] == "flip"),
                            None)
        except Exception:
            return None
        if head is not None:
            return {"die_carrier": "head", "die": head.get("id"),
                    "options": ["return_film", "continue_bond"]}
        if flip is not None:
            return {"die_carrier": "flip", "die": flip.get("id"),
                    "options": ["return_film"]}
        return None

    # ---------------- 一键复位（迭代 13，/api/full_reset） ----------------
    def request_stop(self):
        """请求停链：链循环在下一 stage 边界退出（当前阶段流程由
        FlowEngine.stop 中止 → 本阶段记 fail 后链自收）。"""
        self._stop_req = True

    def reset(self):
        """链状态清零（_state=_blank()）；须在链已停（active=False）后调，
        /api/full_reset 负责先 request_stop + 停流程 + 等停。"""
        with self._lock:
            self._state = self._blank()
            self._stop_req = False

    # ---------------- 内部 ----------------
    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def _stage(self, name, st):
        with self._lock:
            self._state["stages"][name] = st

    def _site_bonded(self, wp):
        """本 site 是否已键合（die carrier=wafer 且 site==该位设计坐标）。
        已键合的 site 整条链幂等跳过——重跑精对准会测到已键合 die 遮挡
        site 标记（物理无效），重复取片/交接也没有意义。"""
        site = self._state.get("site")
        try:
            rec = self.eng.recipe.get()
            cells = process_flows.wafer_sites_cells(rec)
            if site is not None:
                cell = next((c for c in cells
                             if c[0] == site[0] and c[1] == site[1]), None)
            else:
                import math as _m
                cell = min(cells, key=lambda c: _m.hypot(c[2], c[3]),
                           default=None)
        except Exception:
            cell = None
        if cell is None:
            return False
        # 统一 site_bonded 容差匹配（1µm），消灭浮点精确等值
        return workpieces.site_bonded(wp, cell[2], cell[3])

    def _skip(self, name):
        """状态感知跳过判定（每阶段开始前现查）。"""
        wp = self.eng.workpieces
        with wp._lock:
            wafer = bool(wp.wafer["present"])
            head = any(d["carrier"] == "head" for d in wp.dies)
            flip = any(d["carrier"] == "flip" for d in wp.dies)
        if name == "wafer_ega":
            return wafer and getattr(self.eng, "ega_result", None) is not None
        # 本 site 已键合：后续工艺阶段全部幂等跳过（wafer_ega 是全局
        # 对准，不受单 site 键合态影响）
        if name != "wafer_ega" and self._site_bonded(wp):
            return True
        if name == "flip_pick":
            return head or flip
        if name == "flip_handoff":
            return head
        if name == "bond":
            return False    # 未键合时总跑；head 无 die 由 builder 守卫报错
        return False                    # focus_level/fine_align 总跑

    def _build(self, name, site):
        if name == "wafer_ega":
            return process_flows.build_wafer_ega_flow(self.eng)
        if name == "flip_pick":
            return process_flows.build_flip_pick_flow(self.eng)
        if name == "flip_handoff":
            return process_flows.build_flip_handoff_flow(self.eng)
        if name == "focus_level":
            return process_flows.build_focus_level_flow(
                self.eng, mode="per_site", site=site)
        if name == "fine_align":
            return process_flows.build_fine_align_flow(self.eng, site=site)
        if name == "bond":
            return process_flows.build_bond_flow(self.eng, site=site)
        raise KeyError("未知链阶段 %r" % name)

    def _wait_flow(self, timeout=600.0):
        """等当前 FlowEngine 流程终态；返回 state 字符串。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.eng.flow.state_public()["state"]
            if st in ("done", "aborted"):
                return st
            time.sleep(0.2)
        return "timeout"

    def _sim_t(self):
        with self.eng.lock:
            return self.eng.sim_time

    def _run(self, site):
        buf = {"site": site, "die_id": None, "chain": {},
               "timing": {"stages": {}}}          # 结果采集缓冲（timing=迭代 15）
        ok_chain = False
        try:
            for name in STAGES:
                if self._stop_req:       # full_reset 停链（迭代 13）
                    self._set(error="一键复位停链")
                    return
                if self._skip(name):
                    self._stage(name, "skip")
                    # 跳过阶段同样采集快照（如 wafer_ega 跳过时上片注入/
                    # EGA 结果仍属本链数据）
                    if getattr(self.eng, "bond_results", None) is not None:
                        try:
                            import bond_results as _br
                            _br.collect_stage(self.eng, name, site, buf)
                        except Exception:
                            pass
                    continue
                self._set(current=name)
                self._stage(name, "run")
                t0 = self._sim_t()              # 阶段耗时计时起点（sim_t）
                if self._state["t_start"] is None:
                    self._set(t_start=round(t0, 3))   # 本轮首颗起点（live 块）
                try:
                    fname, steps = self._build(name, site)
                    self.eng.flow.run_steps(steps, "一键串联:%s" % fname)
                except Exception as e:
                    self._stage(name, "fail")
                    self._set(error="阶段 %s 启动失败：%s" % (name, e))
                    return
                st = self._wait_flow()
                if st != "done":
                    self._stage(name, "fail")
                    self._set(error="阶段 %s 未完成（flow state=%s）"
                                    % (name, st))
                    return
                t1 = self._sim_t()
                self._stage(name, "done")
                # ---- 迭代 15：产率计时（五阶段分阶段耗时 + 首颗取片开始/
                # 键合结束净流水口径；EGA/上片等不计入）----
                tm = buf["timing"]
                if name in ("flip_pick", "flip_handoff", "focus_level",
                            "fine_align", "bond"):
                    tm["stages"][name] = round(t1 - t0, 3)
                    if "t_start" not in tm:
                        tm["t_start"] = round(t0, 3)   # 首个实跑阶段起点
                    if name == "bond":
                        tm["t_end"] = round(t1, 3)
                # ---- 迭代 11：键合结果采集（每阶段完成后从 eng.* 快照链
                # 数据；bond 完成后落一条完整记录）。采集异常不阻断工艺链。
                br = getattr(self.eng, "bond_results", None)
                if br is not None:
                    try:
                        import bond_results as _br
                        _br.collect_stage(self.eng, name, site, buf)
                        if name == "bond":
                            br.add(_br.finalize_record(self.eng, site, buf))
                            ok_chain = True
                    except Exception as e:
                        print("[chain] bond_results 采集异常（已忽略）：%s"
                              % e, flush=True)
        finally:
            # 迭代 15：产率记账（成功=1 颗净流水 t_end−t_start；失败=1
            # 失败计数、不进 UPH 分母）
            try:
                br = getattr(self.eng, "bond_results", None)
                if br is not None and "t_start" in buf["timing"]:
                    tm = buf["timing"]
                    net = (tm["t_end"] - tm["t_start"]) \
                        if "t_end" in tm else 0.0
                    br.throughput_add(1 if ok_chain else 0,
                                      0 if ok_chain else 1,
                                      net if ok_chain else 0.0,
                                      tm["stages"] if ok_chain else None)
            except Exception:
                pass
            # 迭代 11 第二棒：单片键合链成功结束后按处方
            # post_inspect.auto_after_single 自动触发键合后检测
            try:
                if self._state["error"] is None:
                    process_flows.maybe_auto_post_inspect(self.eng, "single")
            except Exception:
                pass
            self._set(active=False, current=None)
