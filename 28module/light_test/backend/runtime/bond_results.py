# -*- coding: utf-8 -*-
"""bond_results.py — 键合结果收集器（2026-09-02 迭代 11，第一棒 A/B）。

每颗 die 键合完成落一条记录：
  {die_id, site:{row,col,x,y}, t_bond, truth:{dx,dy,drz}|null,
   measured:null|{dx,dy,drz,t}, chain:{...误差传递链}, bond_params:{...},
   bond_offset:{x_um,y_um,rz_urad}（2026-09-09 迭代 23：键合时刻补偿快照，
   统计分代依据）}
truth = 键合偏移真值（全线单位约定 mm/rad）。2026-09-07 迭代 20 起取
**键合接触时刻**真值（eng.bond_commit，bond_truth 步在触力停轴后用
6DOF 真值位姿走 fa 同一 fit 管线采样，键合头下降链进入）+ 压合微滑移
工艺项；在此之前取精对准 fine_align_result.phys_resid 末轮换算（fa
完成时刻快照，键合头下降链不含——物理上缺失，迭代 20 修正；fa 时刻
值保留为 align_resid 分量）。无接触时刻真值且无精对准结果时
truth=null 并在 truth_note 注明。measured 由键合后精度检测流程
（迭代 11 第二棒 post_inspect）回填，常态为 null。**measured 语义
（2026-09-05 迭代 15 起）=套刻相对测量：红外套刻同一机位两次独立
触发（迭代 20 起；quad=site 标记 / cross=die 角标），双组点集各自拟
合后取组间刚体偏差——EGA 拟合残差/下相机绝对位姿系统差/逐标记 AS
重复定位全部对消，与 truth（纯物理残差）同量级可比**；迭代 11~14
旧口径（逐标记 EGA map 归算）混入上述系统差，数值比 truth 大得多。

采集钩子：chain_runner 每阶段完成后调 collect_stage() 从 eng.* 快照
该 site 的链数据（上片注入 offset/rz → 取片粗对准偏移 coarse_offset →
交接倾斜 die.rx/ry → 调平残余 fl_fit → 精对准逐轮 rounds+phys_resid →
键合参数+接触时刻真值分量），bond 阶段完成后 finalize_record() 落一条
完整记录。单片（chain）与全片（迭代 11 第二棒 full_wafer）走同一收集器；
采集异常不阻断工艺链（调用侧 try/except）。

**truth 口径（2026-09-07 迭代 20 起）**：truth = 键合接触时刻真值
（eng.bond_commit：触力停轴后用 6DOF 真值位姿走精对准同一 fit 管线算
得的 die↔site 残差——键合头下降链（BS_Arm_Z 地图差系统项+阿贝项）
自此进入）+ 压合微滑移（process_shift，bond.bond_shift_*，默认 0）；
fa 时刻的 phys_resid 保留为 align_resid 分量。记录带 truth_components
分量分解 {align_resid, descent, process_shift, commit}（µm/µrad 展示
单位），面板结果分析页"误差传递链"末卡消费。无 bond_commit 的老流程
回退 phys_resid 快照口径（truth_note 注明）。详见
docs/键合误差模型_V1.md。

持久化：runtime/bond_results.json（tmp+os.replace 原子替换），启动加载；
POST /api/workpiece/reset、POST /api/full_reset（2026-09-09 迭代 23 起，
此前历史保留——与 workpiece/reset 自相矛盾）与 POST /api/bond/clear
（结果分析页"清空记录"按钮）均清空。
API（ws_gateway）：GET /api/bond/results（全量列表+统计摘要）、
GET /api/bond/results?die=<id>（单条含完整 chain）、POST /api/bond/clear。
"""
import copy
import json
import math
import os
import threading


def _r6(v):
    return round(float(v), 6)


def _site_cell(rec, site):
    """site=[row,col] 查格点 (r,c,x,y)；site=None → 离中心最近位（同
    chain_runner/builder 缺省规则）。找不到返回 None。"""
    from recipe import wafer_sites_cells
    cells = wafer_sites_cells(rec)
    if site is not None:
        return next((c for c in cells if c[0] == site[0] and c[1] == site[1]),
                    None)
    import math as _m
    return min(cells, key=lambda c: _m.hypot(c[2], c[3]), default=None)


def _die_on(eng, carrier):
    wp = getattr(eng, "workpieces", None)
    if wp is None:
        return None
    with wp._lock:
        for d in wp.dies:
            if d["carrier"] == carrier:
                return dict(d)
    return None


def collect_stage(eng, stage, site, buf):
    """单阶段采集：把该阶段相关的 eng.* 快照写进 buf["chain"]（每项数值
    + "_note" 简短来源说明）。buf 由调用侧（runner）在一次链运行期间持有：
    {"site": site, "die_id": None, "chain": {}}。"""
    chain = buf.setdefault("chain", {})
    if stage == "wafer_ega":
        wp = getattr(eng, "workpieces", None)
        if wp is not None:
            with wp._lock:
                w = dict(wp.wafer)
            chain["wafer_load"] = {
                "offset_mm": [_r6(w["offset"][0]), _r6(w["offset"][1])],
                "rz_urad": _r6(w["rz"] * 1e6),
                "_note": "上片注入偏移/旋转真值（workpieces.wafer.offset/rz）"}
        ega = getattr(eng, "ega_result", None)
        if ega:
            chain["ega_fit"] = {
                "n": ega.get("n"),
                "tx_um": _r6(ega.get("tx", 0.0) * 1000),
                "ty_um": _r6(ega.get("ty", 0.0) * 1000),
                "theta_urad": _r6(ega.get("theta", 0.0) * 1e6),
                "resid_3s_um": _r6(ega.get("resid_3s", 0.0) * 1000),
                "_note": "EGA 全局对准拟合修正与 3σ 残差（eng.ega_result）"}
    elif stage == "flip_pick":
        co = getattr(eng, "coarse_offset", None)
        if co:
            buf["die_id"] = co.get("die") or buf.get("die_id")
            chain["pick_coarse"] = {
                "dx_mm": _r6(co["dx"]), "dy_mm": _r6(co["dy"]),
                "die": co.get("die"),
                "_note": "取片粗对准实测偏差（eng.coarse_offset，下相机"
                         "测 die 世界位 − 标定中心）"}
    elif stage == "flip_handoff":
        d = _die_on(eng, "head")
        if d is not None:
            buf["die_id"] = d.get("id") or buf.get("die_id")
            chain["handoff_tilt"] = {
                "rx_urad": _r6(d.get("rx", 0.0) * 1e6),
                "ry_urad": _r6(d.get("ry", 0.0) * 1e6),
                "_note": "交接后 die 在头倾斜真值（die.rx/ry，处方 "
                         "wafer_load.die_head_r*_urad 注入）"}
    elif stage == "focus_level":
        fl = getattr(eng, "fl_fit", {}) or {}
        comp = fl.get("compensate") or {}
        ver = fl.get("verify") or {}
        entry = {"_note": "调焦调平：调平残余/焦面残差（eng.fl_fit."
                          "compensate），复测判定（fl_fit.verify）"}
        if comp:
            entry["residual_urad"] = [_r6(v) for v in
                                      comp.get("residual_urad", [])]
            entry["focus_resid_um"] = [_r6(v) for v in
                                       comp.get("focus_resid_um", [])]
        if ver:
            entry["verify_residual_urad"] = [_r6(v) for v in
                                             ver.get("residual_urad", [])]
            entry["verify_judge"] = ver.get("judge")
        if len(entry) > 1:
            chain["focus_level"] = entry
    elif stage == "fine_align":
        fa = getattr(eng, "fine_align_result", None)
        if fa:
            chain["fine_align"] = {
                "algorithm": fa.get("algorithm"),
                "judge": fa.get("judge"),
                "delta": copy.deepcopy(fa.get("delta")),
                "compensations": len(fa.get("compensations") or []),
                "rounds": copy.deepcopy(fa.get("rounds") or []),
                "phys_resid": copy.deepcopy(fa.get("phys_resid")),
                "_note": "精对准：Δ 解算/补偿次数/确认逐轮残余/物理真值残差"
                         "（eng.fine_align_result；µm/µrad）"}
    elif stage == "bond":
        rec = {}
        try:
            b = (eng.recipe.get().get("bond") or {})
        except Exception:
            b = {}
        params = {}
        for k in ("force_N", "dwell_s", "head_temp_C", "wafer_temp_C"):
            if k in b:
                params[k] = b[k]
        try:
            params["head_actual_C"] = _r6(eng.heaters["PulseHeat_Up"].cur)
            params["wafer_actual_C"] = _r6(eng.heaters["Heat_Dw"].cur)
        except Exception:
            pass
        tp = getattr(eng, "bond_touch_pos", None)
        if tp is not None:
            params["touch_pos_mm"] = _r6(tp)
        ev = getattr(eng, "bond_contact_evidence", None)
        if ev:
            params["contact_verify"] = {
                "actual_contact_z_mm": _r6(ev["actual_contact_z_mm"]),
                "eff_contact_z_mm": _r6(ev["eff_contact_z_mm"]),
                "dev_um": _r6(ev["dev_um"]),
                "touch_force_N": _r6(ev["touch_force_N"]),
                "tol_um": _r6(ev["tol_um"]),
                "judge": ev["judge"]}
            # 迭代 17 WP-B：几何真值间隙（confocal 双源判定；无晶圆/die
            # 或裸引擎时为 None 不出字段）
            if ev.get("geom_gap_um") is not None:
                params["contact_verify"]["geom_gap_um"] = \
                    _r6(ev["geom_gap_um"])
        params["_note"] = "键合参数：处方力/保压/温度 + 键合时刻加热器实温" \
                          " + 寻力实际接触位（eng.bond_touch_pos）" \
                          " + 接触确认证据（eng.bond_contact_evidence，" \
                          "迭代 13）"
        buf["bond_params"] = params
        # 键合接触时刻真值分量（2026-09-07 迭代 20，eng.bond_commit，
        # bond_truth 步写入）：commit=接触时刻真值；align_resid=fa 时刻
        # 对准残差（phys_resid 快照）；descent=下降链=commit−align_resid
        # （BS_Arm_Z 地图差系统项+阿贝项）；process_shift=压合微滑移
        # （bond_mark 落笔，默认 0）。展示单位 µm/µrad。
        cm = getattr(eng, "bond_commit", None)
        if cm:
            def _blk(v, sxy, srz):
                return None if v is None else {
                    "dx_um": _r6(v["tx"] * sxy), "dy_um": _r6(v["ty"] * sxy),
                    "drz_urad": _r6(v["theta"] * srz)}
            align = cm.get("align_resid")
            shift = cm.get("process_shift")
            ent = {"commit": _blk(cm, 1000.0, 1e6),
                   "align_resid": _blk(align, 1000.0, 1e6),
                   "process_shift": _blk(shift, 1000.0, 1e6),
                   "_note": "键合接触时刻真值分量（eng.bond_commit，迭代 "
                            "20）：commit=触力停轴后 6DOF 真值（fa 同一 "
                            "fit 管线）；align_resid=fa 时刻对准残差；"
                            "descent=下降链=commit−align_resid；"
                            "process_shift=压合微滑移（默认 0）"}
            if align:
                ent["descent"] = {
                    "dx_um": _r6((cm["tx"] - align["tx"]) * 1000.0),
                    "dy_um": _r6((cm["ty"] - align["ty"]) * 1000.0),
                    "drz_urad": _r6((cm["theta"] - align["theta"]) * 1e6)}
            chain["bond_commit"] = ent
    return buf


def finalize_record(eng, site, buf):
    """bond 阶段完成后落一条完整记录。truth（2026-09-07 迭代 20 起）=
    键合接触时刻真值（eng.bond_commit，含下降链）+ 压合微滑移
    （process_shift）；无 bond_commit（旧流程/直接 force_search 等）
    回退精对准物理真值残差末轮（phys_resid，fa 时刻快照，truth_note
    注明不含下降链）。truth_components 分量分解随记录落盘（µm/µrad
    展示单位；truth 本体 mm/rad 全线约定）。
    timing（2026-09-05 迭代 15，产率分析）：runner 侧采集的
    {t_start, t_end, stages:{阶段:耗时 s}, overlap_s?}（sim_t 口径；
    t_start=首颗取片开始，t_end=键合结束——回零/EGA/上片/闲置不计入）。"""
    rec_cfg = {}
    try:
        rec_cfg = eng.recipe.get()
    except Exception:
        pass
    cell = _site_cell(rec_cfg, site) if rec_cfg else None
    site_blk = ({"row": cell[0], "col": cell[1], "x": cell[2], "y": cell[3]}
                if cell else
                {"row": site[0] if site else None,
                 "col": site[1] if site else None, "x": None, "y": None})
    die_id = buf.get("die_id")
    if die_id is None and cell is not None:
        # 兜底：按已键合 die（carrier=wafer 且 site 匹配）反查 die_id
        wp = getattr(eng, "workpieces", None)
        if wp is not None:
            with wp._lock:
                for d in wp.dies:
                    if d["carrier"] == "wafer" and d.get("site") \
                            and abs(d["site"][0] - cell[2]) < 0.001 \
                            and abs(d["site"][1] - cell[3]) < 0.001:
                        die_id = d["id"]
                        break
    fa = getattr(eng, "fine_align_result", None)
    phys = (fa or {}).get("phys_resid")
    # ---- truth：键合接触时刻真值优先（迭代 20，含下降链+压合微滑移）----
    cm = getattr(eng, "bond_commit", None)
    commit_hit = False
    if cm and cell is not None:
        cs = cm.get("site") or []
        commit_hit = len(cs) == 2 and abs(cs[0] - cell[2]) < 1e-6 \
            and abs(cs[1] - cell[3]) < 1e-6
    truth_components = None
    if commit_hit:
        shift = cm.get("process_shift") or {"tx": 0.0, "ty": 0.0,
                                            "theta": 0.0}
        truth = {"dx": _r6(cm["tx"] + shift["tx"]),
                 "dy": _r6(cm["ty"] + shift["ty"]),
                 "drz": _r6(cm["theta"] + shift["theta"])}
        truth_note = None
        align = cm.get("align_resid")
        truth_components = {
            "align_resid": ({"dx_um": _r6(align["tx"] * 1000.0),
                             "dy_um": _r6(align["ty"] * 1000.0),
                             "drz_urad": _r6(align["theta"] * 1e6)}
                            if align else None),
            "descent": ({"dx_um": _r6((cm["tx"] - align["tx"]) * 1000.0),
                         "dy_um": _r6((cm["ty"] - align["ty"]) * 1000.0),
                         "drz_urad": _r6((cm["theta"] - align["theta"])
                                         * 1e6)} if align else None),
            "process_shift": {
                "dx_um": _r6(shift["tx"] * 1000.0),
                "dy_um": _r6(shift["ty"] * 1000.0),
                "drz_urad": _r6(shift["theta"] * 1e6)},
            "commit": {"dx_um": _r6(cm["tx"] * 1000.0),
                       "dy_um": _r6(cm["ty"] * 1000.0),
                       "drz_urad": _r6(cm["theta"] * 1e6)}}
    elif phys:
        truth = {"dx": _r6(phys["tx_um"] / 1000.0),
                 "dy": _r6(phys["ty_um"] / 1000.0),
                 "drz": _r6(phys["theta_urad"] / 1e6)}
        truth_note = "回退口径：无键合接触时刻真值（bond_commit 缺/site " \
                     "不符），truth=精对准 fa 时刻物理残差（不含键合头下" \
                     "降链，迭代 20 前口径）"
    else:
        truth = None
        truth_note = "无精对准物理残差（fine_align_result.phys_resid 缺）" \
                     "，键合偏移真值不可得"
    with eng.lock:
        t_bond = round(eng.sim_time, 3)
    # bond_offset 快照（2026-09-09 迭代 23，统计分代）：键合时刻机械常数
    # fine_align.bond_offset_*（µm/µrad）——补偿代划分依据；面板结果分析
    # "仅统计当前补偿代"按本字段与当前机械常数比对过滤（跨代记录灰显、
    # 不进统计，防止补偿前记录污染补偿量建议）
    bo = None
    try:
        fa_mc = eng.constants.get().get("fine_align", {})
        bo = {"x_um": _r6(float(fa_mc.get("bond_offset_x_um", 0.0))),
              "y_um": _r6(float(fa_mc.get("bond_offset_y_um", 0.0))),
              "rz_urad": _r6(float(fa_mc.get("bond_offset_rz_urad", 0.0)))}
    except Exception:
        pass
    return {"die_id": die_id,
            "site": site_blk,
            "t_bond": t_bond,
            "truth": truth,
            "truth_note": truth_note,
            "truth_components": truth_components,
            "measured": None,
            "bond_offset": bo,
            "timing": copy.deepcopy(buf.get("timing")),
            "chain": copy.deepcopy(buf.get("chain") or {}),
            "bond_params": copy.deepcopy(buf.get("bond_params") or {})}


def _col_stats(vals):
    """一组 |误差| 值的展示统计：mean/max/3σ（µm/µrad 已换算）。"""
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return {"mean": _r6(mean), "max": _r6(max(vals)),
            "sigma3": _r6(3.0 * math.sqrt(var))}


def _err_stats(items):
    """truth/measured 块的统计：|dx|/|dy|/|dxy| µm 与 |drz| µrad 的
    mean/max/3σ；另补带符号统计（2026-09-05 迭代 15）：pv2=(max−min)/2
    与 mean（系统偏差，补偿量建议数据源；不补偿随机散布）。
    items=非空误差 dict 列表。"""
    def _signed(key, scale):
        vals = [t[key] * scale for t in items]
        return {"pv2": _r6((max(vals) - min(vals)) / 2.0),
                "mean": _r6(sum(vals) / len(vals))}
    return {
        "abs_dx_um": _col_stats([abs(t["dx"]) * 1000.0 for t in items]),
        "abs_dy_um": _col_stats([abs(t["dy"]) * 1000.0 for t in items]),
        "abs_dxy_um": _col_stats([(t["dx"] ** 2 + t["dy"] ** 2) ** 0.5
                                  * 1000.0 for t in items]),
        "abs_drz_urad": _col_stats([abs(t["drz"]) * 1e6 for t in items]),
        "dx_um": _signed("dx", 1000.0),
        "dy_um": _signed("dy", 1000.0),
        "drz_urad": _signed("drz", 1e6),
    }


def _stats(records):
    """统计摘要：truth 与 measured 两列各自的 |dx|/|dy|/|dxy|/|drz|
    mean/max/3σ（展示用 µm/µrad；无数据列为 None）。"""
    ts = [r["truth"] for r in records if r.get("truth")]
    ms = [r["measured"] for r in records if r.get("measured")]
    out = {"count": len(records), "with_truth": len(ts),
           "measured": len(ms),
           "truth": _err_stats(ts) if ts else None,
           "measured_stats": _err_stats(ms) if ms else None}
    return out


def _live_block(eng):
    """进行中实时产率块（迭代 17 WP-C）：从 ChainRunner/FullWaferRunner
    status + eng.sim_time 组装；无运行返回 None。全片优先于单片（两者
    互斥，正常不会同时 active）。"""
    try:
        with eng.lock:
            sim_t = eng.sim_time
        fw = getattr(eng, "full_wafer", None)
        ch = getattr(eng, "chain", None)
        fwst = fw.status() if fw is not None else None
        chst = ch.status() if ch is not None else None
    except Exception:
        return None
    if fwst and fwst.get("active"):
        cur = fwst.get("current") or {}
        res = fwst.get("results") or []
        run_ok = sum(1 for r2 in res if r2.get("ok"))
        run_fail = len(res) - run_ok
        pipelined = False
        try:
            pipelined = bool((eng.recipe.get().get("full_wafer") or {})
                             .get("pipelined"))
        except Exception:
            pass
        cur_t0 = cur.get("t_start")
        cur_elapsed = (round(sim_t - cur_t0, 3)
                       if cur_t0 is not None and sim_t >= cur_t0 else None)
        t_first = fwst.get("run_t_first")
        est = None
        if run_ok and t_first is not None and sim_t > t_first:
            est = _r6(3600.0 * run_ok / (sim_t - t_first))
        return {"active": True,
                "kind": "pipelined" if pipelined else "full",
                "current": {"die": cur.get("die"), "site": cur.get("site"),
                            "stage": cur.get("stage")},
                "progress": list(fwst.get("progress") or [0, 0]),
                "run_ok": run_ok, "run_fail": run_fail,
                "cur_elapsed_s": cur_elapsed,
                "est_uph": est}
    if chst and chst.get("active"):
        t0 = chst.get("t_start")
        cur_elapsed = (round(sim_t - t0, 3)
                       if t0 is not None and sim_t >= t0 else None)
        return {"active": True, "kind": "single",
                "current": {"die": None, "site": chst.get("site"),
                            "stage": chst.get("current")},
                "progress": None,
                "run_ok": 0, "run_fail": 0,
                "cur_elapsed_s": cur_elapsed,
                # 单片=1 颗，完成前 done=0 → est_uph 恒 None
                "est_uph": None}
    return None


class BondResults(object):
    """键合结果注册表：线程安全追加/查询 + 原子落盘 + 启动加载。"""

    VERSION = 1

    # 产率统计五阶段（迭代 15；与 full_wafer_runner.STAGES 同名）
    TP_STAGES = ("flip_pick", "flip_handoff", "focus_level", "fine_align",
                 "bond")

    def __init__(self, path):
        self.path = path
        self._lock = threading.RLock()
        self._records = []
        self._tp = self._tp_blank()
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
                recs = data.get("records") if isinstance(data, dict) else None
                if isinstance(recs, list):
                    self._records = recs
                tp = data.get("throughput") if isinstance(data, dict) else None
                if isinstance(tp, dict):
                    for k in ("runs", "ok", "fail", "net_time_s",
                              "overlap_s"):
                        if isinstance(tp.get(k), (int, float)):
                            self._tp[k] = float(tp[k]) \
                                if "time" in k or k == "overlap_s" else int(tp[k])
                    for k in ("stage_sums", "stage_counts"):
                        if isinstance(tp.get(k), dict):
                            self._tp[k] = {kk: float(vv) for kk, vv in
                                           tp[k].items()}
            except Exception as e:
                # 损坏文件不阻断启动：备份留证后从空开始
                print("[boot] bond_results.json 加载失败（%s），从空开始"
                      % e, flush=True)
                try:
                    os.replace(path, path + ".corrupt")
                except OSError:
                    pass

    @staticmethod
    def _tp_blank():
        return {"runs": 0, "ok": 0, "fail": 0, "net_time_s": 0.0,
                "overlap_s": 0.0, "stage_sums": {}, "stage_counts": {}}

    def _save_locked(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"version": self.VERSION, "records": self._records,
                       "throughput": self._tp},
                      f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.path)

    def add(self, record):
        with self._lock:
            self._records.append(record)
            self._save_locked()

    def clear(self):
        with self._lock:
            self._records = []
            self._tp = self._tp_blank()
            self._save_locked()

    # ---------------- 产率统计（2026-09-05 迭代 15，UPH 口径见下） ----------------
    def throughput_add(self, ok, fail, net_s, stage_durs=None, overlap_s=0.0,
                       stage_n=1):
        """一次运行（单片链/全片批次）的产率记账：
        ok/fail=成功/失败颗数；net_s=净流水时间（首颗取片开始→末颗键合结
        束，回零/EGA/上片/闲置不计入；失败运行传 0 不进 UPH 分母）；
        stage_durs=本次运行五阶段耗时汇总 {stage: s}（仅成功颗），
        stage_n=stage_durs 覆盖的颗数（单片链=1，全片=成功颗数——阶段
        均耗时按颗平均）；overlap_s=产率模式预取重叠时间（pipelined，
        串行为 0）。"""
        with self._lock:
            tp = self._tp
            tp["runs"] += 1
            tp["ok"] += int(ok)
            tp["fail"] += int(fail)
            tp["net_time_s"] += max(0.0, float(net_s))
            tp["overlap_s"] += max(0.0, float(overlap_s))
            n = max(0, int(stage_n))
            for k, v in (stage_durs or {}).items():
                if k not in self.TP_STAGES:
                    continue
                tp["stage_sums"][k] = tp["stage_sums"].get(k, 0.0) + float(v)
                tp["stage_counts"][k] = tp["stage_counts"].get(k, 0) + n
            self._save_locked()

    def throughput_public(self, eng=None):
        """GET /api/bond/throughput：累计 UPH=3600×成功颗数/净流水时间；
        最近 N 颗 UPH=3600×(N−1)/(末次−首次 t_end)（按记录 timing.t_end
        排序取最近 10 条，流水线友好）；各阶段均耗时；成功/失败计数；
        总生产时间；overlap_s 单列（参照参考项目 backend overlap_ms）。
        eng 非空时追加 live 块（2026-09-06 迭代 17 WP-C 产率实时化）：
        无运行=None；运行中={active, kind("single"/"full"/"pipelined"),
        current:{die,site,stage}, run_ok, run_fail, cur_elapsed_s（当前颗
        已耗时 sim 秒）, est_uph（本轮已完成实测 UPH=3600×done/已耗时，
        done=0 为 None）}。runner 状态走 duck-typing（不 import
        chain_runner/full_wafer_runner，避免循环依赖）。"""
        with self._lock:
            tp = copy.deepcopy(self._tp)
            timed = [r for r in self._records
                     if (r.get("timing") or {}).get("t_end") is not None]
        net = tp["net_time_s"]
        uph_total = (3600.0 * tp["ok"] / net) if net > 0 and tp["ok"] else None
        stage_avg = {}
        for k in self.TP_STAGES:
            cnt = tp["stage_counts"].get(k, 0)
            stage_avg[k] = (_r6(tp["stage_sums"][k] / cnt) if cnt else None)
        timed.sort(key=lambda r: r["timing"]["t_end"])
        recent = timed[-10:]
        uph_recent = None
        if len(recent) >= 2:
            dt = recent[-1]["timing"]["t_end"] - recent[0]["timing"]["t_end"]
            if dt > 0:
                uph_recent = 3600.0 * (len(recent) - 1) / dt
        out = {"ok": True,
               "runs": tp["runs"],
               "ok_count": tp["ok"], "fail_count": tp["fail"],
               "net_time_s": _r6(net),
               "uph_total": _r6(uph_total) if uph_total else None,
               "uph_recent": _r6(uph_recent) if uph_recent else None,
               "recent_n": len(recent),
               "stage_avg_s": stage_avg,
               "overlap_s": _r6(tp["overlap_s"]),
               "live": _live_block(eng) if eng is not None else None,
               "_note": "UPH=成功键合颗数/生产时间（首颗取片开始→末颗键合"
                        "结束的净流水时间；回零/EGA/上片/闲置不计入），"
                        "sim_t 口径；live=进行中实时块（无运行为 null，"
                        "迭代 17）"}
        return out

    def list_public(self):
        """GET /api/bond/results：全量记录 + 统计摘要。"""
        with self._lock:
            recs = copy.deepcopy(self._records)
        return {"ok": True, "count": len(recs),
                "stats": _stats(recs), "results": recs}

    def get_by_die(self, die_id):
        """GET /api/bond/results?die=<id>：单条（含完整 chain）；未命中
        返回 None。"""
        with self._lock:
            for r in self._records:
                if r.get("die_id") == die_id:
                    return copy.deepcopy(r)
        return None

    def update_measured(self, die_id, dx, dy, drz, t, site=None, warn=None):
        """键合后精度检测回填（迭代 11 第二棒 post_inspect）：按 (die_id,
        site) 双键找**最新一条无 measured** 的记录写
        measured={dx,dy,drz,t}（mm/rad 全线约定）并落盘——die_id 是蓝膜
        网格号跨运行重复（复跑同 die 时旧的首条匹配会把测量挂到上一轮
        记录，2026-09-09 迭代 23 修正）；site=[row,col] 可 None（不约束）。
        warn=测量侧异常留证（如 PI drz 物理门限判伪差，迭代 23）写入
        measured_warning 字段。双键无未测记录时回退旧逻辑（die_id 首条，
        允许覆盖）并留 measured_warning。未命中返回 False（不新建——无
        truth 的孤儿测量无意义）。"""
        blk = {"dx": _r6(dx), "dy": _r6(dy),
               "drz": _r6(drz), "t": round(float(t), 3)}
        with self._lock:
            def _site_eq(r):
                if site is None:
                    return True
                rs = r.get("site") or {}
                return rs.get("row") == site[0] and rs.get("col") == site[1]
            for r in reversed(self._records):
                if r.get("die_id") == die_id and _site_eq(r) \
                        and not r.get("measured"):
                    r["measured"] = blk
                    if warn:
                        r["measured_warning"] = warn
                    self._save_locked()
                    return True
            # 回退（旧逻辑）：die_id 首条匹配 + warning 留证（可能挂错运行）
            for r in self._records:
                if r.get("die_id") == die_id:
                    r["measured"] = blk
                    r["measured_warning"] = warn or (
                        "双键（die_id+site=%s）无未测记录，回退 die_id 首条"
                        "匹配——本测量可能挂到历史运行记录" % (site,))
                    self._save_locked()
                    return True
        return False
