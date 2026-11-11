# -*- coding: utf-8 -*-
"""workpieces.py — 工件物料层：晶圆 + 蓝膜 + die 阵列 + 交接判定。

不做工艺流程编排，只维护工件实体状态与"真空 + 示教位"驱动的换载体判定：

- 晶圆：{present, offset:[x,y]}，上片/下片由 /api/workpiece/wafer 驱动。
  下片时已键合到 wafer 上的 die 随片移除（离机），重复下片报错。
  晶圆直径/厚度/标记版图为处方参数（recipe.wafer，见 recipe.py）。
- 蓝膜：{present}，上/下机由 /api/workpiece/film 驱动（2026-08-21 升级为
  正式工件；此前 film 恒在）。上机**幂等**：置在位 + film 上的 die 按当前
  处方重铺（已在 flip/head/wafer 的 die 不动）；下机时 film 上剩余 die
  随机移除（已离膜的留下），重复下机报错。film 不在位 → 无 film die、
  含 film 的交接规则不满足、视觉侧（coarse）无蓝膜面/无标记。
- die：蓝膜台铁环阵列。**挂接处方后行列/间距/原点由 recipe.film.dies 决定**
  （未挂处方时回退 transfers.json "grid"，行为同旧版）。每颗
  {id, film_pos:[x,y]（蓝膜台局部）, carrier: film|flip|head|wafer,
  site:[x,y]|None（新载体上的局部位，占位）, rz, rx/ry（在头倾斜真值 rad，
  默认 0，交接 to=head 时按处方 wafer_load.die_head_rx/ry_urad 注入）}。
  改处方不动当前物料；POST /api/recipe/apply → apply_recipe() 才重建阵列
  （die 重铺满、晶圆卸载、film 置在位、载体清空）。
- 交接规则（transfers.json "rules"，可 GET/PUT /api/transfers 在线改）：
  {name, from, to, vacuum_on:[点位路径...], vacuum_off:[...],
   pose:{轴名:[目标值,容差]}, wafer_present?:bool, die_site?:[x,y,tol],
   site?:[x,y], _note?}
  条件 = 真空点位状态满足 + 相关轴当前值在示教位容差内（+ 晶圆在位要求
  + 涉及 film 侧时 film 须在位）。
  **上升沿触发**：条件从"不满足"变"满足"时交接一颗 die（每 tick 全局最多一次，
  防止同 tick 链式跳变）；条件持续满足不重复交接（一次逼近取一颗，符合真机）。
- 交接日志：内存 deque（200 条），/api/viz 带最近 20 条。

每 tick 检查成本 O(规则数 × (真空点数+姿态轴数))，可忽略。
"""
import hashlib
import json
import os
import threading
from collections import deque

CARRIERS = ("film", "flip", "head", "wafer")


def site_bonded(wp, cx, cy, tol=0.001):
    """键合位（设计坐标 cx,cy）是否已有 die 落位：carrier=wafer 且 site
    在容差内匹配（默认 1µm——bond_mark 写入设计坐标，浮点等值匹配脆弱，
    统一走本函数；process_flows 的 bond/focus_level/fine_align 守卫与
    chain_runner 幂等跳过共用）。"""
    with wp._lock:
        return any(
            d["carrier"] == "wafer"
            and d["site"] is not None
            and abs(d["site"][0] - cx) <= tol
            and abs(d["site"][1] - cy) <= tol
            for d in wp.dies)


class Workpieces(object):
    # 载体容量（物理硬约束）：单吸盘载体同时只能持有 1 颗 die；film/wafer
    # 无限。交接规则触发前查容量，已满不触发（防重复拾取/叠片）。
    CAPACITY = {"flip": 1, "head": 1}

    def __init__(self, transfers_path, recipe=None):
        self._lock = threading.RLock()
        self.transfers_path = transfers_path
        self.recipe = recipe          # Recipe 实例（main.py 注入）；None=旧行为
        self.grid_cfg = {"rows": 10, "cols": 10, "pitch": 5.0}
        self.rules = []
        self._load_transfers()
        self.wafer = {"present": False, "offset": [0.0, 0.0], "rz": 0.0,
                      "z": 0.0, "rx": 0.0, "ry": 0.0}
        self.film = {"present": True}     # 蓝膜默认在位（开机即有满阵列 die）
        self.pick_offset = [0.0, 0.0]     # 取片偏移注入（膜上片误差传递演示）
        self.dies = []
        self.log = deque(maxlen=200)
        self._layout_sig = None       # 铺膜签名（_build_grid 时记，真·陈膜
        self._laid_count = 0          # 判据）；_laid_count=铺膜时格点数
        self._build_grid()

    # ---------------- 阵列 ----------------
    def _grid_source(self):
        """die 阵列几何来源：处方优先（recipe.film.dies 派生格点，
        auto=芯片大小+间隔按膜面圆自动算、已裁掉圆外空位），否则
        transfers grid（旧行为）。返回 {rows, cols, pitch_x, pitch_y,
        origin_x, origin_y, cells}（cells=[(r, c, x, y)] 全量格点，
        film 局部坐标、膜面圆心为原点）。"""
        if self.recipe is not None:
            from recipe import derive_dies, dies_cells
            rec = self.recipe.get()
            g = derive_dies(rec)
            return {"rows": g["rows"], "cols": g["cols"],
                    "pitch_x": g["pitch_x_mm"], "pitch_y": g["pitch_y_mm"],
                    "origin_x": g["effective_origin"][0],
                    "origin_y": g["effective_origin"][1],
                    "cells": dies_cells(rec)}
        g = self.grid_cfg
        rows, cols = int(g["rows"]), int(g["cols"])
        pitch = float(g["pitch"])
        return {"rows": rows, "cols": cols,
                "pitch_x": pitch, "pitch_y": pitch,
                "origin_x": 0.0, "origin_y": 0.0,
                "cells": [(r, c, round(c * pitch, 4), round(r * pitch, 4))
                          for r in range(rows) for c in range(cols)]}

    @staticmethod
    def _layout_signature(g):
        """铺膜签名（2026-09-09 迭代 23）：铺膜时刻派生格点块（行列/间距/
        原点/全量格点）的规范化摘要——处方 film.dies 任何改动（含尺寸/
        间隔/模式切换）都会改变格点集 → 签名变，即"膜是旧处方铺的"的真·
        陈膜判据；键合消耗（die 离膜）不动签名。md5 短摘要只用于相等比较。"""
        s = json.dumps({"rows": g["rows"], "cols": g["cols"],
                        "pitch_x": g["pitch_x"], "pitch_y": g["pitch_y"],
                        "origin_x": g["origin_x"], "origin_y": g["origin_y"],
                        "cells": g["cells"]}, sort_keys=True)
        return hashlib.md5(s.encode("utf-8")).hexdigest()[:16]

    def _build_grid(self):
        g = self._grid_source()
        self.dies = [{
            "id": "R%02dC%02d" % (r, c),
            "film_pos": [x, y],
            "carrier": "film", "site": None, "rz": 0.0,
            # die 在头倾斜真值（rad，默认 0；交接 to=head 时按处方
            # wafer_load.die_head_rx/ry_urad 注入，共焦芯片面高函数消费）
            "rx": 0.0, "ry": 0.0,
        } for r, c, x, y in g["cells"]]
        self._layout_sig = self._layout_signature(g)
        self._laid_count = len(g["cells"])

    # ---------------- 规则表（transfers.json） ----------------
    def _load_transfers(self):
        with open(self.transfers_path, encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data.get("grid"), dict):
            self.grid_cfg.update(data["grid"])
        self.rules = [self._validate_rule(r) for r in data.get("rules", [])]
        for r in self.rules:
            r["_met_prev"] = False          # 上升沿触发用的内部状态

    @staticmethod
    def _validate_rule(r):
        for k in ("name", "from", "to"):
            if k not in r:
                raise ValueError("transfer rule 缺字段 %r: %r" % (k, r))
        if r["from"] not in CARRIERS or r["to"] not in CARRIERS:
            raise ValueError("transfer rule carrier 非法: %r" % r["name"])
        r.setdefault("vacuum_on", [])
        r.setdefault("vacuum_off", [])
        r.setdefault("pose", {})
        for axis, spec in r["pose"].items():
            if not (isinstance(spec, (list, tuple)) and len(spec) == 2):
                raise ValueError("pose 须为 [目标值, 容差]: %r" % r["name"])
        return r

    def save_transfers(self, rules, grid=None):
        """PUT /api/transfers：整体替换规则表。
        挂接处方后 die 阵列几何由 recipe.film.dies 管理，grid 参数拒绝。"""
        if grid and self.recipe is not None:
            raise ValueError("die 阵列几何由处方管理（PUT /api/recipe 改参数 + "
                             "POST /api/recipe/apply 生效），transfers grid 已停用")
        new_rules = [self._validate_rule(dict(r)) for r in (rules or [])]
        with self._lock:
            self.rules = new_rules
            for r in self.rules:
                r["_met_prev"] = False
            if grid:
                self.grid_cfg.update(grid)
                self._build_grid()
            data = {"_comment":
                    "交接规则表（示教位/真空点位为占位待标定，见各规则 _note）",
                    "grid": self.grid_cfg,
                    "rules": [{k: v for k, v in r.items()
                               if not k.startswith("_") or k == "_note"}
                              for r in self.rules]}
            tmp = self.transfers_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
                f.write("\n")
            os.replace(tmp, self.transfers_path)
        return len(self.rules)

    def rules_public(self):
        with self._lock:
            g = self._grid_source()
            # grid 保持旧键（rows/cols/pitch）兼容，另附处方完整 dies 块
            grid = {"rows": g["rows"], "cols": g["cols"], "pitch": g["pitch_x"],
                    "pitch_x_mm": g["pitch_x"], "pitch_y_mm": g["pitch_y"],
                    "origin_x_mm": g["origin_x"], "origin_y_mm": g["origin_y"],
                    "source": "recipe" if self.recipe is not None else "transfers"}
            return {"ok": True, "grid": grid,
                    "rules": [{k: v for k, v in r.items()
                               if not k.startswith("_") or k == "_note"}
                              for r in self.rules]}

    # ---------------- 处方应用 ----------------
    def apply_recipe(self):
        """POST /api/recipe/apply：按当前内存处方重建布局并重置物料状态
        （die 按新阵列重铺满、晶圆卸载、film 置在位、载体清空、交接日志清、
        触发沿复位）。"""
        if self.recipe is None:
            raise RuntimeError("recipe not attached")
        with self._lock:
            self.wafer = {"present": False, "offset": [0.0, 0.0], "rz": 0.0,
                      "z": 0.0, "rx": 0.0, "ry": 0.0}
            self.film["present"] = True
            self._build_grid()
            self.log.clear()
            for r in self.rules:
                r["_met_prev"] = False
            return self.layout_public()

    def layout_public(self):
        """当前布局摘要（apply 响应 / 面板摘要区用）。
        dies_on_film/dies_expected/film_layout_mismatch（2026-09-05 迭代
        15）：膜上 die 按上膜时刻处方铺，处方后改不重铺——两者不符时面板
        提示"请重新上膜"（全片 24 颗问题根因）。
        2026-09-09 迭代 23：mismatch 判据由"存活 on-film 计数≠处方派生
        总数"（键合消耗 N 颗即误报，147/149 问题）改为**铺膜签名不符**
        （真·陈膜）；dies_consumed=铺膜颗数−当前膜上存活（键合/取片消耗，
        信息级展示，不是布局不符）。"""
        with self._lock:
            g = self._grid_source()
            on_film = sum(1 for d in self.dies if d["carrier"] == "film")
            consumed = max(0, self._laid_count - on_film)
            out = {"ok": True, "dies_total": len(self.dies),
                   "rows": g["rows"], "cols": g["cols"],
                   "pitch_x_mm": g["pitch_x"], "pitch_y_mm": g["pitch_y"],
                   "origin_x_mm": g["origin_x"], "origin_y_mm": g["origin_y"],
                   "wafer": dict(self.wafer), "film": dict(self.film),
                   "dies_on_film": on_film,
                   "dies_expected": len(g["cells"]),
                   "dies_laid": self._laid_count,
                   "dies_consumed": consumed,
                   "film_layout_mismatch": bool(self.film["present"])
                   and self._layout_signature(g) != self._layout_sig}
            if self.recipe is not None:
                rec = self.recipe.get()
                out["wafer_diameter_mm"] = rec["wafer"]["diameter_mm"]
                out["wafer_thickness_mm"] = rec["wafer"]["thickness_mm"]
            return out

    # ---------------- 晶圆上下片 ----------------
    def load_wafer(self, offset=None, rz=0.0, z=0.0, rx=0.0, ry=0.0):
        """上片：offset=[x,y] 晶圆心相对吸盘心平移（mm），rz=上片旋转（rad，
        全线统一右手系 +z 逆时针）——上片误差注入通道（EGA 全局对准要找回
        的量，见 docs/晶圆上片与全局对准_EGA_V1.md）。z=晶圆面相对吸盘面
        面高偏移（mm）、rx/ry=晶圆面倾斜（rad，小角）——垂向上片真值注入
        （2026-08-27 调焦调平新增，共焦晶圆面高函数消费，默认 0）。"""
        with self._lock:
            if self.wafer["present"]:
                raise ValueError("wafer already loaded")
            off = offset if offset is not None else [0.0, 0.0]
            if not (isinstance(off, (list, tuple)) and len(off) == 2):
                raise ValueError("offset 须为 [x, y]")
            if not isinstance(rz, (int, float)) or isinstance(rz, bool) \
                    or abs(float(rz)) > 0.01:
                raise ValueError("rz 须为 |≤0.01| 的数值 rad")
            for name, v, lim in (("z", z, 2.0), ("rx", rx, 0.01),
                                 ("ry", ry, 0.01)):
                if not isinstance(v, (int, float)) or isinstance(v, bool) \
                        or abs(float(v)) > lim:
                    raise ValueError("%s 须为 |≤%s| 的数值（mm/rad）"
                                     % (name, lim, v))
            self.wafer = {"present": True,
                          "offset": [float(off[0]), float(off[1])],
                          "rz": float(rz),
                          "z": float(z), "rx": float(rx), "ry": float(ry)}
            return dict(self.wafer)

    def unload_wafer(self):
        """下片：已键合到 wafer 的 die 随片移除（离机）；无片时报错。"""
        with self._lock:
            if not self.wafer["present"]:
                raise ValueError("no wafer loaded")
            removed = sum(1 for d in self.dies if d["carrier"] == "wafer")
            self.dies = [d for d in self.dies if d["carrier"] != "wafer"]
            self.wafer = {"present": False, "offset": [0.0, 0.0], "rz": 0.0,
                      "z": 0.0, "rx": 0.0, "ry": 0.0}
            return {"removed_dies": removed}

    # ---------------- 蓝膜上下机 ----------------
    def load_film(self):
        """蓝膜上机（**幂等**）：置在位 + film 上的 die 按当前处方重铺；
        已在 flip/head/wafer 的 die（已离膜）不动。重复上机 = 重铺。"""
        with self._lock:
            kept = [d for d in self.dies if d["carrier"] != "film"]
            self._build_grid()              # self.dies = 满阵列 film die
            self.dies.extend(kept)
            self.film["present"] = True
            return dict(self.film)

    def unload_film(self):
        """蓝膜下机：film 与其上剩余 die 一起离机；已在 flip/head/wafer 的
        die 留下（已离膜）。不在位时报错。"""
        with self._lock:
            if not self.film["present"]:
                raise ValueError("no film loaded")
            removed = sum(1 for d in self.dies if d["carrier"] == "film")
            self.dies = [d for d in self.dies if d["carrier"] != "film"]
            self.film["present"] = False
            return {"removed_dies": removed}

    def reset(self):
        """工件复位：晶圆离场，蓝膜回在位，die 全回蓝膜初始阵列。"""
        with self._lock:
            self.wafer = {"present": False, "offset": [0.0, 0.0], "rz": 0.0,
                      "z": 0.0, "rx": 0.0, "ry": 0.0}
            self.film = {"present": True}
            self.pick_offset = [0.0, 0.0]
            self._build_grid()
            self.log.clear()
            for r in self.rules:
                r["_met_prev"] = False

    # ---------------- 交接判定（每 tick，engine 持锁调用） ----------------
    def check_transfers(self, eng, pts, sim_time):
        with self._lock:
            for rule in self.rules:
                met = self._conditions_met(rule, eng, pts)
                fire = met and not rule["_met_prev"]
                rule["_met_prev"] = met
                if not fire:
                    continue
                # 载体容量硬约束（物理：单容量载体吸盘上不能叠第二颗）——
                # flip/head 容量 1，已满则本规则不触发（防重复拾取/叠片，
                # 2026-08-25 用户"已吸取一颗为何还能吸取下一颗"修正）
                cap = self.CAPACITY.get(rule["to"])
                if cap is not None:
                    n = sum(1 for d in self.dies
                            if d["carrier"] == rule["to"])
                    if n >= cap:
                        continue
                die = self._select_die(rule, eng)
                if die is None:
                    continue
                frm = die["carrier"]
                die["carrier"] = rule["to"]
                site = [float(v) for v in (rule.get("site") or [0.0, 0.0])]
                if rule["to"] == "flip" and (self.pick_offset[0]
                                             or self.pick_offset[1]):
                    # 取片偏移注入（膜上片误差的传递结果）：蓝膜台按名义
                    # 几何送片，die 实际落吸盘真空面的位置偏离中心——
                    # 后续由粗对准拍照测出（coarse_offset 流程步）
                    site = [site[0] + self.pick_offset[0],
                            site[1] + self.pick_offset[1]]
                die["site"] = site
                if rule["to"] == "head" and self.recipe is not None:
                    # 芯片在头倾斜注入（测校页 wafer_load.die_head_rx/ry_urad）：
                    # 交接完成时写入 die 载体真值，共焦芯片面高函数消费
                    wl = self.recipe.get().get("wafer_load", {})
                    die["rx"] = float(wl.get("die_head_rx_urad", 0.0)) / 1e6
                    die["ry"] = float(wl.get("die_head_ry_urad", 0.0)) / 1e6
                self.log.append({"t": round(sim_time, 3), "rule": rule["name"],
                                 "die": die["id"], "from": frm, "to": rule["to"]})
                return rule["name"]            # 每 tick 最多交接一颗
        return None

    def set_pick_offset(self, x_mm, y_mm):
        """注入取片偏移（膜上片误差演示/测试）：之后 film→flip 交接的 die
        在吸盘局部系带 (x_mm, y_mm) 偏差；workpiece/reset 清零。"""
        for v in (x_mm, y_mm):
            if not isinstance(v, (int, float)) or isinstance(v, bool) \
                    or abs(float(v)) > 5.0:
                raise ValueError("pick_offset 分量须为 |≤5| 的数值 mm")
        with self._lock:
            self.pick_offset = [float(x_mm), float(y_mm)]
            return list(self.pick_offset)

    def _conditions_met(self, rule, eng, pts):
        if not self.film["present"] and "film" in (rule["from"], rule["to"]):
            return False            # 蓝膜不在位：含 film 侧的交接规则不满足
        if rule.get("wafer_present") is not None and \
                bool(rule["wafer_present"]) != self.wafer["present"]:
            return False
        for p in rule["vacuum_on"]:
            if not bool(pts.get(p)):
                return False
        for p in rule["vacuum_off"]:
            if bool(pts.get(p)):
                return False
        for axis, (target, tol) in rule["pose"].items():
            ax = eng.axes.get(axis)
            if ax is None or abs(ax.pos - float(target)) > float(tol):
                return False
        return True

    def _select_die(self, rule, eng=None):
        """选位于该位置的 die：rule.die_site=[x,y,tol]（载体局部坐标）时取
        窗口内最近的一颗；from=film 且无 die_site 时按**物理位置**选——
        拾取头（flip 模块工具点 xy）正下方的那颗 film die（蓝膜台会把
        目标 die 送到取片位，cand[0] 的旧行为会抓错片，2026-08-24 翻转
        取片流程暴露后修正）；其余取该载体上 id 最小的一颗。"""
        cand = [d for d in self.dies if d["carrier"] == rule["from"]]
        if not cand:
            return None
        site = rule.get("die_site")
        if site:
            sx, sy, tol = float(site[0]), float(site[1]), float(site[2])
            def dist2(d):
                p = d["film_pos"] if rule["from"] == "film" else (d["site"] or [0, 0])
                return (p[0] - sx) ** 2 + (p[1] - sy) ** 2
            in_win = [d for d in cand if dist2(d) <= tol * tol]
            if not in_win:
                return None
            return min(in_win, key=dist2)
        if rule["from"] == "film" and eng is not None \
                and eng.kinematics is not None:
            mods = eng.kinematics.config_public()["modules"]
            flip_xy = mods["flip"]["tool_point"]       # 取片位（规则已约束
            ftool = mods["film_stage"]["tool_point"]   # FA_X∈0±3 → 就近）
            with eng.lock:
                esx = eng.axes["ES_X"].pos
                esy = eng.axes["ES_Y"].pos

            def dist2(d):
                wx = ftool[0] + esx + d["film_pos"][0]
                wy = ftool[1] + esy + d["film_pos"][1]
                return (wx - flip_xy[0]) ** 2 + (wy - flip_xy[1]) ** 2
            return min(cand, key=dist2)
        return cand[0]

    # ---------------- 聚合视图 ----------------
    def viz(self):
        with self._lock:
            return {
                "wafer": dict(self.wafer),
                "pick_offset": list(self.pick_offset),
                "film": dict(self.film),
                "dies": [{"id": d["id"], "carrier": d["carrier"],
                          # 载体局部坐标（前端自行挂接）
                          "pos": list(d["film_pos"] if d["carrier"] == "film"
                                      else (d["site"] or [0.0, 0.0])),
                          "rz": d["rz"]} for d in self.dies],
                "transfer_log": list(self.log)[-20:],
            }
