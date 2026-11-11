# -*- coding: utf-8 -*-
"""tuning.py — 下位机调参存储：调试好即交付、不计入机械常数的参数。

与 machine_constants.py（设备本体标定量）互补：本模块管"下位机（PLC/驱动
器侧）调好后随设备交付"的参数——三回路 PID 整定值与键合被控对象硬件
参数（弹簧刚度/加热器速率限与量程上限/环境温度）。纯标准库。

- 组：`pid`（force/headtemp/wafertemp 三回路 TwinCAT 系列型 PID 参数 +
  cycle_s，2026-08-31 迭代 9 自 machine_constants.json pid 段迁入）与
  `bond_hw`（热压键合被控对象硬件参数，同批自 bond 段 6 键迁入）。
- 加载：文件不存在 → 以默认值建文件；LoadError（ValueError 子类）抛出即
  拒绝拉起（main.py 启动路径）。缺失字段以默认值补齐（前向兼容）。
- 更新：update() 部分字段合并 → 全量校验 → 通过才替换 + 原子写盘
  （tmp+os.replace，同 machine_constants.py 惯例）；校验失败抛
  ValueError，errors[] 逐条定位字段路径。
- 迁移：migrate_legacy() 把旧 machine_constants.json 的 pid 段与 bond 段
  6 个硬件键搬到 tuning.json、读数噪声 2 键（temp_noise_C/force_noise_N）
  搬到 recipe.json sim_sensors 组，并从 machine_constants.json 剔除
  （一次性，main.py 启动时调用，写 [boot] 日志）。
"""
import json
import math
import os
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))
TUNING_JSON = os.path.join(ROOT, "tuning.json")

GROUPS = ("pid", "bond_hw")

# pid 组（三回路 TwinCAT 系列型 PID 参数；engine tick 内按 cycle_s 周期
# 调用）。tn_s=0 → 纯 PD（积分通道关闭）。字段 -> (最小, 最大)。
PID_FIELDS = {
    "force_kp": ("num", -1e6, 1e6),     # 键合力回路（输出=压入量 mm）
    "force_tn_s": ("num", -1e-9, 1e5),
    "force_tv_s": ("num", -1e-9, 1e5),
    "force_td_s": ("num", -1e-9, 1e5),
    "headtemp_kp": ("num", -1e6, 1e6),  # 脉冲加热温度回路（输出=功率 %）
    "headtemp_tn_s": ("num", -1e-9, 1e5),
    "headtemp_tv_s": ("num", -1e-9, 1e5),
    "headtemp_td_s": ("num", -1e-9, 1e5),
    "wafertemp_kp": ("num", -1e6, 1e6),  # 硅片台温度回路（输出=功率 %）
    "wafertemp_tn_s": ("num", -1e-9, 1e5),
    "wafertemp_tv_s": ("num", -1e-9, 1e5),
    "wafertemp_td_s": ("num", -1e-9, 1e5),
    "cycle_s": ("num", 0.001, 1.0),      # PID 调用周期（4ms 对齐 PLC 扫描）
}

# bond_hw 组（热压键合被控对象硬件参数：下位机调试好即交付）。
BOND_HW_FIELDS = {
    "force_spring_k_n_per_mm": ("num", 1.0, 1e5),   # 接触弹簧刚度 N/mm
    "ambient_C": ("num", -40.0, 100.0),             # 环境温度
    "head_heater_max_C": ("num", 50.0, 1000.0),     # 脉冲加热器上限
    "pulse_heater_rate_cps": ("num", 0.001, 1e4),   # 脉冲加热满功率速率 ℃/s
    "wafer_heater_rate_cps": ("num", 0.001, 100.0),  # 硅片台满功率速率 ℃/s
    "wafer_heater_max_C": ("num", 50.0, 1000.0),    # 硅片台加热器上限
    # 硅片台主动冷却速率 ℃/s（2026-09-02 迭代 11：降温方向=目标<当前时
    # 按此速率线性逼近，替代自然热损对称速率；键合后检测前快速降温消费）
    "wafer_cool_rate_cps": ("num", 0.001, 100.0),
    # flip 交接接触面高度（迭代 49）：flip 吸盘裸面在 BS_Arm_Z 坐标系下的
    # z（真值旋钮，默认 −51.43：呈现位 FA_Z=fa_z_safe=20 时翻转吸盘面世界
    # z=1002.4 − 键合头吸盘底零位 1053.83）；engine._tick_force_locked 的
    # flip 交接接触力支路消费（flip 有效接触位=本值+芯片厚，与键合接触
    # 支路取 max）
    "flip_contact_z_mm": ("num", -190.0, 0.0),
}

DEFAULT_TUNING = {
    "_comment": "下位机调参：调试好即交付、不计入机械常数的参数（区别于 "
                "machine_constants.json 设备本体标定量）。pid 组=三回路 "
                "TwinCAT 系列型 PID 整定；bond_hw 组=热压键合被控对象硬件"
                "参数（弹簧刚度/双加热器速率限与量程上限/环境温度）。"
                "PUT /api/tuning 在线改 + 原子落盘 + 热生效（engine 每秒"
                "重读）。整定依据见开发交接.md 迭代 8。",
    "pid": {
        "_note": "force_*：键合力回路，输出=BS_Arm_Z 压入量 mm（对象=弹簧 "
                 "F=k·压入量 k=500N/mm，静态对象+4ms 采样构成代数环，P 通路"
                 "稳定条件 kp<1/k=0.002，取 0.0008（环增益 0.4）；Tn 消 P "
                 "静差）；headtemp_*/wafertemp_*：温度回路输出=功率 0~100%"
                 "（一阶热对象+速率限，纯 P 有静差，小 I 消除）；tn_s=0 关"
                 "积分（纯 PD）；tv_s/td_s=微分增益/滤波常数",
        "force_kp": 0.0008,
        "force_tn_s": 0.1,
        "force_tv_s": 0.0,
        "force_td_s": 0.01,
        "headtemp_kp": 3.0,
        "headtemp_tn_s": 1.0,
        "headtemp_tv_s": 0.0,
        "headtemp_td_s": 0.1,
        "wafertemp_kp": 20.0,
        "wafertemp_tn_s": 10.0,
        "wafertemp_tv_s": 0.0,
        "wafertemp_td_s": 1.0,
        "cycle_s": 0.004,
    },
    "bond_hw": {
        "_note": "热压键合被控对象硬件参数（engine 三回路 PID 被控对象消费）。"
                 "force_spring_k_n_per_mm=接触弹簧刚度（压头低于有效接触位后"
                 "真值力 F=k·压入量）；ambient_C=环境温度；pulse_heater_rate_"
                 "cps/head_heater_max_C=脉冲加热器满功率升温速率/量程上限；"
                 "wafer_heater_rate_cps/wafer_heater_max_C=硅片台陶瓷加热片"
                 "满功率速率（511.4℃/11.6min 实测）/上限；wafer_cool_rate_"
                 "cps=硅片台主动冷却速率（2026-09-02 迭代 11：Heat_Dw 降温"
                 "方向=目标<当前时按此速率线性逼近，替代自然热损）。热对象："
                 "dT/dt=功率%×rate − rate·(T−amb)/(max−amb)，再按 ±rate "
                 "限速并夹到 [amb,max]",
        "force_spring_k_n_per_mm": 500.0,
        "ambient_C": 25.0,
        "head_heater_max_C": 450.0,
        "pulse_heater_rate_cps": 200.0,
        "wafer_heater_rate_cps": 0.733,
        "wafer_heater_max_C": 511.4,
        "wafer_cool_rate_cps": 5.0,
        "flip_contact_z_mm": -51.43,
    },
}

# 旧 machine_constants.json 迁移键清单（2026-08-31 迭代 9）
MIGRATE_BOND_HW_KEYS = tuple(BOND_HW_FIELDS)
MIGRATE_NOISE_KEYS = ("temp_noise_C", "force_noise_N")


def _is_finite_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def validate(data):
    """全量 schema 校验 -> 错误明细列表（空=合法）。"""
    errors = []
    if not isinstance(data, dict):
        return ["顶层须为对象"]
    for key in data:
        if key.startswith("_"):
            continue
        if key not in GROUPS:
            errors.append("未知分组 %r（当前只开放 %s）" % (key, GROUPS))
    for name, fields in (("pid", PID_FIELDS), ("bond_hw", BOND_HW_FIELDS)):
        g = data.get(name)
        if g is None:
            errors.append("%s 组缺失" % name)
            continue
        if not isinstance(g, dict):
            errors.append("%s 须为对象" % name)
            continue
        for key in g:
            if key.startswith("_"):
                continue
            if key not in fields:
                errors.append("%s.%s 未知字段（白名单 %s）"
                              % (name, key, tuple(fields)))
        for key, spec in fields.items():
            if key not in g:
                errors.append("%s.%s 缺失" % (name, key))
                continue
            _kind, lo, hi = spec
            v = g[key]
            if not _is_finite_num(v) or not (lo <= v <= hi):
                errors.append("%s.%s 须在 [%s,%s] 数值: %r"
                              % (name, key, lo, hi, v))
    return errors


def _deep_merge(dst, patch):
    for k, v in patch.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def _with_defaults(data):
    """默认值深合并补齐（缺失组/字段以前向兼容；不改传入对象）。"""
    out = json.loads(json.dumps(DEFAULT_TUNING))
    if isinstance(data, dict):
        _deep_merge(out, data)
    return out


class LoadError(ValueError):
    """下位机调参文件加载/校验失败（启动拒绝拉起）。"""


class Tuning(object):
    """下位机调参注册表：线程安全读取 + 校验更新 + 原子落盘。"""

    def __init__(self, path=TUNING_JSON):
        self._path = path
        self._lock = threading.RLock()
        if not os.path.exists(path):
            _atomic_write(path, json.loads(json.dumps(DEFAULT_TUNING)))
        with open(path, encoding="utf-8") as f:
            data = _with_defaults(json.load(f))
        errors = validate(data)
        if errors:
            raise LoadError("tuning.json 校验失败（%d 条）：\n  %s"
                            % (len(errors), "\n  ".join(errors)))
        self._d = data

    def get(self):
        """全量快照（深拷贝）。"""
        with self._lock:
            return json.loads(json.dumps(self._d))

    def update(self, patch):
        """部分字段合并 → 全量校验 → 原子写盘 + 热替换内存。
        校验失败抛 ValueError（消息含逐条 errors），原参数不污染。"""
        if not isinstance(patch, dict):
            raise ValueError("patch 须为对象")
        with self._lock:
            merged = json.loads(json.dumps(self._d))
            _deep_merge(merged, patch)
            errors = validate(merged)
            if errors:
                raise ValueError("下位机调参校验失败（%d 条）：\n  %s"
                                 % (len(errors), "\n  ".join(errors)))
            _atomic_write(self._path, merged)
            self._d = merged

    def public(self):
        """GET 响应：全量分组（无派生量）。"""
        with self._lock:
            const = {g: json.loads(json.dumps(v))
                     for g, v in self._d.items() if not g.startswith("_")}
        return {"ok": True,
                "groups": [g for g in self._d if not g.startswith("_")],
                "tuning": const}


def migrate_legacy(mc_path=None, tuning_path=TUNING_JSON, recipe_path=None,
                   log=print):
    """一次性迁移（2026-08-31 迭代 9）：旧 machine_constants.json 的 pid 段
    与 bond 段 6 个硬件键 → tuning.json；bond 段读数噪声 2 键
    （temp_noise_C/force_noise_N）→ recipe.json sim_sensors 组。迁移后从
    machine_constants.json 剔除并落盘。无可迁内容时静默返回 0。"""
    if mc_path is None:
        mc_path = os.path.join(ROOT, "machine_constants.json")
    if recipe_path is None:
        recipe_path = os.path.join(ROOT, "recipe.json")
    if not os.path.exists(mc_path):
        return 0
    with open(mc_path, encoding="utf-8") as f:
        mc = json.load(f)
    moved_pid, moved_hw, moved_noise = {}, {}, {}
    pid = mc.get("pid")
    if isinstance(pid, dict):
        moved_pid = {k: v for k, v in pid.items() if not k.startswith("_")}
    bond = mc.get("bond")
    if isinstance(bond, dict):
        for k in MIGRATE_BOND_HW_KEYS:
            if k in bond:
                moved_hw[k] = bond[k]
        for k in MIGRATE_NOISE_KEYS:
            if k in bond:
                moved_noise[k] = bond[k]
    n = len(moved_pid) + len(moved_hw) + len(moved_noise)
    if not n:
        return 0
    # 1) pid + 6 硬件键 → tuning.json（tuning.json 已有显式值不覆盖；
    #    注意在"原始文件"上 setdefault——_with_defaults 会先把默认值填满）
    if moved_pid or moved_hw:
        if os.path.exists(tuning_path):
            with open(tuning_path, encoding="utf-8") as f:
                raw = json.load(f)
            if not isinstance(raw, dict):
                raw = {}
        else:
            raw = {}
        pid_g = raw.setdefault("pid", {})
        hw_g = raw.setdefault("bond_hw", {})
        for k, v in moved_pid.items():
            pid_g.setdefault(k, v)
        for k, v in moved_hw.items():
            hw_g.setdefault(k, v)
        _atomic_write(tuning_path, _with_defaults(raw))
    # 2) 噪声 2 键 → recipe.json sim_sensors（已有值不覆盖）
    if moved_noise and os.path.exists(recipe_path):
        with open(recipe_path, encoding="utf-8") as f:
            rec = json.load(f)
        ss = rec.get("sim_sensors")
        if not isinstance(ss, dict):
            ss = rec["sim_sensors"] = {}
        for k, v in moved_noise.items():
            ss.setdefault(k, v)
        _atomic_write(recipe_path, rec)
    # 3) 从 machine_constants.json 剔除并落盘
    if isinstance(pid, dict):
        del mc["pid"]
    if isinstance(bond, dict):
        for k in MIGRATE_BOND_HW_KEYS + MIGRATE_NOISE_KEYS:
            bond.pop(k, None)
    _atomic_write(mc_path, mc)
    log("[boot] tuning 迁移：machine_constants.json → tuning.json "
        "pid %d 键/bond_hw %d 键，→ recipe.json sim_sensors 噪声 %d 键"
        % (len(moved_pid), len(moved_hw), len(moved_noise)))
    return n
