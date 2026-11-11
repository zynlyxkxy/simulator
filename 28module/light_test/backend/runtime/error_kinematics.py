# -*- coding: utf-8 -*-
"""error_kinematics.py — 串行运动链 6DOF 误差合成（工具点位姿误差层）。

与 error_system.py 的单轴读数层正交、两开关相互独立：
- 单轴层（ErrorSystem.enabled）：actual 通道，每轴读数叠加沿轴误差；
- 本模块（six_dof_enabled，默认开）：把链上每根轴的 6 分量误差地图
  （dx/dy/dz/rx/ry/rz）沿"基座→工具点"串行链逐级合成，输出各模块工具点
  的 6DOF 位姿误差。只进测量/显示通道，不进轨迹积分、不推 3D 前端
  （µm 级在 3D 不可见）。

合成模型（精确串行链逐点变换，非一阶雅可比近似）：
- 正解：工具点与下游旋转枢轴先按名义轴值 q 做正解——T 轴平移 q·dir，
  R 轴绕默认枢轴转 q（罗德里格斯精确式）；枢轴随上游名义运动联动。
- 误差：轴 i 在当前位置采样 6 分量地图得平移 t_i 与旋转矢量 r_i；
  另把该轴沿轴的 repeat+fixed+scale 标量叠加进去（直线轴并进 t_i 沿
  dir_i 方向，旋转轴并进 r_i 绕 dir_i——两边量纲一致才加）。
  误差变换 E_i(p) = R(r_i 过 c_i)(p − c_i) + c_i + t_i，其中 c_i 为被上游
  （名义+误差）以及**本轴自身名义运动**变换后的当前生效枢轴；E_i 按链序
  依次作用于下游所有枢轴与工具点。**T/R 链元误差行为完全统一**：直线轴
  的 drx/dry/drz 角度误差同样是台面绕枢轴的微小转动，在工具点产生平移
  偏移（阿贝误差 = θ × 杠杆臂）。
- 旋转用精确罗德里格斯：R(v) = v·cosθ + (k×v)·sinθ + k(k·v)(1−cosθ)，
  k = r̂、θ = |r|。µrad 量级下与一阶式差别不可测，但同一套代码在名义正解
  的大角度（FA_Rx/CH_Rz ±180°）下同样成立。
- 工具点姿态误差输出 = Σ r_i（矢量求和）：µrad 量级小旋转可交换，
  求和的交换误差 ~|r_i||r_j|（prad 级），远小于显示分辨率（0.001µrad），
  足够精确。
- 每轴贡献分解（/api/toolpoints 的 axes 数组）：记录每步 E_i 作用前后的
  工具点实际增量——trans（平移直接贡献 t_i）、rot_trans（旋转致平移=阿贝项，
  即 E_i 的旋转单独作用于当前工具点产生的位移）、attitude（姿态贡献 r_i）、
  sample（6 分量采样含沿轴并入）。因记录的是实际增量，各轴贡献之和 =
  模块总计（伸缩和，精确成立；非线性合成下"贡献"定义本就是增量口径）。

旋转枢轴语义（2026-08-20 澄清，T/R 统一）：每根轴（含直线轴）都有旋转枢轴
——轴角度误差（drx/dry/drz）的物理支点（轴承中心等效点）。配置里的 center
是**默认枢轴**（只读，PUT 拒改）；每根链元可选 custom_center（默认 null）——
**生效枢轴 = custom_center ?? 默认 center**，仅用于误差旋转（阿贝平移量
评估）。枢轴随轴自身名义运动：T 轴的枢轴在机架坐标中随该轴名义位置平移
（c_i 先被上游变换、再被自身名义运动 T_i(q_i) 变换，然后作为 E_i 的支点）；
R 轴自身转动不改变枢轴（绕自身轴转枢轴不动）；下游轴运动不影响本轴枢轴。
R 轴的名义正解始终绕默认枢轴（机械真转轴）。

几何（center / tool_point）为机架坐标系（右手系：X 向右、Y 向后、Z 向上）
零位姿态下的位置；默认值从 sandboxes.json 的零件 AABB 挖出（来源见
error_kinematics.json 各 _note），未经真机标定；custom_center/tool_point 可经
PUT /api/kinematics 在线修改（默认运行时生效、不持久化；请求带
"persist": true 时把当前生效配置原子写回 error_kinematics.json，见
persist()）。

Y 导轨方向真值（迭代 36，stage_geo 消费）：machine_constants stage_geo 组
as/ws_y_rail_dir_urad（对准台 AS_Y/硅片台 WS_LS_Y 链元）为一阶真值矩阵
A=[[1+εx,sinφ],[0,(1+εy)cosφ]] 的 φ（X 基准轴）——误差注入段把该链元
实际平移 y·(sinφ,cosφ) 相对名义 y·(0,1) 的偏差 y·(sinφ,cosφ−1) 并入
E_i 平移（沿链传播同 t_i）。φ=0 偏差恒零=逐字节恒等；six_dof 链关时
不生效（误差注入段整体跳过）；常数经 self.constants 引用读时取最新
（口径同 modal/thermal 注入，calibration 服务构造时兜底接线），未接线=
空表=恒等。as/ws_stage_nonortho 测校 apply 写回的 *_nonortho_urad 是
认知侧结果存储，本层不消费。
"""
import copy
import json
import math
import os
import threading

import axes_config as ac

ROOT = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(ROOT, "error_kinematics.json")


# ---------------- 三维小工具（纯标准库，只够用） ----------------
def _vec3(v, what, errors):
    """校验有限三维向量；合法返回 [x,y,z]，否则记一条错误并返回 None。"""
    ok = (isinstance(v, (list, tuple)) and len(v) == 3
          and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                  and math.isfinite(x) for x in v))
    if not ok:
        errors.append("%s 须为三维有限数向量，得到 %r" % (what, v))
        return None
    return [float(v[0]), float(v[1]), float(v[2])]


def _norm3(v, what, errors):
    """校验并归一化单位向量；零向量记错误。"""
    v = _vec3(v, what, errors)
    if v is None:
        return None
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        errors.append("%s 不能是零向量" % what)
        return None
    return [v[0] / n, v[1] / n, v[2] / n]


def rodrigues(p, c, k, theta):
    """点 p 绕单位向量 k 过点 c 的轴旋转 theta（弧度，精确式）。"""
    x, y, z = p[0] - c[0], p[1] - c[1], p[2] - c[2]
    ct, st = math.cos(theta), math.sin(theta)
    d = k[0] * x + k[1] * y + k[2] * z
    return [c[0] + x * ct + (k[1] * z - k[2] * y) * st + k[0] * d * (1 - ct),
            c[1] + y * ct + (k[2] * x - k[0] * z) * st + k[1] * d * (1 - ct),
            c[2] + z * ct + (k[0] * y - k[1] * x) * st + k[2] * d * (1 - ct)]


def _rotvec_apply(p, c, r):
    """按旋转矢量 r（rad）把点 p 绕过 c 的轴旋转；|r|≈0 时原样返回。"""
    theta = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    if theta < 1e-15:
        return list(p)
    k = [r[0] / theta, r[1] / theta, r[2] / theta]
    return rodrigues(p, c, k, theta)


class ErrorKinematics(object):
    """六自由度误差合成器：配置 + 采样 + 合成，全部读取时解算。"""

    def __init__(self, errors, cfg_path=CFG_PATH):
        self.errors = errors              # ErrorSystem（地图采样来源）
        self._lock = threading.RLock()
        self.six_dof_enabled = True       # 六自由度总开关（运行时态，不持久化）
        # 全机模态仿真（main 注入 modal.ModalSystem，2026-09-08 迭代 22）：
        # compute() 输出叠加 MIMO 模态的模块 6DOF 投影（enabled 热查）
        self.modal = None
        # 组件级热仿真（main 注入 thermal.ThermalSystem，2026-09-09 迭代 23）：
        # compute() 输出叠加热变形映射的模块 6DOF 偏移（enabled 热查）
        self.thermal = None
        # 机器常数（MachineConstants；迭代 36 起 consumption：stage_geo 组
        # Y 导轨方向角真值，compute() 读时取最新，未接线=空表=φ0 恒等）。
        # 接线口径同 modal/thermal 的引用注入（calibration 服务构造时
        # 兜底接线，见 calibration.py）
        self.constants = None
        with open(cfg_path, encoding="utf-8") as f:
            cfg = json.load(f)
        errs = self.validate(cfg)
        if errs:
            raise ValueError("error_kinematics.json 校验失败（%d 条）：\n  %s"
                             % (len(errs), "\n  ".join(errs)))
        self._cfg = cfg

    @classmethod
    def from_dict(cls, errors, cfg):
        """测试用：直接从字典构造（跳过文件）。"""
        self = cls.__new__(cls)
        self.errors = errors
        self._lock = threading.RLock()
        self.six_dof_enabled = True
        self.modal = None               # 同 __init__：模态分量缺省无
        self.thermal = None             # 同 __init__：热变形分量缺省无
        self.constants = None           # 同 __init__：机器常数缺省无=φ0 恒等
        errs = self.validate(cfg)
        if errs:
            raise ValueError("配置校验失败（%d 条）：\n  %s"
                             % (len(errs), "\n  ".join(errs)))
        self._cfg = cfg
        return self

    # ---------------- 校验 ----------------
    @staticmethod
    def validate(cfg):
        """schema 校验 -> 错误明细列表（空=合法）。不修改入参。"""
        errors = []
        if not isinstance(cfg, dict) or not isinstance(cfg.get("modules"), dict):
            return ["顶层须含 modules 字典"]
        if not cfg["modules"]:
            errors.append("modules 不能为空")
        for mname, mod in cfg["modules"].items():
            pre = "模块 %s" % mname
            if not isinstance(mod, dict):
                errors.append("%s 须为对象" % pre)
                continue
            _vec3(mod.get("tool_point"), pre + " tool_point", errors)
            chain = mod.get("chain")
            if not isinstance(chain, list) or not chain:
                errors.append("%s chain 须为非空数组" % pre)
                continue
            for i, link in enumerate(chain):
                lp = "%s chain[%d]" % (pre, i)
                if not isinstance(link, dict):
                    errors.append("%s 须为对象" % lp)
                    continue
                axis = link.get("axis")
                if axis not in ac.NAME_TO_IDX:
                    errors.append("%s axis 未知：%r" % (lp, axis))
                    continue
                kind = link.get("kind")
                if kind not in ("T", "R"):
                    errors.append("%s kind 只能是 T（平移）/ R（旋转），得到 %r"
                                  % (lp, kind))
                    continue
                # kind 与轴类型一致性（直线轴配 T、旋转轴配 R）
                want = "R" if ac.axis_is_rotary(axis) else "T"
                if kind != want:
                    errors.append("%s 轴 %s 是%s轴，kind 应为 %s"
                                  % (lp, axis,
                                     "旋转" if want == "R" else "直线", want))
                if _norm3(link.get("dir"), lp + " dir", errors) is None:
                    continue
                # 所有链元（T/R）都必须有默认旋转枢轴 center
                _vec3(link.get("center"), lp + " center（旋转枢轴，必填）",
                      errors)
                # custom_center：可选，null（=用默认）或三维有限向量
                cc = link.get("custom_center")
                if cc is not None:
                    _vec3(cc, lp + " custom_center", errors)
        return errors

    # ---------------- 在线改配置 ----------------
    def update(self, req):
        """PUT /api/kinematics：部分字段合并（只提交改过的）。
        req = {"modules": {name: {"tool_point"?: [x,y,z],
                                  "chain"?: [{"axis":..., "custom_center"?}]}}}
        链元只允许改 custom_center（null=清除恢复默认）；默认 center 只读，
        传 center 直接报错。-> 错误明细列表（空=已生效）；有错误则整体不应用。"""
        if not isinstance(req, dict) or not isinstance(req.get("modules"), dict):
            return ["请求体须含 modules 字典"]
        with self._lock:
            merged = copy.deepcopy(self._cfg)
        errors = []
        for mname, patch in req["modules"].items():
            pre = "模块 %s" % mname
            mod = merged["modules"].get(mname)
            if mod is None:
                errors.append("%s 不存在" % pre)
                continue
            if not isinstance(patch, dict):
                errors.append("%s 的补丁须为对象" % pre)
                continue
            for key in patch:
                if key not in ("tool_point", "chain"):
                    errors.append("%s 不支持改字段 %r（仅 tool_point/chain）"
                                  % (pre, key))
            if "tool_point" in patch:
                mod["tool_point"] = patch["tool_point"]
            for lpatch in patch.get("chain") or []:
                axis = lpatch.get("axis") if isinstance(lpatch, dict) else None
                link = next((l for l in mod["chain"] if l.get("axis") == axis),
                            None)
                if link is None:
                    errors.append("%s 链中无轴 %r" % (pre, axis))
                    continue
                for key in lpatch:
                    if key == "axis":
                        continue
                    if key == "center":
                        errors.append("%s 链元 %s 默认旋转枢轴不可修改"
                                      "（center 为只读默认值，请用 "
                                      "custom_center，null 恢复默认）"
                                      % (pre, axis))
                        continue
                    if key != "custom_center":
                        errors.append("%s 链元 %s 不支持改字段 %r"
                                      "（仅 custom_center）" % (pre, axis, key))
                        continue
                    link["custom_center"] = lpatch[key]
        errors.extend(self.validate(merged))
        if errors:
            return errors
        with self._lock:
            self._cfg = merged
        return []

    def config_public(self):
        """GET /api/kinematics：当前配置（每链元——T/R 一致——附带
        custom_center 与 effective_center = custom_center ?? center）。"""
        with self._lock:
            modules = copy.deepcopy(self._cfg["modules"])
            enabled = self.six_dof_enabled
        for mod in modules.values():
            for l in mod.get("chain", []):
                l["custom_center"] = l.get("custom_center")
                l["effective_center"] = list(
                    l["custom_center"] or l["center"])
        return {"ok": True, "six_dof_enabled": enabled, "modules": modules}

    def set_enabled(self, on):
        with self._lock:
            self.six_dof_enabled = bool(on)

    def persist(self, cfg_path=CFG_PATH):
        """PUT /api/kinematics 带 "persist": true 时由网关调用：把当前内存
        配置（_note/label 等附属字段随 update 的深合并原样保留）原子写回
        error_kinematics.json（tmp+replace，同 machine_constants 惯例）；
        写前全量 validate、写后重载再校验，坏配置不留在盘上。"""
        with self._lock:
            cfg = copy.deepcopy(self._cfg)
        errors = self.validate(cfg)
        if errors:
            raise ValueError("persist 校验失败（%d 条）：\n  %s"
                             % (len(errors), "\n  ".join(errors)))
        tmp = cfg_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, cfg_path)
        with open(cfg_path, encoding="utf-8") as f:
            errs2 = self.validate(json.load(f))
        if errs2:
            raise ValueError("persist 写后重载校验失败：\n  %s"
                             % "\n  ".join(errs2))

    # ---------------- 合成 ----------------
    def _stage_geo_rad(self):
        """stage_geo 机器常数 → {Y 轴名: φ_rad}（Y 导轨方向角真值，迭代 36）。

        读时取最新（同 flow_engine 等 constants.get() 口径，PUT 机器常数
        热生效即反映）；self.constants 未接线/读取出错 → 空表 → 全轴
        φ=0 → 恒等（零行为变化）。数值经有限性过滤，键映射事实源
        machine_constants.STAGE_GEO_RAIL_KEYS。"""
        c = self.constants
        if c is None:
            return {}
        try:
            g = (c.get() or {}).get("stage_geo") or {}
        except Exception:
            return {}
        try:
            from machine_constants import STAGE_GEO_RAIL_KEYS
        except ImportError:
            STAGE_GEO_RAIL_KEYS = {"WS_LS_Y": "ws_y_rail_dir_urad",
                                   "AS_Y": "as_y_rail_dir_urad"}
        out = {}
        for axis, key in STAGE_GEO_RAIL_KEYS.items():
            v = g.get(key, 0.0)
            if isinstance(v, (int, float)) and not isinstance(v, bool) \
                    and math.isfinite(v) and v != 0.0:
                out[axis] = float(v) * 1e-6     # µrad → rad
        return out

    def _module_pose(self, mod, pos, enabled, stage_geo=None):
        """单模块合成 -> (nominal[3], err_mm[3], err_rad[3], axes[])。

        两遍走链：先名义正解（T 平移 / R 绕**默认**枢轴转——机械真转轴），
        再按链序把每轴的 6 分量误差 E_i 作用于下游枢轴与工具点（误差
        旋转绕**生效**枢轴 = custom_center ?? center，T/R 行为统一）。
        枢轴随动规则：T 轴枢轴随自身名义平移（自身+上游变换后作为 E_i 支点）、
        R 轴自身转动枢轴不动、下游轴运动不影响本轴枢轴。
        axes[] 为每轴贡献分解（记录 E_i 作用前后的实际增量，各轴之和 =
        模块总计，伸缩和精确成立）。

        stage_geo（迭代 36）：{Y 轴名: φ_rad} Y 导轨方向真值——一阶真值
        矩阵 A=[[1+εx,sinφ],[0,(1+εy)cosφ]]（X 基准轴）的 FK 消费：该轴
        名义平移贡献 y·(dir=(0,1,0)) 的真值偏差 Δt=y·(sinφ,cosφ−1,0)
        并入该链元 E_i 的平移部分（沿链传播到下游枢轴与工具点，同 t_i
        语义）。φ=0 → 偏差恒 0 → 逐字节恒等；six_dof 链关时本注入不生效
        （在误差注入段，enabled=False 提前返回零误差）。"""
        chain = mod["chain"]
        tool = list(mod["tool_point"])
        # 名义（默认）枢轴与生效枢轴两套，均随上游+自身名义运动联动
        centers = {i: list(l["center"]) for i, l in enumerate(chain)}
        eff_centers = {i: list(l.get("custom_center") or l["center"])
                       for i, l in enumerate(chain)}

        def downstream(i, table):
            """链元 i 之后的链元下标。"""
            return [j for j in table if j > i]

        # ---- 名义正解（R 绕默认枢轴） ----
        for i, l in enumerate(chain):
            q = pos.get(l["axis"], 0.0)
            if l["kind"] == "T":
                d = [q * l["dir"][0], q * l["dir"][1], q * l["dir"][2]]
                for k in range(3):
                    tool[k] += d[k]
                # 自身枢轴也随本轴名义平移（之后作为 E_i 的支点）
                for table in (centers, eff_centers):
                    for j in downstream(i, table) + [i]:
                        for k in range(3):
                            table[j][k] += d[k]
            else:
                # R 轴自身转动不改变自身枢轴（绕自身轴转，枢轴不动）
                c = centers[i]
                tool = rodrigues(tool, c, l["dir"], q)
                for table in (centers, eff_centers):
                    for j in downstream(i, table):
                        table[j] = rodrigues(table[j], c, l["dir"], q)
        nominal = list(tool)
        axes_out = []
        if not enabled:
            for l in chain:
                axes_out.append({"axis": l["axis"], "kind": l["kind"],
                                 "sample_mm": [0.0] * 3, "sample_rad": [0.0] * 3,
                                 "trans_mm": [0.0] * 3, "rot_trans_mm": [0.0] * 3,
                                 "attitude_rad": [0.0] * 3})
            return nominal, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], axes_out

        # ---- 误差注入（精确串行链合成，绕生效枢轴，T/R 统一） ----
        attitude = [0.0, 0.0, 0.0]
        for i, l in enumerate(chain):
            t, r, along = self.errors.sample6(l["axis"],
                                              pos.get(l["axis"], 0.0))
            t, r = list(t), list(r)
            if l["kind"] == "T":
                # 沿轴标量（repeat+fixed+scale，mm）并进平移沿 dir 方向
                for k in range(3):
                    t[k] += along * l["dir"][k]
                # 迭代 36：Y 导轨方向真值（stage_geo）——Y 链元实际平移
                # y·(sinφ,cosφ) 相对名义 y·(0,1) 的偏差并入平移贡献
                phi = (stage_geo or {}).get(l["axis"], 0.0)
                if phi:
                    q = pos.get(l["axis"], 0.0)
                    t[0] += q * math.sin(phi)
                    t[1] += q * (math.cos(phi) - 1.0)
            else:
                # 旋转轴沿轴标量（rad）并进旋转矢量绕 dir 方向
                for k in range(3):
                    r[k] += along * l["dir"][k]
            for k in range(3):
                attitude[k] += r[k]      # µrad 量级矢量求和（见模块 docstring）
            rot_delta = [0.0, 0.0, 0.0]
            if any(r):
                # 阿贝项：旋转单独作用于当前工具点产生的位移（T/R 统一）
                c = eff_centers[i]       # 自身+上游（名义+误差）变换后的生效枢轴
                rotated = _rotvec_apply(tool, c, r)
                for k in range(3):
                    rot_delta[k] = rotated[k] - tool[k]
                tool = rotated
                for j in downstream(i, eff_centers):
                    eff_centers[j] = _rotvec_apply(eff_centers[j], c, r)
            # E_i 的平移部分（t_i 直接贡献）
            if any(t):
                for k in range(3):
                    tool[k] += t[k]
                for j in downstream(i, eff_centers):
                    for k in range(3):
                        eff_centers[j][k] += t[k]
            axes_out.append({"axis": l["axis"], "kind": l["kind"],
                             "sample_mm": t, "sample_rad": r,
                             "trans_mm": list(t),
                             "rot_trans_mm": rot_delta,
                             "attitude_rad": list(r)})
        err_mm = [tool[k] - nominal[k] for k in range(3)]
        return nominal, err_mm, attitude, axes_out

    def compute(self, positions):
        """positions = {轴名: 名义值(mm/rad)} -> /api/toolpoints 载荷。"""
        with self._lock:
            enabled = self.six_dof_enabled
            cfg = copy.deepcopy(self._cfg["modules"])
        # 模态 MIMO 分量（2026-09-08 迭代 22）：模块 6DOF 投影读取时合成，
        # 与本层 six_dof_enabled 开关独立（modal 未挂载/未启用 → 零）
        modal = self.modal
        modal_on = modal is not None and modal.enabled
        # 热变形分量（2026-09-09 迭代 23）：deform_map 的模块 6DOF 偏移，
        # 同样独立开关（thermal 未挂载/未启用 → 零）
        thermal = self.thermal
        thermal_on = thermal is not None and thermal.enabled
        # Y 导轨方向真值（迭代 36）：本 compute 调用读一次（缓存），
        # 全模块共用——口径同 modal/thermal 的读时热查
        stage_geo = self._stage_geo_rad()
        modules = {}
        for mname, mod in cfg.items():
            nominal, err_mm, err_rad, axes = self._module_pose(
                mod, positions, enabled, stage_geo)
            if modal_on:
                # 模态位移叠加在模块误差总量上（不进 axes[] 逐轴分解——
                # 分解只描述误差地图链；模态是模块级独立来源）
                mt, mr = modal.module_offset(mname)
                for k in range(3):
                    err_mm[k] += mt[k]
                    err_rad[k] += mr[k]
            if thermal_on:
                # 热变形偏移同口径叠加（模块级独立来源，不进 axes[] 分解）
                tt, tr = thermal.module_offset(mname)
                for k in range(3):
                    err_mm[k] += tt[k]
                    err_rad[k] += tr[k]
            axes_pub = []
            for a in axes:
                axes_pub.append({
                    "axis": a["axis"], "kind": a["kind"],
                    # 采样原值（含沿轴并入）：mm / rad
                    "sample_mm": [round(v, 9) for v in a["sample_mm"]],
                    "sample_rad": [round(v, 12) for v in a["sample_rad"]],
                    # 平移直接贡献 / 旋转致平移（阿贝项）：mm，附 nm 显示字段
                    "trans_mm": [round(v, 9) for v in a["trans_mm"]],
                    "trans_nm": [round(v * 1e6, 3) for v in a["trans_mm"]],
                    "rot_trans_mm": [round(v, 9) for v in a["rot_trans_mm"]],
                    "rot_trans_nm": [round(v * 1e6, 3)
                                     for v in a["rot_trans_mm"]],
                    # 姿态贡献：rad，附 µrad 显示字段
                    "attitude_rad": [round(v, 12) for v in a["attitude_rad"]],
                    "attitude_urad": [round(v * 1e6, 6)
                                      for v in a["attitude_rad"]],
                })
            modules[mname] = {
                "label": mod.get("label", mname),
                "nominal_mm": [round(v, 6) for v in nominal],
                "error_mm": [round(v, 9) for v in err_mm],
                "error_rad": [round(v, 12) for v in err_rad],
                # 显示用：nm / µrad
                "error_nm": [round(v * 1e6, 3) for v in err_mm],
                "error_urad": [round(v * 1e6, 6) for v in err_rad],
                "axes": axes_pub,
            }
        return {"ok": True, "six_dof_enabled": enabled, "modules": modules}
