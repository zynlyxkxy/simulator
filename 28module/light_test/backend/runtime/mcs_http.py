# -*- coding: utf-8 -*-
"""mcs_http.py — Cordis MCS HTTP 适配壳（HTTP 9190）。

真机报文格式（wire schema 见 docs/Cordis_MCS协议_V1.md，逐字段还原自
部分接口信息/io_template/.../PlcClient.cpp）：

  POST http://<ip>:9190/JsonServices/com.cordis.explorer.connectors.MachinePart.MachinePartConnector/<Service>
  请求体  [{"SecurityToken":"ADMIN"}, {"InputArgs":[...]}]
  成功    {"type":"SUCCESS","value":...}
  失败    {"type":"FAULT","value":null,"error":{"code":...,"message":...}}（HTTP 仍 200，
          客户端只解析 body 不看 HTTP 码；信封/JSON 错误才用 HTTP 400）

六服务：getMachinePartProperties / executeCommand /
getObserverValuesFromObserverPaths / getSettings /
setCurrentValueForSetting / saveCurrentValueForSetting
（客户端代码实证为后五个里的 saveCurrentValueForSetting；getSettings 仅
 KNOWLEDGE.md 提及，schema 为推断——均实现。）

节点库 = 同目录 mcs_map.json（唯一事实源，fullPath 一律 /→. 归一化后匹配，
蓝本里两种分隔符混用）。命令/观测经 kind 翻译到现有语义层（engine/points/
workpieces），与 9102 plc_http 并存、互不影响。
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import axes_config as ac
from engine import AIR_BASE, InterlockError

MCS_SERVICE_PATH = ("/JsonServices/com.cordis.explorer.connectors."
                    "MachinePart.MachinePartConnector/")
MCS_MAP_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "mcs_map.json")
CONTROLLER_PREFIX = "3000_1_Controller"     # 源自同事模板，1928 真机前缀待复核

# ---- 共享模板（mcs_map.json 里 "STAGE_STD"/"VACUUM_STD" 占位，加载时展开）----
# PLC 运动 stage 标准命令面（motion config1~4 plc/*.json cmd_bindings 实证：
# Stop→Halt、ResetError→"Reset Error"、SetVelAcc→SetMoveParam；Jog/SetDec/
# SetJerk/SetKillDec 绑定为空=不支持，不出现在模板里）
_STAGE_STD_COMMANDS = [
    {"name": "Enable",       "arguments": []},
    {"name": "Disable",      "arguments": []},
    {"name": "Home",         "arguments": []},
    {"name": "MoveAbs",      "arguments": [{"name": "Position",
                                            "type": "FLOAT_TYPED_VALUE"}]},
    {"name": "MoveRel",      "arguments": [{"name": "Position",
                                            "type": "FLOAT_TYPED_VALUE"}]},
    {"name": "Halt",         "arguments": []},
    {"name": "Reset Error",  "arguments": []},
    {"name": "SetMoveParam", "arguments": [{"name": "Velocity",
                                            "type": "FLOAT_TYPED_VALUE"},
                                           {"name": "Acceleration",
                                            "type": "FLOAT_TYPED_VALUE"}]},
]
_STAGE_STD_OBSERVERS = [
    {"name": "oCurrentPos_GC", "type": "FLOAT_OBSERVER"},
    {"name": "obMoveSpd",      "type": "FLOAT_OBSERVER"},
    {"name": "oMotorEnabled",  "type": "BOOLEAN_OBSERVER"},
    {"name": "oMoving",        "type": "BOOLEAN_OBSERVER"},
    {"name": "oInPos",         "type": "BOOLEAN_OBSERVER"},
]
_STAGE_STD_SETTINGS = [
    {"name": "MoveVelocity",   "type": "FLOAT_TYPED_VALUE", "bind": "vmax"},
    {"name": "MoveAcc",        "type": "FLOAT_TYPED_VALUE", "bind": "amax"},
    {"name": "CommandTimeOut", "type": "FLOAT_TYPED_VALUE",
     "bind": None, "default": 30000.0},
]
# 真空吸盘标准命令面（io_config.json 实证：Close/OpenNegative/OpenPositive +
# AirRoute 三观测）
_VACUUM_STD_COMMANDS = [
    {"name": "Close",        "arguments": []},
    {"name": "OpenNegative", "arguments": []},
    {"name": "OpenPositive", "arguments": []},
]
_VACUUM_STD_OBSERVERS = [
    {"name": "AirRouteClose",    "type": "BOOLEAN_OBSERVER"},
    {"name": "AirRouteNegative", "type": "BOOLEAN_OBSERVER"},
    {"name": "AirRoutePositive", "type": "BOOLEAN_OBSERVER"},
]

# MCS 真空命令 → 内部气路算子（内部沿用点位表笔误 OpenNegtive/OpenAir）
_VACUUM_OP = {"Close": "OpenAir", "OpenNegative": "OpenNegtive",
              "OpenPositive": "OpenPositive"}
# AirRoute 观测 → 内部气路状态观测点叶名（点位表笔误 oNegtiveState 沿用）
_AIRROUTE_POINT = {"AirRouteClose": "oAirState",
                   "AirRouteNegative": "oNegtiveState",
                   "AirRoutePositive": "oPositiveState"}


class MCSFault(Exception):
    """真机式业务错误：HTTP 200 + {"type":"FAULT",...}。"""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def _norm(path):
    """fullPath 归一化：蓝本 / 与 . 混用，统一成 . 匹配。"""
    return (path or "").strip().strip("/").replace("/", ".")


def _leaf(path):
    n = _norm(path)
    return n.rsplit(".", 1)[-1]


def _machine_part_obj(entry):
    """machinePart 三字段（KNOWLEDGE.md：必须齐备）。
    internalPath = fullPath 去控制器前缀（推断规则，真机取值待联调核对）。"""
    fp = entry["fullPath"]
    norm = _norm(fp)
    prefix = _norm(CONTROLLER_PREFIX) + "."
    internal = norm[len(prefix):] if norm.startswith(prefix) else norm
    return {"fullPath": fp, "internalPath": internal.replace(".", "/"),
            "name": _leaf(fp)}


def _expand_templates(entry):
    """把 mcs_map.json 里的 STAGE_STD/VACUUM_STD 占位展开成实体模板。"""
    def _exp(v, std):
        return json.loads(json.dumps(std)) if v == std else v
    if entry["kind"] == "axis_stage":
        entry["commands"] = _exp(entry["commands"], _STAGE_STD_COMMANDS)
        entry["observers"] = _exp(entry["observers"], _STAGE_STD_OBSERVERS)
        entry["settings"] = _exp(entry["settings"], _STAGE_STD_SETTINGS)
    elif entry["kind"] == "vacuum":
        entry["commands"] = _exp(entry["commands"], _VACUUM_STD_COMMANDS)
        entry["observers"] = _exp(entry["observers"], _VACUUM_STD_OBSERVERS)


def load_mcs_map(path=MCS_MAP_JSON):
    """加载映射表 -> {归一化 fullPath: entry}（展开共享模板）。"""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for entry in data["machineParts"]:
        _expand_templates(entry)
        out[_norm(entry["fullPath"])] = entry
    return out


def _public_command(cmd):
    """对外命令模板：模板字段原样 + currentValue/savedValue 默认值
    （executeCommand 回传时客户端就地填这两键，模板里先给默认 0.0）。"""
    args = []
    for a in cmd.get("arguments", []):
        args.append({"name": a.get("name", ""), "type": a.get("type", ""),
                     "currentValue": 0.0, "savedValue": 0.0})
    return {"name": cmd["name"], "arguments": args}


def _public_observer(obs):
    return {"name": obs["name"], "type": obs.get("type", "")}


def _public_setting(stg):
    return {"name": stg["name"], "type": stg.get("type", "")}


class McsAdapter(object):
    """MCS 五+1 服务 → 语义层翻译核心（与 HTTP 壳分离，便于测试）。"""

    def __init__(self, engine, points, mcs_map=None):
        self.eng = engine
        self.pts = points
        self.map = mcs_map if mcs_map is not None else load_mcs_map()
        # 无绑定 setting 的内存暂存（如 CommandTimeOut）：norm_path -> {名: 值}
        self._setting_mem = {}
        self._lock = threading.Lock()

    # ---------------- 查找 ----------------
    def _entry(self, full_path):
        e = self.map.get(_norm(full_path))
        if e is None:
            raise MCSFault("PathNotFound",
                           "machinePart 未注册: %s" % full_path)
        if e.get("unmapped") or e["kind"] == "unmapped":
            raise MCSFault("Unmapped",
                           "machinePart 在 1928 机台无对应物（%s）: %s"
                           % (e.get("note", ""), full_path))
        return e

    def _axis_name(self, entry):
        """axis_stage 的仿真轴名（取别名解析后的第一根，如 E_EPIN_Z→leader）。"""
        names = ac.resolve_axis(entry["map"]["axes"][0])
        if not names:
            raise MCSFault("Unmapped", "轴别名无法解析: %s"
                           % entry["map"]["axes"][0])
        return names[0]

    # ---------------- 3.1 getMachinePartProperties ----------------
    def get_machine_part_properties(self, input_args):
        path = (input_args[0] or {}).get("machinePartFullPath", "")
        e = self._entry(path)
        return {"machinePart": _machine_part_obj(e),
                "commands": [_public_command(c) for c in e["commands"]],
                "settings": [_public_setting(s) for s in e["settings"]],
                "observers": [_public_observer(o) for o in e["observers"]]}

    # ---------------- 3.2 executeCommand ----------------
    def execute_command(self, input_args):
        cmd = (input_args[0] or {}).get("command") or {}
        name = cmd.get("name", "")
        mp = cmd.get("machinePart") or {}
        full_path = mp.get("fullPath") or mp.get("name") or ""
        e = self._entry(full_path)
        tmpl = next((c for c in e["commands"] if c["name"] == name), None)
        if tmpl is None:
            raise MCSFault("CmdNotFound",
                           "命令 %r 不在节点 %s 的模板内" % (name, full_path))
        values = [a.get("currentValue") for a in cmd.get("arguments", [])]
        try:
            self._dispatch(e, name, values)
        except InterlockError as ex:
            raise MCSFault("Interlock", str(ex))
        except (KeyError, ValueError, RuntimeError) as ex:
            raise MCSFault("ExecutionFailed", str(ex))
        return []           # 客户端只判 value 非 null（LOCAL_DEBUG 桩即 []）

    def _dispatch(self, e, name, values):
        eng = self.eng
        kind = e["kind"]
        if kind == "vacuum":
            op = _VACUUM_OP[name]            # 模板外命令已被 CmdNotFound 拦下
            eng.air_command(e["map"]["circuit"], op)
            return
        if kind == "ws_stage_move":
            axis = e["map"]["cmd_axes"][name]
            eng.move(axis, float(values[0]))
            return
        if kind == "axis_stage":
            axis = e["map"]["axes"][0]       # 别名（E_EPIN_Z）由 engine 内部展开
            if name == "Enable":
                eng.enable(axis, True)
            elif name == "Disable":
                eng.enable(axis, False)
            elif name == "Home":
                eng.home(axis)
            elif name == "MoveAbs":
                eng.move(axis, float(values[0]))
            elif name == "MoveRel":
                with eng.lock:
                    cur = eng.axes[self._axis_name(e)].pos
                eng.move(axis, cur + float(values[0]))
            elif name == "Halt":
                eng.stop(axis)
            elif name == "Reset Error":
                eng.reset_error(axis)    # W2 起按 stage 复位（原全局 reset）
            elif name == "SetMoveParam":
                eng.set_axis_config(axis, vmax=float(values[0]),
                                    amax=float(values[1]), persist=False)
            return
        raise MCSFault("Unmapped", "kind %r 无命令面" % kind)

    # ---------------- 3.3 getObserverValuesFromObserverPaths ----------------
    def get_observer_values(self, input_args):
        paths = ((input_args[0] or {}).get("observerPaths")) or []
        out = []
        for item in paths:
            raw = (item or {}).get("path", "")
            # 观测 path = fullPath + "/" + observer名（客户端恒用 / 拼接）
            mp_raw, _, obs_name = raw.rpartition("/")
            e = self._entry(mp_raw)
            obs = next((o for o in e["observers"] if o["name"] == obs_name),
                       None)
            if obs is None:
                raise MCSFault("ObserverNotFound",
                               "观测 %r 不在节点 %s 的模板内" % (obs_name, mp_raw))
            out.append({"currentValue": self._observe(e, obs_name)})
        return out

    def _observe(self, e, obs_name):
        kind = e["kind"]
        if kind == "axis_stage":
            ax = self.eng.axes[self._axis_name(e)]
            with self.eng.lock:
                if obs_name == "oCurrentPos_GC":
                    return round(ax.pos, 6)
                if obs_name == "obMoveSpd":
                    return round(ax.vel, 6)
                if obs_name == "oMotorEnabled":
                    return bool(ax.enabled)
                if obs_name == "oMoving":
                    return bool(ax.moving)
                if obs_name == "oInPos":
                    return bool(ax.in_position)
        elif kind == "vacuum":
            # 读点位表气路状态叶（继承 PLC 扫描周期语义）
            return bool(self.pts.get(
                AIR_BASE + e["map"]["circuit"] + "." + _AIRROUTE_POINT[obs_name]))
        elif kind == "di_presence":
            return bool(self._presence(e["map"]["observers"][obs_name]))
        elif kind == "ws_stage_move":
            axis = e["map"]["observers"][obs_name]
            with self.eng.lock:
                return round(self.eng.axes[axis].target, 6)
        raise MCSFault("ObserverNotFound", "kind %r 无观测 %r" % (kind, obs_name))

    def _presence(self, what):
        """di_presence 观测 → workpieces 工件层（假设映射，见 mcs_map.json）。"""
        wp = self.eng.workpieces
        if wp is None:
            return False
        if what == "wafer_present":
            return bool(wp.wafer["present"])
        carrier = {"die_on_head": "head", "die_on_flip": "flip"}.get(what)
        if carrier:
            return any(d["carrier"] == carrier for d in list(wp.dies))
        return False

    # ---------------- 3.4 getSettings（schema 推断，见协议文档 §3.4） ----------------
    def get_settings(self, input_args):
        path = (input_args[0] or {}).get("machinePartFullPath", "")
        e = self._entry(path)
        return [dict(_public_setting(s), **self._setting_typed_value(e, s))
                for s in e["settings"]]

    def _setting_typed_value(self, e, stg):
        """设置当前值（typed value 形式）：绑轴参数的读 engine，无绑定的读内存。"""
        t = stg.get("type", "FLOAT_TYPED_VALUE")
        bind = stg.get("bind")
        if bind in ("vmax", "amax"):
            with self.eng.lock:
                v = getattr(self.eng.axes[self._axis_name(e)], bind)
        else:
            with self._lock:
                v = self._setting_mem.get(_norm(e["fullPath"]), {}).get(
                    stg["name"], stg.get("default", 0.0))
        key = "booleanValue" if t == "BOOLEAN_TYPED_VALUE" else "floatValue"
        return {key: v}

    # ---------------- 3.5 setCurrentValueForSetting ----------------
    def set_current_value_for_setting(self, input_args):
        arg = input_args[0] or {}
        key = arg.get("settingKey") or {}
        typed = arg.get("value") or {}
        e = self._entry(key.get("machinePartFullPath", ""))
        sname = key.get("settingName", "")
        stg = next((s for s in e["settings"] if s["name"] == sname), None)
        if stg is None:
            raise MCSFault("SettingNotFound",
                           "设置 %r 不在节点模板内" % sname)
        # 类型校验：BOOLEAN 设置只收 booleanValue，其余收 floatValue/intValue
        t = stg.get("type", "FLOAT_TYPED_VALUE")
        if t == "BOOLEAN_TYPED_VALUE":
            if "booleanValue" not in typed:
                raise MCSFault("TypeMismatch", "设置 %s 需要 booleanValue" % sname)
            val = bool(typed["booleanValue"])
        else:
            num = typed.get("floatValue", typed.get("intValue"))
            if num is None:
                raise MCSFault("TypeMismatch", "设置 %s 需要 floatValue" % sname)
            val = float(num)
        bind = stg.get("bind")
        axis_alias = e["map"]["axes"][0]    # 用别名（E_EPIN_Z）整组生效
        try:
            if bind == "vmax":
                self.eng.set_axis_config(axis_alias, vmax=val, persist=False)
            elif bind == "amax":
                self.eng.set_axis_config(axis_alias, amax=val, persist=False)
            else:    # 无绑定（CommandTimeOut 等）：内存暂存
                with self._lock:
                    self._setting_mem.setdefault(
                        _norm(e["fullPath"]), {})[sname] = val
        except (KeyError, ValueError) as ex:
            raise MCSFault("ExecutionFailed", str(ex))
        return []

    # ---------------- 3.6 saveCurrentValueForSetting ----------------
    def save_current_value_for_setting(self, input_args):
        key = (input_args[0] or {}).get("settingKey") or {}
        e = self._entry(key.get("machinePartFullPath", ""))
        sname = key.get("settingName", "")
        stg = next((s for s in e["settings"] if s["name"] == sname), None)
        if stg is None:
            raise MCSFault("SettingNotFound", "设置 %r 不在节点模板内" % sname)
        # 绑轴参数的设置：save = 把当前值持久化进 config_overrides.json
        bind = stg.get("bind")
        if bind in ("vmax", "amax"):
            with self.eng.lock:
                cur = getattr(self.eng.axes[self._axis_name(e)], bind)
            try:
                # 用别名整组持久化（E_EPIN_Z → 三根顶针轴）
                self.eng.set_axis_config(e["map"]["axes"][0], persist=True,
                                         **{bind: cur})
            except (KeyError, ValueError) as ex:
                raise MCSFault("ExecutionFailed", str(ex))
        return []           # 无绑定设置：无副作用，直接成功


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):   # 静音
        pass

    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _await_silent_lifted(self):
        """沉默模式（W2）：mcs 沉默期间请求挂起不响应（复现真机"发了不回"），
        50ms 轮询等待沉默解除；60s 兜底返回 False（调用方回 503，防线程泄漏）。"""
        eng = self.server.adapter.eng
        t0 = time.time()
        while eng.is_silent("mcs"):
            if time.time() - t0 > 60.0:
                return False
            time.sleep(0.05)
        return True

    # ---------- GET（调试辅助，非真机面） ----------
    def do_GET(self):
        ad = self.server.adapter
        u = urlparse(self.path)
        if not self._await_silent_lifted():
            return self._send(503, {"type": "FAULT", "value": None,
                                    "error": {"code": "Silent",
                                              "message": "mcs silent"
                                                         "（故障注入），60s 兜底"}})
        try:
            if u.path == "/health":
                h = ad.eng.health()
                h["mcs_parts"] = len(ad.map)
                h["mcs_port"] = self.server.server_address[1]
                return self._send(200, h)
            if u.path == "/mcs/map":
                return self._send(200, {"ok": True, "count": len(ad.map),
                                        "paths": sorted(ad.map)})
            return self._send(404, {"ok": False, "error": "unknown GET " + u.path})
        except Exception as e:
            return self._send(500, {"ok": False, "error": str(e)})

    # ---------- POST（真机 MCS 面） ----------
    def do_POST(self):
        ad = self.server.adapter
        u = urlparse(self.path)
        if not u.path.startswith(MCS_SERVICE_PATH):
            return self._send(404, {"type": "FAULT", "value": None,
                                    "error": {"code": "UnknownService",
                                              "message": "unknown POST "
                                                         + u.path}})
        if not self._await_silent_lifted():
            return self._send(503, {"type": "FAULT", "value": None,
                                    "error": {"code": "Silent",
                                              "message": "mcs silent"
                                                         "（故障注入），60s 兜底"}})
        service = u.path[len(MCS_SERVICE_PATH):]
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n).decode("utf-8") or "null")
            input_args = self._parse_envelope(body)
        except MCSFault as e:
            # 信封/JSON 结构错：HTTP 400 + FAULT body
            return self._send(400, {"type": "FAULT", "value": None,
                                    "error": {"code": e.code,
                                              "message": e.message}})
        handler = {"getMachinePartProperties": ad.get_machine_part_properties,
                   "executeCommand": ad.execute_command,
                   "getObserverValuesFromObserverPaths": ad.get_observer_values,
                   "getSettings": ad.get_settings,
                   "setCurrentValueForSetting": ad.set_current_value_for_setting,
                   "saveCurrentValueForSetting":
                       ad.save_current_value_for_setting}.get(service)
        if handler is None:
            return self._send(404, {"type": "FAULT", "value": None,
                                    "error": {"code": "UnknownService",
                                              "message": "unknown service "
                                                         + service}})
        try:
            return self._send(200, {"type": "SUCCESS",
                                    "value": handler(input_args)})
        except MCSFault as e:
            # 真机式业务错误：HTTP 200 + FAULT body（客户端只解析 body）
            return self._send(200, {"type": "FAULT", "value": None,
                                    "error": {"code": e.code,
                                              "message": e.message}})
        except Exception as e:
            return self._send(200, {"type": "FAULT", "value": None,
                                    "error": {"code": "InternalError",
                                              "message": str(e)}})

    @staticmethod
    def _parse_envelope(body):
        """请求信封：[{"SecurityToken":"ADMIN"},{"InputArgs":[...]}]。
        宽容解析：按"含 InputArgs 键的数组元素"取参，不校验令牌。"""
        if not isinstance(body, list):
            raise MCSFault("BadEnvelope", "请求体须为 JSON 数组信封")
        for el in body:
            if isinstance(el, dict) and "InputArgs" in el:
                args = el["InputArgs"]
                if not isinstance(args, list) or not args:
                    raise MCSFault("BadEnvelope", "InputArgs 须为非空数组")
                return args
        raise MCSFault("BadEnvelope", "信封缺 InputArgs 元素")


class _HTTPServer(ThreadingHTTPServer):
    allow_reuse_address = False     # 见 ws_gateway：防 Windows 双绑定
    daemon_threads = True


def start_mcs_server(engine, points, host="127.0.0.1", port=9190,
                     map_path=MCS_MAP_JSON):
    adapter = McsAdapter(engine, points, load_mcs_map(map_path))
    srv = _HTTPServer((host, port), _Handler)
    srv.adapter = adapter
    t = threading.Thread(target=srv.serve_forever, name="mcs-http", daemon=True)
    t.start()
    n_unmapped = sum(1 for e in adapter.map.values() if e.get("unmapped"))
    print("[mcs] Cordis MCS adapter listening on %s:%d (machineParts=%d, "
          "unmapped=%d)" % (host, port, len(adapter.map), n_unmapped),
          flush=True)
    return srv
