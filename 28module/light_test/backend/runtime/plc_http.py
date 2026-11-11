# -*- coding: utf-8 -*-
"""plc_http.py — 倍福 PLC 模拟器（HTTP 9102）。

  POST /io/command   {"path":"SystemMachine...","cmd":"OpenNegtive","args":{...}}
  GET  /io/observe?path=SystemMachine...   （path 为前缀时返回整棵子树）
  GET  /io/snapshot                        （全部点位）

命令路由（plc_command，按序匹配）：
  气路    cmd ∈ {OpenPositive, OpenNegtive, OpenAir}
  温控    path 含 PulseHeat_Up / Heat_Dw：
          SetTargetTempWithPlan(Steady_Time, Temperature) / StopTargetTempWithPlan / StartPreHeat
          （cmd 亦可为 SetValue/Pulse，此时以 path 末段为真实命令，value 当 Temperature）
  塔灯    E_SignalTower.<色|Buzzer>.UpdateMode(FlashOnPeriod, FlashOffPeriod)
          （args 字典或位置参数数组均可）；SetBool 到 base 路径 = 手动点灯/灭灯
  电源    M_PowerSwitch.<Name>_PowerOn/_PowerOff（Pulse/SetBool 均锁存状态位），
          Fan/IonBar 用 SetBool 到 base 名；断电分系统的轴 move 报 409
  互锁    M_InterLock.{AllDoors_Reset, AllDoors_Mask, InterLock_Left/Front/Right}
  相机    M_AS.{AlignCameraTigger_1/2, AS_CameraOn} 触发 → 拍照记录
          （GET 9010 /api/camera/captures 回读）
  光源    M_AS.{LightOn_1/2, AS_LightOn} SetBool → o* 观测位同步
  校准    M_AS.WaferHeatReset（清加热盘报警）、M_WS.AramPosSensorCalibrate_1/2
  ES      M_ES.{ES_Start/Stop/Reset/EmergencyStop/GvieBackDie/MoveToNextChip}
  使能    叶名 Enable/Disable/ResetError 且父路径末段可解析为轴/stage 别名
          （如 ...M_WS.E_EPIN_Z.Disable）：接引擎使能状态机（W2）
  键合力  cmd=BondingByForceManual，args {"Force":N,"Speed":mm/s}
  buffer  cmd ∈ {SetBufferNumber, RunBuffer, StopBuffer, ResumeBuffer}（PLC 调 ACS buffer）
  备用    path 含 Reserve/Spare/备用/预留 且未匹配上面语义 → ok:true 备注 spare
  其余    通用点位写入：cmd ∈ {SetBool, SetValue, Pulse}，args {"value":...}；
          若同级存在 o<叶名> 观测点则同步镜像

沉默模式（W2）：engine.is_silent("plc") 期间所有请求挂起不响应（50ms 轮询，
60s 兜底 503），复现真机"发了不回"；断线由 main 故障钩子关 listen socket。
"""
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import axes_config as ac
from engine import (AIR_BASE, HEATERS, POWER_BASE, POWER_SUBSYSTEMS,
                    TOWER_BASE, TOWER_ELEMENTS, InterlockError)

AIR_OPS = ("OpenPositive", "OpenNegtive", "OpenAir")
HEATER_CMDS = ("SetTargetTempWithPlan", "StopTargetTempWithPlan", "StartPreHeat")
SPARE_RE = re.compile(r"Reserve|Spare|备用|预留")
INTERLOCK_CMDS = ("AllDoors_Reset", "AllDoors_Mask",
                  "InterLock_Left", "InterLock_Front", "InterLock_Right")
CAMERA_TRIGGERS = ("AlignCameraTigger_1", "AlignCameraTigger_2", "AS_CameraOn")
ES_CMDS = ("ES_Start", "ES_Stop", "ES_Reset", "ES_EmergencyStop",
           "ES_GvieBackDie", "ES_MoveToNextChip")
INTERLOCK_BASE = "SystemMachine.SS_ME.M_InterLock."


def _truthy(args):
    return bool(args.get("value", True))


def _flash_args(args):
    """UpdateMode 参数兼容：字典 {FlashOnPeriod, FlashOffPeriod} 或位置数组。"""
    if isinstance(args, (list, tuple)):
        vals = list(args)
    else:
        vals = [args.get("FlashOnPeriod", args.get("flash_on", 500)),
                args.get("FlashOffPeriod", args.get("flash_off", 500))]
    vals = (vals + [500, 500])[:2]
    return int(vals[0]), int(vals[1])


def _pulse_off(pts, path, width):
    import time
    time.sleep(float(width))
    pts.set(path, False)
    parent, _, leaf = path.rpartition(".")
    mirror = parent + ".o" + leaf
    if pts.has(mirror):
        pts.set(mirror, False)


def plc_command(eng, pts, path, cmd, args):
    """PLC 命令路由（模块级，供 9102 处理器与 9010 网关 /api/io/command 代理共用）。"""
    args = args if args is not None else {}
    leaf = path.rsplit(".", 1)[-1]
    parent = path[:len(path) - len(leaf) - 1] if "." in path else ""

    # 气路
    if cmd in AIR_OPS:
        eng.air_command(path, cmd)
        return {"ok": True, "path": path, "cmd": cmd}
    # 温控（path 含加热器名；cmd 不是加热命令时以 path 末段为准。
    # 只拦截真正的加热命令，其余（如 PreHeatFinish SetBool）落到通用写入）
    for hname in HEATERS:
        if hname in path and (cmd in HEATER_CMDS or leaf in HEATER_CMDS):
            if cmd in HEATER_CMDS:
                eng.heater_command(hname, cmd, args)
            else:
                hargs = dict(args) if isinstance(args, dict) else {}
                if leaf == "SetTargetTempWithPlan" and "Temperature" not in hargs:
                    hargs["Temperature"] = hargs.get("value", 100)
                eng.heater_command(hname, leaf, hargs)
            return {"ok": True, "path": path,
                    "cmd": cmd if cmd in HEATER_CMDS else leaf}
    # 塔灯/蜂鸣器：UpdateMode / SetBool 到 base
    if path.startswith(TOWER_BASE):
        rest = path[len(TOWER_BASE):]
        if rest.endswith(".UpdateMode"):
            elem = rest[:-len(".UpdateMode")]
            on_p, off_p = _flash_args(args)
            r = eng.signal_tower_set(elem, True, flash=(on_p, off_p))
            return {"ok": True, "path": path, **r}
        if rest in TOWER_ELEMENTS and cmd in ("SetBool", "SetValue", "Pulse"):
            r = eng.signal_tower_set(rest, _truthy(args))
            return {"ok": True, "path": path, **r}
    # 电源分系统
    if path.startswith(POWER_BASE):
        pname = path[len(POWER_BASE):]
        if pname.endswith("_PowerOn") and pname[:-8] in POWER_SUBSYSTEMS:
            return {"ok": True, **eng.power_set(pname[:-8], True)}
        if pname.endswith("_PowerOff") and pname[:-9] in POWER_SUBSYSTEMS:
            return {"ok": True, **eng.power_set(pname[:-9], False)}
        if pname in ("Fan", "IonBar") and \
                cmd in ("SetBool", "SetValue", "Pulse"):
            return {"ok": True, **eng.power_set(pname, _truthy(args))}
    # 互锁命令
    if path.startswith(INTERLOCK_BASE) and leaf in INTERLOCK_CMDS:
        return {"ok": True, **eng.interlock_command(leaf, _truthy(args))}
    # 相机触发（SetBool true / Pulse 均产生拍照记录；false 只落观测位）
    if leaf in CAMERA_TRIGGERS:
        if _truthy(args):
            cap = eng.camera_trigger(leaf)
            if cmd == "Pulse":
                obs = "SystemMachine.SS_Bonding.M_AS.o" + leaf
                w = args.get("width", 0.5) if isinstance(args, dict) else 0.5
                threading.Thread(target=_pulse_off,
                                 args=(pts, obs, w), daemon=True).start()
            return {"ok": True, "path": path, "capture_id": cap["id"]}
        obs = "SystemMachine.SS_Bonding.M_AS.o" + leaf
        if pts.has(obs):
            pts.set(obs, False)
        return {"ok": True, "path": path, "value": False}
    # 加热复位 / 位置传感器校准
    if leaf == "WaferHeatReset":
        return {"ok": True, **eng.wafer_heat_reset()}
    if leaf in ("AramPosSensorCalibrate_1", "AramPosSensorCalibrate_2"):
        r = eng.aram_calibrate()
        if leaf.endswith("_2"):
            r["note"] = "点位表 H55/H56 同点，_2 映射到 _1"
        return {"ok": True, **r}
    # ES 流程命令（kind=null，cmd 常为 Pulse 或命令名本身）
    if leaf in ES_CMDS:
        return {"ok": True, **eng.es_command(leaf)}
    if cmd in ES_CMDS:
        return {"ok": True, **eng.es_command(cmd)}
    # 轴/stage 使能命令（W2）：父路径末段可解析为轴名/别名（E_EPIN_Z 整组）
    # 才拦截；解析不到则落到通用点位写入（兼容点位表里的普通 Enable 点位）
    if leaf in ("Enable", "Disable", "ResetError"):
        alias = parent.rsplit(".", 1)[-1]
        if ac.resolve_axis(alias):
            if leaf == "Enable":
                eng.enable(alias, True)
            elif leaf == "Disable":
                eng.enable(alias, False)
            else:
                eng.reset_error(alias)
            return {"ok": True, "path": path, "cmd": leaf,
                    "axes": ac.resolve_axis(alias)}
    # 键合力
    if cmd == "BondingByForceManual":
        eng.bonding_force(args.get("Force", 0.0), args.get("Speed", 1.0))
        return {"ok": True, "path": path, "cmd": cmd}
    # buffer（真机里 PLC 调 ACS buffer 跑交接流程）
    if cmd == "SetBufferNumber":
        eng.set_buffer(args.get("BufferNumber", args.get("buffer")))
        return {"ok": True, "buffer": eng.buf_num}
    if cmd == "RunBuffer":
        eng.run_buffer(args.get("BufferNumber"))
        return {"ok": True}
    if cmd == "StopBuffer":
        eng.stop_buffer()
        return {"ok": True}
    if cmd == "ResumeBuffer":
        eng.resume_buffer()
        return {"ok": True}
    # 备用/预留点位：不报错，无副作用
    if SPARE_RE.search(path):
        return {"ok": True, "path": path, "note": "spare"}
    # 通用点位写入（同级 o<叶名> 观测点同步镜像）
    if cmd in ("SetBool", "SetValue", "Pulse"):
        value = args.get("value", True) if isinstance(args, dict) else True
        pts.set(path, value)
        mirror = parent + ".o" + leaf
        mirrored = False
        if mirror != path and pts.has(mirror):
            pts.set(mirror, value)
            mirrored = True
        if cmd == "Pulse":
            w = args.get("width", 0.5) if isinstance(args, dict) else 0.5
            threading.Thread(target=_pulse_off,
                             args=(pts, path, w), daemon=True).start()
        return {"ok": True, "path": path, "value": value, "mirrored": mirrored}
    raise ValueError("cannot route cmd %r for path %r" % (cmd, path))


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):   # 静音
        pass

    # ---------- 工具 ----------
    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _await_silent_lifted(self):
        """沉默模式（W2）：plc 沉默期间请求挂起不响应（复现真机"发了不回"），
        50ms 轮询等待沉默解除；60s 兜底返回 False（调用方回 503，防线程泄漏）。"""
        eng = self.server.engine
        t0 = time.time()
        while eng.is_silent("plc"):
            if time.time() - t0 > 60.0:
                return False
            time.sleep(0.05)
        return True

    # ---------- GET ----------
    def do_GET(self):
        eng, pts = self.server.engine, self.server.points
        u = urlparse(self.path)
        if not self._await_silent_lifted():
            return self._send(503, {"ok": False,
                                    "error": "plc silent（故障注入），60s 兜底"})
        try:
            if u.path == "/io/observe":
                path = parse_qs(u.query).get("path", [""])[0]
                if not path:
                    return self._send(400, {"ok": False, "error": "missing path"})
                sub = pts.subtree(path)
                if not sub:
                    return self._send(404, {"ok": False,
                                            "error": "no points under %r" % path})
                if len(sub) == 1 and path in sub:
                    return self._send(200, {"ok": True, "path": path,
                                            "value": sub[path]})
                return self._send(200, {"ok": True, "path": path, "values": sub})
            if u.path == "/io/snapshot":
                return self._send(200, {"ok": True, "count": len(pts),
                                        "points": pts.snapshot()})
            if u.path == "/health":
                return self._send(200, eng.health())
            return self._send(404, {"ok": False, "error": "unknown GET " + u.path})
        except Exception as e:
            return self._send(500, {"ok": False, "error": str(e)})

    # ---------- POST ----------
    def do_POST(self):
        eng, pts = self.server.engine, self.server.points
        u = urlparse(self.path)
        if u.path != "/io/command":
            return self._send(404, {"ok": False, "error": "unknown POST " + u.path})
        if not self._await_silent_lifted():
            return self._send(503, {"ok": False,
                                    "error": "plc silent（故障注入），60s 兜底"})
        try:
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            path = req.get("path", "")
            cmd = req.get("cmd", "")
            args = req.get("args") or {}
            return self._send(200, plc_command(eng, pts, path, cmd, args))
        except InterlockError as e:
            # 联锁/掉使能/故障拒绝（与 9010 一致的 409 模式）
            return self._send(409, {"ok": False, "error": str(e)})
        except Exception as e:
            return self._send(400, {"ok": False, "error": str(e)})


class _HTTPServer(ThreadingHTTPServer):
    allow_reuse_address = False     # 见 ws_gateway：防 Windows 双绑定
    daemon_threads = True


def start_plc_server(engine, points, host="127.0.0.1", port=9102):
    srv = _HTTPServer((host, port), _Handler)
    srv.engine = engine
    srv.points = points
    t = threading.Thread(target=srv.serve_forever, name="plc-http", daemon=True)
    t.start()
    print("[plc] HTTP simulator listening on %s:%d" % (host, port), flush=True)
    return srv
