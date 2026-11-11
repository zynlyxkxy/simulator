# -*- coding: utf-8 -*-
"""acs_tcp.py — ACS 运动控制器模拟器（TCP 9101，换行分隔 JSON）。

每行一个 JSON 指令，每个指令回一行 JSON 响应。
沉默模式（W2 故障注入，engine.is_silent("acs")）：连接保持但指令吞掉
不回帧不执行；断线模拟由 main 的故障钩子 shutdown+server_close+断开
已有连接实现（restore 重建服务）。
支持的 cmd（大小写不敏感）：
  enable / disable        {"cmd":"enable","axis":"WS_LS_X"|"ALL"}
  reset_error             {"cmd":"reset_error","axis":"WS_LS_X"|"ALL"}  # W3 补：清 fault 不改 enabled
  home                    {"cmd":"home","axis":"WS_LS_X","mode":17|18}  # 17负/18正
  move / ptp              {"cmd":"move","axis":"WS_LS_X","target":100.0,"speed":200.0}
  stop                    {"cmd":"stop","axis":"WS_LS_X"|"ALL"}
  estop / reset
  getpos                  {"cmd":"getpos","axis":"WS_LS_X"|"ALL"}
  status                  各轴 enabled/homed/moving/in_position
  SetBufferNumber / RunBuffer / StopBuffer / ResumeBuffer
  getBuffer               -> {"oBufferDo":bool,"oBufferNum":int}
"""
import json
import socket
import socketserver
import threading

import axes_config as ac

ACS_AXES = {row[0] for row in ac.AXES if row[5] == "ACS"}


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        eng = self.server.engine
        self.server.track_conn(self.connection, True)
        try:
            while True:
                line = self.rfile.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                if eng.is_silent("acs"):
                    # 沉默（模拟固件挂死）：连接保持，指令吞掉不回帧不执行；
                    # 上位机只能靠自己超时/看门狗发现
                    continue
                try:
                    req = json.loads(line.decode("utf-8"))
                    resp = self.dispatch(eng, req)
                except Exception as e:
                    resp = {"ok": False, "error": str(e)}
                self.wfile.write((json.dumps(resp, ensure_ascii=False) + "\n")
                                 .encode("utf-8"))
                self.wfile.flush()
        finally:
            self.server.track_conn(self.connection, False)

    def dispatch(self, eng, req):
        cmd = str(req.get("cmd", "")).lower()
        axis = req.get("axis")

        def check_acs(names):
            for n in names:
                if n not in ACS_AXES:
                    raise KeyError("axis %s is not on ACS controller" % n)

        if cmd in ("enable", "disable"):
            eng.enable(axis or "ALL", on=(cmd == "enable"))
            return {"ok": True}
        if cmd == "reset_error":
            # W2 状态机对齐：清 fault / 回零 fault 态，不改 enabled（真机 ResetError 语义）
            names = ac.resolve_axis(axis) if axis not in (None, "ALL") else list(ACS_AXES)
            if not names:
                raise KeyError("unknown axis: %r" % axis)
            check_acs(names)
            eng.reset_error(axis or "ALL")
            return {"ok": True}
        if cmd == "home":
            names = ac.resolve_axis(axis) if axis != "ALL" else list(ACS_AXES)
            if not names:
                raise KeyError("unknown axis: %r" % axis)
            check_acs(names)
            eng.home(axis, mode=req.get("mode"))
            return {"ok": True}
        if cmd in ("move", "ptp"):
            names = ac.resolve_axis(axis)
            if not names:
                raise KeyError("unknown axis: %r" % axis)
            check_acs(names)
            dur = eng.move(axis, req["target"], req.get("speed"))
            return {"ok": True, "duration": round(dur, 4)}
        if cmd == "stop":
            eng.stop(axis or "ALL")
            return {"ok": True}
        if cmd == "estop":
            eng.estop()
            return {"ok": True}
        if cmd == "reset":
            eng.reset()
            return {"ok": True}
        if cmd == "getpos":
            with eng.lock:
                if axis and axis != "ALL":
                    names = ac.resolve_axis(axis)
                    if not names:
                        raise KeyError("unknown axis: %r" % axis)
                    return {"ok": True,
                            "pos": {n: round(eng.axes[n].pos, 6) for n in names}}
                return {"ok": True, "pos": {n: round(a.pos, 6)
                                            for n, a in eng.axes.items()
                                            if n in ACS_AXES}}
        if cmd == "status":
            with eng.lock:
                return {"ok": True, "estop": eng.estopped, "axes": {
                    n: {"enabled": a.enabled, "fault": a.fault,
                        "homed": a.homed,
                        "moving": a.moving, "in_position": a.in_position,
                        "pos": round(a.pos, 6)}
                    for n, a in eng.axes.items() if n in ACS_AXES}}
        if cmd == "setbuffernumber":
            eng.set_buffer(req.get("buffer", req.get("num")))
            return {"ok": True, "buffer": eng.buf_num}
        if cmd == "runbuffer":
            eng.run_buffer(req.get("buffer"))
            return {"ok": True}
        if cmd == "stopbuffer":
            eng.stop_buffer()
            return {"ok": True}
        if cmd == "resumebuffer":
            eng.resume_buffer()
            return {"ok": True}
        if cmd == "getbuffer":
            st = eng.buffer_state()
            st["ok"] = True
            return st
        if cmd == "ping":
            return {"ok": True, "pong": True}
        raise ValueError("unknown cmd: %r" % req.get("cmd"))


class _Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._conns = set()         # 活动连接（断线模拟时强制断开）
        self._conns_lock = threading.Lock()

    def track_conn(self, conn, add):
        with self._conns_lock:
            (self._conns.add if add else self._conns.discard)(conn)

    def close_connections(self):
        """断线模拟：强制断开所有已建连接（真机拔线已有连接也会断）。"""
        with self._conns_lock:
            conns = list(self._conns)
        for c in conns:
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                c.close()
            except OSError:
                pass


def start_acs_server(engine, host="127.0.0.1", port=9101):
    srv = _Server((host, port), _Handler)
    srv.engine = engine
    import threading
    t = threading.Thread(target=srv.serve_forever, name="acs-tcp",
                         daemon=True)
    t.start()
    print("[acs] TCP simulator listening on %s:%d" % (host, port), flush=True)
    return srv
