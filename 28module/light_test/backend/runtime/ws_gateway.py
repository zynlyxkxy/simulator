# -*- coding: utf-8 -*-
"""ws_gateway.py — WS 网关 + HTTP 调试口（同端口 9010，纯标准库 RFC6455）。

- ws://127.0.0.1:9010/ws  服务端→客户端二进制帧（opcode 0x2，不掩码，108B）；
  客户端→服务端文本 JSON 指令（掩码帧，按协议 §4 路由）。
- GET /api/axes /api/io /api/health。
握手：SHA1+base64；客户端帧解析：掩码/扩展长度/ping/close。
"""
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import threading
import time as time_mod

import axes_config as ac
from calibration import CalibBusy
from engine import InterlockError
from flow_engine import FlowStateError

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
PANEL_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "panel"))

# ---- PyQt5 测校软件拉起（迭代 27）：POST /api/launch/metrology_gui ----
# 迭代 41 起环境内置：解释器首选项目自带 env/python（含 PyQt5，
# AGENTS.md 例外条款）；入口 = 项目内 metrology_gui/main.py（MQTT 线协议连本后端）
def _mgui_python():
    # 第一候选 = 项目内置 env/python/python.exe；旧 sop_tools/_tools venv
    # 候选+glob 仅留作历史容错兜底（迭代 41 前的外部 venv 可能仍在），
    # 与 metrology_gui/_qt_bootstrap.find_venv_python 同一解析顺序（迭代 30 统一）
    env_py = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "env", "python", "python.exe"))
    if os.path.isfile(env_py):
        return env_py
    base = r"C:\Users\e-hao.wu\Desktop\3000测校SOP_u"
    for sub in (r"\sop_tools\.venv\Scripts\python.exe",
                r"\_tools\.venv\Scripts\python.exe"):
        p = os.path.normpath(base + sub)
        if os.path.isfile(p):
            return p
    import glob
    hits = sorted(glob.glob(base + r"\*\.venv\Scripts\python.exe") +
                  glob.glob(base + r"\*\.venv\Scripts\python.exe"))
    if hits:
        return hits[0]
    return env_py


MGUI_PYTHON = _mgui_python()
MGUI_MAIN = os.path.join(os.path.dirname(PANEL_DIR),
                         "metrology_gui", "main.py")
_mgui_lock = threading.Lock()
_mgui_proc = None


def launch_metrology_gui(req):
    """POST /api/launch/metrology_gui 处理 -> (code, obj)。
    已拉起且存活 → {ok, already:true, pid}；req.dry=true 或环境变量
    D2W_MGUI_DRYRUN=1 → 只校验路径返回 {ok, dry:true}（CI 无窗测试用，
    不起进程）；venv/入口缺失或拉起失败 → 500 + 明确错误。纯标准库。"""
    req = req if isinstance(req, dict) else {}
    dry = bool(req.get("dry")) or os.environ.get("D2W_MGUI_DRYRUN") == "1"
    if not os.path.isfile(MGUI_PYTHON):
        return 500, {"ok": False,
                     "error": "PyQt5 venv 解释器不存在: %s" % MGUI_PYTHON}
    if not os.path.isfile(MGUI_MAIN):
        return 500, {"ok": False,
                     "error": "metrology_gui 入口不存在: %s" % MGUI_MAIN}
    global _mgui_proc
    with _mgui_lock:
        if _mgui_proc is not None and _mgui_proc.poll() is None:
            return 200, {"ok": True, "already": True, "pid": _mgui_proc.pid}
        if dry:
            return 200, {"ok": True, "dry": True}
        flags = 0
        if os.name == "nt":     # 脱离本进程控制台，后端退出不带走 GUI
            flags = (subprocess.DETACHED_PROCESS
                     | subprocess.CREATE_NEW_PROCESS_GROUP)
        try:
            _mgui_proc = subprocess.Popen([MGUI_PYTHON, MGUI_MAIN],
                                          creationflags=flags,
                                          close_fds=True)
        except OSError as e:
            _mgui_proc = None
            return 500, {"ok": False, "error": "PyQt5 测校软件拉起失败: %s"
                                               % e}
        return 200, {"ok": True, "pid": _mgui_proc.pid}


def _read_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("eof")
        buf += chunk
    return buf


def _parse_headers(conn):
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            raise ConnectionError("eof before headers")
        data += chunk
        if len(data) > 65536:
            raise ValueError("headers too large")
    head, _, rest = data.partition(b"\r\n\r\n")
    lines = head.decode("latin1").split("\r\n")
    method, path, _ver = lines[0].split(" ", 2)
    headers = {}
    for ln in lines[1:]:
        if ":" in ln:
            k, v = ln.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    return method, path, headers, rest


def ws_accept_key(key):
    return base64.b64encode(
        hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")


def build_server_frame(opcode, payload):
    """服务端帧：不掩码。"""
    n = len(payload)
    b1 = 0x80 | opcode
    if n < 126:
        return struct.pack("!BB", b1, n) + payload
    if n <= 0xFFFF:
        return struct.pack("!BBH", b1, 126, n) + payload
    return struct.pack("!BBQ", b1, 127, n) + payload


def read_client_frame(conn):
    """解析一个客户端帧 -> (opcode, payload)。处理掩码与扩展长度。"""
    b1, b2 = _read_exact(conn, 2)
    opcode = b1 & 0x0F
    masked = b2 & 0x80
    ln = b2 & 0x7F
    if ln == 126:
        ln = struct.unpack("!H", _read_exact(conn, 2))[0]
    elif ln == 127:
        ln = struct.unpack("!Q", _read_exact(conn, 8))[0]
    mask = _read_exact(conn, 4) if masked else None
    payload = _read_exact(conn, ln) if ln else b""
    if mask:
        payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
    return opcode, payload


class _Client(object):
    def __init__(self, conn, addr):
        self.conn = conn
        self.addr = addr
        self.alive = True
        self.dropped = 0


class WSGateway(object):
    def __init__(self, engine, points, host="127.0.0.1", port=9010):
        self.engine = engine
        self.points = points
        self.host = host
        self.port = port
        self._clients = []
        self._clients_lock = threading.Lock()
        self._srv = None
        self.send_avg_us = 0.0
        self.send_max_us = 0.0

    # ---------------- 广播 ----------------
    def broadcast(self, frame):
        """由物理线程直接调用：sendall 112B 到 loopback 通常 <50us，
        不设发送线程/队列（实测 Windows 下线程交接延迟 5~25ms，不可接受）。
        发送超时 50ms 只在客户端僵死时触发，一次后剔除该客户端。"""
        full = build_server_frame(0x2, frame)
        dead = []
        with self._clients_lock:
            clients = list(self._clients)
        for c in clients:
            try:
                t0 = time_mod.perf_counter()
                c.conn.sendall(full)
                dt_us = (time_mod.perf_counter() - t0) * 1e6
                self.send_avg_us = 0.95 * self.send_avg_us + 0.05 * dt_us
                if dt_us > self.send_max_us:
                    self.send_max_us = dt_us
            except (OSError, socket.timeout):
                c.alive = False
                dead.append(c)
        if dead:
            with self._clients_lock:
                for c in dead:
                    if c in self._clients:
                        self._clients.remove(c)

    def client_count(self):
        with self._clients_lock:
            return len(self._clients)

    # ---------------- 服务 ----------------
    def start(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 不设 SO_REUSEADDR：Windows 上它会允许重复绑定同端口，
        # 新旧两个 runtime 并存时连接被随机分流，排查极坑
        srv.bind((self.host, self.port))
        srv.listen(64)
        self._srv = srv
        threading.Thread(target=self._accept_loop, name="ws-accept",
                         daemon=True).start()
        print("[gw] WS gateway + HTTP on %s:%d (ws path /ws)"
              % (self.host, self.port), flush=True)
        return self

    def _accept_loop(self):
        while True:
            try:
                conn, addr = self._srv.accept()
            except OSError:
                break
            threading.Thread(target=self._handle_conn, args=(conn, addr),
                             daemon=True).start()

    def _handle_conn(self, conn, addr):
        conn.settimeout(10)
        try:
            method, path, headers, rest = _parse_headers(conn)
            if "upgrade" in headers.get("connection", "").lower() and \
               headers.get("upgrade", "").lower() == "websocket":
                self._ws_session(conn, addr, path, headers)
            else:
                self._http_request(conn, method, path, headers, rest)
        except (ConnectionError, ValueError, socket.timeout, OSError):
            try:
                conn.close()
            except OSError:
                pass

    # ---------------- HTTP ----------------
    def _send_json(self, conn, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        resp = ("HTTP/1.1 %d OK\r\nContent-Type: application/json; charset=utf-8\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Access-Control-Allow-Methods: GET, POST, PUT, OPTIONS\r\n"
                "Access-Control-Allow-Headers: Content-Type\r\n"
                "Content-Length: %d\r\nConnection: close\r\n\r\n"
                % (code, len(body))).encode("latin1") + body
        conn.sendall(resp)
        conn.close()

    def _send_file(self, conn, path):
        ctype = {".html": "text/html; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8",
                 ".json": "application/json; charset=utf-8",
                 ".png": "image/png", ".svg": "image/svg+xml",
                 ".ico": "image/x-icon"}.get(
                     os.path.splitext(path)[1].lower(),
                     "application/octet-stream")
        with open(path, "rb") as f:
            body = f.read()
        resp = ("HTTP/1.1 200 OK\r\nContent-Type: %s\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                # 迭代 37：panel 不缓存——旧 index.html 被浏览器复用会导致
                # 新 backend + 旧页面轮询逻辑不匹配（温控压控红条误报）
                "Cache-Control: no-cache\r\n"
                "Content-Length: %d\r\nConnection: close\r\n\r\n"
                % (ctype, len(body))).encode("latin1") + body
        conn.sendall(resp)
        conn.close()

    def _send_bytes(self, conn, data, ctype):
        resp = ("HTTP/1.1 200 OK\r\nContent-Type: %s\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Content-Length: %d\r\nConnection: close\r\n\r\n"
                % (ctype, len(data))).encode("latin1") + data
        conn.sendall(resp)
        conn.close()

    def _serve_panel(self, conn, path):
        """托管 backend/panel/ 静态文件；防目录穿越。"""
        if path in ("/", "/panel"):
            path = "/panel/index.html"
        rel = path[len("/panel"):].lstrip("/")
        base = os.path.realpath(PANEL_DIR)
        full = os.path.realpath(os.path.join(base, rel))
        if not full.startswith(base + os.sep):
            return self._send_json(conn, 403, {"ok": False, "error": "forbidden"})
        if not os.path.isfile(full):
            return self._send_json(conn, 404, {
                "ok": False,
                "error": "panel file not found: %s（面板页尚未就位）" % rel})
        self._send_file(conn, full)

    def _read_body(self, conn, headers, rest):
        n = int(headers.get("content-length", 0) or 0)
        if n > 1024 * 1024:
            raise ValueError("body too large")
        body = rest
        while len(body) < n:
            chunk = conn.recv(min(65536, n - len(body)))
            if not chunk:
                break
            body += chunk
        return json.loads(body[:n].decode("utf-8") or "{}")

    def _http_request(self, conn, method, path, headers, rest):
        eng = self.engine
        try:
            if method == "OPTIONS":
                return self._send_json(conn, 200, {"ok": True})
            if method == "GET" and (path == "/" or path.startswith("/panel")):
                return self._serve_panel(conn, path.split("?", 1)[0])
            if method == "GET" and path == "/api/axes":
                return self._send_json(conn, 200, eng.snapshot_axes())
            if method == "GET" and path == "/api/io":
                return self._send_json(conn, 200, {
                    "ok": True, "count": len(self.points),
                    "points": self.points.snapshot()})
            if method == "GET" and path == "/api/health":
                obj = eng.health(clients=self.client_count())
                obj["ws_send_avg_us"] = round(self.send_avg_us, 1)
                obj["ws_send_max_us"] = round(self.send_max_us, 1)
                return self._send_json(conn, 200, obj)
            if method == "GET" and path == "/api/fault":
                return self._send_json(conn, 200, eng.fault_state())
            if method == "GET" and path == "/api/collision":
                return self._send_json(conn, 200, eng.collision_state())
            if method == "GET" and path == "/api/errormaps":
                if eng.errors is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "error system not attached"})
                info = eng.errors.maps_info()
                if eng.kinematics is not None:
                    info["six_dof_enabled"] = eng.kinematics.six_dof_enabled
                return self._send_json(conn, 200, info)
            if method == "GET" and path == "/api/toolpoints":
                if eng.kinematics is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "kinematics not attached"})
                with eng.lock:
                    positions = {n: eng.axes[n].pos for n in ac.AXIS_NAMES}
                return self._send_json(
                    conn, 200, eng.kinematics.compute(positions))
            if method == "GET" and path == "/api/kinematics":
                if eng.kinematics is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "kinematics not attached"})
                return self._send_json(conn, 200,
                                       eng.kinematics.config_public())
            if method == "PUT" and path == "/api/kinematics":
                if eng.kinematics is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "kinematics not attached"})
                req = self._read_body(conn, headers, rest)
                errors = eng.kinematics.update(req)
                if errors:
                    return self._send_json(conn, 400, {
                        "ok": False, "error": "kinematics 校验失败（%d 条）"
                                              % len(errors),
                        "errors": errors})
                if req.get("persist"):
                    # 写回 error_kinematics.json（ValueError→400 由上层映射）
                    eng.kinematics.persist()
                    return self._send_json(conn, 200, {"ok": True,
                                                       "persisted": True})
                return self._send_json(conn, 200, {"ok": True})
            if method == "GET" and path == "/api/viz":
                return self._send_json(conn, 200, eng.viz())
            if method == "GET" and path.split("?", 1)[0] == "/api/camera/captures":
                q = path.split("?", 1)[1] if "?" in path else ""
                try:
                    limit = int([p.split("=", 1)[1] for p in q.split("&")
                                 if p.startswith("limit=")][0])
                except (IndexError, ValueError):
                    limit = 50
                return self._send_json(conn, 200,
                                       {"ok": True,
                                        "captures": eng.camera_captures(limit)})
            # ---- 视觉系统（vision_service，MQTT 出口 1883 / TCP 帧口 9200） ----
            if method == "GET" and path == "/api/vision/cameras":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                return self._send_json(conn, 200, eng.vision.cameras_public())
            if method == "GET" and path.split("?", 1)[0] == "/api/vision/frame":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                cam = [p.split("=", 1)[1] for p in q.split("&")
                       if p.startswith("cam=")]
                # 迭代 20：ir_overlay=1 红外套刻通道帧（仅下相机有意义）
                ir = any(p in ("ir_overlay=1", "ir_overlay=true")
                         for p in q.split("&"))
                try:
                    png = eng.vision.frame_png(cam[0] if cam else "down",
                                               ir_overlay=ir)
                except RuntimeError as e:
                    return self._send_json(conn, 400, {"ok": False,
                                                       "error": str(e)})
                return self._send_bytes(conn, png, "image/png")
            if method == "GET" and path.split("?", 1)[0] == "/api/vision/solve_pixel":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                qs = {}
                for p in q.split("&"):
                    if "=" in p:
                        _k, _v = p.split("=", 1)
                        qs[_k] = _v
                try:
                    cam_id = qs.get("cam") or "down"
                    col = float(qs["col"])
                    row = float(qs["row"])
                except (KeyError, ValueError):
                    return self._send_json(conn, 400, {
                        "ok": False,
                        "error": "col/row 须为数值（query: cam/col/row）"})
                try:
                    return self._send_json(
                        conn, 200,
                        eng.vision.solve_pixel_public(cam_id, col, row))
                except KeyError:
                    return self._send_json(conn, 400, {
                        "ok": False, "error": "unknown cam: %r" % cam_id})
            if method == "GET" and path.split("?", 1)[0] == "/api/vision/guide":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                cam = [p.split("=", 1)[1] for p in q.split("&")
                       if p.startswith("cam=")]
                try:
                    return self._send_json(
                        conn, 200,
                        eng.vision.guide(cam[0] if cam else "down"))
                except RuntimeError as e:
                    return self._send_json(conn, 400, {"ok": False,
                                                       "error": str(e)})
            if method == "GET" and path.split("?", 1)[0] == "/api/vision/results":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                try:
                    limit = int([p.split("=", 1)[1] for p in q.split("&")
                                 if p.startswith("limit=")][0])
                except (IndexError, ValueError):
                    limit = 20
                return self._send_json(conn, 200,
                                       eng.vision.results_public(limit))
            if method == "GET" and path == "/api/vision/templates":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                return self._send_json(conn, 200,
                                       eng.vision.templates_public())
            if method == "GET" and path.split("?", 1)[0] == \
                    "/api/vision/captures":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                cam = [p.split("=", 1)[1] for p in q.split("&")
                       if p.startswith("cam=")]
                try:
                    limit = int([p.split("=", 1)[1] for p in q.split("&")
                                 if p.startswith("limit=")][0])
                except (IndexError, ValueError):
                    limit = 50
                return self._send_json(conn, 200, eng.vision.captures_public(
                    cam=cam[0] if cam else None, limit=limit))
            if method == "GET" and path.split("?", 1)[0] == \
                    "/api/vision/capture/file":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                name = [p.split("=", 1)[1] for p in q.split("&")
                        if p.startswith("name=")]
                try:
                    png = eng.vision.capture_png(name[0] if name else "")
                except RuntimeError as e:
                    return self._send_json(conn, 400, {"ok": False,
                                                       "error": str(e)})
                return self._send_bytes(conn, png, "image/png")
            if method == "DELETE" and path.split("?", 1)[0] == \
                    "/api/vision/captures":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                req = self._read_body(conn, headers, rest)
                name = req.get("name")
                if not name:        # 兼容 query 形式 ?name=
                    q = path.split("?", 1)[1] if "?" in path else ""
                    name = [p.split("=", 1)[1] for p in q.split("&")
                            if p.startswith("name=")]
                    name = name[0] if name else ""
                return self._send_json(conn, 200,
                                       eng.vision.capture_delete(name))
            if method == "PUT" and path == "/api/vision/templates":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                req = self._read_body(conn, headers, rest)
                return self._send_json(conn, 200, eng.vision.template_update(
                    req.get("name", ""), new_name=req.get("new_name"),
                    threshold=req.get("threshold")))
            if method == "DELETE" and path.split("?", 1)[0] == \
                    "/api/vision/templates":
                if eng.vision is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "vision not attached"})
                req = self._read_body(conn, headers, rest)
                name = req.get("name")
                if not name:        # 兼容 query 形式 ?name=
                    q = path.split("?", 1)[1] if "?" in path else ""
                    name = [p.split("=", 1)[1] for p in q.split("&")
                            if p.startswith("name=")]
                    name = name[0] if name else ""
                return self._send_json(conn, 200,
                                       eng.vision.template_delete(name))
            # ---- 机器常数（machine_constants：相机内参单一事实源） ----
            if method == "GET" and path == "/api/machine_constants":
                if eng.constants is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "machine constants not attached"})
                px = (eng.vision.constants_px_lookup
                      if eng.vision is not None else None)
                # 响应含派生只读 k_um_px/fov_x_mm/fov_y_mm
                return self._send_json(conn, 200, eng.constants.public(px))
            if method == "PUT" and path == "/api/machine_constants":
                if eng.constants is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "machine constants not attached"})
                req = self._read_body(conn, headers, rest)
                # 深合并→全量校验→原子落盘；ValueError 由外层映射 400（消息含逐条 errors）
                eng.constants.update(req)
                if eng.vision is not None:      # 热生效：渲染/解析/对焦立即跟随
                    eng.vision.apply_constants()
                px = (eng.vision.constants_px_lookup
                      if eng.vision is not None else None)
                return self._send_json(conn, 200, {
                    "ok": True, "persisted": True,
                    "derived": eng.constants.public(px)["derived"]})
            # ---- 组件材质工程物性（materials：材料库+组件映射，2026-09-06
            # 迭代 21） ----
            if method == "GET" and path == "/api/materials":
                if eng.materials is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "materials not attached"})
                return self._send_json(conn, 200, eng.materials.materials_public())
            if method == "PUT" and path == "/api/materials":
                if eng.materials is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "materials not attached"})
                req = self._read_body(conn, headers, rest)
                # set/delete→全量校验（含被引用禁删）→原子落盘；
                # ValueError 由外层映射 400（消息含逐条 errors）
                eng.materials.update_materials(req)
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            if method == "GET" and path == "/api/material_map":
                if eng.materials is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "materials not attached"})
                return self._send_json(conn, 200, eng.materials.map_public())
            if method == "PUT" and path == "/api/material_map":
                if eng.materials is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "materials not attached"})
                req = self._read_body(conn, headers, rest)
                # patch 式指派（component_id→材料键/null）；
                # ValueError 由外层映射 400（消息含逐条 errors）
                eng.materials.update_map(req)
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            if method == "GET" and path == "/api/components":
                if eng.materials is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "materials not attached"})
                # 组件目录（前端模型只读解析）+当前映射材料键
                return self._send_json(conn, 200,
                                       eng.materials.components_public())
            # ---- 全机模态仿真（modal：每轴 SISO+全机 MIMO 模态，2026-09-08
            # 迭代 22；开关热切换走 POST /api/modal） ----
            if method == "GET" and path == "/api/modal":
                if eng.modal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "modal not attached"})
                return self._send_json(conn, 200, eng.modal.public())
            if method == "PUT" and path == "/api/modal":
                if eng.modal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "modal not attached"})
                req = self._read_body(conn, headers, rest)
                # 深合并→全量校验→原子落盘 modal.json+状态清零；
                # ValueError 由外层映射 400（消息含逐条 errors）
                eng.modal.update(req)
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            if method == "GET" and path == "/api/modal/state":
                if eng.modal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "modal not attached"})
                return self._send_json(conn, 200,
                                       eng.modal.state_public(eng))
            if method == "GET" and path.split("?", 1)[0] == "/api/modal/scope":
                if eng.modal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "modal not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                try:
                    ch = int([p.split("=", 1)[1] for p in q.split("&")
                              if p.startswith("ch=")][0])
                except (IndexError, ValueError):
                    ch = 0
                try:
                    n = int([p.split("=", 1)[1] for p in q.split("&")
                             if p.startswith("n=")][0])
                except (IndexError, ValueError):
                    n = 2000
                # ch 越界 ValueError 由外层映射 400
                return self._send_json(
                    conn, 200,
                    eng.modal.scope_public(ch, max(1, min(8000, n))))
            # ---- 组件级热仿真（thermal：LPTN 热网络，2026-09-09 迭代 23；
            # 开关热切换走 POST /api/thermal） ----
            if method == "GET" and path == "/api/thermal":
                if eng.thermal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "thermal not attached"})
                return self._send_json(conn, 200, eng.thermal.public())
            if method == "PUT" and path == "/api/thermal":
                if eng.thermal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "thermal not attached"})
                req = self._read_body(conn, headers, rest)
                # 深合并→全量校验+构建预检→原子落盘 thermal.json+状态重置；
                # ValueError 由外层映射 400（消息含逐条 errors）
                eng.thermal.update(req)
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            if method == "GET" and path == "/api/thermal/state":
                if eng.thermal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "thermal not attached"})
                return self._send_json(conn, 200,
                                       eng.thermal.state_public(eng))
            if method == "GET" and path.split("?", 1)[0] == "/api/thermal/curve":
                if eng.thermal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "thermal not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                node = [p.split("=", 1)[1] for p in q.split("&")
                        if p.startswith("node=")]
                try:
                    n = int([p.split("=", 1)[1] for p in q.split("&")
                             if p.startswith("n=")][0])
                except (IndexError, ValueError):
                    n = 2000
                # 未知节点 ValueError 由外层映射 400
                return self._send_json(
                    conn, 200,
                    eng.thermal.curve_public(node[0] if node else "",
                                             max(1, min(3600, n))))
            # ---- 下位机调参（tuning：PID 整定 + bond_hw 硬件参数） ----
            if method == "GET" and path == "/api/tuning":
                if eng.tuning is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "tuning not attached"})
                return self._send_json(conn, 200, eng.tuning.public())
            if method == "PUT" and path == "/api/tuning":
                if eng.tuning is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "tuning not attached"})
                req = self._read_body(conn, headers, rest)
                # 深合并→全量校验→原子落盘；engine 每秒重读热生效；
                # ValueError 由外层映射 400（消息含逐条 errors）
                eng.tuning.update(req)
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            # ---- 光谱共焦读数（confocal：调焦调平高度测量层） ----
            if method == "GET" and path.split("?", 1)[0] == \
                    "/api/confocal/read":
                if eng.confocal is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "confocal not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                probe = [p.split("=", 1)[1] for p in q.split("&")
                         if p.startswith("probe=")]
                try:
                    return self._send_json(conn, 200, eng.confocal.read(
                        probe[0] if probe else ""))
                except ValueError as e:
                    return self._send_json(conn, 400, {"ok": False,
                                                       "error": str(e)})
            # ---- 测校作业（calibration：REAL 链路三程序，作业模式） ----
            if method == "GET" and path == "/api/calibration/jobs":
                if eng.calibration is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "calibration not attached"})
                return self._send_json(conn, 200, eng.calibration.jobs_public())
            if method == "GET" and path.split("?", 1)[0] in \
                    ("/api/calibration/status", "/api/calibration/result"):
                if eng.calibration is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "calibration not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                try:
                    job_id = int([p.split("=", 1)[1] for p in q.split("&")
                                  if p.startswith("job_id=")][0])
                except (IndexError, ValueError):
                    return self._send_json(conn, 400, {
                        "ok": False, "error": "query 需要 job_id=<int>"})
                if path.split("?", 1)[0] == "/api/calibration/status":
                    return self._send_json(conn, 200,
                                           eng.calibration.status(job_id))
                return self._send_json(conn, 200,
                                       eng.calibration.result(job_id))
            if method == "GET" and path == "/api/transfers":
                if eng.workpieces is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "workpieces not attached"})
                return self._send_json(conn, 200,
                                       eng.workpieces.rules_public())
            # ---- 工艺流程编排（W4） ----
            if method == "GET" and path == "/api/flows":
                if eng.flow is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "flow engine not attached"})
                return self._send_json(conn, 200, eng.flow.flows_public())
            if method == "GET" and path == "/api/flow/state":
                if eng.flow is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "flow engine not attached"})
                st = eng.flow.state_public()
                # 一键串联链状态（迭代 7；未挂载时为 None）
                st["chain"] = (eng.chain.status()
                               if getattr(eng, "chain", None) else None)
                # 全片键合状态（迭代 11 第二棒；未挂载时为 None）
                st["full_wafer"] = (eng.full_wafer.status()
                                    if getattr(eng, "full_wafer", None)
                                    else None)
                return self._send_json(conn, 200, st)
            if method == "PUT" and path == "/api/flows":
                if eng.flow is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "flow engine not attached"})
                req = self._read_body(conn, headers, rest)
                errors = eng.flow.validate(req)
                if errors:
                    return self._send_json(conn, 400, {
                        "ok": False, "error": "flows 校验失败（%d 条）"
                                              % len(errors),
                        "errors": errors})
                try:
                    n = eng.flow.save_flows(req["flows"])
                except FlowStateError as e:
                    return self._send_json(conn, 409,
                                           {"ok": False, "error": str(e)})
                return self._send_json(conn, 200, {"ok": True, "flows": n})
            if method == "PUT" and path == "/api/transfers":
                if eng.workpieces is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "workpieces not attached"})
                req = self._read_body(conn, headers, rest)
                n = eng.workpieces.save_transfers(req.get("rules"),
                                                  req.get("grid"))
                return self._send_json(conn, 200, {"ok": True, "rules": n})
            # ---- 处方（recipe） ----
            if method == "GET" and path == "/api/recipe":
                if eng.recipe is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "recipe not attached"})
                # public() 注入派生块 dies_derived / wafer_sites_derived（只读）
                return self._send_json(conn, 200,
                                       {"ok": True, "recipe": eng.recipe.public()})
            if method == "GET" and \
                    path.split("?", 1)[0] == "/api/recipe/wafer_sites":
                if eng.recipe is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "recipe not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                try:
                    limit = int([p.split("=", 1)[1] for p in q.split("&")
                                 if p.startswith("limit=")][0])
                except (IndexError, ValueError):
                    limit = 200
                limit = max(1, min(limit, 100000))
                from recipe import wafer_sites_positions, derive_wafer_sites
                rec = eng.recipe.get()
                pos = wafer_sites_positions(rec)
                return self._send_json(conn, 200, {
                    "ok": True, "total_sites": len(pos), "limit": limit,
                    "returned": min(limit, len(pos)),
                    "derived": derive_wafer_sites(rec),
                    "positions": pos[:limit]})
            if method == "GET" and \
                    path.split("?", 1)[0] == "/api/workpiece/layout":
                # 布局摘要（迭代 15）：dies_on_film/dies_expected/
                # film_layout_mismatch——膜上 die 布局与当前处方不符检测
                # （全片 24 颗问题根因：膜是旧处方上膜时铺的）
                if eng.workpieces is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "workpieces not attached"})
                return self._send_json(conn, 200,
                                       eng.workpieces.layout_public())
            if method == "GET" and \
                    path.split("?", 1)[0] == "/api/workpiece/map":
                # die map 可视化数据（迭代 10）：蓝膜全量 die 格点（带行列/
                # id/当前载体）+ 晶圆全量 site（带行列/已键合态）+ 几何摘要
                if eng.recipe is None or eng.workpieces is None:
                    return self._send_json(conn, 503, {
                        "ok": False,
                        "error": "recipe/workpieces not attached"})
                from recipe import dies_cells, wafer_sites_cells, \
                    wafer_sites_chip_size
                from workpieces import site_bonded
                rec = eng.recipe.get()
                wp = eng.workpieces
                with wp._lock:
                    carriers = {d["id"]: d["carrier"] for d in wp.dies}
                film_dies = []
                for r, c, x, y in dies_cells(rec):
                    did = "R%02dC%02d" % (r, c)
                    film_dies.append({"row": r, "col": c, "x": x, "y": y,
                                      "id": did,
                                      # 离机（蓝膜下片等）的 die 标 gone
                                      "carrier": carriers.get(did, "gone")})
                waf_sites = [{"row": r, "col": c, "x": x, "y": y,
                              "bonded": site_bonded(wp, x, y)}
                             for r, c, x, y in wafer_sites_cells(rec)]
                fdc = rec["film"]["dies"]
                wsx, wsy = wafer_sites_chip_size(rec)
                return self._send_json(conn, 200, {
                    "ok": True,
                    "film_dies": film_dies,
                    "wafer_sites": waf_sites,
                    "geometry": {
                        "film": {
                            "diameter_mm": float(rec["film"]["diameter_mm"]),
                            "die_size_x_mm": float(fdc["size_x_mm"]),
                            "die_size_y_mm": float(fdc["size_y_mm"])},
                        "wafer": {
                            "diameter_mm":
                                float(rec["wafer"]["diameter_mm"]),
                            "site_size_x_mm": wsx,
                            "site_size_y_mm": wsy,
                            "edge_margin_mm": float(
                                rec["wafer_sites"].get("edge_margin_mm",
                                                       0.0))}}})
            # ---- 键合结果（bond_results，2026-09-02 迭代 11） ----
            if method == "GET" and \
                    path.split("?", 1)[0] == "/api/bond/throughput":
                # 产率分析（2026-09-05 迭代 15）：UPH/阶段节拍/计数/overlap
                # + live 进行中实时块（迭代 17 WP-C，传 eng 由 runner 状态
                # + sim_time 组装）
                br = getattr(eng, "bond_results", None)
                if br is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "bond results not attached"})
                return self._send_json(conn, 200, br.throughput_public(eng))
            if method == "GET" and \
                    path.split("?", 1)[0] == "/api/bond/results":
                br = getattr(eng, "bond_results", None)
                if br is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "bond results not attached"})
                q = path.split("?", 1)[1] if "?" in path else ""
                die = [p.split("=", 1)[1] for p in q.split("&")
                       if p.startswith("die=")]
                if die:                     # 单条（含完整 chain）
                    rec1 = br.get_by_die(die[0])
                    if rec1 is None:
                        return self._send_json(conn, 404, {
                            "ok": False,
                            "error": "无 die %r 的键合结果" % die[0]})
                    return self._send_json(conn, 200,
                                           {"ok": True, "result": rec1})
                # 全量列表 + 统计摘要
                return self._send_json(conn, 200, br.list_public())
            if method == "PUT" and path == "/api/recipe":
                if eng.recipe is None:
                    return self._send_json(conn, 503, {
                        "ok": False, "error": "recipe not attached"})
                req = self._read_body(conn, headers, rest)
                errors = eng.recipe.update(req)
                if errors:
                    return self._send_json(conn, 400, {
                        "ok": False, "error": "recipe 校验失败（%d 条）"
                                              % len(errors),
                        "errors": errors})
                return self._send_json(conn, 200, {"ok": True,
                                                   "persisted": True})
            if method == "POST":
                req = self._read_body(conn, headers, rest)
                # path 可能带 query string，路由匹配前先剥掉
                return self._send_json(
                    conn, *self._http_command(path.split("?", 1)[0], req))
            return self._send_json(conn, 404,
                                   {"ok": False, "error": "not found: " + path})
        except InterlockError as e:
            return self._send_json(conn, 409, {"ok": False, "error": str(e)})
        except FlowStateError as e:
            # 流程状态冲突（运行中再 run / 未暂停就 step 等）→ 409
            return self._send_json(conn, 409, {"ok": False, "error": str(e)})
        except (KeyError, ValueError, RuntimeError) as e:
            return self._send_json(conn, 400, {"ok": False, "error": str(e)})
        except Exception as e:
            return self._send_json(conn, 500, {"ok": False, "error": str(e)})

    def _http_command(self, path, req):
        """POST /api/* 指令路由 -> (code, obj)。"""
        eng = self.engine
        if path == "/api/axis/move":
            dur = eng.move(req["axis"], req["target"], req.get("speed"))
            return 200, {"ok": True, "duration": round(dur, 4)}
        if path == "/api/axis/jog":
            names = ac.resolve_axis(req.get("axis", ""))
            if not names:
                raise KeyError("unknown axis: %r" % req.get("axis"))
            with eng.lock:
                cur = eng.axes[names[0]].pos
            dur = eng.move(names[0], cur + float(req["delta"]),
                           req.get("speed"))
            return 200, {"ok": True, "target": cur + float(req["delta"]),
                         "duration": round(dur, 4)}
        if path == "/api/axis/home":
            eng.home(req.get("axis", "ALL"), req.get("mode"))
            return 200, {"ok": True}
        if path == "/api/axis/config":
            cfg = eng.set_axis_config(
                req["axis"], vmax=req.get("vmax"), amax=req.get("amax"),
                jerk=req.get("jerk"), repeat=req.get("repeat"),
                profile=req.get("profile"), limits=req.get("limits"),
                home_switch=req.get("home_switch"),
                home_offset=req.get("home_offset"),
                tau=req.get("tau"), settle_window=req.get("settle_window"),
                settle_time=req.get("settle_time"))
            cfg_resp = {"ok": True, "config": cfg, "persisted": True}
            return 200, cfg_resp
        if path == "/api/stop":
            eng.stop()
            return 200, {"ok": True}
        if path == "/api/estop":
            eng.estop()
            return 200, {"ok": True}
        if path == "/api/reset":
            eng.reset()
            return 200, {"ok": True}
        if path == "/api/full_reset":
            # 一键复位彻底化（2026-09-04 迭代 13）：流程/链/工件/测量结果/
            # 视觉结果环/倍速/温控压控全清；2026-09-09 迭代 23 起键合结果
            # 历史一并清空（与 /api/workpiece/reset 对齐）；recipe/
            # machine_constants/tuning 持久化文件不动
            return 200, self._full_reset()
        if path == "/api/speed":
            scale = eng.set_speed_scale(req.get("scale", 1.0))
            return 200, {"ok": True, "speed_scale": scale}
        if path == "/api/errormaps":
            if eng.errors is None:
                return 503, {"ok": False, "error": "error system not attached"}
            out = {"ok": True}
            if "enabled" in req:            # 单轴读数层开关（缺省不动现状）
                eng.errors.set_enabled(req["enabled"])
            out["enabled"] = eng.errors.enabled
            if eng.kinematics is not None:
                if "six_dof_enabled" in req:    # 6DOF 层开关，与单轴层独立
                    eng.kinematics.set_enabled(req["six_dof_enabled"])
                out["six_dof_enabled"] = eng.kinematics.six_dof_enabled
            return 200, out
        if path == "/api/modal":
            if eng.modal is None:
                return 503, {"ok": False, "error": "modal not attached"}
            # 模态总开关热切换（2026-09-08 迭代 22）：运行时内存态不持久
            # 化、默认 OFF；OFF 时状态清零、tick 零开销早退
            return 200, {"ok": True,
                         "enabled": eng.modal.set_enabled(
                             req.get("enabled", False))}
        if path == "/api/thermal":
            if eng.thermal is None:
                return 503, {"ok": False, "error": "thermal not attached"}
            # 热仿真总开关热切换（2026-09-09 迭代 23）：运行时内存态不持久
            # 化、默认 OFF；OFF 时 tick 早退+变形零注入、节点温度状态保持
            return 200, {"ok": True,
                         "enabled": eng.thermal.set_enabled(
                             req.get("enabled", False))}
        if path == "/api/io/command":       # 代理转发 PLC 命令路由（同进程直调）
            if eng.is_silent("plc"):
                # PLC 沉默（故障注入）：调试通道的指令同样被吞——收到但不执行，
                # 回执无异常（真机固件挂死时上位机只能靠自己超时/看门狗发现）
                return 200, {"ok": True, "swallowed": True,
                             "note": "plc silent：指令被吞，未执行"}
            from plc_http import plc_command
            return 200, plc_command(eng, self.points, req.get("path", ""),
                                    req.get("cmd", ""), req.get("args") or {})
        if path == "/api/fault":
            return 200, self._fault_command(req)
        if path == "/api/workpiece/wafer":
            if eng.workpieces is None:
                return 503, {"ok": False, "error": "workpieces not attached"}
            action = req.get("action")
            if action == "load":
                kwargs = {}
                if req.get("inject_recipe"):
                    # 按处方 wafer_load 注入上片误差（含 jitter，单一事实源
                    # recipe.wafer_load_inject_params；面板"晶圆上片"同款）
                    if eng.recipe is None:
                        return 503, {"ok": False, "error": "recipe not attached"}
                    from recipe import wafer_load_inject_params
                    kwargs.update(wafer_load_inject_params(
                        eng.recipe.get()["wafer_load"]))
                off = req.get("offset")         # 显式字段覆盖注入默认
                if off is not None:
                    kwargs["offset"] = off
                rz = req.get("rz")
                if rz is not None:
                    kwargs["rz"] = rz
                for k in ("z", "rx", "ry"):     # 垂向上片真值（mm/rad）
                    if req.get(k) is not None:
                        kwargs[k] = req[k]
                wafer = eng.workpieces.load_wafer(**kwargs)
                return 200, {"ok": True, "wafer": wafer}
            if action == "unload":
                res = eng.workpieces.unload_wafer()
                return 200, {"ok": True, "wafer": {"present": False},
                             "removed_dies": res["removed_dies"]}
            raise ValueError("action 只能是 load/unload")
        if path == "/api/workpiece/film":
            if eng.workpieces is None:
                return 503, {"ok": False, "error": "workpieces not attached"}
            action = req.get("action")
            if action == "load":            # 幂等：置在位 + 按处方重铺 die
                film = eng.workpieces.load_film()
                return 200, {"ok": True, "film": film}
            if action == "unload":
                res = eng.workpieces.unload_film()
                return 200, {"ok": True, "film": {"present": False},
                             "removed_dies": res["removed_dies"]}
            raise ValueError("action 只能是 load/unload")
        if path == "/api/workpiece/reset":
            if eng.workpieces is None:
                return 503, {"ok": False, "error": "workpieces not attached"}
            eng.workpieces.reset()
            # 迭代 11：键合结果随物料重置一并清空
            br = getattr(eng, "bond_results", None)
            if br is not None:
                br.clear()
            return 200, {"ok": True}
        if path == "/api/bond/clear":
            # 清空键合结果历史（2026-09-09 迭代 23，结果分析页"清空记录"
            # 按钮）：记录+产率统计全清（br.clear 语义），物料/流程不动
            br = getattr(eng, "bond_results", None)
            if br is None:
                return 503, {"ok": False,
                             "error": "bond results not attached"}
            br.clear()
            return 200, {"ok": True}
        if path == "/api/workpiece/pick_offset":
            if eng.workpieces is None:
                return 503, {"ok": False, "error": "workpieces not attached"}
            off = eng.workpieces.set_pick_offset(req.get("x_mm", 0.0),
                                                 req.get("y_mm", 0.0))
            return 200, {"ok": True, "pick_offset": off}
        if path == "/api/plc/scan":
            ms = eng.set_plc_scan(req.get("ms"))
            return 200, {"ok": True, "plc_scan_ms": ms}
        if path == "/api/interlock":
            on = eng.set_interlock(req.get("enabled", True))
            return 200, {"ok": True, "interlock_enabled": on}
        if path == "/api/collision":
            on = eng.set_collision(req.get("enabled", True))
            out = {"ok": True, "collision_enabled": on}
            if "depth_eps" in req:      # 穿模深度阈值在线改（ValueError→400）
                out["depth_eps"] = eng.set_collision_depth(req["depth_eps"])
            return 200, out
        # ---- 工艺流程编排（W4） ----
        if path == "/api/flow/run":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            name = req.get("name")
            if not name or not isinstance(name, str):
                raise ValueError("name 必填（流程名）")
            return 200, dict({"ok": True},
                             **eng.flow.run(name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/flow/pause":
            return 200, dict({"ok": True}, **eng.flow.pause())
        if path == "/api/flow/resume":
            return 200, dict({"ok": True}, **eng.flow.resume())
        if path == "/api/flow/step":
            return 200, dict({"ok": True}, **eng.flow.step())
        if path == "/api/flow/stop":
            return 200, dict({"ok": True}, **eng.flow.stop())
        if path == "/api/flow/reset":
            return 200, dict({"ok": True}, **eng.flow.reset())
        if path == "/api/flows/restore_example":
            return 200, {"ok": True, "flows": eng.flow.restore_example()}
        # ---- 参数化工艺流程（process_flows，机械常数+处方驱动） ----
        if path == "/api/process/flip_pick":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            name, steps = process_flows.build_flip_pick_flow(
                eng, req.get("die_id"))
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/flip_place_back":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            name, steps = process_flows.build_flip_place_back_flow(eng)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/flip_handoff":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            name, steps = process_flows.build_flip_handoff_flow(eng)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/head_to_flip":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            name, steps = process_flows.build_head_to_flip_flow(eng)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/wafer_ega":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            name, steps = process_flows.build_wafer_ega_flow(
                eng, with_load=req.get("with_load"))
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/focus_level":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            site = req.get("site")
            if site is not None:
                if not (isinstance(site, (list, tuple)) and len(site) == 2
                        and all(isinstance(v, (int, float))
                                and not isinstance(v, bool) for v in site)):
                    raise ValueError("site 须为 [row, col] 数值对")
                site = [int(site[0]), int(site[1])]
            name, steps = process_flows.build_focus_level_flow(
                eng, mode=req.get("mode", "per_site"), site=site,
                align_first=req.get("align_first"),
                verify=bool(req.get("verify", True)))
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/chain":
            # 一键串联（迭代 7）：wafer_ega→flip_pick→flip_handoff→
            # focus_level(per_site)→fine_align，状态感知跳过，失败 abort
            if eng.flow is None or getattr(eng, "chain", None) is None:
                return 503, {"ok": False, "error": "chain runner not attached"}
            site = req.get("site")
            if site is not None:
                if not (isinstance(site, (list, tuple)) and len(site) == 2
                        and all(isinstance(v, (int, float))
                                and not isinstance(v, bool) for v in site)):
                    raise ValueError("site 须为 [row, col] 数值对")
                site = [int(site[0]), int(site[1])]
            return 200, dict({"ok": True}, **eng.chain.start(site=site))
        if path == "/api/process/full_wafer":
            # 全片键合（迭代 11 第二棒）：启动时按处方 full_wafer 规划
            # die↔site 配对表，EGA 一次后逐对跑
            # flip_pick→flip_handoff→focus_level→fine_align→bond
            # clear_head（迭代 17 WP-C）：head/flip 有残片时 409 带
            # clear_head_option 提示；请求带 clear_head=true 先自动放回
            # 蓝膜再规划启动
            if eng.flow is None or getattr(eng, "full_wafer", None) is None:
                return 503, {"ok": False,
                             "error": "full wafer runner not attached"}
            try:
                return 200, dict({"ok": True}, **eng.full_wafer.start(
                    clear_head=bool(req.get("clear_head", False))))
            except FlowStateError as e:
                body = {"ok": False, "error": str(e)}
                if "clear_head" in str(e):
                    body["clear_head_option"] = True
                return 409, body
        if path == "/api/process/post_inspect":
            # 键合后精度检测（迭代 11 第二棒）：可选 sites 子集
            # [[row,col],...]，空=全部已键合位
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            sites = req.get("sites")
            if sites is not None:
                if not (isinstance(sites, list)
                        and all(isinstance(s, (list, tuple)) and len(s) == 2
                                and all(isinstance(v, (int, float))
                                        and not isinstance(v, bool)
                                        for v in s) for s in sites)):
                    raise ValueError("sites 须为 [[row,col],...] 数值对数组")
                sites = [[int(s[0]), int(s[1])] for s in sites]
            name, steps = process_flows.build_post_inspect_flow(
                eng, sites=sites or None)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/fine_align":
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            site = req.get("site")
            if site is not None:
                if not (isinstance(site, (list, tuple)) and len(site) == 2
                        and all(isinstance(v, (int, float))
                                and not isinstance(v, bool) for v in site)):
                    raise ValueError("site 须为 [row, col] 数值对")
                site = [int(site[0]), int(site[1])]
            name, steps = process_flows.build_fine_align_flow(eng, site=site)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/process/bond":
            # 热压键合（迭代 8）：守卫（晶圆+真空/head die/EGA/本 site
            # 精对准 pass/未重复键合）→ 等温 → 升温 → 下压 → PID 加压
            # → 保压 → 释放吹离 → 落位修正 → 抬头
            if eng.flow is None:
                return 503, {"ok": False, "error": "flow engine not attached"}
            import process_flows
            site = req.get("site")
            if site is not None:
                if not (isinstance(site, (list, tuple)) and len(site) == 2
                        and all(isinstance(v, (int, float))
                                and not isinstance(v, bool) for v in site)):
                    raise ValueError("site 须为 [row, col] 数值对")
                site = [int(site[0]), int(site[1])]
            name, steps = process_flows.build_bond_flow(eng, site=site)
            return 200, dict({"ok": True},
                             **eng.flow.run_steps(steps, name, paused=bool(
                                 req.get("paused", False))))
        if path == "/api/recipe/apply":
            if eng.recipe is None or eng.workpieces is None:
                return 503, {"ok": False, "error": "recipe/workpieces not attached"}
            # 按当前内存处方重建 die 阵列并重置物料状态（晶圆卸载、载体清空）
            out = dict({"ok": True}, layout=eng.workpieces.apply_recipe())
            # 硅片台常保温（bond.wafer_keep_warm=true）：apply 即设 Heat_Dw
            # 目标 wafer_temp_C 并常开 PID
            eng.apply_wafer_keep_warm()
            return 200, out
        if path == "/api/recipe/restore_default":
            if eng.recipe is None:
                return 503, {"ok": False, "error": "recipe not attached"}
            # 恢复默认处方并落盘；不自动 apply（物料状态不变）
            return 200, {"ok": True, "persisted": True,
                         "recipe": eng.recipe.restore_default()}
        # ---- 视觉系统（vision_service） ----
        if path in ("/api/vision/mode", "/api/vision/acquire",
                    "/api/vision/identify", "/api/vision/templates",
                    "/api/vision/camera/config", "/api/vision/capture/save"):
            if eng.vision is None:
                return 503, {"ok": False, "error": "vision not attached"}
            if path == "/api/vision/mode":
                return 200, eng.vision.set_mode(req.get("mode"))
            if path == "/api/vision/acquire":
                entry = eng.vision.acquire(req.get("cam", ""))
                return 200, {"ok": True, "entry": entry}
            if path == "/api/vision/identify":
                entry = eng.vision.identify(
                    req.get("cam", ""), template=req.get("template"),
                    ir_overlay=bool(req.get("ir_overlay", False)))
                return 200, {"ok": True, "entry": entry}
            if path == "/api/vision/capture/save":
                return 200, eng.vision.capture_save(
                    req.get("cam", ""),
                    ir_overlay=bool(req.get("ir_overlay", False)))
            if path == "/api/vision/templates":
                return 200, eng.vision.template_create(
                    req.get("cam", ""), name=req.get("name"),
                    roi=req.get("roi"), shape=req.get("shape"),
                    threshold=req.get("threshold"),
                    capture=req.get("capture"))
            return 200, eng.vision.camera_config(req.get("cam", ""), req)
        # ---- 测校作业（calibration：start 返 job_id，apply 写回机器常数） ----
        if path in ("/api/calibration/start", "/api/calibration/apply"):
            if eng.calibration is None:
                return 503, {"ok": False, "error": "calibration not attached"}
            try:
                if path == "/api/calibration/start":
                    return 200, eng.calibration.start(
                        req.get("routine", ""), req.get("cam", ""),
                        params=req.get("params"))
                return 200, eng.calibration.apply(
                    req.get("job_id"), fields=req.get("fields"))
            except CalibBusy as e:
                return 409, {"ok": False, "error": str(e)}
        # 协作式取消（迭代 29b）：置 cancel_req，作业在下一循环边界收
        # canceled 终态；CalibError（未知作业/非 running）走外层
        # RuntimeError→400 映射（与 jobs/status 同款）
        if path == "/api/calibration/cancel":
            if eng.calibration is None:
                return 503, {"ok": False, "error": "calibration not attached"}
            return 200, eng.calibration.cancel(req.get("job_id"))
        # ---- PyQt5 测校软件拉起（迭代 27） ----
        if path == "/api/launch/metrology_gui":
            return launch_metrology_gui(req)
        return 404, {"ok": False, "error": "not found: " + path}

    def _full_reset(self):
        """POST /api/full_reset（2026-09-04 迭代 13，一键复位彻底化）。

        顺序：链停请求 → 流程 stop 等停 → 等链退出 → flow/chain/full_wafer
        状态清零 → eng.reset()（estop/fault 原语义）+ workpieces.reset()
        → EGA/调平/精对准/粗对准/接触证据清零 → 视觉结果环清空 → 倍速回
        ×1 → 温控压控回环境态 → bond_results 键合结果历史清空（2026-09-09
        迭代 23 起，与 /api/workpiece/reset 对齐——此前刻意保留，面板
        "测量结果已清但结果分析页还在"自相矛盾）。不做：recipe/
        machine_constants/tuning 持久化文件。"""
        eng = self.engine
        ch = getattr(eng, "chain", None)
        fw = getattr(eng, "full_wafer", None)
        # 1. 链停请求（链循环在下一边界退出；当前阶段流程由 flow.stop 中止）
        if ch is not None and ch.status().get("active"):
            ch.request_stop()
        if fw is not None and fw.status().get("active"):
            fw.request_stop()
        # 2. 流程在跑（running/paused/stopping）先 stop 等停
        flow = eng.flow
        if flow is not None:
            if flow.state_public()["state"] in ("running", "paused",
                                                "stopping"):
                try:
                    flow.stop()
                except Exception:
                    pass
                t0 = time_mod.time()
                while time_mod.time() - t0 < 15.0:
                    if flow.state_public()["state"] not in \
                            ("running", "paused", "stopping"):
                        break
                    time_mod.sleep(0.05)
        # 3. 等链线程自收（stop_req 保证不再起新阶段/新配对）
        t0 = time_mod.time()
        while time_mod.time() - t0 < 10.0:
            busy = (ch is not None and ch.status().get("active")) or \
                   (fw is not None and fw.status().get("active"))
            if not busy:
                break
            time_mod.sleep(0.05)
        # 4. 流程/链状态清零
        if flow is not None and flow.state_public()["state"] != "idle":
            flow.reset()
        if ch is not None:
            ch.reset()
        if fw is not None:
            fw.reset()
        # 5. 引擎复位（estop/fault 原语义）+ 工件物料复位
        eng.reset()
        if eng.workpieces is not None:
            eng.workpieces.reset()
        # 6. EGA/调焦调平/精对准/粗对准/键合接触证据清零
        eng.ega_result = None
        eng.ega_measurements = []
        eng.fl_fit = {}
        eng.fl_measurements = {"wafer": [], "die": [], "stage": [],
                               "wafer_v": [], "die_v": []}
        eng.fl_ws_frozen = None
        eng.level_memory = None
        eng.fine_align_result = None
        eng.fa_meas = None
        eng.fa_delta = None
        eng.coarse_offset = None
        eng.bond_touch_pos = None
        eng.bond_contact_evidence = None
        eng.bond_commit = None
        # 7. 视觉识别结果环形缓冲清空
        if getattr(eng, "vision", None) is not None:
            eng.vision.clear_results()
        # 8. 倍速回 ×1
        eng.set_speed_scale(1.0)
        # 9. 温控压控回环境态（三回路 PID 关、加热器关、温度回环境、力 0）
        eng.reset_thermal_force()
        # 10. 键合结果历史清空（2026-09-09 迭代 23：与 /api/workpiece/reset
        # 对齐——复位语义含结果数据；处方/机械常数/tuning 持久化仍不动）
        br = getattr(eng, "bond_results", None)
        if br is not None:
            br.clear()
        return {"ok": True, "full_reset": True,
                "note": "流程/链/工件/测量结果/视觉结果/倍速/温控压控/键合"
                        "结果历史已清；处方/机械常数/tuning 持久化保留"}

    def _fault_command(self, req):
        """POST /api/fault（W2 故障注入）：
          轴使能 {"action":"disable"|"enable"|"reset_error","axis":"WS_LS_X"}
          目标级 {"action":"silent"|"drop","target":"acs|plc|mcs","on":true|false}
                 {"action":"restore","target":...}  = 解除沉默 + 恢复端口
        参数非法一律 400（ValueError/KeyError 由上层映射）。"""
        eng = self.engine
        action = req.get("action")
        if action in ("disable", "enable", "reset_error"):
            axis = req.get("axis")
            if not axis or not isinstance(axis, str):
                raise ValueError("axis 必填（轴名或别名，如 WS_LS_X / E_EPIN_Z）")
            if action == "disable":
                eng.enable(axis, False)
            elif action == "enable":
                eng.enable(axis, True)
            else:
                eng.reset_error(axis)
            return {"ok": True, "action": action, "axis": axis}
        if action in ("silent", "drop", "restore"):
            target = req.get("target")
            if target not in ("acs", "plc", "mcs"):
                raise ValueError("target 只能是 acs/plc/mcs")
            if action == "silent":
                on = req.get("on")
                if not isinstance(on, bool):
                    raise ValueError("silent 需要布尔 on")
                return {"ok": True, "silent": eng.set_silent(target, on)}
            if action == "drop":
                on = req.get("on")
                if not isinstance(on, bool):
                    raise ValueError("drop 需要布尔 on")
                return {"ok": True, "dropped": eng.set_dropped(target, on)}
            eng.set_silent(target, False)          # restore = 沉默+断线全解除
            return {"ok": True, "dropped": eng.set_dropped(target, False)}
        raise ValueError("action 只能是 disable/enable/reset_error/"
                         "silent/drop/restore")

    # ---------------- WebSocket ----------------
    def _ws_session(self, conn, addr, path, headers):
        if path != "/ws":
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
            conn.close()
            return
        key = headers.get("sec-websocket-key")
        if not key:
            conn.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
            conn.close()
            return
        resp = ("HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                "Sec-WebSocket-Accept: %s\r\n\r\n" % ws_accept_key(key))
        conn.sendall(resp.encode("latin1"))
        conn.settimeout(0.05)          # 发送超时上限；recv 侧容忍 timeout 循环
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        client = _Client(conn, addr)
        with self._clients_lock:
            self._clients.append(client)
        try:
            self._recv_loop(client)
        finally:
            client.alive = False
            with self._clients_lock:
                if client in self._clients:
                    self._clients.remove(client)
            try:
                conn.close()
            except OSError:
                pass

    def _read_frame_tolerant(self, conn):
        """socket.timeout 时返回 None（收发共用带超时的连接）。"""
        try:
            return read_client_frame(conn)
        except socket.timeout:
            return None

    def _recv_loop(self, client):
        conn = client.conn
        while client.alive:
            fr = self._read_frame_tolerant(conn)
            if fr is None:
                continue
            opcode, payload = fr
            if opcode == 0x8:      # close
                try:
                    conn.sendall(build_server_frame(0x8, payload[:125]))
                except OSError:
                    pass
                break
            if opcode == 0x9:      # ping -> pong
                conn.sendall(build_server_frame(0xA, payload))
                continue
            if opcode == 0xA:      # pong
                continue
            if opcode == 0x1:      # text JSON 指令
                err = None
                try:
                    req = json.loads(payload.decode("utf-8"))
                    reply = self._command(req)
                except Exception as e:
                    reply, err = None, str(e)
                if err is not None:
                    out = {"ok": False, "error": err}
                else:
                    out = reply          # 可能为 None（不回执）
                if out is not None:
                    try:
                        conn.sendall(build_server_frame(
                            0x1, json.dumps(out, ensure_ascii=False)
                            .encode("utf-8")))
                    except OSError:
                        break

    # ---------------- 指令路由（协议 §4） ----------------
    def _command(self, req):
        eng = self.engine
        cmd = req.get("cmd")
        if cmd == "move":
            try:
                eng.move(req["axis"], req["target"], req.get("speed"))
            except Exception as e:
                # 拒绝回执带 axis/target，前端据此弹回滑块并提示（干涉/限位）
                return {"ok": False, "error": str(e),
                        "axis": req.get("axis"), "target": req.get("target")}
            return None
        if cmd == "home":
            eng.home(req.get("axis", "ALL"), req.get("mode"))
            return None
        if cmd == "stop":
            eng.stop(req.get("axis"))
            return None
        if cmd == "estop":
            eng.estop()
            return None
        if cmd == "reset":
            eng.reset()
            return None
        if cmd == "setSpeed":
            eng.set_speed_scale(req.get("scale", 1.0))
            return None
        if cmd == "runBuffer":
            eng.run_buffer(req.get("buffer"))
            return None
        if cmd == "stopBuffer":
            eng.stop_buffer()
            return None
        if cmd == "io":
            path, op = req.get("path", ""), req.get("op", "")
            if op in ("OpenPositive", "OpenNegtive", "OpenAir"):
                eng.air_command(path, op)
            else:
                self.points.set(path, req.get("value", True))
            return None
        if cmd == "ping":
            return {"pong": True, "t": req.get("t")}
        raise ValueError("unknown cmd: %r" % cmd)
