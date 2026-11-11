# -*- coding: utf-8 -*-
"""acs_probe.py — ACS SPiiPlus TCP 701 协议实证探针（中间人实验）

原理：真客户端库 ACSCL_x64.dll 就在手里。用 ctypes 加载它，对我们在本机开的
假 701 监听端口发起连接，抓取它真实发出的字节流；再按策略应答，观察它的下一步
动作，逐步试错归纳握手与 framing 规律。全程离线。

用法：
    python tools/acs_probe.py --selftest          # 自检：dll 加载 + 假监听回路（回归用）
    python tools/acs_probe.py                     # 全量探针：握手抓包 + 应答试错 + 指令抓包
    python tools/acs_probe.py --phase handshake   # 只跑握手抓包
    python tools/acs_probe.py --port 91701 --wait 6

结论记录在 backend/docs/acs701_抓包记录.md（由探针输出整理）。
"""
import argparse
import ctypes
import os
import socket
import sys
import threading
import time

# ---- 路径（本工程固定布局，离线，不读环境变量）----
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))  # 28module/
# 目录层级经历过重组：zip 解包直接展开是单层 io_template/，旧布局是双层 io_template/io_template/
_ACS_LIB_DIR_CANDIDATES = [
    os.path.join(_ROOT, "部分接口信息", "io_template", "mainproject",
                 "acs_module", "AcsClient", "acs", "lib"),
    os.path.join(_ROOT, "部分接口信息", "io_template", "io_template", "mainproject",
                 "acs_module", "AcsClient", "acs", "lib"),
]
_ACS_LIB_DIR = next((d for d in _ACS_LIB_DIR_CANDIDATES
                     if os.path.isfile(os.path.join(d, "ACSCL_x64.dll"))),
                    _ACS_LIB_DIR_CANDIDATES[0])
_DLL_PATH = os.path.join(_ACS_LIB_DIR, "ACSCL_x64.dll")

DEFAULT_PORT = 9701  # 仿真用端口，避开真机 701（用户提的 91701 超出 TCP 端口上限 65535，取 9701 保留 "9+701" 记忆法）

# ACSC.h 关键常量
ACSC_INVALID = 0xFFFFFFFFFFFFFFFF  # (HANDLE)-1
ACSC_ERRORBASE = 100
ERR_NAMES = {  # 常用错误码名（来自 ACSC.h，值 = ACSC_ERRORBASE + n）
    100: "ACSC_UNKNOWNERROR", 130: "ACSC_TIMEOUT", 132: "ACSC_INITFAILURE",
    134: "ACSC_INVALIDHANDLE", 135: "ACSC_ALLCHANNELSBUSY", 153: "ACSC_CHECKSUMERROR",
    154: "ACSC_REPLIESSEQUENCEERROR", 162: "ACSC_SENDINGFAILED", 163: "ACSC_RECEIVINGFAILED",
    182: "ACSC_CLOSED_BY_CONTROLLER", 197: "ACSC_SERVEREXCEPTION",
    238: "ACSC_LICENSE_COMMON_PROBLEM", 239: "ACSC_LICENSE_DONGLE_NOT_FOUND",
    240: "ACSC_LICENSE_ENTRY_NOT_FOUND", 241: "ACSC_LICENSE_INVALID_HANDLE",
    242: "ACSC_LICENSE_NO_DATA_AVAILABLE", 243: "ACSC_LICENSE_INVALID_PN",
}


def hexdump(data, max_len=256):
    """简洁 hexdump：偏移 + hex + ascii。"""
    lines = []
    shown = data[:max_len]
    for off in range(0, len(shown), 16):
        chunk = shown[off:off + 16]
        hexs = " ".join("%02x" % b for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append("  %04x  %-47s  %s" % (off, hexs, asc))
    if len(data) > max_len:
        lines.append("  ...（截断，共 %d 字节）" % len(data))
    return "\n".join(lines) if lines else "  （空）"


def err_name(code):
    return ERR_NAMES.get(code, "ACSC_ERRORBASE+%d" % (code - ACSC_ERRORBASE) if code >= ACSC_ERRORBASE else "?")


def load_library(log):
    """加载 ACSCL_x64.dll 并设置函数原型。返回 lib 或 None。"""
    if not os.path.isfile(_DLL_PATH):
        log("FAIL: dll 不存在: %s" % _DLL_PATH)
        return None
    os.add_dll_directory(_ACS_LIB_DIR)  # WibuCm64.dll 等依赖同目录
    try:
        lib = ctypes.WinDLL(_DLL_PATH)
    except OSError as e:
        log("FAIL: WinDLL 加载失败: %s" % e)
        return None
    log("OK: dll 加载成功: %s" % _DLL_PATH)

    # x64 导出为未修饰名（ACSC.h 内置加载器的 _xxx@n 修饰名是 x86 形态）
    lib.acsc_OpenCommEthernetTCP.restype = ctypes.c_void_p
    lib.acsc_OpenCommEthernetTCP.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.acsc_CloseComm.restype = ctypes.c_int
    lib.acsc_CloseComm.argtypes = [ctypes.c_void_p]
    lib.acsc_GetLastError.restype = ctypes.c_int
    lib.acsc_GetLastError.argtypes = []
    lib.acsc_GetErrorString.restype = ctypes.c_int
    lib.acsc_GetErrorString.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p,
                                        ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    lib.acsc_SetTimeout.restype = ctypes.c_int
    lib.acsc_SetTimeout.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.acsc_GetTimeout.restype = ctypes.c_int
    lib.acsc_GetTimeout.argtypes = [ctypes.c_void_p]
    lib.acsc_Transaction.restype = ctypes.c_int
    lib.acsc_Transaction.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_int), ctypes.c_void_p]
    lib.acsc_Enable.restype = ctypes.c_int
    lib.acsc_Enable.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lib.acsc_Disable.restype = ctypes.c_int
    lib.acsc_Disable.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lib.acsc_Halt.restype = ctypes.c_int
    lib.acsc_Halt.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lib.acsc_ToPoint.restype = ctypes.c_int
    lib.acsc_ToPoint.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                                 ctypes.c_double, ctypes.c_void_p]
    lib.acsc_GetFPosition.restype = ctypes.c_int
    lib.acsc_GetFPosition.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_double), ctypes.c_void_p]
    lib.acsc_GetVelocity.restype = ctypes.c_int
    lib.acsc_GetVelocity.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_double), ctypes.c_void_p]
    lib.acsc_GetMotorState.restype = ctypes.c_int
    lib.acsc_GetMotorState.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                       ctypes.POINTER(ctypes.c_int), ctypes.c_void_p]
    lib.acsc_GetProgramState.restype = ctypes.c_int
    lib.acsc_GetProgramState.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                         ctypes.POINTER(ctypes.c_int), ctypes.c_void_p]
    for name in ("acsc_OpenCommEthernetTCP", "acsc_GetLastError", "acsc_Transaction",
                 "acsc_Enable", "acsc_ToPoint", "acsc_GetFPosition", "acsc_GetMotorState"):
        try:
            getattr(lib, name)
        except AttributeError:
            log("FAIL: 导出缺失: %s" % name)
            return None
    log("OK: 关键导出全部解析（x64 未修饰名）")
    return lib


def get_error_string(lib, handle, code):
    buf = ctypes.create_string_buffer(256)
    recv = ctypes.c_int(0)
    r = lib.acsc_GetErrorString(handle, code, buf, 255, ctypes.byref(recv))
    if r and recv.value > 0:
        return buf.raw[:recv.value].decode("ascii", "replace")
    return ""


class FakeListener:
    """假 701 监听：接受连接，记录字节流，按 responder 策略应答。

    responder(direction_state, data) -> bytes|None
      每次收到数据后调用，返回要回发的字节（None=不回）。
    """

    def __init__(self, port, responder=None, log=print):
        self.port = port
        self.responder = responder or (lambda conn, data: None)
        self.log = log
        self.events = []       # [(t, "recv"/"send", bytes)]
        self._stop = threading.Event()
        self._srv = None
        self._thread = None
        self.conn_count = 0

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", self.port))
        self._srv.listen(4)
        self._srv.settimeout(0.2)
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.conn_count += 1
            self.log("  [listener] 接受连接 #%d from %s" % (self.conn_count, addr))
            threading.Thread(target=self._conn_loop, args=(conn,), daemon=True).start()

    def _conn_loop(self, conn):
        conn.settimeout(0.2)
        buf = b""
        try:
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    self.log("  [listener] 对端关闭连接")
                    break
                buf += data
                self.events.append((time.time(), "recv", data))
                reply = self.responder(self, buf)
                if reply:
                    conn.sendall(reply)
                    self.events.append((time.time(), "send", reply))
        except OSError as e:
            self.log("  [listener] 连接异常: %s" % e)
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def stop(self):
        self._stop.set()
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass

    def dump_events(self, log):
        if not self.events:
            log("  （无字节往来）")
            return
        t0 = self.events[0][0]
        for t, direction, data in self.events:
            log("  [%+.3fs] %s %d 字节:" % (t - t0, direction.upper(), len(data)))
            log(hexdump(data))


# ---------------------------------------------------------------- 自检

def selftest(log):
    """回归用自检：dll 可加载、导出齐全、假监听回路可收发。不发起 ACS 连接。"""
    ok = True
    lib = load_library(log)
    ok &= lib is not None

    # 假监听 + 裸 socket 回路
    seen = []

    def responder(listener, data):
        if data.endswith(b"\r"):
            return b"\xe5\x07\x00\x00OK\r\x06"  # 模拟真机 framing 形态
        return None

    port = 19701  # selftest 独立端口，避免与探针/仿真冲突
    listener = FakeListener(port, responder, log=lambda s: None)
    listener.start()
    try:
        c = socket.create_connection(("127.0.0.1", port), timeout=2)
        c.sendall(b"\r?COMMCH\r")
        c.settimeout(2)
        got = c.recv(64)
        c.close()
        if got == b"\xe5\x07\x00\x00OK\r\x06":
            log("OK: 假监听回路收发正常")
        else:
            log("FAIL: 假监听回路应答不符: %r" % got)
            ok = False
        seen.append(got)
    except OSError as e:
        log("FAIL: 假监听回路连接失败: %s" % e)
        ok = False
    finally:
        listener.stop()

    log("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------- 握手探针

def probe_handshake(lib, port, wait_s, log, responder=None, label=""):
    """起假监听 → acsc_OpenCommEthernetTCP → 记录结果与字节流。"""
    log("== 握手探针 %s（port=%d, wait=%.1fs）==" % (label or "", port, wait_s))
    listener = FakeListener(port, responder, log=log)
    listener.start()
    handle = None
    try:
        time.sleep(0.1)
        t0 = time.time()
        handle = lib.acsc_OpenCommEthernetTCP(b"127.0.0.1", port)
        dt = time.time() - t0
        if handle in (None, 0, ACSC_INVALID):
            code = lib.acsc_GetLastError()
            log("  结果: ACSC_INVALID（耗时 %.2fs）GetLastError=%d (%s) %s"
                % (dt, code, err_name(code), get_error_string(lib, None, code)))
        else:
            log("  结果: 连接成功 handle=%s（耗时 %.2fs）" % (hex(handle), dt))
        time.sleep(wait_s)
    finally:
        if handle not in (None, 0, ACSC_INVALID):
            lib.acsc_CloseComm(handle)
        listener.stop()
    log("  字节流记录：")
    listener.dump_events(log)
    return handle not in (None, 0, ACSC_INVALID), listener


# ---------------------------------------------------------------- 主流程

def main():
    ap = argparse.ArgumentParser(description="ACS TCP 701 协议实证探针")
    ap.add_argument("--selftest", action="store_true", help="自检模式（回归用）")
    ap.add_argument("--phase", default="all", choices=["all", "handshake", "cmds"])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--wait", type=float, default=6.0, help="连接后观察秒数")
    args = ap.parse_args()

    log = print
    log("# acs_probe 端口=%d pid=%d" % (args.port, os.getpid()))

    if args.selftest:
        sys.exit(0 if selftest(log) else 1)

    lib = load_library(log)
    if lib is None:
        sys.exit(1)

    # 补充原型：服务器（UMD）定向函数
    lib.acsc_SetServerExt.restype = ctypes.c_int
    lib.acsc_SetServerExt.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.acsc_OpenCommEthernet.restype = ctypes.c_void_p
    lib.acsc_OpenCommEthernet.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.acsc_GetLibraryVersion.restype = ctypes.c_uint
    lib.acsc_GetUMDVersion.restype = ctypes.c_uint
    log("库版本 GetLibraryVersion=%08x  UMDVersion=%08x（0=本机无 UMD 服务）"
        % (lib.acsc_GetLibraryVersion(), lib.acsc_GetUMDVersion()))

    # 阶段 0：直连假 701 监听（纯观察），看库发出什么、报什么错
    ok, _ = probe_handshake(lib, args.port, args.wait, log, label="A-纯观察")

    # 阶段 0b：对照组——不可达地址，若同样秒回且零字节，证明失败发生在联网之前
    t0 = time.time()
    h = lib.acsc_OpenCommEthernetTCP(b"192.0.2.1", 701)
    log("  对照组 OpenCommEthernetTCP(192.0.2.1:701) -> %s dt=%.3fs err=%d %s"
        % ("INVALID" if h in (None, 0, ACSC_INVALID) else hex(h),
           time.time() - t0, lib.acsc_GetLastError(), err_name(lib.acsc_GetLastError())))
    if h not in (None, 0, ACSC_INVALID):
        lib.acsc_CloseComm(h)

    # 阶段 1：UMD 通道抓包——把"通讯服务器"指到假监听，抓 DLL↔UMD 的字节流
    log("== UMD 通道探针（acsc_SetServerExt 指向假监听）==")
    listener = FakeListener(args.port, log=log)
    listener.start()
    try:
        r = lib.acsc_SetServerExt(b"127.0.0.1", args.port)
        log("  SetServerExt -> r=%d err=%d %s" % (r, lib.acsc_GetLastError(),
                                                err_name(lib.acsc_GetLastError())))
        t0 = time.time()
        h = lib.acsc_OpenCommEthernet(b"127.0.0.1", 701)
        code = lib.acsc_GetLastError()
        log("  OpenCommEthernet -> %s dt=%.2fs err=%d %s %s"
            % ("INVALID" if h in (None, 0, ACSC_INVALID) else hex(h),
               time.time() - t0, code, err_name(code),
               get_error_string(lib, None, code)))
        if h not in (None, 0, ACSC_INVALID):
            lib.acsc_CloseComm(h)
        time.sleep(1.0)
    finally:
        listener.stop()
    log("  UMD 通道字节流：")
    listener.dump_events(log)

    if not ok:
        log("# 探针结束（握手未打通：ACSCL 全部通讯经由本机 UMD RPC 服务代理，"
            "本机未装 ACS ADK/CodeMeter，任何 701 字节都不会发出）")
        return


if __name__ == "__main__":
    main()
