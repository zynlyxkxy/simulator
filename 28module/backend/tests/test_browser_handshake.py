#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_browser_handshake.py — 模拟真实浏览器 WS 握手头，验证 ws_gateway 兼容性。

背景：file:// 页面里浏览器 WebSocket 握手与 test_client.py 的精简握手不同：
  - 带 `Origin: null`（file:// 为 opaque origin）
  - 带 `Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits`
  - 带 `User-Agent` / `Accept` / `Accept-Language` / `Cache-Control` / `Pragma`
  - Connection 头可能是 `Upgrade` 或大小写/多值变体

用法：先启动后端（python runtime/main.py），再跑本脚本。
验证：101 握手通过、二进制帧能收（>=100Hz 采 2s）、文本 ping/pong 回执。
"""
import json
import os
import socket
import struct
import sys
import time

WS_HOST, WS_PORT, WS_PATH = "127.0.0.1", 9010, "/ws"
FRAME_LEN = 108
FRAME_FMT = "<BBHddI"

# 逐字模拟 Edge/Chrome 在 file:// 页面发起 WebSocket 时的真实握手头
BROWSER_HEADERS = [
    "GET %s HTTP/1.1" % WS_PATH,
    "Host: %s:%d" % (WS_HOST, WS_PORT),
    "Connection: Upgrade",
    "Pragma: no-cache",
    "Cache-Control: no-cache",
    "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/139.0.0.0 Safari/537.36 Edg/139.0.0.0",
    "Upgrade: websocket",
    "Origin: null",
    "Sec-WebSocket-Version: 13",
    "Accept-Encoding: gzip, deflate, br, zstd",
    "Accept-Language: zh-CN,zh;q=0.9,en;q=0.8",
    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==",
    "Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits",
]


def main():
    sock = socket.create_connection((WS_HOST, WS_PORT), timeout=5)
    req = "\r\n".join(BROWSER_HEADERS) + "\r\n\r\n"
    print("[发送握手]")
    print(req)
    sock.sendall(req.encode("latin1"))

    resp = b""
    while b"\r\n\r\n" not in resp:
        chunk = sock.recv(4096)
        if not chunk:
            print("[FAIL] 对端在握手阶段关闭连接")
            return 1
        resp += chunk
    head, _, rest = resp.partition(b"\r\n\r\n")
    print("[收到响应]")
    print(head.decode("latin1"))

    status_line = head.split(b"\r\n", 1)[0]
    if b"101" not in status_line:
        print("[FAIL] 握手被拒：%s" % status_line.decode("latin1"))
        return 1
    print("[PASS] 握手 101（Origin: null + permessage-deflate 均被容忍）")

    # 收 2s 二进制帧
    sock.settimeout(5)
    buf = rest
    frames = 0
    seqs = []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        while True:
            if len(buf) < 2:
                break
            ln = buf[1] & 0x7F
            off = 2
            if ln == 126:
                if len(buf) < 4:
                    break
                ln = struct.unpack("!H", buf[2:4])[0]
                off = 4
            if len(buf) < off + ln:
                break
            op = buf[0] & 0x0F
            payload, buf = buf[off:off + ln], buf[off + ln:]
            if op == 0x2 and len(payload) == FRAME_LEN:
                magic, ver, nax, sim, wall, seq = struct.unpack(
                    FRAME_FMT, payload[:struct.calcsize(FRAME_FMT)])
                frames += 1
                seqs.append(seq)
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
    dur = time.time() - t0
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b - a != 1)
    print("[收帧] %d 帧 / %.1fs = %.0f Hz, seq 跳号 %d" % (frames, dur, frames / dur, gaps))
    if frames < 100:
        print("[FAIL] 二进制帧接收异常")
        return 1
    print("[PASS] 二进制帧流正常（%.0f Hz, 无压缩扩展干扰）" % (frames / dur))

    # 文本 ping → pong 回执
    payload = json.dumps({"cmd": "ping", "t": time.time()}).encode("utf-8")
    mask = os.urandom(4)
    hdr = struct.pack("!BB", 0x81, 0x80 | len(payload))
    sock.sendall(hdr + mask +
                 bytes(b ^ mask[i & 3] for i, b in enumerate(payload)))
    buf = b""
    got_pong = False
    t0 = time.time()
    while time.time() - t0 < 2.0 and not got_pong:
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 2:
            ln = buf[1] & 0x7F
            off = 2
            if ln == 126:
                if len(buf) < 4:
                    break
                ln = struct.unpack("!H", buf[2:4])[0]
                off = 4
            if len(buf) < off + ln:
                break
            op = buf[0] & 0x0F
            payload, buf = buf[off:off + ln], buf[off + ln:]
            if op == 0x1:
                obj = json.loads(payload.decode("utf-8"))
                if obj.get("pong"):
                    got_pong = True
    print("[%s] 文本 ping/pong 回执" % ("PASS" if got_pong else "FAIL"))
    sock.close()
    return 0 if got_pong else 1


if __name__ == "__main__":
    sys.exit(main())
