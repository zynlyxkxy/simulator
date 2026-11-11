#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""latency_probe.py — 直接用原始 WS 客户端测 9030 的推送频率与单向延迟。

用法：python light_test/tools/latency_probe.py [--port 9030] [--frames 500]

与网页 HUD 的区别：不经浏览器事件循环，纯 socket 收帧，得到的是「后端发出 →
本机收到」的真实单向延迟（p50/p99），可与项目红线 p50<1ms 直接对照。
"""
import argparse
import json
import os
import socket
import struct
import sys
import time

HOST = "127.0.0.1"


def ws_connect(port, path="/ws"):
    s = socket.create_connection((HOST, port), timeout=10)
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    req = ("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
           "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
           "Sec-WebSocket-Version: 13\r\n\r\n" % (path, HOST, port, key))
    s.sendall(req.encode("latin1"))
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    assert b"101" in buf.split(b"\r\n")[0], buf[:200]
    return s


def recv_frame(sock, buf):
    while True:
        if len(buf) >= 2:
            b1, b2 = buf[0], buf[1]
            ln = b2 & 0x7F
            off = 2
            if ln == 126:
                if len(buf) < 4:
                    buf += sock.recv(65536); continue
                ln = struct.unpack("!H", buf[2:4])[0]; off = 4
            elif ln == 127:
                if len(buf) < 10:
                    buf += sock.recv(65536); continue
                ln = struct.unpack("!Q", buf[2:10])[0]; off = 10
            if len(buf) >= off + ln:
                payload = buf[off:off + ln]
                return payload, buf[off + ln:]
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9030)
    ap.add_argument("--frames", type=int, default=500)
    a = ap.parse_args()

    s = ws_connect(a.port)
    buf = b""
    lat, seqs, t0 = [], [], None
    while len(lat) < a.frames:
        payload, buf = recv_frame(s, buf)
        if len(payload) != 108:
            continue
        magic, ver, n, sim_t, wall_t, seq = struct.unpack("<BBHddI", payload[:24])
        assert magic == 0xA1 and n == 21, (magic, n)
        now = time.time()
        if t0 is None:
            t0 = now
        lat.append((now - wall_t) * 1000.0)
        seqs.append(seq)
    dur = now - t0
    s.close()

    lat.sort()
    p50 = lat[len(lat) // 2]
    p99 = lat[int(len(lat) * 0.99)]
    miss = sum(1 for i in range(1, len(seqs)) if seqs[i] != seqs[i - 1] + 1)
    print("端口 %d 帧数 %d 时长 %.2fs → %.1f Hz" %
          (a.port, len(lat), dur, (len(lat) - 1) / dur if dur else 0))
    print("单向延迟 p50 %.3f ms / p99 %.3f ms / max %.3f ms" %
          (p50, p99, lat[-1]))
    print("帧序跳号 %d 次" % miss)
    ok = 480 <= ((len(lat) - 1) / dur) <= 520 and p50 < 1.0 and miss == 0
    print("\n==== %s ====" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
