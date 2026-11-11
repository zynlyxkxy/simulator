# -*- coding: utf-8 -*-
"""
stub_server.py — 前后端数据协议 V1 最小桩服务器（纯标准库）

- ws://127.0.0.1:9010/ws ：500Hz 推送 108 字节定长二进制帧（21 轴正弦运动）
- 接收客户端文本 JSON 指令并打印（move/home/estop/...）
- 同端口 HTTP 调试口：GET /api/health /api/axes /api/io

用法：python stub_server.py  （Ctrl+C 退出）
"""
import base64
import hashlib
import json
import math
import socket
import struct
import threading
import time

HOST = '127.0.0.1'
PORT = 9010
PUSH_HZ = 500
AXIS_COUNT = 21
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

AXIS_IDS = [
    'axis_mssl5svn_ols740', 'axis_msslggo6_6d6mzm', 'axis_msslrthn_qbthvp',
    'axis_epin1_z_0001', 'axis_epin2_z_0001', 'axis_epin3_z_0001',
    'axis_mssq9yhm_gmwadb', 'axis_mssrgvw5_7ds4bo', 'axis_mssrheu1_cfw18w',
    'axis_msst4og7_eby5r6', 'axis_msstd5bj_poz6p6', 'axis_alignz_0001',
    'axis_mssu8pry_ijkgc7', 'axis_mssu8zgl_vi1rbv', 'axis_mswkylsp_nmcvcb',
    'axis_mswlez0m_3z4lwg', 'axis_mswlx2e6_v4wbj4', 'axis_mswlzliz_t1ybur',
    'axis_filmstage_x_0001', 'axis_filmstage_y_0001', 'axis_filmpin_z_0001',
]

# (中值, 幅值, 频率Hz) —— 协议 V1：直线轴 mm、旋转轴 rad（全线统一 rad）
# 旋转轴幅值按前端 deg 行程换算：0.8°→0.014rad、0.08°→0.0014rad、90°→1.57rad、120°→2.09rad
MOTION = [
    (0,    100, 0.20),   # 0  WS_LS_X   mm  ±200
    (0,    100, 0.25),   # 1  WS_LS_Y   mm  ±200
    (0,    0.014, 0.15), # 2  WS_Rz     rad ±0.014 (≈±0.8°)
    (15,   10,  0.50),   # 3  E_EPIN_Z_1 mm 0..30
    (15,   10,  0.55),   # 4  E_EPIN_Z_2
    (15,   10,  0.60),   # 5  E_EPIN_Z_3
    (-95,  50,  0.10),   # 6  BS_Arm_Z  mm  -190..0
    (0,    0.0014, 0.30),# 7  BS_CAL_Rx rad ±0.0014 (≈±0.08°)
    (0,    0.0014, 0.32),# 8  BS_CAL_Ry rad ±0.0014
    (25,   30,  0.22),   # 9  AS_X      mm  -30..80
    (45,   50,  0.18),   # 10 AS_Y      mm  -30..120
    (2,    3,   0.40),   # 11 AS_Z      mm  -2..6
    (0,    0.0014, 0.35),# 12 AS_Rx     rad ±0.0014
    (0,    0.0014, 0.38),# 13 AS_Ry     rad ±0.0014
    (-620, 400, 0.08),   # 14 FA_X      mm  -1240..0
    (0,    30,  0.28),   # 15 FA_Z      mm  ±40
    (0,    1.57, 0.12),  # 16 FA_Rx     rad ±1.57 (≈±90°)
    (0,    2.09, 0.10),  # 17 CH_Rz     rad ±2.09 (≈±120°)
    (-100, 150, 0.15),   # 18 ES_X      mm  -285..85
    (10,   150, 0.13),   # 19 ES_Y      mm  -175..195
    (2.5,  2,   0.60),   # 20 ES_PunchZ mm  0..5
]


def build_frame(seq, sim_time, wall_time, values):
    return struct.pack('<BBHddI%df' % AXIS_COUNT,
                       0xA1, 1, AXIS_COUNT, sim_time, wall_time, seq, *values)


def encode_ws_frame(payload, opcode=0x2):
    """server→client 帧（不掩码）"""
    header = bytearray([0x80 | opcode])
    n = len(payload)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header += bytes([126]) + struct.pack('>H', n)
    else:
        header += bytes([127]) + struct.pack('>Q', n)
    return bytes(header) + payload


def recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('peer closed')
        buf += chunk
    return buf


def recv_ws_frame(conn):
    """返回 (opcode, payload)；连接关闭抛异常"""
    b1, b2 = recv_exact(conn, 2)
    opcode = b1 & 0x0F
    masked = b2 & 0x80
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack('>H', recv_exact(conn, 2))[0]
    elif length == 127:
        length = struct.unpack('>Q', recv_exact(conn, 8))[0]
    mask = recv_exact(conn, 4) if masked else b'\x00' * 4
    payload = bytearray(recv_exact(conn, length))
    for i in range(length):
        payload[i] ^= mask[i % 4]
    return opcode, bytes(payload)


def http_response(conn, body, status='200 OK', ctype='application/json; charset=utf-8'):
    data = body.encode('utf-8')
    head = ('HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n'
            'Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n'
            % (status, ctype, len(data)))
    conn.sendall(head.encode('ascii') + data)


def handle_http(conn, path):
    if path == '/api/health':
        http_response(conn, json.dumps({'ok': True, 'axes': AXIS_COUNT, 'pushHz': PUSH_HZ}))
    elif path == '/api/axes':
        http_response(conn, json.dumps({'axes': [
            {'index': i, 'id': AXIS_IDS[i]} for i in range(AXIS_COUNT)]}, ensure_ascii=False))
    elif path == '/api/io':
        http_response(conn, json.dumps({'io': {}}))
    else:
        http_response(conn, json.dumps({'ok': False, 'error': 'not found'}), '404 Not Found')


def push_loop(conn, stop_event):
    """500Hz 推送线程：忙等+睡眠混合保证 2ms 周期"""
    period = 1.0 / PUSH_HZ
    seq = 0
    t0 = time.time()
    next_t = t0
    while not stop_event.is_set():
        now = time.time()
        sim_time = now - t0
        values = [c + a * math.sin(2 * math.pi * f * sim_time + i * 0.7)
                  for i, (c, a, f) in enumerate(MOTION)]
        try:
            conn.sendall(encode_ws_frame(build_frame(seq, sim_time, now, values)))
        except OSError:
            stop_event.set()
            return
        seq += 1
        next_t += period
        delay = next_t - time.time()
        if delay > 0.001:
            time.sleep(delay - 0.0005)
        while time.time() < next_t:  # 尾部忙等，Windows 睡眠精度补偿
            if stop_event.is_set():
                return


def handle_ws(conn, headers):
    key = headers.get('sec-websocket-key', '')
    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    conn.sendall(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                  'Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n' % accept).encode())
    print('[stub] WS 客户端已连接')
    stop = threading.Event()
    sender = threading.Thread(target=push_loop, args=(conn, stop), daemon=True)
    sender.start()
    try:
        while True:
            opcode, payload = recv_ws_frame(conn)
            if opcode == 0x8:  # close
                conn.sendall(encode_ws_frame(b'', 0x8))
                break
            if opcode == 0x9:  # ping → pong
                conn.sendall(encode_ws_frame(payload, 0xA))
                continue
            if opcode == 0x1:  # text：客户端指令
                text = payload.decode('utf-8', 'replace')
                try:
                    cmd = json.loads(text)
                    print('[stub] 收到指令:', json.dumps(cmd, ensure_ascii=False))
                    if cmd.get('cmd') == 'ping':
                        conn.sendall(encode_ws_frame(
                            json.dumps({'pong': True, 't': cmd.get('t')}).encode(), 0x1))
                except ValueError:
                    print('[stub] 收到非 JSON 文本:', text)
    except (ConnectionError, OSError):
        pass
    finally:
        stop.set()
        sender.join(timeout=1)
        print('[stub] WS 客户端已断开')


def handle_conn(conn):
    try:
        conn.settimeout(None)
        # 读 HTTP 请求头
        buf = b''
        while b'\r\n\r\n' not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
        request = buf.decode('latin-1')
        lines = request.split('\r\n')
        path = lines[0].split(' ')[1] if len(lines[0].split(' ')) > 1 else '/'
        headers = {}
        for line in lines[1:]:
            if ':' in line:
                k, v = line.split(':', 1)
                headers[k.strip().lower()] = v.strip()
        if 'upgrade' in headers and headers['upgrade'].lower() == 'websocket':
            handle_ws(conn, headers)
        else:
            handle_http(conn, path)
    except (ConnectionError, OSError):
        pass
    finally:
        try:
            conn.close()
        except OSError:
            pass


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)
    print('[stub] 桩服务器监听 ws://%s:%d/ws ，%dHz 推送 %d 轴（Ctrl+C 退出）'
          % (HOST, PORT, PUSH_HZ, AXIS_COUNT))
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle_conn, args=(conn,), daemon=True).start()
    except KeyboardInterrupt:
        print('\n[stub] 退出')
    finally:
        srv.close()


if __name__ == '__main__':
    main()
