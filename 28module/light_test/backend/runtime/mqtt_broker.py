# -*- coding: utf-8 -*-
"""mqtt_broker.py — 迷你 MQTT 3.1.1 broker（纯标准库，默认端口 1883）。

视觉链路的结果出口：视觉服务把识别结果 JSON 以 QoS0 发布到
`vision/result/{cam}`，上位机将来可用**真 MQTT 客户端直连 1883**（真协议，
非自定义 JSON——这是与真机链路对齐的迁移路径）。

支持的协议子集（对常规客户端够用）：
- CONNECT / CONNACK（MQTT 3.1.1，level 4；clean session；不支持 will/鉴权，
  收到也忽略并按成功回执）
- SUBSCRIBE / SUBACK（精确主题 + 单层通配 `+` + 多层通配 `#`；granted QoS
  一律回 0——转发只按 QoS0）
- PUBLISH：QoS0 直接转发给所有匹配订阅者；QoS1 回 PUBACK 后按 QoS0 转发；
  **QoS2 完整支持（迭代 37 补丁）**：PUBLISH→**PUBREC**→PUBREL→**PUBCOMP**
  四段握手，报文暂存于连接级 inflight 表，**PUBREL 时刻才恰好一次转发**
  （重复 PUBLISH 只重发 PUBREC 不重复投递；已完成包 id 进短期 completed
  表防迟到重发造成重复投递；inflight/completed 均 60s 惰性过期、上限
  1024 条——真机视觉链路上位机若用 QoS2 发布结果，协议行为与真 broker
  对齐）。订阅侧仍按 QoS0 下发（granted QoS0 语义不变）。
- PINGREQ / PINGRESP；DISCONNECT 关连接

每连接一个线程；broker 自身也是一个裸 socket 服务（与 ws_gateway 同风格，
不设 SO_REUSEADDR——Windows 双绑定排查极坑）。

附带两个轻量客户端帮手（供视觉服务发布与回归测试订阅）：
- mqtt_publish(topic, payload)：一次性连接→发布→断开；
- MqttSub：订阅类，recv(timeout) 逐条取 (topic, payload)。

自测：`python runtime/mqtt_broker.py --selftest`
（临时端口起 broker，订阅/发布回环真 socket，通/断/通配全覆盖，进回归）。
"""
import socket
import struct
import threading
import time

# ---- 控制报文类型（高 4 位） ----
CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
PUBREC, PUBREL, PUBCOMP = 5, 6, 7
SUBSCRIBE, SUBACK = 8, 9
PINGREQ, PINGRESP, DISCONNECT = 12, 13, 14

# QoS2 inflight/completed 表条目有效期（s）与容量上限（迭代 37 补丁）
_QOS2_TTL = 60.0
_QOS2_MAX = 1024


def _enc_remaining(n):
    """MQTT 剩余长度 varint 编码。"""
    out = b""
    while True:
        d = n % 128
        n //= 128
        if n:
            out += bytes([d | 0x80])
        else:
            return out + bytes([d])


def _enc_str(s):
    b = s.encode("utf-8")
    return struct.pack("!H", len(b)) + b


def build_publish(topic, payload, qos=0, packet_id=None):
    """构造 PUBLISH 报文（QoS0/1）。payload 可为 str/bytes。"""
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    body = _enc_str(topic)
    if qos > 0:
        body += struct.pack("!H", packet_id or 1)
    body += payload
    return bytes([PUBLISH << 4 | (qos << 1)]) + _enc_remaining(len(body)) + body


def topic_match(filter_, topic):
    """订阅过滤器匹配：精确 / `+` 单层 / `#` 多层（# 须为最后一段）。"""
    if filter_ == "#":
        return True
    fp = filter_.split("/")
    tp = topic.split("/")
    for i, seg in enumerate(fp):
        if seg == "#":
            return i == len(fp) - 1          # # 只允许收尾
        if i >= len(tp):
            return False
        if seg != "+" and seg != tp[i]:
            return False
    return len(fp) == len(tp)


class _Conn(object):
    def __init__(self, broker, conn, addr):
        self.broker = broker
        self.conn = conn
        self.addr = addr
        self.alive = True
        self.filters = set()           # 本连接订阅的过滤器
        self.qos2 = {}                 # 迭代 37：QoS2 状态表 {packet_id:
                                       # ("wait_rel", topic, payload, ts) |
                                       # ("done", "", b"", ts)}——wait_rel=
                                       # 已收 PUBLISH 待发 PUBREC 握完，done=
                                       # 已恰好一次投递，防迟到重发重复投递
        self._wlock = threading.Lock()

    def send(self, data):
        with self._wlock:
            self.conn.sendall(data)

    def reap_qos2(self, now=None):
        """惰性过期 QoS2 状态表（在 publish/pubrel 处理时顺带调用）。"""
        now = time.time() if now is None else now
        dead = [p for p, e in self.qos2.items() if now - e[3] > _QOS2_TTL]
        for p in dead:
            del self.qos2[p]


class MqttBroker(object):
    """迷你 broker：start() 后台接受连接；stop() 关闭。"""

    def __init__(self, host="127.0.0.1", port=1883):
        self.host = host
        self.port = port
        self._srv = None
        self._subs = []                # [_Conn] 当前有订阅的连接
        self._lock = threading.Lock()
        self.stats = {"connects": 0, "publishes": 0, "forwards": 0}

    def start(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind((self.host, self.port))
        srv.listen(32)
        srv.settimeout(0.5)
        self._srv = srv
        threading.Thread(target=self._accept_loop, name="mqtt-accept",
                         daemon=True).start()
        print("[mqtt] mini broker on %s:%d" % (self.host, self.port),
              flush=True)
        return self

    def stop(self):
        try:
            if self._srv:
                self._srv.close()
        except OSError:
            pass
        with self._lock:
            subs = list(self._subs)
        for c in subs:
            try:
                c.conn.close()
            except OSError:
                pass

    def _accept_loop(self):
        while True:
            try:
                conn, addr = self._srv.accept()
            except (OSError, socket.timeout):
                if self._srv is None:
                    break
                continue
            threading.Thread(target=self._session, args=(conn, addr),
                             daemon=True).start()

    # ---------------- 会话 ----------------
    def _session(self, conn, addr):
        c = _Conn(self, conn, addr)
        conn.settimeout(120)
        try:
            while c.alive:
                try:
                    pkt = self._read_packet(conn)
                except socket.timeout:
                    # 读闲置不杀连接：订阅端（vision-svc/vision-cli/confocal
                    # 等）只收不发，120s 无上行报文是常态；此前超时走外层
                    # except 直接断链，长会话里视觉/共焦 MQTT 请求会正好落
                    # 在断链窗口丢消息（QoS0 无重发）→ 流程 10s 等不到结果
                    # abort。连接死活由 eof / 发送失败判定。
                    continue
                if pkt is None:
                    break
                ptype, flags, body = pkt
                if ptype == CONNECT:
                    self._on_connect(c, body)
                elif ptype == SUBSCRIBE:
                    self._on_subscribe(c, body)
                elif ptype == PUBLISH:
                    self._on_publish(c, flags, body)
                elif ptype == PUBREL:
                    self._on_pubrel(c, body)
                elif ptype == PINGREQ:
                    c.send(bytes([PINGRESP << 4, 0]))
                elif ptype == DISCONNECT:
                    break
                # 其余类型（UNSUBSCRIBE 等）忽略：保持连接不报错
        except (ConnectionError, socket.timeout, OSError, ValueError):
            pass
        finally:
            c.alive = False
            with self._lock:
                if c in self._subs:
                    self._subs.remove(c)
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _read_packet(conn):
        """读一个控制报文 -> (type, flags, body)；对端断开返回 None。"""
        b1 = b""
        while not b1:
            b1 = conn.recv(1)
            if not b1:
                return None
        mult, shift, rem = 1, 0, 0
        while True:                     # 剩余长度 varint
            b = conn.recv(1)
            if not b:
                return None
            d = b[0]
            rem += (d & 0x7F) << shift
            shift += 7
            if not (d & 0x80):
                break
            mult += 1
            if mult > 4:
                raise ValueError("remaining length varint too long")
        body = b""
        while len(body) < rem:
            chunk = conn.recv(min(1 << 20, rem - len(body)))
            if not chunk:
                return None
            body += chunk
        return b1[0] >> 4, b1[0] & 0x0F, body

    # ---------------- 报文处理 ----------------
    def _on_connect(self, c, body):
        # 可变头：协议名/级别/标志/keepalive——校验 "MQTT"+level4，其余宽容
        if len(body) < 12 or body[2:6] != b"MQTT" or body[6] != 4:
            # 协议不接受：CONNACK rc=1，然后关断
            c.send(bytes([CONNACK << 4, 2, 0, 1]))
            raise ConnectionError("bad CONNECT")
        c.send(bytes([CONNACK << 4, 2, 0, 0]))     # session present=0, rc=0
        self.stats["connects"] += 1

    def _on_subscribe(self, c, body):
        if len(body) < 3:
            return
        packet_id = struct.unpack("!H", body[:2])[0]
        i, granted = 2, []
        while i < len(body):
            tlen = struct.unpack("!H", body[i:i + 2])[0]
            topic = body[i + 2:i + 2 + tlen].decode("utf-8", "replace")
            i += 2 + tlen
            _qos = body[i] if i < len(body) else 0
            i += 1
            c.filters.add(topic)
            granted.append(0)           # 一律授予 QoS0
        with self._lock:
            if c not in self._subs:
                self._subs.append(c)
        c.send(bytes([SUBACK << 4]) + _enc_remaining(2 + len(granted))
               + struct.pack("!H", packet_id) + bytes(granted))

    def _on_publish(self, c, flags, body):
        qos = (flags >> 1) & 0x3
        if len(body) < 2:
            return
        tlen = struct.unpack("!H", body[:2])[0]
        topic = body[2:2 + tlen].decode("utf-8", "replace")
        i = 2 + tlen
        packet_id = None
        if qos:
            packet_id = struct.unpack("!H", body[i:i + 2])[0]
            i += 2
        payload = body[i:]
        self.stats["publishes"] += 1
        if qos == 1:
            # QoS1：PUBACK 后按 QoS0 转发（至多一次语义，与既有行为一致）
            c.send(bytes([PUBACK << 4, 2]) + struct.pack("!H", packet_id))
        elif qos == 2:
            # 迭代 37 QoS2 补丁：PUBLISH 只入 inflight 表并回 PUBREC，
            # 此刻**不转发**；恰好一次投递发生在 _on_pubrel（PUBREL）。
            c.reap_qos2()
            ent = c.qos2.get(packet_id)
            if ent is None or ent[0] == "wait_rel":
                if ent is None:
                    # 容量上限：丢最旧（异常 sender 不复位时防膨胀）
                    if len(c.qos2) >= _QOS2_MAX:
                        c.reap_qos2(now=time.time() - 0)   # 先惰性清理
                    if len(c.qos2) >= _QOS2_MAX:
                        oldest = min(c.qos2, key=lambda p: c.qos2[p][3])
                        del c.qos2[oldest]
                    c.qos2[packet_id] = ("wait_rel", topic, payload,
                                         time.time())
                # ent[0]=="done"：迟到重发——不回存、不重投，只重发 PUBREC
            else:
                # 已完成（"done"）的迟到重发：只重发 PUBREC 等待 PUBREL
                pass
            c.send(bytes([PUBREC << 4, 2]) + struct.pack("!H", packet_id))
            return
        self._forward(topic, payload)

    def _on_pubrel(self, c, body):
        """QoS2 第三段：PUBREL → 恰好一次转发（如尚未投递）+ PUBCOMP。"""
        if len(body) < 2:
            return
        packet_id = struct.unpack("!H", body[:2])[0]
        c.reap_qos2()
        ent = c.qos2.get(packet_id)
        if ent is not None and ent[0] == "wait_rel":
            _, topic, payload, _ts = ent
            self._forward(topic, payload)          # 恰好一次投递点
            c.qos2[packet_id] = ("done", "", b"", time.time())
        # ent 缺失（已过期/重复 PUBREL）或已 "done"：只回 PUBCOMP 不重投
        c.send(bytes([PUBCOMP << 4, 2]) + struct.pack("!H", packet_id))

    def _forward(self, topic, payload):
        """按订阅匹配 QoS0 下发给所有匹配连接（订阅侧 granted QoS 恒 0）。"""
        out = build_publish(topic, payload)
        with self._lock:
            subs = list(self._subs)
        for s in subs:
            if any(topic_match(f, topic) for f in s.filters):
                try:
                    s.send(out)
                    self.stats["forwards"] += 1
                except OSError:
                    s.alive = False


# ---------------- 轻量客户端帮手 ----------------
def _recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("eof")
        buf += chunk
    return buf


def _read_packet_once(conn):
    """读一个控制报文 → (ptype, flags, body)（mqtt_publish 握手用）。"""
    b1 = _recv_exact(conn, 1)[0]
    rem, shift = 0, 0
    while True:
        d = _recv_exact(conn, 1)[0]
        rem += (d & 0x7F) << shift
        shift += 7
        if not (d & 0x80):
            break
    body = _recv_exact(conn, rem) if rem else b""
    return b1 >> 4, b1 & 0x0F, body


def mqtt_publish(topic, payload, host="127.0.0.1", port=1883, timeout=5.0,
                 qos=0, packet_id=1):
    """一次性连接 → CONNECT → PUBLISH(qos=0/1/2) → DISCONNECT。返回 True。

    迭代 37：qos=1 等 PUBACK；qos=2 走完整四段握手（PUBLISH→PUBREC→
    PUBREL→PUBCOMP），供真机对齐/回归验证使用。生产发布仍用默认 QoS0。"""
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    conn = socket.create_connection((host, port), timeout=timeout)
    try:
        body = _enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 30) \
            + _enc_str("sim-pub-%d" % (time.time_ns() & 0xFFFFFF))
        conn.sendall(bytes([CONNECT << 4]) + _enc_remaining(len(body)) + body)
        connack = _recv_exact(conn, 4)
        if connack[0] >> 4 != CONNACK or connack[3] != 0:
            raise ConnectionError("CONNACK refused: %r" % connack)
        conn.sendall(build_publish(topic, payload, qos=qos,
                                   packet_id=packet_id))
        if qos == 1:
            pt, _f, _b = _read_packet_once(conn)
            if pt != PUBACK:
                raise ConnectionError("QoS1 期望 PUBACK，收到 type=%d" % pt)
        elif qos == 2:
            pt, _f, b = _read_packet_once(conn)
            if pt != PUBREC or struct.unpack("!H", b[:2])[0] != packet_id:
                raise ConnectionError("QoS2 期望 PUBREC，收到 type=%d" % pt)
            conn.sendall(bytes([PUBREL << 4 | 0x02, 2])
                         + struct.pack("!H", packet_id))
            pt, _f, b = _read_packet_once(conn)
            if pt != PUBCOMP or struct.unpack("!H", b[:2])[0] != packet_id:
                raise ConnectionError("QoS2 期望 PUBCOMP，收到 type=%d" % pt)
        conn.sendall(bytes([DISCONNECT << 4, 0]))
        return True
    finally:
        try:
            conn.close()
        except OSError:
            pass


class MqttSub(object):
    """订阅客户端（测试/调试用）：connect → subscribe(filter) → recv() 逐条取。"""

    def __init__(self, host="127.0.0.1", port=1883, client_id="sim-sub"):
        self.conn = socket.create_connection((host, port), timeout=5)
        self.conn.settimeout(None)
        self._pending = []             # subscribe 握手期间夹带的 PUBLISH 缓存
        body = _enc_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", 30) \
            + _enc_str(client_id)
        self.conn.sendall(bytes([CONNECT << 4]) + _enc_remaining(len(body))
                          + body)
        connack = _recv_exact(self.conn, 4)
        if connack[0] >> 4 != CONNACK or connack[3] != 0:
            raise ConnectionError("CONNACK refused: %r" % connack)

    def _read_packet(self):
        """读一个控制报文 → (ptype, body)。对端断开抛 ConnectionError。"""
        b1 = _recv_exact(self.conn, 1)[0]
        rem, shift = 0, 0
        while True:
            d = _recv_exact(self.conn, 1)[0]
            rem += (d & 0x7F) << shift
            shift += 7
            if not (d & 0x80):
                break
        body = _recv_exact(self.conn, rem) if rem else b""
        return b1 >> 4, body

    @staticmethod
    def _parse_publish(body):
        tlen = struct.unpack("!H", body[:2])[0]
        topic = body[2:2 + tlen].decode("utf-8")
        return topic, body[2 + tlen:].decode("utf-8")

    def subscribe(self, topic_filter, packet_id=1):
        self.conn.settimeout(None)      # recv 可能留了超时；握手须阻塞等
        body = struct.pack("!H", packet_id) + _enc_str(topic_filter) + b"\x00"
        self.conn.sendall(bytes([SUBSCRIBE << 4 | 0x02])
                          + _enc_remaining(len(body)) + body)
        # 等 SUBACK：已订阅话题（如同连接上前一个 subscribe 生效的话题，
        # 10Hz RunTimeInfo）的 PUBLISH 可能夹在 SUBACK 之前到达——缓存进
        # _pending 交后续 recv() 取用，不得误判为 SUBACK 报错断链
        # （迭代 27 联调实测：GUI 订阅 CTL+VSS 两话题时 ~10% 概率踩中）。
        while True:
            ptype, pbody = self._read_packet()
            if ptype == SUBACK:
                return
            if ptype == PUBLISH:
                self._pending.append(self._parse_publish(pbody))
            # 其余报文（PINGRESP 等）跳过继续等

    def recv(self, timeout=5.0):
        """等一条 PUBLISH -> (topic, payload str)；超时返回 None。"""
        if self._pending:
            return self._pending.pop(0)
        self.conn.settimeout(timeout)
        try:
            while True:
                ptype, body = self._read_packet()
                if ptype == PUBLISH:
                    return self._parse_publish(body)
                # 其他报文（PINGRESP 等）跳过继续等
        except (socket.timeout, TimeoutError):
            return None

    def close(self):
        try:
            self.conn.sendall(bytes([DISCONNECT << 4, 0]))
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass


# ---------------- 自测（进回归） ----------------
def _selftest():
    """真 socket 回环：broker（临时端口）+ MqttSub + mqtt_publish。"""
    broker = MqttBroker(port=0)
    broker.start()
    port = broker._srv.getsockname()[1]
    try:
        sub = MqttSub(port=port, client_id="selftest-sub")
        sub.subscribe("vision/result/+")          # 单层通配
        sub2 = MqttSub(port=port, client_id="selftest-sub2")
        sub2.subscribe("vision/#")                # 多层通配
        time.sleep(0.1)
        mqtt_publish("vision/result/down", '{"judge":true}', port=port)
        got = sub.recv(timeout=3)
        assert got is not None and got[0] == "vision/result/down" \
            and '"judge":true' in got[1], "sub 未收到: %r" % (got,)
        got2 = sub2.recv(timeout=3)
        assert got2 is not None and got2[0] == "vision/result/down", \
            "sub2(#) 未收到: %r" % (got2,)
        # 单层通配不跨层：vision/result 不匹配 vision/result/+
        mqtt_publish("vision/result", "x", port=port)
        got3 = sub.recv(timeout=0.5)
        assert got3 is None, "单层通配越层误匹配: %r" % (got3,)
        # PINGREQ
        sub.conn.sendall(bytes([PINGREQ << 4, 0]))
        pong = sub.recv(timeout=2)
        assert pong is None                      # recv 只吃 PUBLISH；PINGRESP 被跳过
        assert broker.stats["publishes"] >= 2
        sub.close()
        sub2.close()
        # ---- 迭代 37：QoS2 四段握手 ----
        sub3 = MqttSub(port=port, client_id="selftest-qos2")
        sub3.subscribe("vision/result/+")
        time.sleep(0.1)
        n_fwd0 = broker.stats["forwards"]
        # 1) mqtt_publish(qos=2) 完整握手 → 订阅端恰好收到一次
        mqtt_publish("vision/result/up", '{"q":2}', port=port, qos=2,
                     packet_id=7)
        g = sub3.recv(timeout=3)
        assert g is not None and g[0] == "vision/result/up" \
            and '"q":2' in g[1], "QoS2 订阅端未收到: %r" % (g,)
        assert sub3.recv(timeout=0.4) is None, "QoS2 重复投递"
        # 2) 裸连接：PUBLISH 重发（sender 没收到 PUBREC）→ 只回 PUBREC、
        #    PUBREL 后恰好一次投递（转发计数恰好 +1）
        raw = socket.create_connection(("127.0.0.1", port), timeout=5)
        try:
            def _connect_raw():
                b = _enc_str("MQTT") + bytes([4, 0x02]) \
                    + struct.pack("!H", 30) + _enc_str("raw-qos2")
                raw.sendall(bytes([CONNECT << 4]) + _enc_remaining(len(b))
                            + b)
                ca = _recv_exact(raw, 4)
                assert ca[0] >> 4 == CONNACK and ca[3] == 0, repr(ca)
            _connect_raw()
            body = _enc_str("vision/result/down") \
                + struct.pack("!H", 8) + b'retry'
            pkt = bytes([PUBLISH << 4 | (2 << 1)]) \
                + _enc_remaining(len(body)) + body
            raw.sendall(pkt)                          # 首投 pid=8
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBREC and struct.unpack("!H", pb)[0] == 8, \
                "期望 PUBREC，得到 type=%d" % pt
            raw.sendall(bytes([PUBLISH << 4 | 0x08 | (2 << 1)])
                        + _enc_remaining(len(body)) + body)   # DUP=1 重发
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBREC and struct.unpack("!H", pb)[0] == 8, \
                "重发应再收 PUBREC，得到 type=%d" % pt
            raw.sendall(bytes([PUBREL << 4 | 0x02, 2])
                        + struct.pack("!H", 8))
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBCOMP and struct.unpack("!H", pb)[0] == 8, \
                "期望 PUBCOMP，得到 type=%d" % pt
            g = sub3.recv(timeout=3)
            assert g is not None and g[1] == "retry", "PUBREL 后未投递: %r" \
                % (g,)
            assert broker.stats["forwards"] == n_fwd0 + 2, \
                "重发造成了重复转发: %d -> %d" % (n_fwd0,
                                              broker.stats["forwards"])
            # 3) 迟到重发（已完成后再收同 pid PUBLISH+PUBREL）→ 不再投递
            raw.sendall(bytes([PUBLISH << 4 | 0x08 | (2 << 1)])
                        + _enc_remaining(len(body)) + body)
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBREC, "迟到重发应回 PUBREC，type=%d" % pt
            raw.sendall(bytes([PUBREL << 4 | 0x02, 2])
                        + struct.pack("!H", 8))
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBCOMP, "迟到重发应回 PUBCOMP，type=%d" % pt
            assert sub3.recv(timeout=0.4) is None, "迟到重发重复投递"
            # 4) 未知 pid 的 PUBREL → 只回 PUBCOMP 不投递不崩
            raw.sendall(bytes([PUBREL << 4 | 0x02, 2])
                        + struct.pack("!H", 4242))
            pt, _f, pb = _read_packet_once(raw)
            assert pt == PUBCOMP and struct.unpack("!H", pb)[0] == 4242, \
                "未知 PUBREL 应回 PUBCOMP，type=%d" % pt
            raw.sendall(bytes([DISCONNECT << 4, 0]))
        finally:
            try:
                raw.close()
            except OSError:
                pass
        sub3.close()
        print("mqtt_broker selftest OK (port %d, stats %s)"
              % (port, broker.stats))
        return True
    finally:
        broker.stop()


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(0 if _selftest() else 1)
    MqttBroker().start()
    print("broker running (Ctrl+C 退出)")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
