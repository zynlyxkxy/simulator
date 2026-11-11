# -*- coding: utf-8 -*-
"""
Filename: cmdjson/PahoMqttClient.py
Description:
    与参考框架 cmdjson/PahoMqttClient.py 同名同 API 的薄封装。
    venv 无 paho，底层改用后端 runtime 的标准库 MQTT 客户端
    （backend/runtime/mqtt_broker.py 的 MqttSub / mqtt_publish）：
    - 订阅侧：一条 MqttSub 长连接 + 后台读线程，把消息投递进 mqttQueue
      （队列元素带 .topic(str)/.payload(bytes)，与 paho 消息对象同形）；
    - 发布侧：mqtt_publish 一次性连接→发布→断开（QoS0 转发，与订阅
      连接互不影响，接收线程重连期间发布照常）。

    迭代 42 接收线程加固（修"接收线程静默死亡 → VSS_Com 回帧无人收 →
    拍照链干等超时"）：__reader 对 recv 异常不再静默 break——stderr
    打原因后进自动重连循环（退避 1s 起步 ×2 封顶 5s）：重建 socket →
    重新订阅全部已订阅 topic → 继续 recv；仅 close()/停止标志可终止
    接收线程。
"""
import os
import sys
import threading
import time
from queue import Queue

# 把后端 runtime 加进 sys.path 以复用标准库 MQTT 客户端
_RUNTIME_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir,
    "runtime"))
if _RUNTIME_DIR not in sys.path:
    sys.path.append(_RUNTIME_DIR)   # append：避免 runtime/main.py 遮蔽本包 main

from mqtt_broker import MqttSub, mqtt_publish  # noqa: E402


class _Msg(object):
    """与 paho MQTTMessage 同形的最小消息对象（topic/payload）。"""

    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = payload if isinstance(payload, bytes) \
            else str(payload).encode("utf-8")


class PahoMqttClient:
    pl = ''

    def __init__(self, log_cb=None):
        self.mqttQueue = Queue(-1)
        self.imgQueue = Queue(-1)
        self.lock = threading.RLock()
        self._sub = None
        self._sub_topics = []        # 已订阅 topic（迭代 42：重连后重新订阅用）
        self._connected = False
        self._reader_running = False
        self._reader_thread = None
        self._host = "127.0.0.1"
        self._port = 1883
        self._packet_id = 0
        # 迭代 48：重连事件可上 Operation Log（log_cb；QoS0 重连窗口丢帧
        # 是"莫名超时"的可观测线索，不再只打 stderr）；None 退 stderr
        self._log_cb = log_cb

    def _log(self, msg):
        if self._log_cb is not None:
            try:
                self._log_cb(msg)
                return
            except Exception:
                pass
        print(msg, file=sys.stderr)

    def connect(self, host, port):
        self._host, self._port = host, int(port)
        self._sub = MqttSub(host=self._host, port=self._port,
                            client_id="metrology-gui-sub")
        self._connected = True
        # 注意：接收线程不在此处启动——subscribe() 要在同一条 socket 上同步
        # 读 SUBACK，若读线程已跑会并发抢字节流（读到半个 PUBLISH/SUBACK
        # 即错帧，下游 json.loads 崩）。接收线程在 subscribe() 完成后启动。

    def __reader(self):
        """接收循环（迭代 42 加固）：recv 异常不再静默 break——stderr
        打原因后进入自动重连循环（__reconnect），恢复后续收；仅
        _reader_running 被 close() 置 False 时退出。"""
        while self._reader_running:
            try:
                got = self._sub.recv(timeout=0.5)
            except Exception as e:
                if not self._reader_running:
                    break          # 主动 close() 掐断的 recv，不算断线
                self._log("[mqtt-reader] 订阅链接收中断（%s: %s），进入自动"
                          "重连（重连窗口内 VSS_Com 回帧会丢失 QoS0，"
                          "拍照超时若与此刻对齐即此因）"
                          % (type(e).__name__, e))
                if not self.__reconnect():
                    break          # 重连/退避期间收到停止标志
                continue
            if got is None:
                continue
            topic, payload = got
            self.pl = payload
            self.mqttQueue.put(_Msg(topic, payload))
        print("[mqtt-reader] 接收线程退出", file=sys.stderr)

    def __reconnect(self):
        """自动重连循环：退避 1s 起步 ×2 封顶 5s——重建 MqttSub socket
        并重新订阅全部已订阅 topic，成功返回 True；重连/退避期间
        _reader_running 被 close() 置 False 返回 False（接收线程随之
        退出，不再重连）。

        竞态防线：
        - 退避 sleep 按 0.2s 切片，close() 后至多 0.2s 内响应退出；
        - self._sub 换边与 close() 的关断同走 self.lock 互斥，且换边
          前在锁内复查 _reader_running——close() 抢锁在前则弃掉新
          连接返回 False（不泄漏 socket）；
        - 新连接的 subscribe 握手在本接收线程内同步完成（沿用迭代 26
          "subscribe 完成后才起接收线程"前提：握手与 recv 永远不并发
          抢字节流）。"""
        backoff = 1.0
        while self._reader_running:
            deadline = time.time() + backoff
            while self._reader_running and time.time() < deadline:
                time.sleep(min(0.2, max(0.0, deadline - time.time())))
            if not self._reader_running:
                return False
            new_sub = None
            try:
                new_sub = MqttSub(host=self._host, port=self._port,
                                  client_id="metrology-gui-sub")
                for topic in self._sub_topics:
                    self._packet_id += 1
                    new_sub.subscribe(topic, packet_id=self._packet_id)
            except Exception as e:
                if new_sub is not None:
                    try:
                        new_sub.close()
                    except Exception:
                        pass
                self._log("[mqtt-reader] 重连失败（%s: %s），%gs 后重试"
                          % (type(e).__name__, e, min(backoff * 2, 5.0)))
                backoff = min(backoff * 2, 5.0)
                continue
            with self.lock:
                if not self._reader_running:
                    # close() 抢锁在前：弃新连接，让接收线程退出
                    try:
                        new_sub.close()
                    except Exception:
                        pass
                    return False
                old_sub, self._sub = self._sub, new_sub
            if old_sub is not None:
                try:
                    old_sub.close()
                except Exception:
                    pass
            self._log("[mqtt-reader] 重连成功：已重建 socket 并重新订阅 %s"
                      % list(self._sub_topics))
            return True
        return False

    # publish 消息
    def publish(self, topic, payload, qos):
        # 后端迷你 broker 只按 QoS0 转发；qos 参数仅保留 API 形状
        mqtt_publish(topic, payload, host=self._host, port=self._port)

    # 消息处理函数（保留参考 API 形状；实际入队由 __reader 完成）
    def on_message_come(self, client, userdata, msg):
        self.pl = msg.payload.decode()
        self.mqttQueue.put(msg)

    # subscribe 消息
    def subscribe(self, topics):
        for topic in topics:
            self._packet_id += 1
            self._sub.subscribe(topic, packet_id=self._packet_id)
            if topic not in self._sub_topics:
                # 登记已订阅 topic（迭代 42：断线重连后自动重新订阅）
                self._sub_topics.append(topic)
        print("subscribe %s" % topics)
        # 全部订阅完成后才启动接收线程（见 connect() 注释）
        if not self._reader_running:
            self._reader_running = True
            self._reader_thread = threading.Thread(
                target=self.__reader, name="mqtt-reader")
            self._reader_thread.daemon = True
            self._reader_thread.start()

    # unsubscribe 消息（迷你 broker 无 UNSUBSCRIBE，仅停止入队该主题）
    def unsubscribe(self, topics):
        print("unsubscribe %s（broker 不支持 UNSUBSCRIBE，仅本端忽略）"
              % topics)

    def send_cmd(self, payload, topic="VisionRcp", qos=2):
        self.publish(topic, payload, qos)

    def rcv_res(self):
        msg = self.mqttQueue.get()
        return msg

    def async_rcv_res(self):
        if self.mqttQueue.empty():
            return ' '
        self.lock.acquire()
        msg = self.mqttQueue.get()
        data = msg.payload.decode()
        self.lock.release()
        return data

    def rcv_img(self):
        return self.imgQueue.get()

    def close(self):
        self._reader_running = False
        with self.lock:
            # 与 __reconnect 的换边互斥：关断的一定是接收线程正在用的
            # 那条连接；被掐断的 recv 在 __reader 里看到停止标志即退出
            if self._sub is not None:
                self._sub.close()
        self._connected = False

    def get_connect_statue(self):
        return self._connected
