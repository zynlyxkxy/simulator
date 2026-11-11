# -*- coding: utf-8 -*-
"""
Filename: mqtt_receive_worker.py
Description:
    MQTT 消息监听/解析线程（对照参考框架 MqttReceiveWorker.py，
    裁剪到本迭代需要的两个话题）：
    - CTL_DR_ACS_Motion：task=CMD_Motion_RunTimeInfo 时遍历
      dic_Stage 的 inPos/moving 汇总台架运动状态（取值路径照参考）；
    - VSS_Com：context.content["1"].lstRow/lstCol[0] 取识别结果
      （取值路径照参考）；
    - 阻塞原语：wait_stage_stop()/wait_image_process()。

    与参考框架 MqttReceiveWorker.py 的刻意偏差（迭代 27，修 0 值污染
    统计问题）：
    - 参考语义：超时/失败一律返回哨兵 [0,0]，且只在 lstRow 非空时置
      应答标志（失败帧被静默丢弃 → 白等满超时）；本实现判失败不判零：
      失败帧同样置应答标志并记录原因，wait_image_process 失败/超时
      返回 None（绝不返回 [0,0]），由上层 take_photo 重拍/抛异常；
    - 参考无请求序号，迟到帧会被下一拍误收；本实现要求发送侧在信封
      task 字段塞序号（IMG_NNNNNN），VSS_Com 回显序号不匹配即判陈旧
      帧丢弃，继续等到超时为止；
    - 迭代 33：期望相机防线——reset 时登记期望 camID，VSS_Com 帧
      camID 不符（串相机/后端积压错位回帧）按陈旧帧同路径丢弃；
    - 迭代 42：默认超时 30s→15s（用户拍板直给口径：后端 REAL 帧上限
      12s + 3s 曝光/渲染余量，到点未回即判败，由 take_photo 重拍
      兜底，不再为积压场景留 30s 长等）；陈旧帧/camID 不符丢弃改走
      GUI 可见日志（log_cb → Operation Log），不再只 print 到控制台
"""
import json
import threading
import time


class MqttReceiveWorker:

    def __init__(self, mqtt_client, run_thread_event, log_cb=None):
        self.paho_client = mqtt_client
        self.run_thread_event = run_thread_event
        self.running = False
        self.lock = threading.Lock()
        # 迭代 42：GUI 可见日志回调（ScriptWidget.log → sig_log 落
        # Operation Log）；None 退化为 print（CLI/单测无宿主场景）
        self._log_cb = log_cb

        self._stage_moving = False

        self.receive_image_res = False
        self._image_res = None            # 成功帧 [row, col]；失败保持 None
        self._image_error = None          # 失败/超时原因（str）
        self._expect_image_task = None    # 期望的 VSS_Com 回显序号；None=不过滤
        self._expect_image_cam = None     # 期望的 camID（迭代 33）；None=不过滤
        self._also_accept = None          # 额外可接受序号集（迭代 48：同一次
                                          # take_photo 内上一 attempt 迟到帧）

    def _log(self, msg):
        """GUI 可见日志（迭代 42）：有 log_cb 走 log_cb（Qt 信号跨线程
        排队，工作线程内调用安全），否则退化 print。"""
        if self._log_cb is not None:
            try:
                self._log_cb(msg)
                return
            except Exception:
                pass
        print(msg)

    def __worker(self):
        """mqtt消息监听（迭代 48 批量排空：每轮最多取 50 条、队列非空
        不睡——闲置积压的 10Hz CTL 帧秒级排空；旧每轮 1 条+10ms 在
        积压场景 ~64~100 条/s，VSS 回帧排在几千条旧帧后必超时）。"""
        while True:
            if not self.running:
                break
            drained = 0
            while self.running and drained < 50:
                try:
                    msg = self.paho_client.mqttQueue.get_nowait()
                except Exception:                # queue.Empty
                    break
                drained += 1
                try:
                    self.parse_data(msg)
                except Exception as e:
                    # 单条坏帧不得杀死监听线程（否则后续所有
                    # wait_image_process/wait_stage_stop 静默超时）
                    print("[mqtt-recv] 跳过无法解析的消息(%s/%s): %s"
                          % (getattr(msg, "topic", "?"), e,
                             repr(getattr(msg, "payload", b"")[:120])))
            if drained == 0:
                self.run_thread_event.wait(0.01)
        print("mqtt thread exit")

    def start(self):
        # 迭代 48：先清闲置积压——worker 只在 run 期间消费，GUI 闲置时
        # 10Hz CTL 帧无限积压（闲 10 分钟≈6000 条），全为陈旧帧无消费
        # 价值，不清则本轮 VSS 回帧排在队尾必 15s 超时（静态超时主因）
        purged = 0
        try:
            while True:
                self.paho_client.mqttQueue.get_nowait()
                purged += 1
        except Exception:                        # queue.Empty
            pass
        if purged:
            self._log("已清空闲置积压 MQTT 帧 %d 条（CTL 周期帧，"
                      "非本次测校产生）" % purged)
        self.running = True
        self.t1 = threading.Thread(target=self.__worker, name="工作线程")
        self.t1.daemon = True
        self.t1.start()
        self.run_thread_event.wait(0.1)

    def stop(self):
        self.running = False

    def parse_data(self, msg):
        if not msg:
            return
        recv_raw = json.loads(msg.payload.decode('utf-8'))
        task = recv_raw.get('task')

        # stage info
        if msg.topic == 'CTL_DR_ACS_Motion' and task == 'CMD_Motion_RunTimeInfo':
            acs_info = json.loads(recv_raw['context'])
            stage_moving = False
            with self.lock:
                for stage_index, stage_info in acs_info['dic_Stage'].items():
                    if not stage_info['inPos'] or stage_info['moving']:
                        stage_moving = True
            self._stage_moving = stage_moving

        # vision
        if msg.topic == "VSS_Com":
            content = json.loads(recv_raw["context"])
            echo_task = content.get("task") or ""
            with self.lock:
                expect = self._expect_image_task
                expect_cam = self._expect_image_cam
                also = self._also_accept
            # 防陈旧帧：设了期望序号就只收回显同序号的帧（also_accept 内
            # 的序号同样收下——同一 take_photo 内上一 attempt 的迟到帧，
            # 同机位物理等价，迭代 48）；其余上一拍迟到帧序号不匹配，丢弃
            # 并让等待方继续等（迭代 42：丢弃走 GUI 可见日志）
            if expect is not None and echo_task != expect \
                    and echo_task not in (also or ()):
                self._log("丢弃陈旧 VSS_Com 帧（echo=%s expect=%s）"
                          % (echo_task or "∅", expect))
                return
            # 迭代 33 期望相机防线：设了期望 cam 且帧 camID 不符（串相机
            # 陈旧帧/后端积压错位回帧）按陈旧帧同路径丢弃（迭代 42：
            # 丢弃走 GUI 可见日志）
            frame_cam = content.get("camID")
            if expect_cam is not None and frame_cam != expect_cam:
                self._log("丢弃 camID 不符 VSS_Com 帧（camID=%s "
                          "expect=%s echo=%s）"
                          % (frame_cam, expect_cam, echo_task or "∅"))
                return
            # 判失败不判零：成败都置应答标志，绝不静默丢弃失败帧
            err = None
            row_col = None
            cell1 = {}
            if content.get('content') is not None:
                cell1 = (json.loads(content["content"]) or {}).get("1") or {}
            if cell1.get("error"):
                err = "识别失败: %s" % cell1["error"]
            elif not cell1.get("judge", content.get("judge", False)):
                err = "识别失败: judge=false"
            elif not cell1.get("lstRow") or not cell1.get("lstCol"):
                err = "识别失败: lstRow/lstCol 为空"
            else:
                row_col = [cell1["lstRow"][0], cell1["lstCol"][0]]
            with self.lock:
                self._image_res = row_col
                self._image_error = err
                self.receive_image_res = True

    def wait_stage_stop(self, timeout=30.0):
        """等台架停止（CTL RunTimeInfo 全 inPos 且不 moving）。
        迭代 48 加超时（默认 30s）：CTL 帧断流（broker 断线/积压堵塞）
        旧实现永久挂起——超时抛 RuntimeError 直给。"""
        self.run_thread_event.wait(0.1)
        t0 = time.time()
        while True and self.running:
            self.run_thread_event.wait(0.1)
            if not self._stage_moving:
                break
            if time.time() - t0 > timeout:
                raise RuntimeError(
                    "等待台架停止超时（>%gs）：CTL_DR_ACS_Motion 帧断流"
                    "（broker 断线或接收积压），请检查后端与网络" % timeout)

    def reset_image_process_status(self, task=None, cam=None,
                                   also_accept=None):
        """开始新一拍前调用：清应答标志，并登记期望的回显序号
        （task=None 表示不做序号过滤，兼容旧用法）与期望相机 camID
        （迭代 33，cam=None 不过滤；设了则 camID 不符的帧按陈旧帧丢弃）。
        also_accept（迭代 48）：额外可接受序号集——同一 take_photo 内
        上一 attempt 的迟到帧（同机位物理等价）有效收下，不再误判陈旧。"""
        with self.lock:
            self.receive_image_res = False
            self._image_res = None
            self._image_error = None
            self._expect_image_task = task
            self._expect_image_cam = cam
            self._also_accept = set(also_accept) if also_accept else None

    def last_image_error(self):
        """最近一次失败/超时原因（无则 None）。"""
        with self.lock:
            return self._image_error

    def wait_image_process(self, timeout=15):
        """等一拍识别结果。成功返回 [row, col]；识别失败或超时返回 None
        （失败原因经 last_image_error() 取）。
        默认 15s（迭代 42 用户拍板直给口径）：后端 REAL 帧上限 12s
        （REAL_FRAME_TIMEOUT_S）+ 3s 曝光/渲染余量；到点未回即判败，
        由上层 take_photo 重拍兜底，不再为积压场景留 30s 长等。"""
        b_time = time.time()
        self.run_thread_event.wait(1)
        while True:
            self.run_thread_event.wait(0.5)
            with self.lock:
                done = self.receive_image_res
                res = self._image_res
            if done:
                return res
            if time.time() - b_time > timeout or not self.running:
                with self.lock:
                    self._image_error = "拍照应答超时(>%gs)" % timeout
                print("image process timeout")
                return None

    def is_stage_moving(self):
        with self.lock:
            return self._stage_moving
