# -*- coding: utf-8 -*-
"""mqtt_cmd_adapter.py — 测校线协议适配器（迭代 26）。

复刻参考测校框架 HoriMetrologyScirptsFramework 的设备通讯链（MQTT 命令/
反馈 topic + 双层 JSON 信封），让按该框架标准写的测校软件能直接驱动仿真
后端——将来换真机只改 broker 地址。

事实源（参考工程，只读）：
- cmdjson/MC_Motion_cmd.py   运动命令信封：外层 {task, context(json str),
  result}；context 内 {name, stage, enable, inPos, moving, lst_Axis:[{name,
  moveAbs, moveRel, vel{targetVal,value}, acc..., unit}]}。
- cmdjson/Camera_CMD.py      视觉命令 VisionCmdJson：外层 {context(json str),
  module, result, task}；context 内 {camID, cmdType, content(json str:
  {"0":{曝光参数}, "1":{lstShapeName:[模板名]}}), image, imgType,
  isImageSend, lstCellType, task}。
- MqttReceiveWorker.py       反馈解析逻辑（本适配器发布的帧必须能被它原样
  消费）：
  * CTL_DR_ACS_Motion / task=CMD_Motion_RunTimeInfo →
    json.loads(context)["dic_Stage"] 逐 stage：stage_info["inPos"] /
    ["moving"] / ["lst_Axis"][i]["coord"]["value"]（wait_stage_stop 依据：
    所有 stage inPos 且非 moving）。
  * VSS_Com → json.loads(context)：["content"] 再 json.loads → ["1"]
    ["lstRow"]/["lstCol"] 非空即取 [0]；["image"] 非空则 base64 解码成图。

订阅（命令入口）：
- GUI_DR_ACS_Motion：CMD_Motion_MoveAbs / CMD_Motion_MoveRel（lst_Axis 每项
  轴名经 mqtt_cmd_map.json 映射 → engine.move；MoveRel 换算绝对目标）。
  其余 task（Home/SetVel/IO/FLS…）不崩、记日志，并向 CTL_DR_ACS_Motion 回
  {task 原样, result: false, context: {"error": ...}} 错误帧。
- VSC_Com：VisionCmdJson → vision.identify(camID, 模板名)（camID 支持仿真
  相机 id up/down/coarse 直传，也兼容参考 int 编码 0/1/2——映射为仿真
  假设，真机 camID 约定待同事确认）→ VSS_Com 结果帧
  （lstRow/lstCol/lstAngle/lstScore/lstScale/judge/strShapeName 齐全）。

发布：
- CTL_DR_ACS_Motion：CMD_Motion_RunTimeInfo 周期帧（RUNTIME_HZ=10Hz），
  dic_Stage 按 stage 分组（STAGES），轴条目字段与参考 lst_Axis 同构
  （coord.value=当前 pos、coord.targetVal=target、vel.value=当前 vel）。
- VSS_Com：识别结果帧（见上）；识别失败回 judge=false + 空 lst + error；
  camID 未知/命令未捕获异常同样回 judge=false 错误帧（2026-09-10 迭代 31，
  原只 print 静默，测校软件干等超时）。

线程模型：motion/vision 各一条订阅线程（broker 断线退避重连，同
vision_service._mqtt_loop 模式）+ 一条 10Hz 反馈线程；视觉命令按
相机分发到 per-cam 队列、每相机一条 daemon 消费线程（迭代 33，
同相机串行/跨相机并发/积压丢最旧）；全部 daemon，绝不
触碰 1kHz 物理线程。main.py 在 broker 起来之后 start()；环境变量
D2W_MQTT_CMD_ADAPTER=0 可关（默认开，资源占用极小）。
"""
import json
import os
import queue
import sys
import threading
import time

from mqtt_broker import MqttSub, mqtt_publish

ROOT = os.path.dirname(os.path.abspath(__file__))
MAP_JSON = os.path.join(ROOT, "mqtt_cmd_map.json")

# 参考框架 topic 契约
TOPIC_MOTION_CMD = "GUI_DR_ACS_Motion"       # 测校软件 → 设备（运动命令）
TOPIC_MOTION_FB = "CTL_DR_ACS_Motion"        # 设备 → 测校软件（运行反馈）
TOPIC_VISION_CMD = "VSC_Com"                 # 测校软件 → 视觉（识别命令）
TOPIC_VISION_FB = "VSS_Com"                  # 视觉 → 测校软件（识别结果）

RUNTIME_HZ = 10.0                            # RunTimeInfo 周期反馈频率

# 迭代 33 视觉命令按相机串行、跨相机并发：每条相机一条消费线程 +
# 一个命令队列；单线程串行处理期间其它相机的命令在 broker 排队、
# 慢 identify 把全部相机测校拖超时的现场问题由此消除。同相机队列
# 积压上限（待处理 ≥ 该值时丢弃最旧命令并回"被新命令取代"错误帧，
# 错误帧回显被丢命令的原始 task）——GUI 15s 超时重发会加剧积压，
# 有界队列让旧命令尽快失败而不是排队 12s+。
VISION_BACKLOG_MAX = 2

# 参考 VisionCmdJson 的 camID 是 int（VisionCmdJsonHardTrigger 里 int(id)）；
# 仿真侧允许直传相机 id 字符串，两者都收。
# 注意：0/1/2 → down/up/coarse 是仿真假设（2026-09-10 迭代 31 标注）——
# 与真机 camID 约定待同事确认，真机接入前以同事视觉软件实际编码为准。
CAM_ID_MAP = {0: "down", 1: "up", 2: "coarse"}

# 反馈 stage 分组：(stage_id, stage_name, [仿真轴名...])。
# 参考 MqttReceiveWorker 只依赖 stage_info["inPos"]/["moving"]/
# ["lst_Axis"][i]["coord"]["value"]，分组语义对解析透明；id 沿用参考
# 1001+ 号段风格。
STAGES = [
    (1001, "WaferStage", ["WS_LS_X", "WS_LS_Y", "WS_Rz"]),
    (1003, "CoarseMoveStage", ["FA_X", "FA_Z", "FA_Rx", "CH_Rz"]),
    (1004, "AlignStage", ["AS_X", "AS_Y", "AS_Z", "AS_Rx", "AS_Ry"]),
    (1005, "FilmStage", ["ES_X", "ES_Y", "ES_PunchZ"]),
    (1006, "BondHead", ["BS_Arm_Z", "BS_CAL_Rx", "BS_CAL_Ry"]),
    (1007, "EpinStage", ["E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"]),
]


def _empty_result(shape):
    """judge=false + 空 lst 错误结果（识别失败/未知 camID/命令异常回帧用）。"""
    return {"strShapeName": shape or "", "judge": False,
            "lstCol": [], "lstRow": [], "lstAngle": [],
            "lstScore": [], "lstScale": []}


def _axis_entry(name, pos, target, vel, unit):
    """参考 lst_Axis 条目同构（MqttReceiveWorker 只取 coord.value，其余字段
    补齐保持信封兼容）。"""
    return {
        "acc": {"targetVal": 0, "value": 0},
        "coord": {"targetVal": round(target, 6), "value": round(pos, 6)},
        "dec": {"targetVal": 0, "value": 0},
        "homeOffset": 0,
        "jerk": {"targetVal": 0, "value": 0},
        "jogRate": 0,
        "moveAbs": round(target, 6), "moveRel": 0,
        "name": name,
        "unit": unit,
        "vel": {"targetVal": 0, "value": round(vel, 6)},
    }


class MqttCmdAdapter(object):
    """测校线协议适配服务。main.py 注入 engine/vision 后 start()。"""

    def __init__(self, engine, vision, host="127.0.0.1", port=1883,
                 map_path=MAP_JSON):
        self.engine = engine
        self.vision = vision
        self.host = host
        self.port = port
        self._stop = False
        self.axis_map = self._load_map(map_path)
        # 迭代 33：per-cam 视觉命令队列 + 消费线程（懒启动，见
        # _dispatch_vision_cmd/_vision_consumer）
        self._vision_queues = {}             # cam -> queue.Queue
        self._vq_lock = threading.Lock()

    # ---------------- 轴名映射 ----------------
    @staticmethod
    def _load_map(path):
        try:
            with open(path, encoding="utf-8") as f:
                m = json.load(f).get("map", {})
            return {str(k): str(v) for k, v in m.items()}
        except (OSError, ValueError):
            print("[mqtt-cmd] 警告：%s 缺失/坏，轴名映射退化为恒等" % path,
                  flush=True)
            return {}

    def map_axis(self, stage_name, axis_name):
        """(参考 stage 名, 参考轴名) → 仿真轴名。查找顺序：
        "stage/axis" → axis → stage → 未命中恒等透传（axis 为空用 stage）。"""
        stage_name = stage_name or ""
        axis_name = axis_name or ""
        for key in ("%s/%s" % (stage_name, axis_name), axis_name, stage_name):
            if key and key in self.axis_map:
                return self.axis_map[key]
        fallback = axis_name or stage_name
        print("[mqtt-cmd] 轴名未映射，恒等透传: stage=%r axis=%r"
              % (stage_name, axis_name), flush=True)
        return fallback

    # ---------------- 生命周期 ----------------
    def start(self, host=None, port=None):
        if host:
            self.host = host
        if port:
            self.port = port
        threading.Thread(target=self._motion_loop, name="mqtt-cmd-motion",
                         daemon=True).start()
        threading.Thread(target=self._vision_loop, name="mqtt-cmd-vision",
                         daemon=True).start()
        threading.Thread(target=self._runtime_loop, name="mqtt-cmd-runtime",
                         daemon=True).start()
        print("[mqtt-cmd] adapter up: sub %s/%s, pub %s@%.0fHz/%s -> %s:%d"
              % (TOPIC_MOTION_CMD, TOPIC_VISION_CMD, TOPIC_MOTION_FB,
                 RUNTIME_HZ, TOPIC_VISION_FB, self.host, self.port),
              flush=True)
        return self

    def stop(self):
        self._stop = True

    # ---------------- 运动命令（GUI_DR_ACS_Motion） ----------------
    def _motion_loop(self):
        while not self._stop:
            try:
                sub = MqttSub(host=self.host, port=self.port,
                              client_id="mqtt-cmd-motion")
                sub.subscribe(TOPIC_MOTION_CMD)
                while not self._stop:
                    got = sub.recv(timeout=30.0)
                    if got is None:              # 闲置唤醒，连接保持
                        continue
                    try:
                        self._on_motion_cmd(got[1])
                    except Exception as e:
                        # 任何单条命令异常不得拖垮服务
                        print("[mqtt-cmd] 运动命令处理异常: %s" % e,
                              flush=True)
            except (ConnectionError, OSError, ValueError):
                time.sleep(0.5)                  # broker 未起/断线：退避重连

    def _on_motion_cmd(self, payload):
        outer = json.loads(payload)
        task = outer.get("task") or ""
        if task in ("CMD_Motion_MoveAbs", "CMD_Motion_MoveRel"):
            ctx = json.loads(outer.get("context") or "{}")
            self._do_move(task, ctx)
            return
        # 未实现 task（Home/SetVel/IO/FLS 等）：不崩、记日志、回错误帧
        print("[mqtt-cmd] 未实现 task 忽略: %r" % task, flush=True)
        self._publish(TOPIC_MOTION_FB, {
            "task": task, "result": False,
            "context": json.dumps(
                {"error": "task not implemented in sim adapter: %s" % task},
                ensure_ascii=False)})

    def _do_move(self, task, ctx):
        stage_name = ctx.get("name") or ""
        lst = ctx.get("lst_Axis") or []
        if not lst:
            raise ValueError("lst_Axis 为空")
        for item in lst:
            sim_axis = self.map_axis(stage_name, item.get("name"))
            if task == "CMD_Motion_MoveAbs":
                target = float(item.get("moveAbs") or 0.0)
            else:
                with self.engine.lock:
                    cur = self.engine.axes[sim_axis].pos
                target = cur + float(item.get("moveRel") or 0.0)
            vel = (item.get("vel") or {}).get("targetVal") or 0
            speed = float(vel) if vel > 0 else None
            self.engine.move(sim_axis, target, speed=speed)

    # ---------------- 视觉命令（VSC_Com） ----------------
    def _vision_loop(self):
        """订阅线程只做 recv + 解析 camID + 分发（迭代 33）：识别在
        per-cam 消费线程里跑——同相机命令天然串行（顺带消除 _do_trigger
        光源竞争），跨相机并发，单条慢 identify 不再堵住其它相机。"""
        while not self._stop:
            try:
                sub = MqttSub(host=self.host, port=self.port,
                              client_id="mqtt-cmd-vision")
                sub.subscribe(TOPIC_VISION_CMD)
                while not self._stop:
                    got = sub.recv(timeout=30.0)
                    if got is None:
                        continue
                    try:
                        self._dispatch_vision_cmd(got[1])
                    except Exception as e:
                        print("[mqtt-cmd] 视觉命令分发异常: %s" % e,
                              flush=True)
                        # 迭代 31：异常路径也向 VSS_Com 回错误帧（原只
                        # print 静默，测校软件干等超时）
                        self._reply_vision_error(got[1], e)
            except (ConnectionError, OSError, ValueError):
                time.sleep(0.5)

    def _vision_queue_for(self, cam):
        """取该相机的命令队列；首次调用懒创建队列并启动 daemon 消费
        线程（随 _stop 退出）。"""
        with self._vq_lock:
            q = self._vision_queues.get(cam)
            if q is None:
                q = queue.Queue()
                self._vision_queues[cam] = q
                threading.Thread(
                    target=self._vision_consumer, args=(cam, q),
                    name="mqtt-cmd-vision-%s" % cam, daemon=True).start()
            return q

    def _vision_consumer(self, cam, q):
        """per-cam 消费线程（迭代 33）：同相机命令串行 identify。异常
        语义与原子版一致——异常也回错误帧，单条坏命令不拖垮线程。"""
        while not self._stop:
            try:
                payload = q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._on_vision_cmd(payload)
            except Exception as e:
                print("[mqtt-cmd] 视觉命令处理异常(%s): %s" % (cam, e),
                      flush=True)
                self._reply_vision_error(payload, e)

    def _dispatch_vision_cmd(self, payload):
        """解析 camID 并分发到 per-cam 队列。未知 camID 保持现有立即回
        错误帧路径；队列积压（待处理 ≥ VISION_BACKLOG_MAX）丢弃最旧命令
        并用 _reply_vision_error 回错误帧（信息含"被新命令取代"与被丢
        命令的原始 task），再入队新命令——队列长度有界。"""
        outer = json.loads(payload)
        content = json.loads(outer.get("context") or "{}")
        cam_id = content.get("camID")
        if isinstance(cam_id, str) and cam_id in self.vision.cams:
            cam = cam_id
        else:
            try:
                cam = CAM_ID_MAP[int(cam_id)]
            except (TypeError, ValueError, KeyError):
                # 未知 camID 回错误帧（迭代 31：原 raise 只落到 print，
                # 测校软件无任何回执）
                self._publish_vision_result(
                    cam_id, content, _empty_result(""),
                    "未知 camID: %r" % (cam_id,))
                return
        q = self._vision_queue_for(cam)
        if q.qsize() >= VISION_BACKLOG_MAX:
            try:
                old_payload = q.get_nowait()    # 丢最旧
            except queue.Empty:
                old_payload = None
            if old_payload is not None:
                self._reply_vision_error(
                    old_payload, "命令被新命令取代（同相机队列积压）")
        q.put_nowait(payload)

    def _reply_vision_error(self, payload, err):
        """VSC_Com 未捕获异常的回执帧（迭代 31）：尽量从原命令取 camID/task
        回显，judge=false + 空 lst + error——测校软件按帧判失败而非干等
        超时。回帧自身失败打 stderr 留痕（迭代 42，不静默；仍不递归）。"""
        content = {}
        try:
            outer = json.loads(payload)
            content = json.loads(outer.get("context") or "{}")
            if not isinstance(content, dict):
                content = {}
        except (TypeError, ValueError):
            pass
        try:
            self._publish_vision_result(content.get("camID"), content,
                                        _empty_result(""), str(err))
        except Exception as e:
            # 回执帧发布失败不再静默（迭代 42）：stderr 留痕一行，不递归
            print("[mqtt_cmd_adapter] 视觉错误回执帧发布失败: %s" % e,
                  file=sys.stderr)

    def _on_vision_cmd(self, payload):
        outer = json.loads(payload)
        content = json.loads(outer.get("context") or "{}")
        cam_id = content.get("camID")
        if isinstance(cam_id, str) and cam_id in self.vision.cams:
            cam = cam_id
        else:
            try:
                cam = CAM_ID_MAP[int(cam_id)]
            except (TypeError, ValueError, KeyError):
                # 未知 camID 回错误帧（迭代 31：原 raise 只落到 print，
                # 测校软件无任何回执）
                self._publish_vision_result(
                    cam_id, content, _empty_result(""),
                    "未知 camID: %r" % (cam_id,))
                return
        cell = json.loads(content.get("content") or "{}")
        shapes = (cell.get("1") or {}).get("lstShapeName") or []
        template = shapes[0] if shapes else None
        try:
            entry = self.vision.identify(cam, template)
            res = entry["result"]
            err = None
        except Exception as e:
            # 识别失败也回一帧（空 lst + judge=false），参考 wait_image_process
            # 按 lstRow 非空判定，空列表会走它自己的超时路径
            res = _empty_result(template)
            err = str(e)
        self._publish_vision_result(cam, content, res, err)

    def _publish_vision_result(self, cam, req_content, res, err):
        """VSS_Com 结果帧：content["1"] 携带 lstRow/lstCol/lstAngle/lstScore/
        judge/strShapeName（参考 MqttReceiveWorker 的取值路径）。"""
        cell1 = {
            "m_nCellId": 1, "m_strName": "",
            "lstShapeName": [res.get("strShapeName") or ""],
            "strShapeName": res.get("strShapeName") or "",
            "judge": bool(res.get("judge")),
            "lstRow": list(res.get("lstRow") or []),
            "lstCol": list(res.get("lstCol") or []),
            "lstAngle": list(res.get("lstAngle") or []),
            "lstScore": list(res.get("lstScore") or []),
            "lstScale": list(res.get("lstScale") or []),
        }
        if err:
            cell1["error"] = err
        content = {
            "camID": req_content.get("camID"),
            "cmdType": req_content.get("cmdType", 2),
            "content": json.dumps({"0": {"m_nCellId": 0, "m_strName": ""},
                                   "1": cell1}, ensure_ascii=False),
            # 图像回传（isImageSend）本迭代不支持：image 恒 ""（参考解析
            # 对 "" 跳过图像分支，只看 content）
            "image": "",
            "imgType": -1,
            "isImageSend": 0,
            "judge": bool(res.get("judge")),
            "lstCellType": [0, 1],
            "task": req_content.get("task", ""),
        }
        self._publish(TOPIC_VISION_FB, {
            "context": json.dumps(content, ensure_ascii=False),
            "module": "", "result": 0 if not err else -1,
            "task": req_content.get("task", "")})

    # ---------------- 周期反馈（CTL_DR_ACS_Motion） ----------------
    def _runtime_loop(self):
        period = 1.0 / RUNTIME_HZ
        while not self._stop:
            t0 = time.perf_counter()
            try:
                self._publish(TOPIC_MOTION_FB, self._runtime_frame())
            except (ConnectionError, OSError, ValueError):
                pass                           # broker 未起：下拍再试
            dt = time.perf_counter() - t0
            time.sleep(max(0.0, period - dt))

    def _runtime_frame(self):
        """CMD_Motion_RunTimeInfo 帧：context.dic_Stage 按 STAGES 分组，
        条目字段与参考 lst_Axis 同构（参考解析路径：stage.inPos /
        stage.moving / lst_Axis[i].coord.value）。"""
        snap = {}
        with self.engine.lock:
            for _sid, _sname, axes in STAGES:
                for n in axes:
                    ax = self.engine.axes.get(n)
                    if ax is not None:
                        snap[n] = (ax.pos, ax.target or 0.0, ax.vel,
                                   ax.unit, ax.moving, ax.in_position,
                                   ax.enabled, ax.fault)
        dic_stage = {}
        for sid, sname, axes in STAGES:
            lst = []
            moving, in_pos, fault = False, True, False
            for n in axes:
                if n not in snap:
                    continue
                pos, target, vel, unit, mv, ip, en, flt = snap[n]
                lst.append(_axis_entry(n, pos, target, vel, unit))
                moving = moving or mv
                in_pos = in_pos and ip
                fault = fault or flt
            dic_stage[str(sid)] = {
                "enable": True, "inPos": bool(in_pos and not moving),
                "moving": bool(moving), "fault": bool(fault),
                "name": sname, "stage": sid, "lst_Axis": lst,
            }
        return {"task": "CMD_Motion_RunTimeInfo", "result": True,
                "context": json.dumps({"dic_Stage": dic_stage},
                                      ensure_ascii=False)}

    def _publish(self, topic, obj):
        mqtt_publish(topic, json.dumps(obj, ensure_ascii=False),
                     self.host, self.port)
