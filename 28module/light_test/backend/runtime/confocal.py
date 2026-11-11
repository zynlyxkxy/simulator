# -*- coding: utf-8 -*-
"""confocal.py — 光谱共焦读数模型（调焦调平的高度测量传感层，2026-08-27 起，
见 docs/调焦调平与高度链_V1.md）。纯标准库，单位 mm/rad。

安装事实（前端模型 CAD 真值，不可改）：上/下共焦与上/下精对准相机同挂
对准相机承载台（align_stage 模块，随 AS_X/Y/Z 平移 + AS_Rx/Ry 微转）：
- up   offset [0,-47.5,+15.0]（零位世界 z=925.0，朝上量键合头芯片下表面）
- down offset [0,-47.5,-15.0]（零位世界 z=895.0，朝下量晶圆面/裸吸盘面）
共焦相对相机 ΔY=-25.0mm（机器常数 focus_level.confocal_cam_dy_mm，测校可改）。

读数模型（近大远小，量程 [0, confocal_range_um]，量程外无读数 None）：
- **中点共面约定**：共焦读数 = confocal_mid_*_um（量程中点）↔ 被测面正好在
  对应相机的最佳焦面上（装配约定）→ 焦面世界 z 直接取相机焦面公式
  （相机 z + look_dir×wd_mm，vision_model.camera_pose 同一来源）。
- 下探头（朝下看，面越高读数越大）：
    reading_um = mid_down + (z_surface − z_focusplane) × 1000 + noise
- 上探头（朝上看，面越低读数越大）：
    reading_um = mid_up + (z_focusplane − z_surface) × 1000 + noise
- noise ~ N(0, σ)；σ=recipe sim_sensors.confocal_sigma_um（按探头 up/down
  逐实例，2026-09-08 迭代 22 拆分；测校页"传感器仿真"卡，2026-08-27 由
  机械常数迁入）；加噪后落在 [0, range] 之外 → 无读数。

真值面高（世界 z，mm）：
- 晶圆面 = 884.9（陶瓷面顶）+ t_w（recipe.wafer.thickness_mm）
    + z_err + rx_w·(x−cx) − ry_w·(y−cy)
  （cx,cy=晶圆中心世界坐标，wafer 上片真值；z_err/rx_w/ry_w 为上片注入，
  默认 0，测校页 wafer_load.z_um/rx_urad/ry_urad）
- 芯片下表面（键合头所吸 die）= 1053.83（吸盘面零位）+ BS_Arm_Z − t_d
    + rx_eff·(x−xd) − ry_eff·(y−yd)
  （xd,yd=die 中心世界坐标；rx_d/ry_d=芯片在头倾斜真值，默认 0，
  测校页 wafer_load.die_head_rx/ry_urad 注入，交接完成时写入 die 真值；
  BS_CAL_Rx/Ry 调平轴物理改变芯片面世界坡度，按 rotvec 约定合成进有效
  坡度：dz/dx 方向 rx_eff = rx_d − BS_CAL_Ry、dz/dy 方向 ry_eff = ry_d
  − BS_CAL_Rx——面高公式约定 rx=∂z/∂x、ry=−∂z/∂y，而旋转轴 Rx 是
  y 向坡度来源，两者交叉对应，推导见 docs/调焦调平与高度链_V1.md）
- 裸吸盘面 = 884.9（无晶圆时下探头目标，测校用）。
- 键合头吸盘（工具）面 = 1053.83 + BS_Arm_Z（head 无 die 时上探头目标，
  2026-09-08 修正：光打吸盘面同样反射，与下探头裸吸盘面同口径的测校
  基准；原"无 die → 无读数"是模型缺陷，用户指出）。

MQTT 服务化（2026-08-27，契约见 docs/前后端数据协议_V1.md §12）：
- ConfocalModel.start_mqtt()：订阅 `confocal/trigger/+`，收到触发即执行
  read() 并把读数发布到 `confocal/result/{probe}`（QoS0，req_id 关联）——
  与视觉 vision/result/{cam} 同一 broker(1883)、同一风格；
- ConfocalClient：主控侧（flow_engine fl_measure）真订阅消费端——发布
  触发→等结果主题上对应 req_id（后台收线线程只解析 JSON 填字典 +
  Condition，不碰引擎状态），超时判失败；
- 同步 read() 保留供调试 API GET /api/confocal/read 使用。
"""
import json
import random
import socket
import threading
import time

import vision_model as vm
from mqtt_broker import MqttSub, mqtt_publish

# 探头安装事实（align_stage 局部 offset，CAD 真值）
PROBE_MOUNT = {
    "up": [0.0, -47.5, 15.0],
    "down": [0.0, -47.5, -15.0],
}
CHUCK_TOP_Z_MM = 884.9        # 吸盘陶瓷面顶世界 z（vision_cameras.json down._note）
HEAD_TOOL_Z0_MM = 1053.83     # 键合头吸盘面零位（BS_Arm_Z=0）世界 z

# MQTT 契约主题（docs/前后端数据协议_V1.md §12）
TRIGGER_TOPIC = "confocal/trigger/%s"     # % probe（up/down）
RESULT_TOPIC = "confocal/result/%s"       # % probe
TRIGGER_FILTER = "confocal/trigger/+"
RESULT_FILTER = "confocal/result/+"


class _ProbeSpec(object):
    """探头伪相机规格：复用 vision_model.camera_pose 的模块 6DOF 位姿合成
    （align_stage 名义+误差、AS_Rx/Ry 轴姿态并入、offset 随姿态旋转）。"""

    def __init__(self, offset):
        self.mount_module = "align_stage"
        self.mount_offset = list(offset)
        self.rot = 0.0
        self.view_rot = 0.0   # camera_pose duck-typing 契约：探头无安装微旋概念（真值朝向 0；迭代 38 视觉真值/认知分离后 camera_pose 改读 view_rot，此补）


class ConfocalModel(object):
    """上/下光谱共焦读数模型：位姿合成 + 真值面高 + 量程/噪声读数。"""

    def __init__(self, engine):
        self.engine = engine
        self._meas_until = {"up": 0.0, "down": 0.0}
        self._lock = threading.Lock()

    # ---------------- 位姿 ----------------
    def probe_pose(self, probe, positions=None, poses=None):
        """探头中心世界坐标 [x,y,z]（复用相机位姿合成路径）。"""
        pos, _yaw = vm.camera_pose(self.engine, _ProbeSpec(PROBE_MOUNT[probe]),
                                   poses, positions)
        return pos

    def focus_plane_z(self, probe, positions=None, poses=None):
        """共焦焦面世界 z：中点共面约定 = 对应相机（up/down）最佳焦面 z。"""
        cam = self.engine.vision.cams[probe].model
        cpos, _yaw = vm.camera_pose(self.engine, cam, poses, positions)
        return cpos[2] + cam.look_dir * cam.wd_mm

    # ---------------- 真值面高 ----------------
    def _thermal_dz(self, module):
        """模块热变形 Z 分量（mm；thermal.module_offset 的 t[2]）。
        热 OFF/未挂载（bare 离线引擎）恒 0——面高/读数默认逐位不变；
        热 ON 时吸盘 Z 胀（chuck_z_growth）/头 Z 伸长（head_z_growth）
        等 6DOF Z 变形进入共焦面高（迭代 56，与视觉侧模块位姿口径
        对齐的增量最小实现：静态误差链差异仍由既有测校链覆盖）。"""
        th = getattr(self.engine, "thermal", None)
        if th is None:
            return 0.0
        try:
            return float(th.module_offset(module)[0][2])
        except Exception:
            return 0.0

    def wafer_surface_z(self, x, y, positions, poses, recipe, wp):
        """世界点 (x,y) 处晶圆面真值 z（含上片注入 z/rx/ry）。"""
        center, rot = vm._module_pose(poses, "wafer_stage")
        rz = positions.get("WS_Rz", 0.0)
        rot = [rot[0], rot[1], rot[2] + rz]
        # WS_Rz 真值枢轴（迭代 6，与 vision_model wafer_marks 同步）：
        # 默认 [0,0]=绕吸盘心；注入时绕 吸盘心+真值偏移
        pivot = vm.wafer_rz_pivot(recipe)
        with wp._lock:
            w = dict(wp.wafer)
        off = vm._rotvec_apply_pivot([w["offset"][0], w["offset"][1], 0.0],
                                     rot, pivot)
        cx, cy = center[0] + off[0], center[1] + off[1]
        t_w = float(recipe["wafer"]["thickness_mm"])
        return (CHUCK_TOP_Z_MM + t_w + float(w.get("z", 0.0))
                + float(w.get("rx", 0.0)) * (x - cx)
                - float(w.get("ry", 0.0)) * (y - cy) \
                + self._thermal_dz("wafer_stage"))  # 吸盘热胀 Z（迭代 56）

    def die_center_world(self, positions, poses, wp):
        """键合头 die 中心世界 [x, y]（head 无 die → None）。
        与 die_surface_z 同一套位姿来源（含 BS_CAL_Rx/Ry 姿态并入）。"""
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "head"]
        if not dies:
            return None
        d = dies[0]
        center, rot = vm._module_pose(poses, "bond_head")
        rot = [rot[0] + positions.get("BS_CAL_Rx", 0.0),
               rot[1] + positions.get("BS_CAL_Ry", 0.0), rot[2]]
        site = d["site"] or [0.0, 0.0]
        off = vm._rotvec_apply([site[0], site[1], 0.0], rot)
        return [center[0] + off[0], center[1] + off[1]]

    def die_surface_z(self, x, y, positions, poses, recipe, wp):
        """世界点 (x,y) 处键合头芯片下表面真值 z；head 无 die → None。"""
        with wp._lock:
            dies = [dict(d) for d in wp.dies if d["carrier"] == "head"]
        if not dies:
            return None
        d = dies[0]
        center, rot = vm._module_pose(poses, "bond_head")
        # 键合头名义姿态（BS_CAL_Rx/Ry 轴值，小角）并入（同 head_die_marks）
        rot = [rot[0] + positions.get("BS_CAL_Rx", 0.0),
               rot[1] + positions.get("BS_CAL_Ry", 0.0), rot[2]]
        site = d["site"] or [0.0, 0.0]
        off = vm._rotvec_apply([site[0], site[1], 0.0], rot)
        xd, yd = center[0] + off[0], center[1] + off[1]
        t_d = float(recipe["film"]["dies"]["thickness_mm"])
        z = HEAD_TOOL_Z0_MM + positions.get("BS_Arm_Z", 0.0) - t_d \
                + self._thermal_dz("bond_head")      # 头热伸长 Z（迭代 56）
        # BS_CAL 调平轴物理旋转芯片面：rotvec 约定下 Rx 产生 y 向坡度
        #（dz/dy=+Rx）、Ry 产生 x 向坡度（dz/dx=−Ry）；与本面高公式的
        # rx=∂z/∂x、ry=−∂z/∂y 约定交叉对应，合成有效坡度：
        rx_eff = float(d.get("rx", 0.0)) - positions.get("BS_CAL_Ry", 0.0)
        ry_eff = float(d.get("ry", 0.0)) - positions.get("BS_CAL_Rx", 0.0)
        return z + rx_eff * (x - xd) - ry_eff * (y - yd)

    def collet_surface_z(self, x, y, positions, poses):
        """世界点 (x,y) 处键合头吸盘（工具）面真值 z。物理事实：无 die
        时上探头光斑打在吸盘面上同样有反射（用户指出 2026-09-08）——
        与 down 探头无晶圆读裸吸盘面（CHUCK_TOP_Z）同口径的测校基准。
        z = HEAD_TOOL_Z0 + BS_Arm_Z（不减 die 厚），随 BS_CAL 调平轴
        坡度（rotvec 约定：Rx→dz/dy、Ry→dz/dx，与 die 面同交叉对应；
        die 真值坡度 rx_d/ry_d 项不适用——无 die）。"""
        center, _rot = vm._module_pose(poses, "bond_head")
        xd, yd = center[0], center[1]
        z = HEAD_TOOL_Z0_MM + positions.get("BS_Arm_Z", 0.0) \
                + self._thermal_dz("bond_head")      # 头热伸长 Z（迭代 56）
        rx_eff = -positions.get("BS_CAL_Ry", 0.0)
        ry_eff = -positions.get("BS_CAL_Rx", 0.0)
        return z + rx_eff * (x - xd) - ry_eff * (y - yd)

    # ---------------- 读数 ----------------
    def read(self, probe):
        """读一次共焦 → {ok, probe, reading_um, valid, z_surface, z_focus}。
        reading_um=None/valid=False = 无读数（量程外或无有效目标）。
        每次读数置位 250ms 可视化窗口，供 /api/viz 光束显示捕获。"""
        if probe not in PROBE_MOUNT:
            raise ValueError("probe 只能是 up/down: %r" % (probe,))
        with self._lock:
            self._meas_until[probe] = time.time() + 0.25
        if self.engine.constants is None or self.engine.vision is None:
            raise RuntimeError("confocal 需要 engine.constants/vision")
        fl = self.engine.constants.get()["focus_level"]
        rng = float(fl["confocal_range_um"])
        recipe = self.engine.recipe.get() if self.engine.recipe else None
        # 读数噪声 σ 属传感器仿真参数（recipe sim_sensors 组，测校页改，
        # 热生效——每次读实时取；缺省 0.065µm（2026-09-08 用户设定）；
        # 迭代 22 按探头 up/down 逐实例拆分，旧标量兼容回退）
        sigma = 0.065
        if recipe is not None:
            cs = (recipe.get("sim_sensors") or {}).get("confocal_sigma_um",
                                                        0.065)
            sigma = float(cs.get(probe, 0.065)) \
                if isinstance(cs, dict) else float(cs)
        positions, poses = vm._engine_poses(self.engine)
        ppos = self.probe_pose(probe, positions, poses)
        zf = self.focus_plane_z(probe, positions, poses)
        wp = self.engine.workpieces
        if probe == "down":
            mid = float(fl["confocal_mid_down_um"])
            if wp is not None and recipe is not None and wp.wafer["present"]:
                zs = self.wafer_surface_z(ppos[0], ppos[1], positions, poses,
                                          recipe, wp)
            else:
                zs = CHUCK_TOP_Z_MM + self._thermal_dz("wafer_stage") \
                                                # 裸吸盘面（测校基准；
                                                #   迭代 56 吸盘热胀 Z 入面高）
            raw = mid + (zs - zf) * 1000.0     # 近大远小（朝下看）
        else:
            mid = float(fl["confocal_mid_up_um"])
            if wp is not None and recipe is not None:
                zs = self.die_surface_z(ppos[0], ppos[1], positions, poses,
                                        recipe, wp)
                if zs is None:
                    # head 无 die：光斑打在吸盘（工具）面上——与 down
                    # 无晶圆读裸吸盘面同口径的测校基准（2026-09-08 修正：
                    # 原直接 None 是模型缺陷，物理上吸盘面同样反射）
                    zs = self.collet_surface_z(ppos[0], ppos[1],
                                               positions, poses)
            else:
                zs = None
            raw = None if zs is None else mid + (zf - zs) * 1000.0  # 朝上看
        reading = None
        valid = False
        if raw is not None:
            r = raw + (random.gauss(0.0, sigma) if sigma > 0 else 0.0)
            if 0.0 <= r <= rng:
                reading = round(r, 4)
                valid = True
        return {"ok": True, "probe": probe,
                "reading_um": reading, "valid": valid,
                "z_surface": None if zs is None else round(zs, 6),
                "z_focus": round(zf, 6)}

    def measuring_public(self):
        """当前哪些探头正在测量（供 /api/viz 显示光束）。"""
        now = time.time()
        with self._lock:
            return {p: now < self._meas_until[p] for p in PROBE_MOUNT}

    # ---------------- MQTT 服务化（触发→测量→发布） ----------------
    def start_mqtt(self, host="127.0.0.1", port=1883):
        """共焦服务线程：订阅 confocal/trigger/+，收到触发即 read() 并把
        结果发布到 confocal/result/{probe}（与视觉服务同一 broker、同一
        mqtt_publish 一次性发布模式；断线自动重连）。在 main.py 里于
        broker 起来之后调用。"""
        self._mqtt_hp = (host, port)
        threading.Thread(target=self._mqtt_loop, name="confocal-mqtt",
                         daemon=True).start()
        return self

    def _mqtt_loop(self):
        host, port = self._mqtt_hp
        while True:
            try:
                sub = MqttSub(host=host, port=port, client_id="confocal-svc")
                sub.subscribe(TRIGGER_FILTER)
                while True:
                    got = sub.recv(timeout=30.0)
                    if got is None:              # 闲置不重建：保持订阅，
                        continue                 # 断线由外层 except 重连
                    topic, payload = got
                    try:
                        req = json.loads(payload)
                    except ValueError:
                        req = {}
                    probe = req.get("probe") or topic.rsplit("/", 1)[-1]
                    if probe not in PROBE_MOUNT:
                        continue
                    r = self.read(probe)
                    r["req_id"] = req.get("req_id")
                    mqtt_publish(RESULT_TOPIC % probe,
                                 json.dumps(r, ensure_ascii=False),
                                 host, port)
            except (ConnectionError, socket.timeout, OSError, ValueError):
                time.sleep(0.5)                  # broker 未起/断线：退避重连


class ConfocalClient(object):
    """主控侧共焦 MQTT 客户端（flow_engine fl_measure 真订阅消费端）。

    线程模型：后台收线线程订阅 confocal/result/+，收到消息**只**解析
    JSON 按 req_id 填字典并 notify（绝不碰引擎状态）；request() 由流程
    线程调用——发布触发 → Condition 等对应 req_id → 返回读数 dict
    （字段与 ConfocalModel.read() 相同 + req_id），超时抛 RuntimeError。
    """

    def __init__(self, host="127.0.0.1", port=1883):
        self._hp = (host, port)
        self._cond = threading.Condition()
        self._by_req = {}                # req_id -> 结果 dict（读后即删）
        self._seq = 0

    def start(self):
        threading.Thread(target=self._recv_loop, name="confocal-client",
                         daemon=True).start()
        return self

    def _recv_loop(self):
        host, port = self._hp
        while True:
            try:
                sub = MqttSub(host=host, port=port,
                              client_id="confocal-cli-%d" % id(self))
                sub.subscribe(RESULT_FILTER)
                while True:
                    got = sub.recv(timeout=30.0)
                    if got is None:              # 闲置不重建：保持订阅，
                        continue                 # 断线由外层 except 重连
                    _topic, payload = got
                    try:
                        msg = json.loads(payload)
                    except ValueError:
                        continue
                    rid = msg.get("req_id")
                    if rid is None:
                        continue
                    with self._cond:
                        self._by_req[rid] = msg
                        # 防泄漏：滞留消息上限（正常路径读后即删）
                        if len(self._by_req) > 64:
                            for k in sorted(self._by_req)[:-64]:
                                self._by_req.pop(k, None)
                        self._cond.notify_all()
            except (ConnectionError, socket.timeout, OSError):
                time.sleep(0.5)

    def request(self, probe, timeout=5.0):
        """发布一次测量触发并等结果 -> 读数 dict；超时抛 RuntimeError。"""
        if probe not in PROBE_MOUNT:
            raise ValueError("probe 只能是 up/down: %r" % (probe,))
        with self._cond:
            self._seq += 1
            req_id = "fl-%06d" % self._seq
        host, port = self._hp
        mqtt_publish(TRIGGER_TOPIC % probe,
                     json.dumps({"req_id": req_id, "probe": probe}),
                     host, port)
        deadline = time.time() + timeout
        with self._cond:
            while req_id not in self._by_req:
                left = deadline - time.time()
                if left <= 0:
                    raise RuntimeError(
                        "共焦 MQTT 读数超时（%.1fs 未收到 %s 的结果，"
                        "confocal 服务/broker 异常）" % (timeout, req_id))
                self._cond.wait(min(left, 0.5))
            return self._by_req.pop(req_id)
