#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bare_harness.py — 裸引擎测试 harness（迭代 19：回归方法论升级）。

用途：`python tests/test_client.py --full --bare --segs <段名,...> --out <json>`
在多进程分片回归（tools/run_full_parallel.py）里替代完整 runtime 子进程：

- 照 runtime/main.py:110-168 删减组装真实组件（Points/Engine/ErrorSystem/
  ErrorKinematics/Recipe/Tuning/Workpieces/FlowEngine/ChainRunner/
  FullWaferRunner/BondResults/MachineConstants/VisionService(不 start)/
  ConfocalModel/CollisionChecker），不拉起任何端口服务（WSGateway 不 start、
  无 plc_http/acs_tcp/mcs_http/vision_client/confocal_client/calibration）；
- HTTP shim：与 test_client 的 http_json/http_get_raw/http_post/http_put/
  http_delete 同签名——/api/* 全部经假 conn 复用 WSGateway._http_request
  真实路由（GET/POST/PUT/DELETE 逐分支 + 异常→状态码映射，与 9010 端口
  行为逐字节同构）；PLC /io/observe|/io/snapshot 直读 points（语义同
  plc_http.do_GET）；
- tick 由驱动线程全速推进（Engine.tick 每拍 1ms 仿真，不受墙钟节拍限）；
  speed_scale 语义保留（/api/speed 照常走引擎，仅影响 WS 帧构建节流）；
- 状态文件全部指向 per-shard tempdir（recipe/tuning/machine_constants/
  error_kinematics/vision_cameras/vision_templates(+PNG 目录)/flows/
  bond_results/config_overrides 从 runtime/ 复制或全新播种），分片互踩
  与污染仓库数据文件双防；
- 可选 per-shard 真 MQTT broker（mqtt=True）：MqttBroker +
  confocal.start_mqtt + ConfocalClient（端口 mqtt_port），供
  focus_level 段 fl.mqtt_* 契约用例真跑 confocal/trigger→result 链路；
  不启用时 flow_engine._fl_read/_vision_identify 自动回退进程内
  （eng.confocal.read / eng.vision.identify，flow_engine.py:1589/1612）。

与 HTTP 全栈的差异（已知、有意）：无 WS 帧流/无端口层/无 HTTP 传输开销；
墙钟语义用例（stream.freq/speed.walltime/perf_guard/REAL 视觉/homing
duration 等）不分配给裸引擎分片（留在全栈片跑）。
"""
import json
import os
import shutil
import sys
import tempfile
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
RT = os.path.join(_ROOT, "runtime")
sys.path.insert(0, RT)

from points import Points                      # noqa: E402
from engine import Engine, InterlockError      # noqa: E402
from error_system import ErrorSystem           # noqa: E402
from error_kinematics import ErrorKinematics   # noqa: E402
from recipe import Recipe                      # noqa: E402
from tuning import Tuning                      # noqa: E402
from workpieces import Workpieces              # noqa: E402
from flow_engine import FlowEngine             # noqa: E402
from chain_runner import ChainRunner           # noqa: E402
from full_wafer_runner import FullWaferRunner  # noqa: E402
from bond_results import BondResults           # noqa: E402
from machine_constants import MachineConstants  # noqa: E402
from vision_service import (VisionService, TemplateStore,  # noqa: E402
                            CaptureStore)
from confocal import ConfocalModel, ConfocalClient  # noqa: E402
from mqtt_broker import MqttBroker             # noqa: E402
from ws_gateway import WSGateway               # noqa: E402

# 从 runtime/ 复制进 tempdir 的可变数据文件（PUT/持久化会写，必须隔离）
_COPY_FILES = ("recipe.json", "tuning.json", "machine_constants.json",
               "error_kinematics.json", "vision_cameras.json",
               "vision_templates.json", "flows.json",
               "materials.json", "material_map.json")


class _FakeConn(object):
    """假 socket：sendall 捕获响应字节；_read_body 的 rest 已含全 body，
    recv 永不真被调（兜底返回 b"" 防死循环）。"""

    def __init__(self):
        self.buf = bytearray()

    def sendall(self, b):
        self.buf += b

    def recv(self, n):
        return b""

    def settimeout(self, t):
        pass

    def close(self):
        pass


def _parse_response(buf):
    """解析 _send_json/_send_bytes 产物 -> (status, body_bytes)。"""
    head, _sep, body = bytes(buf).partition(b"\r\n\r\n")
    code = int(head.split(b" ", 2)[1])
    return code, body


class BareHarness(object):
    """裸引擎组装 + HTTP shim + 全速 tick 驱动。"""

    def __init__(self, shard_id=0, mqtt=False, mqtt_port=11883):
        self.shard_id = shard_id
        self.tmp = tempfile.mkdtemp(prefix="bare_h%d_" % shard_id)
        for fn in _COPY_FILES:
            shutil.copy(os.path.join(RT, fn), os.path.join(self.tmp, fn))
        shutil.copytree(os.path.join(RT, "vision_templates"),
                        os.path.join(self.tmp, "vision_templates"))
        os.makedirs(os.path.join(self.tmp, "vision_captures"),
                    exist_ok=True)

        points = Points(os.path.join(RT, "io_points.json"))
        eng = Engine(points)
        # 真实 Points 映像（EMO=False/门关=True），联锁语义与全栈一致，
        # 保持默认 enabled（iter9 的 _FakePts 屏蔽理由在此不成立）
        eng.overrides_path = os.path.join(self.tmp, "config_overrides.json")
        eng.errors = ErrorSystem()
        eng.recipe = Recipe(os.path.join(self.tmp, "recipe.json"))
        eng.tuning = Tuning(os.path.join(self.tmp, "tuning.json"))
        eng.workpieces = Workpieces(os.path.join(RT, "transfers.json"),
                                    recipe=eng.recipe)
        eng.flow = FlowEngine(eng, points, os.path.join(self.tmp,
                                                        "flows.json"))
        eng.chain = ChainRunner(eng)
        eng.full_wafer = FullWaferRunner(eng)
        self.bond_results_path = os.path.join(self.tmp, "bond_results.json")
        eng.bond_results = BondResults(self.bond_results_path)
        eng.kinematics = ErrorKinematics(
            eng.errors, os.path.join(self.tmp, "error_kinematics.json"))
        eng.constants = MachineConstants(
            os.path.join(self.tmp, "machine_constants.json"))
        # 视觉：不 start（不起 TCP 9200/相机循环/预热）；模板/采图库指向
        # tempdir 副本（template_create/capture_save 会持久化，防写仓库）
        eng.vision = VisionService(
            eng, points,
            cfg_path=os.path.join(self.tmp, "vision_cameras.json"),
            constants=eng.constants, mqtt_port=mqtt_port)
        eng.vision.templates = TemplateStore(
            os.path.join(self.tmp, "vision_templates.json"),
            os.path.join(self.tmp, "vision_templates"))
        eng.vision.captures = CaptureStore(
            os.path.join(self.tmp, "vision_captures.json"),
            os.path.join(self.tmp, "vision_captures"))
        eng.confocal = ConfocalModel(eng)
        # 迭代 50：测校服务挂接（/api/calibration/* 状态查询类用例需要；
        # CalibrationService 无状态文件，纯内存作业。注意：nonortho 栅格
        # 测校走视觉 TCP 9200 帧通道——VisionService 不 start 无此通道，
        # 故 19.29.nonortho 段本质上不能 bare，留全栈片）
        from calibration import CalibrationService
        eng.calibration = CalibrationService(eng, points, eng.vision)
        # 迭代 50：组件材质（19.25.iter21 段 i21.* 用例需要；目录只读解析
        # 前端模型 JSON 与全栈共享，材料库/映射落盘指向 tempdir 副本）
        from materials import ComponentCatalog, Materials
        eng.components = ComponentCatalog()
        eng.materials = Materials(
            eng.components,
            mats_path=os.path.join(self.tmp, "materials.json"),
            map_path=os.path.join(self.tmp, "material_map.json"))
        # 可选真 MQTT 链路（focus_level 段 fl.mqtt_* 契约用例需要）：
        # broker + 共焦服务端订阅 + 主控客户端（端口 per-shard 隔离）
        self.broker = None
        if mqtt:
            self.broker = MqttBroker(port=mqtt_port).start()
            eng.confocal.start_mqtt(port=mqtt_port)
            eng.confocal_client = ConfocalClient(port=mqtt_port).start()
        sb = os.path.join(RT, "sandboxes.json")
        if os.path.exists(sb):
            from collision import CollisionChecker
            eng.collision = CollisionChecker(sb)
        eng.apply_wafer_keep_warm()
        eng.load_overrides()     # tempdir 下缺失 → 0 条，与干净启动一致
        self.engine = eng
        self.points = points
        # WSGateway 不 start：仅借 _http_request 的真实路由与异常映射
        self.gw = WSGateway(eng, points)

        # ---- 全速 tick 驱动线程（每 ~256 拍让一次 GIL，防流线程饿死）----
        self._tick_stop = False
        self._tick_thread = threading.Thread(target=self._tick_loop,
                                             name="bare-tick", daemon=True)
        self._tick_thread.start()

    def _tick_loop(self):
        eng = self.engine
        n = 0
        while not self._tick_stop:
            eng.tick()
            n += 1
            if n >= 256:
                n = 0
                time.sleep(0.0005)

    def close(self):
        self._tick_stop = True
        self._tick_thread.join(timeout=3.0)
        if self.broker is not None:
            try:
                self.broker.stop()
            except Exception:
                pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ---------------- HTTP shim（与 test_client http_* 同签名） ----------------
    def _api(self, method, url, data):
        """/api/* → 假 conn 复用 WSGateway._http_request 真实路由。"""
        from urllib.parse import urlsplit
        u = urlsplit(url)
        path = u.path + (("?" + u.query) if u.query else "")
        body = json.dumps(data).encode("utf-8") if data is not None else b""
        conn = _FakeConn()
        self.gw._http_request(conn, method, path,
                              {"content-length": str(len(body))}, body)
        return _parse_response(conn.buf)

    def _plc(self, method, url, data=None):
        """PLC /io/observe|/io/snapshot 直读 points（语义同 plc_http）；
        迭代 50：POST /io/command 同路由 plc_command（语义同 plc_http.do_POST，
        供 3.plc 气路/温控段裸跑；沉默语义不模拟——11.fault 留全栈片）。"""
        from urllib.parse import urlsplit, parse_qs
        u = urlsplit(url)
        pts = self.points
        if u.path == "/io/command" and method == "POST":
            from plc_http import plc_command
            req = data or {}
            try:
                return 200, plc_command(self.engine, pts, req.get("path", ""),
                                        req.get("cmd", ""),
                                        req.get("args") or {})
            except InterlockError as e:
                return 409, {"ok": False, "error": str(e)}
            except Exception as e:
                return 400, {"ok": False, "error": str(e)}
        if u.path == "/io/observe":
            path = parse_qs(u.query).get("path", [""])[0]
            if not path:
                return 400, {"ok": False, "error": "missing path"}
            sub = pts.subtree(path)
            if not sub:
                return 404, {"ok": False,
                             "error": "no points under %r" % path}
            if len(sub) == 1 and path in sub:
                return 200, {"ok": True, "path": path, "value": sub[path]}
            return 200, {"ok": True, "path": path, "values": sub}
        if u.path == "/io/snapshot":
            return 200, {"ok": True, "count": len(pts),
                         "points": pts.snapshot()}
        return 404, {"ok": False, "error": "unknown GET " + u.path}

    def _dispatch(self, method, url, data):
        from urllib.parse import urlsplit
        if urlsplit(url).path.startswith("/io/"):
            code, obj = self._plc(method, url, data)
            return code, json.dumps(obj, ensure_ascii=False).encode("utf-8")
        return self._api(method, url, data)

    # 以下五个与 tests/test_client.py 的 http_* 同签名，由 main() 在
    # --bare 模式下替换模块全局（测试体零改动）
    def http_json(self, url, data=None):
        _code, body = self._dispatch(
            "GET" if data is None else "POST", url, data)
        return json.loads(body.decode("utf-8"))

    def http_get_raw(self, url):
        code, body = self._dispatch("GET", url, None)
        return code, json.loads(body.decode("utf-8"))

    def http_post(self, url, data):
        code, body = self._dispatch("POST", url, data)
        return code, json.loads(body.decode("utf-8"))

    def http_put(self, url, data):
        code, body = self._dispatch("PUT", url, data)
        return code, json.loads(body.decode("utf-8"))

    def http_delete(self, url, data):
        code, body = self._dispatch("DELETE", url, data)
        return code, json.loads(body.decode("utf-8"))
