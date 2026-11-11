#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main.py — 一键启动 D2W 键合设备仿真后端运行时（同进程五服务）。

  python runtime/main.py

  物理引擎 1kHz tick（所有轴共用）
  ACS 模拟器   TCP  9101（换行分隔 JSON）
  PLC 模拟器   HTTP 9102（/io/command /io/observe /io/snapshot）
  MCS 适配壳   HTTP 9190（Cordis MCS 真机报文五服务 → 语义层，见 mcs_http.py）
  WS 网关      9010（/ws 二进制帧 500Hz；/api/axes /api/io /api/health /api/vision/*）
  MQTT broker  1883（迷你 MQTT 3.1.1，视觉结果出口 vision/result/{cam}）
  视觉服务     TCP 9200（相机帧接收口；渲染/匹配/MQTT 全独立线程）

  端口可用环境变量覆盖（独立测试套件用，见 tests/test_metrology.py）：
  D2W_PORT_WS / D2W_PORT_ACS / D2W_PORT_PLC / D2W_PORT_MCS /
  D2W_PORT_MQTT / D2W_PORT_VTCP
"""
import os
import sys
import threading
import time

import faulthandler
# 致命错误（Windows 访问冲突/栈溢出等 C 层崩溃）落 Python 栈到 stderr——
# 迭代 11 曾出现倍速 ×3 下进程无声退出无 traceback，开此兜底定位。
faulthandler.enable()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gen_io_points
import gen_error_maps
from points import Points
from engine import Engine
from error_system import ErrorSystem
from workpieces import Workpieces
from recipe import Recipe
from flow_engine import FlowEngine
from error_kinematics import ErrorKinematics
from acs_tcp import start_acs_server
from plc_http import start_plc_server
from mcs_http import start_mcs_server
from ws_gateway import WSGateway

IO_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "io_points.json")
ERROR_MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "error_maps", "manifest.json")
TRANSFERS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "transfers.json")
FLOWS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "flows.json")
KIN_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "error_kinematics.json")
SANDBOXES_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "sandboxes.json")
RECIPE_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "recipe.json")


def _srv_holder(start_fn):
    """断线/恢复钩子（W2）：drop=shutdown+关 listen socket+断开已有连接；
    restore=重建服务并重绑端口（bind 冲突时短退避重试，防旧 socket 未净）。"""
    h = {"srv": start_fn(), "lock": threading.Lock()}

    def drop():
        with h["lock"]:
            srv = h["srv"]
            if srv is None:
                return
            h["srv"] = None
        try:
            srv.shutdown()          # 等 serve 循环退出（poll 间隔 ~0.5s）
        except Exception:
            pass
        try:
            cc = getattr(srv, "close_connections", None)
            if cc:
                cc()                # 真机拔线：已有连接一并断
        except Exception:
            pass
        try:
            srv.server_close()      # 关 listen socket：新连接被拒（ECONNREFUSED）
        except Exception:
            pass

    def restore():
        with h["lock"]:
            if h["srv"] is not None:
                return
        srv = None
        for _ in range(50):         # 最多等 5s：旧 listen socket 释放需时间
            try:
                srv = start_fn()
                break
            except OSError:
                time.sleep(0.1)
        if srv is None:
            raise RuntimeError("断线恢复失败：端口重绑超时")
        with h["lock"]:
            h["srv"] = srv

    return drop, restore


def main():
    sys.setswitchinterval(0.002)   # 更频繁的线程切换，压低 WS 发送延迟
    # 端口可用环境变量覆盖（默认=生产端口；独立测试套件用 93xx 段避免与
    # 已启动的生产后端冲突，见 tests/test_metrology.py）
    def _port(env, dfl):
        try:
            return int(os.environ.get(env, "") or dfl)
        except ValueError:
            return dfl
    p_ws = _port("D2W_PORT_WS", 9010)
    p_acs = _port("D2W_PORT_ACS", 9101)
    p_plc = _port("D2W_PORT_PLC", 9102)
    p_mcs = _port("D2W_PORT_MCS", 9190)
    p_mqtt = _port("D2W_PORT_MQTT", 1883)
    p_vtcp = _port("D2W_PORT_VTCP", 9200)
    if not os.path.exists(IO_JSON):
        print("[boot] io_points.json 不存在，从点位表重新解析...")
        gen_io_points.main()
    if not os.path.exists(ERROR_MANIFEST):
        print("[boot] error_maps 不存在，生成合成误差地图...")
        gen_error_maps.generate()
    points = Points(IO_JSON)
    engine = Engine(points)
    engine.errors = ErrorSystem()
    # 2026-08-31 迭代 9 一次性迁移：旧 machine_constants.json 的 pid 段 +
    # bond 段 6 硬件键 → tuning.json，噪声 2 键 → recipe.json sim_sensors
    # （须在 Recipe/MachineConstants 构造之前，防旧值丢失/启动被拒）
    from tuning import Tuning, migrate_legacy
    migrate_legacy()
    engine.recipe = Recipe(RECIPE_JSON)      # 处方（晶圆/键合位/蓝膜布局）
    engine.tuning = Tuning()                 # 下位机调参（PID 整定+bond_hw）
    engine.workpieces = Workpieces(TRANSFERS_JSON, recipe=engine.recipe)
    engine.flow = FlowEngine(engine, points, FLOWS_JSON)   # W4 工艺流程编排
    from chain_runner import ChainRunner
    engine.chain = ChainRunner(engine)        # W4+ 一键串联（迭代 7）
    from full_wafer_runner import FullWaferRunner
    engine.full_wafer = FullWaferRunner(engine)  # 全片键合（迭代 11）
    from bond_results import BondResults
    BOND_RESULTS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "bond_results.json")
    engine.bond_results = BondResults(BOND_RESULTS_JSON)  # 键合结果（迭代 11）
    # 6DOF 误差合成（工具点位姿误差层，与单轴读数层独立）
    engine.kinematics = ErrorKinematics(engine.errors, KIN_JSON)
    # 全机模态仿真（2026-09-08 迭代 22）：每轴 SISO+全机 MIMO 模态 1kHz
    # 解析传播，参数落盘 runtime/modal.json；开关默认 OFF（POST /api/modal
    # 热开，运行时内存态不持久化）。outputs 模块键按 error_kinematics.json
    # 模块名校验；轴读数层/6DOF 层注入引用同步接线
    from modal import ModalSystem
    engine.modal = ModalSystem(
        module_names=list(engine.kinematics.config_public()["modules"]))
    engine.errors.modal = engine.modal        # actual() 轴 SISO 分量
    engine.kinematics.modal = engine.modal    # compute() 模块 MIMO 投影
    # 视觉系统：MQTT broker(1883) + 视觉服务(TCP 9200 图像接收)，全独立线程。
    # engine 只在 camera_trigger 尾部 put_nowait 通知，绝不阻塞 1kHz tick。
    # 机器常数（相机内参单一事实源）先于视觉加载；vision_cameras.json 或
    # machine_constants.json 坏配置在构造时抛错、拒绝拉起。
    from machine_constants import MachineConstants
    from mqtt_broker import MqttBroker
    from vision_service import VisionService
    from calibration import CalibrationService
    engine.constants = MachineConstants()
    # 组件材质工程物性（2026-09-06 迭代 21）：组件目录只读解析前端模型
    # JSON（文件缺失时组件列表为空但服务照常拉起），材料库/映射落盘
    # materials.json/material_map.json
    from materials import ComponentCatalog, Materials
    engine.components = ComponentCatalog()
    engine.materials = Materials(engine.components)
    # 组件级热仿真（2026-09-09 迭代 23）：LPTN 热网络（168 组件归并 21 热
    # 节点，能量守恒真网络），参数落盘 runtime/thermal.json；开关默认 OFF
    # （POST /api/thermal 热开，运行时内存态不持久化，OFF 状态保持）。
    # deform_map 模块键按 error_kinematics.json 模块名校验；轴读数层
    # scale/6DOF 层注入引用同步接线；modal 热软化 ΔT 源经 engine.thermal
    # 热查（OFF 回退 heater.cur）
    from thermal import ThermalSystem
    engine.thermal = ThermalSystem(
        catalog=engine.components, materials=engine.materials,
        module_names=list(engine.kinematics.config_public()["modules"]))
    engine.errors.thermal = engine.thermal      # actual()/sample6() 热 scale 分量
    engine.kinematics.thermal = engine.thermal  # compute() 模块热变形 6DOF
    mqtt_broker = MqttBroker(port=p_mqtt).start()
    engine.vision = VisionService(engine, points,
                                  constants=engine.constants,
                                  mqtt_port=p_mqtt, tcp_port=p_vtcp).start()
    # 视觉 MQTT 服务化：服务端订阅 vision/trigger/+ 触发识别并经 _publish
    # 唯一出口发布 vision/result/{cam}（载荷 {req_id, result}）；主控客户
    # 端供 flow_engine 四个拍照点真订阅消费（须在 broker 起来之后启动）
    from vision_service import VisionClient
    engine.vision.start_mqtt(port=p_mqtt)
    engine.vision_client = VisionClient(port=p_mqtt).start()
    # 光谱共焦读数模型（调焦调平高度测量层）：依赖 constants(focus_level)
    # 与 vision（相机焦面=共焦中点共面基准），须在两者之后装配
    from confocal import ConfocalModel, ConfocalClient
    engine.confocal = ConfocalModel(engine)
    # 共焦 MQTT 服务化：服务端订阅 confocal/trigger/+ 触发测量并发布
    # confocal/result/{probe}；主控客户端供 flow_engine fl_measure 真订阅
    # 消费（须在 mqtt_broker 起来之后启动）
    engine.confocal.start_mqtt(port=p_mqtt)
    engine.confocal_client = ConfocalClient(port=p_mqtt).start()
    # 测校线协议适配器（迭代 26）：复刻参考测校框架 MQTT 命令/反馈链
    # （GUI_DR_ACS_Motion 命令 + VSC_Com 视觉命令 → CTL_DR_ACS_Motion
    # RunTimeInfo 10Hz + VSS_Com 结果帧），默认开，D2W_MQTT_CMD_ADAPTER=0
    # 关闭；须在 mqtt_broker 起来之后启动
    if os.environ.get("D2W_MQTT_CMD_ADAPTER", "1") != "0":
        from mqtt_cmd_adapter import MqttCmdAdapter
        engine.mqtt_cmd = MqttCmdAdapter(engine, engine.vision) \
            .start(port=p_mqtt)
    # 测校系统：三个标定程序走真实视觉链路（REAL acquire+identify），
    # 结果经机器常数校验写回；作业线程独立，点动走引擎正常 move 路径
    engine.calibration = CalibrationService(engine, points, engine.vision)
    # 硅片台常保温（处方 bond.wafer_keep_warm=true）：启动加载即设 Heat_Dw
    # 目标 wafer_temp_C 并常开 PID
    engine.apply_wafer_keep_warm()
    # 沙盒干涉检查（W5）：sandboxes.json 缺失时停用（tools/gen_sandboxes.py 生成）
    # 【旁路副本】现成 sandboxes.json 来自旧 21 轴模型，与细化前端 1928_front_light
    # 的零件/坐标不一致——实测会误拦交接（FA_X→-1242 报对准相机×翻转模块 7 对冲突）
    # 与键合（BS_Arm_Z→-148 报上气环×对准相机承载台）。因此本副本默认关闭，
    # D2W_COLLISION=1 可强制打开；重建盒子的方法见 差距报告_轻量前端适配_V1.md §5。
    if os.environ.get("D2W_COLLISION", "0") != "1":
        print("[boot] 碰撞检查按副本默认关闭（sandboxes 来自旧模型；D2W_COLLISION=1 打开）",
              flush=True)
        engine.collision = None
    elif os.path.exists(SANDBOXES_JSON):
        from collision import CollisionChecker
        engine.collision = CollisionChecker(SANDBOXES_JSON)
    else:
        print("[boot] sandboxes.json 不存在，沙盒干涉检查停用"
              "（tools/gen_sandboxes.py 生成）", flush=True)
    n_ov = engine.load_overrides()
    print("[boot] error system: %d maps, enabled=%s; config overrides: %d axes; "
          "workpieces: %d dies, %d transfer rules; flows: %d; collision: %s; "
          "vision: %s(MQTT:%d/TCP:%d)"
          % (len(engine.errors._maps), engine.errors.enabled, n_ov,
             len(engine.workpieces.dies), len(engine.workpieces.rules),
             len(engine.flow.flows),
             ("off" if engine.collision is None else
              "%d boxes/%d pairs" % (len(engine.collision.boxes),
                                     len(engine.collision.pairs))),
             engine.vision.mode, p_mqtt, p_vtcp),
          flush=True)
    gw = WSGateway(engine, points, port=p_ws)
    engine.add_frame_listener(gw.broadcast)

    phys = threading.Thread(target=engine.run, name="physics", daemon=True)
    phys.start()
    # 三目标各注册断线/恢复钩子（drop/restore 供 /api/fault 调用）
    for target, start_fn in (
            ("acs", lambda: start_acs_server(engine, port=p_acs)),
            ("plc", lambda: start_plc_server(engine, points, port=p_plc)),
            ("mcs", lambda: start_mcs_server(engine, points, port=p_mcs))):
        engine.register_fault_hooks(target, *_srv_holder(start_fn))
    gw.start()
    print("[boot] runtime up. axes=%d io_points=%d  (Ctrl+C 退出)"
          % (len(engine.axes), len(points)), flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        engine.shutdown()
        print("\n[boot] bye.")


if __name__ == "__main__":
    main()
