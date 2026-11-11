# -*- coding: utf-8 -*-
"""
Filename: AlignStabilityDynamic.py
Description:
    相机对准重复性·动态（对照参考框架 AlignStabilityDynamic.py）。
    运动类型随相机映射：up→对准台 AS_X/Y、coarse→翻转模块 FA_X 仅X
    （Y 输入禁用）为只读标签；down 相机为运动组下拉——晶圆台
    WS_LS_X/Y（默认，jog_group=wafer）/ 对准台 AS_X/Y（jog_group=
    align，Y 输入保持可用），与后端 /api/calibration start 的
    jog_group 契约同名。
    流程：循环{move 起始位 → wait_stage_stop → 等待 → move 拍照位 →
    wait_stage_stop → 等待 → take_photo → 记轴读数/世界坐标} → CSV → 双曲线
    （row/col 子图，标题 3σ nm，np.std ddof=0 × 3 × nm_per_px）。
    CSV（迭代 49 起 20 列行基准，参考非正交 nodes CSV 口径）：index/col/
    row + d_col_px/d_row_px（相对第 0 帧像素差）+ t_s（相对开始时刻秒）+
    AS_X/AS_Y/WS_LS_X/WS_LS_Y 四轴名义读数 + 四轴 actual 实际位
    （HTTP /api/axes，fetch_axes_state）+ mark_world_x/y、cam_world_x/y
    （/api/vision/solve_pixel 反算，含运动台实际位姿与 rot_mrad 控制器
    认知链）+ mark_world_dx/dy_nm（相对第 0 帧世界坐标差×1e6）；取数
    失败降级空列不阻塞。

    与参考框架的刻意偏差：
    - take_photo 失败抛 TakePhotoError 时经 sig_error 中止作业，
      绝不把 0 值 append 进统计（参考语义超时返回 [0,0] 会被照单
      全收，正是 3σ 爆表的根因）；异常文本含后端 error 原文
      （模板未建/视野无标记等，经 mqtt_receive_worker.last_image_error
      透传不截断）；
    - 迭代 42：删 warm-up 首拍（迭代 27 曾"首循环 move 到拍照位后
      先打 1 拍结果丢弃"，29 起移到拍照位）——后端 _warmup 启动
      已预热，GUI 侧 warm-up 多余且是拍照链卡等重灾区；首拍即
      测量拍。

    迭代 39：起始位置/拍照位置各配"取当前位"按钮（job_base.fetch_axes_pos
    纯 HTTP GET /api/axes，与 MQTT 链无关）——按相机+运动组
    （jog_group）映射 camera_stage_map 轴组回填 axes[0]→x、axes[1]→y；
    coarse 仅回填 X（Y 框本就禁用）；后端离线仅追加日志不弹窗。
"""
import os
import threading
import time

import numpy as np
import pandas as pd
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import QApplication, QVBoxLayout

from metrology_config import MetrologyConfig
from script_widget import ScriptWidget
from job_base import fetch_axes_pos, fetch_axes_state, fetch_shot_world
from cmdjson.PahoMqttClient import PahoMqttClient
from mqtt_receive_worker import MqttReceiveWorker
import device_actuator
import AlignStabilityDynamic_ui as ui


class AlignStabilityDynamic(ScriptWidget):
    TOPICS = ['CTL_DR_ACS_Motion', "VSS_Com"]

    def __init__(self):
        super().__init__()
        self.ui = ui.Ui_Widget()
        self.ui.setupUi(self)
        self.running = False
        self.event = threading.Event()

        self.config = MetrologyConfig()
        # 启动时刷新 nm_per_px（后端不在线则保留初值）
        refreshed = self.config.refresh_nm_per_px()
        self.ui.add_camera_dict(self.config.camera_dict)
        self._sync_stage_type(self.ui.camera_ID.currentText())
        self.ui.camera_ID.currentTextChanged.connect(self._sync_stage_type)
        self.ui.stage_move_combo.currentIndexChanged.connect(
            lambda _i: self._sync_stage_type(self.ui.camera_ID.currentText()))

        # MQTT（后端不在线时允许界面照常打开，运行时才报错）
        self.paho_client = PahoMqttClient(log_cb=self.log)
        try:
            self.paho_client.connect(str(self.config.MQTTHOST),
                                     int(self.config.MQTTPORT))
            self.paho_client.subscribe(self.TOPICS)
        except Exception as e:
            self.ui.textEdit.append("[warn] MQTT broker 未连接（%s），"
                                    "开始测校前请先启动后端仿真" % e)
        # log_cb：陈旧帧/camID 不符丢弃落 Operation Log（迭代 42）
        self.mqtt_receive_worker = MqttReceiveWorker(self.paho_client,
                                                     self.event,
                                                     log_cb=self.log)

        self.res_fig = Figure()
        self.res_ax_row = self.res_fig.add_subplot(211)
        self.res_ax_col = self.res_fig.add_subplot(212)
        self.res_fig.tight_layout()
        self.res_canvas = FigureCanvas(self.res_fig)
        self.res_fig_layout = QVBoxLayout()
        self.res_fig_layout.addWidget(self.res_canvas)
        self.ui.res_display.setLayout(self.res_fig_layout)

        # 运行控制
        self.ui.btn_run.clicked.connect(self._on_run_clicked)
        self.ui.btn_interrupt.clicked.connect(self.stop_)
        self.sig_progress.connect(self._on_progress)
        # 取当前位（纯 HTTP GET /api/axes，与 MQTT 链无关）
        self.ui.btn_snap_start.clicked.connect(
            lambda: self._snap_pos(self.ui.start_x, self.ui.start_y,
                                   "起始位置"))
        self.ui.btn_snap_photo.clicked.connect(
            lambda: self._snap_pos(self.ui.photo_x, self.ui.photo_y,
                                   "拍照位置"))
        self.ui.textEdit.append(
            "nm_per_px: %s（%s）" % (
                {k: v["nm_per_px"] for k, v in self.config.camera_info.items()},
                "已从 machine_constants 刷新" if refreshed else "初值，后端未在线"))

    def _sync_stage_type(self, camera_cn):
        """运动类型随相机映射：down 相机显示运动组下拉（晶圆台默认 /
        对准台，对准台有 Y 故 Y 输入保持可用）；up/coarse 显示只读标签，
        coarse 仅 X 向禁用 Y 输入。"""
        cam_id = self.config.camera_dict.get(camera_cn, "up")
        if cam_id == "down":
            self.ui.stage_move_type.setVisible(False)
            self.ui.stage_move_combo.setVisible(True)
            stage = self.config.camera_stage_map[self._jog_group_key()]
        else:
            self.ui.stage_move_combo.setVisible(False)
            self.ui.stage_move_type.setVisible(True)
            stage = self.config.camera_stage_map[cam_id]
            self.ui.stage_move_type.setText(stage["name_cn"])
        has_y = len(stage["axes"]) > 1
        self.ui.start_y.setEnabled(has_y)
        self.ui.photo_y.setEnabled(has_y)

    def _jog_group_key(self):
        """down 相机当前下选运动组对应的 camera_stage_map 键
        （wafer 晶圆台→"down" / align 对准台→"align"）。"""
        return "align" if self.ui.stage_move_combo.currentIndex() == 1 else "down"

    def jog_group(self):
        """当前 jog_group 契约值（仅 down 相机有意义）：wafer / align。"""
        cam_id = self.config.camera_dict.get(self.ui.camera_ID.currentText(),
                                             "up")
        if cam_id != "down":
            return ""
        return "align" if self._jog_group_key() == "align" else "wafer"

    # ---------------- 取当前位（GUI 线程；纯 HTTP，与 MQTT 无关） ----------------
    def _snap_stage_key(self):
        """取当前位用的 camera_stage_map 键：down 按运动组下拉
        （wafer→"down" 晶圆台 / align→"align" 对准台），up/coarse 按相机。"""
        cam_id = self.config.camera_dict.get(self.ui.camera_ID.currentText(),
                                             "up")
        if cam_id == "down":
            return self._jog_group_key()
        return cam_id

    def _snap_pos(self, sb_x, sb_y, name):
        """GET /api/axes 按相机+运动组映射轴组回填 (x,y)（axes[0]→x、
        axes[1]→y；coarse 轴组仅 FA_X 故只回填 X）；失败仅追加日志。"""
        try:
            pos = fetch_axes_pos(timeout=5)
        except Exception as e:
            self.ui.textEdit.append("[warn] 取当前位失败：%s" % e)
            return
        axes = self.config.camera_stage_map[self._snap_stage_key()]["axes"]
        vx = pos.get(axes[0])
        vy = pos.get(axes[1]) if len(axes) > 1 else None
        if vx is not None:
            sb_x.setValue(float(vx))
        if vy is not None:
            sb_y.setValue(float(vy))
        self.ui.textEdit.append(
            "%s 已回填当前位：x=%s y=%s（运动组 %s）"
            % (name, sb_x.value(), sb_y.value(), "/".join(axes)))

    # ---------------- 参数 ----------------
    def get_saved_param(self):
        INPUT = {
            'camera_ID': self.ui.camera_ID.currentText(),
            'vision_template': self.ui.cb_vision_template.text(),
            'repeat_time': int(self.ui.repeat_times.value()),
            'wait_time': int(self.ui.wait_time.value()),
            'start_pos_x': float(self.ui.start_x.value()),
            'start_pos_y': float(self.ui.start_y.value()),
            'photo_pos_x': float(self.ui.photo_x.value()),
            'photo_pos_y': float(self.ui.photo_y.value()),
            'jog_group': self.jog_group(),
        }
        return INPUT, {}

    def set_saved_param(self, param_dict, mc):
        self.ui.camera_ID.setCurrentText(param_dict.get('camera_ID', '上精对准'))
        self.ui.cb_vision_template.setText(param_dict.get('vision_template', ''))
        self.ui.repeat_times.setValue(param_dict.get('repeat_time', 100))
        self.ui.wait_time.setValue(param_dict.get('wait_time', 10))
        self.ui.start_x.setValue(param_dict.get('start_pos_x', 0))
        self.ui.start_y.setValue(param_dict.get('start_pos_y', 0))
        self.ui.photo_x.setValue(param_dict.get('photo_pos_x', 0))
        self.ui.photo_y.setValue(param_dict.get('photo_pos_y', 0))
        self.ui.stage_move_combo.setCurrentIndex(
            1 if param_dict.get('jog_group') == 'align' else 0)

    # ---------------- 运行（worker 线程；UI 更新只走信号） ----------------
    def run(self, dict_input, dict_mc):
        if not self.paho_client.get_connect_statue():
            raise RuntimeError("MQTT broker 未连接，请先启动后端仿真")
        cam_id = self.config.camera_dict[dict_input['camera_ID']]
        vision_template = dict_input['vision_template'].strip()
        if not vision_template:
            raise ValueError("视觉模板名称必填（模板须在视觉页已注册）")
        repeat_time = int(dict_input['repeat_time'])
        wait_time = float(dict_input['wait_time'])
        k_nm = self.config.nm_per_px(cam_id)
        # down 相机可选 jog_group=align（对准台 AS_X/Y）；其余按相机映射
        stage_key = "align" if (cam_id == "down" and
                                dict_input.get('jog_group') == "align") \
            else cam_id
        axes = self.config.camera_stage_map[stage_key]["axes"]
        start_pos = [dict_input['start_pos_x'], dict_input['start_pos_y']][:len(axes)]
        photo_pos = [dict_input['photo_pos_x'], dict_input['photo_pos_y']][:len(axes)]

        self.running = True
        self.event.clear()
        self.mqtt_receive_worker.start()
        row_list, col_list = [], []
        ctx_rows = []                    # 每拍 (轴状态{pos/actual}, 世界坐标, t_s)
        degraded_logged = False
        t0 = time.time()                 # CSV t_s 列基准（相对开始时刻秒）
        self.log("test start: camera=%s jog_group=%s axes=%s start=%s "
                 "photo=%s repeat=%d wait_ms=%s"
                 % (cam_id, dict_input.get('jog_group') or "-", axes,
                    start_pos, photo_pos, repeat_time, wait_time))
        try:
            for i in range(repeat_time):
                if not self.running:
                    break
                device_actuator.move_stage(axes, start_pos, self.paho_client,
                                           self.mqtt_receive_worker)
                self.event.wait(wait_time / 1000.0)
                device_actuator.move_stage(axes, photo_pos, self.paho_client,
                                           self.mqtt_receive_worker)
                self.event.wait(wait_time / 1000.0)
                # 拍照（失败抛 TakePhotoError → sig_error 中止，
                # 绝不把 0 值 append 进统计）
                row, col = device_actuator.take_photo(
                    cam_id, vision_template, self.paho_client,
                    self.mqtt_receive_worker)
                row_list.append(row)
                col_list.append(col)
                # 数据富化（参考非正交行基准记录口径）：四轴名义读数+
                # 实际位（HTTP /api/axes，fetch_axes_state）+ 拍照像素→
                # 世界坐标反算（/api/vision/solve_pixel，含运动台实际位姿
                # 与 rot_mrad 认知链）；取数失败降级空列+日志一次，不阻塞测校
                axes_now, world = {}, None
                try:
                    axes_now = fetch_axes_state(timeout=5)
                except Exception:
                    pass
                world = fetch_shot_world(cam_id, col, row, timeout=5)
                if (not axes_now or world is None) and not degraded_logged:
                    degraded_logged = True
                    self.log("[warn] 轴读数/世界坐标获取失败（后端不在线或"
                             "旧后端无 solve_pixel 端点），CSV 对应列留空")
                ctx_rows.append((axes_now, world, time.time() - t0))
                self.log("detect result: row: %.2f, col: %.2f" % (row, col))
                self.log("cycle time: %d / %d" % (i + 1, repeat_time))
                self.set_progress(i + 1, repeat_time)
        finally:
            self.mqtt_receive_worker.stop()

        if not row_list:
            self.log("no data, test aborted")
            return {}

        # save data（迭代 49 行基准 20 列：像素 + 相对第 0 帧像素差 +
        # t_s + 四轴名义/实际读数 + 标记/相机世界坐标 + 相对第 0 帧标记
        # 世界坐标差 nm——每行一个测量循环，单位进列名，与非正交 nodes
        # CSV 同族）
        saved_columns_name = ['index', 'col', 'row',
                              'd_col_px', 'd_row_px', 't_s',
                              'AS_X_mm', 'AS_Y_mm',
                              'WS_LS_X_mm', 'WS_LS_Y_mm',
                              'AS_X_act_mm', 'AS_Y_act_mm',
                              'WS_LS_X_act_mm', 'WS_LS_Y_act_mm',
                              'mark_world_x_mm', 'mark_world_y_mm',
                              'mark_world_dx_nm', 'mark_world_dy_nm',
                              'cam_world_x_mm', 'cam_world_y_mm']

        def _f(v, nd=7):
            return "" if v is None else round(float(v), nd)

        def _ax(axes_now, name, key):
            st = axes_now.get(name)
            return st.get(key) if isinstance(st, dict) else None

        # 第 0 帧基准：像素与标记世界坐标（缺数据则差值列留空）
        col0, row0 = col_list[0], row_list[0]
        mw0 = (ctx_rows[0][1] or {}).get("mark_world_mm") or [None, None]
        save_data = []
        for i, (axes_now, world, t_rel) in enumerate(ctx_rows):
            mw = (world or {}).get("mark_world_mm") or [None, None, None]
            cw = (world or {}).get("cam_world_mm") or [None, None, None]
            mdx_nm = mdy_nm = None
            if mw[0] is not None and mw[1] is not None \
                    and mw0[0] is not None and mw0[1] is not None:
                mdx_nm = (float(mw[0]) - float(mw0[0])) * 1e6
                mdy_nm = (float(mw[1]) - float(mw0[1])) * 1e6
            save_data.append([i, col_list[i], row_list[i],
                              round(col_list[i] - col0, 4),
                              round(row_list[i] - row0, 4),
                              round(t_rel, 3),
                              _f(_ax(axes_now, "AS_X", "pos")),
                              _f(_ax(axes_now, "AS_Y", "pos")),
                              _f(_ax(axes_now, "WS_LS_X", "pos")),
                              _f(_ax(axes_now, "WS_LS_Y", "pos")),
                              _f(_ax(axes_now, "AS_X", "actual"), 9),
                              _f(_ax(axes_now, "AS_Y", "actual"), 9),
                              _f(_ax(axes_now, "WS_LS_X", "actual"), 9),
                              _f(_ax(axes_now, "WS_LS_Y", "actual"), 9),
                              _f(mw[0], 6), _f(mw[1], 6),
                              _f(mdx_nm, 3), _f(mdy_nm, 3),
                              _f(cw[0], 6), _f(cw[1], 6)])
        save_file_name = "align_stability_dynamic_%s.csv" \
                         % time.strftime('%m%d_%H_%M_%S')
        os.makedirs(self.config.DATA_SAVE_ROOT, exist_ok=True)
        pd.DataFrame(save_data, columns=saved_columns_name).to_csv(
            self.config.DATA_SAVE_ROOT + save_file_name, index=False)
        self.log("data saved: %s" % save_file_name)
        self.log("提示: mark_world 与台读数之差为固定装配偏置(工具点+标记"
                 "设计位置)，逐行应恒定")

        # display result（经信号回 GUI 线程绘图）
        self.set_realtime_data({'row': row_list, 'col': col_list,
                                'nm_per_px': k_nm})
        return {'csv': save_file_name, 'n': len(row_list)}

    def interrupt(self):
        self.event.set()
        self.running = False
        self.mqtt_receive_worker.stop()

    # ---------------- GUI 线程槽 ----------------
    def plot_realtime_data(self, data):
        row_list, col_list = data['row'], data['col']
        k_nm = data['nm_per_px']
        self.res_ax_row.clear()
        self.res_ax_col.clear()
        self.res_ax_row.plot(list(range(len(row_list))), row_list)
        self.res_ax_row.set_title("row: %.3f nm (3σ)"
                                  % (np.std(row_list) * 3 * k_nm))
        self.res_ax_row.set_ylabel("pixel")
        self.res_ax_col.plot(list(range(len(col_list))), col_list)
        self.res_ax_col.set_title("col: %.3f nm (3σ)"
                                  % (np.std(col_list) * 3 * k_nm))
        self.res_ax_col.set_ylabel("pixel")
        self.res_fig.tight_layout()
        self.res_canvas.draw()

    def _on_run_clicked(self):
        self.ui.btn_run.setEnabled(False)
        self.ui.btn_interrupt.setEnabled(True)
        self.ui.progress_bar.setValue(0)
        self.begin_()

    def _on_progress(self, cur, total):
        self.ui.progress_bar.setMaximum(max(total, 1))
        self.ui.progress_bar.setValue(cur)

    def _on_finish(self, output):
        super()._on_finish(output)
        self.ui.btn_run.setEnabled(True)
        self.ui.btn_interrupt.setEnabled(False)

    def _on_error(self, err_msg):
        super()._on_error(err_msg)
        self.ui.btn_run.setEnabled(True)
        self.ui.btn_interrupt.setEnabled(False)


if __name__ == '__main__':
    import sys
    myapp = QApplication(sys.argv)
    test = AlignStabilityDynamic()
    test.show()
    sys.exit(myapp.exec_())
