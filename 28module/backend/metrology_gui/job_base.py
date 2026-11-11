# -*- coding: utf-8 -*-
"""
Filename: job_base.py
Description:
    作业型测校项公共基类（四项后端作业测校共用：光谱共焦重复性/
    运动台移动重复性/芯片交接重复性/硅片台旋转重复性）。

    定位：只负责"参数下发 + 作业轮询 + 进度/日志/结果分发"的机械
    部分（对应参考框架里"设备测校服务"的调用侧）；每个测校项仍是
    独立的一对 Xxx.py（逻辑）+ Xxx_ui.py（三段式界面），参数区、
    结果可视化、统计表全部各写各的——本基类不含任何界面与结果
    解读逻辑。

    后端契约（runtime/calibration.py + ws_gateway.py，已就绪）：
      POST /api/calibration/start  {routine, cam, params} → job_id
      GET  /api/calibration/status?job_id=  state(running/done/error/
           canceled) + progress{i,n,note} + log[...]（环形 200 行）
      GET  /api/calibration/result?job_id=  {ok, result}
      POST /api/calibration/cancel {job_id}（协作式，循环边界生效）
      GET  /api/calibration/jobs  {active, jobs[...]}（启动前查占用）

    线程模型：run() 在 ScriptWidget 的 worker 线程内执行，UI 更新
    全部走 log()/set_progress()/set_realtime_data() 信号。
"""
import json
import time
import urllib.error
import urllib.request

from metrology_config import MetrologyConfig
from script_widget import ScriptWidget

BASE_URL = "http://127.0.0.1:9010"


def http_json(method, path, body=None, timeout=10):
    """HTTP JSON 请求（纯标准库）。4xx/5xx 带后端 error 原文抛 RuntimeError。"""
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(BASE_URL + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        detail = e.read().decode("utf-8", "replace")
        try:
            detail = json.loads(detail).get("error") or detail
        except Exception:
            pass
        raise RuntimeError("HTTP %d：%s" % (e.code, detail))
    except urllib.error.URLError as e:
        raise RuntimeError("后端仿真未在线（%s）——请先运行 start.bat"
                           % getattr(e, "reason", e))


def fetch_axes_pos(timeout=5):
    """GET /api/axes → {轴名: pos}（"取当前位"按钮回填用）。"""
    body = http_json("GET", "/api/axes", timeout=timeout)
    rows = body.get("axes") if isinstance(body, dict) else body
    out = {}
    for row in rows or []:
        if isinstance(row, dict) and row.get("name"):
            out[row["name"]] = row.get("pos")
    return out


def fetch_axes_state(timeout=5):
    """GET /api/axes → {轴名: {"pos":…, "actual":…}}（pos=名义指令位，
    actual=实际位，row 缺 actual 键时给 None——调用方降级写空列不阻塞）。
    签名与 fetch_axes_pos 并存：回填类场景继续用 fetch_axes_pos。"""
    body = http_json("GET", "/api/axes", timeout=timeout)
    rows = body.get("axes") if isinstance(body, dict) else body
    out = {}
    for row in rows or []:
        if isinstance(row, dict) and row.get("name"):
            out[row["name"]] = {"pos": row.get("pos"),
                                "actual": row.get("actual")}
    return out


def fetch_shot_world(cam_id, col, row, timeout=5):
    """GET /api/vision/solve_pixel → {cam_world_mm, mark_world_mm}
    （迭代 48 动态 CSV 富化：拍照像素→世界坐标反算，后端
    vision_model/camera_pose/solve_pixel 同链，含运动台实际位姿与
    rot_mrad 控制器认知）。失败（后端不在线/旧后端无此端点/真机）
    返回 None——调用方降级写空列不阻塞测校。"""
    try:
        body = http_json(
            "GET",
            "/api/vision/solve_pixel?cam=%s&col=%.4f&row=%.4f"
            % (cam_id, col, row), timeout=timeout)
    except Exception:
        return None
    if not isinstance(body, dict) or not body.get("ok"):
        return None
    return body


class JobRoutineBase(ScriptWidget):
    """作业型测校项基类。子类须给 ROUTINE，并实现 build_cam()/
    build_params()/plot_realtime_data()；ui 须含 btn_run/
    btn_interrupt/progress_bar/textEdit（各 _ui.py 三段式提供）。"""

    ROUTINE = ""
    TITLE = ""
    POLL_S = 0.5

    def __init__(self):
        super().__init__()
        self.config = MetrologyConfig()
        self._job_id = None
        self.running = False

    # ---------------- 子类钩子 ----------------
    def build_routine(self, dict_input):
        """由输入参数决定 routine 契约名（默认用类常量 ROUTINE）。"""
        return self.ROUTINE

    def build_cam(self, dict_input):
        """由输入参数决定 cam 契约值（探头/观测相机）。"""
        raise NotImplementedError

    def build_params(self, dict_input):
        """由输入参数组装 POST start 的 params dict。"""
        return dict(dict_input)

    def result_log_lines(self, res):
        """结果摘要日志行（data saved 行由基类统一落）。"""
        return []

    # ---------------- 运行控制（GUI 线程） ----------------
    def bind_run_controls(self):
        self.ui.btn_run.clicked.connect(self._on_run_clicked)
        self.ui.btn_interrupt.clicked.connect(self.stop_)
        self.sig_progress.connect(self._on_progress)

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

    # ---------------- 作业执行（worker 线程） ----------------
    def run(self, dict_input, dict_mc):
        routine = self.build_routine(dict_input)
        cam = self.build_cam(dict_input)
        params = self.build_params(dict_input)
        self.log("%s 启动：routine=%s cam=%s params=%s"
                 % (self.TITLE, routine, cam, params))
        # 启动前查占用：后端已有测校作业在跑则拒绝启动（比 409 更明确的
        # 提示）；查询失败不阻断（后端离线时 start 会给出更直接的报错）
        try:
            jobs = http_json("GET", "/api/calibration/jobs", timeout=5)
            active = jobs.get("active")
        except Exception as e:
            active = None
            print("[job_base] jobs 占用查询失败（不阻断启动）: %s" % e)
        if active is not None:
            raise RuntimeError("后端有测校作业 #%s 在跑，请先等待其完成"
                               "或中断该作业后再启动" % active)
        r = http_json("POST", "/api/calibration/start",
                      {"routine": routine, "cam": cam,
                       "params": params}, timeout=15)
        jid = r.get("job_id")
        if not r.get("ok") or not jid:
            raise RuntimeError("启动失败：%s" % (r.get("error") or r))
        self._job_id = jid
        self.log("作业 #%d 已启动" % jid)
        self.running = True
        finished = False
        try:
            last_n = 0
            while self.running:
                st = http_json("GET", "/api/calibration/status?job_id=%d"
                               % jid, timeout=10)
                logs = st.get("log") or []
                for line in logs[last_n:]:
                    # "data saved:" 行由基类在完成后按 result.csv_name
                    # 统一落（避免与后端日志重复）
                    if not str(line).startswith("data saved:"):
                        self.log(line)
                last_n = len(logs)
                prog = st.get("progress") or {}
                if prog.get("n"):
                    self.set_progress(prog.get("i", 0), prog.get("n", 0))
                state = st.get("state")
                if state in ("done", "error", "canceled"):
                    if state == "error":
                        raise RuntimeError("测校失败：%s"
                                           % (st.get("error") or ""))
                    if state == "canceled":
                        self.log("作业 #%d 已被后端取消" % jid)
                        return {"canceled": True}
                    finished = True
                    break
                time.sleep(self.POLL_S)
        finally:
            self.running = False
            self._job_id = None
        if not finished:
            # 中断路径：interrupt() 已向后端发 cancel，不再拉结果
            self.log("作业 #%d 已中断" % jid)
            return {"canceled": True}
        res = http_json("GET", "/api/calibration/result?job_id=%d" % jid,
                        timeout=15).get("result") or {}
        self.log("作业 #%d 完成（%.1fs）" % (jid, res.get("elapsed_s") or 0))
        for line in self.result_log_lines(res):
            self.log(line)
        csv_name = res.get("csv_name") or res.get("csv") or ""
        if csv_name:
            self.log("data saved: %s" % csv_name)
        # 结果可视化（经信号回 GUI 线程，子类 plot_realtime_data 绘制）
        self.set_realtime_data(res)
        # job_id 随输出透出：有写回能力的测校项（camera_microrot）
        # "写回"按钮按它 POST /api/calibration/apply
        return {"stats": res.get("stats") or {}, "csv": csv_name,
                "job_id": jid}

    def interrupt(self):
        """中断（GUI 线程）：停轮询 + 请求后端取消当前作业（协作式，
        后端在循环边界生效；只停轮询后端作业仍会跑完）。"""
        was_running = self.running
        self.running = False
        jid = self._job_id
        if not (was_running and jid):
            return
        try:
            r = http_json("POST", "/api/calibration/cancel",
                          {"job_id": jid}, timeout=5)
            self.log("已请求取消后端作业 #%d（%s）"
                     % (jid, r.get("state") or r))
        except Exception as e:
            print("[job_base] cancel job #%s 失败（忽略）: %s" % (jid, e))
