# -*- coding: utf-8 -*-
"""vision_service.py — 视觉服务：相机运行时状态机 + TCP 图像接收 + 模板匹配 + MQTT 发布。

链路（REAL 模式，全真实）：
  相机渲染线程（本模块内，按当前机器状态合成灰度图）
    → TCP socket 推帧到 9200（帧格式：[4B BE json头长][json头{cam,w,h,t,seq,
       axes,enc}][4B BE 图像字节长][图像字节]——真 socket 真传输；enc="raw"
       （2026-09-05 迭代 17 起 REAL 内部链路默认）= 直接 img.tobytes() 灰度
       字节免 PNG 编解码，enc 缺省="png" 保持旧兼容）
    → 接收线程解帧、cv2 模板匹配（NCC 全图粗定位 → 全分辨率开窗 NCC
       +3×3 抛物线亚像素位置（2026-09-09 迭代 23，取代粗网格亚像素+边缘
       质心）→ 角度 ±3° 锚点精化 + 环形扇区灰度质心星座几何角连续化
       （迭代 23，取代 0.2° 量化，噪声地板 <100µrad）；比例精化仿真
       scale 恒 1.0 默认关闭（Matcher.refine_scale））
    → 结果组装 {strShapeName, judge, lstCol, lstRow, lstAngle, lstScore, lstScale}
       （strShapeName=本次触发模板名，多模板 "+" 连接——迭代 16 契约修订）
    → mqtt_publish 到 broker(1883) 的 vision/result/{cam}（迭代 8B 起载荷
     外包 {req_id, result[, error]}、迭代 31 起附 cam 字段；非 MQTT 触发
     req_id=null；另订阅
     vision/trigger/+ 供主控 VisionClient 触发识别，见 start_mqtt/
     VisionClient 与 docs/前后端数据协议_V1.md §9）。

FAST 模式：跳过大图渲染/TCP/模板匹配，由 vision_model 解析投影器
直接合成同 schema 结果（仍经本服务与 MQTT 出口）；live 预览图仍走真实渲染
（降采样 640×480）——FAST 省的是匹配链路，画面始终真实。
默认模式 REAL（2026-09-05 迭代 17 起；cv2 缺失时启动回退 fast 并打印警告）。

相机两模式（每相机独立）：
- continuous：光源常开 → 持续有画面（REAL 下内部节流 ~5Hz 推 live 帧）；
  identify 指令对当前帧匹配。
- trigger：闪光——收到触发（PLC 硬触发 AlignCameraTigger_1/2 / AS_CameraOn，
  或 /api/vision/acquire 软触发）→ 若关联光源灭则自动点亮（走 points.set，
  吃 4ms PLC 扫描语义）→ 等曝光 exposure_ms → 以帧时刻轴位+误差渲染
  → 光源灭 → 处理发布。

线程模型（绝不允许拖慢 engine 1kHz tick）：engine.camera_trigger 只做
queue.put_nowait 通知（满则丢最旧）；渲染/传输/匹配/MQTT 全部在本模块的
独立线程。cv2/numpy lazy import——cv2 缺失时 REAL 报清晰错误，FAST 可用。
"""
import json
import math
import os
import queue
import socket
import struct
import threading
import time
from collections import deque

import vision_model as vm
from mqtt_broker import MqttSub, mqtt_publish

# 视觉 MQTT 服务化契约（docs/前后端数据协议_V1.md §9）：
# 触发 vision/trigger/{cam} 载荷 {req_id, cam, template?}
# 结果 vision/result/{cam} 载荷 {req_id, cam, result:<七字段>[, error]}
# （cam 迭代 31 起随载荷回填——认领按 topic 后缀+req_id+cam；
#  非 MQTT 触发的结果 req_id=null，出口统一走 _publish；
#  未知相机触发有 req_id 时回 error 帧到同名结果 topic，不静默丢弃）
TRIGGER_TOPIC = "vision/trigger/%s"       # % cam
RESULT_TOPIC = "vision/result/%s"         # % cam
TRIGGER_FILTER = "vision/trigger/+"
RESULT_FILTER = "vision/result/+"

ROOT = os.path.dirname(os.path.abspath(__file__))
TEMPLATES_DIR = os.path.join(ROOT, "vision_templates")
TEMPLATES_JSON = os.path.join(ROOT, "vision_templates.json")
CAPTURES_DIR = os.path.join(ROOT, "vision_captures")
CAPTURES_JSON = os.path.join(ROOT, "vision_captures.json")

RESULTS_MAX = 100
TRIGGER_QUEUE_MAX = 8
LIVE_FPS = 5.0                 # continuous 模式 live 帧节流
REAL_FRAME_TIMEOUT_S = 12.0    # REAL 9200 帧处理等待上限（2026-09-05 迭代 17：
                               # 5→12s，留足 9MP 渲染+匹配余量防误判超时卡住）
MATCH_ANGLE_RANGE = 3.0        # 角度精化 ±3°
MATCH_ANGLE_COARSE_STEP = 1.0  # 角度粗搜步长（2026-09-05 迭代 17：0.6→1.0，
                               # 细搜跨度跟随粗搜步长，0.2° 细步不变）
MATCH_ANGLE_STEP = 0.2
MATCH_SCALE_RANGE = 0.02       # 比例精化 ±2%（仿真 scale 恒 1.0，默认关闭，
                               # 见 Matcher.refine_scale）
MATCH_SCALE_STEP = 0.005
MATCH_MAX_INSTANCES = 16       # NMS 多标记上限
MATCH_COARSE_MAX_PX = 1024.0 * 768.0   # 全图粗定位像素预算（历史基准
                                       # 1280×960，2026-09-05 迭代 17 收紧到
                                       # 1024×768：NCC 成本∝预算²，峰位量化
                                       # 损失由亚像素+精化窗口吸收；超过则
                                       # 同比例降采样再 NCC）
DEFAULT_TEMPLATE_SIZE = None     # 建模板默认 ROI：None=画幅短边 3/4 取奇
                                 #（须含标记边缘特征：0.6µm/px 时代标记可
                                 # 与 FOV 同级（2mm cross vs 1.8mm FOV），
                                 # 1/4 短边会整块落进标记臂内部均匀区导致
                                 # NCC 零方差退化；3/4 稳含臂缘/背景过渡）
# 相机内参字段（机器常数，单一事实源 machine_constants.json）——
# /api/vision/camera/config 收到这些字段一律 400 指路 /api/machine_constants
# （px_x/px_y 是迁入前的旧键名，一并拦截指路 px_u/px_v）
INTRINSIC_FIELDS = ("pixel_um", "mag", "dir_x", "dir_y", "swap_xy",
                    "rot_mrad", "wd_mm", "dof_um", "px_u", "px_v",
                    "px_x", "px_y")

# 对焦状态枚举（identify/acquire 结果 entry 与 /api/vision/cameras 运行态
# 共用，写进 docs/视觉系统_V1.md §4）：
#   no_mark   视野无任何标记（无工件/载体空）——几何事实，优先判定
#   light_off 有标记但光源灭（暗图必废）
#   in_focus  blur_ratio ≤ 1（在景深内）
#   defocused 1 < blur_ratio ≤ 3（离焦但未到判废线）
#   rejected  blur_ratio > 3（离焦判废，同 judge=false 语义）
# 拍帧异常（_record_error）的 entry 三字段一律 null（error 有明细）。
FOCUS_STATES = ("in_focus", "defocused", "rejected", "no_mark", "light_off")

# 模态曝光积分（2026-09-08 迭代 22）：相机标记来源 → 承载模块（modal
# outputs 键）；曝光窗相对振动 = 标记模块 − 相机安装模块（None=固连机架）
_MARK_MODULE = {"wafer_marks": "wafer_stage", "flip_die_marks": "flip",
                "film_die_marks": "film_stage", "head_die_marks": "bond_head"}


def _motion_blur(img, vib_sig_mm, angle_rad, k_mm_px):
    """运动模糊核滤波（模态曝光积分，REAL）：按曝光窗位移峰峰值
    vib_pp=2√2·σ 生成线型核（核长=round(vib_pp_px)，<2px 无感不处理，
    限幅 21px 防极端参数拖垮渲染线程；方向=主导模态投影方向）。"""
    pp_px = 2.0 * math.sqrt(2.0) * vib_sig_mm / max(k_mm_px, 1e-9)
    klen = min(21, int(round(pp_px)))
    if klen < 2:
        return img
    cv2, n = vm.cv2_or_none(), vm.np()
    kern = n.zeros((klen, klen), dtype=n.float32)
    kern[klen // 2, :] = 1.0 / klen
    m = cv2.getRotationMatrix2D((klen / 2.0 - 0.5, klen / 2.0 - 0.5),
                                math.degrees(angle_rad), 1.0)
    kern = cv2.warpAffine(kern, m, (klen, klen))
    kern /= kern.sum()
    return cv2.filter2D(img, -1, kern)


def _focus_state(marks, light_on, blur_ratio):
    """对焦状态分档。优先级：no_mark > light_off > blur 分档。
    blur_ratio = 目标标记（最靠近光轴）的离焦比（marks 非空时必填）。"""
    if not marks:
        return "no_mark"
    if not light_on:
        return "light_off"
    if blur_ratio <= 1.0:
        return "in_focus"
    if blur_ratio <= vm.BLUR_REJECT:
        return "defocused"
    return "rejected"

# Windows 线程优先级（真机实时线程语义）：物理线程 HIGHEST（engine.run 内
# 自提），视觉重线程一律 BELOW_NORMAL——REAL 匹配的大量小 cv2 调用不释放
# GIL，同优先级下会稳定压制 1kHz 物理忙等线程（perf_guard 贴线根因）。
# 2026-09-05 迭代 17：全链路 sleep(0) 让步 + 精化迭代减半后实测让步已够，
# 匹配/相机线程提到 NORMAL 消 GIL 稀释（BELOW_NORMAL 下 42ms 的匹配被
# 稀释到 400ms+），accept/light/warmup 维持 BELOW_NORMAL；perf_guard 回归
# 守住 tick≥999/push≥499 即不推翻 2026-08-21 的修复结论。
PRIO_BELOW_NORMAL = -1
PRIO_NORMAL = 0


def _set_os_prio(prio):
    """设当前线程 OS 优先级（ctypes 标准库）；非 Windows/失败静默降级。
    坑：必须 c_void_p 收伪句柄——默认 c_int 会把 GetCurrentThread 的 -2
    零扩展成 0xFFFFFFFE，SetThreadPriority 静默失败（已实测验证）。"""
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.GetCurrentThread.restype = ctypes.c_void_p
        k32.SetThreadPriority(k32.GetCurrentThread(), prio)
    except Exception:
        pass


class VisionError(RuntimeError):
    """视觉链路错误（API 映射 400，消息须清晰可定位）。"""


# ---------------- TCP 帧编解码 ----------------
def encode_frame(header, png_bytes):
    h = json.dumps(header).encode("utf-8")
    return struct.pack("!I", len(h)) + h \
        + struct.pack("!I", len(png_bytes)) + png_bytes


def _recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ConnectionError("eof")
        buf += chunk
    return buf


def read_frame(conn):
    """读一帧 -> (header dict, png bytes)；断开抛 ConnectionError。"""
    hlen = struct.unpack("!I", _recv_exact(conn, 4))[0]
    if hlen > 65536:
        raise ValueError("header too large")
    header = json.loads(_recv_exact(conn, hlen).decode("utf-8"))
    ilen = struct.unpack("!I", _recv_exact(conn, 4))[0]
    if ilen > 32 << 20:
        raise ValueError("image too large")
    return header, _recv_exact(conn, ilen)


# ---------------- 模板库 ----------------
class TemplateStore(object):
    """runtime/vision_templates/*.png + vision_templates.json 索引。
    模板 = 从指定相机当前帧取 ROI 存的灰度图 + 元数据
    {name, cam, shape, threshold, roi, w, h, created}。"""

    def __init__(self, path=TEMPLATES_JSON, dir_=TEMPLATES_DIR):
        self.path = path
        self.dir = dir_
        self._lock = threading.Lock()
        self._d = {}
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                self._d = json.load(f).get("templates", {})

    def _save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"_comment": "视觉模板库索引（PNG 在 vision_templates/）",
                       "templates": self._d}, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.path)

    def list(self):
        with self._lock:
            return json.loads(json.dumps(self._d))

    def get(self, name):
        with self._lock:
            return self._d.get(name)

    def for_cam(self, cam):
        with self._lock:
            return [dict(t) for t in self._d.values() if t["cam"] == cam]

    def put(self, name, meta):
        with self._lock:
            self._d[name] = meta
            self._save()

    def remove(self, name):
        with self._lock:
            meta = self._d.pop(name, None)
            if meta is None:
                raise VisionError("模板不存在: %r" % name)
            self._save()
        try:
            os.remove(os.path.join(self.dir, meta["file"]))
        except OSError:
            pass

    def rename(self, name, new_name):
        with self._lock:
            meta = self._d.pop(name, None)
            if meta is None:
                raise VisionError("模板不存在: %r" % name)
            if new_name in self._d:
                self._d[name] = meta
                raise VisionError("模板名已存在: %r" % new_name)
            old_file = meta["file"]
            meta = dict(meta)
            meta["name"] = new_name
            meta["file"] = "tpl_%s.png" % new_name
            self._d[new_name] = meta
            self._save()
        try:
            os.replace(os.path.join(self.dir, old_file),
                       os.path.join(self.dir, meta["file"]))
        except OSError:
            pass


# ---------------- 采图库 ----------------
# 从采图建模板的最小 ROI 边长与灰度方差下限：过小/过于均匀的 ROI 让 NCC
# 退化（零方差响应面平坦，匹配结果无意义），建模板时就拦下指路。
TEMPLATE_ROI_MIN = 16
TEMPLATE_ROI_MIN_VAR = 25.0


class CaptureStore(object):
    """runtime/vision_captures/*.png + vision_captures.json 索引。
    采图 = POST /api/vision/capture/save 按当前机器状态真实渲染的全幅灰度图
    + 元数据 {name, cam, file, ts, axes, focus_state, light_on}。"""

    def __init__(self, path=CAPTURES_JSON, dir_=CAPTURES_DIR):
        self.path = path
        self.dir = dir_
        self._lock = threading.Lock()
        self._d = {}
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                self._d = json.load(f).get("captures", {})

    def _save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"_comment": "视觉采图库索引（PNG 在 vision_captures/）",
                       "captures": self._d}, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self.path)

    def list(self):
        with self._lock:
            return json.loads(json.dumps(self._d))

    def get(self, name):
        with self._lock:
            return self._d.get(name)

    def put(self, name, meta):
        with self._lock:
            self._d[name] = meta
            self._save()

    def remove(self, name):
        with self._lock:
            meta = self._d.pop(name, None)
            if meta is None:
                raise VisionError("采图不存在: %r" % name)
            self._save()
        try:
            os.remove(os.path.join(self.dir, meta["file"]))
        except OSError:
            pass

    def read_png(self, name):
        """取采图 (meta, PNG 字节)；索引没有/文件丢失都报清晰错误。"""
        meta = self.get(name)
        if meta is None:
            raise VisionError("采图不存在: %r" % name)
        path = os.path.join(self.dir, meta["file"])
        try:
            with open(path, "rb") as f:
                return meta, f.read()
        except OSError:
            raise VisionError("采图文件丢失: %s（索引仍在 %s，可 DELETE "
                              "/api/vision/captures 清掉该条目）"
                              % (path, self.path))


# ---------------- 模板匹配器（REAL，cv2 lazy） ----------------
class Matcher(object):
    """cv2 NCC 模板匹配：全图粗定位 → NMS 多实例 →
    全分辨率开窗 NCC + 3×3 抛物线亚像素位置精化（2026-09-09 迭代 23，
    取代粗网格亚像素+边缘质心修正——Otsu 二值掩膜是像素量化的，亚像素
    位移丢失，实测系统偏差 (−0.17,−0.04)px / 散布 0.49px；全分辨率 NCC
    亚像素实测散布 0.03px、均值 ~0，见 Temp/_i23_real_precision_probe.py）
    → 角度精化（±3° 粗搜步 MATCH_ANGLE_COARSE_STEP → 三点抛物线锚点）
    → 几何角连续化（迭代 23：环形扇区灰度加权质心星座，输出连续角取代
    0.2° 量化——measured drz 不再恒 0，实测噪声地板 47~51µrad <100µrad
    目标；方案选型依据：NCC 旋转模板路线受 warp 插值偏差+位置补偿平台
    限制（实测 ~2000µrad），几何质心无插值偏差且 4 重对称星座天然对消
    中心误差，详见 probe 脚本与模块尾注）。比例精化（±2%/0.5%）默认关闭
    （refine_scale=False——仿真相机正交俯视 scale 恒 1.0，九步
    warpAffine+NCC 纯浪费）。"""

    refine_scale = False       # 比例精化开关（真机接入需要时再开）
    centroid_refine = False    # 边缘质心位置修正开关（迭代 17 引入；
                               # 迭代 23 起默认关——被全分辨率 NCC 亚像素
                               # 取代（Otsu 二值量化损失亚像素灵敏度）；
                               # 保留开关便于 A/B 回退

    # 全分辨率位置精化的模板像素上限：超出（大模板，如 coarse 相机
    # 1201×1164）回退粗网格亚像素（精化收益不抵成本；coarse 对准无需
    # 亚像素）；400×400 精对准/套刻模板远在下限内
    REFINE_POS_MAX_TPL_PX = 600000
    # 几何角模板像素上限（mgrid 全量分配成本；cv_chipback 1.4M 在内）
    GEOM_ANGLE_MAX_TPL_PX = 1500000

    def __init__(self):
        self.cv2 = vm.cv2_or_none()
        if self.cv2 is None:
            raise VisionError(
                "cv2 未安装，REAL 模式不可用（pip install opencv-python，"
                "用户已批准视觉链路用 OpenCV）；FAST 模式不受影响")
        self.np = vm.np()
        self._tpl_cache = {}        # name -> (mtime, np array)
        self._centroid_cache = {}   # name -> (mtime, (cx,cy) | None)
        self._mask_cache = {}       # name -> (mtime, mask | None)
        self._geom_cache = {}       # name -> (mtime, annuli | None)
        self._mask_prefer = set()   # 上次靠掩膜重试救回的模板名（下次该模板
                                    # 掩膜优先——套刻连拍省一次无效粗 NCC）

    def load_template_img(self, meta):
        path = os.path.join(TEMPLATES_DIR, meta["file"])
        mtime = os.path.getmtime(path)
        ent = self._tpl_cache.get(meta["name"])
        if ent and ent[0] == mtime:
            return ent[1]
        # cv2.imread 在 Windows 不支持非 ASCII 路径（本项目目录含中文），
        # 走 open()+imdecode 绕过
        with open(path, "rb") as f:
            data = f.read()
        img = self.cv2.imdecode(
            self.np.frombuffer(data, dtype=self.np.uint8),
            self.cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise VisionError("模板图像读取失败: %s" % path)
        self._tpl_cache[meta["name"]] = (mtime, img)
        return img

    def template_centroid(self, meta, tpl):
        """模板内形状质心（模板像素坐标），按 name+mtime 缓存——
        template_update/rename/重建改文件后 mtime 变即自动失效重算。
        质心求不出（低对比/无边缘）时 None（匹配侧回退 NCC 峰位）。"""
        path = os.path.join(TEMPLATES_DIR, meta["file"])
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        ent = self._centroid_cache.get(meta["name"])
        if ent and ent[0] == mtime:
            return ent[1]
        c = self._shape_centroid(tpl)
        self._centroid_cache[meta["name"]] = (mtime, c)
        return c

    def _shape_centroid(self, img, mask=None):
        """暗色形状质心：Otsu 反阈值（暗标记→白掩膜）+ cv2.moments。
        低对比（方差不足）/掩膜过小（无形状）/掩膜过大（极性反或满幅）
        一律 None 让调用方回退。
        mask=模板形状掩膜（2026-09-07 迭代 20）：套刻同帧时命中窗里混有
        配对标记的暗像素（quad 窗里的 cross / cross 窗里的 quad 四角块），
        无掩膜质心被拖向对方（REAL 套刻 measured≈0 假象的根因）；相与后
        只统计本模板形状覆盖的暗像素。相与面积过小时回退无掩膜口径。"""
        cv2, n = self.cv2, self.np
        if img is None or img.shape[0] < 8 or img.shape[1] < 8:
            return None
        if float(n.var(img.astype(n.float32))) < 25.0:
            return None
        _th, otsu = cv2.threshold(
            img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        if mask is not None and mask.shape == otsu.shape:
            anded = cv2.bitwise_and(otsu, mask)
            if float(n.count_nonzero(anded)) >= 50.0:
                otsu = anded           # 面积够才用掩膜口径，否则回退
        m = cv2.moments(otsu, binaryImage=True)
        area = m["m00"]
        if area < 50.0 or area > 0.9 * img.shape[0] * img.shape[1]:
            return None
        return (m["m10"] / area, m["m01"] / area)

    def _shape_mask(self, img):
        """模板形状掩膜（Otsu 反阈值 + 3×3 膨胀一档）：只保留暗标记本体
        像素参与匹配。面积越界（<2% 无形状 />60% 近似满幅）时 None——
        调用方回退无掩膜匹配。"""
        cv2, n = self.cv2, self.np
        if img is None or img.shape[0] < 8 or img.shape[1] < 8:
            return None
        if float(n.var(img.astype(n.float32))) < 25.0:
            return None
        _th, mask = cv2.threshold(
            img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        area = float(n.count_nonzero(mask))
        frac = area / float(img.shape[0] * img.shape[1])
        if frac < 0.02 or frac > 0.60:
            return None
        return cv2.dilate(mask, n.ones((3, 3), dtype=n.uint8))

    def template_mask(self, meta, tpl):
        """模板形状掩膜（按 name+mtime 缓存，同 template_centroid 失效语义）。
        用途（2026-09-05 迭代 17）：套刻同帧双标记场景——quad 模板窗口
        中央会被叠印的 cross 占据，无掩膜 NCC 被压到阈下；掩膜只留 quad
        角块像素，cross 干扰区不参与相关计算。"""
        path = os.path.join(TEMPLATES_DIR, meta["file"])
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        ent = self._mask_cache.get(meta["name"])
        if ent and ent[0] == mtime:
            return ent[1]
        mk = self._shape_mask(tpl)
        self._mask_cache[meta["name"]] = (mtime, mk)
        return mk

    # ---------------- 迭代 23：几何角（环形扇区灰度质心星座） ----------------
    def template_annuli(self, meta, tpl):
        """模板几何分析（按 name+mtime 缓存，失效语义同 template_mask）：
        Otsu 掩膜 → 连通域判型（≥3 块=quad 类→轴象限分割；1 块=cross 类
        →对角象限分割）→ 每象限取外 40% 半径像素簇（臂尖/块外缘，角度
        灵敏度最高且排除 cross 轮毂稀释）→ 环形扇区 {dir, half（簇角展
        ×1.3+2°），r0, r1, 模板簇质心}。扇区+半径带的选择区域对旋转不变
        （无矩形 ROI 的径向边界通量滞后），且与配对标记在角度/半径上天然
        分离（quad 块在对角、cross 臂在轴向）——套刻同帧污染免掩膜排除。
        分析失败（无形状/扇区空）返回 None（调用方回退 NCC 锚点角）。"""
        path = os.path.join(TEMPLATES_DIR, meta["file"])
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        ent = self._geom_cache.get(meta["name"])
        if ent and ent[0] == mtime:
            return ent[1]
        out = self._build_annuli(tpl)
        self._geom_cache[meta["name"]] = (mtime, out)
        return out

    def _build_annuli(self, tpl, outer_frac=0.4):
        cv2, n = self.cv2, self.np
        if tpl is None or tpl.shape[0] < 8 or tpl.shape[1] < 8:
            return None
        if float(n.var(tpl.astype(n.float32))) < 25.0:
            return None
        _th, m = cv2.threshold(
            tpl, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        ncomp, _lab = cv2.connectedComponents(m)
        h, w = m.shape
        cx0, cy0 = w / 2.0, h / 2.0
        ys, xs = n.nonzero(m)
        if len(xs) < 100:
            return None
        dx, dy = xs - cx0, ys - cy0
        if ncomp - 1 >= 3:           # quad 类多块（角块在对角线上）→ 轴象限
            conds = [(dx >= 0) & (dy >= 0), (dx < 0) & (dy >= 0),
                     (dx < 0) & (dy < 0), (dx >= 0) & (dy < 0)]
        else:                        # cross 类单块（臂在轴向上）→ 对角象限
            conds = [(n.abs(dx) >= n.abs(dy)) & (dx >= 0),
                     (n.abs(dx) < n.abs(dy)) & (dy >= 0),
                     (n.abs(dx) >= n.abs(dy)) & (dx < 0),
                     (n.abs(dx) < n.abs(dy)) & (dy < 0)]
        out = []
        for c in conds:
            if c.sum() < 50:
                return None
            r = n.hypot(dx[c], dy[c])
            thr = n.quantile(r, 1.0 - outer_frac)
            cc = c.copy()
            cc[c] = r >= thr
            xs2, ys2 = xs[cc], ys[cc]
            ang = n.degrees(n.arctan2(ys2 - cy0, xs2 - cx0))
            rr = n.hypot(xs2 - cx0, ys2 - cy0)
            a0 = float(n.degrees(math.atan2((ys2 - cy0).mean(),
                                            (xs2 - cx0).mean())))
            d = n.abs((ang - a0 + 180) % 360 - 180)
            out.append({"dir": a0,
                        "half": min(44.0, float(d.max()) * 1.3 + 2.0),
                        "r0": max(0.0, float(rr.min()) - 3.0),
                        "r1": float(rr.max()) + 3.0,
                        "tpl_c": (float((xs2 - cx0).mean()),
                                  float((ys2 - cy0).mean()))})
        return out

    def _geom_angle(self, win, annuli):
        """命中窗（模板尺寸）→ 连续角（与 getRotationMatrix2D 同号约定，
        同 NCC 精化输出）。环形扇区内灰度加权质心星座：w=max(bg−I−gate,
        0)（gate=背景噪声门限，只留显著变暗像素——截断正噪声权重是此前
        角度噪声主因）；4 簇 atan2 差圆均值（4 重对称对消提取中心误差）。
        符号：图像系 y 向下，内容正向旋转（getRotationMatrix2D 正向）使
        簇 atan2 减小 → 返回 −meanΔ。返回 None=失败（回退锚点角）。"""
        cv2, n = self.cv2, self.np
        h, w = win.shape
        img32 = win.astype(n.float32)
        # 背景灰与噪声门限：窗角 10px 边带统计（MAD→σ 稳健估计）
        band = n.concatenate([img32[:10, :].ravel(), img32[-10:, :].ravel(),
                              img32[:, :10].ravel(), img32[:, -10:].ravel()])
        bg = float(n.median(band))
        sig = float(n.median(n.abs(band - bg))) / 0.6745
        gate = max(3.0, 2.0 * sig)
        wmap = n.clip(bg - img32 - gate, 0, None)
        ys, xs = n.mgrid[0:h, 0:w]
        dx, dy = xs - w / 2.0, ys - h / 2.0
        rr = n.hypot(dx, dy)
        aa = n.degrees(n.arctan2(dy, dx))
        ds = []
        for an in annuli:
            d = n.abs((aa - an["dir"] + 180) % 360 - 180)
            sel = (d <= an["half"]) & (rr >= an["r0"]) & (rr <= an["r1"])
            ws = wmap[sel]
            s = float(ws.sum())
            if s < 1e-6:
                return None
            wx = float((dx[sel] * ws).sum() / s)
            wy = float((dy[sel] * ws).sum() / s)
            tx, ty = an["tpl_c"]
            dd = math.degrees(math.atan2(wy, wx) - math.atan2(ty, tx))
            while dd > 45.0:
                dd -= 90.0
            while dd < -45.0:
                dd += 90.0
            ds.append(dd)
        ds.sort()
        md = ds[len(ds) // 2] if len(ds) % 2 else 0.5 * (ds[1] + ds[2])
        # 抗差：剔 |d−中位数|>0.1° 的离群簇；全离群（形状失真/离焦严重）
        # 时回退中位数——任何情况下本函数不得抛异常（匹配主链兜底在
        # match() 的 try/except，见下）
        keep = [x for x in ds if abs(x - md) <= 0.1] or [md]
        return -float(sum(keep) / len(keep))

    def _refine_pos(self, img, tpl, peak_xy, cscale, angle=0.0, mask=None):
        """全分辨率位置精化（迭代 23）：粗定位峰位开窗（margin 覆盖粗网格
        量化+旋转余量）→ 全分辨率 NCC（模板按终角旋转，位置-角度耦合解
        算）→ 3×3 邻域抛物线亚像素（cx=(L−R)/(2(L−2C+R)) 标准式，X/Y 独
        立——沿用 _subpixel）。peak_xy=(px,py) 粗网格坐标。返回
        (col, row, score)（模板中心图像坐标）或 None（窗口越界/模板过大）。"""
        cv2, n = self.cv2, self.np
        th, tw = tpl.shape[:2]
        if th * tw > self.REFINE_POS_MAX_TPL_PX:
            return None
        margin = max(8, int(math.ceil(2.5 / cscale)))
        cx0, cy0 = peak_xy[0] / cscale, peak_xy[1] / cscale
        x0 = max(0, int(round(cx0)) - margin)
        y0 = max(0, int(round(cy0)) - margin)
        x1 = min(img.shape[1], int(round(cx0)) + tw + margin)
        y1 = min(img.shape[0], int(round(cy0)) + th + margin)
        win = img[y0:y1, x0:x1]
        if win.shape[0] < th or win.shape[1] < tw:
            return None
        if angle:
            m = cv2.getRotationMatrix2D((tw / 2.0, th / 2.0), angle, 1.0)
            tpl_u = cv2.warpAffine(tpl, m, (tw, th),
                                   borderMode=cv2.BORDER_REPLICATE)
            if mask is not None:
                mask_u = cv2.warpAffine(mask, m, (tw, th),
                                        flags=cv2.INTER_NEAREST,
                                        borderValue=0)
            else:
                mask_u = None
        else:
            tpl_u, mask_u = tpl, mask
        res = cv2.matchTemplate(win, tpl_u, cv2.TM_CCOEFF_NORMED,
                                mask=mask_u)
        _mv, mx, _ml, loc = cv2.minMaxLoc(res)
        dx, dy = self._subpixel(res, loc[0], loc[1])
        return (x0 + loc[0] + dx + tw / 2.0, y0 + loc[1] + dy + th / 2.0,
                mx)

    @staticmethod
    def _subpixel(res, x, y):
        """NCC 响应面峰位抛物线亚像素拟合。"""
        def fit(a, b, c):
            d = (a - 2 * b + c)
            return 0.0 if abs(d) < 1e-12 else 0.5 * (a - c) / d
        h, w = res.shape
        dx = fit(res[y, x - 1], res[y, x], res[y, x + 1]) \
            if 0 < x < w - 1 else 0.0
        dy = fit(res[y - 1, x], res[y, x], res[y + 1, x]) \
            if 0 < y < h - 1 else 0.0
        return dx, dy

    def match(self, img, templates, default_threshold, decode_ms=0.0):
        """-> (entries, match_ms)。img=已解码灰度 np 图（解码在 _match 按帧
        enc 做，decode_ms 计入 timing 分解）。templates: [meta...]（同一相机）。
        全图粗定位在"像素预算"内进行：图像超过 MATCH_COARSE_MAX_PX（历史
        1280×960 基准）时图像与模板同比例降采样（INTER_AREA）再 NCC，峰位
        换算回原图坐标——9MP（3000×3000）传感器下粗定位成本与旧分辨率
        同阶，物理 1kHz 红线不退化；亚像素/角度/质心精化语义不变。"""
        cv2, n = self.cv2, self.np
        t0 = time.perf_counter()
        t_dec = t0                   # 解码在 _match 按帧 enc 做（decode_ms）
        cscale = min(1.0, math.sqrt(MATCH_COARSE_MAX_PX
                                    / float(img.shape[0] * img.shape[1])))
        entries = []
        for meta in templates:
            tpl = self.load_template_img(meta)
            threshold = float(meta.get("threshold", default_threshold))
            th, tw = tpl.shape[:2]
            if img.shape[0] < th or img.shape[1] < tw:
                continue
            if cscale < 1.0:
                img_c = cv2.resize(
                    img, (max(16, int(img.shape[1] * cscale)),
                          max(16, int(img.shape[0] * cscale))),
                    interpolation=cv2.INTER_AREA)
                tpl_c = cv2.resize(
                    tpl, (max(8, int(tw * cscale)),
                          max(8, int(th * cscale))),
                    interpolation=cv2.INTER_AREA)
            else:
                img_c, tpl_c = img, tpl
            tch, tcw = tpl_c.shape[:2]

            def _nms(res_):
                """NMS 取 ≤MATCH_MAX_INSTANCES 个峰（整模板边长抑制半径：
                2026-09-05 迭代 17——quad 四角块在 ±1 块距处产生 0.68 分
                内部别名峰（超 threshold*0.8 下限），半模板半径压不住，白跑
                4~5 轮 subpixel+让步；真实多标记间距 ≫ 模板边长，不受影响）。"""
                work_ = res_.copy()
                out = []
                for _ in range(MATCH_MAX_INSTANCES):
                    _minv, maxv, _minl, maxl = cv2.minMaxLoc(work_)
                    if maxv < max(0.30, threshold * 0.8):
                        break
                    out.append((maxl[0], maxl[1], maxv))
                    x0 = max(0, maxl[0] - tcw)
                    y0 = max(0, maxl[1] - tch)
                    x1 = min(work_.shape[1], maxl[0] + tcw + 1)
                    y1 = min(work_.shape[0], maxl[1] + tch + 1)
                    work_[y0:y1, x0:x1] = -1.0
                    time.sleep(0)  # GIL 让步：别压制 1kHz 物理线程
                return out

            res = None
            used_mask = None
            msk = self.template_mask(meta, tpl)
            # 掩膜优先学习：上次该模板靠掩膜重试才命中（套刻同帧场景），
            # 本次直接掩膜先行，省掉注定失败的无掩膜粗 NCC（迭代 17）
            if msk is not None and meta["name"] in self._mask_prefer:
                msk_c = cv2.resize(msk, (tcw, tch),
                                   interpolation=cv2.INTER_NEAREST) \
                    if cscale < 1.0 else msk
                res_m = cv2.matchTemplate(img_c, tpl_c,
                                          cv2.TM_CCOEFF_NORMED, mask=msk_c)
                t_full = time.perf_counter()
                peaks = _nms(res_m)
                if peaks:
                    res, used_mask = res_m, msk
                else:
                    self._mask_prefer.discard(meta["name"])
            if res is None:
                res = cv2.matchTemplate(img_c, tpl_c, cv2.TM_CCOEFF_NORMED)
                t_full = time.perf_counter()
                peaks = _nms(res)
                if not peaks and msk is not None:
                    # 掩膜重试（迭代 17）：套刻同帧——配对标记叠印进本模板
                    # 窗口中央（quad 窗里的 cross）会把无掩膜 NCC 压到阈下；
                    # 用模板形状掩膜（只留标记本体像素）重算响应面再 NMS
                    msk_c = cv2.resize(msk, (tcw, tch),
                                       interpolation=cv2.INTER_NEAREST) \
                        if cscale < 1.0 else msk
                    res_m = cv2.matchTemplate(img_c, tpl_c,
                                              cv2.TM_CCOEFF_NORMED,
                                              mask=msk_c)
                    peaks = _nms(res_m)
                    if peaks:
                        res = res_m
                        used_mask = msk
                        self._mask_prefer.add(meta["name"])
            if not peaks:
                continue
            # 最优峰做角度精化（±3° 锚点），其余实例沿用；掩膜重试救回
            # 的模板精化同样带掩膜（掩膜随模板一起旋转），分数口径一致
            best = max(range(len(peaks)), key=lambda i: peaks[i][2])
            ang_q, scale, bs = self._refine(img, tpl, (peaks[best][0] / cscale,
                                                       peaks[best][1] / cscale,
                                                       peaks[best][2]),
                                            mask=used_mask)
            peaks[best] = (peaks[best][0], peaks[best][1], bs)
            # 迭代 23 角度连续化：最优峰全分辨率位置初值 → 命中窗环形扇区
            # 灰度质心星座几何角（连续，噪声地板 <100µrad，取代 0.2° 量化
            # ——measured drz 不再恒 0）；失败/超限时回退 NCC 锚点（量化）
            ang = ang_q
            p0 = self._refine_pos(img, tpl, (peaks[best][0], peaks[best][1]),
                                  cscale, mask=used_mask)
            if p0 is not None and th * tw <= self.GEOM_ANGLE_MAX_TPL_PX:
                try:
                    annuli = self.template_annuli(meta, tpl)
                    if annuli is not None:
                        win_g = cv2.getRectSubPix(img, (tw, th),
                                                  (p0[0] - 0.5, p0[1] - 0.5))
                        ga = self._geom_angle(win_g, annuli)
                        if ga is not None:
                            # 几何角直出模 90° 值，按 NCC 锚点解环绕
                            ang = ang_q + ((ga - ang_q + 45.0) % 90.0 - 45.0)
                except Exception:
                    # 几何角任何异常不得拖垮匹配主链——回退 NCC 锚点角
                    ang = ang_q
            # 迭代 17 边缘质心修正（迭代 23 起默认关，见 centroid_refine）
            tc = self.template_centroid(meta, tpl) \
                if self.centroid_refine else None
            ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
            for i, (px, py, sc) in enumerate(peaks):
                # 迭代 23 全分辨率开窗 NCC + 抛物线亚像素（模板按终角旋
                # 转，过阈值峰才值得算）；失败/不过阈回退粗网格亚像素
                rp = self._refine_pos(img, tpl, (px, py), cscale, angle=ang,
                                      mask=used_mask) \
                    if sc >= threshold else None
                if rp is not None:
                    col, row = rp[0], rp[1]
                    dx, dy = self._subpixel(res, px, py)
                    ncc_col = (px + dx) / cscale + tw / 2.0
                    ncc_row = (py + dy) / cscale + th / 2.0
                else:
                    dx, dy = self._subpixel(res, px, py)
                    # 粗定位在降采样网格上做 → 峰位/亚像素换算回原图坐标
                    ncc_col = (px + dx) / cscale + tw / 2.0
                    ncc_row = (py + dy) / cscale + th / 2.0
                    col, row = ncc_col, ncc_row
                # 质心修正只对过阈值的候选峰做（不过阈的判废条目不值得算）
                if tc is not None and sc >= threshold:
                    wx = int(round(px / cscale))
                    wy = int(round(py / cscale))
                    if 0 <= wx and 0 <= wy and wx + tw <= img.shape[1] \
                            and wy + th <= img.shape[0]:
                        # 迭代 20：质心精化带模板形状掩膜（msk=全分辨率
                        # 模板形状掩膜，与命中窗同尺寸对齐）——套刻同帧时
                        # 排除配对标记暗像素对质心的拖曳
                        cc = self._shape_centroid(img[wy:wy + th,
                                                      wx:wx + tw],
                                                  mask=msk)
                        if cc is not None:
                            # getRotationMatrix2D 正向角在图像系（y 向下）
                            # 的映射：dx'=cos·dx+sin·dy, dy'=−sin·dx+cos·dy
                            ox = ca * (tc[0] - tw / 2.0) \
                                + sa * (tc[1] - th / 2.0)
                            oy = -sa * (tc[0] - tw / 2.0) \
                                + ca * (tc[1] - th / 2.0)
                            c_col = wx + cc[0] - ox
                            c_row = wy + cc[1] - oy
                            lim = 0.25 * max(tw, th)
                            if abs(c_col - ncc_col) <= lim \
                                    and abs(c_row - ncc_row) <= lim:
                                col, row = c_col, c_row
                entries.append({
                    "col": col, "row": row,
                    "ncc_col": ncc_col, "ncc_row": ncc_row,   # 调试字段，
                    "angle": ang,                              # 不进 MQTT 七字段
                    "score": float(sc), "scale": scale if i == best else 1.0,
                    "judge": bool(sc >= threshold), "in_fov": True,
                    "blur": 0.0, "mark": meta["name"]})
            if os.environ.get("VISION_DEBUG_TIMING"):
                print("[match] decode=%.1f full=%.1f nms+refine=%.1f"
                      % (decode_ms, (t_full - t_dec) * 1000,
                         (time.perf_counter() - t_full) * 1000),
                      flush=True)
            time.sleep(0)          # 多模板间隔也让出 GIL
        match_ms = (time.perf_counter() - t0) * 1000.0 + decode_ms
        return entries, match_ms

    def _refine(self, img, tpl, peak, mask=None):
        """角度精化（两段：0° 快速确认 → 不满分才 ±MATCH_ANGLE_RANGE 粗扫
        （步 MATCH_ANGLE_COARSE_STEP）→ 最佳位 ±粗步两点复评做三点抛物线
        拟合，顶点量化到 MATCH_ANGLE_STEP 网格）+ 比例精化
        （±MATCH_SCALE_RANGE 步 MATCH_SCALE_STEP，self.refine_scale=False
        时整段跳过——仿真相机正交俯视 scale 恒 1.0）。在 1/4 分辨率窗口内
        重匹配——空域 NCC 成本 ∝ 模板像素×窗口像素，1/4 分辨率省 ~64×，
        角度精度不受分辨率影响。mask=模板形状掩膜（掩膜重试救回的模板，
        精化同口径带掩膜，随模板一起 warpAffine 旋转）。
        2026-09-05 迭代 17：细搜由 ±0.6°/0.2° 全扫（11 次 warpAffine+NCC）
        改抛物线拟合（2 次复评）+ 0° 快速路径（仿真标记轴对齐、安装微旋
        <0.1°，绝大多数帧 4 次评估收工）——1kHz 物理忙等持 GIL，精化段的
        Python 迭代编排是 REAL 单拍耗时大头（离线 42ms 的匹配在运行时被
        稀释到数百 ms），迭代数减半而 0.2° 量化语义不变。"""
        cv2, n = self.cv2, self.np
        th, tw = tpl.shape[:2]
        scale4 = 0.25
        tpl_h = cv2.resize(tpl, (max(8, tw // 4), max(8, th // 4)),
                           interpolation=cv2.INTER_AREA)
        th2, tw2 = tpl_h.shape[:2]
        cx, cy = tw2 / 2.0, th2 / 2.0
        if mask is not None:
            mask_h = cv2.resize(mask, (tw2, th2),
                                interpolation=cv2.INTER_NEAREST)
        else:
            mask_h = None
        # 窗口 = 峰位 ± 0.6×模板尺寸（控制精化耗时，含旋转余量），1/4 分辨率
        px, py = int(peak[0] * scale4), int(peak[1] * scale4)
        img_h = cv2.resize(img, (max(16, img.shape[1] // 4),
                                 max(16, img.shape[0] // 4)),
                           interpolation=cv2.INTER_AREA)
        pad = int(max(tw2, th2) * 0.6)
        x0 = max(0, px - pad)
        y0 = max(0, py - pad)
        x1 = min(img_h.shape[1], px + tw2 + pad)
        y1 = min(img_h.shape[0], py + th2 + pad)
        win = img_h[y0:y1, x0:x1]
        if win.shape[0] < th2 + 4 or win.shape[1] < tw2 + 4:
            return 0.0, 1.0, peak[2]

        def score_at(angle_deg, scale):
            m = cv2.getRotationMatrix2D((cx, cy), angle_deg, scale)
            rot = cv2.warpAffine(tpl_h, m, (tw2, th2),
                                 borderMode=cv2.BORDER_REPLICATE)
            if mask_h is not None:
                mk = cv2.warpAffine(mask_h, m, (tw2, th2),
                                    flags=cv2.INTER_NEAREST, borderValue=0)
            else:
                mk = None
            r = cv2.matchTemplate(win, rot, cv2.TM_CCOEFF_NORMED, mask=mk)
            s = float(cv2.minMaxLoc(r)[1])
            time.sleep(0)          # 每迭代末尾让出 GIL（小 cv2 调用不释放
                                   # GIL，否则稳定压制 1kHz 物理线程）
            return s

        best_a, best_s = 0.0, peak[2]
        # 0° 快速路径（迭代 17）：仿真标记轴对齐、安装微旋 <0.1°，0° 近满分
        # 时跳过 ±3° 全扫，直接进抛物线细搜
        s_zero = score_at(0.0, 1.0)
        if s_zero > best_s:
            best_s = s_zero
        if s_zero < 0.95:
            # 角度粗搜 ±3° 步 MATCH_ANGLE_COARSE_STEP（0° 已评过）
            a = -MATCH_ANGLE_RANGE
            while a <= MATCH_ANGLE_RANGE + 1e-9:
                if abs(a) > 1e-9:
                    s = score_at(a, 1.0)
                    if s > best_s:
                        best_a, best_s = a, s
                a += MATCH_ANGLE_COARSE_STEP
        # 角度细搜：粗最佳位 ±粗步 两点复评 → 三点抛物线顶点（NCC 响应面
        # 在峰附近光滑，抛物线拟合即达亚步进精度），量化回 0.2° 网格
        sl = score_at(best_a - MATCH_ANGLE_COARSE_STEP, 1.0)
        s0 = score_at(best_a, 1.0)
        sr = score_at(best_a + MATCH_ANGLE_COARSE_STEP, 1.0)
        den = sl - 2.0 * s0 + sr
        if abs(den) > 1e-12:
            da = 0.5 * (sl - sr) / den * MATCH_ANGLE_COARSE_STEP
            da = max(-MATCH_ANGLE_COARSE_STEP,
                     min(MATCH_ANGLE_COARSE_STEP, da))
            a_f = best_a + da
            a_f = round(a_f / MATCH_ANGLE_STEP) * MATCH_ANGLE_STEP
            if abs(a_f - best_a) > 1e-9:
                s_f = score_at(a_f, 1.0)
                if s_f > best_s:
                    best_a, best_s = round(a_f, 3), s_f
        best_sc = 1.0
        if self.refine_scale:
            n_steps = int(round(2 * MATCH_SCALE_RANGE
                                / MATCH_SCALE_STEP)) + 1
            for i in range(n_steps):
                sc = 1.0 - MATCH_SCALE_RANGE + i * MATCH_SCALE_STEP
                s = score_at(best_a, sc)
                if s > best_s:
                    best_sc, best_s = sc, s
        return best_a, best_sc, best_s


# ---------------- 迭代 28：模板噪声响应度自标定 ----------------
# 合成试验噪声幅度（DN）：远大于亚像素量化地板、远小于模板对比度（~87
# DN），停留在线性区；K=8 次试验（一次性开销 ~百 ms 级，只在建模板/旧
# 模板补标定时发生）
NOISE_PROBE_SIGMA_DN = 10.0
NOISE_PROBE_TRIALS = 5         # 2026-09-08：8→5（REAL 渲染 ~0.25s/帧下
                               # 8 试次×多级降档会超建模板 API 的 5s 客户端
                               # 超时；MAD 对 5×2=10 样本仍稳健，配合
                               # _probe_deadline 时间预算双保险）


def _robust_sigma(vals):
    """中位绝对偏差 → σ 估计（1.4826×MAD）：粗网格 NCC 亚像素在噪声下
    偶发 ~1px 跳变，普通 std 会被单点带飞（resp 实测曾因此飘 5×），
    MAD 取分布主体的 σ。vals=带符号偏差列表（调用方双轴合并）。"""
    if not vals:
        return 0.0
    n = vm.np()
    med = float(n.median(vals))
    return 1.4826 * float(n.median([abs(v - med) for v in vals]))


def noise_response_probe(matcher, meta, frame_wh):
    """模板噪声响应度自标定（2026-09-10 迭代 28）：模板贴到**相机全画幅
    尺寸**等灰背景画布中心，叠 K 次已知 σ_test 的 N(0,σ_test) 高斯噪声，
    每次走 `Matcher.match` **生产匹配链全程**（降采样粗定位+NMS+角度精化
    +位置终值——大模板自动落在粗网格亚像素路径，与线上行为一致）测质心
    峰位，散布 σ_pos/σ_test = 响应度 resp（px/DN）。连同模板梯度能量
    grad_gx=Σ(∂I/∂x)²、grad_gy（解析参考值，σ_pos≈σ_DN/√G 量级）一起
    返回 dict（键即模板 meta 字段名）。模板退化（无梯度）/匹配全失败返
    回 None——不阻塞建模板，REAL 噪声回退 K_NOISE 旧公式。
    用途：render_frame 把 fast_noise_px（质心噪声目标 px）÷resp 换算成
    图像灰度噪声 σ_gray——测校面板旋钮直接决定 REAL 链实测散布。
    一次性开销 ~秒级（9MP 画布 ×K 次全链匹配），只在建模板/旧模板补标定
    时发生。matcher 须已构造（无 cv2 时 Matcher() 抛 VisionError，调用方
    捕获）；meta["file"] 须已落盘（matcher 按文件读模板图）。
    已知局限：画布背景在模板边界截断，对长臂 cross 这类实际延伸出模板
    边界的标记系统性低估响应（实测画布 0.00012 vs 线上 0.002 px/DN，
    孔径效应）——live 建模板路径优先用 VisionService._live_noise_probe
    现场实测，本探针只作采图建模板/补标定的兜底。
    返回字段含 tpl_std_dn（模板 RMS 灰度差，DN）——render_frame 的
    SNR 封顶（σ_gray ≤ 0.45×tpl_std，再高 NCC 信号淹没）依据。"""
    n = vm.np()
    tpl = matcher.load_template_img(meta)
    if tpl is None or tpl.shape[0] < 16 or tpl.shape[1] < 16:
        return None
    f = tpl.astype(n.float32)
    gy, gx = n.gradient(f)
    gxv = float((gx * gx).sum())
    gyv = float((gy * gy).sum())
    if gxv <= 0.0 or gyv <= 0.0:
        return None
    th, tw = tpl.shape[:2]
    # 画布 = 模板 + 2×边距（2026-09-08 缩窗：原"相机全画幅"画布在 9MP
    # 相机上 5 次全链匹配 ≈4~6s，live 探针对行向歧义模板（竖直 die 边缘
    # 框出的 1D 特征——行向天然多峰，|dy| 过滤全灭）回退 canvas 时直接
    # 顶爆建模板 API 5s 客户端超时；匹配只需模板窗口+峰位邻域，缩窗后
    # 成本 ~1/100，粗定位降采样 cscale 与线上一致（小画布 cscale=1）
    W, H = (int(frame_wh[0]), int(frame_wh[1])) if frame_wh else (0, 0)
    pad = max(64, tw // 2, th // 2)
    cw = min(max(W, tw + 2 * pad), tw + 2 * pad)
    ch = min(max(H, th + 2 * pad), th + 2 * pad)
    # 画布背景灰 = 模板边缘带中位数（标记居中模板的四边即背景）
    band = n.concatenate([f[:4, :].ravel(), f[-4:, :].ravel(),
                          f[:, :4].ravel(), f[:, -4:].ravel()])
    canvas = n.full((ch, cw), float(n.median(band)), dtype=n.float32)
    x0, y0 = (cw - tw) // 2, (ch - th) // 2
    canvas[y0:y0 + th, x0:x0 + tw] = f
    thr = float(meta.get("threshold", 0.6))
    rng = vm._rng()
    dxs, dys = [], []
    for _ in range(NOISE_PROBE_TRIALS):
        noisy = canvas + rng.normal(0.0, NOISE_PROBE_SIGMA_DN,
                                    canvas.shape).astype(n.float32)
        img_u = n.clip(noisy, 0, 255).astype(n.uint8)
        entries, _ms = matcher.match(img_u, [meta], thr)
        if entries:
            e = max(entries, key=lambda e_: e_["score"])
            dxs.append(e["col"] - (x0 + tw / 2.0))
            dys.append(e["row"] - (y0 + th / 2.0))
        time.sleep(0)          # 让出 GIL：别压制 1kHz 物理线程
    if len(dxs) < NOISE_PROBE_TRIALS // 2:
        return None
    spos = _robust_sigma(dxs + dys)     # MAD 抗粗网格亚像素偶发跳变
    if spos <= 1e-9:
        return None
    return {"noise_resp_px_per_dn": spos / NOISE_PROBE_SIGMA_DN,
            "noise_resp_sigma_test_dn": NOISE_PROBE_SIGMA_DN,
            "noise_resp_trials": len(dxs),
            # 画布路径无线上识别存活证据，噪声上限不写死（2026-09-08 修复：
            # 原保守 2DN ≈ 迭代 27 前旧量级，导致旧模板（补标定路径）与新
            # 模板（live 探针 10~40DN）在同一 fast_noise_px 旋钮下静态 3σ
            # 相差数倍——旧模板测校结果只有十几 nm 的直接原因）；置 None
            # 让 template_noise_resp 走 SNR 启发式（NOISE_SNR_CAP×模板
            # RMS，绝对上限 NOISE_SIGMA_CAP_DN 兜底），与缺省分支同口径
            "noise_sigma_max_dn": None,
            "noise_resp_via": "canvas",
            "tpl_std_dn": float(n.std(f)),
            "grad_gx": gxv, "grad_gy": gyv}


# ---------------- 相机运行时 ----------------
class CameraRuntime(object):
    """每相机运行时状态（配置在 vm.CameraModel，此处是运行态）。"""

    def __init__(self, model):
        self.model = model
        self.light_on = False
        self.queue = queue.Queue(maxsize=TRIGGER_QUEUE_MAX)
        self.sock = None                 # 9200 推送连接（懒连接）
        self.sock_lock = threading.Lock()
        self.frame_seq = 0
        self.last_live = 0.0             # 上次 live 帧时刻（节流）
        self.last_result = None
        self.last_flash_ms = None        # 最近一次闪光时长（strobe 证据）


class VisionService(object):
    """视觉服务主体。main.py 注入 engine.vision 并 start()。"""

    def __init__(self, engine, points, cfg_path=vm.CAMERAS_JSON,
                 constants=None,
                 mqtt_host="127.0.0.1", mqtt_port=1883,
                 tcp_host="127.0.0.1", tcp_port=9200):
        self.engine = engine
        self.points = points
        if constants is None:
            from machine_constants import MachineConstants
            constants = MachineConstants()
        self.constants = constants      # 机器常数（相机内参单一事实源）
        self._cfg_path = cfg_path       # persist 写回的采配文件
        self.cams = {cid: CameraRuntime(m)
                     for cid, m in vm.load_cameras(cfg_path, constants).items()}
        # fast | real（全局开关）：默认 REAL（2026-09-05 迭代 17 起）；
        # cv2 不可用时回退 fast 并警告（boot log）——REAL 链路依赖 OpenCV
        if vm.cv2_or_none() is not None:
            self.mode = "real"
        else:
            self.mode = "fast"
            print("[vision] 警告：cv2 不可用，默认模式回退 fast"
                  "（REAL 需 opencv-python）", flush=True)
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.templates = TemplateStore()
        self.captures = CaptureStore()
        self.results = deque(maxlen=RESULTS_MAX)
        self._seq = 0
        self._lock = threading.RLock()
        self._srv = None
        self._latest = {}                # cam -> (header, png, recv_ts)
        self._latest_lock = threading.Lock()
        self._pending = {}               # seq -> {event, entries, match_ms, err}
        self._pending_lock = threading.Lock()
        self._matcher = None             # 懒构造（首次 REAL 需要时）
        self._stop = False

    # ================= 生命周期 =================
    def start(self):        # 迭代 33 启动维护：先清 PNG 缺失的模板索引条目（否则 warmup 的        # load_template_img 抛 FileNotFoundError），再启动补标定——REAL        # 模式由 _warmup 末尾做（含现有预热），非 REAL 模式单独起一条        # 补标定线程（服务启动时一次）        self._prune_missing_template_files()        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind((self.tcp_host, self.tcp_port))
        srv.listen(16)
        srv.settimeout(0.5)
        self._srv = srv
        threading.Thread(target=self._accept_loop, name="vision-accept",
                         daemon=True).start()
        for cid in self.cams:
            threading.Thread(target=self._camera_loop, args=(cid,),
                             name="vision-cam-%s" % cid, daemon=True).start()
        threading.Thread(target=self._light_loop, name="vision-light",
                         daemon=True).start()
        if self.mode == "real":
            threading.Thread(target=self._warmup, name="vision-warmup",
                             daemon=True).start()
        else:
            threading.Thread(target=self._calibrate_missing_resp,
                             name="vision-resp-calib", daemon=True).start()
        print("[vision] service up: TCP %s:%d, MQTT -> %s:%d, mode=%s, "
              "cams=%s" % (self.tcp_host, self.tcp_port, self.mqtt_host,
                           self.mqtt_port, self.mode,
                           ",".join(self.cams)), flush=True)
        return self

    def _prune_missing_template_files(self):
        """启动清理（迭代 33）：索引在但 PNG 文件缺失的模板条目会让
        Matcher.load_template_img 抛 FileNotFoundError（warmup 曾裸吞，
        首个匹配请求才炸）——启动时按 os.path.exists 检查模板 PNG 路径，
        缺失则删除索引项并打印警告。"""
        for name, meta in self.templates.list().items():
            f = meta.get("file")
            if f and not os.path.exists(os.path.join(TEMPLATES_DIR, f)):
                print("[vision] 警告：模板 %s 的 PNG 缺失（%s），"
                      "已删除索引条目" % (name, f), flush=True)
                try:
                    self.templates.remove(name)
                except VisionError:
                    pass

    def _warmup(self):
        """REAL 默认模式预热（2026-09-05 迭代 17）：把全幅渐晕/噪声底版
        缓存、cv2 内核、模板图像缓存预先热起来——否则首拍吃 ~600ms 冷启动
        （9MP 噪声底版生成 2×150ms 是大头）。只做大 numpy 分配/cv2 小调用
        （都让 GIL），不在此渲整帧匹配（避免启动期与物理线程争 GIL）。"""
        _set_os_prio(PRIO_BELOW_NORMAL)
        try:
            n = vm.np()
            for rt in self.cams.values():
                cam = rt.model
                W, H = int(cam.px_x), int(cam.px_y)
                vm._vignette(n, W, H)
                vm._noise_unit(n, W, H)     # 迭代 28：单位噪声底版（与 σ 解耦）
            cv2 = vm.cv2_or_none()
            if cv2 is not None:
                with self._lock:
                    if self._matcher is None:
                        self._matcher = Matcher()
                    matcher = self._matcher
                dummy = n.zeros((64, 64), dtype=n.uint8)
                cv2.matchTemplate(dummy, dummy[:8, :8],
                                  cv2.TM_CCOEFF_NORMED)   # cv2 内核热身
                for cid in self.cams:
                    for meta in self.templates.for_cam(cid):
                        matcher.load_template_img(meta)
                    # 迭代 28：预热期补标定该相机默认模板的噪声响应度
                    # （旧模板缺 resp 字段——一次性，否则首个 REAL 识别帧
                    # 在渲染线程里现算）
                    self.template_noise_resp(cid)
                # 迭代 33：预热后做一次全量补标定——其余缺 resp 的旧模板
                # 在此一次性标定落盘，否则首个 REAL 识别帧会在渲染热路径
                # 里惰性跑秒级画布探针（每帧税）
                self._calibrate_missing_resp()
        except Exception as e:
            print("[vision] warmup 警告: %s" % e, flush=True)

    def stop(self):
        self._stop = True
        try:
            if self._srv:
                self._srv.close()
        except OSError:
            pass
        for rt in self.cams.values():
            try:
                if rt.sock:
                    rt.sock.close()
            except OSError:
                pass

    # ---------------- MQTT 服务化（触发→识别→发布） ----------------
    def start_mqtt(self, host=None, port=None):
        """视觉 MQTT 服务线程：订阅 vision/trigger/+，收到触发即同步
        identify()（结果经 _publish 唯一出口发布到 vision/result/{cam}，
        req_id 原样回填；与共焦服务同一 broker、同一退避重连模式）。
        在 main.py 里于 broker 起来之后调用。"""
        self._mqtt_hp = (host or self.mqtt_host, port or self.mqtt_port)
        threading.Thread(target=self._mqtt_loop, name="vision-mqtt",
                         daemon=True).start()
        return self

    def _mqtt_loop(self):
        # MQTT 触发在同线程同步跑 identify（渲染+等匹配），迭代 17 起 NORMAL
        #（渲染/匹配的 GIL 让步网足够覆盖物理线程，BELOW_NORMAL 会被稀释）
        _set_os_prio(PRIO_NORMAL)
        host, port = self._mqtt_hp
        while not self._stop:
            try:
                sub = MqttSub(host=host, port=port, client_id="vision-svc")
                sub.subscribe(TRIGGER_FILTER)
                while not self._stop:
                    got = sub.recv(timeout=30.0)
                    if got is None:              # 闲置 recv 仅作唤醒，
                        continue                 # 连接保持（broker 不再
                    topic, payload = got         # 杀闲置；断线走外层重连）
                    try:
                        req = json.loads(payload)
                    except ValueError:
                        req = {}
                    cid = req.get("cam") or topic.rsplit("/", 1)[-1]
                    req_id = req.get("req_id")
                    if cid not in self.cams:
                        # 未知相机不再静默丢弃（2026-09-10 迭代 31 防线）：
                        # 有 req_id 时按原相机名回 error 帧，消费方按
                        # topic+req_id 认领到失败回执而非干等超时
                        if req_id is not None:
                            mqtt_publish(
                                RESULT_TOPIC % cid,
                                json.dumps({"req_id": req_id, "cam": cid,
                                            "result": vm.build_result("", []),
                                            "error": "未知相机 %r（有 up,down,coarse）"
                                            % cid},
                                           ensure_ascii=False),
                                host, port)
                        continue
                    try:
                        self.identify(cid, req.get("template"),
                                      req_id=req_id,
                                      ir_overlay=bool(
                                          req.get("ir_overlay", False)))
                    except Exception as e:
                        # entry 错误（_capture_process 已带 error 发出）不
                        # 重复发；其余失败（校验/异常）补发 error 载荷，
                        # 客户端按 error 判失败
                        if getattr(e, "published", False):
                            continue
                        mqtt_publish(
                            RESULT_TOPIC % cid,
                            json.dumps({"req_id": req_id, "cam": cid,
                                        "result": vm.build_result("", []),
                                        "error": str(e)},
                                       ensure_ascii=False),
                            host, port)
            except (ConnectionError, socket.timeout, OSError, ValueError):
                time.sleep(0.5)                  # broker 未起/断线：退避重连


    def notify_trigger(self, source, cap):
        """engine.camera_trigger 尾部调用：source→cam 映射，非阻塞入队
        （满则丢最旧）。"""
        for cid, rt in self.cams.items():
            if rt.model.trigger_source != source:
                continue
            evt = {"source": source, "cap": cap, "t0": time.perf_counter()}
            try:
                rt.queue.put_nowait(evt)
            except queue.Full:
                try:
                    rt.queue.get_nowait()       # 丢最旧
                except queue.Empty:
                    pass
                try:
                    rt.queue.put_nowait(evt)
                except queue.Full:
                    pass

    # ================= 光源跟踪（100Hz，同进程直读 points live） =================
    def _light_loop(self):
        _set_os_prio(PRIO_BELOW_NORMAL)
        while not self._stop:
            for rt in self.cams.values():
                rt.light_on = bool(self.points.get(rt.model.light_point, False))
            time.sleep(0.01)

    def _set_light(self, rt, on):
        """点亮/熄灭关联光源：写命令点 + o* 镜像观测点（走 points.set，
        吃 PLC 扫描周期语义——pending→live 最多晚一个扫描周期）。"""
        p = rt.model.light_point
        self.points.set(p, bool(on))
        parent, _, leaf = p.rpartition(".")
        mirror = parent + ".o" + leaf
        if self.points.has(mirror):
            self.points.set(mirror, bool(on))

    def _await_light(self, rt, want, timeout=0.3):
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            if bool(self.points.get(rt.model.light_point, False)) == want:
                return True
            time.sleep(0.002)
        return False

    # ================= TCP 接收服务（9200） =================
    def _accept_loop(self):
        _set_os_prio(PRIO_BELOW_NORMAL)
        while not self._stop:
            try:
                conn, addr = self._srv.accept()
            except (OSError, socket.timeout):
                continue
            try:
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            except OSError:
                pass
            threading.Thread(target=self._recv_conn, args=(conn,),
                             daemon=True).start()

    def _recv_conn(self, conn):
        _set_os_prio(PRIO_NORMAL)         # 匹配在本线程做（迭代 17 起 NORMAL，
        # 让步网足够覆盖；BELOW_NORMAL 会被物理忙等 GIL 稀释）
        # 迭代 33 韧性：外层重连循环——任何异常（不再只捕 ConnectionError/
        # ValueError/OSError，坏帧头令 header 非 dict 的 AttributeError 等
        # 同样走这里）→ 关闭 socket、退避 0.5s、重新 accept 等发送侧重连
        # （_send_frame 的失败重连路径会 create_connection 新连接）后再继续
        # 解帧，直到 _stop。原实现线程异常即死，该相机后续 pending 全部
        # 等满 REAL_FRAME_TIMEOUT_S。帧头 seq 认领协议在 _recv_stream 内，
        # 重连路径不触碰；帧路由按 header cam 字段，与连接归属无关。
        while not self._stop:
            try:
                self._recv_stream(conn)
            except Exception:
                pass            # 任何异常都走关闭+重连
            try:
                conn.close()
            except OSError:
                pass
            if self._stop:
                break
            time.sleep(0.5)     # 退避，别空转抢 accept
            try:
                conn, _addr = self._srv.accept()   # 等发送侧重连（srv 有
            except OSError:                        # 0.5s 超时，循环内可察
                continue                           # 觉 _stop）

    def _recv_stream(self, conn):
        while not self._stop:
            header, payload = read_frame(conn)
            recv_ts = time.perf_counter()
            cam = header.get("cam", "?")
            with self._latest_lock:
                self._latest[cam] = (header, payload, recv_ts)
            seq = header.get("seq")
            with self._pending_lock:
                pend = self._pending.get(seq)
            if pend is not None:
                pend["recv_ts"] = recv_ts
                try:
                    pend["entries"], pend["match_ms"] =                         self._match(header, payload, cam,
                                    pend.get("template"))
                except Exception as e:
                    pend["err"] = str(e)
                    pend["entries"], pend["match_ms"] = [], 0.0
                finally:
                    pend["event"].set()

    def _match(self, header, payload, cam, template=None):
        with self._lock:
            if self._matcher is None:
                self._matcher = Matcher()
            matcher = self._matcher
        # 帧解码按头 enc 字段（2026-09-05 迭代 17）：raw=直接灰度字节
        # （w/h 取自 json 头，免 PNG 编解码）；缺省 png 保持旧兼容
        t_dec = time.perf_counter()
        if header.get("enc", "png") == "raw":
            w, h = int(header["w"]), int(header["h"])
            n = matcher.np
            img = n.frombuffer(payload, dtype=n.uint8)
            if img.size != w * h:
                raise VisionError("raw 帧数据长度不符（%d != %dx%d）"
                                  % (img.size, w, h))
            img = img.reshape(h, w)
        else:
            img = matcher.cv2.imdecode(
                matcher.np.frombuffer(payload, dtype=matcher.np.uint8),
                matcher.cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise VisionError("帧解码失败（非 PNG？）")
        decode_ms = (time.perf_counter() - t_dec) * 1000.0
        if template:
            # 模板名或模板名列表（迭代 15：post-inspect 一张图双模板套刻）
            names = template if isinstance(template, (list, tuple)) \
                else [template]
            tpls = []
            for tn in names:
                meta = self.templates.get(tn)
                if meta is None:
                    raise VisionError("模板不存在: %r" % tn)
                tpls.append(meta)
        else:
            tpls = self.templates.for_cam(cam)
            if not tpls:
                raise VisionError(
                    "相机 %s 无模板：先 POST /api/vision/templates 建模板" % cam)
        return matcher.match(img, tpls, self.cams[cam].model.score_threshold,
                             decode_ms=decode_ms)

    # ================= 相机工作线程 =================
    def _camera_loop(self, cid):
        _set_os_prio(PRIO_NORMAL)       # 渲染在本线程做（迭代 17 起 NORMAL，
        # 让步网足够；BELOW_NORMAL 被物理忙等 GIL 稀释）
        rt = self.cams[cid]
        while not self._stop:
            try:
                evt = rt.queue.get(timeout=0.05)
            except queue.Empty:
                evt = None
            if evt is not None:
                try:
                    self._do_trigger(cid, evt["source"], evt["t0"])
                except Exception as e:
                    self._record_error(cid, evt["source"], str(e), evt["t0"])
                continue
            # continuous + 光源开 + REAL：节流推 live 帧（画面真实、只看不匹配）
            if (rt.model.mode == "continuous" and rt.light_on
                    and self.mode == "real"
                    and time.perf_counter() - rt.last_live
                    >= 1.0 / LIVE_FPS):
                try:
                    img = vm.render_frame(self.engine, rt.model,
                                          light_on=rt.light_on)
                    png = vm.png_encode_gray(img)
                    self._push_frame(cid, png, match=False)
                    rt.last_live = time.perf_counter()
                except Exception:
                    rt.last_live = time.perf_counter()  # 出错也节流，不刷错误

    def _do_trigger(self, cid, source, t0, template=None, req_id=None,
                    ir_overlay=False):
        """触发/闪光时序：光源灭→自动点亮(吃扫描语义)→等曝光→帧时刻渲染
        →(REAL: TCP→9200→匹配)→光源灭→MQTT 发布+结果入环形缓冲。"""
        rt = self.cams[cid]
        # 直读点位真值（rt.light_on 是 100Hz 镜像，连拍间隔 <10ms 时会
        # 读到上一拍灭灯前的陈旧 True → 误判"灯已开"不闪光 → 暗帧判废，
        # 2026-09-05 迭代 17 连拍实测暴露）
        flash = not bool(self.points.get(rt.model.light_point, False))
        t_flash0 = time.perf_counter()
        if flash:
            self._set_light(rt, True)
            self._await_light(rt, True)          # 等 live 生效（≤1 扫描周期）
        # 曝光
        time.sleep(max(0.0, rt.model.exposure_ms / 1000.0))
        try:
            entry = self._capture_process(cid, source, t0, flash, template,
                                          req_id, ir_overlay=ir_overlay)
        finally:
            if flash:
                self._set_light(rt, False)
                rt.last_flash_ms = (time.perf_counter() - t_flash0) * 1000.0
        return entry

    def _modal_exposure_vib(self, cam):
        """模态曝光积分（2026-09-08 迭代 22）：曝光窗内标记模块相对相机
        安装模块的横向模态位移 → (σ_mm, 主导方向 rad)；模态未挂载/未
        启用/无相关模块 → (0.0, 0.0)（调用方按原链路，零影响）。"""
        m = getattr(self.engine, "modal", None)
        if m is None or not m.enabled:
            return 0.0, 0.0
        mark_mod = _MARK_MODULE.get(cam.mark_source)
        if mark_mod is None:
            return 0.0, 0.0
        return m.exposure_vib(mark_mod, cam.mount_module)

    def _capture_process(self, cid, source, t0, flash, template=None,
                         req_id=None, ir_overlay=False):
        """拍帧 + 处理 + 发布（FAST 解析 / REAL 全链路）。返回结果记录。
        ir_overlay=True（迭代 15）：下相机红外套刻通道——被 die 覆盖的
        site quad 标记恢复可见、die 角标 z 按晶圆面计（详见
        vision_model.collect_marks）。"""
        rt = self.cams[cid]
        cam = rt.model
        # 帧时刻机器状态（轴位+误差）——渲染/解析共用同一份快照
        positions, poses = vm._engine_poses(self.engine)
        cam_pos, _yaw = vm.camera_pose(self.engine, cam, poses, positions)
        marks = vm.collect_marks(self.engine, cam, positions, poses,
                                 ir_overlay=ir_overlay)
        vm.apply_focus(cam, cam_pos[2], marks)
        layers = vm.collect_layers(self.engine, cam, positions, poses, cam_pos,
                                   ir_overlay=ir_overlay)
        surfaces = vm.surfaces_public(cam, layers, cam_pos)
        light_on = bool(self.points.get(cam.light_point, False))
        tgt = min(marks, key=lambda m: abs(m.x - cam_pos[0])
                  + abs(m.y - cam_pos[1])) if marks else None
        with self._lock:
            self._seq += 1
            seq = self._seq
        timing = {"render_ms": 0.0, "tcp_ms": 0.0, "match_ms": 0.0,
                  "total_ms": 0.0}
        if self.mode == "fast":
            t_r = time.perf_counter()
            # FAST 质心噪声 σ 属传感器仿真参数（recipe sim_sensors 组，测校
            # 页改，热生效——每次拍照实时取；2026-09-06 迭代 21 由
            # vision_model.FAST_NOISE_PX 常量迁入，缺省 0.15）
            recipe = self.engine.recipe.get() if self.engine.recipe else None
            noise_px = 0.15
            if recipe is not None:
                # 按相机 id 取逐实例 σ（2026-09-08 迭代 22 拆分；旧标量
                # 兼容回退——加载侧 _migrate 已保证新文件恒为字典）
                fnp = (recipe.get("sim_sensors") or {}) \
                    .get("fast_noise_px", 0.15)
                noise_px = float(fnp.get(cid, 0.15)) \
                    if isinstance(fnp, dict) else float(fnp)
            # 曝光积分（迭代 22）：模态 ON 时曝光窗内标记-相机相对振动
            # 等效 σ 按平方和并入 FAST 质心噪声（σ_eff=√(σ²+(vib_pp/
            # (2√2·k_px))²)，vib_pp=2√2·σ_vib）；OFF/无影响时零变化
            vib_sig_mm, _vib_ang = self._modal_exposure_vib(cam)
            if vib_sig_mm > 0.0 and cam.k_mm > 0.0:
                noise_px = math.hypot(noise_px, vib_sig_mm / cam.k_mm)
            result, _entries = vm.analytic_result(cam, marks, cam_pos,
                                                  light_on, noise_px=noise_px)
            timing["render_ms"] = (time.perf_counter() - t_r) * 1000.0
            err = None
        else:
            result, err = self._real_pipeline(cid, seq, positions, poses,
                                              light_on, timing, template,
                                              marks, cam_pos, layers)
        # strShapeName 契约（2026-09-05 迭代 16 修订）：回显本次触发模板名
        # （真实视觉软件按模板名启动/回查识别）；多模板（红外套刻双模板
        # 列表）"+"连接；无 template 的旧式触发保留 FAST/REAL 各自旧回退
        # （场景标记 shape / 首个模板 shape）。
        if template:
            tpl_names = template if isinstance(template, (list, tuple)) \
                else [template]
            result["strShapeName"] = "+".join(str(t) for t in tpl_names)
        timing["total_ms"] = (time.perf_counter() - t0) * 1000.0
        entry = {"seq": seq, "t": round(time.time(), 3),
                 "sim_t": round(self.engine.sim_time, 3),
                 "cam": cid, "source": source, "mode": self.mode,
                 "light_on": light_on, "flash": flash,
                 "focus_err_um": round(tgt.focus_err_um, 2) if tgt else None,
                 "blur_ratio": round(tgt.blur_ratio, 3) if tgt else None,
                 "focus_state": _focus_state(marks, light_on,
                                             tgt.blur_ratio if tgt else None),
                 "surfaces": surfaces,
                 "n_marks": len(marks), "timing": timing, "result": result}
        if template:
            entry["template"] = template   # 本次识别依据的模板名（迭代 12；
                                           # 迭代 15 起可为双模板列表）
        if ir_overlay:
            entry["ir_overlay"] = True     # 红外套刻通道（迭代 15）
        if err:
            entry["error"] = err
        self._publish(cid, entry, req_id)
        rt.last_result = entry
        return entry

    def _real_pipeline(self, cid, seq, positions, poses, light_on, timing,
                       template=None, marks=None, cam_pos=None, layers=None):
        """REAL：渲染→PNG→TCP→9200→接收侧模板匹配→等结果。cv2 缺失报清晰
        错误（返回 result=judge false + error 说明）。"""
        rt = self.cams[cid]
        if vm.cv2_or_none() is None:
            return vm.build_result("", []), (
                "cv2 未安装，REAL 模式不可用（pip install opencv-python）；"
                "FAST 模式不受影响")
        try:
            t_r = time.perf_counter()
            # 迭代 28：渲染噪声按本次触发模板的响应度换算（σ_gray=
            # fast_noise_px/resp——噪声量级跟着实际用于识别的模板走）；
            # 无模板触发（连续出图/live）由 render_frame 回退相机默认模板
            resp = None
            if template:
                tn0 = template[0] if isinstance(template, (list, tuple)) \
                    else template
                resp = self.template_noise_resp(cid, name=tn0)
            img = vm.render_frame(self.engine, rt.model, light_on=light_on,
                                  marks=marks, cam_pos=cam_pos, layers=layers,
                                  positions=positions, poses=poses,
                                  noise_resp=resp)
            # 曝光积分（2026-09-08 迭代 22）：模态 ON 时按曝光窗振动峰峰值
            # 叠运动模糊核（核长=vib_pp_px 取整限幅，方向=主导模态投影
            # 方向）；OFF/无影响时零变化
            vib_sig_mm, vib_ang = self._modal_exposure_vib(rt.model)
            if vib_sig_mm > 0.0:
                img = _motion_blur(img, vib_sig_mm, vib_ang,
                                   rt.model.k_mm)
            # 迭代 17：内部链路走 raw 灰度字节（enc="raw"），免 PNG 编解码
            # （9MP 下定制 PNG 编码 ~210ms + cv2 解码 ~55ms 全省）
            payload = img.tobytes()
            timing["render_ms"] = (time.perf_counter() - t_r) * 1000.0
            dbg = os.environ.get("VISION_DEBUG_SAVE_FRAME")
            if dbg:                      # 调试钩子（默认关）：落最近一帧 PNG
                try:
                    with open(dbg, "wb") as f:
                        f.write(vm.png_encode_gray(img))
                except OSError:
                    pass
            pend = {"event": threading.Event(), "template": template,
                    "send_ts": time.perf_counter()}
            with self._pending_lock:
                self._pending[seq] = pend
            try:
                self._push_frame(cid, payload, match=True, seq=seq,
                                 positions=positions, enc="raw",
                                 shape=(img.shape[1], img.shape[0]))
                if not pend["event"].wait(timeout=REAL_FRAME_TIMEOUT_S):
                    raise VisionError("9200 帧处理超时（%gs）"
                                      % REAL_FRAME_TIMEOUT_S)
            finally:
                with self._pending_lock:
                    self._pending.pop(seq, None)
            timing["tcp_ms"] = (pend.get("recv_ts", time.perf_counter())
                                - pend["send_ts"]) * 1000.0
            timing["match_ms"] = pend.get("match_ms", 0.0)
            if pend.get("err"):
                return vm.build_result("", []), pend["err"]
            # strShapeName 由 _capture_process 统一按触发模板名回写
            # （迭代 16）；此处仅保留无 template 旧式触发的回退（首个模板
            # shape/名）。
            tpls = self.templates.for_cam(cid)
            shape_name = (tpls[0].get("shape") or tpls[0]["name"]) \
                if tpls else ""
            # 迭代 39 真值/认知分离：相机物理安装旋转 view_rot 已在投影链
            # （渲染/FAST 同一 project_rel R(−θ)，真值）施加，匹配峰位/角度
            # 即原始传感器系——不再做任何剥旋（旧 _unrotate_entries 已删，
            # FAST 与 REAL 两模式结果同坐标语义）
            return vm.build_result(shape_name, pend["entries"]), None
        except VisionError as e:
            return vm.build_result("", []), str(e)

    def _push_frame(self, cid, payload, match, seq=None, positions=None,
                    enc="png", shape=None):
        """相机侧：真 socket 推帧到视觉服务 9200（懒连接、断线重连一次）。
        enc="png"（缺省，旧兼容）：payload=PNG 字节，w/h 从 IHDR 读；
        enc="raw"（迭代 17 内部链路）：payload=灰度字节，shape=(w,h) 必填。"""
        rt = self.cams[cid]
        rt.frame_seq += 1
        if enc == "raw":
            w, h = shape
        else:
            w, h = vm.png_size(payload)
        header = {"cam": cid, "w": w, "h": h, "enc": enc,
                  "t": time.time(), "sim_t": round(self.engine.sim_time, 3),
                  "seq": seq or 0, "match": bool(match),
                  "frame_seq": rt.frame_seq,
                  "axes": {n: round(p, 4) for n, p
                           in (positions or {}).items()}}
        data = encode_frame(header, payload)
        with rt.sock_lock:
            for attempt in (0, 1):
                try:
                    if rt.sock is None:
                        rt.sock = socket.create_connection(
                            (self.tcp_host, self.tcp_port), timeout=5)
                        # 9MP raw 帧（9MB/帧）loopback 传输：放大收发缓冲
                        # 减少内核拷贝轮次（迭代 17，tcp_ms ~100ms→~30ms）
                        rt.sock.setsockopt(socket.SOL_SOCKET,
                                           socket.SO_SNDBUF, 4 << 20)
                        rt.sock.setsockopt(socket.IPPROTO_TCP,
                                           socket.TCP_NODELAY, 1)
                    rt.sock.sendall(data)
                    return
                except OSError:
                    try:
                        if rt.sock:
                            rt.sock.close()
                    except OSError:
                        pass
                    rt.sock = None
                    if attempt == 1:
                        raise VisionError("9200 推帧失败（连接发送均失败）")

    def _publish(self, cid, entry, req_id=None):
        """结果出口（唯一）：MQTT 发布 {req_id, cam, result:<七字段>}
        到 vision/result/{cam}（cam 2026-09-10 迭代 31 起随载荷回填，
        消费方可 topic+cam+req_id 三键认领；非 MQTT 触发的结果
        req_id=null）；完整记录（含耗时分解）入环形缓冲。"""
        with self._lock:
            self.results.append(entry)
        payload = {"req_id": req_id, "cam": cid, "result": entry["result"]}
        if entry.get("template"):
            payload["template"] = entry["template"]
        if entry.get("error"):
            payload["error"] = entry["error"]
        try:
            mqtt_publish(RESULT_TOPIC % cid,
                         json.dumps(payload, ensure_ascii=False),
                         self.mqtt_host, self.mqtt_port)
        except Exception as e:
            entry["mqtt_error"] = str(e)

    def _record_error(self, cid, source, err, t0):
        # 拍帧异常：对焦三字段 null（无法判定；明细在 error）
        entry = {"seq": -1, "t": round(time.time(), 3),
                 "sim_t": round(self.engine.sim_time, 3),
                 "cam": cid, "source": source, "mode": self.mode,
                 "light_on": False, "flash": False, "focus_err_um": None,
                 "blur_ratio": None, "focus_state": None, "n_marks": 0,
                 "timing": {"render_ms": 0.0, "tcp_ms": 0.0, "match_ms": 0.0,
                            "total_ms": (time.perf_counter() - t0) * 1000.0},
                 "result": vm.build_result("", []), "error": err}
        with self._lock:
            self.results.append(entry)

    # ================= API 方法（ws_gateway 同进程直调） =================
    def clear_results(self):
        """识别结果环形缓冲清空（2026-09-04 迭代 13，/api/full_reset）。"""
        with self._lock:
            self.results.clear()

    def cameras_public(self):
        """三相机配置 + 运行态。"""
        out = {}
        for cid, rt in self.cams.items():
            cam = rt.model
            # 当前对焦状态（按当前机器状态现算；focus_state 枚举见模块头）
            focus_err_um, in_dof = None, False
            focus_state = "no_mark"
            surfaces = []
            try:
                positions, poses = vm._engine_poses(self.engine)
                cam_pos, _ = vm.camera_pose(self.engine, cam, poses, positions)
                marks = vm.collect_marks(self.engine, cam, positions, poses)
                vm.apply_focus(cam, cam_pos[2], marks)
                layers = vm.collect_layers(self.engine, cam, positions,
                                           poses, cam_pos)
                surfaces = vm.surfaces_public(cam, layers, cam_pos)
                tgt = None
                if marks:
                    tgt = min(marks, key=lambda m: abs(m.x - cam_pos[0])
                              + abs(m.y - cam_pos[1]))
                    focus_err_um = round(tgt.focus_err_um, 2)
                    in_dof = tgt.blur_ratio <= 1.0
                focus_state = _focus_state(marks, rt.light_on,
                                           tgt.blur_ratio if tgt else None)
            except Exception:
                pass
            last = rt.last_result
            out[cid] = {
                "label": cam.label,
                "mount": {"module": cam.mount_module,
                          "offset": cam.mount_offset},
                "look_dir": cam.look_dir, "wd_mm": cam.wd_mm,
                "dof_um": cam.dof_um, "px": [cam.px_x, cam.px_y],
                "pixel_equiv_um": round(cam.k_mm * 1000.0, 4),
                "fov_mm": [round(v, 3) for v in cam.fov_mm],
                "dir_x": cam.dir_x, "dir_y": cam.dir_y,
                "swap_xy": cam.swap_xy, "rot_mrad": cam.rot * 1000.0,  # 控制器认知（反算链）
                "light_point": cam.light_point,
                "trigger_source": cam.trigger_source,
                "mark_source": cam.mark_source,
                "mode": cam.mode, "exposure_ms": cam.exposure_ms,
                "gain": cam.gain, "score_threshold": cam.score_threshold,
                "view_rot_deg": cam.view_rot_deg,  # 相机物理安装旋转角·真值（投影链，°）
                "light_on": rt.light_on,
                "focus_err_um": focus_err_um, "in_dof": in_dof,
                "focus_state": focus_state,
                "surfaces": surfaces,
                "frame_seq": rt.frame_seq,
                "last_flash_ms": (round(rt.last_flash_ms, 1)
                                  if rt.last_flash_ms else None),
                "last_result": ({"seq": last["seq"], "t": last["t"],
                                 "judge": last["result"]["judge"],
                                 "n": len(last["result"]["lstCol"]),
                                 "score_max": (max(last["result"]["lstScore"])
                                               if last["result"]["lstScore"]
                                               else None),
                                 "timing": last["timing"]}
                                if last else None),
            }
        return {"ok": True, "mode": self.mode, "cameras": out,
                "templates": len(self.templates.list()),
                "results_buffered": len(self.results),
                "cv2": vm.cv2_or_none() is not None}

    def set_mode(self, mode):
        if mode not in ("fast", "real"):
            raise VisionError("mode 只能是 fast|real")
        if mode == "real" and vm.cv2_or_none() is None:
            raise VisionError(
                "cv2 未安装，REAL 模式不可用（pip install opencv-python）；"
                "FAST 模式不受影响")
        with self._lock:
            self.mode = mode
        return {"ok": True, "mode": self.mode}

    def frame_png(self, cam_id, width=640, height=None, ir_overlay=False):
        """live 预览图：两种模式都走真实渲染器（降采样）。
        高度缺省按传感器画幅比例推导（2026-09-08 修复：原默认 640×480 把
        方形传感器（px_u=px_v）的方形 FOV 横向压扁 4:3——面板 live 画面
        比例失真的根源；现 640 宽 → 方形传感器出 640×640）。
        ir_overlay=True（2026-09-07 迭代 20，仅下相机有意义）：红外套刻
        通道——被 die 覆盖的 site quad 恢复可见、die 角标按晶圆面焦
        （见 vision_model.collect_marks），面板"红外套刻帧"查看消费。"""
        rt = self._cam(cam_id)
        if height is None:
            height = max(1, int(round(int(width)
                                      * rt.model.px_y
                                      / float(rt.model.px_x))))
        light_on = bool(self.points.get(rt.model.light_point, False))
        if not ir_overlay:
            img = vm.render_frame(self.engine, rt.model, width, height,
                                  light_on=light_on)
        else:
            positions, poses = vm._engine_poses(self.engine)
            cam_pos, _yaw = vm.camera_pose(self.engine, rt.model, poses,
                                           positions)
            marks = vm.collect_marks(self.engine, rt.model, positions,
                                     poses, ir_overlay=True)
            vm.apply_focus(rt.model, cam_pos[2], marks)
            layers = vm.collect_layers(self.engine, rt.model, positions,
                                       poses, cam_pos, ir_overlay=True)
            img = vm.render_frame(self.engine, rt.model, width, height,
                                  light_on=light_on, marks=marks,
                                  cam_pos=cam_pos, layers=layers,
                                  positions=positions, poses=poses)
        return vm.png_encode_gray(img)

    def acquire(self, cam_id, source="api", template=None, req_id=None,
                ir_overlay=False):
        """软触发：trigger 模式走硬触发同路径（闪光时序 + 处理发布），同步等
        结果。continuous + 光源灭时不替你闪光（真机语义：常开模式下灯灭是
        工况异常），直接拍暗帧走判废路径。"""
        rt = self._cam(cam_id)
        if rt.model.mode == "continuous":
            return self._capture_process(cam_id, source,
                                         time.perf_counter(), False,
                                         template, req_id,
                                         ir_overlay=ir_overlay)
        return self._do_trigger(cam_id, source, time.perf_counter(),
                                template, req_id, ir_overlay=ir_overlay)

    def identify(self, cam_id, template=None, req_id=None, ir_overlay=False):
        """立即识别：同 acquire，但可指定模板；返回结果记录（含耗时分解）。
        REAL 无模板时报清晰错误。req_id 仅 MQTT 触发路径用（原样回填到
        vision/result/{cam} 的发布载荷，见 _publish）。
        template 可为模板名列表（迭代 15：post-inspect 一张图双模板套刻，
        逐个做存在性校验）；ir_overlay=True 走红外套刻通道（仅下相机，
        见 vision_model.collect_marks）。"""
        rt = self._cam(cam_id)
        # 两种模式都按模板名识别（2026-09-04 迭代 12：每次拍照必须依据
        # 模板名；模板未建立=不能识别，模拟真机未建模板行为）
        if template:
            tpl_names = template if isinstance(template, (list, tuple)) \
                else [template]
            for tn in tpl_names:
                if self.templates.get(tn) is None:
                    raise VisionError(
                        "模板未建立: %r（vision_templates.json 无此模板，先 "
                        "POST /api/vision/templates 建模板）" % tn)
        if self.mode == "real":
            if vm.cv2_or_none() is None:
                raise VisionError(
                    "cv2 未安装，REAL 模式不可用；FAST 模式不受影响")
            if not template and not self.templates.for_cam(cam_id):
                raise VisionError(
                    "相机 %s 无模板：先 POST /api/vision/templates 建模板"
                    % cam_id)
        entry = self.acquire(cam_id, source="identify", template=template,
                             req_id=req_id, ir_overlay=ir_overlay)
        if entry.get("error"):
            err = VisionError(entry["error"])
            err.published = True      # 结果（含 error）已经 _publish 出口发出
            raise err
        return entry

    def results_public(self, limit=20):
        with self._lock:
            return {"ok": True, "results": list(self.results)[-int(limit):]}

    def solve_pixel_public(self, cam_id, col, row):
        """像素坐标→世界坐标反算（GET /api/vision/solve_pixel，迭代 48；
        测校 GUI 动态 CSV 富化用）：与 metrology._context 同链——
        vm._engine_poses+camera_pose（含 6DOF 误差链/运动台实际位姿）+
        cam.solve_pixel（含 dir/swap/rot_mrad 控制器认知反算链）。
        cam 非法 KeyError（网关转 400）；col/row 数值由网关校验。
        返回 {ok, cam, col, row, cam_world_mm, mark_world_mm}。"""
        cam = self.cams[cam_id].model
        positions, poses = vm._engine_poses(self.engine)
        cam_pos, _yaw = vm.camera_pose(self.engine, cam, poses, positions)
        wx, wy, wz = cam.solve_pixel(float(col), float(row), cam_pos)
        return {"ok": True, "cam": cam_id,
                "col": float(col), "row": float(row),
                "cam_world_mm": [round(v, 6) for v in cam_pos],
                "mark_world_mm": [round(wx, 6), round(wy, 6),
                                  round(wz, 6)]}

    def guide(self, cam_id):
        """标记引导（GET /api/vision/guide?cam=X）：当前视野内/外全部标记的
        世界坐标、投影像素、相对视野中心的世界偏差 delta_mm、in_fov、
        focus_err_um，以及把标记移进视野中心所需的轴增量 suggested
        （世界系反算：down/coarse 动标记载体（WS_LS/FA_X，轴+→标记世界+）→
        delta=−(mark−cam)；up 动相机（AS，轴+→相机世界+）→ delta=+(mark−cam)。
        与 calibration.JOG_AXES 同符号约定）。按 |delta| 升序（最近在前）。"""
        from calibration import JOG_AXES     # 懒导入，避免模块环
        rt = self._cam(cam_id)
        cam = rt.model
        positions, poses = vm._engine_poses(self.engine)
        cam_pos, _ = vm.camera_pose(self.engine, cam, poses, positions)
        marks = vm.collect_marks(self.engine, cam, positions, poses)
        vm.apply_focus(cam, cam_pos[2], marks)
        jogs = JOG_AXES.get(cam_id, ())
        out = []
        for m in marks:
            dx = m.x - cam_pos[0]
            dy = m.y - cam_pos[1]
            col, row = cam.project(dx, dy)
            suggested = []
            for axis, sign in jogs:
                d = dx if axis.endswith("_X") else dy
                suggested.append({"axis": axis,
                                  "delta_mm": round(-sign * d, 4)})
            out.append({"name": m.name,
                        "world": [round(m.x, 4), round(m.y, 4), round(m.z, 4)],
                        "fov_center": [round(col, 2), round(row, 2)],
                        "delta_mm": [round(dx, 4), round(dy, 4)],
                        "in_fov": bool(cam.in_fov(col, row)),
                        "focus_err_um": round(m.focus_err_um, 2),
                        "suggested": suggested})
        out.sort(key=lambda e: math.hypot(e["delta_mm"][0], e["delta_mm"][1]))
        return {"ok": True, "cam": cam_id,
                "fov_center_px": [cam.px_x / 2.0, cam.px_y / 2.0],
                "cam_world": [round(v, 4) for v in cam_pos],
                "marks": out}

    # ---- 采图落盘 ----
    def capture_save(self, cam_id, ir_overlay=False):
        """POST /api/vision/capture/save：按当前机器状态真实渲染全幅灰度图，
        PNG 落盘 vision_captures/ 并登记索引（帧时刻轴位快照/对焦状态/
        灯态）。灯态语义与 identify/acquire 一致：trigger 模式且光源灭时
        自动闪光（点亮→等曝光→拍帧→灭，复用 _do_trigger 时序）；
        continuous 模式灯态按当前 IO（灯灭出暗图，真机语义）。
        ir_overlay=True（2026-09-07 迭代 20，仅下相机）：红外套刻通道
        采图（被 die 覆盖的 site quad 恢复可见、die 角标按晶圆面焦），
        元数据带 ir_overlay 标记。返回采图元数据。"""
        rt = self._cam(cam_id)
        cam = rt.model
        flash = False
        if cam.mode == "trigger" and not rt.light_on:
            flash = True
            self._set_light(rt, True)
            self._await_light(rt, True)      # 等 live 生效（≤1 扫描周期）
            time.sleep(max(0.0, cam.exposure_ms / 1000.0))
        try:
            return self._capture_save_frame(cam_id, flash,
                                            ir_overlay=ir_overlay)
        finally:
            if flash:
                self._set_light(rt, False)

    def _capture_save_frame(self, cam_id, flash, ir_overlay=False):
        """capture_save 的拍帧落盘主体（灯态已就序）。"""
        rt = self._cam(cam_id)
        cam = rt.model
        # 帧时刻机器状态（轴位+误差）——渲染与元数据共用同一份快照
        positions, poses = vm._engine_poses(self.engine)
        cam_pos, _yaw = vm.camera_pose(self.engine, cam, poses, positions)
        marks = vm.collect_marks(self.engine, cam, positions, poses,
                                 ir_overlay=ir_overlay)
        vm.apply_focus(cam, cam_pos[2], marks)
        layers = vm.collect_layers(self.engine, cam, positions, poses,
                                   cam_pos, ir_overlay=ir_overlay)
        light_on = bool(self.points.get(cam.light_point, False))
        img = vm.render_frame(self.engine, cam, light_on=light_on,
                              marks=marks, cam_pos=cam_pos, layers=layers,
                              positions=positions, poses=poses)
        tgt = min(marks, key=lambda m: abs(m.x - cam_pos[0])
                  + abs(m.y - cam_pos[1])) if marks else None
        now = time.time()
        name = "cap_%s_%s.%03d" % (cam_id, time.strftime(
            "%Y%m%d_%H%M%S", time.localtime(now)),
            int((now % 1) * 1000))
        base = name
        suffix = 0
        while self.captures.get(name) is not None:    # 同毫秒连拍防重名
            suffix += 1
            name = "%s_%d" % (base, suffix)
        os.makedirs(CAPTURES_DIR, exist_ok=True)
        fname = name + ".png"
        with open(os.path.join(CAPTURES_DIR, fname), "wb") as f:
            f.write(vm.png_encode_gray(img))
        meta = {"name": name, "cam": cam_id, "file": fname,
                "ts": round(now, 3),
                "sim_t": round(self.engine.sim_time, 3),
                "axes": {n: round(p, 4) for n, p in positions.items()},
                "focus_state": _focus_state(marks, light_on,
                                            tgt.blur_ratio if tgt else None),
                "light_on": light_on, "flash": flash}
        if ir_overlay:
            meta["ir_overlay"] = True      # 红外套刻通道采图（迭代 20）
        self.captures.put(name, meta)
        return {"ok": True, "capture": meta}

    def captures_public(self, cam=None, limit=50):
        """GET /api/vision/captures?cam=&limit=：按 ts 倒序（最新在前）。"""
        lst = [m for m in self.captures.list().values()
               if not cam or m["cam"] == cam]
        lst.sort(key=lambda m: m["ts"], reverse=True)
        return {"ok": True, "captures": lst[:max(1, int(limit))]}

    def capture_png(self, name):
        """GET /api/vision/capture/file?name=：取采图 PNG 字节。"""
        _meta, png = self.captures.read_png(name)
        return png

    def capture_delete(self, name):
        """DELETE /api/vision/captures {name}：删索引 + PNG 文件。"""
        self.captures.remove(name)
        return {"ok": True, "deleted": name}

    # ---- 模板 CRUD ----
    def templates_public(self):
        return {"ok": True, "templates": list(self.templates.list().values())}

    # ---- 迭代 28：模板噪声响应度（REAL 渲染噪声换算依据） ----
    def template_noise_resp(self, cam_id, name=None):
        """REAL 渲染噪声换算参考 → (resp px/DN, cap DN, scene_gain) 或
        None：name 指定=该模板（须属本相机）；否则机械常数 vision_marks
        引用的本相机模板优先，再次任一本相机模板。meta 缺
        noise_resp_px_per_dn / tpl_std_dn（迭代 28 前的旧模板）时即时
        标定并回写索引（一次性，向后兼容）。全部不可用 → None
        （render_frame 回退 K_NOISE 旧公式）。render 热路径调用：已标定
        后纯 dict 查表，零开销。scene_gain（2026-09-08 结果倒推）：建模板
        时实测的平移传递增益（缺失 None，渲染侧经验缺省）。"""
        cands = []
        if name:
            meta = self.templates.get(name)
            if meta is not None and meta.get("cam") == cam_id:
                cands.append(meta)
        if not cands:
            try:
                vmarks = (self.constants.get() or {}).get("vision_marks") or {}
            except Exception:
                vmarks = {}
            for tname in vmarks.values():
                if not isinstance(tname, str) or not tname:
                    continue
                meta = self.templates.get(tname)
                if meta is not None and meta.get("cam") == cam_id:
                    cands.append(meta)
            for meta in self.templates.for_cam(cam_id):
                if all(meta["name"] != c["name"] for c in cands):
                    cands.append(meta)
        for meta in cands:
            if meta.get("noise_resp_px_per_dn") is None \
                    or meta.get("tpl_std_dn") is None:
                self._backfill_noise_resp(meta)
                meta = self.templates.get(meta["name"]) or meta
            try:
                resp = float(meta.get("noise_resp_px_per_dn"))
            except (TypeError, ValueError):
                continue
            if resp <= 1e-9:
                continue
            # 噪声上限：live 探针的经验值 noise_sigma_max_dn（匹配健康
            # 实测）优先；缺省（画布探针兜底路径）用 SNR 启发式
            # NOISE_SNR_CAP×模板 RMS；再与绝对上限取小
            cap = meta.get("noise_sigma_max_dn")
            if cap is None:
                std = meta.get("tpl_std_dn")
                cap = (vm.NOISE_SNR_CAP * float(std)
                       if std else vm.NOISE_SIGMA_CAP_DN)
            try:
                sg_col = meta.get("noise_scene_gain_col")
                sg_row = meta.get("noise_scene_gain_row")
                sg_col = [float(sg_col[0]), float(sg_col[1])] \
                    if sg_col else None
                sg_row = [float(sg_row[0]), float(sg_row[1])] \
                    if sg_row else None
            except (TypeError, ValueError, IndexError):
                sg_col = sg_row = None
            return (resp, min(vm.NOISE_SIGMA_CAP_DN, float(cap)),
                    (sg_col, sg_row))
        return None

    def _live_noise_probe(self, cam, meta, matcher):
        """建模板现场实测响应度（迭代 28，live 建模板路径）：当前场景连渲
        K 帧并强制注入已知 σ_test 灰度噪声（render_frame noise_sigma_dn
        覆盖），生产链 Matcher.match 测模板中心散布 → resp=σ_pos/σ_test。
        真实场景+真实匹配链——画布探针对长臂 cross 这类延伸出模板边界的
        标记系统性低估响应（实测画布 0.00012 vs 线上 0.002 px/DN，孔径效
        应：标记在画内延续使相关峰更平坦），现场实测值即线上值。
        经验噪声上限 noise_sigma_max_dn：探测级 10→3→1DN 降档，只采纳
        |峰位偏差|≤1.5px 且 score≥0.65 的试次（错锁伪峰剔除——低对比模板
        在 10DN 下匹配已崩，不剔会测出 resp≈12 px/DN 的垃圾；粗网格一格
        3.4px，跳格即超门）；某级有效试次≥半且均分≥0.90 才接受该级为上限
        （均分余量不足=动态作业尾巴会跌破阈值，实测 up 内禀 0.76 的模板
        8DN 即零峰）；10DN 级且均分≥0.95（对比度富余）放宽到绝对上限
        NOISE_SIGMA_CAP_DN（实测高对比 down 模板 40DN 下 score≈0.94 仍
        稳健）——大模板（粗网格终值路径）在 10DN 下抖动 ≈ 亚像素伪影
        地板（resp 逐次飘 10×），均分≥0.95 时提级 40DN 重测取散布。
        三级都不健康返回 None（回退画布探针，其上限取保守 2DN
        ≈旧默认噪声量级——采图/补标定路径无线上识别证据，宁保守勿杀
        识别）。散布用 MAD 估计（1.4826×中位绝对偏差，抗亚像素偶发跳变）。
        一次性开销 ~K×(渲染+匹配)/级（9MP 下 ~4s/级），只在建模板时发生。"""
        n = vm.np()
        if not bool(self.points.get(cam.light_point, False)):

            return None                 # 光源灭：暗图模板，标定无意义
        tpl = matcher.load_template_img(meta)
        if tpl is None or tpl.shape[0] < 16 or tpl.shape[1] < 16:

            return None
        roi = meta.get("roi")
        if not roi:

            return None
        ex = roi[0] + roi[2] / 2.0      # 模板中心在全幅图像坐标（期望峰位）
        ey = roi[1] + roi[3] / 2.0
        # 迭代 39：view_rot 已进投影链（渲染与模板示教同一真值旋转），
        # 期望峰位=模板 ROI 中心原样——旧出图 warp 的峰位正变换随
        # warp 一并删除，探针与渲染天然一致
        thr = float(meta.get("threshold", cam.score_threshold))
        f = tpl.astype(n.float32)
        gy, gx = n.gradient(f)
        tpl_std = float(n.std(f))
        def _trial_level(sig_t, deadline=None):
            """在 σ_t DN 下连渲 K 帧过生产链，返回 (有效试次数, 均分,
            MAD 散布 px)。deadline（2026-09-08）：时间预算（perf_counter
            秒），超时且已攒 ≥3 个有效试次即提前收——REAL 渲染在物理
            线程 GIL 竞争下 ~0.25s/帧，多级降档全跑满会让建模板总时长
            超 API 客户端 5s 超时（实测 8.2s）；MAD 统计对 3~8 样本仍
            稳健，标定语义不变。"""
            dxs, dys, scores = [], [], []
            for _ in range(NOISE_PROBE_TRIALS):
                if deadline is not None and time.perf_counter() > deadline:
                    break
                img = vm.render_frame(self.engine, cam, light_on=True,
                                      noise_sigma_dn=sig_t)
                entries, _ms = matcher.match(img, [meta], thr)
                if entries:
                    e = max(entries, key=lambda e_: e_["score"])
                    dx, dy = e["col"] - ex, e["row"] - ey
                    # 错锁伪峰剔除（|d|>1.5px 不可能是噪声抖动——粗网格
                    # 一格 3.4px，跳格即超门）+ 低分剔除
                    if abs(dx) <= 1.5 and abs(dy) <= 1.5 \
                            and e["score"] >= 0.65:
                        dxs.append(dx)
                        dys.append(dy)
                        scores.append(e["score"])
                time.sleep(0)          # 让出 GIL：别压制 1kHz 物理线程
            n_ok = len(dxs)
            ms = sum(scores) / n_ok if n_ok else 0.0
            # MAD：粗网格亚像素偶发 ~1px 跳变会把普通 std 带飞（resp 实测
            # 曾因此飘 5×）
            return n_ok, ms, _robust_sigma(dxs + dys)

        big = tpl.shape[0] * tpl.shape[1] > Matcher.REFINE_POS_MAX_TPL_PX
        # 建模板现场标定总预算（2026-09-08 收紧重排）：白噪声两级 + 平移
        # 增益标定 3 帧，从当下起 ~2.6s；超时用已有效试次收尾。
        # 标定用 σ_test 从 10DN 起、大模板提级 40DN——**标定信噪比优先**
        # （大模板在低 σ 下抖动≈粗网格亚像素伪影地板 ~0.05px，10DN 响应
        # ~0.075px 只有 1.5:1，resp 会被地板抬虚——40DN 响应 ~0.25px 分离
        # 度 5:1）；运行时注入封顶独立为 min(NOISE_SIGMA_CAP_DN=12,
        # SNR×tpl_std)（雪花视觉口径，与标定级解耦）
        _probe_deadline = time.perf_counter() + 2.6
        # 大模板直接从 40DN 起测（10DN 级响应 ~0.075px 与粗网格亚像素伪影
        # 地板 ~0.05px 只差 1.5:1，resp 会被地板抬虚 3×——实测 0.0069 vs
        # 真值 ~0.002；40DN 响应 ~0.25px 分离度 5:1）；小模板地板小，
        # 10DN 起测、不行升 40
        _levels = (40.0,) if big else (NOISE_PROBE_SIGMA_DN, 40.0)
        for sig_test in _levels:
            n_ok, mscore, spos = _trial_level(sig_test,
                                              deadline=_probe_deadline)
            thr_ms = 0.90 if sig_test < 20.0 else 0.75
            if n_ok < NOISE_PROBE_TRIALS // 2 or mscore < thr_ms:
                continue               # 该级匹配崩/均分余量不足，升档重测
            if spos <= 1e-9:
                return None
            out = {"noise_resp_px_per_dn": spos / sig_test,
                   "noise_resp_sigma_test_dn": sig_test,
                   "noise_resp_trials": n_ok,
                   "noise_resp_score": round(mscore, 4),
                   "noise_resp_via": "live",
                   "noise_sigma_max_dn": None,   # 运行封顶走启发式缺省分支
                   "tpl_std_dn": tpl_std,
                   "grad_gx": float((gx * gx).sum()),
                   "grad_gy": float((gy * gy).sum())}
            # 平移传递增益不做逐模板标定（2026-09-08 两轮实测结论）：
            # ①增益随幅度非线性且方向各向异性（0.48~1.8），但 2 帧×2 点
            # 的 MAD 估计噪声 ±60% 比缺省更差；②resp 用 40DN 级标定准确
            # 后（分离粗网格伪影地板 5:1），白噪声贡献真实，缺省增益 1.0
            # 下实测输出=预算 ±16%（n=16 抽样内）——够用且省 ~1s 建模板
            # 时间；渲染侧 g(σ)=1 恒等，meta 兼容字段（noise_scene_gain_
            # col/row）保留读取但不写入
            return out
        return None

    def _backfill_noise_resp(self, meta):
        """旧模板补标定（一次性）：读模板 PNG 跑 noise_response_probe，
        结果回写模板索引（vision_templates.json 向后兼容加字段）。"""
        try:
            with self._lock:
                if self._matcher is None:
                    self._matcher = Matcher()
                matcher = self._matcher
            rt = self.cams.get(meta.get("cam"))
            fwh = (rt.model.px_x, rt.model.px_y) if rt is not None else None
            calib = noise_response_probe(matcher, meta, fwh)
            if not calib:
                return None
            meta = dict(meta)
            meta.update(calib)
            self.templates.put(meta["name"], meta)
            return float(calib["noise_resp_px_per_dn"])
        except Exception:
            return None

    def _calibrate_missing_resp(self):
        """启动期全量补标定（迭代 33）：遍历模板库，对所有缺
        noise_resp_px_per_dn / tpl_std_dn 的旧模板跑画布探针（复用
        noise_response_probe；秒级、一次性），成功的 templates.put 落盘。
        失败逐条警告跳过，不阻塞启动——否则首个 REAL 识别帧会在渲染
        热路径里惰性现算探针（每帧秒级税，down 相机曾因此每拍必付）。
        调用点：_warmup（REAL 模式预热末尾）与服务启动（非 REAL 模式）；
        template_create 成功路径仍走自己的 live/画布标定（见该方法）。"""
        try:
            with self._lock:
                if self._matcher is None:
                    self._matcher = Matcher()
                matcher = self._matcher
        except Exception as e:
            print("[vision] 补标定跳过（matcher 不可用）: %s" % e,
                  flush=True)
            return
        for meta in self.templates.list().values():
            if meta.get("noise_resp_px_per_dn") is not None \
                    and meta.get("tpl_std_dn") is not None:
                continue
            name = meta.get("name") or "?"
            try:
                rt = self.cams.get(meta.get("cam"))
                fwh = (rt.model.px_x, rt.model.px_y) \
                    if rt is not None else None
                calib = noise_response_probe(matcher, meta, fwh)
            except Exception as e:
                print("[vision] 模板 %s 噪声响应度补标定失败（跳过）: %s"
                      % (name, e), flush=True)
                continue
            if not calib:
                print("[vision] 模板 %s 噪声响应度补标定无结果（探针失败，"
                      "跳过）" % name, flush=True)
                continue
            new_meta = dict(meta)
            new_meta.update(calib)
            self.templates.put(name, new_meta)
            print("[vision] 模板 %s 噪声响应度补标定完成: resp=%.6g px/DN"
                  % (name, calib["noise_resp_px_per_dn"]), flush=True)

    def template_create(self, cam_id, name=None, roi=None, shape=None,
                        threshold=None, capture=None):
        """建模板：从指定相机当前帧取 ROI（默认画幅短边 1/4 取奇，1280×960
        下为 241×241）存 PNG + 元数据。
        capture=<采图名> 时改为从已存采图裁 ROI：相机随采图（请求 cam 与
        采图不一致报 400），roi 必填，且须 ≥16×16px 并含足够灰度方差
        （纯均匀区 NCC 退化，建模板时拦下指路）。"""
        source = "live"
        if capture is not None:
            cmeta, png = self.captures.read_png(capture)
            if cam_id and cam_id != cmeta["cam"]:
                raise VisionError("采图 %r 属相机 %s，与请求 cam=%r 不一致"
                                  % (capture, cmeta["cam"], cam_id))
            cam_id = cmeta["cam"]
            try:
                img = vm.png_decode_gray(png)
            except ValueError as e:
                raise VisionError("采图 %r 解码失败：%s" % (capture, e))
            light_on = bool(cmeta.get("light_on"))
            source = capture
        else:
            rt = self._cam(cam_id)
            light_on = bool(self.points.get(rt.model.light_point, False))
            # 示教帧零噪声（2026-09-08 修复）：模板必须是干净参考——创建时
            # 用相机默认模板的 resp 换算噪声（与未来识别时按本模板 resp 的
            # 噪声水平不一致，实测 up 低对比标记下 模板 std 18 vs 识别帧
            # std 9.7，NCC 塌到阈下 judge=false）。真机示教同样取好帧
            img = vm.render_frame(self.engine, rt.model, light_on=light_on,
                                  noise_sigma_dn=0.0)
        rt = self._cam(cam_id)
        cam = rt.model
        H, W = img.shape
        if roi is None:
            if capture is not None:
                raise VisionError(
                    "从采图建模板须显式给 roi=[x,y,w,h]（默认中央 ROI 不一定"
                    "框得住目标标记；可先 GET /api/vision/capture/file?name=%s"
                    " 看图再框）" % capture)
            # 默认 ROI = 画幅短边 3/4（取奇）：小 ROI 容易整块落进大标记
            # 内部均匀区（NCC 零方差退化）——0.6µm/px 下 2mm cross 臂宽
            # 833px，1/4 短边（751）全落臂内，3/4 稳含臂缘/背景过渡
            n = ((min(W, H) * 3) // 4) | 1
            x0, y0 = (W - n) // 2, (H - n) // 2
            roi = [x0, y0, n, n]
        if not (isinstance(roi, (list, tuple)) and len(roi) == 4):
            raise VisionError("roi 须为 [x,y,w,h]")
        x, y, w, h = [int(v) for v in roi]
        if not (0 <= x < W and 0 <= y < H and 0 < w <= W - x
                and 0 < h <= H - y):
            raise VisionError("roi 越界: %r（画幅 %dx%d）" % (list(roi), W, H))
        crop = img[y:y + h, x:x + w]
        if capture is not None:
            if w < TEMPLATE_ROI_MIN or h < TEMPLATE_ROI_MIN:
                raise VisionError(
                    "roi 至少 %d×%dpx（得到 %d×%d），否则 NCC 退化"
                    % (TEMPLATE_ROI_MIN, TEMPLATE_ROI_MIN, w, h))
            var = float(vm.np().var(crop.astype(vm.np().float32)))
            if var < TEMPLATE_ROI_MIN_VAR:
                raise VisionError(
                    "roi 区域过于均匀（灰度方差 %.2f < %.1f，NCC 响应面平坦"
                    "无意义）：请框到包含标记边缘/特征的区域（可先 GET "
                    "/api/vision/capture/file?name=%s 看图）"
                    % (var, TEMPLATE_ROI_MIN_VAR, capture))
        if name is None:
            name = "%s_tpl_%d" % (cam_id, int(time.time()) % 100000)
        if self.templates.get(name) is not None:
            raise VisionError("模板名已存在: %r" % name)
        if threshold is not None and not (0.0 < float(threshold) <= 1.0):
            raise VisionError("threshold 须在 (0,1]")
        os.makedirs(TEMPLATES_DIR, exist_ok=True)
        fname = "tpl_%s.png" % name
        with open(os.path.join(TEMPLATES_DIR, fname), "wb") as f:
            f.write(vm.png_encode_gray(crop))
        meta = {"name": name, "cam": cam_id, "file": fname,
                "shape": shape or "", "roi": [x, y, w, h], "w": w, "h": h,
                "threshold": float(threshold) if threshold is not None
                else cam.score_threshold,
                "created": round(time.time(), 3),
                "source": source,
                "light_on_when_created": light_on}
        # 迭代 28：模板噪声响应度自标定（resp=σ_pos/σ_DN，render_frame 按
        # 它把 fast_noise_px 换算成灰度噪声 σ）+ 梯度能量 G 存 meta（模板
        # 列表 API 随之带出；旧模板缺字段由 template_noise_resp 惰性补标定
        # 回写）。live 建模板走现场实测（连渲 K 帧已知 σ 噪声过生产匹配链）；
        # 采图建模板/现场标定失败回退画布探针（已知对长臂 cross 低估， capped
        # 工况下无影响）。标定失败（无 cv2/退化模板）不阻塞建模板。
        # PNG 已落盘，探针按文件读模板图
        try:
            with self._lock:
                if self._matcher is None:
                    self._matcher = Matcher()
                matcher = self._matcher
            calib = self._live_noise_probe(cam, meta, matcher) \
                if source == "live" else None
            if calib is None:
                calib = noise_response_probe(matcher, meta,
                                             (cam.px_x, cam.px_y))
        except VisionError:
            calib = None
        if calib:
            meta.update(calib)
        self.templates.put(name, meta)
        return {"ok": True, "template": meta,
                "warning": None if light_on else "光源灭时建的模板（暗图）"}

    def template_update(self, name, new_name=None, threshold=None):
        meta = self.templates.get(name)
        if meta is None:
            raise VisionError("模板不存在: %r" % name)
        if new_name and new_name != name:
            self.templates.rename(name, new_name)
            name = new_name
            meta = self.templates.get(name)
        if threshold is not None:
            if not (0.0 < float(threshold) <= 1.0):
                raise VisionError("threshold 须在 (0,1]")
            meta = dict(meta)
            meta["threshold"] = float(threshold)
            self.templates.put(name, meta)
        return {"ok": True, "template": self.templates.get(name)}

    def template_delete(self, name):
        self.templates.remove(name)
        return {"ok": True, "deleted": name}

    # ---- 相机参数在线改 ----
    def camera_config(self, cam_id, req):
        rt = self._cam(cam_id)
        cam = rt.model
        # 内参字段已迁至机器常数（单一事实源）：这里写一律 400 指路
        moved = [k for k in req if k in INTRINSIC_FIELDS]
        if moved:
            raise VisionError(
                "字段 %s 是相机内参（机器常数），请用 PUT "
                "/api/machine_constants（体 cameras.%s.<字段>）"
                % (moved, cam_id))
        out = {}
        if "exposure_ms" in req:
            v = req["exposure_ms"]
            if not isinstance(v, (int, float)) or isinstance(v, bool) \
                    or not (0.1 <= v <= 1000):
                raise VisionError("exposure_ms 须在 [0.1,1000] ms")
            cam.exposure_ms = float(v)
            out["exposure_ms"] = cam.exposure_ms
        if "gain" in req:
            v = req["gain"]
            if not isinstance(v, (int, float)) or isinstance(v, bool) \
                    or not (0.1 <= v <= 64):
                raise VisionError("gain 须在 [0.1,64]")
            cam.gain = float(v)
            out["gain"] = cam.gain
        if "score_threshold" in req:
            v = req["score_threshold"]
            if not isinstance(v, (int, float)) or not (0.0 < v <= 1.0):
                raise VisionError("score_threshold 须在 (0,1]")
            cam.score_threshold = float(v)
            out["score_threshold"] = cam.score_threshold
        if "view_rot_deg" in req:
            v = req["view_rot_deg"]
            if not isinstance(v, (int, float)) or isinstance(v, bool) \
                    or not math.isfinite(float(v)) \
                    or abs(float(v)) > 5.0:
                raise VisionError("view_rot_deg 须在 [-5,5]"
                                     "（相机物理安装旋转角·真值 °，逆时针为正，图像内容反向旋转；与机器常数 rot_mrad（控制器认知）配套，相等=补偿到位）")
            cam.view_rot_deg = float(v)   # 真值热更（view_rot property 即时跟随，投影链）
            out["view_rot_deg"] = cam.view_rot_deg
        if "mode" in req:
            if req["mode"] not in vm.CAMERA_MODES:
                raise VisionError("mode 只能是 %s" % (vm.CAMERA_MODES,))
            cam.mode = req["mode"]
            out["mode"] = cam.mode
        if not out:
            raise VisionError("无可改字段（exposure_ms/gain/score_threshold"
                              "/mode/view_rot_deg 至少一项）")
        resp = {"ok": True, "cam": cam_id, "config": out}
        if req.get("persist"):
            self._persist_camera_config(cam_id)
            resp["persisted"] = True
        return resp

    def _persist_camera_config(self, cam_id):
        """把该相机当前生效的采集参数（exposure_ms/gain/score_threshold/
        mode/view_rot_deg=相机物理安装旋转角·真值）原子写回 vision_cameras.json 对应相机条目
        （tmp+replace，同 machine_constants 惯例）；写前全量 
        validate_config，写后重载再校验一次，坏配置不留在盘上。"""
        cam = self.cams[cam_id].model
        with open(self._cfg_path, encoding="utf-8") as f:
            cfg = json.load(f)
        ent = cfg["cameras"][cam_id]
        ent["exposure_ms"] = cam.exposure_ms
        ent["gain"] = cam.gain
        ent["score_threshold"] = cam.score_threshold
        ent["mode"] = cam.mode
        ent["view_rot_deg"] = cam.view_rot_deg
        errors = vm.validate_config(cfg)
        if errors:
            raise VisionError("persist 校验失败（%d 条）：\n  %s"
                              % (len(errors), "\n  ".join(errors)))
        tmp = self._cfg_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=1)
            f.write("\n")
        os.replace(tmp, self._cfg_path)
        with open(self._cfg_path, encoding="utf-8") as f:
            errs2 = vm.validate_config(json.load(f))
        if errs2:
            raise VisionError("persist 写后重载校验失败：\n  %s"
                              % "\n  ".join(errs2))

    # ---- 机器常数热生效（PUT /api/machine_constants 落盘后调用） ----
    def apply_constants(self):
        """把机器常数内参热写回现有 CameraModel（渲染/解析/对焦即时跟随）。"""
        with self._lock:
            vm.apply_constants({cid: rt.model for cid, rt in self.cams.items()},
                               self.constants)

    def constants_px_lookup(self, cid):
        rt = self.cams.get(cid)
        return (rt.model.px_x, rt.model.px_y) if rt else (1280, 960)

    def _cam(self, cam_id):
        rt = self.cams.get(cam_id)
        if rt is None:
            raise VisionError("未知相机 %r（有 %s）"
                              % (cam_id, ",".join(self.cams)))
        return rt


class VisionClient(object):
    """主控侧视觉 MQTT 客户端（flow_engine coarse_offset/ega_measure/
    fl align_first/fine_align 的真订阅消费端，照 ConfocalClient 模式）。

    线程模型：后台收线线程订阅 vision/result/+，收到消息**只**解析
    JSON 按 (cam, req_id) 双键填字典并 notify（绝不碰引擎状态；cam
    迭代 31 起随结果载荷，旧帧/无 cam 键的帧落 (None, req_id) 兼容位）；
    request() 由流程
    线程调用——发布触发 → Condition 等对应 req_id → 返回 entry 形态
    dict（{"result": <七字段>, "error": ...}，与 VisionService.identify
    返回值同用法），超时抛 RuntimeError。断线 0.5s 退避重连。
    """

    def __init__(self, host="127.0.0.1", port=1883):
        self._hp = (host, port)
        self._cond = threading.Condition()
        self._by_req = {}                # (cam|None, req_id) -> 结果 dict（读后即删）
        self._seq = 0

    def start(self):
        threading.Thread(target=self._recv_loop, name="vision-client",
                         daemon=True).start()
        return self

    def _recv_loop(self):
        host, port = self._hp
        while True:
            try:
                sub = MqttSub(host=host, port=port,
                              client_id="vision-cli-%d" % id(self))
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
                    # 迭代 31 起 (cam, req_id) 双键：结果载荷带 cam，
                    # 跨相机同 req_id 不串答；旧帧/无 cam 键的帧落
                    # (None, rid) 兼容位（request 侧 optional 匹配）
                    with self._cond:
                        self._by_req[(msg.get("cam"), rid)] = msg
                        # 防泄漏：滞留消息上限（正常路径读后即删；
                        # dict 保序=插入序，淘汰最旧）
                        if len(self._by_req) > 64:
                            for k in list(self._by_req)[:-64]:
                                self._by_req.pop(k, None)
                        self._cond.notify_all()
            except (ConnectionError, socket.timeout, OSError):
                time.sleep(0.5)

    def request(self, cam, template=None, timeout=10.0, ir_overlay=False):
        """发布一次识别触发并等结果 -> {"result": <七字段>, "error": ...}
        （无 error 键为 None）；超时抛 RuntimeError。template 可为模板名
        列表（双模板套刻，迭代 15）；ir_overlay=红外套刻通道。
        req_id 带实例后缀（2026-09-05 迭代 17）：两个 VisionClient 实例
        共用 broker 时 seq 命名空间相同，后发的请求会被对端历史结果
        （同 req_id 的滞留消息）误答——迭代 17 冒烟实测踩中（脚本客户端
        收到 chain 流程客户端的同号旧结果，FAST/REAL 串答）。
        迭代 31 起认领升级 (cam, req_id) 双键：载荷有 cam 才校验
        （旧帧/无 cam 键的帧仍收），跨相机同 req_id 不再串答。"""
        with self._cond:
            self._seq += 1
            req_id = "vs-%06d-%x" % (self._seq, id(self) & 0xFFFF)
        host, port = self._hp
        payload = {"req_id": req_id, "cam": cam, "template": template}
        if ir_overlay:
            payload["ir_overlay"] = True
        mqtt_publish(TRIGGER_TOPIC % cam,
                     json.dumps(payload, ensure_ascii=False),
                     host, port)
        deadline = time.time() + timeout
        with self._cond:
            while True:
                # 双键认领：优先 (本相机, req_id)；兼容位 (None, req_id)
                # 收旧帧/无 cam 键的帧（有 cam 才校验，缺省不拒）
                msg = self._by_req.pop((cam, req_id), None)
                if msg is None:
                    msg = self._by_req.pop((None, req_id), None)
                if msg is not None:
                    break
                left = deadline - time.time()
                if left <= 0:
                    raise RuntimeError(
                        "视觉 MQTT 识别超时（%.1fs 未收到 %s 的结果，"
                        "vision 服务/broker 异常）" % (timeout, req_id))
                self._cond.wait(min(left, 0.5))
        return {"result": msg.get("result") or {},
                "error": msg.get("error"),
                "template": msg.get("template")}
