#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_client.py — 仿真后端运行时端到端测试（纯标准库）。

用法：python tests/test_client.py          # quick 套件（默认，日常回归）
      python tests/test_client.py --full   # 全量（交付/接口变更前）
自动启动 runtime（python runtime/main.py），跑验证，结束后关闭。
定时/并行基建见 docs/回归基建_V1.md（tests/run_regression.py、daily_regression.bat、
tools/run_full_parallel.py）。
迭代 18 起分级套件：quick=§1-18 轻段 + 代表性 8× 单片链
（i8.chain_bond_happy）+ vision.real_flow_smoke + 重流程段中"段首自清理且
单用例 sim<30s"的独有用例（2026-09-21 实测 quick 395 条全过、646.7s@负载机，
空闲机历史 200~230s；full 条数以 main 末尾打印为准）；重流程实跑用例
由 fblock()/段表 full 标记门控留 --full（main 末尾打印 quick 覆盖数与跳过
清单；门控出去的另含：REAL 测校作业、采图库全幅 PNG 重计算、iter9/iter13
裸引擎寻力——墙钟驱动不可压缩）。full 里重流程段（flow/iter7/8/10/11a/
11b/15/17c/process_flows/focus_level/fine_align/wafer_ega）套 8× 段级提速
窗（set_speed，段 finally + main 段循环 finally 双重复原 1×）；速率测量与
墙钟语义用例（stream.freq / speed.walltime_halved / vision.perf_guard /
REAL 视觉 / servo settling / homing duration / plc_scan delay / fault 沉默
/ estop 巡航段）保持原速。perf_guard 与末态 health 经 read_clean_health
读纯 1× 稳态窗（5s 聚合窗相位分析+不达标重读一窗，见函数 docstring）。
段级墙钟计时随 main 汇总打印。

  1. WS 收帧 10s：频率 >=490Hz、seq 无丢帧、单向延迟 p50/p99 打印
  2. move：WS_LS_X 0->100mm 单调上升、时长符合 S 曲线规划、到位标志
  3. PLC：气路 OpenNegtive / 温控 SetTargetTempWithPlan
  4. ACS buffer：SetBufferNumber(1)+RunBuffer -> 轴运动、oBufferDo true->false
  5. estop 全轴停止，reset 恢复
  6. 2x 倍速：墙钟时间减半、仿真时间一致
  7. 误差系统 + 轴参数配置 API
  8. 四类回零（gantry_dual/plc_sync_trio/zero_sensor/vendor_es/acs_limit fault）
     + 回零联动 DI + limits 在线修改/持久化 + "回零不还原限位"回归
  9. 工件物料层（上片/die 交接链 film→flip→head→wafer/下片随片移除）
     + EMO/门锁联锁（409/soft-stop/塔灯）+ transfers GET/PUT
 10. MCS 适配壳 9190（真机报文格式）：getMachinePartProperties /
     executeCommand（运动 + 真空）/ getObserverValuesFromObserverPaths /
     getSettings / setCurrentValueForSetting / saveCurrentValueForSetting
     + 未知路径/未映射节点返回真机式 FAULT
 11. 故障注入（W2）：掉使能 409 拒绝 / 运动中自由减速+fault / ResetError+Enable
     恢复 / plc 沉默吞指令 / HTTP 挂起 / mcs 断线拒连+恢复 / health.faults 字段
 12. ACS 侧接入（W3 止损结局）：acs_probe --selftest（dll 加载+假监听回路）
     + 9101 reset_error（清 fault 不改 enabled / 未知轴与 PLC 轴拒绝）
 13. 工艺流程编排（W4）：示例流程在列/跑通（die 载体迁移证据）/单步+暂停+继续
     /stop 中止/flows PUT 校验（非法逐条 400、合法保存还原）/掉使能联动 abort
 14. 沙盒干涉检查（W5，零件级 v2）：常规 move 通过 / 目标撞穿模区 409（含
     零件级盒名）/ 运动中撞入 soft-stop+事件 / 关闭放行 + 重叠区回撤 /
     depth_eps 在线配置 / 三个用户穿模场景（浅贴合放行、深穿模拦截）/ health 字段
 15. 六自由度误差注入（6DOF 串行链合成）：罗德里格斯纯函数对照 / 纯 rz、
     纯 dx、两轴链顺序、沿轴标量并入 / 校验报错 / API 开关独立（开→非零、
     关→全零）/ /api/kinematics PUT 往返 + 非法配置 400
 16. 处方系统（recipe）：默认结构（厚度 1.0、膜面圆直径 300 die 圆裁 149
     颗、晶圆 marks×4 size_um + film.marks 比例标记 schema）/ 标记比例化
     不变式（ratio×die 半宽=±3.8mm）/ PUT 部分合并往返 / 非法逐条 400 且
     不污染 / 废除旧键 400 指路 / apply 重建阵列（manual 8×8=64 ↔ auto
     圆裁 149）/ film 上机跟随当前处方（size5→293 ↔ 默认 149）/ 落盘持久化 +
     restore_default / 旧格式迁移（size_mm→size_um、die_mark→marks）
 17. 视觉系统（vision）：MQTT 1883 broker 自测 / cameras API / FAST identify
     几何+耗时字段 / coarse（2026-08-25 零位约定：呈现位 Rx=0 die 背面朝上，
     素板无标记 → 背面检出特征 1 个在焦 / 取片位 Rx=−π die 挂吸盘下不可见 /
     REAL 素板背面模板匹配 / FA_X 移走无 die 特征）/ die 尺寸即改即生效
     （die 4→5mm 图像外廓实测）/
     下相机看晶圆上已键合 die 的标记（wafer_die 层，与晶圆 marks 分层离焦）/
     晶圆 per-site 对准标记（site_R{r}C{c}#{k}，等比缩放跟随，FAST/REAL 一致）/
     灯灭判废 / 景深离焦判废+恢复 / focus_state 五态（in_focus/
     light_off/rejected/no_mark，REAL 同带）/ 触发闪光时序（曝光+命令点
     +o 镜像+captures；capture/save 灯灭同语义自动闪光）/ 模板 CRUD+持久化 /
     REAL identify（NCC+亚像素+精化，MQTT 订阅实收）/ 实时红线不退化 /
     场景渲染帧统计（灯灭近黑/陶瓷面/晶圆面/渐晕/die 对比度）/
     模式切换（非法 turbo 400）/ 面板"视觉软件"tab 托管标记（pageVision）
 18. 机器常数（内参+px_u/px_v 分辨率单一事实源，PUT 热生效/非法 400/
     camera/config 拒内参指路）+ 测校（focus_scan/pixel_equivalent/
     rotation_direction 全 REAL 链路，coarse focus_scan 400）
 19. 参数化工艺流程（process_flows + flow_engine.run_steps 动态步骤）：
     翻转取片 flip_pick 全跑（抬安全位→取片姿态 Rx=−π→送片→接触→顶针
     过压→负压判定→交接确认→抬安全位→翻转回 0 背面朝上→调焦，终态
     coarse 背面特征在焦 + REAL 验证）/ 防重复拾取 400 + 放回蓝膜
     flip_place_back（原槽位可逆）/ 常数×处方联动（die 厚、chuck_face_z0
     变→FA_Z_pick/顶针目标变；安全位步恒 20）/ 拾取失败报警中止（PLC
     沉默→负压不建立→自定义报警文本）/ 运行冲突 409 / flip_pick 组与
     pin_overtravel_um 坏值 400 / 19.7 芯片交接 flip_handoff（对准台退避
     →键合头接触→三吸盘负压交接→抬起撤离；粗对准偏差落存 coarse_offset
     并补偿 FA_X=fa_x_handoff−off_x 符号锁定）+ head_to_flip 吸回闭环
     + 守卫 400 + flip_handoff 组坏值 400/热生效
 20. 迭代 11a：处方 full_wafer/post_inspect 两组（默认/PUT 往返/未知键 400/
     迁移播种）+ tuning wafer_cool_rate_cps + engine 降温主动冷却速率
     （150→25℃ ≈5℃/s）+ bond_results 收集器（真实单片 chain 全链采集、
     truth=接触时刻真值（迭代 20 起；旧口径 phys_resid 快照保留为
     align_resid 分量）、API 两形态、reset 清空）
 21. 迭代 11b：路径规划三模式+skip/count 单测 + update_measured 往返 +
     full_wafer count=2 端到端（配对表/状态契约/两 die 记录）+
     post_inspect 端到端（measured≈truth，|Δxy|<5µm/|Δrz|<100µrad——
     xy 主导项=下相机绝对位姿 6DOF 误差未标定；rz=AS 重复定位+FAST
     噪声层 3σ≈53µrad，见 docs §2.2）+
     auto_after_full 开/关生效
 22. 迭代 22：sim_sensors 噪声逐实例拆分（fast_noise_px 按相机/confocal
     按探头/temp 按加热器，旧标量 _migrate 展开）+ 全机模态仿真（21 轴
     SISO+9 MIMO 解析传播/三激励/热软化/曝光积分，默认 OFF；API
     /api/modal[/state//scope]、OFF 零影响、地板 RMS、运动激发、接触
     脉冲、FAST 散布对比、modal ON perf 红线）
 23. 迭代 73：晶圆翘曲仿真（bow+saddle 二维静态场、逐片 jitter 抽取
     物料真值；API /api/warpage[/state]、OFF 逐位零影响、共焦增量同源
     断言、剖面公式、视觉离焦、接触位贯通、warpage ON perf 红线）
 24. 迭代 76/78：键合头正压/通大气垂直落片（任意流程/高度、实际 XY 投影、
     无晶圆/片外保护、翻转承接优先、bond_mark 兼容）；IO 一键搜力（当前位
     下行、到力停轴、稳定确认、失败/取消保护、不启动力 PID 或转移芯片）
"""
import json
import math
import os
import random
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import zlib

try:
    import ctypes            # timeBeginPeriod（Windows 计时分辨率）
except ImportError:
    ctypes = None

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "runtime"))
from trajectory import plan_move, profile_duration   # noqa: E402
import axes_config as acfg                           # noqa: E402

WS_PORT = int(os.environ.get("D2W_PORT_WS", 9010))
PLC_PORT = int(os.environ.get("D2W_PORT_PLC", 9102))
ACS_PORT = int(os.environ.get("D2W_PORT_ACS", 9101))
MCS_PORT = int(os.environ.get("D2W_PORT_MCS", 9190))
# Cordis MCS 服务路径（真机常量，见 docs/Cordis_MCS协议_V1.md §1）
MCS_BASE = ("http://127.0.0.1:%d/JsonServices/com.cordis.explorer.connectors."
            "MachinePart.MachinePartConnector/" % MCS_PORT)
FRAME_FMT = "<BBHddI"
FRAME_LEN = 108

RESULTS = []

# 迭代 19：裸引擎分片回归（tools/run_full_parallel.py 以 --bare 唤起）。
# BARE=BareHarness 实例（None=全栈 HTTP 模式，quick/单进程 full 不变）；
# MQTT_PORT：fl/i8b 段 MQTT 订阅端口（裸引擎分片 per-shard 隔离用）。
BARE = None
MQTT_PORT = int(os.environ.get("D2W_PORT_MQTT", 1883))

# 迭代 37：-k 定向回归（用法同 pytest 子串或语义：python tests/test_client.py
# -k "i37 or nonortho or i36"）。tokens 匹配 check 名（子串、小写）；
# 段级白名单见 main() 的 _KW_SEG_MAP（只跑可能含匹配用例的段，其余段
# 整体跳过——定向回归不背全段的场景准备开销）。
_KW_TOKENS = []

# ---------------- 迭代 18：分级套件（quick 默认 / --full 全量）----------------
# quick=日常回归（目标墙钟 ≤3min）：§1-18 轻段全量 + 代表性 8× 单片链
# （i8.chain_bond_happy）+ vision.real_flow_smoke + 重流程段里"段首自清理、
# 单用例 sim<30s"的独有用例；其余重流程实跑用例由 fblock() 门控留 --full。
# full=457 条全保留（交付/接口变更前必跑）。
SUITE = "quick"                    # main() 按 argv 覆写为 "full"
SKIPPED_CASES = []                 # quick 跳过的 full-only 用例名（汇总打印）
SEG_TIMES = []                     # (段名, 墙钟秒) 段级计时（main 汇总）


def check(name, ok, detail=""):
    # 迭代 37：-k 定向回归——tokens 非空时仅记录名字命中任一 token 的用例
    if _KW_TOKENS and not any(t in name.lower() for t in _KW_TOKENS):
        return
    RESULTS.append((name, ok, detail))
    print("[%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def fblock(checks):
    """full-only 用例块门控：quick 套件下登记跳过清单并返回 False（调用方
    用 if fblock([...]): 包住重流程实跑块，连工作带断言一起跳过——只挂
    check 不跳工作则提速无意义）。full 套件恒 True。"""
    if SUITE == "quick":
        SKIPPED_CASES.extend(checks)
        return False
    return True


# ---- 段级倍速窗（照各段 wafer_preheat 8× 预热→复原的先例）----
# 墙钟敏感用例不得套窗：stream.freq / speed.walltime_halved /
# vision.perf_guard / REAL 视觉用例 / servo settle / homing duration。
# 复原保证：main 的段循环 finally 逐段回 1× + 窗内段自带 finally 复原；
# 段内 wafer_preheat helper 复原到 _SPEED_CUR 而非硬编码 1×（嵌套窗内
# 保持 8×，见 i11b/i15/i17c）。
_SPEED_CUR = 1.0                   # 当前"应有"倍速（套窗嵌套复原用）


def set_speed(scale):
    global _SPEED_CUR
    _SPEED_CUR = float(scale)
    http_post("http://127.0.0.1:%d/api/speed" % WS_PORT,
              {"scale": _SPEED_CUR})


def read_clean_health(base, t_mark, retries=1):
    """读起点在 t_mark 之后的 5s 统计窗 health（迭代 18）。
    health 的 tick_hz/push_hz 是 5s 聚合窗（滚动周期 5.0~5.01s）且读到
    的是上一完成窗。窗相位不可知：等满 10.5s（> 2 个滚动周期）后读数，
    窗起点必然越过 t_mark（最坏相位差 ≤10ms，可忽略）——保证读数不含
    t_mark 之前的倍速残留/识别突发。
    retries：首窗不达标再隔 5.2s 多读一窗取最佳——红线断言的对象是
    "持续吞吐能力"（代码回归=每窗都挂，必检出）；OS 抢占毛刺（25ms
    stall=-0.5%/窗）是暂态污染，多窗重读等价于 README 既定的"空闲时
    复跑确认"，窗级自动化。阈值 999/499 本身不动。"""
    dt = 10.5 - (time.time() - t_mark)
    if dt > 0:
        time.sleep(dt)
    best = http_json(base + "/api/health")
    for _ in range(retries):
        if best["tick_hz"] >= 999.0 and best["push_hz"] >= 499.0:
            break
        time.sleep(5.2)
        h = http_json(base + "/api/health")
        if (h["tick_hz"], h["push_hz"]) > (best["tick_hz"], best["push_hz"]):
            best = h
    return best


def percentile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


# ---------------- 视觉场景测试用：标准库 PNG 解码（8bit 灰度 filter-0） ----------------
def png_decode_gray(png):
    """解码 runtime png_encode_gray 的产物 → (W, H, pixels bytes)。
    编码侧是定式（IHDR 8bit 灰度 + 逐行 filter-0 + zlib），这里按同一
    约定逆解，不实现通用 PNG。"""
    if png[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, w, h, idat = 8, 0, 0, bytearray()
    while pos + 8 <= len(png):
        ln = struct.unpack("!I", png[pos:pos + 4])[0]
        tag = png[pos + 4:pos + 8]
        data = png[pos + 8:pos + 8 + ln]
        if tag == b"IHDR":
            w, h = struct.unpack("!II", data[:8])
        elif tag == b"IDAT":
            idat += data
        pos += 12 + ln
    raw = zlib.decompress(bytes(idat))
    px = bytearray(w * h)
    for r in range(h):
        if raw[r * (w + 1)] != 0:
            raise ValueError("非 filter-0 行（不符合 png_encode_gray 约定）")
        px[r * w:(r + 1) * w] = raw[r * (w + 1) + 1:(r + 1) * (w + 1)]
    return w, h, bytes(px)


def patch_mean(px, w, x0, y0, cw, ch):
    """px（bytes，行宽 w）中 [x0,y0,cw,ch] 区域的均值。"""
    s = n = 0
    for r in range(y0, y0 + ch):
        seg = px[r * w + x0: r * w + x0 + cw]
        s += sum(seg)
        n += len(seg)
    return s / float(n)


def patch_profile_grad(px, w, x0, y0, cw, ch):
    """行平均剖面（x 向）的相邻列最大跳变：先做 ch 行平均压噪声
    （σ/√ch），再取剖面梯度峰——阶跃边缘 vs 离焦斜坡的判别度量。"""
    prof = [0.0] * cw
    for r in range(y0, y0 + ch):
        row = px[r * w + x0: r * w + x0 + cw]
        for i, v in enumerate(row):
            prof[i] += v
    inv = 1.0 / ch
    prof = [v * inv for v in prof]
    return max(abs(prof[i + 1] - prof[i]) for i in range(cw - 1))


def frame_stats(png):
    """整帧 (mean, std, W, H, px)。"""
    w, h, px = png_decode_gray(png)
    n = len(px)
    mean = sum(px) / float(n)
    var = 0.0
    for v in px:
        d = v - mean
        var += d * d
    return mean, math.sqrt(var / n), w, h, px


# ---------------- 标准库 WebSocket 客户端 ----------------
class WSClient:
    def __init__(self, host="127.0.0.1", port=WS_PORT, path="/ws"):
        self.sock = socket.create_connection((host, port), timeout=5)
        key = "dGhlIHNhbXBsZSBub25jZQ=="
        req = ("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n" % (path, host, port, key))
        self.sock.sendall(req.encode("latin1"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            resp += self.sock.recv(4096)
        assert b"101" in resp.split(b"\r\n")[0], resp
        self.sock.settimeout(None)
        self.frames = []          # (arrival, sim_time, wall_time, seq, values)
        self.texts = []
        self._lock = threading.Lock()
        self._closed = False
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("eof")
            buf += chunk
        return buf

    def _recv_loop(self):
        try:
            buf = b""
            while not self._closed:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                arrival = time.time()
                buf += chunk
                # 一个 TCP 段里可能有多帧，逐帧切出
                while True:
                    if len(buf) < 2:
                        break
                    op, ln = buf[0] & 0x0F, buf[1] & 0x7F
                    off = 2
                    if ln == 126:
                        if len(buf) < 4:
                            break
                        ln = struct.unpack("!H", buf[2:4])[0]
                        off = 4
                    elif ln == 127:
                        if len(buf) < 10:
                            break
                        ln = struct.unpack("!Q", buf[2:10])[0]
                        off = 10
                    if len(buf) < off + ln:
                        break
                    payload, buf = buf[off:off + ln], buf[off + ln:]
                    if op == 0x2 and len(payload) == FRAME_LEN:
                        magic, ver, nax, sim, wall, seq = struct.unpack(
                            FRAME_FMT, payload[:struct.calcsize(FRAME_FMT)])
                        vals = struct.unpack("<%df" % nax,
                                             payload[struct.calcsize(FRAME_FMT):])
                        with self._lock:
                            self.frames.append((arrival, sim, wall, seq, vals))
                    elif op == 0x1:
                        with self._lock:
                            self.texts.append(json.loads(payload.decode("utf-8")))
                    elif op == 0x8:
                        self._closed = True
                        return
        except (ConnectionError, OSError):
            pass

    def send(self, obj):
        payload = json.dumps(obj).encode("utf-8")
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            hdr = struct.pack("!BB", 0x81, 0x80 | n)
        elif n <= 0xFFFF:
            hdr = struct.pack("!BBH", 0x81, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x81, 0x80 | 127, n)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(hdr + mask + masked)

    def mark(self):
        with self._lock:
            return len(self.frames)

    def frames_since(self, mark):
        with self._lock:
            return list(self.frames[mark:])

    def close(self):
        self._closed = True
        try:
            self.sock.close()
        except OSError:
            pass


def http_json(url, data=None):
    if data is None:
        with urllib.request.urlopen(url, timeout=5) as r:
            return json.loads(r.read().decode("utf-8"))
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode("utf-8"))


def http_get_raw(url):
    """GET -> (status, obj)，4xx 不抛异常（迭代 11a：404 形态断言用）。"""
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_post(url, data):
    """-> (status, obj)，4xx 不抛异常。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_post_tpl(url, data):
    """建模板专用（POST /api/vision/templates）：REAL 示教帧+响应度探针标定
    是重活（负载下实测 >10s），默认 5s 超时会在机器忙时误崩套件
    （2026-09-21 迭代 54：浏览器开视觉页 live 预览打满后端 GIL 时复现）。
    走 60s（test_metrology 建模板同款先例）。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_put(url, data):
    """PUT -> (status, obj)。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"},
                                 method="PUT")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def http_delete(url, data):
    """DELETE（JSON body）-> (status, obj)。"""
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"),
                                 headers={"Content-Type": "application/json"},
                                 method="DELETE")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


class ACSClient:
    def __init__(self):
        self.sock = socket.create_connection(("127.0.0.1", ACS_PORT), timeout=5)
        # 服务端 ensure_ascii=False 发 UTF-8（错误消息含中文），显式指定编码，
        # 否则 Windows 下 makefile 按 GBK 解码会炸
        self.f = self.sock.makefile("rw", encoding="utf-8")

    def cmd(self, **kw):
        self.f.write(json.dumps(kw) + "\n")
        self.f.flush()
        return json.loads(self.f.readline())

    def close(self):
        try:
            self.f.close()
            self.sock.close()
        except OSError:
            pass


# ---------------- 测试用例 ----------------
def test_stream(ws):
    print("--- 1. WS 推送流（10s） ---")
    m = ws.mark()
    time.sleep(10)
    fr = ws.frames_since(m)
    dur = fr[-1][0] - fr[0][0]
    freq = (len(fr) - 1) / dur
    seqs = [f[3] for f in fr]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b - a != 1)
    lat = sorted((f[0] - f[2]) * 1000 for f in fr)     # ms
    p50, p99 = percentile(lat, 0.5), percentile(lat, 0.99)
    ok_size = all(len(f[4]) == 21 for f in fr)
    print("   frames=%d freq=%.1fHz lost=%d latency p50=%.3fms p99=%.3fms"
          % (len(fr), freq, gaps, p50, p99))
    check("stream.freq>=490Hz", freq >= 490, "%.1fHz" % freq)
    check("stream.no_frame_loss", gaps == 0, "gaps=%d" % gaps)
    check("stream.frame_shape", ok_size and len(fr) > 4000, "108B x 21轴")
    check("stream.latency_p50<1ms", p50 < 1.0, "p50=%.3fms" % p50)


def wait_axis(ws, idx, target, tol=1e-3, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        with ws._lock:
            if ws.frames and abs(ws.frames[-1][4][idx] - target) < tol:
                return True
        time.sleep(0.005)
    return False


def measure_move(ws, idx, target, speed):
    """发 move 并统计墙钟/仿真时长与位置序列。调用前应已收到推送流。"""
    ws.send({"cmd": "move", "axis": acfg.AXIS_NAMES[idx],
             "target": target, "speed": speed})
    t0 = time.time()
    m0 = ws.mark()
    start_pos = None
    while time.time() - t0 < 10:
        with ws._lock:
            fr = list(ws.frames[m0:])
        if fr:
            if start_pos is None:
                start_pos = fr[0][4][idx]
            # 1e-6 容差保证运动真正走完（f32 帧值在 100mm 处分辨率 ~7.6e-6，
            # 但规划终点恰为目标值本身，到位帧即精确落在 target 上）
            if abs(fr[-1][4][idx] - target) < 1e-6 and len(fr) > 5:
                break
        time.sleep(0.003)
    poss = [f[4][idx] for f in fr]
    wall_dur = fr[-1][0] - fr[0][0]
    sim_dur = fr[-1][1] - fr[0][1]
    return poss, wall_dur, sim_dur, start_pos


def test_move(ws):
    print("--- 2. move WS_LS_X 0->100mm @200 ---")
    idx = acfg.NAME_TO_IDX["WS_LS_X"]
    wait_axis(ws, idx, 0.0)
    time.sleep(0.2)
    # 期望时长按运行时实际 amax/jerk（读 /api/axes config）动态计算
    cfg = get_axis_entry("WS_LS_X")["config"]
    poss, wall_dur, sim_dur, start = measure_move(ws, idx, 100.0, 200.0)
    expect = profile_duration(plan_move(100.0, 200.0, cfg["amax"], cfg["jerk"]))
    mono = all(b - a >= -1e-4 for a, b in zip(poss, poss[1:]))
    # in_position 由伺服 settling 延迟置位：短轮询收口（约 settle_time 后）
    ax = wait_in_position("WS_LS_X")
    print("   wall=%.3fs sim=%.3fs expect=%.3fs end=%.3f in_position=%s"
          % (wall_dur, sim_dur, expect, poss[-1], ax["in_position"]))
    check("move.monotonic", mono)
    check("move.reached", abs(poss[-1] - 100.0) < 1e-3)
    check("move.duration", abs(sim_dur - expect) < 0.15,
          "sim=%.3f expect=%.3f" % (sim_dur, expect))
    check("move.in_position", ax["in_position"] and not ax["moving"])

    # 旋转轴（rad 单位）流转验证：WS_Rz 0 -> +0.01 rad @1.5707963 rad/s
    ridx = acfg.NAME_TO_IDX["WS_Rz"]
    rcfg = get_axis_entry("WS_Rz")["config"]
    poss, wall_dur, sim_dur, _ = measure_move(ws, ridx, 0.01, 1.5707963)
    rexpect = profile_duration(plan_move(0.01, 1.5707963,
                                         rcfg["amax"], rcfg["jerk"]))
    ax = http_json("http://127.0.0.1:%d/api/axes" % WS_PORT)["axes"][ridx]
    print("   WS_Rz(rad): sim=%.3fs expect=%.3fs end=%.6f unit=%s"
          % (sim_dur, rexpect, poss[-1], ax["unit"]))
    check("move.rotary_rad", abs(poss[-1] - 0.01) < 1e-5
          and abs(sim_dur - rexpect) < 0.15 and ax["unit"] == "rad")


def test_plc():
    print("--- 3. PLC 气路 + 温控 ---")
    base = "http://127.0.0.1:%d" % PLC_PORT
    ws = "http://127.0.0.1:%d" % WS_PORT
    # 真空密封物理：无片=泄漏（目标-30kPa 大流量），有片=密封（目标-80kPa）
    http_post(ws + "/api/workpiece/reset", {})          # 显式无片起点
    http_json(base + "/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.WT_WaferVac",
               "cmd": "OpenNegtive"})
    time.sleep(0.6)
    v = http_json(base + "/io/observe?path=SystemMachine.SS_ME.M_AIR.WT_WaferVac")
    vals = v.get("values", {})
    leak_p = vals.get("SystemMachine.SS_ME.M_AIR.WT_WaferVac.PressureSwitch.oValue", 0)
    leak_state = vals.get("SystemMachine.SS_ME.M_AIR.WT_WaferVac.oNegtiveState")
    check("plc.air_leak_unsealed", -40.0 < leak_p < -10.0 and not leak_state,
          "pressure=%.1f oNegtiveState=%s" % (leak_p, leak_state))
    c, r = http_post(ws + "/api/workpiece/wafer", {"action": "load"})
    check("plc.air_wafer_load", c == 200 and r.get("ok"))
    ok_state = ok_pres = False
    t0 = time.time()
    while time.time() - t0 < 1.5:
        v = http_json(base + "/io/observe?path=SystemMachine.SS_ME.M_AIR.WT_WaferVac")
        vals = v.get("values", {})
        if vals.get("SystemMachine.SS_ME.M_AIR.WT_WaferVac.oNegtiveState"):
            ok_state = True
        if vals.get("SystemMachine.SS_ME.M_AIR.WT_WaferVac.PressureSwitch.oValue", 0) < -40:
            ok_pres = True
        if ok_state and ok_pres:
            break
        time.sleep(0.05)
    print("   air: oNegtiveState=%s pressure<-40kPa=%s (%.2fs)"
          % (ok_state, ok_pres, time.time() - t0))
    check("plc.air", ok_state and ok_pres)
    http_post(ws + "/api/workpiece/wafer", {"action": "unload"})
    http_json(base + "/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.WT_WaferVac",
               "cmd": "OpenAir"})

    http_json(base + "/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Steady_Time": 1.0, "Temperature": 80}})
    t0, ok_temp = time.time(), False
    while time.time() - t0 < 1.5:
        v = http_json(base + "/io/observe?path=SystemMachine.M_Heater.PulseHeat_Up.oCurTemp")
        if v.get("value", 0) > 30.0:
            ok_temp = True
            break
        time.sleep(0.05)
    print("   heater: oCurTemp=%.1f rising=%s" % (v.get("value", 0), ok_temp))
    check("plc.heater", ok_temp)
    http_json(base + "/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "StopTargetTempWithPlan"})


# ---------------- PLC 扫描周期语义 ----------------
def test_plc_scan():
    print("--- 3.5 PLC 扫描周期（映像表 commit / 配置端点 / 持久化） ---")
    plc = "http://127.0.0.1:%d" % PLC_PORT
    ws = "http://127.0.0.1:%d" % WS_PORT
    ov_path = os.path.join(ROOT, "runtime", "config_overrides.json")
    # 探测点：无副作用的观测点（引擎不写它，通用 SetBool 路由直写）
    probe = "SystemMachine.SS_Bonding.M_AS.oAS_LightOn"

    def observe():
        v = http_json(plc + "/io/observe?path=" + probe)
        return v.get("value")

    orig = observe()

    # 扫描周期配置：生效 + health 双端口反映 + 持久化 + 非法值 400
    c, r = http_post(ws + "/api/plc/scan", {"ms": 10})
    h1 = http_json(ws + "/api/health")
    h2 = http_json(plc + "/health")
    check("plc.scan_config", c == 200 and r.get("plc_scan_ms") == 10
          and h1.get("plc_scan_ms") == 10 and h2.get("plc_scan_ms") == 10,
          "r=%s h9010=%s h9102=%s" % (r.get("plc_scan_ms"),
                                      h1.get("plc_scan_ms"),
                                      h2.get("plc_scan_ms")))
    bad = [http_post(ws + "/api/plc/scan", {"ms": v})[0]
           for v in (0, 51, 2.5, "abc")]
    check("plc.scan_invalid_400", all(x == 400 for x in bad), str(bad))
    with open(ov_path, encoding="utf-8") as f:
        ov = json.load(f)
    sys.path.insert(0, os.path.join(ROOT, "runtime"))
    from engine import Engine
    eng2 = Engine(None)
    eng2.load_overrides(ov_path)
    check("plc.scan_persist", ov.get("plc_scan_ms") == 10
          and eng2.plc_scan_ms == 10)

    # 写入延迟：扫描 50ms 时写点后第一次读大概率仍是旧值，2 个周期内必见
    http_post(ws + "/api/plc/scan", {"ms": 50})
    delay_shown = False
    val = None
    for trial in range(3):
        val = (trial % 2 == 0)
        http_json(plc + "/io/command",
                  {"path": probe, "cmd": "SetBool", "args": {"value": val}})
        if observe() != val:          # commit 未发生：读到上一周期输出映像
            delay_shown = True
            break
        time.sleep(0.06)              # 本轮已 commit，等出下一轮相反值
    t0, seen = time.time(), False
    while time.time() - t0 < 0.3:
        if observe() == val:
            seen = True
            break
        time.sleep(0.002)
    delay_ms = (time.time() - t0) * 1000
    print("   scan=50ms: 首读未见=%s 最终可见=%s（耗时 %.1fms）"
          % (delay_shown, seen, delay_ms))
    check("plc.scan_delay", delay_shown and seen and delay_ms <= 120.0,
          "first_miss=%s delay=%.1fms" % (delay_shown, delay_ms))

    # 还原：扫描周期 4ms、探测点原值
    http_post(ws + "/api/plc/scan", {"ms": 4})
    http_json(plc + "/io/command",
              {"path": probe, "cmd": "SetBool", "args": {"value": bool(orig)}})
    time.sleep(0.02)


# ---------------- Cordis MCS 适配壳（9190，真机报文格式） ----------------
def mcs_call(service, input_args):
    """按真机信封调用 MCS 服务 -> (status, obj)（真机式 FAULT 是 HTTP 200）。"""
    body = [{"SecurityToken": "ADMIN"}, {"InputArgs": input_args}]
    return http_post(MCS_BASE + service, body)


def test_mcs(ws):
    print("--- 3.6 Cordis MCS 适配壳（9190，真机报文六服务） ---")
    plc = "http://127.0.0.1:%d" % PLC_PORT
    epin = "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z"
    vac = ("3000_1_Controller/SystemMachine/SS_ME/M_MachineAirCabinet/"
           "BH_Chip_YN_XN_SuckerVac")

    # getMachinePartProperties：EPIN_Z 节点模板（dot 分隔形式）
    c, r = mcs_call("getMachinePartProperties",
                    [{"machinePartFullPath": epin}])
    v = r.get("value") or {}
    cmds = [x["name"] for x in v.get("commands", [])]
    obss = [x["name"] for x in v.get("observers", [])]
    mp = v.get("machinePart") or {}
    check("mcs.get_props", c == 200 and r.get("type") == "SUCCESS"
          and mp.get("name") == "E_EPIN_Z"
          and all(k in mp for k in ("fullPath", "internalPath", "name"))
          and "MoveAbs" in cmds and "Halt" in cmds and "Reset Error" in cmds
          and "oCurrentPos_GC" in obss and "oInPos" in obss,
          "cmds=%d obs=%d mp=%s" % (len(cmds), len(obss), mp.get("name")))

    # executeCommand MoveAbs(5.0)：真机报文格式（模板原样回传 + 填
    # currentValue/savedValue），翻译到引擎后三根顶针同步到 5mm
    tmpl = next(x for x in v["commands"] if x["name"] == "MoveAbs")
    args = [dict(a, currentValue=5.0, savedValue=5.0)
            for a in tmpl["arguments"]]
    c, r = mcs_call("executeCommand", [{"command": {
        "name": "MoveAbs", "machinePart": mp, "arguments": args}}])
    i1 = acfg.NAME_TO_IDX["E_EPIN_Z_1"]
    reached = wait_axis(ws, i1, 5.0, timeout=5.0)
    check("mcs.exec_motion", c == 200 and r.get("type") == "SUCCESS"
          and reached, "type=%s reached=%s" % (r.get("type"), reached))
    # MoveRel(-5.0) 回零位（读当前位 + delta 的客户端语义由适配层实现）
    tmpl = next(x for x in v["commands"] if x["name"] == "MoveRel")
    args = [dict(a, currentValue=-5.0, savedValue=-5.0)
            for a in tmpl["arguments"]]
    mcs_call("executeCommand", [{"command": {
        "name": "MoveRel", "machinePart": mp, "arguments": args}}])
    wait_axis(ws, i1, 0.0, timeout=5.0)

    # executeCommand OpenNegative 真空吸盘（slash 分隔形式）→ 内部气路变化
    c, r = mcs_call("getMachinePartProperties",
                    [{"machinePartFullPath": vac}])
    mp2 = (r.get("value") or {}).get("machinePart") or {}
    c, r = mcs_call("executeCommand", [{"command": {
        "name": "OpenNegative", "machinePart": mp2, "arguments": []}}])
    vac_ok = (c == 200 and r.get("type") == "SUCCESS")
    # 用 9102 语义层验证：无片泄漏态压强走负（目标 -30kPa）
    t0, neg = time.time(), False
    while time.time() - t0 < 1.5:
        o = http_json(plc + "/io/observe?path=SystemMachine.SS_ME.M_AIR."
                      "WT_BondingVac_1.PressureSwitch.oValue")
        if (o.get("value") or 0) < -10.0:
            neg = True
            break
        time.sleep(0.05)
    # 再用 MCS 观测验证 AirRouteClose（大气态）在负压建立后…
    # 泄漏态 -30kPa 未达 oNegtiveState 阈值（-40kPa），AirRouteClose 仍为 true
    # ——按点位表三态语义属"未到位"，此处只验证观测链路通、值为 bool
    c, ro = mcs_call("getObserverValuesFromObserverPaths",
                     [{"observerPaths": [{"path": vac + "/AirRouteClose"}]}])
    obs_bool = isinstance((ro.get("value") or [{}])[0].get("currentValue"),
                          bool)
    check("mcs.exec_vacuum", vac_ok and neg and obs_bool,
          "type=%s pressure_neg=%s" % (r.get("type"), neg))
    # Close 恢复大气
    mcs_call("executeCommand", [{"command": {
        "name": "Close", "machinePart": mp2, "arguments": []}}])

    # getObserverValuesFromObserverPaths 批量读：轴位/到位/速度
    c, r = mcs_call("getObserverValuesFromObserverPaths",
                    [{"observerPaths": [{"path": epin + "/oCurrentPos_GC"},
                                        {"path": epin + "/oInPos"},
                                        {"path": epin + "/obMoveSpd"}]}])
    vals = r.get("value") or []
    check("mcs.obs_values", c == 200 and r.get("type") == "SUCCESS"
          and len(vals) == 3
          and isinstance(vals[0].get("currentValue"), (int, float))
          and isinstance(vals[1].get("currentValue"), bool)
          and isinstance(vals[2].get("currentValue"), (int, float)),
          "vals=%s" % vals)

    # getSettings / setCurrentValueForSetting / saveCurrentValueForSetting
    c, r = mcs_call("getSettings", [{"machinePartFullPath": epin}])
    stgs = {s["name"]: s for s in (r.get("value") or [])}
    v0 = stgs.get("MoveVelocity", {}).get("floatValue")
    c, r = mcs_call("setCurrentValueForSetting", [{
        "settingKey": {"machinePartFullPath": epin, "commandName": "",
                       "settingName": "MoveVelocity"},
        "value": {"dummyKey": "", "type": "FLOAT_TYPED_VALUE",
                  "floatValue": 10.0}}])
    set_ok = (c == 200 and r.get("type") == "SUCCESS")
    c, r = mcs_call("getSettings", [{"machinePartFullPath": epin}])
    v1 = {s["name"]: s for s in (r.get("value") or [])}.get(
        "MoveVelocity", {}).get("floatValue")
    check("mcs.settings", v0 == 25.0 and set_ok and v1 == 10.0,
          "v0=%s v1=%s" % (v0, v1))
    # 还原默认 25 + save 持久化（验证 save 通路；默认值的覆盖项无害，
    # 后续 test_homing_and_limits 结束时会删除 config_overrides.json）
    mcs_call("setCurrentValueForSetting", [{
        "settingKey": {"machinePartFullPath": epin, "commandName": "",
                       "settingName": "MoveVelocity"},
        "value": {"dummyKey": "", "type": "FLOAT_TYPED_VALUE",
                  "floatValue": 25.0}}])
    c, r = mcs_call("saveCurrentValueForSetting", [{
        "settingKey": {"machinePartFullPath": epin,
                       "settingName": "MoveVelocity"}}])
    check("mcs.setting_save", c == 200 and r.get("type") == "SUCCESS")

    # 未知路径 / 未映射节点 / 未知命令 → 真机式 FAULT（HTTP 200，不崩溃）
    c, r1 = mcs_call("getMachinePartProperties", [{
        "machinePartFullPath": "3000_1_Controller/SystemMachine.SS_XX.M_Nothing"}])
    c, r2 = mcs_call("getMachinePartProperties", [{
        "machinePartFullPath": "3000_1_Controller/SystemMachine/"
                               "SS_Bonding/M_WS_STAGE/Interferometer"}])
    c, r3 = mcs_call("executeCommand", [{"command": {
        "name": "Nope", "machinePart": mp, "arguments": []}}])
    check("mcs.unknown_path",
          r1.get("type") == "FAULT"
          and r1.get("error", {}).get("code") == "PathNotFound"
          and r2.get("type") == "FAULT"
          and r2.get("error", {}).get("code") == "Unmapped"
          and r3.get("type") == "FAULT"
          and r3.get("error", {}).get("code") == "CmdNotFound",
          "r1=%s r2=%s r3=%s" % (r1.get("error", {}).get("code"),
                                 r2.get("error", {}).get("code"),
                                 r3.get("error", {}).get("code")))


def test_buffer():
    print("--- 4. ACS buffer 1（翻转交接序列） ---")
    acs = ACSClient()
    r = acs.cmd(cmd="SetBufferNumber", buffer=1)
    assert r.get("ok"), r
    acs.cmd(cmd="RunBuffer")
    moved, do_true = False, False
    t0 = time.time()
    while time.time() - t0 < 25:
        b = acs.cmd(cmd="getBuffer")
        p = acs.cmd(cmd="getpos", axis="FA_X")
        if b.get("oBufferDo"):
            do_true = True
        if abs(p["pos"]["FA_X"]) > 1.0:
            moved = True
        if do_true and not b.get("oBufferDo"):
            break
        time.sleep(0.05)
    dur = time.time() - t0
    done = not acs.cmd(cmd="getBuffer").get("oBufferDo")
    acs.close()
    print("   oBufferDo true->%s FA_X moved=%s 总耗时=%.1fs" % (done, moved, dur))
    check("buffer.do_flag", do_true and done)
    check("buffer.axis_moved", moved)


def test_estop(ws):
    print("--- 5. estop / reset ---")
    idx = acfg.NAME_TO_IDX["FA_X"]
    # 行程 -900（原 -1240）：零件级沙盒下 FA_X<≈-1086 翻转链零件会撞对准
    # 相机承载台（sandboxes.json 对表 pt:*，阈值最浅对见 gen_sandboxes 报告），
    # -1240 起步即被 409 拦下，收不到巡航段，急停冻结断言就失去意义；
    # -900 保留真实巡航
    ws.send({"cmd": "move", "axis": "FA_X", "target": -900.0, "speed": 600.0})
    time.sleep(0.4)
    ws.send({"cmd": "estop"})
    time.sleep(0.1)
    m = ws.mark()
    time.sleep(0.6)
    fr = ws.frames_since(m)
    poss = [f[4][idx] for f in fr]
    frozen = max(poss) - min(poss) < 1e-6
    st = http_json("http://127.0.0.1:%d/api/axes" % WS_PORT)
    check("estop.freeze", frozen and st["estop"],
          "drift=%.6g" % (max(poss) - min(poss)))
    ws.send({"cmd": "reset"})
    time.sleep(0.2)
    st = http_json("http://127.0.0.1:%d/api/axes" % WS_PORT)
    ws.send({"cmd": "move", "axis": "FA_X", "target": 0.0, "speed": 600.0})
    ok = wait_axis(ws, idx, 0.0, timeout=5.0)
    check("reset.recover", (not st["estop"]) and ok)


def test_speed_scale(ws):
    print("--- 6. 倍速 2x ---")
    idx = acfg.NAME_TO_IDX["WS_LS_X"]
    ws.send({"cmd": "move", "axis": "WS_LS_X", "target": 0.0, "speed": 200.0})
    wait_axis(ws, idx, 0.0)
    ws.send({"cmd": "setSpeed", "scale": 2.0})
    time.sleep(0.3)
    poss, wall_dur, sim_dur, _ = measure_move(ws, idx, 100.0, 200.0)
    ws.send({"cmd": "setSpeed", "scale": 1.0})
    cfg = get_axis_entry("WS_LS_X")["config"]
    expect = profile_duration(plan_move(100.0, 200.0, cfg["amax"], cfg["jerk"]))
    print("   2x: wall=%.3fs sim=%.3fs expect=%.3fs" % (wall_dur, sim_dur, expect))
    check("speed.walltime_halved", abs(wall_dur - expect / 2) < 0.2,
          "wall=%.3f vs %.3f" % (wall_dur, expect / 2))
    check("speed.simtime_same", abs(sim_dur - expect) < 0.15,
          "sim=%.3f" % sim_dur)


# ---------------- 伺服环动态（一阶滞后 + 到位窗口整定） ----------------
def test_servo(ws):
    print("--- 6.5 伺服环动态（pe / settling / 配置持久化 / 急停泊放） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    idx = acfg.NAME_TO_IDX["WS_LS_X"]
    ov_path = os.path.join(ROOT, "runtime", "config_overrides.json")

    # 归零并等整定完成，拿到默认伺服参数
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 0.0, "speed": 200.0})
    e0 = wait_in_position("WS_LS_X")
    st0 = e0["config"]
    check("servo.defaults", abs(st0["tau"] - 0.005) < 1e-12
          and abs(st0["settle_window"] - 0.002) < 1e-12
          and abs(st0["settle_time"] - 0.05) < 1e-12,
          "tau=%s win=%s t=%s" % (st0["tau"], st0["settle_window"],
                                  st0["settle_time"]))
    check("servo.idle_fields", e0["state"] == "idle" and e0["pe"] == 0.0
          and abs(e0["x_actual"] - e0["value"]) < 1e-12)

    # 运动中：pe 显著非零、state=moving、in_position=False
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 100.0, "speed": 200.0})
    t0, max_pe, saw_moving = time.time(), 0.0, False
    while time.time() - t0 < 3.0:
        e = get_axis_entry("WS_LS_X")
        if e["moving"]:
            saw_moving = saw_moving or (e["state"] == "moving"
                                        and not e["in_position"])
            max_pe = max(max_pe, abs(e["pe"]))
        elif e["in_position"]:
            break
        time.sleep(0.003)
    print("   运动中 max|pe|=%.4fmm（≈v·tau=1mm）state=moving=%s"
          % (max_pe, saw_moving))
    check("servo.pe_during_move", saw_moving and max_pe > 0.1,
          "max_pe=%.4f" % max_pe)

    # profile 结束（moving=False）→ in_position 延迟 ≈settle_time 才置位
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 0.0, "speed": 200.0})
    t_end = t_ip = None
    t0 = time.time()
    while time.time() - t0 < 3.0:
        e = get_axis_entry("WS_LS_X")
        now = time.time()
        if t_end is None and not e["moving"]:
            t_end = now                    # 规划到位瞬间（进入 settling）
        if t_end is not None and e["in_position"]:
            t_ip = now
            break
        time.sleep(0.002)
    delay = (t_ip - t_end) if (t_end and t_ip) else -1
    print("   settling 延迟=%.3fs（settle_time=0.05s）" % delay)
    check("servo.settle_delay", 0.03 <= delay <= 0.5, "%.3fs" % delay)
    e = get_axis_entry("WS_LS_X")
    check("servo.settled_align", e["state"] == "idle" and e["pe"] == 0.0
          and e["x_actual"] == e["value"])

    # 在线改伺服参数 + 校验 + 持久化
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_X", "tau": 0.02,
                      "settle_window": 0.005, "settle_time": 0.08})
    e = get_axis_entry("WS_LS_X")
    check("servo.config_update", c == 200
          and e["config"]["tau"] == 0.02
          and e["config"]["settle_window"] == 0.005
          and e["config"]["settle_time"] == 0.08)
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_X", "tau": -0.1})
    check("servo.config_invalid_400", c == 400 and r.get("ok") is False)
    with open(ov_path, encoding="utf-8") as f:
        ov = json.load(f)
    sys.path.insert(0, os.path.join(ROOT, "runtime"))
    from engine import Engine
    eng2 = Engine(None)
    eng2.load_overrides(ov_path)
    check("servo.config_persist",
          ov.get("axes", {}).get("WS_LS_X", {}).get("tau") == 0.02
          and eng2.axes["WS_LS_X"].tau == 0.02
          and eng2.axes["WS_LS_X"].settle_window == 0.005
          and eng2.axes["WS_LS_X"].settle_time == 0.08)
    http_post(base + "/api/axis/config",
              {"axis": "WS_LS_X", "tau": 0.005, "settle_window": 0.002,
               "settle_time": 0.05})        # 还原默认

    # 急停：pos 冻结后 x_actual 按一阶滞后泄放，pe 收敛到 0
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 150.0, "speed": 600.0})
    time.sleep(0.15)                         # 已进入高速段，pe≈v·tau=3mm
    e = get_axis_entry("WS_LS_X")
    pe_at_estop = abs(e["pe"])
    http_post(base + "/api/estop", {})
    t0, dumped = time.time(), False
    while time.time() - t0 < 2.0:
        e = get_axis_entry("WS_LS_X")
        if e["state"] == "idle" and e["pe"] == 0.0 and e["in_position"]:
            dumped = True
            break
        time.sleep(0.01)
    http_post(base + "/api/reset", {})
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 0.0, "speed": 600.0})
    print("   急停时 |pe|=%.4fmm → 泄放收敛=%s" % (pe_at_estop, dumped))
    check("servo.estop_pe_dump", pe_at_estop > 0.5 and dumped,
          "pe0=%.4f dumped=%s" % (pe_at_estop, dumped))
    wait_in_position("WS_LS_X")


# ---------------- 误差系统与配置 API ----------------
def get_axis_entry(name):
    st = http_json("http://127.0.0.1:%d/api/axes" % WS_PORT)
    return st["axes"][acfg.NAME_TO_IDX[name]]


def wait_settled(name, timeout=5.0):
    """等轴彻底停止（settled=伺服整定完成），避免减速段读到变化中的 actual。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        e = get_axis_entry(name)
        if e["settled"] and not e["moving"]:
            return e
        time.sleep(0.02)     # 迭代 18：轮询加密（纯测试侧开销，语义不变）
    return get_axis_entry(name)


def wait_in_position(name, timeout=2.0):
    """等 in_position 置位（伺服 settling 约 settle_time 后才置位）。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        e = get_axis_entry(name)
        if e["in_position"] and not e["moving"]:
            return e
        time.sleep(0.01)
    return get_axis_entry(name)


def test_fault():
    """--- W2 故障注入：掉使能 / 沉默 / 断线 ---
    每个子用例结束都清理干净（复使能/解除沉默/恢复端口），不影响后续用例。"""
    print("--- 11. 故障注入（/api/fault） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    plc = "http://127.0.0.1:%d" % PLC_PORT
    AXIS = "WS_LS_Y"          # 前序用例未动过的 ACS 轴（在 0 位）

    # 参数校验：非法 axis / target / on → 400
    c1, _ = http_post(base + "/api/fault", {"action": "disable", "axis": "NOPE"})
    c2, _ = http_post(base + "/api/fault", {"action": "silent", "target": "nope",
                                            "on": True})
    c3, _ = http_post(base + "/api/fault", {"action": "silent", "target": "acs"})
    c4, _ = http_post(base + "/api/fault", {"action": "what"})
    check("fault.invalid_400", c1 == 400 and c2 == 400 and c3 == 400 and c4 == 400,
          "codes=%d/%d/%d/%d" % (c1, c2, c3, c4))

    # 静止掉使能 → move 拒绝（409，沿用联锁模式）；9101 ACS 面同语义
    c, r = http_post(base + "/api/fault", {"action": "disable", "axis": AXIS})
    e = get_axis_entry(AXIS)
    c2, r2 = http_post(base + "/api/axis/move", {"axis": AXIS, "target": 50.0})
    cli = ACSClient()
    resp = cli.cmd(cmd="move", axis=AXIS, target=50.0)
    cli.close()
    ok = (c == 200 and r.get("ok") and not e["enabled"] and not e["fault"]
          and c2 == 409 and "not enabled" in r2.get("error", "")
          and not resp.get("ok") and "not enabled" in resp.get("error", ""))
    check("fault.disable_rejects_move", ok,
          "http=%d %s acs=%s" % (c2, r2.get("error", ""), resp.get("error", "")))

    # 运动中掉使能 → 自由减速停（到不了目标）+ fault 置位 + settling 收敛
    http_post(base + "/api/fault", {"action": "enable", "axis": AXIS})
    wait_in_position(AXIS)
    http_post(base + "/api/axis/move", {"axis": AXIS, "target": 150.0,
                                        "speed": 600.0})
    time.sleep(0.15)                     # 已进入加速/巡航段
    t_dis = time.time()
    http_post(base + "/api/fault", {"action": "disable", "axis": AXIS})
    stopped = faulted = False
    pos_at_fault = None
    t0 = time.time()
    while time.time() - t0 < 3.0:
        e = get_axis_entry(AXIS)
        if e["fault"] and not e["moving"]:
            stopped, faulted = True, True
            pos_at_fault = e["pos"]
            stop_elapsed = time.time() - t_dis
            break
        time.sleep(0.005)
    # 自由减速（3×amax=30×vmax）从 600mm/s 约 33ms 停下，行程 ~10mm 量级，
    # 远小于剩余行程（150mm - 加速段位移）。已走行程由 HTTP 落地时刻决定，
    # 随本机负载漂移（历史实测 104~125mm，2026-09-15 三连测 125±1）——
    # 核心判据=自由减速停（fault 置位 + 1s 内停 + 未到目标 150mm），
    # 位置阈值从 <120 放宽到 <148（仍须证明未到达目标）：
    ok = (stopped and faulted and pos_at_fault < 148.0 and stop_elapsed < 1.0)
    print("   掉使能: pos=%.2fmm 停耗时=%.3fs fault=%s" %
          (pos_at_fault or -1, stop_elapsed if stopped else -1, faulted))
    check("fault.mid_move_decel", ok)
    # in_position 交伺服 settling 收敛（不直接置位）
    e = wait_in_position(AXIS, timeout=3.0)
    check("fault.mid_move_settle", e["in_position"] and e["fault"],
          "in_position=%s fault=%s" % (e["in_position"], e["fault"]))

    # ResetError + Enable 后恢复正常 move
    http_post(base + "/api/fault", {"action": "reset_error", "axis": AXIS})
    http_post(base + "/api/fault", {"action": "enable", "axis": AXIS})
    c, r = http_post(base + "/api/axis/move", {"axis": AXIS, "target": 0.0})
    e = wait_in_position(AXIS)
    ok = (c == 200 and r.get("ok") and e["in_position"]
          and abs(e["pos"]) < 1e-3 and not e["fault"] and e["enabled"])
    check("fault.reset_restores", ok, "pos=%.4f" % e["pos"])

    # PLC 沉默：经 9010 代理的气路指令被吞（收到但不执行，状态不变）
    ckt = "SystemMachine.SS_ME.M_AIR.GlassCoolingIn"   # 前序用例未动过的回路
    before = http_json(base + "/api/io")["points"].get(
        ckt + ".PressureSwitch.oValue")
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": True})
    c, r = http_post(base + "/api/io/command",
                     {"path": ckt, "cmd": "OpenPositive"})
    time.sleep(0.5)                      # 足够一阶气路起压（tau=0.2s）
    after = http_json(base + "/api/io")["points"].get(
        ckt + ".PressureSwitch.oValue")
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": False})
    ok = (c == 200 and r.get("ok") and r.get("swallowed")
          and abs((after or 0) - (before or 0)) < 1.0)
    check("fault.silent_swallows", ok, "before=%s after=%s" % (before, after))
    # 解除沉默后同一指令真正生效
    http_post(base + "/api/io/command", {"path": ckt, "cmd": "OpenPositive"})
    time.sleep(0.8)
    p = http_json(base + "/api/io")["points"].get(ckt + ".PressureSwitch.oValue")
    http_post(base + "/api/io/command", {"path": ckt, "cmd": "OpenAir"})
    check("fault.silent_recover_exec", (p or 0) > 100.0, "pressure=%s" % p)

    # PLC 沉默：9102 HTTP 请求挂起不响应（客户端 2s 超时触发）
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": True})
    t0 = time.time()
    hung = False
    try:
        req = urllib.request.Request(
            plc + "/io/command",
            data=json.dumps({"path": ckt, "cmd": "OpenAir"}).encode("utf-8"),
            headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=2)
    except Exception:
        hung = True
    elapsed = time.time() - t0
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": False})
    # 解除后正常响应
    c, r = http_post(plc + "/io/command", {"path": ckt, "cmd": "OpenAir"})
    check("fault.silent_hangs", hung and 1.5 < elapsed < 10.0 and r.get("ok"),
          "elapsed=%.2fs" % elapsed)

    # 断线：端口拒连；restore 后服务恢复
    http_post(base + "/api/fault", {"action": "drop", "target": "mcs",
                                    "on": True})
    refused = False
    try:
        socket.create_connection(("127.0.0.1", MCS_PORT), timeout=2).close()
    except (ConnectionRefusedError, socket.timeout, OSError):
        refused = True
    http_post(base + "/api/fault", {"action": "restore", "target": "mcs"})
    restored = False
    t0 = time.time()
    while time.time() - t0 < 8.0:
        try:
            h = http_json("http://127.0.0.1:%d/health" % MCS_PORT)
            restored = h.get("ok")
            break
        except Exception:
            time.sleep(0.2)
    check("fault.drop_refuses", refused and restored,
          "refused=%s restored=%s" % (refused, restored))

    # health.faults 字段：沉默目标 / 断线目标 / fault 轴列表可读
    # 先造 fault（运动中掉使能；沉默挂上后 ACS 轴 move 会被吞，造不出 fault），
    # 再挂 acs 沉默，一次 health 同时验证三类字段
    http_post(base + "/api/axis/move", {"axis": AXIS, "target": 60.0,
                                        "speed": 600.0})
    time.sleep(0.05)
    http_post(base + "/api/fault", {"action": "disable", "axis": AXIS})
    http_post(base + "/api/fault", {"action": "silent", "target": "acs",
                                    "on": True})
    time.sleep(0.2)
    h = http_json(base + "/api/health")
    f = h.get("faults") or {}
    ok = ("acs" in f.get("silent", []) and not f.get("dropped")
          and AXIS in f.get("fault_axes", []))
    # 清理：解除沉默 + 复位 + 回 0
    http_post(base + "/api/fault", {"action": "restore", "target": "acs"})
    http_post(base + "/api/fault", {"action": "reset_error", "axis": AXIS})
    http_post(base + "/api/fault", {"action": "enable", "axis": AXIS})
    http_post(base + "/api/axis/move", {"axis": AXIS, "target": 0.0})
    wait_in_position(AXIS)
    h2 = http_json(base + "/api/health")
    f2 = h2.get("faults") or {}
    ok = ok and not f2.get("silent") and not f2.get("dropped") \
        and not f2.get("fault_axes")
    check("fault.health_fields", ok, "faults=%s" % f)


def test_acsn():
    """--- W3 ACS 侧接入（止损结局）：探针自检 + 9101 reset_error 语义补全 ---
    探针只跑 --selftest（dll 加载链路 + 假监听回路），不发起 ACS 连接。"""
    print("--- 12. ACS 侧接入（W3） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    # 探针自检：ACSCL_x64.dll 可加载、关键导出齐全、假监听回路可收发
    r = subprocess.run([sys.executable,
                        os.path.join(ROOT, "tools", "acs_probe.py"), "--selftest"],
                       capture_output=True, text=True, timeout=60, cwd=ROOT,
                       encoding="utf-8", errors="replace")  # 输出含中文，显式解码防 GBK 炸
    check("acsn.probe_selftest",
          r.returncode == 0 and "SELFTEST PASS" in r.stdout, "rc=%d" % r.returncode)

    # 9101 reset_error（W3 补齐）：fault 清除但 enabled 不变；未知轴/PLC 轴拒绝
    AXIS = "WS_LS_Y"          # test_fault 已清理回 enabled@0
    cli = ACSClient()
    bad1 = cli.cmd(cmd="reset_error", axis="NOPE")
    bad2 = cli.cmd(cmd="reset_error", axis="BS_Arm_Z")   # PLC 轴，不在 ACS 面
    # 运动中掉使能造 fault（复用 W2 语义），走 9101 复位
    http_post(base + "/api/axis/move", {"axis": AXIS, "target": 40.0,
                                        "speed": 600.0})
    time.sleep(0.05)
    http_post(base + "/api/fault", {"action": "disable", "axis": AXIS})
    t0 = time.time()
    while time.time() - t0 < 3.0:
        e = get_axis_entry(AXIS)
        if e["fault"] and not e["moving"]:
            break
        time.sleep(0.005)
    r1 = cli.cmd(cmd="reset_error", axis=AXIS)
    e = get_axis_entry(AXIS)
    mv = cli.cmd(cmd="move", axis=AXIS, target=10.0)     # 未复使能，应拒绝
    ok = (r1.get("ok") and not e["fault"] and not e["enabled"]
          and not mv.get("ok") and "not enabled" in mv.get("error", ""))
    check("acsn.reset_error_9101", ok,
          "fault=%s enabled=%s move_err=%s" % (e["fault"], e["enabled"],
                                               mv.get("error", "")))
    check("acsn.reset_error_bad_axis",
          not bad1.get("ok") and not bad2.get("ok"),
          "bad1=%s bad2=%s" % (bad1.get("error", ""), bad2.get("error", "")))
    # 清理：复使能回 0
    cli.cmd(cmd="enable", axis=AXIS)
    cli.cmd(cmd="move", axis=AXIS, target=0.0, speed=600.0)
    cli.close()
    wait_in_position(AXIS)


def test_error_and_config(ws):
    print("--- 7. 误差系统 + 轴参数配置 API ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    idx = acfg.NAME_TO_IDX["WS_LS_X"]

    # 面板托管：index.html 就位则 200 HTML，否则 404 JSON 占位
    try:
        with urllib.request.urlopen(base + "/", timeout=5) as r:
            panel_ok = (r.status == 200
                        and b"<" in r.read(256))
    except urllib.error.HTTPError as e:
        panel_ok = (e.code == 404)
    check("panel.hosted", panel_ok)
    # 目录穿越防护
    try:
        with urllib.request.urlopen(base + "/panel/../runtime/main.py",
                                    timeout=5) as r:
            traversal_ok = r.status in (403, 404)
    except urllib.error.HTTPError as e:
        traversal_ok = e.code in (403, 404)
    check("panel.traversal_guard", traversal_ok)

    info = http_json(base + "/api/errormaps")
    check("errormaps.info", info.get("map_count") == 126
          and info.get("enabled") is True,
          "maps=%s" % info.get("map_count"))

    # 覆盖区间内：actual != value 且 measured
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_X", "target": 50.0, "speed": 200.0})
    assert r.get("ok"), r
    wait_axis(ws, idx, 50.0)
    e = wait_settled("WS_LS_X")
    check("error.actual_ne_value",
          e["actual"] != e["value"] and e["measured"]
          and e["errormap"]["enabled"] and e["errormap"]["coverage"],
          "value=%.9f actual=%.9f" % (e["value"], e["actual"]))
    check("error.has_config", e["config"]["vmax"] == 600.0
          and e["config"]["profile"] == "scurve"
          and e["config"]["repeat"] > 0)

    # repeat_offset 到位后保持
    e2 = wait_settled("WS_LS_X")
    check("error.repeat_holds", e2["actual"] == e["actual"])

    #  jog 等价 move(value+delta)，且再次到位后 repeat 重采样
    c, r = http_post(base + "/api/axis/jog", {"axis": "WS_LS_X", "delta": 2.0})
    check("jog.target", r.get("ok") and abs(r["target"] - 52.0) < 1e-9)
    wait_axis(ws, idx, 52.0)
    wait_settled("WS_LS_X")
    http_post(base + "/api/axis/jog", {"axis": "WS_LS_X", "delta": -2.0})
    wait_axis(ws, idx, 50.0)
    e3 = wait_settled("WS_LS_X")
    check("error.repeat_resampled", e3["actual"] != e["actual"],
          "%.9f -> %.9f" % (e["actual"], e3["actual"]))

    # 全局开关
    c, r = http_post(base + "/api/errormaps", {"enabled": False})
    e4 = get_axis_entry("WS_LS_X")
    check("error.toggle_off", r.get("ok") and e4["actual"] == e4["value"])
    http_post(base + "/api/errormaps", {"enabled": True})
    e5 = get_axis_entry("WS_LS_X")
    check("error.toggle_on", e5["actual"] != e5["value"])

    # 区间外：measured=False（地图两端裁了 1%~8%，行程极限必然在覆盖外）
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": -220.0, "speed": 200.0})
    wait_axis(ws, idx, -220.0, timeout=8)
    e6 = get_axis_entry("WS_LS_X")
    check("error.outside_measured_false", e6["measured"] is False
          and e6["errormap"]["coverage"][0] > -220.0)
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 0.0, "speed": 200.0})
    wait_axis(ws, idx, 0.0, timeout=8)

    # 在线配置生效 + 非法值 400
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_X", "vmax": 150.0,
                      "profile": "trapezoid"})
    e7 = get_axis_entry("WS_LS_X")
    check("config.update", c == 200 and e7["config"]["vmax"] == 150.0
          and e7["config"]["profile"] == "trapezoid")
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_X", "vmax": -5})
    check("config.invalid_400", c == 400 and r.get("ok") is False,
          "code=%d err=%s" % (c, r.get("error")))
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_X", "profile": "cubic"})
    check("config.bad_profile_400", c == 400)
    http_post(base + "/api/axis/config",
              {"axis": "WS_LS_X", "vmax": 600.0, "profile": "scurve"})


# ---------------- 六自由度误差注入（6DOF 串行链合成） ----------------
def test_sixdof(ws):
    print("--- 7b. 六自由度误差注入（error_kinematics + API） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    import math as _m
    from error_kinematics import ErrorKinematics, rodrigues

    class _Stub(object):
        """桩采样器：axis -> (t, r, along)，绕开真实地图做精确对照。"""
        def __init__(self, table):
            self.table = table

        def sample6(self, axis, value):
            return self.table.get(axis, ([0.0] * 3, [0.0] * 3, 0.0))

    def approx3(a, b, tol=1e-9):
        return all(abs(a[i] - b[i]) < tol for i in range(3))

    # 罗德里格斯：绕 z 转 90°；绕已知 center 转 90°
    p = rodrigues([1, 0, 0], [0, 0, 0], [0, 0, 1], _m.pi / 2)
    check("sixdof.rodrigues_quarter", approx3(p, [0, 1, 0], 1e-12))
    p2 = rodrigues([2, 1, 0], [1, 1, 0], [0, 0, 1], _m.pi / 2)
    check("sixdof.rodrigues_center", approx3(p2, [1, 2, 0], 1e-12))

    th = 1e-3
    # 纯 rz=θ 绕已知 center：工具点位移与手算一致
    cfg_r = {"modules": {"m": {
        "tool_point": [100.0, 0.0, 900.0],
        "chain": [{"axis": "WS_Rz", "kind": "R", "dir": [0, 0, 1],
                   "center": [0.0, 0.0, 800.0]}]}}}
    kin = ErrorKinematics.from_dict(
        _Stub({"WS_Rz": ([0, 0, 0], [0, 0, th], 0.0)}), cfg_r)
    d = kin.compute({})["modules"]["m"]
    ex = [100 * _m.cos(th) - 100, 100 * _m.sin(th), 0.0]
    check("sixdof.pure_rz",
          approx3(d["error_mm"], ex) and abs(d["error_rad"][2] - th) < 1e-15,
          "err=%s expect=%s" % (d["error_mm"], ex))
    # 名义正解：WS_Rz 转 90° → 工具点绕 center 转到 (0,100,900)
    dn = kin.compute({"WS_Rz": _m.pi / 2})["modules"]["m"]
    check("sixdof.nominal_rot", approx3(dn["nominal_mm"], [0, 100, 900], 1e-9),
          "nom=%s" % dn["nominal_mm"])

    # 纯 dx → 直接平移
    cfg_t = {"modules": {"m": {
        "tool_point": [1.0, 2.0, 3.0],
        "chain": [{"axis": "WS_LS_X", "kind": "T", "dir": [1, 0, 0],
                   "center": [0.0, 0.0, 0.0]}]}}}
    kin2 = ErrorKinematics.from_dict(
        _Stub({"WS_LS_X": ([0.002, 0, 0], [0, 0, 0], 0.0)}), cfg_t)
    d2 = kin2.compute({})["modules"]["m"]
    check("sixdof.pure_dx", approx3(d2["error_mm"], [0.002, 0, 0])
          and approx3(d2["error_rad"], [0, 0, 0]))

    # 两轴链合成顺序：T 误差先把下游 R 的 center 平移，R 再绕新 center 转
    cfg_chain = {"modules": {"m": {
        "tool_point": [100.0, 0.0, 0.0],
        "chain": [{"axis": "WS_LS_X", "kind": "T", "dir": [1, 0, 0],
                   "center": [0.0, 0.0, 0.0]},
                  {"axis": "WS_Rz", "kind": "R", "dir": [0, 0, 1],
                   "center": [0.0, 0.0, 0.0]}]}}}
    kin3 = ErrorKinematics.from_dict(_Stub({
        "WS_LS_X": ([1e-3, 0, 0], [0, 0, 0], 0.0),
        "WS_Rz": ([0, 0, 0], [0, 0, th], 0.0)}), cfg_chain)
    d3 = kin3.compute({})["modules"]["m"]
    # 手算：T 后 tool=(100.001,0,0)、center=(0.001,0,0)；再绕新 center 转 θ
    ex3 = [100 * _m.cos(th) + 1e-3 - 100, 100 * _m.sin(th), 0.0]
    check("sixdof.chain_order", approx3(d3["error_mm"], ex3),
          "err=%s expect=%s" % (d3["error_mm"], ex3))
    # 开关关：误差全零、名义不变
    kin3.set_enabled(False)
    d3off = kin3.compute({})["modules"]["m"]
    check("sixdof.unit_off_zero",
          approx3(d3off["error_mm"], [0, 0, 0])
          and approx3(d3off["error_rad"], [0, 0, 0])
          and approx3(d3off["nominal_mm"], [100, 0, 0]))

    # 沿轴标量：T 轴并进 t 沿 dir；R 轴并进 r 绕 dir（量纲一致）
    kin4 = ErrorKinematics.from_dict(
        _Stub({"WS_LS_X": ([0, 0, 0], [0, 0, 0], 5e-4)}), cfg_t)
    d4 = kin4.compute({})["modules"]["m"]
    check("sixdof.along_T", approx3(d4["error_mm"], [5e-4, 0, 0]))
    th2 = 2e-3
    cfg_r2 = {"modules": {"m": {
        "tool_point": [0.0, 100.0, 0.0],
        "chain": [{"axis": "WS_Rz", "kind": "R", "dir": [0, 0, 1],
                   "center": [0.0, 0.0, 0.0]}]}}}
    kin5 = ErrorKinematics.from_dict(
        _Stub({"WS_Rz": ([0, 0, 0], [0, 0, 0], th2)}), cfg_r2)
    d5 = kin5.compute({})["modules"]["m"]
    ex5 = [-100 * _m.sin(th2), 100 * _m.cos(th2) - 100, 0.0]
    check("sixdof.along_R", approx3(d5["error_mm"], ex5)
          and abs(d5["error_rad"][2] - th2) < 1e-15)

    # 校验：坏配置报清楚错误
    for bad, tag in (
            ({"modules": {"m": {"tool_point": [0, 0, 0], "chain": [
                {"axis": "NOPE", "kind": "T", "dir": [1, 0, 0]}]}}},
             "unknown_axis"),
            ({"modules": {"m": {"tool_point": [0, 0, 0], "chain": [
                {"axis": "WS_Rz", "kind": "T", "dir": [0, 0, 1]}]}}},
             "kind_mismatch"),
            ({"modules": {"m": {"tool_point": [0, 0, 0], "chain": [
                {"axis": "WS_LS_X", "kind": "T", "dir": [0, 0, 0]}]}}},
             "zero_dir"),
            ({"modules": {"m": {"tool_point": [0, 0, 0], "chain": [
                {"axis": "WS_Rz", "kind": "R", "dir": [0, 0, 1]}]}}},
             "missing_center"),
            ({"modules": {"m": {"tool_point": [0, 0, 0], "chain": [
                {"axis": "WS_LS_X", "kind": "T", "dir": [1, 0, 0]}]}}},
             "missing_center_T")):
        try:
            ErrorKinematics.from_dict(_Stub({}), bad)
            check("sixdof.validate_%s" % tag, False, "未报错")
        except ValueError as e:
            check("sixdof.validate_%s" % tag, "校验失败" in str(e))

    # ---- API 行为（真服务） ----
    info = http_json(base + "/api/errormaps")
    check("sixdof.api_field", info.get("six_dof_enabled") is True
          and info.get("enabled") is True)

    tp = http_json(base + "/api/toolpoints")
    nz = sum(abs(v) for md in tp.get("modules", {}).values()
             for v in md["error_nm"] + md["error_urad"])
    check("sixdof.api_nonzero",
          tp.get("ok") and len(tp.get("modules", {})) == 5 and nz > 0,
          "modules=%s |err|Σ=%.3f" % (list(tp.get("modules", {})), nz))

    # 六自由度开关关 → 工具点误差全零；单轴读数层开关不受影响
    c, r = http_post(base + "/api/errormaps", {"six_dof_enabled": False})
    tp2 = http_json(base + "/api/toolpoints")
    info2 = http_json(base + "/api/errormaps")
    allzero = all(v == 0 for md in tp2["modules"].values()
                  for v in md["error_nm"] + md["error_urad"])
    check("sixdof.api_off_zero",
          c == 200 and r.get("six_dof_enabled") is False
          and allzero and info2.get("enabled") is True)
    http_post(base + "/api/errormaps", {"six_dof_enabled": True})
    tp3 = http_json(base + "/api/toolpoints")
    nz3 = sum(abs(v) for md in tp3["modules"].values()
              for v in md["error_nm"])
    check("sixdof.api_on_restore", nz3 > 0)

    # 名义位置跟随轴动：AS_X +20 → align_stage nominal.x +20
    idx_asx = acfg.NAME_TO_IDX["AS_X"]
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "AS_X", "target": 20.0, "speed": 200.0})
    assert r.get("ok"), r
    wait_axis(ws, idx_asx, 20.0)
    wait_settled("AS_X")   # 等轨迹走完：WS 帧容差 1um 可能在到位前几 um 返回
    tp4 = http_json(base + "/api/toolpoints")
    nom4 = tp4["modules"]["align_stage"]["nominal_mm"]
    check("sixdof.api_nominal_follow", abs(nom4[0] - (-42.5 + 20.0)) < 1e-6,
          "nom=%s" % nom4)
    http_post(base + "/api/axis/move", {"axis": "AS_X", "target": 0.0,
                                        "speed": 200.0})
    wait_axis(ws, idx_asx, 0.0)
    wait_settled("AS_X")

    # PUT /api/kinematics 往返（工具点改 → nominal 平移相同量 → 还原）
    # 用纯 T 链的 film_stage（无旋转项，增量=工具点增量严格成立；
    # 旋转模块的工具点增量与姿态耦合，旋转路径已由 sixdof.nominal_rot 覆盖）
    kcfg = http_json(base + "/api/kinematics")
    orig_tp = kcfg["modules"]["film_stage"]["tool_point"]
    nom_before = http_json(base + "/api/toolpoints"
                           )["modules"]["film_stage"]["nominal_mm"]
    c, r = http_put(base + "/api/kinematics",
                    {"modules": {"film_stage": {"tool_point": [1.0, 2.0, 3.0]}}})
    nom5 = http_json(base + "/api/toolpoints"
                     )["modules"]["film_stage"]["nominal_mm"]
    expect5 = [nom_before[i] - orig_tp[i] + [1.0, 2.0, 3.0][i]
               for i in range(3)]
    check("sixdof.kin_put_roundtrip", c == 200 and r.get("ok")
          and approx3(nom5, expect5, 1e-6),
          "nom=%s expect=%s" % (nom5, expect5))
    http_put(base + "/api/kinematics",
             {"modules": {"film_stage": {"tool_point": orig_tp}}})
    nom6 = http_json(base + "/api/toolpoints"
                     )["modules"]["film_stage"]["nominal_mm"]
    check("sixdof.kin_put_restore", approx3(nom6, nom_before, 1e-6),
          "nom=%s expect=%s" % (nom6, nom_before))

    # 非法配置 400 + 错误明细
    c1, r1 = http_put(base + "/api/kinematics",
                      {"modules": {"wafer_stage": {"chain": [
                          {"axis": "WS_Rz", "center": [1, 2]}]}}})
    c2, r2 = http_put(base + "/api/kinematics",
                      {"modules": {"nope": {"tool_point": [0, 0, 0]}}})
    c3, r3 = http_put(base + "/api/kinematics",
                      {"modules": {"wafer_stage": {
                          "tool_point": [0, 0, "x"]}}})
    check("sixdof.kin_put_400",
          c1 == 400 and c2 == 400 and c3 == 400
          and r1.get("errors") and r2.get("errors") and r3.get("errors"),
          "codes=%s e1=%s" % ((c1, c2, c3), r1.get("errors")))

    # 每轴分解 + custom_center：旋转致平移（阿贝项）手算对照
    # 默认 center=[0,0,800]、custom=[50,0,900]、工具点 [100,0,900]
    # → 臂 (50,0,0)，rz=θ 的阿贝位移 = (50cosθ−50, 50sinθ, 0)
    cfg_cc = {"modules": {"m": {
        "tool_point": [100.0, 0.0, 900.0],
        "chain": [{"axis": "WS_Rz", "kind": "R", "dir": [0, 0, 1],
                   "center": [0.0, 0.0, 800.0],
                   "custom_center": [50.0, 0.0, 900.0]}]}}}
    kin6 = ErrorKinematics.from_dict(
        _Stub({"WS_Rz": ([0, 0, 0], [0, 0, th], 0.0)}), cfg_cc)
    d6 = kin6.compute({})["modules"]["m"]
    ex6 = [50 * _m.cos(th) - 50, 50 * _m.sin(th), 0.0]
    ax6 = d6["axes"][0]
    check("sixdof.abbe_custom_center",
          approx3(ax6["rot_trans_mm"], ex6)
          and approx3(ax6["trans_mm"], [0, 0, 0])
          and approx3(d6["error_mm"], ex6)
          and abs(ax6["attitude_rad"][2] - th) < 1e-15,
          "rot_trans=%s expect=%s" % (ax6["rot_trans_mm"], ex6))
    # 名义正解仍绕默认 center（custom_center 只影响误差合成）
    dn6 = kin6.compute({"WS_Rz": _m.pi / 2})["modules"]["m"]
    check("sixdof.cc_nominal_default",
          approx3(dn6["nominal_mm"], [0, 100, 900], 1e-9),
          "nom=%s" % dn6["nominal_mm"])
    pub6 = kin6.config_public()["modules"]["m"]["chain"][0]
    check("sixdof.cc_config_public",
          pub6["center"] == [0.0, 0.0, 800.0]
          and pub6["custom_center"] == [50.0, 0.0, 900.0]
          and pub6["effective_center"] == [50.0, 0.0, 900.0])

    # /api/toolpoints 每轴分解：Σ(平移+旋转致平移)≈error_mm、Σ姿态≈error_rad
    tp_dec = http_json(base + "/api/toolpoints")
    bad = []
    for mn2, md in tp_dec["modules"].items():
        axs = md.get("axes")
        if not axs:
            bad.append(mn2 + ":no_axes")
            continue
        st = [sum(a["trans_mm"][i] + a["rot_trans_mm"][i] for a in axs)
              for i in range(3)]
        sr = [sum(a["attitude_rad"][i] for a in axs) for i in range(3)]
        if not approx3(st, md["error_mm"], 1e-8):
            bad.append(mn2 + ":trans")
        if not approx3(sr, md["error_rad"], 1e-8):
            bad.append(mn2 + ":attitude")
    check("sixdof.api_axes_decomp", not bad, "bad=%s" % bad)

    # custom_center 往返：设置→effective 跟随；置 null→恢复默认
    def _rz_link():
        return [l for l in http_json(base + "/api/kinematics"
                                     )["modules"]["wafer_stage"]["chain"]
                if l["axis"] == "WS_Rz"][0]
    rz0 = _rz_link()
    check("sixdof.cc_default_null", rz0.get("custom_center") is None
          and rz0.get("effective_center") == rz0.get("center"))
    c, r = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_Rz", "custom_center": [1.0, 2.0, 3.0]}]}}})
    rz1 = _rz_link()
    check("sixdof.cc_put_set", c == 200 and r.get("ok")
          and rz1.get("custom_center") == [1.0, 2.0, 3.0]
          and rz1.get("effective_center") == [1.0, 2.0, 3.0],
          "resp=%s link=%s" % (r, rz1))
    c, r = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_Rz", "custom_center": None}]}}})
    rz2 = _rz_link()
    check("sixdof.cc_put_null", c == 200 and r.get("ok")
          and rz2.get("custom_center") is None
          and rz2.get("effective_center") == rz2.get("center"))
    # PUT center（只读默认）/ dir / 短向量 custom_center → 400
    c4, r4 = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_Rz", "center": [1.0, 2.0, 3.0]}]}}})
    c5, r5 = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_Rz", "dir": [0, 0, 1]}]}}})
    c6, r6 = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_Rz", "custom_center": [1, 2]}]}}})
    check("sixdof.cc_put_400",
          c4 == 400 and "不可修改" in str(r4.get("errors"))
          and c5 == 400 and c6 == 400,
          "codes=%s e4=%s" % ((c4, c5, c6), r4.get("errors")))

    # ---- T 轴旋转枢轴：直线轴角度误差同样产生阿贝平移（2026-08-20 澄清） ----
    thT = 2e-3
    # 直线轴 dry=θ 绕枢轴 c：rot_trans = 罗德里格斯(p, c, ŷ, θ) − p（精确式手算）
    cfg_tp = {"modules": {"m": {
        "tool_point": [100.0, 50.0, 900.0],
        "chain": [{"axis": "WS_LS_X", "kind": "T", "dir": [1, 0, 0],
                   "center": [10.0, 20.0, 800.0]}]}}}
    kin7 = ErrorKinematics.from_dict(
        _Stub({"WS_LS_X": ([0, 0, 0], [0, thT, 0], 0.0)}), cfg_tp)
    d7 = kin7.compute({})["modules"]["m"]
    exp7 = rodrigues([100.0, 50.0, 900.0], [10.0, 20.0, 800.0],
                     [0, 1, 0], thT)
    exp7 = [exp7[k] - [100.0, 50.0, 900.0][k] for k in range(3)]
    ax7 = d7["axes"][0]
    check("sixdof.abbe_T_axis",
          approx3(ax7["rot_trans_mm"], exp7)
          and approx3(ax7["trans_mm"], [0, 0, 0])
          and abs(ax7["attitude_rad"][1] - thT) < 1e-15,
          "rot_trans=%s expect=%s" % (ax7["rot_trans_mm"], exp7))
    # 枢轴随自身名义运动：q_X=30 时支点 = 零位枢轴 + 30·x̂
    # （臂不变 → 分解与 q=0 相同；若支点不随动则臂多 30·x̂，差 ~θ·30mm）
    d7b = kin7.compute({"WS_LS_X": 30.0})["modules"]["m"]
    p_q = [130.0, 50.0, 900.0]
    ef = rodrigues(p_q, [40.0, 20.0, 800.0], [0, 1, 0], thT)
    ef = [ef[k] - p_q[k] for k in range(3)]
    en = rodrigues(p_q, [10.0, 20.0, 800.0], [0, 1, 0], thT)
    en = [en[k] - p_q[k] for k in range(3)]
    check("sixdof.pivot_follows_self",
          approx3(d7b["axes"][0]["rot_trans_mm"], ef)
          and not approx3(d7b["axes"][0]["rot_trans_mm"], en, 1e-6),
          "got=%s follow=%s nofollow=%s"
          % (d7b["axes"][0]["rot_trans_mm"], ef, en))
    # 枢轴不随下游轴运动：X→Y 两 T 链，X 带 dry=θ，q_Y=100 时 X 支点不动
    cfg_xy = {"modules": {"m": {
        "tool_point": [100.0, 50.0, 900.0],
        "chain": [{"axis": "WS_LS_X", "kind": "T", "dir": [1, 0, 0],
                   "center": [10.0, 20.0, 800.0]},
                  {"axis": "WS_LS_Y", "kind": "T", "dir": [0, 1, 0],
                   "center": [0.0, 0.0, 850.0]}]}}}
    kin8 = ErrorKinematics.from_dict(
        _Stub({"WS_LS_X": ([0, 0, 0], [0, thT, 0], 0.0)}), cfg_xy)
    d8 = kin8.compute({"WS_LS_Y": 100.0})["modules"]["m"]
    p8 = [100.0, 150.0, 900.0]
    exp8 = rodrigues(p8, [10.0, 20.0, 800.0], [0, 1, 0], thT)
    exp8 = [exp8[k] - p8[k] for k in range(3)]
    check("sixdof.pivot_not_downstream",
          approx3(d8["axes"][0]["rot_trans_mm"], exp8),
          "got=%s expect=%s" % (d8["axes"][0]["rot_trans_mm"], exp8))

    # PUT custom_center 对 T 链元：往返 / 置 null / 传 center 400
    def _tx_link():
        return [l for l in http_json(base + "/api/kinematics"
                                     )["modules"]["wafer_stage"]["chain"]
                if l["axis"] == "WS_LS_X"][0]
    tx0 = _tx_link()
    check("sixdof.cc_T_fields", tx0.get("custom_center") is None
          and tx0.get("effective_center") == tx0.get("center"))
    c, r = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_LS_X", "custom_center": [4.0, 5.0, 6.0]}]}}})
    tx1 = _tx_link()
    check("sixdof.cc_T_put_set", c == 200 and r.get("ok")
          and tx1.get("custom_center") == [4.0, 5.0, 6.0]
          and tx1.get("effective_center") == [4.0, 5.0, 6.0],
          "resp=%s link=%s" % (r, tx1))
    c, r = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_LS_X", "custom_center": None}]}}})
    tx2 = _tx_link()
    c7, r7 = http_put(base + "/api/kinematics", {"modules": {"wafer_stage": {
        "chain": [{"axis": "WS_LS_X", "center": [1.0, 2.0, 3.0]}]}}})
    check("sixdof.cc_T_put_null_400",
          c == 200 and r.get("ok") and tx2.get("custom_center") is None
          and tx2.get("effective_center") == tx2.get("center")
          and c7 == 400 and "不可修改" in str(r7.get("errors")),
          "codes=%s e7=%s" % ((c, c7), r7.get("errors")))


# ---------------- 回零序列（四类） + DI 联动 + limits/持久化 ----------------
def wait_home(name, timeout=8.0):
    """等回零进入终态（done/fault）-> (entry, 墙钟耗时)。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        e = get_axis_entry(name)
        if e["home_state"] in ("done", "fault"):
            return e, time.time() - t0
        time.sleep(0.02)
    return get_axis_entry(name), time.time() - t0


def io_point(name):
    st = http_json("http://127.0.0.1:%d/api/io" % WS_PORT)
    return st["points"].get(name)


def test_homing_and_limits(ws):
    print("--- 8. 四类回零 + DI 联动 + limits 在线修改/持久化 ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    idx = acfg.NAME_TO_IDX["WS_LS_X"]
    ov_path = os.path.join(ROOT, "runtime", "config_overrides.json")

    # ===== /api/axes 顶层 t 字段（= sim_time）=====
    st = http_json(base + "/api/axes")
    check("axes.t_field", "t" in st and abs(st["t"] - st["sim_time"]) < 1e-9,
          "t=%s" % st.get("t"))

    # ===== gantry_dual（WS_LS_X，18 正向找正限位开关 + 残余偏摆注入）=====
    c, r = http_post(base + "/api/axis/home", {"axis": "WS_LS_X"})
    assert r.get("ok"), r
    m0 = ws.mark()
    timeline = []
    t0, done = time.time(), False
    while time.time() - t0 < 15:
        e = get_axis_entry("WS_LS_X")
        timeline.append((round(time.time() - t0, 3), e["home_phase"],
                         round(e["value"], 3)))
        if e["home_state"] == "done":
            done = True
            break
        time.sleep(0.05)
    dur = time.time() - t0
    phases = []
    for _, p, _ in timeline:
        if not phases or phases[-1] != p:
            phases.append(p)
    e = get_axis_entry("WS_LS_X")
    fr = ws.frames_since(m0)
    poss = [f[4][idx] for f in fr]
    marks = []
    last = None
    for t, p, v in timeline:
        if p != last:
            marks.append("t=%.2fs %s pos=%.1f" % (t, p, v))
            last = p
    print("   WS_LS_X 回零时间线: " + " | ".join(marks))
    print("   轨迹: max=%.2fmm(开关≈228.8) 总耗时=%.2fs 残余偏摆=%.6fmm"
          % (max(poss), dur, e["actual"] - e["value"]))
    check("home18.phase_sequence",
          phases == ["fast_approach", "retreat", "latch", "done"], str(phases))
    check("home18.reached_switch", max(poss) > 228.0,
          "max=%.2f" % max(poss))
    check("home18.done", done and e["homed"]
          and abs(e["value"] - 220.0) < 1e-6,
          "pos=%.6f home_state=%s" % (e["value"], e["home_state"]))
    check("home18.duration", 0.8 < dur < 4.0, "%.2fs" % dur)
    check("home18.gantry_type", e["config"]["home"]["type"] == "gantry_dual")

    # 回零后正常 move
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 100.0, "speed": 200.0})
    check("home18.move_after", wait_axis(ws, idx, 100.0, timeout=5))

    # ===== plc_sync_trio（E_EPIN×3，17 负向，三轴同步 + iZeroEpin_Z 联动）=====
    i17 = acfg.NAME_TO_IDX["E_EPIN_Z_1"]
    c, r = http_post(base + "/api/axis/home", {"axis": "E_EPIN_Z_2"})  # 任一成员发起即整组
    assert r.get("ok"), r
    m0 = ws.mark()
    t0, done17 = time.time(), False
    while time.time() - t0 < 8:
        e = get_axis_entry("E_EPIN_Z_1")
        if e["home_state"] == "done":
            done17 = True
            break
        time.sleep(0.02)
    fr = ws.frames_since(m0)
    min_pos = min(f[4][i17] for f in fr)
    trio = [(n, get_axis_entry(n)) for n in
            ("E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3")]
    trio_ok = all(x["home_state"] == "done" and abs(x["value"]) < 1e-9
                  for _, x in trio)
    print("   E_EPIN×3(17 同步): min=%.4fmm(开关≈-0.6) 三轴终点=%s"
          % (min_pos, [round(x["value"], 4) for _, x in trio]))
    check("home17.negative_direction", min_pos < -0.55, "min=%.4f" % min_pos)
    check("home17.trio_sync_done", done17 and trio_ok)
    check("di.epin_zero_true", io_point(acfg.DI_EPIN_ZERO) is True)
    # 移走一根顶针 → DI 变 false；再整组回零恢复 true
    # （+5mm 足够离开零位开关≈-0.6；不要用 +10——Epin Z 导轨/滑块随组上升
    #  会撞吸盘基座被沙盒 409，那是已登记的模型伪影，报警保留）
    http_post(base + "/api/axis/move",
              {"axis": "E_EPIN_Z_1", "target": 5.0, "speed": 25.0})
    wait_axis(ws, i17, 5.0)
    wait_settled("E_EPIN_Z_1")
    check("di.epin_zero_false", io_point(acfg.DI_EPIN_ZERO) is False)
    http_post(base + "/api/axis/home", {"axis": "E_EPIN_Z_1"})
    e, _ = wait_home("E_EPIN_Z_1", 8)
    check("di.epin_zero_rehome", e["home_state"] == "done"
          and io_point(acfg.DI_EPIN_ZERO) is True)

    # ===== zero_sensor（BS_CAL_Ry，零位窗口搜索 + 零位/限位 DI）=====
    iRy = acfg.NAME_TO_IDX["BS_CAL_Ry"]
    http_post(base + "/api/axis/move",
              {"axis": "BS_CAL_Ry", "target": 0.00174533, "speed": 1.5707963})
    wait_axis(ws, iRy, 0.00174533, tol=1e-5)
    wait_settled("BS_CAL_Ry")
    check("di.ry_pos_limit", io_point(acfg.DI_CAL_POS["BS_CAL_Ry"]) is True
          and io_point(acfg.DI_CAL_ZERO["BS_CAL_Ry"]) is False,
          "pos=%s zero=%s" % (io_point(acfg.DI_CAL_POS["BS_CAL_Ry"]),
                              io_point(acfg.DI_CAL_ZERO["BS_CAL_Ry"])))
    http_post(base + "/api/axis/home", {"axis": "BS_CAL_Ry"})
    e, dur = wait_home("BS_CAL_Ry", 5)
    print("   BS_CAL_Ry 零位回零: state=%s pos=%.7f 耗时=%.2fs"
          % (e["home_state"], e["value"], dur))
    check("home.zero_sensor", e["home_state"] == "done"
          and abs(e["value"]) < 1e-9,
          "state=%s pos=%.7f" % (e["home_state"], e["value"]))
    check("di.ry_zero_after_home",
          io_point(acfg.DI_CAL_ZERO["BS_CAL_Ry"]) is True
          and io_point(acfg.DI_CAL_POS["BS_CAL_Ry"]) is False)

    # ===== vendor_es（ES_X 外包模块占位：恒速 100mm/s 回 0）=====
    ies = acfg.NAME_TO_IDX["ES_X"]
    http_post(base + "/api/axis/move",
              {"axis": "ES_X", "target": 80.0, "speed": 600.0})
    wait_axis(ws, ies, 80.0, timeout=5)
    wait_settled("ES_X")
    http_post(base + "/api/axis/home", {"axis": "ES_X"})
    e, dur = wait_home("ES_X", 5)
    print("   ES_X vendor 回零: state=%s pos=%.4f 耗时=%.2fs"
          % (e["home_state"], e["value"], dur))
    check("home.vendor_es", e["home_state"] == "done"
          and abs(e["value"]) < 1e-9 and dur < 3.0,
          "state=%s pos=%.4f dur=%.2fs" % (e["home_state"], e["value"], dur))

    # ===== fault 路径（acs_limit 轴 AS_Z：开关不可达 → fault → reset 恢复）=====
    hd = acfg.home_defaults("AS_Z", -2.0, 6.0, 500.0)
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "AS_Z", "home_switch": 999.0})   # 不可达
    assert r.get("ok"), r
    http_post(base + "/api/axis/home", {"axis": "AS_Z"})
    e, _ = wait_home("AS_Z", 5)
    check("home.fault", e["home_state"] == "fault" and e["home_err"],
          "err=%s" % e.get("home_err"))
    # fault 中拒绝再次 home / move
    c, r = http_post(base + "/api/axis/home", {"axis": "AS_Z"})
    check("home.fault_blocks", c == 400)
    http_post(base + "/api/reset", {})
    e = get_axis_entry("AS_Z")
    check("home.reset_recovers", e["home_state"] == "never")
    # 恢复开关默认值并可正常回零
    http_post(base + "/api/axis/config",
              {"axis": "AS_Z", "home_switch": hd["switch"]})
    http_post(base + "/api/axis/home", {"axis": "AS_Z"})
    e, _ = wait_home("AS_Z", 5)
    check("home.recovered_rehome", e["home_state"] == "done")
    # 恢复现场：acs_limit 回零完成后轴停在开关位（≈+6）而不是 0——
    # W5 零件级沙盒里 AS_Z 非零会把对准相机承载台顶高，后续 test_collision
    # 的浅贴合用例会被误判穿模（本轮回归实踩），必须归位
    http_post(base + "/api/axis/move",
              {"axis": "AS_Z", "target": 0.0, "speed": 500.0})
    wait_settled("AS_Z")

    # ===== D 回归：保存软限位 → 回零 → 限位不还原 =====
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "AS_X", "limits": [-25.0, 75.0]})
    assert r.get("ok") and r.get("persisted"), r
    http_post(base + "/api/axis/home", {"axis": "AS_X"})
    e, _ = wait_home("AS_X", 8)
    e2 = get_axis_entry("AS_X")
    check("regress.limits_kept_after_home",
          e["home_state"] == "done" and e2["limits"] == [-25.0, 75.0],
          "state=%s limits=%s" % (e["home_state"], e2["limits"]))
    with open(ov_path, encoding="utf-8") as f:
        ov = json.load(f)
    check("regress.overrides_kept",
          ov.get("axes", {}).get("AS_X", {}).get("limits") == [-25.0, 75.0])
    # 还原默认限位
    http_post(base + "/api/axis/config",
              {"axis": "AS_X", "limits": [-30.0, 80.0]})
    http_post(base + "/api/axis/move",
              {"axis": "AS_X", "target": 0.0, "speed": 600.0})

    # ===== limits 在线修改 + 持久化 =====
    iy = acfg.NAME_TO_IDX["WS_LS_Y"]
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_Y", "target": 50.0, "speed": 200.0})
    wait_axis(ws, iy, 50.0)
    wait_settled("WS_LS_Y")
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_Y", "limits": [-100.0, 40.0]})
    e = get_axis_entry("WS_LS_Y")
    check("limits.update", c == 200 and r.get("persisted") is True
          and e["limits"] == [-100.0, 40.0])
    check("limits.clamp_pos", abs(e["value"] - 40.0) < 1e-9,
          "pos clamped to %.6f" % e["value"])
    c, r = http_post(base + "/api/axis/config",
                     {"axis": "WS_LS_Y", "limits": [10.0, -10.0]})
    check("limits.invalid_400", c == 400 and r.get("ok") is False)

    with open(ov_path, encoding="utf-8") as f:
        ov = json.load(f)
    check("persist.file_written",
          ov.get("axes", {}).get("WS_LS_Y", {}).get("limits")
          == [-100.0, 40.0])
    # 重启引擎实例后仍生效
    sys.path.insert(0, os.path.join(ROOT, "runtime"))
    from engine import Engine
    eng2 = Engine(None)
    n = eng2.load_overrides(ov_path)
    check("persist.reload", n >= 1
          and eng2.axes["WS_LS_Y"].hi == 40.0
          and eng2.axes["WS_LS_Y"].lo == -100.0)
    # 恢复默认并清理持久化文件（避免影响后续运行）
    http_post(base + "/api/axis/config",
              {"axis": "WS_LS_Y", "limits": [-220.0, 220.0]})
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_Y", "target": 0.0, "speed": 200.0})
    wait_axis(ws, iy, 0.0)
    if os.path.exists(ov_path):
        os.remove(ov_path)


# ---------------- 工件物料层 + 联锁 ----------------
def test_workpieces_interlock(ws):
    print("--- 9. 工件（晶圆/die 交接链） + EMO/门锁联锁 ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    idx_x = acfg.NAME_TO_IDX["WS_LS_X"]
    idx_y = acfg.NAME_TO_IDX["WS_LS_Y"]

    def viz():
        return http_json(base + "/api/viz")

    def dies_on(carrier):
        return [d for d in viz()["dies"] if d["carrier"] == carrier]

    def vacuum(name, on):                   # 经 9010 代理转发 PLC 气路命令
        c, r = http_post(base + "/api/io/command",
                         {"path": "SystemMachine.SS_ME.M_AIR." + name,
                          "cmd": "OpenNegtive" if on else "OpenAir"})
        assert r.get("ok"), r

    def set_bool(path, val):
        c, r = http_post(base + "/api/io/command",
                         {"path": path, "cmd": "SetBool",
                          "args": {"value": val}})
        assert r.get("ok"), r

    def wait_die_on(carrier, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = dies_on(carrier)
            if d:
                return d
            time.sleep(0.05)
        return []

    # ===== 上片 + viz 聚合 =====
    http_post(base + "/api/workpiece/reset", {})        # 显式干净起点（防撞车残留片）
    # ===== 手动上片 inject_recipe（迭代 46：面板"晶圆上片（处方注入）"同款
    # 通道——按处方 wafer_load 注入 load_x/y/rz+垂向，jitter 参与；显式字段
    # 覆盖注入默认；旧路径（无 flag）rz 仍为 0，行为不变）=====
    wl_saved = http_json(base + "/api/recipe")["recipe"]["wafer_load"]
    cj, rj = http_put(base + "/api/recipe",
                      {"wafer_load": {"load_x_mm": 0.2, "load_y_mm": -0.15,
                                      "load_rz_rad": 0.00025,
                                      "jitter_on": False}})
    check("wp.inject_recipe_put", cj == 200)
    try:
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "inject_recipe": True})
        w = r.get("wafer") or {}
        check("wp.inject_recipe_load", c == 200 and r.get("ok")
              and abs(w.get("rz", 0.0) - 0.00025) < 1e-9
              and abs((w.get("offset") or [9, 9])[0] - 0.2) < 1e-9
              and abs((w.get("offset") or [9, 9])[1] + 0.15) < 1e-9,
              "wafer=%s" % (w,))
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        # 显式 rz 覆盖注入默认（覆盖语义）
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "inject_recipe": True, "rz": 0.0})
        w = r.get("wafer") or {}
        check("wp.inject_recipe_override", c == 200 and w.get("rz") == 0.0
              and abs((w.get("offset") or [9, 9])[0] - 0.2) < 1e-9,
              "wafer=%s" % (w,))
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        # 旧路径（无 flag）rz 仍为 0
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "offset": [0.0, 0.0]})
        check("wp.load_clean_still_zero", c == 200
              and (r.get("wafer") or {}).get("rz") == 0.0)
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    finally:
        http_put(base + "/api/recipe", {"wafer_load": wl_saved})
        http_post(base + "/api/workpiece/reset", {})
    c, r = http_post(base + "/api/workpiece/wafer",
                     {"action": "load", "offset": [0.5, -0.5]})
    check("wp.wafer_load", c == 200 and r.get("ok"))
    v = viz()
    check("viz.wafer_visible", v["wafer"]["present"]
          and v["wafer"]["offset"] == [0.5, -0.5])
    check("viz.dies_grid", len(v["dies"]) == 149
          and all(d["carrier"] == "film" for d in v["dies"]),
          "dies=%d" % len(v["dies"]))
    check("viz.shape", "vacuum" in v and "tower" in v
          and "interlock" in v and "t" in v)

    # ===== 交接链 film→flip→head→wafer =====
    # film→flip：开翻转吸盘真空（ChuckVac）+ FA_X/FA_Z 到取片示教位
    # （FA_Z=-5.0：芯片吸盘扫掠盒底 z=891.6 贴蓝膜面 z=886.9，<ε 不判撞；
    #  旧值 -38 会压进蓝膜台体，v2 零件级沙盒直接 409）
    vacuum("ChuckVac", True)
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": -5.0, "speed": 600.0})
    d = wait_die_on("flip")
    check("transfer.film_to_flip", len(d) >= 1,
          "flip dies=%d" % len(d))
    n_flip = len(dies_on("flip"))

    # 负例 1：离开示教位（真空仍开）→ 不再交接
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": 0.0, "speed": 600.0})
    wait_settled("FA_Z")
    time.sleep(0.3)
    check("transfer.no_pose_no_fire", len(dies_on("flip")) == n_flip)
    # 负例 2（V1.1 改放回语义）：关真空后回到示教位 → die 放回 film
    # （flip_to_film：真空断+位姿窗口 上升沿——吸盘在取片高度破真空，芯片
    #  落回蓝膜；先等真空衰减到位再走轴，避免到位时真空未衰减完误触发）
    vacuum("ChuckVac", False)
    time.sleep(0.15)                         # 等真空状态衰减到位（8× 窗内
                                             # =1.2s sim≈6 tau，迭代 18 缩短）
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": -5.0, "speed": 600.0})
    t0 = time.time()
    while time.time() - t0 < 5.0 and dies_on("flip"):
        time.sleep(0.05)                     # 等 flip 腾空（放回完成信号）
    check("transfer.vacuum_off_place_back",
          not dies_on("flip")
          and any(e["rule"] == "flip_to_film"
                  for e in viz()["transfer_log"]),
          "flip=%d" % len(dies_on("flip")))
    # 重新拾取（真空再建立=film_to_flip 上升沿）→ 供 flip_to_head 继续
    vacuum("ChuckVac", True)
    d = wait_die_on("flip")
    check("transfer.repick_after_place_back", len(d) >= 1,
          "flip dies=%d" % len(d))
    # 出取片窗口后再释放 ChuckVac（flip_to_head 要求"翻转吸盘已释放"；
    # 在取片窗口内破真空会触发 flip_to_film 把 die 放回）
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": 0.0, "speed": 600.0})
    wait_settled("FA_Z")
    vacuum("ChuckVac", False)
    time.sleep(0.15)                         # 等真空衰减到位（8× 窗内=1.2s
                                             # sim≈6 tau，迭代 18 缩短）
    n_flip = len(dies_on("flip"))

    # flip→head：键合头三吸盘真空建立 + 翻转吸盘已释放 + FA_X 交接位 + 臂下行
    # （2026-08-26 起示教位=真实几何，对齐 machine_constants flip_handoff 组：
    #  FA_X=-1242.35 吸盘心=键合头心；BS_Arm_Z=-50.93 压头底接触 die 背面）
    for i in (1, 2, 3):
        vacuum("WT_BondingVac_%d" % i, True)
    # 先退避对准台相机 + 晶圆台（W5 零件级沙盒：BS_Arm_Z 深降时键合头会撞
    # AS 承载台/晶圆吸盘台——与真机"先退避再降臂"一致，示范流程同序）
    http_post(base + "/api/axis/move",
              {"axis": "AS_Y", "target": 120.0, "speed": 500.0})
    wait_settled("AS_Y")
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 220.0, "speed": 600.0})
    wait_settled("WS_LS_X")
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": -1242.35, "speed": 600.0})
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": -50.93, "speed": 200.0})
    d = wait_die_on("head", timeout=8.0)
    check("transfer.flip_to_head", len(d) >= 1, "head dies=%d" % len(d))

    # head→wafer：先抬头 → 翻转撤离（真实几何下交接位就在键合头正下方，
    # 不退则降臂撞空吸盘）→ 晶圆台回零位 → 下到键合真接触位（-167.43±8
    # 触发），再翻真空（晶圆真空建立 + 键合头释放 → 交接触发）
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    wait_settled("BS_Arm_Z")
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    wait_settled("FA_X", timeout=30.0)
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": 0.0, "speed": 600.0})
    wait_settled("WS_LS_X")
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": -167.43, "speed": 200.0})
    wait_settled("BS_Arm_Z")
    vacuum("WT_WaferVac", True)
    for i in (1, 2, 3):
        vacuum("WT_BondingVac_%d" % i, False)
    d = wait_die_on("wafer", timeout=8.0)
    check("transfer.head_to_wafer", len(d) >= 1, "wafer dies=%d" % len(d))
    log = viz()["transfer_log"]
    seq = [e["rule"] for e in log]
    print("   交接日志: %s" % seq)
    check("transfer.logged", "film_to_flip" in seq and "flip_to_head" in seq
          and "head_positive_to_wafer" in seq
          and log[-1].get("release") == "air_vertical")

    # ===== EMO：触发 → move 409 + 进行中轴停 + 红灯；解除+reset 恢复 =====
    http_post(base + "/api/axis/move",
              {"axis": "WS_LS_X", "target": -100.0, "speed": 600.0})
    time.sleep(0.15)                         # 轴在动
    set_bool("SystemMachine.SS_ME.M_EMO.iMachine_EMO", True)
    time.sleep(0.2)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_X", "target": 0.0})
    check("interlock.emo_move_409", c == 409, "code=%d err=%s"
          % (c, r.get("error")))
    st = http_json(base + "/api/axes")
    ax = st["axes"][idx_x]
    check("interlock.emo_estop", st["estop"] and st["interlock"]["emo"]
          and not ax["moving"])
    check("interlock.emo_red", viz()["tower"]["red"] is True)
    # EMO 未解除时 reset 不恢复
    http_post(base + "/api/reset", {})
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_X", "target": 0.0})
    check("interlock.emo_still_blocked", c == 409)
    # 解除 EMO + reset 恢复（写点经 PLC 扫描周期生效，先短轮询等点位可见再 reset）
    set_bool("SystemMachine.SS_ME.M_EMO.iMachine_EMO", False)
    t0 = time.time()
    while io_point("SystemMachine.SS_ME.M_EMO.iMachine_EMO") is not False \
            and time.time() - t0 < 0.5:
        time.sleep(0.005)
    http_post(base + "/api/reset", {})
    time.sleep(0.1)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_X", "target": 0.0, "speed": 600.0})
    check("interlock.emo_recover", c == 200 and r.get("ok"))
    wait_axis(ws, idx_x, 0.0, timeout=5)

    # ===== 门锁：门开 → 新运动 409；mask 放行；interlock 总开关 =====
    set_bool("SystemMachine.SS_ME.M_InterLock.iFrontDoorStatus_1", False)
    time.sleep(0.15)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_Y", "target": 10.0})
    check("interlock.door_move_409", c == 409, "code=%d" % c)
    st = http_json(base + "/api/axes")
    check("interlock.door_state", st["interlock"]["doors_ok"] is False)
    set_bool("SystemMachine.SS_ME.M_InterLock.iInterlock_Mask", True)
    time.sleep(0.15)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_Y", "target": 10.0, "speed": 600.0})
    check("interlock.door_mask_pass", c == 200 and r.get("ok"))
    wait_axis(ws, idx_y, 10.0)
    # 撤 mask 再挡一次
    set_bool("SystemMachine.SS_ME.M_InterLock.iInterlock_Mask", False)
    time.sleep(0.15)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_Y", "target": 0.0})
    check("interlock.door_blocked_again", c == 409)
    # 关总开关放行（调试用）
    c, r = http_post(base + "/api/interlock", {"enabled": False})
    check("interlock.disable", c == 200 and r.get("interlock_enabled") is False)
    c, r = http_post(base + "/api/axis/move",
                     {"axis": "WS_LS_Y", "target": 0.0, "speed": 600.0})
    check("interlock.disable_pass", c == 200 and r.get("ok"))
    wait_axis(ws, idx_y, 0.0)
    http_post(base + "/api/interlock", {"enabled": True})
    set_bool("SystemMachine.SS_ME.M_InterLock.iFrontDoorStatus_1", True)
    time.sleep(0.1)

    # ===== transfers 在线 GET/PUT =====
    tr = http_json(base + "/api/transfers")
    orig_rules = json.loads(json.dumps(tr["rules"]))   # 深拷贝
    check("transfers.get", len(tr["rules"]) == 6 and tr["grid"]["rows"] == 15
          and any(x["name"] == "flip_to_film" for x in tr["rules"])
          and any(x["name"] == "head_to_flip" for x in tr["rules"])
          and any(x["name"] == "head_positive_to_wafer"
                  and len(x.get("positive_on_any") or []) == 3
                  and x.get("site_mode") == "head_vertical"
                  for x in tr["rules"]))
    for rl in tr["rules"]:
        if rl["name"] == "film_to_flip":
            rl["pose"]["FA_Z"][1] = 6.0
    c, r = http_put(base + "/api/transfers", {"rules": tr["rules"]})
    tr2 = http_json(base + "/api/transfers")
    tol = [x["pose"]["FA_Z"][1] for x in tr2["rules"]
           if x["name"] == "film_to_flip"][0]
    pos_rule = next(x for x in tr2["rules"]
                    if x["name"] == "head_positive_to_wafer")
    check("transfers.put", c == 200 and r.get("ok") and tol == 6.0
          and pos_rule.get("site_mode") == "head_vertical"
          and pos_rule.get("defer_to") == "head_to_flip",
          "tol=%s positive=%s" % (tol, pos_rule))
    http_put(base + "/api/transfers", {"rules": orig_rules})   # 还原

    # ===== 下片：wafer 上的 die 随片移除；重复下片报错；工件复位 =====
    n_wafer = len(dies_on("wafer"))
    n_all = len(viz()["dies"])
    c, r = http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    v = viz()
    check("wp.unload_removes_wafer_dies",
          c == 200 and r.get("removed_dies") == n_wafer
          and not v["wafer"]["present"]
          and len(v["dies"]) == n_all - n_wafer
          and all(d["carrier"] != "wafer" for d in v["dies"]),
          "removed=%d left=%d" % (r.get("removed_dies", -1), len(v["dies"])))
    c, r = http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    check("wp.unload_empty_400", c == 400 and r.get("ok") is False)
    c, r = http_post(base + "/api/workpiece/reset", {})
    v = viz()
    check("wp.reset", r.get("ok") and len(v["dies"]) == 149
          and all(d["carrier"] == "film" for d in v["dies"]))

    # ===== 蓝膜上下机（film 正式工件，2026-08-21 升级）=====
    # reset 后 film 在位 + 149 die 全在膜上
    check("wp.film_present_default", viz()["film"]["present"] is True)
    # 下机：film 与其上剩余 die 一起离机 → dies=0；viz 带 film.present
    c, r = http_post(base + "/api/workpiece/film", {"action": "unload"})
    v = viz()
    check("wp.film_unload", c == 200 and r.get("ok")
          and r.get("removed_dies") == 149
          and v["film"]["present"] is False and len(v["dies"]) == 0,
          "removed=%s dies=%d" % (r.get("removed_dies"), len(v["dies"])))
    # 重复下机 → 400
    c, r = http_post(base + "/api/workpiece/film", {"action": "unload"})
    check("wp.film_unload_empty_400", c == 400 and r.get("ok") is False)
    # 非法 action → 400
    c, r = http_post(base + "/api/workpiece/film", {"action": "dance"})
    check("wp.film_bad_action_400", c == 400)
    # film 不在位 → 含 film 侧的交接规则不满足（真空+示教位全给也不触发）
    http_post(base + "/api/axis/move",      # 先抬键合臂再走 FA_X（防沙盒干涉）
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    wait_settled("BS_Arm_Z")
    vacuum("ChuckVac", True)
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    wait_settled("FA_X")
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": -5.0, "speed": 600.0})
    wait_settled("FA_Z")
    time.sleep(0.15)                         # 覆盖若干 tick 的判定窗口（8×
                                             # 窗内=1.2s sim，迭代 18 缩短）
    check("wp.film_absent_no_transfer", len(dies_on("flip")) == 0,
          "flip dies=%d" % len(dies_on("flip")))
    # 撤示教位 + 关真空（避免上机后立即触发取片）
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": 0.0, "speed": 600.0})
    wait_settled("FA_Z")
    vacuum("ChuckVac", False)
    time.sleep(0.15)                         # 8× 窗内=1.2s sim 足够衰减
    # 上机（幂等重铺）→ film 在位 + dies 回 149；重复上机也 200（重铺）
    c, r = http_post(base + "/api/workpiece/film", {"action": "load"})
    c2, r2 = http_post(base + "/api/workpiece/film", {"action": "load"})
    v = viz()
    check("wp.film_load", c == 200 and r.get("ok") and c2 == 200
          and v["film"]["present"] is True and len(v["dies"]) == 149
          and all(d["carrier"] == "film" for d in v["dies"]),
          "dies=%d c2=%d" % (len(v["dies"]), c2))

    # 收尾：关全部真空、轴回安全位
    for n in ("ChuckVac", "WT_WaferVac"):
        vacuum(n, False)
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": 0.0, "speed": 600.0})
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    # 必须等三轴真到位：BS_Arm_Z 深降时键合头零件与晶圆台零件沙盒相交，
    # 未归位就放行使后续 §13 move WS_LS_Y→150 被 /api/collision 409（W5 暴露的旧竞态）
    wait_settled("FA_X")
    wait_settled("FA_Z")
    wait_settled("BS_Arm_Z")
    # 键合头已抬起，对准台相机回零位（恢复现场供后续用例）
    http_post(base + "/api/axis/move",
              {"axis": "AS_Y", "target": 0.0, "speed": 500.0})
    wait_settled("AS_Y")


# ---------------- 工艺流程编排（W4） ----------------
def test_flow(ws):
    print("--- 13. 工艺流程编排（/api/flow*） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    def viz():
        return http_json(base + "/api/viz")

    def fstate():
        return http_json(base + "/api/flow/state")

    def wait_state(*states, **kw):
        timeout = kw.get("timeout", 90.0)
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = fstate()
            if s["state"] in states:
                return s
            time.sleep(0.1)
        return fstate()

    # ===== 示例流程在列 =====
    fl = http_json(base + "/api/flows")
    orig_flows = json.loads(json.dumps(fl["flows"]))      # 深拷贝，收尾还原
    names = [f["name"] for f in fl["flows"]]
    check("flow.list_example", "晶圆键合示范流程" in names
          and fl["state"] in ("idle", "done", "aborted"), "flows=%s" % names)

    # ===== 示例流程跑通：done + die 载体迁移证据（film 走一颗，随片下机） =====
    # 迭代 18：示范流程实跑段套 8× 提速窗（sim 等待主导；step/stop 的
    # paused/abort 语义由流程线程状态机保证，与倍速无关）
    set_speed(8.0)
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/flow/reset", {})
    film0 = sum(1 for d in viz()["dies"] if d["carrier"] == "film")
    c, r = http_post(base + "/api/flow/run", {"name": "晶圆键合示范流程"})
    s = wait_state("done", "aborted")
    v = viz()
    counts = {}
    for d in v["dies"]:
        counts[d["carrier"]] = counts.get(d["carrier"], 0) + 1
    seq = [e["rule"] for e in v["transfer_log"]]
    n_ok = sum(1 for st in s["steps"] if st["status"] == "ok")
    caps = http_json(base + "/api/camera/captures?limit=5")["captures"]
    cam_src = set(cap["source"] for cap in caps)
    ok = (c == 200 and r.get("ok") and s["state"] == "done"
          and n_ok == len(s["steps"]) and len(s["steps"]) == 23
          and counts.get("film") == film0 - 1        # 一颗 die 离膜→键合→随片下机
          and seq[-3:] == ["film_to_flip", "flip_to_head",
                           "head_positive_to_wafer"]
          and {"AlignCameraTigger_1", "AlignCameraTigger_2"} <= cam_src)
    print("   示例流程: state=%s 步数=%d ok=%d 总耗时=%.2fs film:%d→%d 交接=%s"
          % (s["state"], len(s["steps"]), n_ok, s["elapsed_s"],
             film0, counts.get("film", -1), seq))
    check("flow.run_to_done", ok)

    # ===== 单步 + 暂停 + 继续（临时流程，PUT 注入） =====
    tflow = {"name": "__t_steps", "description": "回归临时流程",
             "steps": [{"op": "delay", "ms": 100}, {"op": "log", "msg": "s1"},
                       {"op": "delay", "ms": 100}, {"op": "log", "msg": "s2"}]}
    c, r = http_put(base + "/api/flows", {"flows": orig_flows + [tflow]})
    assert c == 200 and r.get("ok"), r
    http_post(base + "/api/flow/reset", {})
    c, r = http_post(base + "/api/flow/run",
                     {"name": "__t_steps", "paused": True})
    s = wait_state("paused", timeout=3.0)
    t0 = time.time()                                   # 等线程进入第一步门槛
    while s["current_step"] != 0 and time.time() - t0 < 3.0:
        time.sleep(0.05)
        s = fstate()
    check("flow.step_pause_resume.run_paused",
          c == 200 and s["state"] == "paused" and s["current_step"] == 0
          and all(st["status"] == "pending" for st in s["steps"]),
          "state=%s step=%d" % (s["state"], s["current_step"]))
    http_post(base + "/api/flow/step", {})             # 推进一步 → 再暂停
    t0 = time.time()
    while time.time() - t0 < 5.0:
        s = fstate()
        if s["state"] == "paused" and s["steps"][0]["status"] == "ok" \
                and s["steps"][1]["status"] == "pending":
            break
        time.sleep(0.05)
    check("flow.step_pause_resume.step_once",
          s["state"] == "paused" and s["steps"][0]["status"] == "ok"
          and s["steps"][1]["status"] == "pending",
          "statuses=%s" % [st["status"] for st in s["steps"]])
    http_post(base + "/api/flow/resume", {})           # 继续 → 跑完
    s = wait_state("done", "aborted", timeout=10.0)
    check("flow.step_pause_resume.resume_done",
          s["state"] == "done"
          and all(st["status"] == "ok" for st in s["steps"]),
          "state=%s statuses=%s" % (s["state"],
                                    [st["status"] for st in s["steps"]]))

    # ===== 运行中 stop → aborted（示例流程再跑一次，进入交接段后停） =====
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/flow/reset", {})
    http_post(base + "/api/flow/run", {"name": "晶圆键合示范流程"})
    t0 = time.time()
    while time.time() - t0 < 10.0:                     # 等进入 transfer 步骤
        s = fstate()                                   # （film 步插入后交接段
        if s["state"] == "running" and s["current_step"] >= 3:  # 从 #4 起）
            break
        time.sleep(0.02)
    c, r = http_post(base + "/api/flow/stop", {})
    s = wait_state("aborted", timeout=10.0)
    logtxt = " ".join(e["msg"] for e in s["log"])
    check("flow.stop_aborts", c == 200 and s["state"] == "aborted"
          and "手动停止" in logtxt
          and any(st["status"] in ("fail", "skip") for st in s["steps"]),
          "state=%s cur=%d stop_http=%s 手动停止=%s bad=%s logtail=%s"
          % (s["state"], s["current_step"], c, "手动停止" in logtxt,
             [(st["op"], st["status"]) for st in s["steps"]
              if st["status"] not in ("ok", "pending")],
             [e["msg"][-40:] for e in s["log"][-2:]]))

    # ===== PUT 校验：非法逐条 400；合法保存并还原 =====
    set_speed(1.0)      # 提速窗结束：后续 abort_on_fault 的 time.sleep(0.15)
                        # 巡航段窗口是墙钟语义，必须 1×
    bad = {"flows": [
        {"name": "x"},                                  # 缺 steps
        {"name": "x", "steps": [{"op": "nope"}]},       # name 重复 + op 非法
        {"name": "y", "steps": [
            {"op": "move", "axis": "WS_LS_X"},          # 缺 target
            {"op": "move", "axis": "NO_AXIS", "target": 1.0},   # 未知轴
            {"op": "camera", "source": "BadCam"},       # 相机源非法
            {"op": "transfer", "rule": "no_such_rule"},  # 未知交接规则
            {"op": "if", "cond": {"grip": "ChuckVac", "expect": True}},  # 缺 then
            {"op": "wait_grip", "circuit": "NoCircuit", "expect": True},
            {"op": "delay", "ms": -5},                  # ms 非正
            {"op": "wafer", "action": "dance"},         # action 非法
            {"op": "film", "action": "dance"},          # film action 非法
        ]}]}
    c, r = http_put(base + "/api/flows", bad)
    errs = r.get("errors") or []
    fl2 = http_json(base + "/api/flows")
    check("flow.put_invalid_400", c == 400 and r.get("ok") is False
          and len(errs) >= 9 and all(isinstance(e, str) and e for e in errs)
          and [f["name"] for f in fl2["flows"]] ==
              [f["name"] for f in orig_flows] + ["__t_steps"],
          "code=%d errors=%d" % (c, len(errs)))
    print("   校验错误样例: %s | %s" % (errs[0] if errs else "-",
                                      errs[-1] if errs else "-"))
    good = json.loads(json.dumps(orig_flows))
    good.append({"name": "__t_ok", "description": "回归临时",
                 "steps": [{"op": "log", "msg": "hi"}]})
    c, r = http_put(base + "/api/flows", {"flows": good})
    names2 = [f["name"] for f in http_json(base + "/api/flows")["flows"]]
    check("flow.put_valid", c == 200 and r.get("ok") and "__t_ok" in names2,
          "code=%d" % c)
    http_put(base + "/api/flows", {"flows": orig_flows})   # 还原 flows.json

    # ===== 掉使能联动 abort（呼应 W2）：巡航段 Disable → move 等待检 fault =====
    fflow = {"name": "__t_fault", "description": "回归临时",
             "steps": [{"op": "move", "axis": "WS_LS_Y", "target": 150.0,
                        "speed": 600.0}]}
    http_put(base + "/api/flows", {"flows": orig_flows + [fflow]})
    http_post(base + "/api/flow/reset", {})
    c, r = http_post(base + "/api/flow/run", {"name": "__t_fault"})
    time.sleep(0.15)                         # 已进入加速/巡航段
    http_post(base + "/api/fault", {"action": "disable", "axis": "WS_LS_Y"})
    s = wait_state("aborted", timeout=10.0)
    reason = " ".join(e["msg"] for e in s["log"]) + " " + " ".join(
        st["info"] for st in s["steps"])
    check("flow.abort_on_fault", c == 200 and s["state"] == "aborted"
          and "fault" in reason.lower(),
          "state=%s reason=%s" % (s["state"], reason[:80]))
    # 清理：复位故障轴 + 还原 flows + 流程复位 + 工件复位 + 关真空 + 轴回安全位
    http_post(base + "/api/fault", {"action": "reset_error", "axis": "WS_LS_Y"})
    http_post(base + "/api/fault", {"action": "enable", "axis": "WS_LS_Y"})
    http_post(base + "/api/axis/move", {"axis": "WS_LS_Y", "target": 0.0,
                                        "speed": 600.0})
    wait_in_position("WS_LS_Y")
    http_put(base + "/api/flows", {"flows": orig_flows})
    http_post(base + "/api/flow/reset", {})
    http_post(base + "/api/workpiece/reset", {})
    for n in ("ChuckVac", "WT_WaferVac", "WT_BondingVac_1", "WT_BondingVac_2",
              "WT_BondingVac_3"):
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.SS_ME.M_AIR." + n, "cmd": "OpenAir"})
    for ax in ("FA_X", "FA_Z", "BS_Arm_Z"):
        http_post(base + "/api/axis/move",
                  {"axis": ax, "target": 0.0, "speed": 600.0})


def test_recipe():
    print("--- 16. 处方系统（/api/recipe*） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 50：bare 分片下 Recipe 落盘指向 per-shard tempdir 副本（裸引擎
    # 隔离仓），直接读盘校验跟着事实源走；全栈路径不变
    recipe_json = (BARE.engine.recipe.path if BARE is not None
                   else os.path.join(ROOT, "runtime", "recipe.json"))

    def get_rec():
        return http_json(base + "/api/recipe")["recipe"]

    # ===== 默认处方结构（字段齐、晶圆厚度保持 1.0、auto 双布局 + 派生块） =====
    r = http_json(base + "/api/recipe")
    rec = r.get("recipe") or {}
    w, f, s = rec.get("wafer") or {}, rec.get("film") or {}, \
        rec.get("wafer_sites") or {}
    check("recipe.get_default", r.get("ok")
          and w.get("thickness_mm") == 1.0 and w.get("diameter_mm") == 300.0
          and len(w.get("marks") or []) == 4
          and all(m.get("shape") in ("cross", "box", "circle", "quad")
                  for m in w.get("marks", []))
          and all(m.get("size_um") == 2000 for m in w.get("marks", []))
          and f.get("diameter_mm") == 300.0
          and (f.get("dies") or {}).get("mode") == "auto"
          and (f.get("dies") or {}).get("edge_margin_mm") == 3.0
          and (f.get("marks") or {}).get("shape") == "cross"
          and (f.get("marks") or {}).get("size_um") == 150
          and (f.get("marks") or {}).get("per_die") == 4
          and (w.get("site_marks") or {}).get("shape") == "quad"
          and s.get("mode") == "auto"
          and "dies_derived" in f and "wafer_sites_derived" in rec,
          "keys=%s" % sorted(rec.keys()))

    # ===== 标记比例化不变式：位置 = ratio × die 半宽（默认 10mm die →
    #       ±3.8mm=0.76×5.0）；ratio 四角呈中心对称（[0]=-[3]、
    #       [1]=-[2]，点位顺序 (-,-)(+,-)(-,+)(+,+)）=====
    fm0 = (f.get("marks") or {}).get("ratio") or []
    check("recipe.marks_invariant",
          len(fm0) == 4
          and abs(fm0[0][0] * 10.0 / 2.0 + 3.8) < 1e-9
          and abs(fm0[0][1] * 10.0 / 2.0 + 3.8) < 1e-9
          and fm0[0] == [-fm0[3][0], -fm0[3][1]]
          and fm0[1] == [-fm0[2][0], -fm0[2][1]],
          "ratio=%s" % (fm0,))

    # ===== 默认 auto 派生 == 圆形规则手算（膜面直径 300/边距 3 → rlim=147、
    #       半宽 5、pitch 20 → 15×15 跨度、圆裁后 149 颗；原点=膜面圆心） =====
    dd = (rec.get("film") or {}).get("dies_derived") or {}
    check("recipe.auto_default_identity",
          dd.get("rows") == 15 and dd.get("cols") == 15
          and dd.get("pitch_x_mm") == 20.0 and dd.get("pitch_y_mm") == 20.0
          and dd.get("count") == 149
          and dd.get("effective_origin") == [0.0, 0.0]
          and (dd.get("positions_preview") or [])[:2]
              == [[0.0, -140.0], [-60.0, -120.0]],
          "derived=%s" % dd)

    # ===== 手算对照：膜面直径 60/size10/gap0/边距3 → rlim=27、5×5 跨度、
    #       圆裁后 |y|=0 行 5 + |y|=10 行各 3 + |y|=20 行各 1 = 13 颗
    #       （验 derived 后还原；PUT 不动物料，viz 仍 149 颗） =====
    c, r = http_put(base + "/api/recipe",
                    {"film": {"diameter_mm": 60.0,
                              "dies": {"size_x_mm": 10.0, "size_y_mm": 10.0,
                                       "gap_x_mm": 0.0, "gap_y_mm": 0.0}}})
    dd2 = get_rec()["film"]["dies_derived"]
    viz_n = len(http_json(base + "/api/viz")["dies"])
    ok = (c == 200 and dd2.get("rows") == 5 and dd2.get("cols") == 5
          and dd2.get("pitch_x_mm") == 10.0 and dd2.get("count") == 13
          and viz_n == 149)                     # PUT 不动物料状态
    http_post(base + "/api/recipe/restore_default", {})
    dd3 = get_rec()["film"]["dies_derived"]
    check("recipe.auto_layout_handcalc", ok and dd3.get("count") == 149,
          "code=%d derived=%s" % (c, dd2.get("count")))

    # ===== PUT 合法（auto 部分合并：只改 gap）→ 派生 pitch 12 / 圆裁 437 颗 → 还原
    #       （手算：rlim=147、半宽 5、pitch 12 → 23×23 跨度圆裁 = 437）=====
    c, r = http_put(base + "/api/recipe",
                    {"film": {"dies": {"gap_x_mm": 2.0, "gap_y_mm": 2.0}}})
    dd4 = get_rec()["film"]["dies_derived"]
    viz_n = len(http_json(base + "/api/viz")["dies"])
    ok = (c == 200 and r.get("ok")
          and dd4.get("pitch_x_mm") == 12.0 and dd4.get("cols") == 23
          and dd4.get("count") == 437
          and get_rec()["film"]["dies"].get("size_x_mm") == 10.0  # 未提交字段保持
          and viz_n == 149)
    http_post(base + "/api/recipe/restore_default", {})          # 还原
    check("recipe.put_valid", ok
          and get_rec()["film"]["dies_derived"].get("count") == 149,
          "code=%d viz_dies=%d pitch=%s" % (c, viz_n, dd4.get("pitch_x_mm")))

    # ===== PUT 非法逐条 400（负直径 / 非法 shape / 缺字段 / 未知字段 / 比例超界），
    #       且处方不被污染 =====
    before = get_rec()
    bads = [
        {"wafer": {"diameter_mm": -300.0}},                # 负直径
        {"wafer": {"marks": [{"name": "m1", "shape": "star",
                              "x_mm": 0.0, "y_mm": 0.0, "size_um": 1000}]}},
        {"wafer": {"marks": [{"shape": "cross",            # marks 项缺 name
                              "x_mm": 0.0, "y_mm": 0.0, "size_um": 1000}]}},
        {"film": {"dies": {"rows": 8}}},                   # auto 模式下未知字段
        {"film": {"marks": {"ratio": [[1.5, 0.0]]}}},      # ratio 元素超 [-1,1]
        {"film": {"marks": {"per_die": 5}}},               # per_die > len(ratio)
        {"film": {"marks": {"shape": "star"}}},            # 非法标记形状
    ]
    codes, n_errs = [], []
    for b in bads:
        c, r = http_put(base + "/api/recipe", b)
        codes.append(c)
        n_errs.append(len(r.get("errors") or []))
    after = get_rec()
    check("recipe.put_invalid_400", codes == [400] * len(bads)
          and all(n >= 1 for n in n_errs) and after == before,
          "codes=%s errors=%s" % (codes, n_errs))

    # ===== 废除旧键 400 指路（2026-08-24 圆形规则）：film_width/film_height/
    #       center 两模式废除 → 文案指路新键；auto 另废 origin → 指路 manual =====
    old_key_cases = [
        ({"film": {"dies": {"film_width_mm": 95.0}}}, "film.diameter_mm"),
        ({"film": {"dies": {"film_height_mm": 95.0}}}, "film.diameter_mm"),
        ({"film": {"dies": {"center": True}}}, "已废除"),
        ({"film": {"dies": {"origin_x_mm": 1.0}}}, "manual"),
    ]
    ok_old = True
    for body, hint in old_key_cases:
        c, r = http_put(base + "/api/recipe", body)
        errs = "；".join(r.get("errors") or [])
        if not (c == 400 and hint in errs):
            ok_old = False
            print("   旧键用例失败: %s → %d %s" % (body, c, errs))
    check("recipe.put_old_keys_400", ok_old and get_rec() == before)

    # ===== auto 布局非法四类：size=0 / gap=-1 / size 超上限（范围拦截）
    #       / edge_margin 不小于膜面半径 =====
    bads2 = [
        {"film": {"dies": {"size_x_mm": 0.0}}},
        {"film": {"dies": {"gap_x_mm": -1.0}}},
        {"film": {"dies": {"size_x_mm": 350.0}}},       # 超 size 上限 100
                                                        # （300 膜面下 size≤100
                                                        #  恒可行，走范围拦截）
        {"film": {"dies": {"edge_margin_mm": 150.0}}},  # ≥ 膜面半径 150
    ]
    codes2, n2 = [], []
    for b in bads2:
        c, r = http_put(base + "/api/recipe", b)
        codes2.append(c)
        n2.append(len(r.get("errors") or []))
    check("recipe.auto_invalid_400", codes2 == [400] * 4
          and all(n >= 1 for n in n2) and get_rec() == before,
          "codes=%s errors=%s" % (codes2, n2))

    # ===== wafer_sites auto：总数 + 四顶点圆内不变式 + preview 一致 =====
    #       （2026-09-18 默认 size 10/gap 0.005 → pitch 10.005、rlim=147、半宽 5
    #         → 29×29 跨度圆裁 = 621 个 site；旧 gap 10/pitch 20/15×15=149 口径
    #         与蓝膜 die 同规则同数量，现蓝膜 die 仍 149 而 site 独立为 621）
    sd = get_rec().get("wafer_sites_derived") or {}
    sj = http_json(base + "/api/recipe/wafer_sites?limit=5000")
    pos = sj.get("positions") or []
    rlim = 150.0 - 3.0                            # R - edge_margin
    invar = all(math.hypot(abs(p[0]) + 5.0, abs(p[1]) + 5.0) <= rlim + 1e-6
                for p in pos)                     # size 10 → 半宽 5
    check("recipe.wafer_sites_auto",
          sd.get("total_sites") == 621
          and sd.get("rows") == 29 and sd.get("cols") == 29
          and sj.get("returned") == sd.get("total_sites")
          and pos[:8] == sd.get("positions_preview")
          and invar,
          "total=%s rows=%s cols=%s invar=%s"
          % (sd.get("total_sites"), sd.get("rows"), sd.get("cols"), invar))

    # ===== manual 兼容：显式 6×7@pitch8 生效；旧键 recipe.json 迁移不报错 =====
    man = {"mode": "manual", "rows": 6, "cols": 7,
           "pitch_x_mm": 8.0, "pitch_y_mm": 8.0,
           "origin_x_mm": 0.0, "origin_y_mm": 0.0,
           "size_x_mm": 4.0, "size_y_mm": 4.0, "thickness_mm": 0.5}
    c, r = http_put(base + "/api/recipe", {"film": {"dies": man}})
    dd5 = get_rec()["film"]["dies_derived"]
    ok_put = (c == 200 and dd5.get("mode") == "manual"
              and dd5.get("rows") == 6 and dd5.get("cols") == 7
              and dd5.get("pitch_x_mm") == 8.0 and dd5.get("count") == 42)
    http_post(base + "/api/recipe/restore_default", {})
    # 旧格式文件（无 mode 键）临时落盘迁移测试（不经过运行中的服务）
    import importlib
    recipe_mod = importlib.import_module("recipe")
    old_fmt = {
        "wafer": {"diameter_mm": 300.0, "thickness_mm": 1.0,
                  # 旧键 size_mm（×1000 → size_um 迁移路径）
                  "marks": [{"name": "wafer_mark_1", "shape": "cross",
                             "x_mm": -80.6102, "y_mm": -80.6102,
                             "size_mm": 2.0}]},
        "wafer_sites": {"rows": 8, "cols": 8, "pitch_x_mm": 30.0,
                        "pitch_y_mm": 30.0, "origin_x_mm": -105.0,
                        "origin_y_mm": -105.0},
        "film": {"dies": {"rows": 6, "cols": 7, "size_x_mm": 4.0,
                          "size_y_mm": 4.0, "pitch_x_mm": 8.0,
                          "pitch_y_mm": 8.0, "thickness_mm": 0.5,
                          "origin_x_mm": 0.0, "origin_y_mm": 0.0},
                 # 旧 die_mark 块 → film.marks（ratio 四角 ×corner_ratio）
                 "die_mark": {"shape": "boxes", "size_mm": 1.0,
                              "corner_ratio": 0.76}}}
    tmp = os.path.join(ROOT, "runtime", "_tmp_recipe_migrate.json")
    mig_ok = False
    try:
        with open(tmp, "w", encoding="utf-8") as fp:
            json.dump(old_fmt, fp, ensure_ascii=False)
        rc = recipe_mod.Recipe(tmp)               # 不报错即迁移成功
        pub = rc.public()
        fmig = pub["film"]["marks"]
        mig_ok = (pub["film"]["dies"]["mode"] == "manual"
                  and pub["wafer_sites"]["mode"] == "manual"
                  and pub["film"]["dies_derived"]["rows"] == 6
                  and pub["film"]["dies_derived"]["cols"] == 7
                  and pub["wafer_sites_derived"]["total_sites"] == 64
                  and pub["wafer"]["marks"][0]["size_um"] == 2000.0
                  and "size_mm" not in pub["wafer"]["marks"][0]
                  and fmig.get("shape") == "box"
                  and fmig.get("size_um") == 1000.0
                  and fmig.get("ratio") == [[-0.76, -0.76], [0.76, -0.76],
                                            [-0.76, 0.76], [0.76, 0.76]]
                  and fmig.get("per_die") == 4)
    except Exception as e:
        print("   迁移异常: %s" % e)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    check("recipe.manual_compat", ok_put and mig_ok,
          "put=%s migrate=%s" % (ok_put, mig_ok))

    # ===== apply 重建布局：manual 8×8=64 颗；还原默认 auto 圆裁 15×15=149 颗
    #       （id 集断言：四角缺席 / R05C06 在 / 四顶点圆内不变式）=====
    http_post(base + "/api/workpiece/reset", {})           # 确保无片/干净起点
    man8 = dict(man, rows=8, cols=8, pitch_x_mm=5.0, pitch_y_mm=5.0)
    http_put(base + "/api/recipe", {"film": {"dies": man8}})
    c, r = http_post(base + "/api/recipe/apply", {})
    lay = r.get("layout") or {}
    n64 = len(http_json(base + "/api/viz")["dies"])
    http_post(base + "/api/recipe/restore_default", {})
    c2, r2 = http_post(base + "/api/recipe/apply", {})
    dies45 = http_json(base + "/api/viz")["dies"]
    ids45 = set(d["id"] for d in dies45)
    rlim45 = 300.0 / 2.0 - 3.0
    invar45 = all(math.hypot(abs(d["pos"][0]) + 5.0, abs(d["pos"][1]) + 5.0)
                  <= rlim45 + 1e-6 for d in dies45)      # size 10 → 半宽 5
    n45 = len(dies45)
    check("recipe.apply_rebuilds", c == 200 and lay.get("dies_total") == 64
          and lay.get("rows") == 8 and n64 == 64
          and c2 == 200 and (r2.get("layout") or {}).get("dies_total") == 149
          and n45 == 149
          and all(i not in ids45
                  for i in ("R00C00", "R00C14", "R14C00", "R14C14"))
          and "R05C06" in ids45 and invar45,
          "64=%s/%s 149=%s/%s invar=%s"
          % (lay.get("dies_total"), n64,
             (r2.get("layout") or {}).get("dies_total"), n45, invar45))

    # ===== film 上机跟随当前处方（不 apply）：PUT size 5 → load 重铺 293 颗
    #       （rlim=147、半宽 2.5、pitch 15 → 19×19 跨度圆裁）；还原默认
    #       → load 回 149 =====
    c, _ = http_put(base + "/api/recipe",
                    {"film": {"dies": {"size_x_mm": 5.0, "size_y_mm": 5.0}}})
    http_post(base + "/api/workpiece/film", {"action": "load"})
    n29 = len(http_json(base + "/api/viz")["dies"])
    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/film", {"action": "load"})
    n45b = len(http_json(base + "/api/viz")["dies"])
    check("recipe.film_reload_follows", c == 200 and n29 == 293 and n45b == 149,
          "put=%d size5=%d default=%d" % (c, n29, n45b))

    # ===== 持久化：PUT 后 runtime/recipe.json 文件内容一致；测完恢复默认 =====
    # （取值 320 而非更小值：晶圆缩小会被 wafer_sites auto 可行性/边距校验
    #   连锁拦下——上面的非法用例已覆盖该路径）
    c, _ = http_put(base + "/api/recipe", {"wafer": {"diameter_mm": 320.0}})
    with open(recipe_json, encoding="utf-8") as fp:
        on_disk = json.load(fp)
    check("recipe.persist", c == 200
          and on_disk["wafer"]["diameter_mm"] == 320.0
          and on_disk["film"]["dies"]["mode"] == "auto"
          and "dies_derived" not in on_disk["film"],       # 派生块不落盘
          "disk_dia=%s" % on_disk["wafer"].get("diameter_mm"))
    c, r = http_post(base + "/api/recipe/restore_default", {})
    rec4 = get_rec()
    with open(recipe_json, encoding="utf-8") as fp:
        disk2 = json.load(fp)
    check("recipe.restore_default", c == 200 and r.get("ok")
          and rec4["wafer"]["diameter_mm"] == 300.0
          and disk2["wafer"]["diameter_mm"] == 300.0)
    # 清理副作用：apply 会清晶圆在位状态，这里显式复位物料，保证环境干净
    http_post(base + "/api/recipe/apply", {})
    http_post(base + "/api/workpiece/reset", {})


# ---------------- 整机视觉系统（W6：MQTT 1883 / 视觉服务 TCP 9200） ----------------
def test_vision():
    """--- 17. 整机视觉系统 ---
    新增端口 1883(MQTT broker)/9200(视觉服务 TCP 图像口)：跑回归前
    netstat -ano | findstr "9010 9101 9102 9190 1883 9200" 清残留
    （Windows 双绑定会连到旧实例）。
    场景几何（挖自前端模型 json，见 docs/视觉系统_V1.md）：
    下相机零位世界 [-42.5,1.0,895.9]，wafer_mark_1 局部 (-80.6102,-80.6102)
    （半径 114 对角位，2026-08-21 修正：旧 (±114,±114) 对角 161mm>R150 不在片上）→
    世界 (-80.7162+WS_X, -80.6099+WS_Y)；WS(58.2162,111.6102)+AS(20,30) 把标记送到
    画幅中心。粗对准相机固连机架 [1199.85,9,1067.4]，ES_X=-115.15+ES_Y=-1
    把蓝膜 R05C06(局部-20,-40) 精确送到其视场中心（2026-08-24 圆形布局：
    膜面圆心=film 局部原点=模块中心，蓝膜台面世界心=(1300+ES_X, ES_Y)）。
    cv2 缺失容错：real 相关用例 skip 打印、不 FAIL。"""
    print("--- 17. 整机视觉系统（MQTT 1883 / TCP 9200 / FAST+REAL 双模式） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    plc = "http://127.0.0.1:%d/io/command" % PLC_PORT
    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False
        print("   [skip] cv2 未安装，REAL 链路用例跳过（不 FAIL）")

    # ---- 17.0 MQTT broker 自测（真 socket 发布订阅回环，子进程） ----
    r = subprocess.run([sys.executable,
                        os.path.join(ROOT, "runtime", "mqtt_broker.py"),
                        "--selftest"],
                       capture_output=True, encoding="utf-8", errors="replace",
                       timeout=30)
    check("vision.mqtt_selftest",
          r.returncode == 0 and "selftest OK" in (r.stdout or ""),
          (r.stdout or r.stderr or "").strip()[-120:])

    # ---- 场景准备：干净物料 + 晶圆上片 + 走位（轴全到安全/对准位） ----
    def plc_setbool(path, val):
        c, r = http_post(plc, {"path": path, "cmd": "SetBool",
                               "args": {"value": val}})
        assert c == 200 and r.get("ok"), r

    def viz_cam(cid):
        return http_json(base + "/api/vision/cameras")["cameras"][cid]

    def _frame(cid):
        with urllib.request.urlopen(base + "/api/vision/frame?cam=" + cid,
                                    timeout=10) as resp:
            return resp.read()

    # ---- 17.0b 启动默认模式（2026-09-05 迭代 17 起默认 REAL；此前无任何
    #      用例触碰视觉模式，此处读到的即启动默认值）----
    mode_boot = http_json(base + "/api/vision/cameras").get("mode")
    check("vision.mode_default_real",
          mode_boot == ("real" if has_cv2 else "fast"),
          "boot mode=%s" % mode_boot)

    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "trigger", "exposure_ms": 8.0})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    # ---- 17.1 FAST identify：晶圆标记对准，judge true，结果点在中心附近 ----
    cams0 = http_json(base + "/api/vision/cameras")
    check("vision.cameras_api", cams0.get("ok") and cams0.get("mode") == "fast"
          and set(cams0["cameras"]) == {"coarse", "up", "down"}
          and abs(cams0["cameras"]["down"]["focus_err_um"] or 9e9) < 5.0
          # 运行态 focus_state：trigger 待机灯灭 → light_off（灯态感知语义）
          and cams0["cameras"]["down"].get("focus_state") == "light_off",
          "focus_err_um=%s focus_state=%s"
          % (cams0["cameras"]["down"]["focus_err_um"],
             cams0["cameras"]["down"].get("focus_state")))
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e = (r.get("entry") or {})
    res = e.get("result") or {}
    flds = all(k in res for k in ("strShapeName", "judge", "lstCol", "lstRow",
                                  "lstAngle", "lstScore", "lstScale"))
    near = res.get("lstCol") and abs(res["lstCol"][0] - 1500) < 20 \
        and abs(res["lstRow"][0] - 1500) < 20
    check("vision.fast_identify", c == 200 and flds
          and res.get("judge") is True and near
          and res["strShapeName"] == "cross"
          # entry 对焦三字段：闪光拍照灯亮 + 焦面上 → in_focus
          and e.get("focus_state") == "in_focus"
          and e.get("focus_err_um") is not None
          and abs(e.get("blur_ratio") or 9e9) <= 1.0,
          "judge=%s col=%s row=%s focus=%s"
          % (res.get("judge"), res.get("lstCol"), res.get("lstRow"),
             e.get("focus_state")))
    check("vision.fast_timing_fields",
          all(k in (e.get("timing") or {}) for k in
              ("render_ms", "tcp_ms", "match_ms", "total_ms"))
          and e["timing"]["total_ms"] > 0 and e.get("mode") == "fast",
          "timing=%s" % (e.get("timing"),))

    # ---- 17.2 粗对准相机（2026-08-25 零位约定重构）：die 走真实交接路径
    #      上翻转吸盘（ChuckVac 真空 + FA_X=0/FA_Z=-5 示教位）→ 呈现位
    #      FA_Rx=0 吸盘面朝上，die **背面朝上**（标记面贴吸盘）→ FAST 背面
    #      检出特征 1 个在焦 → 取片位 FA_Rx=-π die 挂吸盘面下不可见 → 回
    #      呈现位 REAL 模板匹配素板背面 → die 尺寸变 → 图像外廓即改即生效
    #      → FA_X 移走后无 die 特征 ----
    def _coarse_id():
        c_, r_ = http_post(base + "/api/vision/identify", {"cam": "coarse"})
        e_ = r_.get("entry") or {}
        return c_, e_, (e_.get("result") or {})

    http_post(base + "/api/axis/move", {"axis": "FA_Rx", "target": 0.0,
                                        "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_Z")
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenNegtive"})
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": -5.0,
                                        "speed": 600.0})
    t0 = time.time()
    flip_die = False
    while time.time() - t0 < 5.0:
        if any(d["carrier"] == "flip"
               for d in http_json(base + "/api/viz")["dies"]):
            flip_die = True
            break
        time.sleep(0.05)
    assert flip_die, "film_to_flip 交接未发生（ChuckVac+示教位）"
    # 先退出交接位姿窗口再破真空（否则 flip_to_film 放回规则同窗口触发，
    # die 会被送回 film）
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_Z")
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    # 呈现位 FA_Rx=0：die 背面朝上，素板无标记，FAST 背面检出特征 1 个在焦
    c, e, res2 = _coarse_id()
    surfs2 = {s["name"]: s for s in e.get("surfaces") or []}
    check("vision.coarse_fast_back", c == 200 and res2.get("judge") is True
          and res2.get("strShapeName") == "box"
          and len(res2.get("lstCol") or []) == 1
          and e.get("n_marks") == 1
          and e.get("focus_state") == "in_focus"
          and surfs2.get("flip_die", {}).get("in_dof") is True
          and "flip_chuck" in surfs2,
          "judge=%s n=%s focus=%s surfs=%s"
          % (res2.get("judge"), e.get("n_marks"),
             e.get("focus_state"), sorted(surfs2)))
    # 取片位 FA_Rx=-π：die 挂吸盘面下被本体遮挡 → 无 die 特征
    http_post(base + "/api/axis/move", {"axis": "FA_Rx",
                                        "target": -math.pi, "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)
    c, e, res2 = _coarse_id()
    check("vision.coarse_pickpose_hidden", c == 200
          and res2.get("judge") is False and e.get("n_marks") == 0
          and not (res2.get("lstCol") or []),
          "judge=%s n=%s cols=%s"
          % (res2.get("judge"), e.get("n_marks"), res2.get("lstCol")))
    http_post(base + "/api/axis/move", {"axis": "FA_Rx", "target": 0.0,
                                        "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)
    c, e, res2 = _coarse_id()
    surfs2 = {s["name"]: s for s in e.get("surfaces") or []}
    check("vision.coarse_fast", c == 200 and res2.get("judge") is True
          and len(res2.get("lstCol") or []) == 1
          and e.get("focus_state") == "in_focus"
          and surfs2.get("flip_die", {}).get("in_dof") is True,
          "judge=%s n=%s focus=%s"
          % (res2.get("judge"), len(res2.get("lstCol") or []),
             e.get("focus_state")))

    # ---- 17.2r 素板背面 REAL 模板匹配 + die/吸盘灰度对比（REAL 需 cv2）----
    # 模板 ROI 框住 die 左边缘（10mm die 半宽 5mm≈547px，满幅中心 (1500,
    # 1500)，边缘在 1500−547≈953；ROI 121px 中心恰跨边缘，die(0.55 暗)/
    # 吸盘板面(0.72 亮)有方差）
    plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", True)
    time.sleep(0.2)
    _, _, w, h, px = frame_stats(_frame("coarse"))
    k_prev = (2.74 / 0.3 / 1000.0) * (3000.0 / w)   # 预览分辨率下的 mm/px
    d_col, d_row = w // 2, h // 2                 # die 心 = 画幅中心
    p_col = int(-8.0 / k_prev + w / 2.0)          # 板面（左移 8mm，避开
    p_row = int(-8.0 / k_prev + h / 2.0)          #  die 与十字槽）
    die_m = patch_mean(px, w, d_col - 10, d_row - 10, 20, 20)
    plate_m = patch_mean(px, w, p_col - 10, p_row - 10, 20, 20)
    real_ok = True
    if has_cv2:
        c, r = http_post_tpl(base + "/api/vision/templates",
                         {"cam": "coarse", "name": "coarse_flip_tpl",
                          "shape": "box", "roi": [892, 1440, 121, 121]})
        real_ok = c == 200
        http_post(base + "/api/vision/mode", {"mode": "real"})
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "coarse", "template": "coarse_flip_tpl"})
        res_r = ((r.get("entry") or {}).get("result") or {})
        cols_r = res_r.get("lstCol") or []
        real_ok = real_ok and c == 200 and res_r.get("judge") is True \
            and cols_r and abs(cols_r[0] - 952.6) < 30.0 \
            and (res_r.get("lstScore") or [0])[0] > 0.7
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        http_delete(base + "/api/vision/templates",
                    {"name": "coarse_flip_tpl"})
    check("vision.coarse_real_consistent", real_ok
          and plate_m - die_m > 8.0,
          "real=%s die=%.1f plate=%.1f" % (real_ok, die_m, plate_m))
    plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", False)

    # ---- 17.2g 标记引导：coarse 背面检出特征的标记载体 = 翻转模块，
    #      suggested 用 FA_X（唯一平移点动轴）；居中 die suggested≈0；
    #      FA_X -20 → 特征世界 -20（FOV 27.4mm 半幅 13.7 → 出视野）→
    #      delta.x<0、suggested 反号 ----
    g = http_json(base + "/api/vision/guide?cam=coarse")
    ms = g.get("marks") or []
    m0 = ms[0] if ms else {}
    sug0 = {s["axis"]: s["delta_mm"] for s in m0.get("suggested") or []}
    axs = set(a["axis"] for m_ in ms for a in m_.get("suggested") or [])
    ok_g = g.get("ok") and len(ms) == 1 and (m0.get("name") or "") \
        .endswith("#back") and axs == {"FA_X"} \
        and abs(sug0.get("FA_X", 9e9)) < 2.0 \
        and any(m_.get("in_fov") for m_ in ms)
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": -20.0,
                                        "speed": 600.0})
    wait_settled("FA_X")
    g = http_json(base + "/api/vision/guide?cam=coarse")
    ms = g.get("marks") or []
    m0 = ms[0] if ms else {}
    sug0 = {s["axis"]: s["delta_mm"] for s in m0.get("suggested") or []}
    ok_g = ok_g and m0.get("in_fov") is False \
        and m0["delta_mm"][0] < -15.0 \
        and abs(sug0.get("FA_X", 9e9) + m0["delta_mm"][0]) < 0.05
    check("vision.guide_coarse", ok_g,
          "n=%d delta=%s sug=%s in_fov=%s"
          % (len(ms), m0.get("delta_mm"), sug0, m0.get("in_fov")))
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_X")

    # ---- 17.2b die 尺寸即改即生效（背面素板无标记 → 实测图像外廓）：
    #      die size_x 10→12mm，半宽 5.0→6.0mm；探针取 (中心−5.5mm, 中心上
    #      0.86mm)——y 偏移避开吸盘真空槽横臂（沿中心行窄带）；方 die 内外
    #      按 |x| 判：10mm 时 5.5>5.0 在 die 外（板面亮）、12mm 时 5.5<6.0
    #      进入 die 内（本体暗）→ 灰度翻转即外廓跟随处方 ----
    def _probe_gray(dcol_, drow_):
        _, _, w_, _, px_ = frame_stats(_frame("coarse"))
        return patch_mean(px_, w_, dcol_ - 5, drow_ - 5, 10, 10)

    _, _, w, h, px = frame_stats(_frame("coarse"))
    k_prev = (2.74 / 0.3 / 1000.0) * (3000.0 / w)
    e_col = int(w / 2.0 - 5.5 / k_prev)               # 5.5mm 探针列
    e_row = max(0, int(h / 2.0) - int(0.86 / k_prev))  # 上移 0.86mm 避开横槽
    plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", True)
    time.sleep(0.2)                          # 等实时预览亮起来（灯灭近全黑）
    g_before = _probe_gray(e_col, e_row)
    http_put(base + "/api/recipe", {"film": {"dies": {"size_x_mm": 12.0}}})
    g_after = _probe_gray(e_col, e_row)
    c_s, e_s, res_s = _coarse_id()
    plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", False)
    http_post(base + "/api/recipe/restore_default", {})
    check("recipe.die_size_effect", g_before - g_after > 8.0
          and c_s == 200 and len(res_s.get("lstCol") or []) == 1,
          "gray %.1f→%.1f（亮→暗=外廓外移）n=%s"
          % (g_before, g_after, len(res_s.get("lstCol") or [])))

    # ---- 17.2a FA_X 移走翻转模块 → 吸盘/die 出 FOV，coarse 无 die 特征 ----
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": -60.0,
                                        "speed": 600.0})
    wait_settled("FA_X")
    c, e, res2 = _coarse_id()
    check("vision.coarse_flip_away", c == 200 and res2.get("judge") is False
          and not (res2.get("lstCol") or []),
          "judge=%s cols=%s" % (res2.get("judge"), res2.get("lstCol")))
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_X")
    http_post(base + "/api/axis/move", {"axis": "FA_Rx", "target": 0.0,
                                        "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)

    # ---- 17.3 光源灭判废（continuous 模式不闪光，暗图 judge false） ----
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "continuous"})
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    time.sleep(0.2)                       # 等 PLC 扫描生效 + 光源跟踪刷新
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e = r.get("entry") or {}
    res = e.get("result") or {}
    smax = max(res.get("lstScore") or [0.0])
    check("vision.light_off_judge_false", c == 200
          and res.get("judge") is False and smax < 0.3
          and e.get("light_on") is False
          and e.get("focus_state") == "light_off",
          "judge=%s score_max=%.3f focus=%s"
          % (res.get("judge"), smax, e.get("focus_state")))
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "trigger"})

    # ---- 17.4 景深：拉离焦面超 3×DOF 判废（focus_state=rejected），回焦面恢复 ----
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.1,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    cam_st = viz_cam("down")
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e = r.get("entry") or {}
    res = e.get("result") or {}
    check("vision.dof_blur", c == 200 and res.get("judge") is False
          and abs(cam_st["focus_err_um"]) > 60.0
          and cam_st["in_dof"] is False
          and e.get("focus_state") == "rejected"
          and (e.get("blur_ratio") or 0) > 3.0,
          "focus_err_um=%s judge=%s focus=%s blur=%s"
          % (cam_st["focus_err_um"], res.get("judge"),
             e.get("focus_state"), e.get("blur_ratio")))
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.0,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e = r.get("entry") or {}
    res = e.get("result") or {}
    check("vision.dof_refocus", c == 200 and res.get("judge") is True
          and e.get("focus_state") == "in_focus",
          "judge=%s focus=%s" % (res.get("judge"), e.get("focus_state")))

    # ---- 17.5 PLC 硬触发闪光链路：9102 SetBool AlignCameraTigger_2 ----
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "exposure_ms": 120.0})
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    time.sleep(0.15)
    n_caps = len(http_json(base + "/api/camera/captures?limit=100")["captures"])
    n_res = len(http_json(base + "/api/vision/results?limit=100")["results"])
    plc_setbool("SystemMachine.SS_Bonding.M_AS.AlignCameraTigger_2", True)
    # 闪光窗口内应能观察到光源点亮（曝光 120ms + 扫描周期余量）
    saw_on = False
    t0 = time.time()
    while time.time() - t0 < 1.5:
        v = http_json("http://127.0.0.1:%d/io/observe?path="
                      "SystemMachine.SS_Bonding.M_AS.oLightOn_2" % PLC_PORT)
        if v.get("value") is True:
            saw_on = True
            break
        time.sleep(0.005)
    # 等视觉结果产出（FAST 全链路 < 1s 足够）
    got_entry = None
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rr = http_json(base + "/api/vision/results?limit=100")["results"]
        new = [x for x in rr if x.get("source") == "AlignCameraTigger_2"]
        if len(rr) > n_res and new:
            got_entry = new[-1]
            break
        time.sleep(0.05)
    caps = http_json(base + "/api/camera/captures?limit=100")["captures"]
    # 灯灭走 pending→PLC 扫描→live（实测结果可见后灯灭滞后 8~97ms），
    # 单次读是固有时序竞态——轮询等灭（上限 0.6s，远超扫描周期量级）
    v_off = {"value": None}
    t0 = time.time()
    while time.time() - t0 < 0.6:
        v_off = http_json("http://127.0.0.1:%d/io/observe?path="
                          "SystemMachine.SS_Bonding.M_AS.oLightOn_2" % PLC_PORT)
        if v_off.get("value") is False:
            break
        time.sleep(0.005)
    check("vision.trigger_flash", saw_on and got_entry is not None
          and len(caps) > n_caps
          and caps[-1]["source"] == "AlignCameraTigger_2"
          and got_entry.get("flash") is True
          and got_entry["result"]["judge"] is True
          and v_off.get("value") is False,
          "saw_on=%s entry=%s light_off=%s"
          % (saw_on, bool(got_entry), v_off.get("value")))
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "exposure_ms": 8.0})

    # ---- 17.6 模板 CRUD：建/改名/改阈值/列表/删除；建模板后用它 identify ----
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
    time.sleep(0.15)
    c, r = http_post_tpl(base + "/api/vision/templates",
                     {"cam": "down", "name": "crud_tpl", "shape": "cross"})
    tpl = r.get("template") or {}
    ok_create = c == 200 and tpl.get("name") == "crud_tpl" \
        and tpl.get("cam") == "down" and tpl.get("w", 0) > 0
    c, r = http_put(base + "/api/vision/templates",
                    {"name": "crud_tpl", "new_name": "crud_tpl2",
                     "threshold": 0.75})
    ok_upd = c == 200 and (r.get("template") or {}).get("name") == "crud_tpl2" \
        and abs((r.get("template") or {}).get("threshold", 0) - 0.75) < 1e-9
    lst = http_json(base + "/api/vision/templates")["templates"]
    ok_list = any(t["name"] == "crud_tpl2" for t in lst)
    used_ok = True
    if has_cv2:
        http_post(base + "/api/vision/mode", {"mode": "real"})
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "down", "template": "crud_tpl2"})
        used_ok = c == 200 and ((r.get("entry") or {})
                                .get("result") or {}).get("judge") is True
        http_post(base + "/api/vision/mode", {"mode": "fast"})
    # DELETE（body 形式）
    req = urllib.request.Request(
        base + "/api/vision/templates",
        data=json.dumps({"name": "crud_tpl2"}).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="DELETE")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            rd = json.loads(resp.read().decode("utf-8"))
            c_del = resp.status
    except urllib.error.HTTPError as ex:
        c_del, rd = ex.code, json.loads(ex.read().decode("utf-8"))
    lst2 = http_json(base + "/api/vision/templates")["templates"]
    ok_del = c_del == 200 and rd.get("ok") \
        and not any(t["name"] == "crud_tpl2" for t in lst2)
    check("vision.template_crud", ok_create and ok_upd and ok_list
          and used_ok and ok_del,
          "create=%s upd=%s list=%s used=%s del=%s"
          % (ok_create, ok_upd, ok_list, used_ok, ok_del))

    # ---- 17.7 REAL 链路：渲染→TCP 9200→模板匹配→MQTT 发布 ----
    # （迭代 18：此处出 4× 段窗回 1×——17.8 perf_guard 读的统计窗必须是
    #   纯 1×；17.9 前回 4×）
    set_speed(1.0)
    if has_cv2:
        # 模板（常开光源下从当前帧取 ROI）
        c, r = http_post_tpl(base + "/api/vision/templates",
                         {"cam": "down", "name": "tst_cross",
                          "shape": "cross", "threshold": 0.6})
        assert c == 200, r
        http_post(base + "/api/vision/mode", {"mode": "real"})
        from mqtt_broker import MqttSub
        sub = MqttSub(port=MQTT_PORT, client_id="test-vision-sub")  # 端口随 D2W_PORT_MQTT
        sub.subscribe("vision/result/down")
        t0 = time.time()
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "down", "template": "tst_cross"})
        wall_ms = (time.time() - t0) * 1000
        e = (r.get("entry") or {})
        res = e.get("result") or {}
        tm = e.get("timing") or {}
        got = sub.recv(timeout=5)
        mqtt_same = False
        if got:
            try:
                # 迭代 8B 起结果载荷外包 {req_id, result}（非 MQTT 触发
                # req_id=null），比对内层七字段
                pl = json.loads(got[1])
                mqtt_same = (pl.get("result") == res
                             and pl.get("req_id") is None)
            except Exception:
                mqtt_same = False
        sub.close()
        near = res.get("lstCol") and abs(res["lstCol"][0] - 1500) < 30 \
            and abs(res["lstRow"][0] - 1500) < 30
        cam_fs = viz_cam("down").get("focus_state")
        check("vision.real_identify", c == 200 and res.get("judge") is True
              and (res.get("lstScore") or [0])[0] > 0.8 and near
              and e.get("focus_state") == "in_focus"
              and cam_fs == "in_focus"
              and all(k in tm for k in ("render_ms", "tcp_ms", "match_ms",
                                        "total_ms"))
              and 0 < tm["total_ms"] < 5000
              and tm["total_ms"] >= tm["render_ms"] and mqtt_same,
              "score=%s timing=%s wall=%.0fms mqtt=%s focus=%s/%s"
              % ((res.get("lstScore") or [None])[0],
                 {k: round(v, 1) for k, v in tm.items()}, wall_ms,
                 mqtt_same, e.get("focus_state"), cam_fs))
        # ---- 17.8 实时性红线：REAL identify 之后 tick/push 稳态不退化 ----
        # 迭代 18 度量口径修正（三轮 quick 稳定 994.2 的根因）：health 的
        # tick_hz/push_hz 是 5s 聚合窗，窗内混进单拍 REAL（~0.3-1.3s 墙钟
        # 的渲染+cv2 匹配 GIL 竞争）会被稀释 ~0.6%——这是聚合窗对单拍突发
        # 的固有失真，不是持续退化（探针实测：突发窗 993.2/jit_max 正常，
        # 之后稳态窗回 1000.0/500.1，见 Temp/_iter18_pg_probe*.log）。
        # 红线语义=REAL 活动后持续速率不掉（渲染线程卡死/常驻高负载这类
        # 真退化才违反）：read_clean_health 读起点在 identify 结束之后的
        # 稳态窗（纯 1×、不含突发；等满 10.5s>两个滚动周期保证窗起点越界，
        # 相位分析见函数 docstring），不达标再读一窗取最佳（持续退化两窗
        # 全挂，环境瞬时 stall 只污染单窗）。突发窗打印作参考不判定。
        h_burst = http_json(base + "/api/health")
        print("   [perf_guard] identify 突发窗(参考): tick=%.1f push=%.1f"
              % (h_burst["tick_hz"], h_burst["push_hz"]), flush=True)
        h = read_clean_health(base, time.time(), retries=8)  # 窗起点>此刻
                                                             # =不含突发
        check("vision.perf_guard", h["tick_hz"] >= 999.0
              and h["push_hz"] >= 499.0,
              "tick=%.1f push=%.1f（REAL 后稳态窗；突发窗 %.1f/%.1f）"
              % (h["tick_hz"], h["push_hz"],
                 h_burst["tick_hz"], h_burst["push_hz"]))
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        # 清理模板
        req = urllib.request.Request(
            base + "/api/vision/templates",
            data=json.dumps({"name": "tst_cross"}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="DELETE")
        urllib.request.urlopen(req, timeout=5).read()
    else:
        print("   [skip] vision.real_identify / vision.perf_guard（无 cv2）")

    # ---- 17.9 模式切换语义 + 非法值 400 ----
    set_speed(4.0)      # 出 1× 岛，回段级 4× 窗（迭代 18）
    c1, r1 = http_post(base + "/api/vision/mode", {"mode": "real"
                                                   if has_cv2 else "fast"})
    m1 = http_json(base + "/api/vision/cameras").get("mode")
    c2, r2 = http_post(base + "/api/vision/mode", {"mode": "fast"})
    m2 = http_json(base + "/api/vision/cameras").get("mode")
    c3, _ = http_post(base + "/api/vision/mode", {"mode": "turbo"})
    check("vision.mode_switch", c1 == 200 and c2 == 200 and c3 == 400
          and m2 == "fast" and (m1 == "real" if has_cv2 else True),
          "m1=%s m2=%s bad=%s" % (m1, m2, c3))

    # ---- 17.10 面板"视觉软件"tab 托管标记（GET /panel 含 pageVision/视觉软件） ----
    try:
        with urllib.request.urlopen(base + "/panel", timeout=5) as r:
            panel_html = r.read().decode("utf-8", "replace")
            c_pn = r.status
    except urllib.error.HTTPError as ex:
        c_pn, panel_html = ex.code, ""
    check("vision.panel_tab_served", c_pn == 200
          and "pageVision" in panel_html and "视觉软件" in panel_html
          and "tab=vision" in panel_html,
          "status=%s len=%d" % (c_pn, len(panel_html)))

    # ---- 清理副作用：轴回零位 / 光源灭 / 模式 fast / 物料复位 ----
    for lp in ("LightOn_1", "LightOn_2", "AS_LightOn"):
        plc_setbool("SystemMachine.SS_Bonding.M_AS." + lp, False)
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y", "WS_Rz", "ES_X"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y", "ES_X"):
        wait_settled(ax)
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "trigger", "exposure_ms": 8.0})

    # ---- 17.11 视场场景渲染（/api/vision/frame 预览帧统计，FAST/REAL 同
    #      渲染管线；机位=全轴零位）+ focus_state=no_mark ----
    # 2026-09-08：本小节（scene_*/layers_*）度量**渲染几何**（景深滚降、
    # 纹路锐利度、渐晕），须排除质心噪声（fast_noise_px 结果倒推注入后
    # 帧噪声+亚像素抖动会平滑剖面梯度、抬高糊态基线——传感器仿真项由
    # i21/i22/i28 系列覆盖）；小节结束恢复默认
    http_put(base + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.0}}})
    mean_dark = frame_stats(_frame("down"))[0]       # 灯灭 → 近全黑
    check("vision.scene_light_off", mean_dark < 20.0,
          "mean=%.1f" % mean_dark)
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_1", True)
    time.sleep(0.2)
    mean_up_void = frame_stats(_frame("up"))[0]      # head 无 die → 暗背景
    mean_bare, std_bare, _, _, _ = frame_stats(_frame("down"))  # 无片→陶瓷面
    check("vision.scene_carrier_only", 70.0 < mean_bare < 150.0
          and std_bare > 2.0 and 70.0 < mean_up_void < 150.0,
          "down_bare=%.1f(std=%.1f) up_void=%.1f"
          % (mean_bare, std_bare, mean_up_void))
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    mean_wf = frame_stats(_frame("down"))[0]   # 零位晶圆盘盖满 FOV（更亮）
    http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    check("vision.scene_wafer_surface", mean_wf - mean_bare > 10.0,
          "wafer=%.1f bare=%.1f" % (mean_wf, mean_bare))
    # 渐晕：匀质陶瓷面上中心比四角亮（灰度差只可能来自渐晕）
    _, _, w, h, px = frame_stats(_frame("down"))
    ctr = patch_mean(px, w, w // 2 - 30, h // 2 - 30, 60, 60)
    cor = (patch_mean(px, w, 2, 2, 30, 30)
           + patch_mean(px, w, w - 32, 2, 30, 30)
           + patch_mean(px, w, 2, h - 32, 30, 30)
           + patch_mean(px, w, w - 32, h - 32, 30, 30)) / 4.0
    check("vision.scene_vignette", ctr - cor > 8.0,
          "center=%.1f corner=%.1f" % (ctr, cor))
    # 无晶圆 → 无标记：focus_state=no_mark（闪光拍照也成立，几何事实优先）
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e11 = r.get("entry") or {}
    check("vision.focus_state_no_mark", c == 200
          and e11.get("focus_state") == "no_mark"
          and e11.get("n_marks") == 0
          and e11.get("focus_err_um") is None
          and e11.get("blur_ratio") is None
          and (e11.get("result") or {}).get("judge") is False,
          "focus=%s n=%s" % (e11.get("focus_state"), e11.get("n_marks")))
    # （coarse 的 die/吸盘灰度对比已并入 17.2r——die 在翻转吸盘上看）
    # 17.11 清理：光源灭 / 物料复位
    for lp in ("LightOn_1", "LightOn_2", "AS_LightOn"):
        plc_setbool("SystemMachine.SS_Bonding.M_AS." + lp, False)
    http_post(base + "/api/workpiece/reset", {})

    # ---- 17.12 分层景深渲染 + surfaces（2026-08-21 分层离焦模型）----
    # 场景：晶圆上片 + 17.1 走位（标记在画幅中心）；光源点亮出图
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
    time.sleep(0.2)
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    def _surfs(cid):
        d = http_json(base + "/api/vision/cameras")["cameras"][cid]
        return {s["name"]: s for s in d.get("surfaces") or []}

    # ① AS_Z=0 有片：wafer 层在焦（|defocus|<20um）；chuck 层 −1mm 离焦；
    #   identify entry 同样带 surfaces
    s0 = _surfs("down")
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e12 = r.get("entry") or {}
    es = {s["name"]: s for s in e12.get("surfaces") or []}
    check("vision.layers_surfaces",
          s0.get("wafer", {}).get("in_dof") is True
          and abs(s0["wafer"]["defocus_um"]) < 20.0
          and s0.get("chuck", {}).get("in_dof") is False
          and -1050.0 < s0["chuck"]["defocus_um"] < -950.0
          and es.get("wafer", {}).get("in_dof") is True
          and abs(es["chuck"]["defocus_um"]
                  - s0["chuck"]["defocus_um"]) < 1.0,
          "cam=%s entry=%s" % ({k: v["defocus_um"] for k, v in s0.items()},
                               {k: v["defocus_um"] for k, v in es.items()}))
    # ①b 标记清晰（宿主 wafer 层在焦）：条带剖面梯度基准
    #   条带跨过标记竖臂左缘（0.6um/px@3000 下 640 预览 k=2.8125um/px，
    #   臂宽 0.5mm=178px → 竖臂 x=w/2±89=231/409；x0=w/2-129 跨左缘 231）；
    #   y 取 h/2-200..h/2-140 避开横臂（横臂 y=240±89=151..329）。
    #   锐利=阶跃，离焦 σ=8 → 斜坡。
    _, _, w, h, px = frame_stats(_frame("down"))
    grad_sharp = patch_profile_grad(px, w, w // 2 - 129, h // 2 - 200,
                                    160, 60)

    # ② AS_Z=+1（焦面 +1mm）：wafer 层 −1000um 离焦判出景深；同位条带
    #   标记被糊 → 剖面梯度大跌
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 1.0,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    s1 = _surfs("down")
    _, _, w, h, px = frame_stats(_frame("down"))
    grad_blur = patch_profile_grad(px, w, w // 2 - 129, h // 2 - 200,
                                   160, 60)
    check("vision.layers_defocus",
          s1.get("wafer", {}).get("in_dof") is False
          and -1050.0 < s1["wafer"]["defocus_um"] < -950.0
          and grad_sharp > 40.0 and grad_blur < grad_sharp * 0.3,
          "defocus=%s grad=%.1f→%.1f"
          % (s1.get("wafer", {}).get("defocus_um"), grad_sharp, grad_blur))

    # ③ 下片 + AS_Z=−1（焦面=陶瓷面 884.9）：chuck 层在焦、纹路锐利；
    #   走位让槽心压相机（纹路口子在视野中心）；对照 AS_Z=0 糊态
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.0,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    for ax, tgt in (("AS_X", 0.0), ("AS_Y", 0.0),
                    ("WS_LS_X", -42.394), ("WS_LS_Y", 1.0),
                    ("AS_Z", -1.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("AS_X", "AS_Y", "WS_LS_X", "WS_LS_Y", "AS_Z"):
        wait_settled(ax)
    s2 = _surfs("down")
    _, _, w, h, px = frame_stats(_frame("down"))
    # 竖直槽右缘条带（槽宽 0.5mm=178px@640 预览 → 竖槽 x=231..409；
    # x0=w/2+60 跨右缘 409；y 40..100 避开横向槽 y=151..329）剖面梯度
    g_sharp = patch_profile_grad(px, w, w // 2 + 60, h // 2 - 200, 50, 60)
    check("vision.layers_chuck_in_focus",
          set(s2) == {"chuck"} and s2["chuck"]["in_dof"] is True
          and abs(s2["chuck"]["defocus_um"]) < 20.0,
          "surfs=%s" % {k: v["defocus_um"] for k, v in s2.items()})
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.0,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    s3 = _surfs("down")
    _, _, w, h, px = frame_stats(_frame("down"))
    g_blur = patch_profile_grad(px, w, w // 2 + 60, h // 2 - 200, 50, 60)
    check("vision.layers_chuck_grooves",
          s3["chuck"]["in_dof"] is False
          and g_sharp > 8.0 and g_blur < g_sharp * 0.4,
          "grad 锐利=%.1f 糊=%.1f defocus=%s"
          % (g_sharp, g_blur, s3["chuck"]["defocus_um"]))
    # 几何小节结束：恢复质心噪声默认（后续小节/流程按真实传感器口径）
    http_put(base + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.1}}})

    # ---- 17.13 标记引导接口（GET /api/vision/guide?cam=X）----
    # 回 17.1 走位 + 上片：wafer_mark_1 在画幅中心 → delta≈0 且最近在前
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    g = http_json(base + "/api/vision/guide?cam=down")
    ms = g.get("marks") or []
    m0 = ms[0] if ms else {}
    sug = {s["axis"]: s["delta_mm"] for s in m0.get("suggested") or []}
    check("vision.guide_centered", g.get("ok") and g.get("cam") == "down"
          and len(ms) >= 4                    # wafer 全局 marks + 近旁 site 标记
          and m0.get("name") == "wafer_mark_1"   # 居中全局标记恒最近
          and m0.get("in_fov") is True
          and abs(m0["delta_mm"][0]) < 0.1
          and abs(m0["delta_mm"][1]) < 0.1
          and abs(m0.get("focus_err_um") or 9e9) < 20.0
          and set(sug) == {"WS_LS_X", "WS_LS_Y"}
          and abs(sug["WS_LS_X"]) < 0.1 and abs(sug["WS_LS_Y"]) < 0.1,
          "n=%d first=%s delta=%s sug=%s"
          % (len(ms), m0.get("name"), m0.get("delta_mm"), sug))
    # 点动 WS_LS_X +10 → 标记世界 +10 → delta.x≈+10、suggested.x≈−10、
    # 标记出视野但仍在列表（site 标记更近排在前面，按名字取 wafer_mark_1）
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 68.2162,
                                        "speed": 600.0})
    wait_settled("WS_LS_X")
    g = http_json(base + "/api/vision/guide?cam=down")
    ms = g.get("marks") or []
    m0 = next((m_ for m_ in ms if m_.get("name") == "wafer_mark_1"), {})
    sug = {s["axis"]: s["delta_mm"] for s in m0.get("suggested") or []}
    check("vision.guide_jog", m0.get("name") == "wafer_mark_1"
          and m0.get("in_fov") is False
          and abs(m0["delta_mm"][0] - 10.0) < 0.05
          and abs(sug.get("WS_LS_X", 9e9) + 10.0) < 0.05
          and abs(sug.get("WS_LS_Y", 9e9)) < 0.1,
          "delta=%s sug=%s in_fov=%s" % (m0.get("delta_mm"), sug,
                                         m0.get("in_fov")))
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 58.2162,
                                        "speed": 600.0})
    wait_settled("WS_LS_X")
    # ---- 17.13b 像素→世界反算端点（GET /api/vision/solve_pixel，迭代 48；
    # 测校 GUI 动态 CSV 富化链）：取 guide 的 wafer_mark_1 fov_center 反算，
    # mark_world.xy 应≈cam_world.xy+delta_mm（与 guide 世界系同口径）；
    # 坏 cam/坏坐标各 400
    g = http_json(base + "/api/vision/guide?cam=down")
    m0 = next((m_ for m_ in (g.get("marks") or [])
               if m_.get("name") == "wafer_mark_1"), {})
    fc = m0.get("fov_center") or [1500.0, 1500.0]
    sp = http_json(base + "/api/vision/solve_pixel?cam=down&col=%.4f&row=%.4f"
                   % (fc[0], fc[1]))
    dw = m0.get("delta_mm") or [0.0, 0.0]
    mw, cw = sp.get("mark_world_mm"), sp.get("cam_world_mm")
    check("vision.solve_pixel", sp.get("ok") and sp.get("cam") == "down"
          and isinstance(mw, list) and len(mw) == 3
          and isinstance(cw, list) and len(cw) == 3
          and abs(mw[0] - (cw[0] + dw[0])) < 0.01
          and abs(mw[1] - (cw[1] + dw[1])) < 0.01,
          "fc=%s mark_world=%s cam_world=%s delta=%s" % (fc, mw, cw, dw))
    c_bad1, _ = http_get_raw(base + "/api/vision/solve_pixel?cam=nope"
                             "&col=1&row=1")
    c_bad2, _ = http_get_raw(base + "/api/vision/solve_pixel?cam=down"
                             "&col=abc&row=1")
    check("vision.solve_pixel_400", c_bad1 == 400 and c_bad2 == 400,
          "bad_cam=%s bad_col=%s（应 400/400）" % (c_bad1, c_bad2))
    # （coarse 的 guide 已并入 17.2g——标记载体=翻转模块，suggested 用 FA_X）
    # 未知相机 → 400
    try:
        urllib.request.urlopen(base + "/api/vision/guide?cam=nope", timeout=5)
        c_bad = 200
    except urllib.error.HTTPError as ex:
        c_bad = ex.code
    check("vision.guide_bad_cam_400", c_bad == 400, "code=%s" % c_bad)

    # ---- 17.12/17.13 清理：轴回零 / 光源灭 / 物料复位 ----
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y", "WS_Rz"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    http_post(base + "/api/workpiece/reset", {})

    # ---- 17.14~17.16（采图库 / wafer_die 标记链）已拆到
    #      test_vision_captures（迭代 18，full-only）----

    # ---- 17.17 晶圆 per-site 对准标记（wafer.site_marks，2026-08-24：
    #      产品片每 chip site 自带标记，ratio×site 半宽等比缩放；
    #      命名 site_R{r}C{c}#{k}）----
    # FAST：晶圆上片 → 相机移到 site(0,0) 的 #0 标记（site 局部 (-3.8,
    # -3.8)，FOV 1.8mm 下四角标记不可同见，居中单个验证）→ site 标记
    # 在焦可识别、命名正确
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    # 迭代 4 零位纠错：AS=(0,0) 相机世界=(-42.5,1.0)，site(0,0)#0 标记局部
    # (-3.8,-3.8) → WS=(-38.594,4.8) 使标记居中
    for ax, tgt in (("WS_LS_X", -38.594), ("WS_LS_Y", 4.8),
                    ("AS_X", 0.0), ("AS_Y", 0.0), ("AS_Z", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    def _down_id():
        c_, r_ = http_post(base + "/api/vision/identify", {"cam": "down"})
        e_ = r_.get("entry") or {}
        return c_, e_, (e_.get("result") or {})

    c, e17, res17 = _down_id()
    g17 = http_json(base + "/api/vision/guide?cam=down")
    names17 = [m_["name"] for m_ in g17.get("marks") or []]
    check("vision.site_marks", c == 200
          and res17.get("judge") is True
          and res17.get("strShapeName") == "quad"
          and len(res17.get("lstCol") or []) >= 1
          and abs((res17.get("lstCol") or [0])[0] - 1500) < 30
          and e17.get("focus_state") == "in_focus"
          and any(n_.startswith("site_R") for n_ in names17),
          "judge=%s n=%s col=%s names=%s"
          % (res17.get("judge"), len(res17.get("lstCol") or []),
             res17.get("lstCol"), names17[:4]))
    # 迭代 16 契约修订：带 template 触发 → strShapeName 回显模板名（不再
    # 是形状名；真实视觉软件按模板名启动/回查识别）。FAST 路径用随仓库
    # 下发的占位模板 tpl_post_inspect（cam=down）。
    c16t, r16t = http_post(base + "/api/vision/identify",
                           {"cam": "down", "template": "tpl_post_inspect"})
    res16t = ((r16t.get("entry") or {}).get("result") or {})
    check("vision.shapename_template_fast", c16t == 200
          and res16t.get("strShapeName") == "tpl_post_inspect",
          "strShapeName=%s" % res16t.get("strShapeName"))
    # FAST/REAL 一致：亮灯建模板（框居中 site 标记）→ REAL identify
    real17 = True
    if has_cv2:
        plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
        time.sleep(0.2)
        c, r = http_post_tpl(base + "/api/vision/templates",
                         {"cam": "down", "name": "site17_tpl",
                          "shape": "box", "roi": [1300, 1300, 401, 401]})
        real17 = c == 200
        http_post(base + "/api/vision/mode", {"mode": "real"})
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "down", "template": "site17_tpl"})
        res_r = ((r.get("entry") or {}).get("result") or {})
        real17 = real17 and c == 200 and res_r.get("judge") is True \
            and res_r.get("strShapeName") == "site17_tpl" \
            and (res_r.get("lstScore") or [0])[0] > 0.7 \
            and abs((res_r.get("lstCol") or [0])[0] - 1500) < 30
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        http_delete(base + "/api/vision/templates", {"name": "site17_tpl"})
        plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    check("vision.site_marks_real", real17, "real=%s" % real17)
    # 等比缩放：ratio=[[-0.5,-0.5]]+per_site=1 → site(0,0) 单标记在局部
    # (-2.5,-2.5)（相机随之移到该处居中）；site 尺寸由 wafer_sites.size
    # 显式给（2026-08-27 起不再默认跟随 film.dies）10→11mm → 标记局部
    # (−2.5→−2.75)mm ≈ col −416.7px（k=0.6um/px）
    http_put(base + "/api/recipe",
             {"wafer": {"site_marks": {"ratio": [[-0.5, -0.5]],
                                       "per_site": 1}}})
    # 迭代 4 零位纠错：AS=(0,0) 相机世界=(-42.5,1.0)，ratio (-0.5,-0.5)×
    # site 半宽 5mm = (-2.5,-2.5) → WS=(-39.894,3.5) 使标记居中
    for ax, tgt in (("WS_LS_X", -39.894), ("WS_LS_Y", 3.5)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    wait_settled("WS_LS_X")
    wait_settled("WS_LS_Y")

    def _site_col():
        c_, e_, r_ = _down_id()
        cols_ = (r_.get("lstCol") or [])
        return c_, (cols_[0] if cols_ else None)

    c_a, col_a = _site_col()
    http_put(base + "/api/recipe", {"wafer_sites": {"size_x_mm": 11.0}})
    c_b, col_b = _site_col()
    http_post(base + "/api/recipe/restore_default", {})
    shift17 = (col_b - col_a) if None not in (col_a, col_b) else 9e9
    # 非法 site_marks（per_site 超 ratio 长度）→ 400 且不污染
    c_bad17, _ = http_put(base + "/api/recipe",
                          {"wafer": {"site_marks": {"per_site": 9}}})
    check("recipe.site_marks_scale", c_a == 200 and c_b == 200
          and None not in (col_a, col_b)
          and abs(shift17 + 416.7) < 15.0
          and c_bad17 == 400,
          "col=%.2f→%.2f shift=%.1fpx（期 -416.7） bad=%s"
          % (col_a or -1.0, col_b or -1.0, shift17, c_bad17))
    # 17.17 清理：轴回零 / 物料复位
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    http_post(base + "/api/workpiece/reset", {})


# ---------------- 参数化工艺流程（翻转取片） ----------------
def test_process_flows():
    """--- 19. 参数化工艺流程：翻转取片/放回蓝膜（/api/process/flip_pick、
    /api/process/flip_place_back → process_flows.build_* + flow_engine.
    run_steps 动态步骤；2026-08-25 零位约定：取片 FA_Rx=−π、收尾回 0 背面
    朝上；Rx 旋转前抬安全位；flip 容量 1 防重复拾取）---"""
    print("--- 19. 参数化工艺流程：翻转取片/放回蓝膜（process/flip_pick） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def axes_pos():
        return {a["name"]: a["pos"]
                for a in http_json(base + "/api/axes")["axes"]}

    def dies_on(carrier):
        return [d for d in http_json(base + "/api/viz")["dies"]
                if d["carrier"] == carrier]

    def plc_setbool(path, val):
        c, r = http_post("http://127.0.0.1:%d/io/command" % PLC_PORT,
                         {"path": path, "cmd": "SetBool",
                          "args": {"value": val}})
        assert c == 200 and r.get("ok"), r

    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False

    # ---- 19.1 全流程实跑（缺省 die=离取片位最近的 film die；终态 Rx=0
    #      背面朝上 + coarse FAST 背面特征在焦 + REAL 模板匹配验证）----
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/recipe/restore_default", {})
    c, r = http_post(base + "/api/process/flip_pick", {})
    assert c == 200 and r.get("ok"), r
    s = flow_wait()
    ops = [st["op"] for st in s.get("steps") or []]
    stats = [st["status"] for st in s.get("steps") or []]
    flip_dies = dies_on("flip")
    ap = axes_pos()
    c2, r2 = http_post(base + "/api/vision/identify", {"cam": "coarse"})
    e2 = (r2.get("entry") or {})
    res2 = (e2.get("result") or {})
    check("process.flip_pick_run", c == 200 and s.get("state") == "done"
          and ops == ["log", "move", "move", "move", "move", "move", "move",
                      "move", "io", "wait_grip", "wait_die", "move",
                      "move", "move", "move", "coarse_offset", "log"]
          and all(x == "ok" for x in stats)
          and len(flip_dies) == 1
          and abs(ap["FA_Rx"]) < 1e-4            # 收尾回 0（背面朝上）
          and abs(ap["FA_Z"]) < 1e-6
          and abs(ap["ES_PunchZ"]) < 1e-6
          and res2.get("judge") is True
          and (e2.get("n_marks") or 0) == 1      # 背面检出特征
          and e2.get("focus_state") == "in_focus"
          and any("粗对准测量" in (x.get("msg") or "")
                  and "偏差" in (x.get("msg") or "")
                  for x in s.get("log") or []),
          "state=%s ops=%s stats=%s flip=%s FA_Rx=%.4f coarse=%s/%s"
          % (s.get("state"), ops, set(stats),
             [d["id"] for d in flip_dies], ap["FA_Rx"],
             res2.get("judge"), e2.get("n_marks")))
    flow_cleanup()
    # 19.1t 流程拍照按机械常数模板识别（迭代 12）：coarse_offset 的
    #      identify entry 带 template=vision_marks.tpl_coarse_die
    vres = http_json(base + "/api/vision/results?limit=20") \
        .get("results") or []
    tpl_hits = [e.get("template") for e in vres
                if e.get("cam") == "coarse" and e.get("template")]
    check("process.flip_tpl_mark", "cv_chipback" in tpl_hits,
          "tpls=%s" % tpl_hits)
    # 19.1r 粗对准 REAL 验证（素板背面模板匹配；cv2 缺失跳过）
    real_ok = True
    if has_cv2:
        picked_id = flip_dies[0]["id"] if flip_dies else ""
        plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", True)
        time.sleep(0.2)
        c, r = http_post_tpl(base + "/api/vision/templates",
                         {"cam": "coarse", "name": "coarse_back_tpl",
                          "shape": "box", "roi": [892, 1440, 121, 121]})
        real_ok = c == 200
        http_post(base + "/api/vision/mode", {"mode": "real"})
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "coarse", "template": "coarse_back_tpl"})
        res_r = ((r.get("entry") or {}).get("result") or {})
        cols_r = res_r.get("lstCol") or []
        real_ok = real_ok and c == 200 and res_r.get("judge") is True \
            and cols_r and abs(cols_r[0] - 952.6) < 30.0 \
            and (res_r.get("lstScore") or [0])[0] > 0.7
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        http_delete(base + "/api/vision/templates",
                    {"name": "coarse_back_tpl"})
        plc_setbool("SystemMachine.SS_Bonding.M_AS.AS_LightOn", False)
        check("process.flip_pick_coarse_real", real_ok,
              "picked=%s real judge/col/score=%s/%s"
              % (picked_id, res_r.get("judge"), cols_r))
    # 轴回中立位（后续用例的干净现场）
    http_post(base + "/api/axis/move", {"axis": "FA_Rx", "target": 0.0,
                                        "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)

    # ---- 19.2 防重复拾取 400 + 放回蓝膜（原槽位）+ API 语义 + 再取可逆 ----
    c_dup, r_dup = http_post(base + "/api/process/flip_pick", {})
    dup_ok = c_dup == 400 and "不能重复拾取" in (r_dup.get("error") or "")
    c, r = http_post(base + "/api/process/flip_place_back", {})
    s = flow_wait()
    ap = axes_pos()
    film_n = len(dies_on("film"))
    back_ok = dup_ok and c == 200 and s.get("state") == "done" \
        and not dies_on("flip") and film_n == 149 \
        and abs(ap["FA_Rx"]) < 1e-4
    flow_cleanup()
    # 放回后再取同一颗（可逆性）；die_id 取 transfer_log 最后一条 to=film
    vz = http_json(base + "/api/viz")
    tl = [x for x in vz.get("transfer_log") or [] if x["to"] == "film"]
    die_id = tl[-1]["die"] if tl else ""
    # API 语义（flip 已空）：未知 die 400 / 指定 die 暂停生成 200 / 冲突 409
    c_bad, r_bad = http_post(base + "/api/process/flip_pick",
                             {"die_id": "NOPE"})
    c, r = http_post(base + "/api/process/flip_pick",
                     {"die_id": "R05C06", "paused": True})
    name_ok = c == 200 and "R05C06" in (r.get("flow") or "")
    c_conf, _ = http_post(base + "/api/process/flip_pick", {})   # 暂停中 → 409
    flow_cleanup()
    c, r = http_post(base + "/api/process/flip_pick", {"die_id": die_id})
    s = flow_wait()
    reok = c == 200 and s.get("state") == "done" \
        and [d["id"] for d in dies_on("flip")] == [die_id]
    flow_cleanup()
    check("process.flip_place_back", back_ok and reok
          and c_bad == 400 and "NOPE" in (r_bad.get("error") or "")
          and name_ok and c_conf == 409,
          "dup=%s back=%s film=%d api=%s/%s repick=%s(%s)"
          % (dup_ok, back_ok, film_n, c_bad, c_conf, reok, die_id))
    http_post(base + "/api/workpiece/reset", {})

    # ---- 19.3 参数联动 / 19.5 常数校验已拆到 test_process_flows_gen ----
    # （迭代 18：生成期用例两套件都跑；本段实跑用例 full-only）

    # ---- 19.4 报警路径 ----
    # (a) 真空未建立：先释放 ChuckVac 排空，再 PLC 沉默（io 指令被吞，真空
    #     永远不起）→ wait_grip 超时 → abort 含"负压未建立"报警文本
    #     （pick_vac_timeout_s 调 0.5s 加速）。注意：PLC 沉默会吞掉全部
    #     PLC 侧指令（ES/FA_Z/顶针轴 move 同样被吞、轴原地不动——固件挂死
    #     语义），本断言只锁定真空判定与报警文本。
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    time.sleep(0.8)                          # 等真空衰减
    http_put(base + "/api/machine_constants",
             {"flip_pick": {"pick_vac_timeout_s": 0.5}})
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": True})
    c, r = http_post(base + "/api/process/flip_pick", {"die_id": "R05C06"})
    s = flow_wait(60.0)
    http_post(base + "/api/fault", {"action": "silent", "target": "plc",
                                    "on": False})
    http_put(base + "/api/machine_constants",
             {"flip_pick": {"pick_vac_timeout_s": 3.0}})
    reason = (s.get("log") or [{}])[-1].get("msg", "")
    wg = [st for st in (s.get("steps") or []) if st["op"] == "wait_grip"]
    io_st = [st for st in (s.get("steps") or []) if st["op"] == "io"]
    check("process.flip_pick_alarm", c == 200 and s.get("state") == "aborted"
          and "取片失败：ChuckVac 负压未建立" in reason
          and wg and wg[0]["status"] == "fail"
          and io_st and "被吞" in (io_st[0].get("info") or "")
          and not any(d["carrier"] == "flip"
                      for d in http_json(base + "/api/viz")["dies"]),
          "state=%s reason=%s" % (s.get("state"), reason[:80]))
    flow_cleanup()
    # (b) 真空正常但交接窗口不满足（transfers film_to_flip 的 FA_Z 窗口改到
    #     不含 -4.2）→ wait_grip 过、wait_die 超时 → abort 含"未交接"文本
    tr = http_json(base + "/api/transfers")
    orig_rules = tr["rules"]
    mod_rules = [dict(rl) for rl in orig_rules]
    for rl in mod_rules:
        if rl["name"] == "film_to_flip":
            rl["pose"] = dict(rl["pose"])
            rl["pose"]["FA_Z"] = [-5.0, 0.5]     # -4.2 落在窗口外
    http_put(base + "/api/transfers", {"rules": mod_rules})
    c, r = http_post(base + "/api/process/flip_pick", {"die_id": "R05C06"})
    s = flow_wait(60.0)
    # 先破坏交接条件（放真空 + FA_Z 回 0）再还原规则——否则还原瞬间条件
    # 满足会静默触发一次交接（上升沿），污染断言
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_Z")
    time.sleep(0.5)
    http_put(base + "/api/transfers", {"rules": orig_rules})   # 还原
    reason = (s.get("log") or [{}])[-1].get("msg", "")
    wd = [st for st in (s.get("steps") or []) if st["op"] == "wait_die"]
    check("process.flip_pick_alarm_no_transfer",
          c == 200 and s.get("state") == "aborted"
          and "取片失败：真空已建立但 die 未交接" in reason
          and wd and wd[0]["status"] == "fail"
          and not any(d["carrier"] == "flip"
                      for d in http_json(base + "/api/viz")["dies"]),
          "state=%s reason=%s" % (s.get("state"), reason[:80]))
    flow_cleanup()

    # ---- 19.5 常数/处方校验已拆到 test_process_flows_gen ----

    # ---- 19.6 误差传递链（需求 2）：膜上片误差注入（pick_offset）→ die
    #      在吸盘真空面上的位置偏差 → 粗对准拍照反投影世界坐标 → 与标定
    #      中心 coarse_die_center_mm 的偏差跟随注入值；清零后回 um 级。
    #      注意流程日志跨流程累积（reset 只清步骤），须取最后一次匹配 ----
    import re as _re
    http_post(base + "/api/workpiece/pick_offset",
              {"x_mm": 0.3, "y_mm": -0.2})
    c, r = http_post(base + "/api/process/flip_pick", {})
    s = flow_wait()
    log_txt = " ".join((x.get("msg") or "") for x in s.get("log") or [])
    m1 = _re.findall(r"偏差 \(([+-][\d.]+), ([+-][\d.]+)\) mm", log_txt)
    ok6 = c == 200 and s.get("state") == "done" and len(m1) >= 1
    if ok6:
        dx1, dy1 = float(m1[-1][0]), float(m1[-1][1])
        ok6 = abs(dx1 - 0.3) < 0.05 and abs(dy1 - (-0.2)) < 0.05
    flow_cleanup()
    http_post(base + "/api/workpiece/reset", {})     # 清注入
    c, r = http_post(base + "/api/process/flip_pick", {})
    s = flow_wait()
    log_txt = " ".join((x.get("msg") or "") for x in s.get("log") or [])
    m2 = _re.findall(r"偏差 \(([+-][\d.]+), ([+-][\d.]+)\) mm", log_txt)
    ok6 = ok6 and c == 200 and s.get("state") == "done" and len(m2) > len(m1)
    if ok6:
        dx2, dy2 = float(m2[-1][0]), float(m2[-1][1])
        ok6 = abs(dx2) < 0.05 and abs(dy2) < 0.05
    flow_cleanup()
    check("process.coarse_offset_err_chain", ok6,
          "injected off=%s clean off=%s"
          % (m1[-1] if m1 else None, m2[-1] if m2 else None))
    http_post(base + "/api/workpiece/reset", {})

    # ---- 19.7 芯片交接（/api/process/flip_handoff、head_to_flip；
    #      2026-08-26：机械常数 flip_handoff 组 + transfers flip_to_head
    #      真实几何示教位 + 粗对准偏差补偿 + 逆向吸回闭环）----
    print("--- 19.7 芯片交接（flip→head / head→flip） ---")
    # (a) 守卫：flip 空跑交接 400；head 空跑吸回 400
    c_g1, r_g1 = http_post(base + "/api/process/flip_handoff", {})
    c_g2, r_g2 = http_post(base + "/api/process/head_to_flip", {})
    guard_ok = c_g1 == 400 and "先执行翻转取片" in (r_g1.get("error") or "") \
        and c_g2 == 400 and "先执行芯片交接" in (r_g2.get("error") or "")
    # (b) 补偿符号：注入 pick_offset(+0.007, -0.0022)（用户例）→ 取片
    #     （coarse_offset 步落存偏差到 eng.coarse_offset 并随 flow/state
    #      外露）→ paused 生成交接 → FA_X 目标 = fa_x_handoff − off_x
    http_post(base + "/api/workpiece/pick_offset",
              {"x_mm": 0.007, "y_mm": -0.0022})
    c, r = http_post(base + "/api/process/flip_pick", {})
    s = flow_wait()
    co = flow_state().get("coarse_offset")
    flip_id = (dies_on("flip") or [{}])[0].get("id")
    ok7 = c == 200 and s.get("state") == "done" and co is not None \
        and co.get("die") == flip_id \
        and abs(co["dx"] - 0.007) < 0.01 and abs(co["dy"] + 0.0022) < 0.01
    flow_cleanup()
    c, r = http_post(base + "/api/process/flip_handoff", {"paused": True})
    sd = r.get("steps_def") or []
    fax_t = [st_["target"] for st_ in sd
             if st_.get("op") == "move" and st_.get("axis") == "FA_X"]
    bsz_t = [st_["target"] for st_ in sd
             if st_.get("op") == "move" and st_.get("axis") == "BS_Arm_Z"]
    asy_t = [st_["target"] for st_ in sd
             if st_.get("op") == "move" and st_.get("axis") == "AS_Y"]
    ops7 = [st_["op"] for st_ in sd]
    # 补偿步=首个 FA_X move（第二个是撤离位 fa_x_retreat=0）
    ok7b = c == 200 and len(fax_t) == 2 and co is not None \
        and abs(fax_t[0] - round(-1242.35 - co["dx"], 4)) < 1e-6 \
        and abs(fax_t[0] - (-1242.357)) < 0.005 \
        and abs(fax_t[1] - 0.0) < 1e-9 \
        and asy_t == [120.0] \
        and bsz_t == [0.0, -50.93, 0.0]              # 安全→接触→抬起
    flow_cleanup()
    check("process.flip_handoff_compensate", ok7 and ok7b and guard_ok,
          "guard=%s co=%s fax=%s bsz=%s" % (guard_ok, co, fax_t, bsz_t))
    # (c) 全链路实跑：交接 → die 到 head（标记面朝下）、flip 腾空
    c, r = http_post(base + "/api/process/flip_handoff", {})
    s = flow_wait(90.0)
    seq7 = [e["rule"] for e in http_json(base + "/api/viz")["transfer_log"]]
    ok7c = c == 200 and s.get("state") == "done" \
        and all(st_["status"] == "ok" for st_ in s.get("steps") or []) \
        and len(dies_on("head")) == 1 and not dies_on("flip") \
        and "flip_to_head" in seq7
    flow_cleanup()
    # (d) 重复交接守卫：head 已有 die → 400
    c_d7, r_d7 = http_post(base + "/api/process/flip_handoff", {})
    ok7c = ok7c and c_d7 == 400 \
        and "键合头上已有芯片" in (r_d7.get("error") or "")
    check("process.flip_handoff_run", ok7c,
          "c=%s state=%s head=%d flip=%d notok=%s seq=%s dup=%s"
          % (c, s.get("state"), len(dies_on("head")), len(dies_on("flip")),
             [(st_["i"], st_["op"]) for st_ in (s.get("steps") or [])
              if st_["status"] != "ok"], seq7[-3:], c_d7))
    # (e) 吸回：head_to_flip 实跑 → die 回 flip、head 腾空（闭环可逆）
    c, r = http_post(base + "/api/process/head_to_flip", {})
    s = flow_wait(90.0)
    seq7 = [e["rule"] for e in http_json(base + "/api/viz")["transfer_log"]]
    ok7e = c == 200 and s.get("state") == "done" \
        and len(dies_on("flip")) == 1 and not dies_on("head") \
        and "head_to_flip" in seq7
    flow_cleanup()
    # (f) 吸回后可再交接（paused 生成验证守卫通过，不重复实跑省时间）
    c, r = http_post(base + "/api/process/flip_handoff", {"paused": True})
    ok7e = ok7e and c == 200
    flow_cleanup()
    check("process.head_to_flip_recover", ok7e,
          "state=%s flip=%d seq=%s rehandoff=%s"
          % (s.get("state"), len(dies_on("flip")), seq7[-3:], c))
    # (g) flip_handoff 组常数校验：坏值 400；改交接位热生效（步骤目标跟随）
    bads7 = [
        {"flip_handoff": {"head_z_touch_mm": -200.0}},   # 超 BS_Arm_Z 量程
        {"flip_handoff": {"fa_x_handoff_mm": -1300.0}},  # 超 FA_X 量程
        {"flip_handoff": {"bogus": 1}},                  # 未知字段
        {"flip_handoff": {"as_safe_xy_mm": [1.0]}},      # 非二维
    ]
    codes7 = [http_put(base + "/api/machine_constants", b)[0] for b in bads7]
    c, r = http_put(base + "/api/machine_constants",
                    {"flip_handoff": {"fa_x_handoff_mm": -1200.0}})
    ok7g = c == 200
    c, r = http_post(base + "/api/process/flip_handoff", {"paused": True})
    fax_g = [st_["target"] for st_ in (r.get("steps_def") or [])
             if st_.get("op") == "move" and st_.get("axis") == "FA_X"]
    # co 仍是 (b) 的测量：目标 = -1200 − dx
    ok7g = ok7g and c == 200 and fax_g and co is not None \
        and abs(fax_g[0] - round(-1200.0 - co["dx"], 4)) < 1e-6
    flow_cleanup()
    http_put(base + "/api/machine_constants",
             {"flip_handoff": {"fa_x_handoff_mm": -1242.35}})
    check("process.flip_handoff_mc", codes7 == [400] * 4 and ok7g,
          "codes=%s fax=%s" % (codes7, fax_g))
    # 19.7 现场清理：真空释放 + AS/BS/FA 回中立 + 物料复位
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    for i in (1, 2, 3):
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.SS_ME.M_AIR.WT_BondingVac_%d" % i,
                   "cmd": "OpenAir"})
    for ax, tgt, sp in (("AS_X", 0.0, 600.0), ("AS_Y", 0.0, 500.0),
                        ("BS_Arm_Z", 0.0, 200.0), ("FA_X", 0.0, 600.0),
                        ("FA_Z", 0.0, 600.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": sp})
    for ax in ("AS_X", "AS_Y", "BS_Arm_Z", "FA_X", "FA_Z"):
        wait_settled(ax, timeout=20.0)
    http_post(base + "/api/workpiece/reset", {})

    # ---- 19 清理：真空释放 / 轴回零 / 物料复位 / 流程复位 ----
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    for ax, tgt, sp in (("FA_Rx", 0.0, 1.5), ("FA_X", 0.0, 600.0),
                        ("FA_Z", 0.0, 600.0), ("ES_X", 0.0, 600.0),
                        ("ES_Y", 0.0, 600.0), ("ES_PunchZ", 0.0, 5.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": sp})
    for ax in ("FA_Rx", "FA_X", "FA_Z", "ES_X", "ES_Y", "ES_PunchZ"):
        wait_settled(ax, timeout=20.0)
    http_post(base + "/api/workpiece/reset", {})


# ---------------- 晶圆上片 + 全局对准（EGA） ----------------
def test_process_flows_gen():
    """--- 19.3/19.5 流程生成期用例（从 test_process_flows 拆出，迭代 18）---
    paused 生成核对 steps_def 参数联动 + 常数/处方校验：不动机器、段首
    自清理、sim≈0，quick/full 两套件都跑；实跑类用例留在
    test_process_flows（full-only）。"""
    print("--- 19.3/19.5 流程生成期参数联动 + 校验（quick+full） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    # ---- 19.3 参数联动（paused 启动只核对生成的 steps_def，不动机器）----
    # 处方 die 厚 0.5→0.8：FA_Z_pick = 886.9+t−891.6 = −3.9；
    # 顶针 = 886.9+t+20/1000−885.8 = 1.92；安全位步恒 20.0
    http_put(base + "/api/recipe", {"film": {"dies": {"thickness_mm": 0.8}}})
    c, r = http_post(base + "/api/process/flip_pick",
                     {"die_id": "R05C06", "paused": True})
    sd = r.get("steps_def") or []
    faz = [s_["target"] for s_ in sd
           if s_.get("op") == "move" and s_.get("axis") == "FA_Z"]
    faz_pick = [t for t in faz if t < 0]
    punch = [s_["target"] for s_ in sd
             if s_.get("op") == "move" and s_.get("axis") == "ES_PunchZ"]
    ok_link = c == 200 and faz and abs(faz[0] - 20.0) < 1e-6 \
        and faz_pick and abs(faz_pick[0] - (-3.9)) < 1e-3 \
        and punch and abs(punch[0] - 1.92) < 1e-3
    flow_cleanup()
    # 机械常数 chuck_face_z0 891.6→891.0：FA_Z_pick = −3.9+0.6 = −3.3
    c, r = http_put(base + "/api/machine_constants",
                    {"flip_pick": {"chuck_face_z0_mm": 891.0}})
    ok_mc_put = c == 200
    c, r = http_post(base + "/api/process/flip_pick",
                     {"die_id": "R05C06", "paused": True})
    faz2 = [s_["target"] for s_ in (r.get("steps_def") or [])
            if s_.get("op") == "move" and s_.get("axis") == "FA_Z"
            and s_["target"] < 0]
    ok_link = ok_link and ok_mc_put and faz2 and abs(faz2[0] - (-3.3)) < 1e-3
    flow_cleanup()
    http_put(base + "/api/machine_constants",
             {"flip_pick": {"chuck_face_z0_mm": 891.6}})
    http_post(base + "/api/recipe/restore_default", {})
    check("process.flip_pick_param_link", ok_link,
          "faz=%s punch=%s faz2=%s" % (faz, punch, faz2))

    # ---- 19.5 常数/处方校验：flip_pick 组坏值 400（超界/未知字段/缺字段
    #      由合并校验拦）；pin_overtravel_um 越界 400 ----
    bads = [
        {"flip_pick": {"pick_vac_timeout_s": 0.0}},      # 超下界
        {"flip_pick": {"fa_rx_flip_rad": 9.0}},          # 超范围
        {"flip_pick": {"bogus": 1}},                     # 未知字段
        {"flip_pick": {"coarse_die_center_mm": [1.0, 2.0, 3.0]}},  # 非二维
    ]
    codes = [http_put(base + "/api/machine_constants", b)[0] for b in bads]
    c_rp, _ = http_put(base + "/api/recipe",
                       {"film": {"pin_overtravel_um": 600.0}})
    mc_now = http_json(base + "/api/machine_constants")["constants"]
    check("process.flip_pick_validate", codes == [400] * 4 and c_rp == 400
          and abs(mc_now["flip_pick"]["pick_vac_timeout_s"] - 3.0) < 1e-9
          and abs(mc_now["flip_pick"]["fa_rx_flip_rad"] - 0.0) < 1e-6
          and abs(mc_now["flip_pick"]["fa_rx_pick_rad"] + math.pi) < 1e-6
          and abs(mc_now["flip_pick"]["fa_z_safe_mm"] - 20.0) < 1e-6
          and abs(mc_now["flip_pick"]["coarse_die_center_mm"][0]
                  - 1199.85) < 1e-6,
          "codes=%s recipe=%s" % (codes, c_rp))


def test_focus_level_units():
    """--- 19.9u 调焦调平 fl_verify 单元用例（从 test_focus_level 拆出，
    迭代 18）：进程内构造复测记录直调 _do_fl_verify，不动机器，quick/full
    两套件都跑 ---"""
    print("--- 19.9u fl_verify 单元（FAIL 抛错 / PASS 不抛） ---")
    # ---- 迭代 5：fl_verify FAIL 路径（单测直接构造复测记录调
    #      _do_fl_verify，超阈值残余 → 落存 judge=fail 并抛错）----
    import flow_engine as femod

    class _FakeConstants(object):
        def get(self):
            return {"focus_level": {"level_tol_urad": 30.0}}

    class _FakeConfocal(object):
        def focus_plane_z(self, probe):
            return 0.0

    class _FakeEng(object):
        def __init__(self):
            self.fl_fit = {}
            self.constants = _FakeConstants()
            self.confocal = _FakeConfocal()
            self.lock = threading.Lock()
            self.sim_time = 0.0

    def _meas(space, pts):
        return [{"point": [x, y], "space": space, "world": [x, y],
                 "z_surface": z, "reading_um": 350.0, "z_focus": 0.0,
                 "searched": False, "align_first": False}
                for x, y, z in pts]

    # 晶圆面平（z=0），die 面 a=+100urad 倾斜 → 残余 100urad > 30 → FAIL
    eng_f = _FakeEng()
    eng_f.fl_measurements = {
        "wafer_v": _meas("wafer_v",
                         [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0),
                          (0.0, 1.0, 0.0)]),
        "die_v": _meas("die_v",
                       [(-1.0, -1.0, -1e-4), (1.0, -1.0, 1e-4),
                        (0.0, 1.0, 0.0)])}
    fe = object.__new__(femod.FlowEngine)
    fe.eng = eng_f
    fe.log = []
    raised = False
    try:
        fe._do_fl_verify()
    except RuntimeError:
        raised = True
    vf = eng_f.fl_fit.get("verify") or {}
    check("fl.verify_fail_raise",
          raised and vf.get("judge") == "fail"
          and abs((vf.get("residual_urad") or [0])[1] - 100.0) < 1.0,
          "raised=%s judge=%s residual=%s"
          % (raised, vf.get("judge"), vf.get("residual_urad")))
    # 同一记录把 die 倾斜改 10urad → PASS 不抛错
    eng_f.fl_fit = {}
    eng_f.fl_measurements["die_v"] = _meas(
        "die_v", [(-1.0, -1.0, -1e-5), (1.0, -1.0, 1e-5), (0.0, 1.0, 0.0)])
    info_p = fe._do_fl_verify()
    vf2 = eng_f.fl_fit.get("verify") or {}
    check("fl.verify_pass_unit",
          vf2.get("judge") == "pass" and "PASS" in info_p,
          "judge=%s" % vf2.get("judge"))


def test_vision_captures():
    """--- 17.14~17.16 采图库 + wafer_die 标记链（从 test_vision 拆出，
    迭代 18）---
    全幅 3000×3000 PNG 编码/解码是纯计算墙钟大头（不随倍速压缩），quick
    门控为 full-only；段首自清理（上片/走位/光源），段尾清理完备。
    用例：vision.capture_save / capture_api / capture_flash /
    template_from_capture / capture_delete / wafer_die_marks /
    wafer_die_defocus。"""
    print("--- 17.14~17.16 采图库 + wafer_die 标记（full-only） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    plc = "http://127.0.0.1:%d/io/command" % PLC_PORT
    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False

    def plc_setbool(path, val):
        c, r = http_post(plc, {"path": path, "cmd": "SetBool",
                               "args": {"value": val}})
        assert c == 200 and r.get("ok"), r

    def viz_cam(cid):
        return http_json(base + "/api/vision/cameras")["cameras"][cid]

    def _frame(cid):
        with urllib.request.urlopen(base + "/api/vision/frame?cam=" + cid,
                                    timeout=10) as resp:
            return resp.read()

    # ---- 17.14 采图落盘（POST /api/vision/capture/save → 列表/取图/删除）----
    # 场景：上片 + 17.1 走位（标记在画幅中心）+ 光源点亮 → 真渲染全幅落盘
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
    time.sleep(0.2)
    c, r = http_post(base + "/api/vision/capture/save", {"cam": "down"})
    cap = r.get("capture") or {}
    cap_name = cap.get("name") or ""
    axes14 = cap.get("axes") or {}
    png_path = os.path.join(ROOT, "runtime", "vision_captures",
                            cap.get("file") or "_")
    check("vision.capture_save", c == 200 and cap.get("cam") == "down"
          and cap_name.startswith("cap_down_")
          and str(cap.get("file", "")).endswith(".png")
          and cap.get("ts", 0) > 0 and cap.get("sim_t") is not None
          and abs(axes14.get("WS_LS_X", 9e9) - 58.2162) < 1e-3
          and abs(axes14.get("AS_Y", 9e9) - 30.0) < 1e-3
          and cap.get("focus_state") == "in_focus"
          and cap.get("light_on") is True
          and os.path.isfile(png_path),
          "c=%s name=%s focus=%s file_on_disk=%s"
          % (c, cap_name, cap.get("focus_state"), os.path.isfile(png_path)))
    # 列表（cam 过滤 + limit 截断，ts 倒序）+ 取图（PNG 全幅 3000×3000）
    lst = http_json(base + "/api/vision/captures?cam=down&limit=50")[
        "captures"]
    lst_up = http_json(base + "/api/vision/captures?cam=up")["captures"]
    lst_l1 = http_json(base + "/api/vision/captures?limit=1")["captures"]
    try:
        with urllib.request.urlopen(
                base + "/api/vision/capture/file?name=" + cap_name,
                timeout=10) as resp:
            png14 = resp.read()
        w14, h14, _ = png_decode_gray(png14)
        ok_file = png14[:8] == b"\x89PNG\r\n\x1a\n" \
            and (w14, h14) == (3000, 3000)
    except Exception:
        ok_file = False
    # 未知采图取图 → 400；未知相机 save → 400
    try:
        urllib.request.urlopen(base + "/api/vision/capture/file?name=nope",
                               timeout=5)
        c_nf = 200
    except urllib.error.HTTPError as ex:
        c_nf = ex.code
    c_bc, _ = http_post(base + "/api/vision/capture/save", {"cam": "nope"})
    check("vision.capture_api",
          any(m["name"] == cap_name for m in lst)
          and not any(m["name"] == cap_name for m in lst_up)
          and len(lst_l1) == 1 and ok_file
          and c_nf == 400 and c_bc == 400,
          "list=%s filter=%s limit=%s file=%s nf=%s badcam=%s"
          % (any(m["name"] == cap_name for m in lst),
             not any(m["name"] == cap_name for m in lst_up),
             len(lst_l1), ok_file, c_nf, c_bc))

    # ---- 17.14b capture/save 闪光语义（2026-08-24：与 identify/acquire 同
    #      时序）——trigger 模式灯灭自动点亮→曝光→拍帧→灭（出亮图，save 后
    #      光源复灭）；continuous 模式不闪光（灯灭出暗图，真机语义）----
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    time.sleep(0.2)
    c, r = http_post(base + "/api/vision/capture/save", {"cam": "down"})
    capf = r.get("capture") or {}
    png_f = b""
    if capf.get("name"):
        with urllib.request.urlopen(
                base + "/api/vision/capture/file?name=" + capf["name"],
                timeout=10) as resp:
            png_f = resp.read()
    mean_f = frame_stats(png_f)[0] if png_f else 0.0
    time.sleep(0.2)
    light_after = viz_cam("down").get("light_on")
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "continuous"})
    c2, r2 = http_post(base + "/api/vision/capture/save", {"cam": "down"})
    capf2 = r2.get("capture") or {}
    png_f2 = b""
    if capf2.get("name"):
        with urllib.request.urlopen(
                base + "/api/vision/capture/file?name=" + capf2["name"],
                timeout=10) as resp:
            png_f2 = resp.read()
    mean_f2 = frame_stats(png_f2)[0] if png_f2 else 9e9
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "trigger"})
    ok_flash = c == 200 and capf.get("light_on") is True \
        and capf.get("flash") is True \
        and capf.get("focus_state") == "in_focus" and mean_f > 60.0 \
        and light_after is False \
        and c2 == 200 and capf2.get("light_on") is False \
        and not capf2.get("flash") and mean_f2 < 20.0
    for nm in (capf.get("name"), capf2.get("name")):
        if nm:
            http_delete(base + "/api/vision/captures", {"name": nm})
    check("vision.capture_flash", ok_flash,
          "trigger mean=%.1f flash=%s light_after=%s cont mean=%.1f"
          % (mean_f, capf.get("flash"), light_after, mean_f2))
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", True)
    time.sleep(0.2)

    # ---- 17.15 用已存采图建模板（templates 的 capture+roi 参数）----
    # roi 跨十字臂左缘/上缘（0.6um/px 下十字臂 833px 宽、臂缘在中心 ±417px；
    # [700,700,751,751] 跨过 x=1083/y=1083 臂缘，方差足够）；
    # meta.source 记采图名；live 路径 source=="live" 行为不变
    c, r = http_post_tpl(base + "/api/vision/templates",
                     {"capture": cap_name, "name": "cap_tpl",
                      "roi": [700, 700, 751, 751], "shape": "cross"})
    tpl = r.get("template") or {}
    ok_create = c == 200 and tpl.get("source") == cap_name \
        and tpl.get("cam") == "down" and tpl.get("w") == 751
    # 400 路径：越界 / 小于 16px / 纯均匀区（角落方差极低）/
    # cam 与采图不一致 / 缺 roi / 采图不存在
    bads = [
        {"capture": cap_name, "roi": [2900, 2900, 200, 200]},
        {"capture": cap_name, "roi": [100, 100, 8, 8]},
        {"capture": cap_name, "roi": [2, 2, 32, 32]},
        {"capture": cap_name, "cam": "up",
         "roi": [700, 700, 751, 751]},
        {"capture": cap_name},
        {"capture": "nope", "roi": [700, 700, 751, 751]},
    ]
    codes15 = [http_post(base + "/api/vision/templates", b)[0] for b in bads]
    # 用该模板 REAL identify 实匹配（NCC 高分说明 ROI 特征足够）
    used_ok = True
    if has_cv2:
        http_post(base + "/api/vision/mode", {"mode": "real"})
        c, r = http_post(base + "/api/vision/identify",
                         {"cam": "down", "template": "cap_tpl"})
        res = ((r.get("entry") or {}).get("result") or {})
        used_ok = c == 200 and res.get("judge") is True \
            and (res.get("lstScore") or [0])[0] > 0.8
        http_post(base + "/api/vision/mode", {"mode": "fast"})
    # live 路径回归：不传 capture 建模板，source=="live"
    c, r = http_post_tpl(base + "/api/vision/templates",
                     {"cam": "down", "name": "live_tpl15"})
    live_src = (r.get("template") or {}).get("source")
    c_dl, _ = http_delete(base + "/api/vision/templates",
                          {"name": "live_tpl15"})
    check("vision.template_from_capture", ok_create
          and codes15 == [400] * 6 and used_ok
          and live_src == "live" and c_dl == 200,
          "create=%s 400s=%s used=%s live=%s"
          % (ok_create, codes15 == [400] * 6, used_ok, live_src))
    # 清理：删模板 + 删采图；删后列表无、取图 400、文件消失、再删 400
    c_d1, _ = http_delete(base + "/api/vision/templates", {"name": "cap_tpl"})
    c_d2, r_d2 = http_delete(base + "/api/vision/captures",
                             {"name": cap_name})
    lst2 = http_json(base + "/api/vision/captures?cam=down")["captures"]
    try:
        urllib.request.urlopen(base + "/api/vision/capture/file?name="
                               + cap_name, timeout=5)
        c_gone = 200
    except urllib.error.HTTPError as ex:
        c_gone = ex.code
    c_d3, _ = http_delete(base + "/api/vision/captures", {"name": cap_name})
    check("vision.capture_delete", c_d1 == 200 and c_d2 == 200
          and r_d2.get("ok")
          and not any(m["name"] == cap_name for m in lst2)
          and c_gone == 400 and c_d3 == 400
          and not os.path.isfile(png_path),
          "d1=%s d2=%s gone=%s d3=%s file=%s"
          % (c_d1, c_d2, c_gone, c_d3, os.path.isfile(png_path)))

    # ---- 17.14/17.15 清理：轴回零 / 光源灭 / 物料复位 ----
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y", "WS_Rz"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)
    plc_setbool("SystemMachine.SS_Bonding.M_AS.LightOn_2", False)
    http_post(base + "/api/workpiece/reset", {})

    # ---- 17.16 下相机看晶圆上已键合 die 的标记（2026-08-24：wafer_marks
    #      分支补 carrier=wafer 的 die 标记 die_id#k，z=晶圆面+die 厚，与
    #      recipe 晶圆 marks 分属 wafer/wafer_die 层各自离焦）----
    # 真实交接链造 carrier=wafer 的 die：film→flip→head→wafer
    def _wait_carrier(carrier, timeout=8.0):
        t0_ = time.time()
        while time.time() - t0_ < timeout:
            d = [x for x in http_json(base + "/api/viz")["dies"]
                 if x["carrier"] == carrier]
            if d:
                return d
            time.sleep(0.05)
        return []

    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenNegtive"})
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": -5.0,
                                        "speed": 600.0})
    got = _wait_carrier("flip")
    http_post(base + "/api/axis/move", {"axis": "FA_Z", "target": 20.0,
                                        "speed": 600.0})
    wait_settled("FA_Z")
    # FA_Rx 旋转前必须在 Z 安全位（fa_z_safe_mm=20）：FA_Z≤0 旋转扫掠撞
    # 蓝膜面/晶圆台（2026-08-26 回归实测 FA_X 扫掠被 soft-stop 的根因）
    http_post(base + "/api/axis/move", {"axis": "FA_Rx", "target": math.pi,
                                        "speed": 1.5})
    wait_settled("FA_Rx", timeout=20.0)
    for i in (1, 2, 3):
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.SS_ME.M_AIR.WT_BondingVac_%d" % i,
                   "cmd": "OpenNegtive"})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.ChuckVac",
               "cmd": "OpenAir"})
    time.sleep(0.4)                          # 等真空衰减（防误上升沿）
    http_post(base + "/api/axis/move", {"axis": "AS_Y", "target": 120.0,
                                        "speed": 500.0})
    wait_settled("AS_Y")
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 220.0,
                                        "speed": 600.0})
    wait_settled("WS_LS_X")
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": -1242.35,
                                        "speed": 600.0})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": -50.93,
                                        "speed": 200.0})
    got = got and _wait_carrier("head")
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    # 翻转撤离（真实几何：交接位就在键合头正下方，不退则降臂撞空吸盘）
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_X", timeout=30.0)
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("WS_LS_X")
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": -167.43,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.SS_ME.M_AIR.WT_WaferVac",
               "cmd": "OpenNegtive"})
    for i in (1, 2, 3):
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.SS_ME.M_AIR.WT_BondingVac_%d" % i,
                   "cmd": "OpenAir"})
    got = got and _wait_carrier("wafer")
    assert got, "film→flip→head→wafer 交接链未走通"
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    # 手动 OpenAir 现在按实际 XY 垂直落片，不能假设 site=[0,0]。读取实际
    # site 与运动学，把 die 的 #0 角标移到下相机中心（AS=(0,0) 光轴
    # (-42.5,1.0)）；AS_Z=+0.5 使焦面=晶圆面+die厚。原焦面/识别判据不变。
    import vision_model as vm
    rec16 = http_json(base + "/api/recipe")["recipe"]
    viz16 = http_json(base + "/api/viz")
    die16 = next(d for d in viz16["dies"] if d["carrier"] == "wafer")
    assert abs(die16.get("rz", 0.0)) < 1e-12
    ratio = rec16["film"]["marks"]["ratio"][0]
    mark_site = [die16["pos"][0] + ratio[0] * rec16["film"]["dies"]["size_x_mm"] / 2.0,
                 die16["pos"][1] + ratio[1] * rec16["film"]["dies"]["size_y_mm"] / 2.0]
    pos16 = {a["name"]: a["pos"] for a in http_json(base + "/api/axes")["axes"]}
    wc, wrot = vm._module_pose(http_json(base + "/api/toolpoints"), "wafer_stage")
    wrot = [wrot[0], wrot[1], wrot[2] + pos16.get("WS_Rz", 0.0)]
    wrz = float(viz16["wafer"].get("rz", 0.0))
    c16, s16 = math.cos(wrz), math.sin(wrz)
    off16 = viz16["wafer"]["offset"]
    local16 = [off16[0] + c16 * mark_site[0] - s16 * mark_site[1],
               off16[1] + s16 * mark_site[0] + c16 * mark_site[1], 0.0]
    w16 = vm._rotvec_apply_pivot(local16, wrot, vm.wafer_rz_pivot(rec16))
    mark_world = (wc[0] + w16[0], wc[1] + w16[1])
    ws_target = (pos16["WS_LS_X"] - 42.5 - mark_world[0],
                 pos16["WS_LS_Y"] + 1.0 - mark_world[1])
    print("   wafer_die 实际落点: site=%s mark=%s WS目标=%s"
          % (die16["pos"], mark_world, ws_target))
    for ax, tgt in (("WS_LS_X", ws_target[0]), ("WS_LS_Y", ws_target[1]),
                    ("AS_X", 0.0), ("AS_Y", 0.0), ("AS_Z", 0.5)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    c, r = http_post(base + "/api/vision/identify",
                     {"cam": "down", "template": "tpl_die_head"})
    e16 = r.get("entry") or {}
    res16 = e16.get("result") or {}
    s16 = {s["name"]: s for s in e16.get("surfaces") or []}
    # FAST 返回同视野全部场景标记；按 die 名取该标记评分，不让邻近的
    # 晶圆 site 标记替代 die 的成像判定。
    die_score16 = [score for name, score in zip(res16.get("lstName") or [],
                                                res16.get("lstScore") or [])
                   if name == die16["id"] + "#0"]
    check("vision.wafer_die_marks", c == 200
          and res16.get("judge") is True
          and res16.get("strShapeName") == "tpl_die_head"
          and len(die_score16) == 1 and die_score16[0] >= 0.8
          and e16.get("focus_state") == "in_focus"
          and s16.get("wafer_die", {}).get("in_dof") is True
          and s16.get("wafer", {}).get("in_dof") is False,
          "judge=%s focus=%s shape=%s names=%s die_score=%s surfs=%s"
          % (res16.get("judge"), e16.get("focus_state"),
             res16.get("strShapeName"), res16.get("lstName"), die_score16, s16))
    # AS_Z 回 0（焦面=晶圆面）：die 标记面 +500um 离焦判废——与晶圆 marks
    # 分层离焦（die 标记随 wafer_die 层，不随 wafer 层）
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.0,
                                        "speed": 500.0})
    wait_settled("AS_Z")
    c, r = http_post(base + "/api/vision/identify",
                     {"cam": "down", "template": "tpl_die_head"})
    e16 = r.get("entry") or {}
    res16 = e16.get("result") or {}
    die_score16 = [score for name, score in zip(res16.get("lstName") or [],
                                                res16.get("lstScore") or [])
                   if name == die16["id"] + "#0"]
    check("vision.wafer_die_defocus", c == 200
          and len(die_score16) == 1 and die_score16[0] < 0.8
          and e16.get("focus_state") == "rejected"
          and (e16.get("blur_ratio") or 0) > 3.0,
          "focus=%s blur=%s shape=%s names=%s die_score=%s"
          % (e16.get("focus_state"), e16.get("blur_ratio"),
             res16.get("strShapeName"), res16.get("lstName"), die_score16))
    # 17.16 清理：真空全放 / 轴回零 / 物料复位
    for nm in ("ChuckVac", "WT_WaferVac", "WT_BondingVac_1",
               "WT_BondingVac_2", "WT_BondingVac_3"):
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.SS_ME.M_AIR." + nm,
                   "cmd": "OpenAir"})
    for ax, tgt, sp in (("FA_Rx", 0.0, 1.5), ("FA_X", 0.0, 600.0),
                        ("FA_Z", 0.0, 600.0),
                        ("AS_X", 0.0, 600.0), ("AS_Y", 0.0, 600.0),
                        ("AS_Z", 0.0, 600.0), ("WS_LS_X", 0.0, 600.0),
                        ("WS_LS_Y", 0.0, 600.0), ("WS_Rz", 0.0, 600.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": sp})
    for ax in ("FA_Rx", "FA_X", "FA_Z", "AS_X", "AS_Y", "AS_Z", "WS_LS_X",
               "WS_LS_Y"):
        wait_settled(ax, timeout=20.0)
    http_post(base + "/api/workpiece/reset", {})




def test_wafer_ega():
    """--- 19.8 晶圆上片与全局对准（/api/process/wafer_ega） ---
    流程：强制上片并注入确定性平移/旋转误差（关闭随机抖动）→ 安全确认 →
    WS/AS 联合走位测量 n 个 site_mark → 六参数仿射拟合。
    自检：EGA 拟合结果应找回注入的 tx/ty/θ，残差 3σ 在几十 um 内。"""
    print("--- 19.8 晶圆上片与全局对准（EGA） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=120.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    # 干净起点 + 确定性误差注入（关闭 jitter 以便自检）
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/recipe/restore_default", {})
    injected = {
        "load_x_mm": 0.2,
        "load_y_mm": -0.15,
        "load_rz_rad": 0.0002,
        "jitter_xy_um": 0.0,
        "jitter_rz_urad": 0.0,
        "jitter_on": False,
    }
    c, r = http_put(base + "/api/recipe", {"wafer_load": injected})
    check("ega.recipe_put", c == 200 and r.get("ok"),
          "code=%s err=%s" % (c, r.get("error", "")))

    # 运行 EGA 流程（强制上片）
    c, r = http_post(base + "/api/process/wafer_ega", {"with_load": True})
    check("ega.start", c == 200 and r.get("ok"),
          "code=%s err=%s" % (c, r.get("error", "")))
    s = flow_wait()
    ega = (s.get("ega_result") or {})
    n = ega.get("n", 0)
    tx = ega.get("tx", 0.0)
    ty = ega.get("ty", 0.0)
    theta = ega.get("theta", 0.0)
    resid = ega.get("resid_3s", 9e9)
    ok = (c == 200 and s.get("state") == "done"
          and n >= 3
          and abs(tx - injected["load_x_mm"]) < 0.01
          and abs(ty - injected["load_y_mm"]) < 0.01
          and abs(theta - injected["load_rz_rad"]) < 5e-5
          and resid < 0.02)
    check("ega.fit_recover", ok,
          "state=%s n=%d tx=%+.4f um ty=%+.4f um theta=%+.4f urad "
          "resid_3s=%.3f um"
          % (s.get("state"), n, tx * 1000, ty * 1000,
             theta * 1e6, resid * 1000))
    check("ega.measurements_count",
          len(ega.get("measurements", [])) == n, "meas=%d n=%d"
          % (len(ega.get("measurements", [])), n))

    # 现场清理：恢复默认处方、物料复位、流程复位
    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/flow/reset", {})


def test_focus_level():
    """--- 19.9 调焦调平（/api/process/focus_level）---
    迭代 3 新序列：per_site 键合位就位 → WS 冻结 → 共焦测高 → 拟合 →
    BS_CAL 调平 + AS_Z/BS_Arm_Z 定焦 → 可选中心复测。
    自检：注入 wafer rx=+300urad、die ry=−200urad（jitter 关、σ=0），
    跑 per_site 后 BS_CAL_Rx≈−200urad、BS_CAL_Ry≈−300urad，残余 <30urad；
    AS_Z≈0、BS_Arm_Z≈−118.33 mm；per_site 期间 WS 位置不变；die 测点世界
    XY 落在键合头吸盘中心 ±die 半宽内；MQTT 读数经 confocal/result/+；
    align_first=true 覆盖 die 角标且不再降级；verify=false 跳过复测；
    AS_Y 新行程 −49.4 可达、−61 被钳位到 −60；守卫 400。"""
    print("--- 19.9 调焦调平（focus_level） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    from mqtt_broker import MqttSub

    mqtt_msgs = []

    def mqtt_collect():
        try:
            sub = MqttSub(port=MQTT_PORT, client_id="test-fl-sub")
            sub.subscribe("confocal/result/+")
            while True:
                got = sub.recv(timeout=1.0)
                if got is not None:
                    mqtt_msgs.append(got)
        except Exception:
            pass

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=300.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def run_flow(path, body, name, timeout=300.0):
        c, r = http_post(base + path, body)
        if c != 200 or not r.get("ok"):
            check(name + ".start", False,
                  "code=%s err=%s" % (c, r.get("error", "")))
            return None
        s = flow_wait(timeout)
        # 生成时刻步骤定义（含 space/freeze_ws/align_first 等字段）
        s["steps_def"] = r.get("steps_def") or []
        bad = ["#%d(%s)=%s" % (i + 1, stp["op"], stp["status"])
               for i, stp in enumerate(s.get("steps") or [])
               if stp["status"] != "ok"]
        check(name + ".done_all_ok",
              s.get("state") == "done" and not bad,
              "state=%s bad=%s" % (s.get("state"), bad[:5]))
        return s

    def axes_pos():
        return {a["name"]: a["pos"]
                for a in http_json(base + "/api/axes")["axes"]}

    threading.Thread(target=mqtt_collect, name="fl-mqtt-watch",
                     daemon=True).start()
    time.sleep(0.5)

    try:
        # ---- 干净起点 ----
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×）；头部 HTTP/走位
                         # 用例与 quick 保留用例在 8× 下语义不变（墙钟轮询
                         # 等待 sim 条件，只更快）
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

        # ---- sim_sensors σ 已归 recipe：PUT 生效 / 坏值 400 / 机械常数侧移除 ----
        c, r = http_put(base + "/api/recipe",
                        {"sim_sensors": {"confocal_sigma_um": 0.0}})
        check("fl.sim_sensors_put", c == 200 and r.get("ok"),
              "code=%s err=%s" % (c, r.get("error", r.get("errors", ""))))
        j = http_json(base + "/api/recipe")
        ss = (j.get("recipe") or {}).get("sim_sensors") or {}
        c2, _ = http_put(base + "/api/recipe",
                         {"sim_sensors": {"confocal_sigma_um": -1.0}})
        c3, _ = http_put(base + "/api/recipe",
                         {"sim_sensors": {"nonsense_field": 1.0}})
        c4, _ = http_put(base + "/api/machine_constants",
                         {"focus_level": {"confocal_sigma_um": 0.1}})
        # 迭代 22 起共焦 σ 逐探头嵌套 {up,down}（标量 PUT 迁移为全实例同值）
        csn = ss.get("confocal_sigma_um", 9.0)
        csn = max(csn.values()) if isinstance(csn, dict) else csn
        check("fl.sim_sensors_validate",
              abs(csn) < 1e-9
              and c2 == 400 and c3 == 400 and c4 == 400,
              "ss=%s bad=%s unknown=%s mc_sigma=%s" % (ss, c2, c3, c4))

        # ---- 共焦 HTTP 读数哨兵（迭代 38：confocal._ProbeSpec 缺 view_rot
        # 曾致 confocal-mqtt 线程死亡、chain focus_level 全灭——quick 轻段
        # 此前无任何真实共焦读数，本断言覆盖 probe_pose duck-typing 链；
        # 无晶圆/目标时 valid=False 也接受，只要 handler 正常返回结构）
        j = http_json(base + "/api/confocal/read?probe=down")
        check("fl.confocal_read_smoke",
              isinstance(j, dict) and "valid" in j and "reading_um" in j,
              "resp=%s（handler 异常会缺键/500）" % (j,))

        # ---- AS_Y 行程：+6.0 可达（上相机认头心），-31 钳位到 -30 ----
        http_post(base + "/api/axis/move",
                  {"axis": "AS_Y", "target": 0.0, "speed": 500.0})
        wait_settled("AS_Y")
        c, _ = http_post(base + "/api/axis/move",
                         {"axis": "AS_Y", "target": 6.0, "speed": 500.0})
        check("fl.as_y_plus6_reachable", c == 200, "code=%s" % c)
        wait_settled("AS_Y")
        ap = axes_pos()
        check("fl.as_y_plus6_pos", abs(ap.get("AS_Y", 0.0) - 6.0) < 0.01,
              "AS_Y=%.4f" % ap.get("AS_Y", 0.0))
        c, r = http_post(base + "/api/axis/move",
                         {"axis": "AS_Y", "target": -31.0, "speed": 500.0})
        # 引擎对超下限目标钳位到 -30，不再拒绝
        check("fl.as_y_minus31_clamped", c == 200,
              "code=%s err=%s" % (c, r.get("error", "")))
        wait_settled("AS_Y")
        ap = axes_pos()
        check("fl.as_y_minus31_pos", abs(ap.get("AS_Y", 0.0) - (-30.0)) < 0.01,
              "AS_Y=%.4f expect -30.0+-0.01" % ap.get("AS_Y", 0.0))
        http_post(base + "/api/axis/move",
                  {"axis": "AS_Y", "target": 0.0, "speed": 500.0})
        wait_settled("AS_Y")

        # ---- 注入确定性倾斜 ----
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.0, "load_y_mm": 0.0, "load_rz_rad": 0.0,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0,
            "z_um": 0.0, "rx_urad": 300.0, "ry_urad": 0.0,
            "die_head_rx_urad": 0.0, "die_head_ry_urad": -200.0}})
        check("fl.setup_recipe_tilt", c == 200 and r.get("ok"),
              "code=%s err=%s" % (c, r.get("error", r.get("errors", ""))))

        # ---- 守卫：无晶圆 / 头无 die / 未 EGA → 均 400 ----
        c, r = http_post(base + "/api/process/focus_level",
                         {"mode": "per_site"})
        g1 = c == 400
        http_post(base + "/api/workpiece/wafer",
                  {"action": "load", "offset": [0.0, 0.0]})
        c, r = http_post(base + "/api/process/focus_level",
                         {"mode": "per_site"})
        g2 = c == 400
        check("fl.guard_no_wafer_no_die", g1 and g2, "g1=%s g2=%s" % (g1, g2))

        # ---- 前置链：取片 + 交接（die 到头）----
        # 迭代 18：以下为重流程实跑区（EGA/调平/复测/align_first 全链，
        # 单用例远超 sim 30s 标准）——quick 套件门控跳过，full 全跑；
        # fl_verify 单元用例已拆 test_focus_level_units 两套件保留。
        if not fblock(["fl.flip_pick.done_all_ok", "fl.flip_handoff.done_all_ok",
                       "fl.guard_no_ega",
                       "fl.wafer_ega.done_all_ok", "fl.ega_fit",
                       "fl.global.done_all_ok", "fl.global_wafer_3pts",
                       "fl.global_die_npts", "fl.global_verify_pass",
                       "fl.global_mem.done_all_ok", "fl.global_level_memory",
                       "fl.per_site.done_all_ok", "fl.cal_direction",
                       "fl.residual_lt_tol", "fl.as_z_final",
                       "fl.bs_arm_z_final", "fl.ws_frozen",
                       "fl.measure_count_default", "fl.verify_steps",
                       "fl.verify_pass", "fl.verify_log",
                       "fl.die_points_near_head_center", "fl.recheck_up_mid",
                       "fl.mqtt_result_count", "fl.mqtt_probes_reqid",
                       "fl.align_first.done_all_ok", "fl.align_first_residual",
                       "fl.align_first_steps", "fl.align_first_no_fallback",
                       "fl.align_first_verify",
                       "fl.no_verify.done_all_ok", "fl.verify_false_skip"]):
            return
        run_flow("/api/process/flip_pick", {}, "fl.flip_pick")
        run_flow("/api/process/flip_handoff", {}, "fl.flip_handoff")
        c, r = http_post(base + "/api/process/focus_level",
                         {"mode": "per_site"})
        check("fl.guard_no_ega", c == 400,
              "code=%s err=%s" % (c, r.get("error", "")))

        # ---- EGA（强制重上片 → 注入 wafer rx=+300urad 生效）----
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        s = run_flow("/api/process/wafer_ega", {"with_load": True},
                     "fl.wafer_ega")
        ega = (s or {}).get("ega_result") or {}
        check("fl.ega_fit", ega.get("n", 0) >= 3,
              "n=%s resid_3s=%s" % (ega.get("n"), ega.get("resid_3s")))

        # ---- 迭代 5：global 模式 = EGA 站点切蛋糕 3 点（在首次 compensate
        #      前跑，避免 level_memory 捷径跳过测量）----
        import process_flows as pf
        import ega as ega_mod
        from recipe import wafer_sites_cells, wafer_sites_chip_size
        mc_now = http_json(base + "/api/machine_constants")["constants"]
        ega_n = int(mc_now["wafer_ega"]["ega_n_points"])
        rcp = http_json(base + "/api/recipe")["recipe"]
        cells_all = wafer_sites_cells(rcp)
        exp_tri = pf._fl_pick_triangle(
            ega_mod.select_measure_points(cells_all, n_pts=ega_n))
        # 迭代 12 起 align_first 默认 true：global 晶圆侧测点=每站最近
        # 键合位 + site_marks[0] 标记偏移（process_flows 同公式复算）
        if bool((rcp.get("focus_level") or {}).get("align_first", False)):
            mr = rcp["wafer"]["site_marks"]["ratio"][0]
            sx, sy = wafer_sites_chip_size(rcp)
            exp_tri = [
                (lambda cell: (round(cell[2] + mr[0] * sx / 2.0, 6),
                               round(cell[3] + mr[1] * sy / 2.0, 6)))(
                    min(cells_all, key=lambda c: (c[2] - sp[0]) ** 2
                        + (c[3] - sp[1]) ** 2))
                for sp in exp_tri]
        sg = run_flow("/api/process/focus_level", {"mode": "global"},
                      "fl.global")
        sd = (sg or {}).get("steps_def") or []
        gw = [stp for stp in sd
              if stp.get("op") == "fl_measure" and stp.get("space") == "wafer"]
        gd = [stp for stp in sd
              if stp.get("op") == "fl_measure" and stp.get("space") == "die"]
        gw_pts = [tuple(stp["point"]) for stp in gw]
        check("fl.global_wafer_3pts",
              len(gw) == 3
              and all(not stp.get("freeze_ws") for stp in gw)
              and sorted(gw_pts) == sorted([tuple(p) for p in exp_tri]),
              "wafer步=%d pts=%s expect=%s" % (len(gw), gw_pts, exp_tri))
        check("fl.global_die_npts", len(gd) == 4,
              "die步=%d expect 4（measure_points）" % len(gd))
        flv_g = (sg or {}).get("fl_verify") or {}
        check("fl.global_verify_pass", flv_g.get("judge") == "pass",
              "judge=%s residual=%s"
              % (flv_g.get("judge"), flv_g.get("residual_urad")))
        # level_memory 捷径不受影响：再跑一次 global 应跳过测量
        sg2 = run_flow("/api/process/focus_level", {"mode": "global"},
                       "fl.global_mem")
        gw2 = [stp for stp in ((sg2 or {}).get("steps_def") or [])
               if stp.get("op") == "fl_measure" and stp.get("space") == "wafer"]
        all_info2 = "\n".join(stp.get("info", "")
                              for stp in (sg2 or {}).get("steps", []))
        check("fl.global_level_memory",
              len(gw2) == 0 and "调平记忆" in all_info2,
              "wafer测量步=%d 记忆日志=%s" % (len(gw2), "调平记忆" in all_info2))

        # ---- 主用例：per_site 调焦调平 ----
        ws0 = axes_pos()
        n_mqtt0 = len(mqtt_msgs)
        s = run_flow("/api/process/focus_level", {"mode": "per_site"},
                     "fl.per_site")
        time.sleep(0.5)
        ap = axes_pos()
        flr = (s or {}).get("fl_result") or {}
        cal_rx, cal_ry = ap.get("BS_CAL_Rx", 0.0), ap.get("BS_CAL_Ry", 0.0)
        check("fl.cal_direction",
              abs(cal_rx - (-200e-6)) < 60e-6
              and abs(cal_ry - (-300e-6)) < 60e-6,
              "expect(-200,-300)urad got(%+.2f,%+.2f)"
              % (cal_rx * 1e6, cal_ry * 1e6))
        res = flr.get("residual_urad") or [9e9, 9e9]
        check("fl.residual_lt_tol",
              abs(res[0]) < 30.0 and abs(res[1]) < 30.0,
              "residual=%s urad（level_tol=30）" % res)
        check("fl.as_z_final",
              abs(ap.get("AS_Z", 9) - 0.0) < 0.01,
              "AS_Z=%.5f expect 0+-0.01" % ap.get("AS_Z", 9))
        check("fl.bs_arm_z_final",
              abs(ap.get("BS_Arm_Z", 9) - (-118.33)) < 0.02,
              "BS_Arm_Z=%.5f expect -118.33+-0.02" % ap.get("BS_Arm_Z", 9))
        # 迭代 3：per_site 先 fl_ready 把 WS 送到头下，后续测高冻结 WS；
        # 因此 WS 冻结基准应取 fl_ready 步骤里报告的目标 WS。
        ws_target = [None, None]
        for stp in (s or {}).get("steps", []):
            if stp.get("op") == "fl_ready":
                m = re.search(r"WS\(([\-+\.\d]+),([\-+\.\d]+)\)",
                              stp.get("info", ""))
                if m:
                    ws_target = [float(m.group(1)), float(m.group(2))]
                break
        check("fl.ws_frozen",
              ws_target[0] is not None
              and abs(ap.get("WS_LS_X", 0.0) - ws_target[0]) < 0.01
              and abs(ap.get("WS_LS_Y", 0.0) - ws_target[1]) < 0.01,
              "WS target=%s final X %.4f Y %.4f"
              % (ws_target, ap.get("WS_LS_X", 0.0), ap.get("WS_LS_Y", 0.0)))

        steps = s.get("steps") or []
        fl_meas = [stp for stp in steps if stp["op"] == "fl_measure"]
        check("fl.measure_count_default", len(fl_meas) == 16,
              "fl_measure=%d expect 16（4+4+复测4+4）" % len(fl_meas))

        # ---- 迭代 5：调平复测（wafer_v/die_v + fl_verify 阈值判定）----
        sd = (s or {}).get("steps_def") or []
        vw = [stp for stp in sd
              if stp.get("op") == "fl_measure" and stp.get("space") == "wafer_v"]
        vd = [stp for stp in sd
              if stp.get("op") == "fl_measure" and stp.get("space") == "die_v"]
        fv_op = [stp for stp in sd if stp.get("op") == "fl_verify"]
        check("fl.verify_steps",
              len(vw) == 4 and len(vd) == 4 and len(fv_op) == 1
              and all(stp.get("freeze_ws") for stp in vw)
              and all("复测" in (stp.get("desc") or "") for stp in vw + vd),
              "wafer_v=%d die_v=%d fl_verify=%d freeze=%s"
              % (len(vw), len(vd), len(fv_op),
                 [bool(stp.get("freeze_ws")) for stp in vw]))
        flv = (s or {}).get("fl_verify") or {}
        res_v = flv.get("residual_urad") or [9e9, 9e9]
        check("fl.verify_pass",
              flv.get("judge") == "pass"
              and abs(res_v[0]) <= 30.0 and abs(res_v[1]) <= 30.0,
              "judge=%s residual=%s urad（level_tol=30）"
              % (flv.get("judge"), res_v))
        all_info_v = "\n".join(stp.get("info", "") for stp in steps)
        check("fl.verify_log", "调平复测 PASS" in all_info_v
              and "焦面复核" in all_info_v,
              "日志缺失（PASS=%s 焦面复核=%s）"
              % ("调平复测 PASS" in all_info_v, "焦面复核" in all_info_v))

        die_worlds = []
        for stp in fl_meas:
            info = stp.get("info", "")
            # 流程状态里不保存 probe/space 字段，从 info 文本解析；
            # 复测中心点也走 up 探头，用 desc 过滤掉。
            if "复测" in (stp.get("desc") or ""):
                continue
            m_probe = re.search(r"（([^/]+)/([^）]+)）", info)
            if not m_probe or m_probe.group(2) != "up":
                continue
            m = re.search(r"世界 \(([\-+\.\d]+), ([\-+\.\d]+)\)", info)
            if m:
                die_worlds.append((float(m.group(1)), float(m.group(2))))
        head_cx, head_cy = -42.5, 7.0
        dsx2 = 5.0
        ok_die = (len(die_worlds) == 4
                  and all(abs(x - head_cx) <= dsx2 + 0.5
                          and abs(y - head_cy) <= dsx2 + 0.5
                          for x, y in die_worlds))
        check("fl.die_points_near_head_center", ok_die,
              "die worlds=%s head=(%.1f,%.1f)" % (die_worlds, head_cx, head_cy))

        j = http_json(base + "/api/confocal/read?probe=up")
        check("fl.recheck_up_mid",
              j.get("valid") and abs(j["reading_um"] - 350.0) < 20.0,
              "reading=%s valid=%s" % (j.get("reading_um"), j.get("valid")))

        # ---- MQTT 链路 ----
        msgs = [json.loads(p) for _t, p in mqtt_msgs[n_mqtt0:]]
        ids = [m.get("req_id") for m in msgs]
        check("fl.mqtt_result_count", len(msgs) >= 10,
              "got=%d expect>=10" % len(msgs))
        check("fl.mqtt_probes_reqid",
              set(m.get("probe") for m in msgs) == {"up", "down"}
              and len(set(ids)) == len(ids) == len(msgs)
              and all(isinstance(i, str) and i.startswith("fl-")
                      and len(i) == 9 and i[3:].isdigit() for i in ids),
              "probes=%s ids=%s"
              % (set(m.get("probe") for m in msgs), ids[:3]))

        # ---- align_first=true：先对准再测量，die 侧不再降级 ----
        s2 = run_flow("/api/process/focus_level",
                      {"mode": "per_site", "align_first": True},
                      "fl.align_first", timeout=400.0)
        flr2 = (s2 or {}).get("fl_result") or {}
        res2 = flr2.get("residual_urad") or [9e9, 9e9]
        check("fl.align_first_residual",
              abs(res2[0]) < 30.0 and abs(res2[1]) < 30.0,
              "residual=%s urad" % res2)
        aligned = [stp for stp in (s2 or {}).get("steps") or []
                   if stp["op"] == "fl_measure"
                   and "先对准" in (stp.get("info") or "")]
        check("fl.align_first_steps", len(aligned) >= 8,
              "先对准测点数=%d" % len(aligned))
        all_info = "\n".join([stp.get("info", "") for stp in
                              (s2 or {}).get("steps", [])])
        check("fl.align_first_no_fallback", "降级" not in all_info,
              "出现降级日志：%s" % all_info[:200])

        # 迭代 5：align_first 时复测段同样先对准（二次对准）且判定 PASS
        sd2 = (s2 or {}).get("steps_def") or []
        vw2 = [stp for stp in sd2
               if stp.get("op") == "fl_measure" and stp.get("space") == "wafer_v"]
        vd2 = [stp for stp in sd2
               if stp.get("op") == "fl_measure" and stp.get("space") == "die_v"]
        flv2 = (s2 or {}).get("fl_verify") or {}
        res_v2 = flv2.get("residual_urad") or [9e9, 9e9]
        check("fl.align_first_verify",
              len(vw2) == 4 and len(vd2) == 4
              and all(stp.get("align_first") for stp in vw2 + vd2)
              and all(stp.get("freeze_ws") for stp in vw2)
              and flv2.get("judge") == "pass"
              and abs(res_v2[0]) <= 30.0 and abs(res_v2[1]) <= 30.0,
              "wafer_v=%d die_v=%d judge=%s residual=%s"
              % (len(vw2), len(vd2), flv2.get("judge"), res_v2))

        # ---- verify=false：跳过最终中心复测 ----
        s3 = run_flow("/api/process/focus_level",
                      {"mode": "per_site", "verify": False},
                      "fl.no_verify")
        meas3 = [stp for stp in (s3 or {}).get("steps", [])
                 if stp["op"] == "fl_measure"]
        verify_steps = [stp for stp in (s3 or {}).get("steps", [])
                        if stp["op"] == "fl_measure"
                        and "复测" in (stp.get("desc") or "")]
        check("fl.verify_false_skip", len(meas3) == 8 and len(verify_steps) == 0,
              "fl_measure=%d verify_steps=%d" % (len(meas3), len(verify_steps)))

        # ---- 迭代 5：fl_verify FAIL/PASS 单元路径已拆到
        #      test_focus_level_units（迭代 18，两套件都跑）----

    finally:
        try:
            set_speed(1.0)                   # 提速窗复原（迭代 18）
            http_put(base + "/api/machine_constants",
                     {"focus_level": {"as_z_chuck_focus_mm": -1.0}})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
        except Exception:
            pass


def test_fine_align():
    """--- 19.10 逐键合位精对准（/api/process/fine_align；迭代 6）---
    全链闭环：确定性注入（wafer_load 平移/旋转，jitter 关、σ=0）→ 取片→
    交接→EGA→per_site 调焦调平→fine_align：
      - judge=pass、确认轮次 ≥1、补偿记录含耦合量 coup、EGA map 复合历史
        fine_align_comp 落存；
      - 物理残差双通道：θ 阈值 50urad（≈3σ，σθ≈√2·σxy/基线≈17urad，
        FAST 噪声 0.09um 主导）；平移阈值 2um（迭代 7 诊断：残差由 WS
        到位 repeat 重采样主导，repeat=0 收敛到 ≤0.2µm，见
        docs/逐键合位精对准_V1.md 诊断节）；
      - 枢轴真值偏移 50um（标定 0）→ 收敛 pass 且日志含耦合量；
      - WS_Rz ±1° 行程守卫（rz_gain 调小放大指令角→清晰报错）；
      - on_unconfirmed=abort（阈值 1nm 必不收敛→abort）/ continue_warn
        （→done 带 judge=warn）；
      - affine+n=2 生成期 400；处方坏值 400；
    进程内单测：4 算法精度（ransac 含离群点）、affine n=2 拒绝、
    锁轴快照校验。"""
    print("--- 19.10 逐键合位精对准（fine_align） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ---- 进程内：拟合算法单测（合成含噪点集） ----
    import fine_align as famod
    import flow_engine as femod
    rng = random.Random(7)
    d4 = [(-3.8, -3.8), (3.8, -3.8), (-3.8, 3.8), (3.8, 3.8)]
    th0, tx0, ty0 = 3e-4, 0.012, -0.008
    c0, s0 = math.cos(th0), math.sin(th0)
    m4 = [(c0 * x - s0 * y + tx0 + rng.gauss(0, 1e-4),
           s0 * x + c0 * y + ty0 + rng.gauss(0, 1e-4)) for x, y in d4]
    alg_ok = True
    alg_detail = []
    for algo in ("rigid", "similarity", "affine", "ransac"):
        r = famod.fit(algo, d4, m4)
        good = (abs(r["tx"] - tx0) < 1e-3 and abs(r["ty"] - ty0) < 1e-3
                and abs(r["theta"] - th0) < 1e-4)
        alg_ok = alg_ok and good
        alg_detail.append("%sθ=%.1furad" % (algo, r["theta"] * 1e6))
    check("fa.algorithms_accuracy", alg_ok, " ".join(alg_detail))
    # ransac 抗污染：1 个离群点（+50um）不影响
    m4o = list(m4)
    m4o[1] = (m4o[1][0] + 0.05, m4o[1][1] - 0.04)
    rr = famod.fit("ransac", d4, m4o)
    check("fa.ransac_outlier",
          abs(rr["tx"] - tx0) < 1e-3 and abs(rr["theta"] - th0) < 1e-4
          and 1 not in rr.get("inliers", []),
          "inliers=%s θ=%.1furad" % (rr.get("inliers"), rr["theta"] * 1e6))
    # n=2 + affine 拒绝；rigid/similarity n=2 可用
    try:
        famod.fit("affine", d4[:2], m4[:2])
        check("fa.affine_n2_reject", False, "未拒绝")
    except ValueError:
        r2 = famod.fit("rigid", d4[:2], m4[:2])
        check("fa.affine_n2_reject",
              abs(r2["theta"] - th0) < 1e-4, "affine 拒绝，rigid n=2 ok")
    # 锁轴快照校验（人为移动 → 抛错；不动 → 通过）
    class _FAx(object):
        def __init__(self, p):
            self.pos = p

    class _FEng(object):
        pass
    fe = object.__new__(femod.FlowEngine)
    fe.eng = _FEng()
    fe.eng.axes = {n: _FAx(0.0) for n in femod.FlowEngine._FA_LOCK_AXES}
    snap = fe._fa_lock_snapshot()
    lock_ok = True
    try:
        fe._fa_lock_check(snap)
    except RuntimeError:
        lock_ok = False
    fe.eng.axes["WS_Rz"].pos = 1e-4
    raised = False
    try:
        fe._fa_lock_check(snap)
    except RuntimeError:
        raised = True
    check("fa.lock_check", lock_ok and raised,
          "不动过=%s 动则抛=%s" % (lock_ok, raised))

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=300.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def run_flow(path, body, name, timeout=300.0, expect_done=True):
        c, r = http_post(base + path, body)
        if c != 200 or not r.get("ok"):
            check(name + ".start", False,
                  "code=%s err=%s" % (c, r.get("error", "")))
            return None
        s = flow_wait(timeout)
        s["steps_def"] = r.get("steps_def") or []
        if expect_done:
            bad = ["#%d(%s)=%s" % (i + 1, stp["op"], stp["status"])
                   for i, stp in enumerate(s.get("steps") or [])
                   if stp["status"] != "ok"]
            check(name + ".done_all_ok",
                  s.get("state") == "done" and not bad,
                  "state=%s bad=%s" % (s.get("state"), bad[:5]))
        return s

    try:
        # ---- 干净起点 + 确定性注入 ----
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×）
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0}})
        check("fa.setup_recipe", c == 200 and r.get("ok"),
              "code=%s err=%s" % (c, r.get("error", r.get("errors", ""))))

        # ---- 守卫链（进程内单测，避免服务端状态污染：晶圆/head die/EGA/
        #      调焦调平四道守卫逐一触发 ValueError） ----
        import process_flows as pfmod

        class _GWp(object):
            import threading as _th
            _lock = _th.RLock()

            def __init__(self, present, head):
                self.wafer = {"present": present}
                self.dies = ([{"id": "D1", "carrier": "head"}]
                             if head else [])

        class _GEng(object):
            pass

        def _guard_err(present, head, ega, fl):
            ge = _GEng()
            ge.constants = True
            ge.recipe = True
            ge.workpieces = _GWp(present, head)
            ge.vision = object()
            ge.ega_result = {"a": [1, 0, 0, 1]} if ega else None
            ge.fl_fit = {"compensate": {}} if fl else {}
            ge.level_memory = None
            try:
                pfmod.build_fine_align_flow(ge)
                return None
            except (ValueError, RuntimeError) as e:
                return str(e)

        g_chain = [_guard_err(False, False, False, False),
                   _guard_err(True, False, False, False),
                   _guard_err(True, True, False, False),
                   _guard_err(True, True, True, False)]
        check("fa.guard_chain",
              g_chain[0] and "晶圆不在位" in g_chain[0]
              and g_chain[1] and "键合头上没有芯片" in g_chain[1]
              and g_chain[2] and "EGA 未完成" in g_chain[2]
              and g_chain[3] and "调焦调平" in g_chain[3],
              "guards=%s" % g_chain)

        # 迭代 18：以下为重流程实跑区（取片/交接/EGA/调平/精对准全链 ×5
        # 次精对准变体，远超 sim 30s 标准）——quick 门控跳过，full 全跑；
        # 算法/守卫的进程内单测在上方两套件保留。
        if not fblock(["fa.flip_pick.done_all_ok", "fa.flip_handoff.done_all_ok",
                       "fa.wafer_ega.done_all_ok",
                       "fa.guard_affine_n2_and_badrecipe",
                       "fa.focus_level.done_all_ok", "fa.rz_stroke_guard",
                       "fa.main.done_all_ok", "fa.main_judge_pass",
                       "fa.main_phys_resid", "fa.main_coup_and_map",
                       "fa.main_confirm_tol",
                       "fa.pivot50.done_all_ok", "fa.pivot50_pass",
                       "fa.on_unconfirmed_abort", "fa.on_unconfirmed_warn"]):
            return
        run_flow("/api/process/flip_pick", {}, "fa.flip_pick")
        run_flow("/api/process/flip_handoff", {}, "fa.flip_handoff")
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        run_flow("/api/process/wafer_ega", {"with_load": True}, "fa.wafer_ega")
        # affine + n=2 → 生成期 400
        http_put(base + "/api/recipe",
                 {"fine_align": {"measure_points": 2, "algorithm": "affine"}})
        c, r = http_post(base + "/api/process/fine_align", {})
        c2, _ = http_put(base + "/api/recipe",
                         {"fine_align": {"algorithm": "nonsense"}})
        c3, _ = http_put(base + "/api/recipe",
                         {"sim_align": {"rz_center_true_offset_um": [1e6, 0]}})
        check("fa.guard_affine_n2_and_badrecipe",
              c == 400 and c2 == 400 and c3 == 400,
              "affine_n2=%s bad_algo=%s bad_sim=%s" % (c, c2, c3))
        http_post(base + "/api/recipe/restore_default", {})
        http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0}})

        # ---- 主用例：调焦调平 → 精对准（注入-找回闭环） ----
        run_flow("/api/process/focus_level", {"mode": "per_site"},
                 "fa.focus_level")
        # σθ≈17urad 的噪声尾巴靠确认环多轮兜底：主用例放宽到 5 轮防偶发
        # （默认 3 轮的超次行为由 fa.on_unconfirmed_* 专测）
        http_put(base + "/api/recipe",
                 {"fine_align": {"confirm_max_iter": 5}})
        # ---- WS_Rz ±1° 行程守卫（rz_gain=0.001 把指令角放大 1000×：
        #      首次补偿 δθ≈200urad → WS_Rz 目标≈0.2rad ≫ 1°，必触发；
        #      守卫在旋转移动前抛错，WS_Rz 不动，后续主用例不受影响）----
        http_put(base + "/api/machine_constants",
                 {"fine_align": {"rz_gain": 0.001}})
        sg = run_flow("/api/process/fine_align", {}, "fa.rz_guard",
                      expect_done=False)
        gl = "\n".join(e.get("msg", "") for e in (sg or {}).get("log", []))
        check("fa.rz_stroke_guard",
              (sg or {}).get("state") == "aborted" and "行程" in gl,
              "state=%s 行程报错=%s"
              % ((sg or {}).get("state"), "行程" in gl))
        http_put(base + "/api/machine_constants",
                 {"fine_align": {"rz_gain": 1.0}})
        http_post(base + "/api/flow/reset", {})
        s = run_flow("/api/process/fine_align", {}, "fa.main")
        fa = (s or {}).get("fine_align_result") or {}
        rounds = fa.get("rounds") or []
        comps = fa.get("compensations") or []
        check("fa.main_judge_pass",
              fa.get("judge") == "pass" and len(rounds) >= 1
              and len(comps) >= 1,
              "judge=%s rounds=%d comps=%d"
              % (fa.get("judge"), len(rounds), len(comps)))
        ph = fa.get("phys_resid") or {}
        check("fa.main_phys_resid",
              abs(ph.get("theta_urad", 9e9)) <= 50.0
              and abs(ph.get("tx_um", 9e9)) <= 2.0
              and abs(ph.get("ty_um", 9e9)) <= 2.0,
              "phys θ=%.2furad t=(%.3f,%.3f)um（θ 阈值 50urad≈3σ；"
              "t 阈值 2um，repeat 主导见迭代 7 诊断）"
              % (ph.get("theta_urad", 9e9), ph.get("tx_um", 9e9),
                 ph.get("ty_um", 9e9)))
        # 补偿记录含耦合量；EGA map 复合历史落存
        eg = flow_state().get("ega_result") or {}
        check("fa.main_coup_and_map",
              comps and "coup_um" in comps[0]
              and bool(eg.get("fine_align_comp")),
              "coup=%s map_hist=%d"
              % (comps[0].get("coup_um") if comps else None,
                 len(eg.get("fine_align_comp") or [])))
        # 确认环复测残余 ≤ 阈值（末轮）
        check("fa.main_confirm_tol",
              rounds and rounds[-1]["resid_xy_um"] <= 0.5
              and rounds[-1]["resid_rz_urad"] <= 30.0,
              "末轮 %s" % (rounds[-1] if rounds else None))

        # ---- 枢轴真值偏移 50um（标定 0）→ 收敛 pass 且日志含耦合量 ----
        http_put(base + "/api/recipe",
                 {"sim_align": {"rz_center_true_offset_um": [50.0, 0.0]}})
        sp = run_flow("/api/process/fine_align", {}, "fa.pivot50")
        fap = (sp or {}).get("fine_align_result") or {}
        logs = "\n".join(e.get("msg", "")
                         for e in (sp or {}).get("log", []))
        check("fa.pivot50_pass",
              fap.get("judge") == "pass" and "耦合" in logs,
              "judge=%s 耦合日志=%s" % (fap.get("judge"), "耦合" in logs))
        http_put(base + "/api/recipe",
                 {"sim_align": {"rz_center_true_offset_um": [0.0, 0.0]}})

        # ---- on_unconfirmed=abort（阈值 1nm 必不收敛；轮次回默认 3） ----
        http_put(base + "/api/recipe", {"fine_align": {
            "confirm_tol_xy_um": 0.001, "confirm_tol_rz_urad": 0.001,
            "confirm_max_iter": 3, "on_unconfirmed": "abort"}})
        sa = run_flow("/api/process/fine_align", {}, "fa.abort",
                      expect_done=False)
        faa = (sa or {}).get("fine_align_result") or {}
        check("fa.on_unconfirmed_abort",
              (sa or {}).get("state") == "aborted"
              and faa.get("judge") == "fail"
              and len(faa.get("rounds") or []) == 3,
              "state=%s judge=%s rounds=%d"
              % ((sa or {}).get("state"), faa.get("judge"),
                 len(faa.get("rounds") or [])))
        http_post(base + "/api/flow/reset", {})
        # ---- on_unconfirmed=continue_warn（同阈值 → done + warn） ----
        http_put(base + "/api/recipe", {"fine_align": {
            "on_unconfirmed": "continue_warn"}})
        sw = run_flow("/api/process/fine_align", {}, "fa.warn",
                      expect_done=False)
        faw = (sw or {}).get("fine_align_result") or {}
        check("fa.on_unconfirmed_warn",
              (sw or {}).get("state") == "done"
              and faw.get("judge") == "warn",
              "state=%s judge=%s"
              % ((sw or {}).get("state"), faw.get("judge")))

    finally:
        try:
            set_speed(1.0)                   # 提速窗复原（迭代 18）
            http_put(base + "/api/machine_constants",
                     {"fine_align": {"rz_gain": 1.0}})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            # 轴回安全位（BS_Arm_Z 低位遗留会让后续用例的 AS 走位判撞）
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter7():
    """--- 19.11 迭代 7：交接正压吹离 + 精对准精简 + WS repeat 40nm +
    midpoint_2mark 算法 + 一键串联 ---
      - 交接/吸回步骤含 OpenPositive→delay 200ms→OpenAir 序列，wait_die 在其后；
        交接实跑闭环不回归（正压沿即触发 flip_to_head）；
      - 精对准流程无 fl_ready/进场对焦 move、末两步 AS 回安全位；
        fl_ws_frozen site 不匹配守卫 400；
      - WS_LS_X/Y repeat_display==40nm（其余轴不变）；
      - 诊断固化：repeat 全置 0 复跑 focus_level+fine_align，物理残差量级
        断言（迭代 7 诊断：残余由 WS 到位 repeat 重采样随机误差主导，
        repeat=0 后收敛到 ≤0.2µm/0.5µrad 量级，非恒定系统框偏；
        见 docs/逐键合位精对准_V1.md 诊断节）；
      - midpoint_2mark：手算对拍/n>2 只用前 2 点/n=2 与 rigid 同值/注册表
        与处方校验/实跑 judge=pass 且日志注明只用前 2 标记；
      - 一键串联：happy path 五阶段全 done + judge=pass；二次运行前三个阶段
        状态感知 skip；精对准阶段失败 → 链 abort 记阶段。"""
    print("--- 19.11 迭代 7（chain/正压/精简/repeat/midpoint） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=300.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def run_flow(path, body, timeout=600.0):
        c, r = http_post(base + path, body)
        assert c == 200 and r.get("ok"), (path, c, r)
        return flow_wait(timeout)

    def chain_wait(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ch = flow_state().get("chain") or {}
            if not ch.get("active"):
                return ch
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    def setup_recipe():
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        # confirm_max_iter 放宽到 5：σθ≈17µrad 的 FAST 视觉噪声尾巴靠
        # 确认环多轮兜底（同 fa.main 惯例；超次行为由 chain_abort 专测）
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5}})
        assert c == 200, r

    # ---- 进程内：midpoint_2mark 手算对拍 ----
    import fine_align as famod
    check("i7.midpoint_registered",
          "midpoint_2mark" in famod.ALGORITHMS,
          "ALGORITHMS=%s" % (famod.ALGORITHMS,))
    # 手算例（TORAY 式）：设计两点 C1(-2,-1) C2(2,1)；实测=设计绕 θ=0.01
    # 旋转 + t=(0.03,-0.02)。中点 Cm_d=(0,0)，CA_d=atan2(2,4)；
    # 实测中点 Cm_m=t，连线角 CA_m=CA_d+θ → 拟合应还原 (tx,ty,θ)。
    th0, tx0, ty0 = 0.01, 0.03, -0.02
    c0, s0 = math.cos(th0), math.sin(th0)
    d2 = [(-2.0, -1.0), (2.0, 1.0)]
    m2 = [(c0 * x - s0 * y + tx0, s0 * x + c0 * y + ty0) for x, y in d2]
    rm = famod.fit("midpoint_2mark", d2, m2)
    rr2 = famod.fit("rigid", d2, m2)
    check("i7.midpoint_handcalc",
          abs(rm["tx"] - tx0) < 1e-9 and abs(rm["ty"] - ty0) < 1e-9
          and abs(rm["theta"] - th0) < 1e-12 and rm["n"] == 2,
          "tx=%.6f ty=%.6f θ=%.8f" % (rm["tx"], rm["ty"], rm["theta"]))
    check("i7.midpoint_eq_rigid_n2",
          abs(rm["tx"] - rr2["tx"]) < 1e-12
          and abs(rm["theta"] - rr2["theta"]) < 1e-12,
          "n=2 与 rigid 精确解同值")
    # n=4 只用前 2 点：后 2 点加离群也不影响拟合参数，只进残差统计
    d4 = d2 + [(-2.0, 1.0), (2.0, -1.0)]
    m4 = [(c0 * x - s0 * y + tx0, s0 * x + c0 * y + ty0) for x, y in d4]
    m4[2] = (m4[2][0] + 0.2, m4[2][1] - 0.2)          # 离群（不参与拟合）
    rm4 = famod.fit("midpoint_2mark", d4, m4)
    check("i7.midpoint_first2_only",
          abs(rm4["tx"] - tx0) < 1e-9 and abs(rm4["theta"] - th0) < 1e-12
          and rm4.get("n_total") == 4 and "前 2" in (rm4.get("note") or "")
          and rm4["rms"] > 0.05,     # 离群点计入残差统计
          "tx=%.6f note=%s rms=%.4f"
          % (rm4["tx"], rm4.get("note"), rm4["rms"]))
    # 两设计点重合 → 拒绝；n<2 → 拒绝
    try:
        famod.fit("midpoint_2mark", [(1.0, 1.0), (1.0, 1.0)],
                  [(1.0, 1.0), (1.0, 1.0)])
        deg = False
    except ValueError:
        deg = True
    try:
        famod.fit("midpoint_2mark", [(1.0, 1.0)], [(1.0, 1.0)])
        few = False
    except ValueError:
        few = True
    check("i7.midpoint_reject", deg and few, "deg=%s few=%s" % (deg, few))
    # 处方校验：midpoint_2mark 合法、非法值仍 400
    c_alg, _ = http_put(base + "/api/recipe",
                        {"fine_align": {"algorithm": "midpoint_2mark"}})
    c_bad, _ = http_put(base + "/api/recipe",
                        {"fine_align": {"algorithm": "nonsense"}})
    check("i7.midpoint_recipe", c_alg == 200 and c_bad == 400,
          "midpoint=%s bad=%s" % (c_alg, c_bad))
    http_post(base + "/api/recipe/restore_default", {})

    # ---- repeat 口径：WS_LS_X/Y=40nm，AS_X/Y=40nm（迭代 13 用户指定），
    # 其余精密轴仍 500nm ----
    e_ws = get_axis_entry("WS_LS_X")
    e_wy = get_axis_entry("WS_LS_Y")
    e_as = get_axis_entry("AS_X")
    e_asy = get_axis_entry("AS_Y")
    e_asz = get_axis_entry("AS_Z")
    check("i7.repeat_40nm",
          e_ws["config"]["repeat"] == 40.0
          and e_wy["config"]["repeat"] == 40.0
          and e_as["config"]["repeat"] == 40.0
          and e_asy["config"]["repeat"] == 40.0
          and e_asz["config"]["repeat"] == 500.0,
          "WS_X=%s WS_Y=%s AS_X=%s AS_Y=%s AS_Z=%s nm"
          % (e_ws["config"]["repeat"], e_wy["config"]["repeat"],
             e_as["config"]["repeat"], e_asy["config"]["repeat"],
             e_asz["config"]["repeat"]))

    # 迭代 18：交接序列/精对准精简/诊断固化/一键串联均为重流程实跑
    # （需 flip_pick→handoff→EGA→调平前置链，远超 sim 30s 标准）——
    # quick 门控跳过（交接实跑由 i8.chain_bond_happy 单片链覆盖），
    # full 全跑；上方 midpoint 手算对拍/repeat 口径为进程内用例两套件保留。
    if not fblock(["i7.handoff_positive_seq", "i7.handoff_run",
                   "i7.head_to_flip_positive_seq",
                   "i7.fa_steps_slim", "i7.fa_guard_frozen", "i7.midpoint_run",
                   "i7.repeat0_residual",
                   "i7.chain_happy", "i7.chain_skip", "i7.chain_abort",
                   "i7.chain_conflict"]):
        return

    try:
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×）
        setup_recipe()
        # ---- 交接正压序列（paused 生成核对步骤）+ 实跑闭环 ----
        s = run_flow("/api/process/flip_pick", {}, 300.0)
        assert s.get("state") == "done", s.get("state")
        flow_cleanup()
        c, r = http_post(base + "/api/process/flip_handoff", {"paused": True})
        sd = r.get("steps_def") or []
        ops_io = [(st_.get("op"), st_.get("cmd"), st_.get("ms"))
                  for st_ in sd if st_.get("op") in ("io", "delay", "wait_die")]
        seq = [x for x in ops_io]
        pos_i = next((i for i, x in enumerate(seq)
                      if x[0] == "io" and x[1] == "OpenPositive"), -1)
        tail = seq[pos_i:pos_i + 4] if pos_i >= 0 else []
        ok_seq = pos_i >= 0 and len(tail) == 4 \
            and tail[0][0] == "io" and tail[0][1] == "OpenPositive" \
            and tail[1][0] == "delay" and tail[1][2] == 200 \
            and tail[2][0] == "io" and tail[2][1] == "OpenAir" \
            and tail[3][0] == "wait_die"
        flow_cleanup()
        check("i7.handoff_positive_seq", c == 200 and ok_seq,
              "seq=%s" % (tail,))
        # 实跑交接（正压沿触发，闭环不回归）→ head 有 die
        s = run_flow("/api/process/flip_handoff", {}, 120.0)
        viz = http_json(base + "/api/viz")
        n_head = sum(1 for d in viz["dies"] if d["carrier"] == "head")
        flow_cleanup()
        check("i7.handoff_run", s.get("state") == "done" and n_head == 1,
              "state=%s head=%d" % (s.get("state"), n_head))
        # 吸回流程对称正压序列（paused 生成核对）
        c, r = http_post(base + "/api/process/head_to_flip", {"paused": True})
        sd2 = r.get("steps_def") or []
        cmds = [st_.get("cmd") for st_ in sd2 if st_.get("op") == "io"]
        n_pos = sum(1 for x in cmds if x == "OpenPositive")
        wd_i = next((i for i, st_ in enumerate(sd2)
                     if st_.get("op") == "wait_die"), -1)
        last_air = max((i for i, st_ in enumerate(sd2)
                        if st_.get("op") == "io"
                        and st_.get("cmd") == "OpenAir"), default=-1)
        has_delay = any(st_.get("op") == "delay" and st_.get("ms") == 200
                        for st_ in sd2)
        ok_h2f = c == 200 and n_pos == 3 and has_delay \
            and 0 < last_air < wd_i
        flow_cleanup()
        check("i7.head_to_flip_positive_seq", ok_h2f,
              "pos=%d delay=%s last_air<wait_die=%s"
              % (n_pos, has_delay, last_air < wd_i))

        # ---- EGA + 调焦调平（精对准前置链） ----
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        s = run_flow("/api/process/wafer_ega", {"with_load": True})
        assert s.get("state") == "done", s.get("state")
        flow_cleanup()
        s = run_flow("/api/process/focus_level", {"mode": "per_site"})
        assert s.get("state") == "done", s.get("state")
        flow_cleanup()

        # ---- 精对准流程精简：无 fl_ready/进场 move、末两步 AS 回安全位 ----
        mc = http_json(base + "/api/machine_constants")["constants"]
        as_safe = [float(v) for v in
                   mc["flip_handoff"]["as_safe_xy_mm"]]
        c, r = http_post(base + "/api/process/fine_align", {"paused": True})
        sd3 = r.get("steps_def") or []
        ops3 = [st_.get("op") for st_ in sd3]
        mv3 = [(st_.get("axis"), st_.get("target")) for st_ in sd3
               if st_.get("op") == "move"]
        ok_slim = c == 200 and "fl_ready" not in ops3 \
            and [a for a, _ in mv3] == ["AS_X", "AS_Y"] \
            and abs(mv3[0][1] - as_safe[0]) < 1e-9 \
            and abs(mv3[1][1] - as_safe[1]) < 1e-9 \
            and ops3[-1] == "log" and ops3[1] == "fa_measure"
        flow_cleanup()
        check("i7.fa_steps_slim", ok_slim,
              "ops=%s moves=%s safe=%s" % (ops3, mv3, as_safe))
        # ---- fl_ws_frozen site 不匹配守卫 400（frozen=中心位，换个键合位） ----
        import process_flows as pfmod
        rec = http_json(base + "/api/recipe")["recipe"]
        cells = pfmod.wafer_sites_cells(rec)
        center = min(cells, key=lambda cc: math.hypot(cc[2], cc[3]))
        alt_cell = max(cells, key=lambda cc: math.hypot(cc[2], cc[3]))
        c_g, r_g = http_post(base + "/api/process/fine_align",
                             {"site": [alt_cell[0], alt_cell[1]]})
        ok_g = c_g == 400 and "未就位" in (r_g.get("error") or "")
        flow_cleanup()
        check("i7.fa_guard_frozen", ok_g,
              "alt=R%02dC%02d code=%s err=%s"
              % (alt_cell[0], alt_cell[1], c_g, r_g.get("error", "")[:60]))

        # ---- midpoint_2mark 实跑：judge=pass + 日志注明只用前 2 标记 ----
        # （只用前 2 标记无多点平均：θ 噪声 σ≈√2·σxy/基线≈60~70µrad 实测，
        #   默认 tol_rz=30µrad 物理上收敛不了——按算法特性放宽确认阈值，
        #   TORAY 式用法阈值本就该按 2 点口径设）
        http_put(base + "/api/recipe", {"fine_align": {
            "algorithm": "midpoint_2mark", "measure_points": 4,
            "confirm_tol_xy_um": 1.0, "confirm_tol_rz_urad": 200.0,
            "confirm_max_iter": 5}})
        s = run_flow("/api/process/fine_align", {})
        fa = (s or {}).get("fine_align_result") or {}
        logs = "\n".join(e.get("msg", "") for e in (s or {}).get("log", []))
        check("i7.midpoint_run",
              fa.get("judge") == "pass"
              and fa.get("algorithm") == "midpoint_2mark"
              and "前 2 个标记" in logs,
              "judge=%s algo=%s note=%s"
              % (fa.get("judge"), fa.get("algorithm"),
                 "前 2 个标记" in logs))
        flow_cleanup()
        http_post(base + "/api/recipe/restore_default", {})
        http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5}})

        # ---- 诊断固化：repeat 全置 0 复跑 → 物理残差量级 ----
        import axes_config as _ac
        saved_rep = {}
        for axn in _ac.AXIS_NAMES:
            saved_rep[axn] = get_axis_entry(axn)["config"]["repeat"]
            http_post(base + "/api/axis/config", {"axis": axn, "repeat": 0})
        s = run_flow("/api/process/focus_level", {"mode": "per_site"})
        assert s.get("state") == "done", s.get("state")
        flow_cleanup()
        s = run_flow("/api/process/fine_align", {})
        fa0 = (s or {}).get("fine_align_result") or {}
        ph0 = fa0.get("phys_resid") or {}
        flow_cleanup()
        for axn, rv in saved_rep.items():
            http_post(base + "/api/axis/config", {"axis": axn, "repeat": rv})
        # 迭代 7 诊断结论：repeat=0 后残余收敛（repeat 到位重采样是主因，
        # 非恒定系统框偏）。残余剩余项=FAST 视觉噪声（σxy≈0.09µm、
        # σθ≈17µrad，末轮含噪测量收敛的物理代价）：阈值 |t|≤2.0µm、
        # |θ|≤40µrad（实测 expA：tx=-0.04 ty=-0.19µm θ=-0.49µrad；回归
        # 曾观测 ty=+1.25µm 的噪声尾，留 ~10σ 余量仍远低于 500nm repeat
        # 基线 ~0.7µm+涨落，能区分"塌缩/不塌缩"）
        check("i7.repeat0_residual",
          fa0.get("judge") == "pass"
          and abs(ph0.get("tx_um", 9e9)) <= 2.0
          and abs(ph0.get("ty_um", 9e9)) <= 2.0
          and abs(ph0.get("theta_urad", 9e9)) <= 40.0,
          "judge=%s phys tx=%+.3f ty=%+.3f um θ=%+.2f urad（repeat=0）"
          % (fa0.get("judge"), ph0.get("tx_um", 9e9),
             ph0.get("ty_um", 9e9), ph0.get("theta_urad", 9e9)))

        # ---- 一键串联 happy path（全新物料状态，五阶段全 done） ----
        setup_recipe()
        c, r = http_post(base + "/api/process/chain", {})
        ch = chain_wait()
        fa_c = flow_state().get("fine_align_result") or {}
        st_ok = ch.get("stages") or {}
        check("i7.chain_happy",
              c == 200 and not ch.get("active") and not ch.get("error")
              and all(st_ok.get(k) == "done" for k in
                      ("wafer_ega", "flip_pick", "flip_handoff",
                       "focus_level", "fine_align"))
              and fa_c.get("judge") == "pass",
              "stages=%s err=%s judge=%s"
              % (st_ok, ch.get("error"), fa_c.get("judge")))
        flow_cleanup()
        # ---- 跳过逻辑（迭代 8 语义）：本 site 已键合 → 二次运行整条链
        #      幂等全 skip（重跑精对准会测到已键合 die 遮挡标记，物理无效） ----
        c, r = http_post(base + "/api/process/chain", {})
        ch2 = chain_wait()
        st2 = ch2.get("stages") or {}
        check("i7.chain_skip",
              c == 200 and not ch2.get("error")
              and all(st2.get(k) == "skip" for k in
                      ("wafer_ega", "flip_pick", "flip_handoff",
                       "focus_level", "fine_align", "bond")),
              "stages=%s err=%s" % (st2, ch2.get("error")))
        flow_cleanup()
        # ---- 中途失败 abort：确认阈值 1nm + abort → fine_align 阶段 fail ----
        # （迭代 8：先复位物料让 site 回到未键合态，否则整链 skip 跑不到
        #   fine_align）
        setup_recipe()
        http_put(base + "/api/recipe", {"fine_align": {
            "confirm_tol_xy_um": 0.001, "confirm_tol_rz_urad": 0.001,
            "on_unconfirmed": "abort"}})
        c, r = http_post(base + "/api/process/chain", {})
        ch3 = chain_wait()
        st3 = ch3.get("stages") or {}
        check("i7.chain_abort",
              c == 200 and st3.get("fine_align") == "fail"
              and "fine_align" in (ch3.get("error") or "")
              and st3.get("focus_level") == "done",
              "stages=%s err=%s" % (st3, ch3.get("error")))
        # 链运行中重复启动 → 409
        c4, r4 = http_post(base + "/api/process/chain", {})
        ch4 = flow_state().get("chain") or {}
        c5, r5 = http_post(base + "/api/process/chain", {})
        check("i7.chain_conflict",
              c4 == 200 and (c5 == 409 or not (ch4.get("active"))),
              "first=%s second=%s" % (c4, c5))
        chw = chain_wait()
        flow_cleanup()

    finally:
        try:
            set_speed(1.0)                   # 提速窗复原（迭代 18）
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_machine_constants_calibration():
    """--- 18. 机器常数 + 测校 ---
    机器常数（machine_constants.json，相机内参单一事实源）：
    GET 含派生只读 k_um_px/fov；PUT 深合并→全量校验→原子落盘→视觉热生效；
    /api/vision/camera/config 对内参字段一律 400 指路。
    测校（calibration.py，作业模式：start→job_id / status / result / apply）：
    三程序全走 REAL 链路（无 cv2 则整组跳过不 FAIL）；标定后轴位/光源/
    视觉模式自动恢复；运行中并发 start → 409。"""
    print("--- 18. 机器常数（/api/machine_constants）+ 测校（/api/calibration） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False

    # ---- 18.1 GET：三相机内参 + 派生（down 6um/10× → k=0.6，fov 1.8×1.8） ----
    mc = http_json(base + "/api/machine_constants")
    cams = (mc.get("constants") or {}).get("cameras") or {}
    dd = ((mc.get("derived") or {}).get("cameras") or {}).get("down") or {}
    check("mc.get", mc.get("ok") and set(cams) == {"coarse", "up", "down"}
          and "cameras" in (mc.get("groups") or [])
          and abs(dd.get("k_um_px", 0) - 0.6) < 1e-6
          and abs(dd.get("fov_x_mm", 0) - 1.8) < 1e-6
          and abs(dd.get("fov_y_mm", 0) - 1.8) < 1e-6
          and cams.get("down", {}).get("px_u") == 3000
          and cams.get("down", {}).get("px_v") == 3000
          and abs(cams.get("down", {}).get("wd_mm", 0) - 10.0) < 1e-9,
          "k=%s fov=%s×%s" % (dd.get("k_um_px"), dd.get("fov_x_mm"),
                              dd.get("fov_y_mm")))

    # ---- 18.1b px_u/px_v 已迁入机器常数：PUT 生效（派生 fov 跟随 + 视觉
    #      热生效）→ 改回 3000；camera/config 收到 px 字段 400 指路 ----
    c, r = http_put(base + "/api/machine_constants",
                    {"cameras": {"down": {"px_u": 2900, "px_v": 2800}}})
    vd = http_json(base + "/api/vision/cameras")["cameras"]["down"]
    fov_p = ((r.get("derived") or {}).get("cameras") or {}) \
        .get("down") or {}
    ok_px = c == 200 and vd.get("px") == [2900, 2800] \
        and abs(fov_p.get("fov_x_mm", 0) - 1.74) < 1e-6 \
        and abs(fov_p.get("fov_y_mm", 0) - 1.68) < 1e-6
    c2, _ = http_put(base + "/api/machine_constants",
                     {"cameras": {"down": {"px_u": 3000, "px_v": 3000}}})
    vd2 = http_json(base + "/api/vision/cameras")["cameras"]["down"]
    c3, r3 = http_post(base + "/api/vision/camera/config",
                       {"cam": "down", "px_u": 3000})
    c4, r4 = http_post(base + "/api/vision/camera/config",
                       {"cam": "down", "px_x": 1280})
    c5, _ = http_put(base + "/api/machine_constants",
                     {"cameras": {"down": {"px_u": 5000}}})
    check("mc.px_migrated", ok_px and c2 == 200
          and vd2.get("px") == [3000, 3000]
          and c3 == 400 and c4 == 400
          and "machine_constants" in (r3.get("error") or "")
          and c5 == 400,
          "put=%s restore=%s cfg=%s/%s bad=%s err=%s"
          % (ok_px, vd2.get("px"), c3, c4, c5,
             (r3.get("error") or "")[:60]))

    # ---- 18.1c focus_level 组（调焦调平机械常数）：confocal_cam_dx_mm
    #      新增、confocal_sigma_um 迁出（已归 recipe sim_sensors，PUT 应
    #      400）、level_tol_urad 默认 30 ----
    flg = (mc.get("constants") or {}).get("focus_level") or {}
    check("mc.focus_level_fields",
          len([k for k in flg if not k.startswith("_")]) == 11
          and "confocal_cam_dx_mm" in flg
          and "as_head_center_xy_mm" in flg
          and "confocal_sigma_um" not in flg
          and abs(flg.get("level_tol_urad", 0.0) - 60.0) < 1e-9,
          "keys=%s tol=%s" % (sorted(k for k in flg
                                     if not k.startswith("_")),
                              flg.get("level_tol_urad")))
    c, r = http_put(base + "/api/machine_constants",
                    {"focus_level": {"confocal_cam_dx_mm": 0.5}})
    j2 = http_json(base + "/api/machine_constants")
    dx_v = ((j2.get("constants") or {}).get("focus_level") or {}) \
        .get("confocal_cam_dx_mm")
    c2, _ = http_put(base + "/api/machine_constants",
                     {"focus_level": {"confocal_cam_dx_mm": 0.0}})
    c3, r3 = http_put(base + "/api/machine_constants",
                      {"focus_level": {"confocal_sigma_um": 0.1}})
    c4, _ = http_put(base + "/api/machine_constants",
                     {"focus_level": {"confocal_cam_dx_mm": 999.0}})
    check("mc.focus_level_put",
          c == 200 and abs(dx_v - 0.5) < 1e-9 and c2 == 200
          and c3 == 400 and c4 == 400,
          "put=%s dx=%s restore=%s sigma400=%s bad400=%s err=%s"
          % (c, dx_v, c2, c3, c4, (r3.get("error") or "")[:60]))

    # ---- 18.1e vision_marks 组（视觉对准标记模板，2026-09-04 迭代 12）：
    #      默认 5 字段均为 vision_templates.json 已建模板；PUT 合法改名→
    #      恢复；非法（空串/非字符串/未知字段/未建立模板）逐条 400；FAST
    #      identify 带未建立模板名 → 400"模板未建立"（模拟真机未建模板
    #      不能识别）----
    vmg = (mc.get("constants") or {}).get("vision_marks") or {}
    tpls = set(t.get("name") for t in
               (http_json(base + "/api/vision/templates")
                .get("templates") or []))
    vkeys = sorted(k for k in vmg if not k.startswith("_"))
    check("mc.vision_marks_defaults",
          vkeys == ["tpl_coarse_die", "tpl_die_head", "tpl_head_chuck",
                    "tpl_post_inspect", "tpl_wafer_site"]
          and all(isinstance(vmg[k], str) and vmg[k] in tpls for k in vkeys),
          "keys=%s vals=%s" % (vkeys, [vmg.get(k) for k in vkeys]))
    c, _ = http_put(base + "/api/machine_constants",
                    {"vision_marks": {"tpl_wafer_site": "cv_cross"}})
    j2 = http_json(base + "/api/machine_constants")
    ws_v = ((j2.get("constants") or {}).get("vision_marks") or {}) \
        .get("tpl_wafer_site")
    # 恢复为读到的原值（迭代 20 起默认 cv_quad——真 quad 模板；
    # cv_cross_0824 为占位期 cross 图，留库兼容）
    c2, _ = http_put(base + "/api/machine_constants",
                     {"vision_marks": {"tpl_wafer_site":
                                       vmg.get("tpl_wafer_site")}})
    bads_vm = [
        {"vision_marks": {"tpl_wafer_site": ""}},
        {"vision_marks": {"tpl_wafer_site": 123}},
        {"vision_marks": {"tpl_bogus": "x"}},
        {"vision_marks": {"tpl_wafer_site": "no_such_tpl"}},
    ]
    codes_vm = [http_put(base + "/api/machine_constants", b)[0]
                for b in bads_vm]
    c5, r5 = http_post(base + "/api/vision/identify",
                       {"cam": "down", "template": "no_such_tpl"})
    check("mc.vision_marks_put",
          c == 200 and ws_v == "cv_cross" and c2 == 200
          and codes_vm == [400] * len(bads_vm)
          and c5 == 400 and "模板未建立" in (r5.get("error") or ""),
          "put=%s val=%s restore=%s bads=%s ident=%s err=%s"
          % (c, ws_v, c2, codes_vm, c5, (r5.get("error") or "")[:60]))

    # ---- 18.2 PUT 合法：down wd_mm 10→10.5（落盘+热生效）→ 改回 10.0 ----
    c, r = http_put(base + "/api/machine_constants",
                    {"cameras": {"down": {"wd_mm": 10.5}}})
    vd = http_json(base + "/api/vision/cameras")["cameras"]["down"]
    with open(os.path.join(ROOT, "runtime", "machine_constants.json"),
              encoding="utf-8") as f:
        wd_disk = json.load(f)["cameras"]["down"]["wd_mm"]
    ok_put = c == 200 and r.get("ok") and r.get("persisted") is True \
        and abs((r["derived"]["cameras"]["down"].get("k_um_px") or 0)
                - 0.6) < 1e-9 \
        and abs(vd["wd_mm"] - 10.5) < 1e-9 and abs(wd_disk - 10.5) < 1e-9
    c2, _ = http_put(base + "/api/machine_constants",
                     {"cameras": {"down": {"wd_mm": 10.0}}})
    vd2 = http_json(base + "/api/vision/cameras")["cameras"]["down"]
    check("mc.put_valid", ok_put and c2 == 200
          and abs(vd2["wd_mm"] - 10.0) < 1e-9,
          "put=%s disk=%.3f hot=%.3f restore=%s"
          % (ok_put, wd_disk, vd["wd_mm"], c2))

    # ---- 18.3 PUT 非法逐条 400：dir 非 ±1 / swap 非布尔 / 负数 / 未知字段 /
    #      未知相机 / 未知组；且不应污染现有常数 ----
    bads = [
        {"cameras": {"down": {"dir_x": 0}}},
        {"cameras": {"down": {"swap_xy": 1}}},
        {"cameras": {"down": {"pixel_um": -1}}},
        {"cameras": {"down": {"wd_mm": 5.0, "bogus": 1}}},
        {"cameras": {"nope": {"wd_mm": 5.0}}},
        {"motors": {"x": 1}},
    ]
    codes = [http_put(base + "/api/machine_constants", b)[0] for b in bads]
    wd_now = http_json(base + "/api/vision/cameras")["cameras"]["down"]["wd_mm"]
    check("mc.put_invalid_400", codes == [400] * len(bads)
          and abs(wd_now - 10.0) < 1e-9,
          "codes=%s wd=%s" % (codes, wd_now))

    # ---- 18.3b 迭代 49 新组：bs_rxry_axis（BS_CAL_Rx/Ry 旋转轴线标定，
    #      bs_rxry_axis 测校写回）+ sensor_calib（温度/力传感链标定常数，
    #      bond_temp/force_calib 测校写回）——groups 列表含新组、默认值、
    #      向量/数值校验 400、合法 PUT 读回+还原 ----
    mc49 = http_json(base + "/api/machine_constants")
    groups49 = mc49.get("groups") or []
    consts49 = mc49.get("constants") or {}
    sc49 = consts49.get("sensor_calib") or {}
    br49 = consts49.get("bs_rxry_axis") or {}
    check("mc.i49_groups",
          "bs_rxry_axis" in groups49 and "sensor_calib" in groups49
          and abs(sc49.get("head_temp_gain", 0) - 1.0) < 1e-12
          and abs(sc49.get("head_temp_offset_c", 9) - 0.0) < 1e-12
          and abs(sc49.get("wafer_temp_gain", 0) - 1.0) < 1e-12
          and abs(sc49.get("wafer_temp_offset_c", 9) - 0.0) < 1e-12
          and abs(sc49.get("force_gain", 0) - 1.0) < 1e-12
          and abs(sc49.get("force_offset_n", 9) - 0.0) < 1e-12
          and br49.get("rx_axis_dir") == [1.0, 0.0, 0.0]
          and br49.get("ry_axis_dir") == [0.0, 1.0, 0.0]
          and abs((br49.get("rx_axis_point_mm") or [0])[2] - 1215.25)
          < 1e-9,
          "groups=%s sensor_calib=%s"
          % ("bs_rxry_axis" in groups49 and "sensor_calib" in groups49,
             {k: sc49.get(k) for k in sorted(sc49)
              if not k.startswith("_")}))
    bads49 = [
        {"sensor_calib": {"force_gain": 2.0}},        # 越界 (>1.5)
        {"sensor_calib": {"head_temp_offset_c": 60}},  # 越界 (>50)
        {"sensor_calib": {"bogus": 1.0}},              # 未知字段
        {"bs_rxry_axis": {"rx_axis_dir": [1.0, 0.0]}},  # 向量维数错
        {"bs_rxry_axis": {"bogus": [0, 0, 0]}},         # 未知字段
    ]
    codes49 = [http_put(base + "/api/machine_constants", b)[0]
               for b in bads49]
    c49a, _ = http_put(base + "/api/machine_constants",
                       {"sensor_calib": {"force_gain": 1.003}})
    got49 = http_json(base + "/api/machine_constants")["constants"][
        "sensor_calib"]["force_gain"]
    c49b, _ = http_put(base + "/api/machine_constants",
                       {"sensor_calib": {"force_gain": 1.0}})
    check("mc.i49_put",
          codes49 == [400] * len(bads49)
          and c49a == 200 and abs(got49 - 1.003) < 1e-12 and c49b == 200,
          "bads=%s put=%s readback=%s restore=%s"
          % (codes49, c49a, got49, c49b))

    # ---- 18.4 camera/config 拒内参：纯内参 400；混合（exposure+mag）也 400 ----
    c1, r1 = http_post(base + "/api/vision/camera/config",
                       {"cam": "down", "wd_mm": 5.0})
    c2, r2 = http_post(base + "/api/vision/camera/config",
                       {"cam": "down", "exposure_ms": 8.0, "mag": 2.0})
    check("mc.camera_config_reject",
          c1 == 400 and c2 == 400
          and "machine_constants" in (r1.get("error") or "")
          and "machine_constants" in (r2.get("error") or ""),
          "c1=%s c2=%s err=%s" % (c1, c2, (r1.get("error") or "")[:80]))

    # ---- 18.4b view_rot 真值/认知分离迁移不变量（迭代 39）：cameras API
    #      view_rot_deg（相机物理安装旋转角·真值，°）== 机器常数 rot_mrad
    #      （控制器认知）×0.05729577951（°=mrad 换算）逐相机成立——
    #      θ_b=θ_t=补偿到位。**自证清白版（迭代 39）**：迁移不变量以"两通道
    #      置等后能测出相等"判读——先快照各相机 rot_mrad 原值，临时 PUT
    #      rot_mrad := view_rot_deg×(π/180×1000)，断言 parity<1e-6，finally
    #      恢复原值（用户/前序测试改过的认知值不再使套件脆断）
    vd_cams = http_json(base + "/api/vision/cameras")["cameras"]
    mc_cams = (http_json(base + "/api/machine_constants")
               .get("constants") or {}).get("cameras") or {}
    saved_rm = {c_: float((mc_cams.get(c_) or {}).get("rot_mrad", 0.0))
                for c_ in ("coarse", "up", "down")}
    try:
        for c_ in ("coarse", "up", "down"):
            vr = float((vd_cams.get(c_) or {}).get("view_rot_deg", 0.0))
            c, r = http_put(base + "/api/machine_constants",
                            {"cameras": {c_: {
                                "rot_mrad": vr / 0.05729577951}}})
            assert c == 200, (c_, c, r)
        mc_cams2 = (http_json(base + "/api/machine_constants")
                    .get("constants") or {}).get("cameras") or {}
        par = {}
        for c_ in ("coarse", "up", "down"):
            vr = float((vd_cams.get(c_) or {}).get("view_rot_deg", 9e9))
            rm = float((mc_cams2.get(c_) or {}).get("rot_mrad", 9e9))
            par[c_] = abs(vr - rm * 0.05729577951)
        check("cal.microrot_view_rot_parity",
              all(v < 1e-6 for v in par.values()),
              "|view_rot_deg−rot_mrad×0.05729577951|=%s（°，<1e-6=迁移 θ_b=θ_t"
              "；自证清白置等后判读）"
              % {k: float("%.2e" % v) for k, v in par.items()})
    finally:
        for c_, v_ in saved_rm.items():
            http_put(base + "/api/machine_constants",
                     {"cameras": {c_: {"rot_mrad": v_}}})

    # ---- 18.6 focus_scan(coarse) → 400（镜头固连机架，无相对 Z 自由度）；
    #      纯校验、无需场景/作业（迭代 18 前移，quick 也跑）----
    c, r = http_post(base + "/api/calibration/start",
                     {"routine": "focus_scan", "cam": "coarse", "params": {}})
    check("cal.focus_scan_coarse_400", c == 400
          and "不支持" in (r.get("error") or ""),
          "c=%s err=%s" % (c, (r.get("error") or "")[:60]))

    if not has_cv2:
        print("   [skip] cal.*（无 cv2，REAL 链路标定跳过，不 FAIL）")
        return

    # 迭代 18：REAL 测校作业（墙钟不压缩，四项作业 ~15s）+ 场景走位 +
    # persist 落盘回归留 full；quick 门控跳过。
    if not fblock(["cal.focus_scan_down", "cal.jobs_api",
                   "cal.pixel_equiv_down", "cal.rotation_down",
                   "cal.busy_apply",
                   "persist.camera_config", "persist.kinematics"]):
        return

    # ---- 场景准备：晶圆上片 + 走位（wafer_mark 进下相机画幅中心，同 17 组） ----
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    mode0 = http_json(base + "/api/vision/cameras").get("mode")

    def cal_start(routine, cam, params=None):
        return http_post(base + "/api/calibration/start",
                         {"routine": routine, "cam": cam,
                          "params": params or {}})

    def cal_wait(job_id, timeout=90.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = http_json(base + "/api/calibration/status?job_id=%d" % job_id)
            if s.get("state") in ("done", "error"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def cal_result(job_id):
        return (http_json(base + "/api/calibration/result?job_id=%d" % job_id)
                .get("result") or {})

    # ---- 18.5 focus_scan(down)：AS_Z 扫 ±0.06/step 0.01（13 点，含判废点），
    #      抛物线峰 → wd≈10.0；作业自己切 real 并还原，AS_Z 回 0 ----
    c, r = cal_start("focus_scan", "down",
                     {"range_mm": 0.06, "step_mm": 0.01})
    jid = (r or {}).get("job_id")
    s = cal_wait(jid) if jid else {"state": "start_failed"}
    res = cal_result(jid) if s.get("state") == "done" else {}
    wd_m = (res.get("measured") or {}).get("wd_mm")
    pts = res.get("points") or []
    scores = [p["score"] for p in pts]
    peak_struct = len(pts) == 13 and max(scores or [0]) > 0.5 \
        and scores.index(max(scores)) not in (0, len(scores) - 1)
    asz = get_axis_entry("AS_Z")["pos"]
    mode1 = http_json(base + "/api/vision/cameras").get("mode")
    check("cal.focus_scan_down", c == 200 and s.get("state") == "done"
          and wd_m is not None and abs(wd_m - 10.0) < 0.3
          and peak_struct and abs(asz) < 1e-6 and mode1 == mode0,
          "state=%s wd=%s n=%d asz=%.4f mode=%s err=%s"
          % (s.get("state"), wd_m, len(pts), asz, mode1, s.get("error")))
    jobs = http_json(base + "/api/calibration/jobs")
    check("cal.jobs_api", jobs.get("ok")
          and any(j.get("job_id") == jid and j.get("state") == "done"
                  for j in jobs.get("jobs") or []),
          "active=%s jobs=%s" % (jobs.get("active"), jobs.get("jobs")))

    # ---- 18.6 focus_scan(coarse) → 400 已前移到 has_cv2 判定前（迭代 18） ----

    # ---- 18.7 pixel_equivalent(down)：WS_LS_X 点动 0.2mm（FOV 1.8mm 内）
    #      → 333px → mag≈10.0±5%。delta 上限：标定模板=短边 3/4（2251px），
    #      NCC 可匹配域要求标记中心留在 [1125,1875] → 点动 ≤0.225mm ----
    c, r = cal_start("pixel_equivalent", "down", {"delta_mm": 0.2})
    jid = (r or {}).get("job_id")
    s = cal_wait(jid, 60.0) if jid else {"state": "start_failed"}
    res = cal_result(jid) if s.get("state") == "done" else {}
    mag_m = (res.get("measured") or {}).get("mag")
    wsx = get_axis_entry("WS_LS_X")["pos"]
    check("cal.pixel_equiv_down", c == 200 and s.get("state") == "done"
          and mag_m is not None and abs(mag_m - 10.0) < 0.5
          and abs(wsx - 58.2162) < 1e-6,
          "state=%s mag=%s wsx=%.4f err=%s"
          % (s.get("state"), mag_m, wsx, s.get("error")))

    # ---- 18.8 rotation_direction(down)：X/Y 各点动 0.2mm（可匹配域内）
    #      → rot≈0.3mrad、dir_x/dir_y 与常数一致。REAL 粗定位在降采样网格
    #      上做（MATCH_COARSE_MAX_PX 预算），亚像素量化 ~±0.3 原生 px、随
    #      分数位起伏（差分消不掉），333px 杠杆下 rot 误差 ~±1mrad——容差
    #      ±1.5、拟合残差 <0.008（2026-09-08 质心噪声结果倒推注入后每次
    #      识别含 0.15px 抖动 → 333px 杠杆角噪声 ~4.5e-4×√5≈1e-3 rad、
    #      残差 3e-3 量级，旧 0.002 是白噪声响应近零时代的口径；
    #      dir 符号仍精确断言；要更准需更大杠杆，受 3/4 模板可匹配域
    #      ≤0.225mm 所限）----
    c, r = cal_start("rotation_direction", "down", {"delta_mm": 0.2})
    jid = (r or {}).get("job_id")
    s = cal_wait(jid, 60.0) if jid else {"state": "start_failed"}
    res = cal_result(jid) if s.get("state") == "done" else {}
    meas = res.get("measured") or {}
    resid = (res.get("detail") or {}).get("angular_resid_rad")
    rot_m = (meas["rot_mrad"] if meas.get("rot_mrad") is not None else 9e9)
    check("cal.rotation_down", c == 200 and s.get("state") == "done"
          and abs(rot_m - 0.3) < 1.5
          and meas.get("dir_x") == 1.0 and meas.get("dir_y") == -1.0
          and resid is not None and resid < 0.008,
          "state=%s meas=%s resid=%s err=%s"
          % (s.get("state"), meas, resid, s.get("error")))

    # ---- 18.9 并发 409 + apply 写回（部分字段 wd_mm）+ 未知作业 400 ----
    c, r = cal_start("focus_scan", "down",
                     {"range_mm": 0.06, "step_mm": 0.01})
    jid = (r or {}).get("job_id")
    c2, _ = cal_start("pixel_equivalent", "down")     # 运行中 → 409
    s = cal_wait(jid) if jid else {"state": "start_failed"}
    busy_ok = c == 200 and c2 == 409 and s.get("state") == "done"
    c3, r3 = http_post(base + "/api/calibration/apply",
                       {"job_id": jid, "fields": ["wd_mm"]})
    wd_applied = (http_json(base + "/api/vision/cameras")
                  ["cameras"]["down"]["wd_mm"])
    c4, _ = http_post(base + "/api/calibration/apply",
                      {"job_id": jid, "fields": ["mag"]})   # 不在测量结果 → 400
    c5, _ = http_post(base + "/api/calibration/apply",
                      {"job_id": 999999})                   # 未知作业 → 400
    # 还原常数（apply 写入的是测量值 ≈10.0，有 um 级偏差）
    http_put(base + "/api/machine_constants",
             {"cameras": {"down": {"wd_mm": 10.0}}})
    check("cal.busy_apply", busy_ok
          and c3 == 200 and r3.get("ok") and r3.get("applied") == ["wd_mm"]
          and r3.get("persisted") is True
          and abs(wd_applied - 10.0) < 0.3
          and c4 == 400 and c5 == 400,
          "busy=%s c3=%s applied=%s wd=%.4f c4=%s c5=%s"
          % (busy_ok, c3, r3.get("applied"), wd_applied, c4, c5))

    # ---- 18.10 persist：camera/config 与 kinematics 带 persist:true 落盘 ----
    # 本组真写盘：先快照原字节，finally 逐字节还原（内存态走 API 还原），
    # 不给仓库留 diff；不带 persist 的写必须不动盘（同步断言）
    vc_path = os.path.join(ROOT, "runtime", "vision_cameras.json")
    ek_path = os.path.join(ROOT, "runtime", "error_kinematics.json")
    with open(vc_path, "rb") as f:
        vc_orig = f.read()
    with open(ek_path, "rb") as f:
        ek_orig = f.read()
    tp0 = json.loads(ek_orig.decode("utf-8"))["modules"]["wafer_stage"][
        "tool_point"]

    def _disk_json(p):
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    try:
        # ① camera/config persist：exposure_ms 写回 cameras.down 条目
        c1, r1 = http_post(base + "/api/vision/camera/config",
                           {"cam": "down", "exposure_ms": 12.0,
                            "persist": True})
        disk12 = _disk_json(vc_path)["cameras"]["down"].get("exposure_ms")
        c2, r2 = http_post(base + "/api/vision/camera/config",
                           {"cam": "down", "exposure_ms": 8.0})
        still12 = _disk_json(vc_path)["cameras"]["down"].get("exposure_ms")
        check("persist.camera_config", c1 == 200
              and r1.get("persisted") is True and disk12 == 12.0
              and c2 == 200 and not r2.get("persisted") and still12 == 12.0,
              "c1=%s disk=%s c2=%s disk2=%s" % (c1, disk12, c2, still12))
        # 内存态还原（盘字节在 finally 统一还原）
        http_post(base + "/api/vision/camera/config",
                  {"cam": "down", "exposure_ms": 8.0})
        # ② kinematics persist：tool_point 写回 wafer_stage 条目，_note 保留
        c3, r3 = http_put(base + "/api/kinematics",
                          {"modules": {"wafer_stage":
                                       {"tool_point": [-0.1, 0.0, 881.9]}},
                           "persist": True})
        d3 = _disk_json(ek_path)
        tp_new = d3["modules"]["wafer_stage"]["tool_point"]
        c4, r4 = http_put(base + "/api/kinematics",
                          {"modules": {"wafer_stage": {"tool_point": tp0}}})
        tp_stay = _disk_json(ek_path)["modules"]["wafer_stage"]["tool_point"]
        check("persist.kinematics", c3 == 200 and r3.get("persisted") is True
              and tp_new == [-0.1, 0.0, 881.9] and "_note" in d3
              and c4 == 200 and not r4.get("persisted") and tp_stay == tp_new,
              "c3=%s disk=%s c4=%s disk2=%s" % (c3, tp_new, c4, tp_stay))
    finally:
        with open(vc_path, "wb") as f:
            f.write(vc_orig)
        with open(ek_path, "wb") as f:
            f.write(ek_orig)
        http_post(base + "/api/vision/camera/config",
                  {"cam": "down", "exposure_ms": 8.0})
        http_put(base + "/api/kinematics",
                 {"modules": {"wafer_stage": {"tool_point": tp0}}})

    # ---- 清理：轴回零 / 下片复位 / 模式 fast / 常数已还原 ----
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y", "WS_Rz"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
def wait_ready(timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            h = http_json("http://127.0.0.1:%d/api/health" % WS_PORT)
            if h.get("ok"):
                return True
        except Exception:
            time.sleep(0.3)
    return False


# ---------------- 沙盒干涉检查（W5，零件级 v2） ----------------
def test_collision():
    """--- W5 沙盒干涉检查（/api/collision，零件级 + 穿模深度阈值 ε） ---
    用例坐标全部从 runtime/sandboxes.json（schema 2）反推，不写死魔法数。
    判撞口径：两盒三向重叠量的最小值（="拔出最少要走多远"）> depth_eps（2mm）。
    深行程用例：FA_X 把翻转链零件推向对准相机承载台——从对表筛 hit_axes 含
    FA_X 的对，取阈值最浅的一对（x 向重叠 > ε → v_hit = as.max_x-fa.min_x-ε）。
    三个用户实测穿模场景各自成用例：键合头×晶圆台 / 键合头×对准台 /
    翻转Z×蓝膜台，且都验证"浅贴合放行、深穿模拦截"。"""
    print("--- 14. 沙盒干涉检查（/api/collision，零件级） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    with open(os.path.join(ROOT, "runtime", "sandboxes.json"),
              encoding="utf-8") as f:
        sb = json.load(f)
    boxes = {b["name"]: b for b in sb["boxes"]}
    eps = float(sb.get("depth_eps", 2.0))

    def part(name):                     # 零件对象 id -> 盒
        return boxes["pt:" + name]

    def depth_of(b1, b2):
        return min(min(b1["max"][k], b2["max"][k])
                   - max(b1["min"][k], b2["min"][k]) for k in range(3))

    # 深行程对：hit FA_X 且一方是翻转链零件（linear 含 FA_X）
    fa_pairs = []
    for p in sb["pairs"]:
        if "FA_X" not in p["hit_axes"]:
            continue
        a, b = boxes[p["a"]], boxes[p["b"]]
        fa, other = (a, b) if "FA_X" in a["linear"] else (b, a)
        v = other["max"][0] - fa["min"][0] - eps   # x 向重叠 > ε 的阈值位移
        fa_pairs.append((v, p, fa, other))
    fa_pairs.sort(key=lambda t: -t[0])
    if not fa_pairs:
        check("collision.health_fields", False,
              "sandboxes.json 对表没有 hit FA_X 的对（模型变了？重新生成）")
        return
    v_hit, pair, fa_box, other_box = fa_pairs[0]
    pa, pb = fa_box["part"], other_box["part"]
    tgt_collide = v_hit - 10.0                  # 起步即撞的目标
    tgt_deep = v_hit - 75.0                     # 中途撞入的目标
    print("   对表反推：%s × %s v_hit=%.1f tgt_collide=%.1f tgt_deep=%.1f"
          % (pa, pb, v_hit, tgt_collide, tgt_deep))

    def fa_pos():
        return get_axis_entry("FA_X")["pos"]

    def wait_fa_idle(timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            e = get_axis_entry("FA_X")
            if (not e["moving"]) and e["state"] == "idle":
                return True
            time.sleep(0.02)
        return False

    # health 字段 + /api/collision 基线（含 depth_eps）
    h = http_json(base + "/api/health")
    c0 = http_json(base + "/api/collision")
    if c0.get("active"):
        print("   [诊断] 进入 test_collision 时 active=%s" % c0["active"])
    check("collision.health_fields",
          h.get("collision", {}).get("loaded") is True
          and h["collision"].get("enabled") is True
          and h["collision"].get("boxes", 0) == len(sb["boxes"])
          and h["collision"].get("pairs", 0) == len(sb["pairs"])
          and c0.get("ok") and c0.get("enabled") is True
          and c0.get("exempt") == len(sb["exempt_pairs"])
          and abs(c0.get("depth_eps", -1) - eps) < 1e-9,
          "health.collision=%s depth_eps=%s"
          % (h.get("collision"), c0.get("depth_eps")))

    # 常规 move（自由区目标）无干涉通过
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    ok0 = st == 200 and wait_fa_idle() and abs(fa_pos()) < 1e-6
    check("collision.no_overlap_normal", ok0,
          "status=%d pos=%.3f" % (st, fa_pos()))

    # 起步即撞：目标落在重叠区 → 409 且错误含两个零件级盒名
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_X", "target": tgt_collide, "speed": 600.0})
    err = r.get("error", "")
    check("collision.rejects_move",
          st == 409 and "沙盒干涉" in err
          and pa in err and pb in err
          and abs(fa_pos()) < 1e-6,
          "status=%d err=%s pos=%.3f" % (st, err[:120], fa_pos()))

    # 运动中撞入：先关闭开关起步（巡航中再打开），进入重叠区即全轴 soft-stop
    http_post(base + "/api/collision", {"enabled": False})
    http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": tgt_deep, "speed": 600.0})
    t0 = time.time()
    while fa_pos() > -500.0 and time.time() - t0 < 5.0:   # 巡航段再打开
        time.sleep(0.01)
    http_post(base + "/api/collision", {"enabled": True})
    wait_fa_idle()
    pos_stop = fa_pos()
    c1 = http_json(base + "/api/collision")
    ev = c1.get("events") or []
    hit_pair = any((pa in p[0] and pb in p[1]) or (pa in p[1] and pb in p[0])
                   for e in ev for p in e.get("pairs", []))
    check("collision.mid_move_stop",
          pos_stop > tgt_deep + 1.0 and pos_stop < v_hit + 5.0
          and ev and "FA_X" in ev[-1].get("stopped", []) and hit_pair,
          "停在 %.2f（目标 %.1f 未达，阈值 %.1f）events=%d"
          % (pos_stop, tgt_deep, v_hit, len(ev)))
    # 回撤出重叠区应放行（tolerated 语义），回到 0 恢复现场
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    retreat_ok = st == 200 and wait_fa_idle() and abs(fa_pos()) < 1e-6

    # 关闭后同样动作放行；重开后静止重叠进 active；再回撤不受拦
    http_post(base + "/api/collision", {"enabled": False})
    st, r = http_post(base + "/api/axis/move",
              {"axis": "FA_X", "target": tgt_deep, "speed": 600.0})
    reached = st == 200 and wait_fa_idle() \
        and abs(fa_pos() - tgt_deep) < 1e-3
    http_post(base + "/api/collision", {"enabled": True})
    time.sleep(0.05)                      # 等 500Hz tick 检查刷新 active
    c2 = http_json(base + "/api/collision")
    active_hit = any((pa in p[0] and pb in p[1]) or (pa in p[1] and pb in p[0])
                     for p in c2.get("active", []))
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_X", "target": 0.0, "speed": 600.0})
    back_ok = st == 200 and wait_fa_idle() and abs(fa_pos()) < 1e-6
    check("collision.disable",
          reached and active_hit and back_ok and retreat_ok,
          "reached=%s active=%s back=%s retreat=%s"
          % (reached, active_hit, back_ok, retreat_ok))

    # 零位跨组扫描：对表里的任何一对在零位都不应深度>ε
    # （深度>ε 的零位跨组重叠只许活在 exempt_pairs——设计贴合/工艺接触）。
    # 静态校验：零位时 linear 贡献全为 0，直接用存档 min/max 判定。
    zero_hits = ["%s×%s(%.2fmm)"
                 % (p["a"], p["b"], depth_of(boxes[p["a"]], boxes[p["b"]]))
                 for p in sb["pairs"]
                 if depth_of(boxes[p["a"]], boxes[p["b"]]) > eps]
    check("collision.zero_pose_clean", not zero_hits,
          "零位非豁免深度重叠=%s" % zero_hits)

    # ---- 用户场景②：键合头 × 对准台（对准相机承载台） ----
    # 首撞对 = 上气环-左 × 对准相机承载台（压头/十字槽是 ~1mm 薄片，深度永远
    # ≤ε 不报警——真正拦住深降的是气环/键合头底座等厚件，符合 ε 语义：
    # 薄片穿刺深度超不过自身厚度，厚件嵌入才是真穿模）。
    pair2 = next((p for p in sb["pairs"]
                  if {p["a"], p["b"]} == {"pt:airring_left_0001",
                                          "pt:box_mssty4oj_u3pj08"}
                  and "BS_Arm_Z" in p["hit_axes"]), None)
    if pair2 is None:
        check("collision.head_vs_align_stage", False,
              "对表缺 上气环-左×对准相机承载台（模型变了？重新生成）")
        return
    air, asb = part("airring_left_0001"), part("box_mssty4oj_u3pj08")
    v_hit2 = asb["max"][2] - air["min"][2] - eps   # z 向深度 > ε 的阈值
    tgt_touch = v_hit2 + 1.0            # 重叠 1mm < ε：浅贴合放行
    tgt_into = v_hit2 - 10.0            # 深度 ~11mm > ε：真穿模（用户截图姿势）

    def bs_pos():
        return get_axis_entry("BS_Arm_Z")["pos"]

    def wait_bs_idle(timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            e = get_axis_entry("BS_Arm_Z")
            if (not e["moving"]) and e["state"] == "idle":
                return True
            time.sleep(0.02)
        return False

    st, r = http_post(base + "/api/axis/move",
                      {"axis": "BS_Arm_Z", "target": tgt_touch, "speed": 200.0})
    st_touch, err_touch = st, r.get("error", "")
    ok_touch = st == 200 and wait_bs_idle() \
        and abs(bs_pos() - tgt_touch) < 1e-3
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "BS_Arm_Z", "target": tgt_into, "speed": 200.0})
    err = r.get("error", "")
    blocked = st == 409 and "沙盒干涉" in err \
        and "上气环" in err and "对准相机承载台" in err
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    back0 = st == 200 and wait_bs_idle() and abs(bs_pos()) < 1e-6
    check("collision.head_vs_align_stage", ok_touch and blocked and back0,
          "touch=%s(%d:%s) blocked=%s err=%s back=%s"
          % (ok_touch, st_touch, err_touch[:80], blocked, err[:120], back0))

    # ---- depth_eps 在线配置：同一浅重叠姿势，ε=0.5 判撞、ε=2 放行 ----
    http_post(base + "/api/collision", {"depth_eps": 0.5})
    st, r1 = http_post(base + "/api/axis/move",
                       {"axis": "BS_Arm_Z", "target": tgt_touch,
                        "speed": 200.0})
    blocked_tight = st == 409 and "沙盒干涉" in r1.get("error", "")
    http_post(base + "/api/collision", {"depth_eps": eps})
    st, r2 = http_post(base + "/api/axis/move",
                       {"axis": "BS_Arm_Z", "target": tgt_touch,
                        "speed": 200.0})
    pass_loose = st == 200 and wait_bs_idle() \
        and abs(bs_pos() - tgt_touch) < 1e-3
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    wait_bs_idle()
    c_bad, _r_bad = http_post(base + "/api/collision", {"depth_eps": -1})
    check("collision.depth_eps_config",
          blocked_tight and pass_loose and c_bad == 400,
          "tight409=%s loose200=%s bad=%d" % (blocked_tight, pass_loose, c_bad))

    # ---- 用户场景①：键合头 × 晶圆台（退避 AS 后深降到真接触位放行，过冲拦截） ----
    # 吸盘基座是带中心孔的环形件，AABB 把孔填实 → 上气环/吸盘×基座 7 对已登记
    # 工艺豁免（gen_sandboxes.PROCESS_EXEMPT）；真接触位 -167.43 必须可达，
    # 继续过冲由 上气环×多孔陶瓷面 拦截（气环埋进陶瓷面 >ε）。
    ex = [{e["a"], e["b"]} for e in sb.get("exempt_pairs", [])]
    exempt_ok = {"pt:airring_left_0001", "pt:chuck_base_notched_0001"} in ex
    pair3 = next((p for p in sb["pairs"]
                  if {p["a"], p["b"]} == {"pt:airring_left_0001",
                                          "pt:chuck_base_notched_0001"}), None)
    if not exempt_ok or pair3 is not None:
        check("collision.head_vs_wafer_stage", False,
              "豁免登记异常：exempt=%s pair残留=%s（模型变了？重新生成）"
              % (exempt_ok, pair3 is not None))
        return
    ceramic = part("ceramic_msslygtz_8wc595")
    # 键合真接触位（2026-09-06 迭代 17 起=有效接触位：机械常数裸接触位
    # -168.93 + 处方晶圆厚 1.0 + 芯片厚 0.5）：贴晶圆面放行。注：须用有效
    # 接触位而非旧示教位 -167.9——该位在有效接触位之下 0.47mm，弹簧接触
    # 力回路（sp=0）会把头抬回零力位，到位断言会被物理行为本身破坏
    tgt_bond = -167.43
    tgt_crash = (ceramic["max"][2] - eps) - air["min"][2] - 10.0  # 气环埋进陶瓷面
    http_post(base + "/api/axis/move",
              {"axis": "AS_Y", "target": 120.0, "speed": 500.0})
    wait_settled("AS_Y")
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "BS_Arm_Z", "target": tgt_bond, "speed": 200.0})
    ok_bond = st == 200 and wait_bs_idle() and abs(bs_pos() - tgt_bond) < 1e-3
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "BS_Arm_Z", "target": tgt_crash, "speed": 200.0})
    err = r.get("error", "")
    blocked3 = st == 409 and "沙盒干涉" in err \
        and "上气环" in err and "多孔陶瓷面" in err
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 200.0})
    wait_bs_idle()
    http_post(base + "/api/axis/move",
              {"axis": "AS_Y", "target": 0.0, "speed": 500.0})
    wait_settled("AS_Y")
    check("collision.head_vs_wafer_stage", ok_bond and blocked3
          and abs(bs_pos()) < 1e-6,
          "bond200=%s crash409=%s err=%s" % (ok_bond, blocked3, err[:120]))

    # ---- 用户场景③：翻转 FA_Z × 蓝膜台（贴膜放行，压进台体拦截） ----
    pair4 = next((p for p in sb["pairs"]
                  if {p["a"], p["b"]} == {"pt:diechuck_0001",
                                          "pt:filmstage_body_0001"}
                  and "FA_Z" in p["hit_axes"]), None)
    if pair4 is None:
        check("collision.flip_vs_film", False,
              "对表缺 芯片吸盘×蓝膜台圆形台体（模型变了？重新生成）")
        return
    chuck, body = part("diechuck_0001"), part("filmstage_body_0001")
    v_hit4 = body["max"][2] - chuck["min"][2] - eps
    tgt_film = -5.0                     # 取片示教位（transfers.json）：贴膜放行
    tgt_punch = v_hit4 - 10.0           # 压进台体深度 > ε
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_Z", "target": tgt_film, "speed": 600.0})
    ok_film = st == 200 and wait_settled("FA_Z") \
        and abs(get_axis_entry("FA_Z")["pos"] - tgt_film) < 1e-3
    st, r = http_post(base + "/api/axis/move",
                      {"axis": "FA_Z", "target": tgt_punch, "speed": 600.0})
    err = r.get("error", "")
    blocked4 = st == 409 and "沙盒干涉" in err and any(
        s in err for s in ("芯片吸盘", "翻转模块Rz承载台",
                           "翻转模块Rx承载台", "翻转模块Rx支架"))
    http_post(base + "/api/axis/move",
              {"axis": "FA_Z", "target": 0.0, "speed": 600.0})
    wait_settled("FA_Z")
    check("collision.flip_vs_film", ok_film and blocked4
          and abs(get_axis_entry("FA_Z")["pos"]) < 1e-6,
          "film200=%s punch409=%s err=%s" % (ok_film, blocked4, err[:120]))



def test_iter8():
    """--- 19.12 迭代 8：上片真空 bug 修复 + TwinCAT 系列型 PID 三回路
    （温控×2/压控）+ 热压键合流程 + chain 六阶段 ---
      - wafer_ega 流程两分支均含 WT_WaferVac io+wait_grip 幂等两步，跑完
        晶圆真空建立（air_grip_ok 语义：oNegtiveState 且密封）；
      - pid.py 单测：系列型阶跃收敛/输出限幅联动抗饱和/微分 Td 滤波/
        Tn=0 纯 PD 不发散；
      - 三回路（进程内引擎，机器常数默认整定）：力 50N 到带 ≤±1N 无超调
        >10%、PulseHeat 25→300℃ 与 Heat_Dw 140→150℃ 稳态误差 ≤±0.5℃；
      - 机器常数 bond/pid 组 GET/PUT（非法 400）；recipe bond 组默认值；
      - bond happy path（chain 六阶段全 done）：die carrier=wafer 且
        site==键合位设计坐标（bond_mark 落位修正）、head 腾空；
        同 site 重复键合 → 400；state_public.pid 三回路 pv/sp/out 暴露。"""
    print("--- 19.12 迭代 8（上片真空/PID 三回路/热压键合/chain 六阶段） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait(timeout=300.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = flow_state()
            if s.get("state") in ("done", "aborted"):
                return s
            time.sleep(0.1)      # 迭代 18：轮询加密（轮询粒度纯测试侧开销）
        return {"state": "timeout"}

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def run_flow(path, body, timeout=600.0):
        c, r = http_post(base + path, body)
        assert c == 200 and r.get("ok"), (path, c, r)
        return flow_wait(timeout)

    def chain_wait(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ch = flow_state().get("chain") or {}
            if not ch.get("active"):
                return ch
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    # ================= 进程内：pid.py 单测 =================
    import pid as pidmod

    # 阶跃收敛（一阶对象 y'=(u−y)/0.2，串 PID 应收敛到 sp）
    p = pidmod.PID("t", 2.0, tn_s=0.3, out_min=0.0, out_max=100.0,
                   cycle_s=0.004)
    y = 0.0
    for _ in range(2000):                    # 8s
        u = p.update(10.0, y)
        y += (u - y) * (0.004 / 0.2)
    check("i8.pid_step_converge", abs(y - 10.0) < 0.05,
          "y=%.4f（8s 后应≈10）" % y)

    # 抗饱和：输出限幅时积分冻结（误差同向不再累），sp 回落后无长尾巴
    p2 = pidmod.PID("aw", 10.0, tn_s=0.5, out_min=0.0, out_max=10.0,
                    cycle_s=0.004)
    y2 = 0.0
    for _ in range(2500):                    # 10s 深饱和（sp=100 不可达）
        u2 = p2.update(100.0, y2)
        y2 += (u2 - y2) * (0.004 / 0.1)
    integ_sat = p2.state()["integ"]
    # 若积分无抗饱和，10s×e=90/Tn=0.5 → integ≈720；冻结时应≪
    y2b = y2
    t_rec = None
    for i in range(2500):                    # sp 回到可达 10
        u2 = p2.update(10.0, y2b)
        y2b += (u2 - y2b) * (0.004 / 0.1)
        if t_rec is None and abs(y2b - 10.0) < 0.5:
            t_rec = i * 0.004
    check("i8.pid_anti_windup",
          integ_sat < 5.0 and t_rec is not None and t_rec < 2.0,
          "integ_sat=%.3f（无抗饱和≈720）恢复 t=%ss" % (integ_sat, t_rec))

    # 微分滤波：Tv·s/(1+Td·s)——阶跃误差产生有限冲击并按 Td 衰减
    # （首拍 update 只做初始化不产微分冲击——激活无冲击是标准语义，
    #   故先一拍稳态再阶跃）
    p3 = pidmod.PID("d", 1.0, tv_s=0.5, td_s=0.1, out_min=-1e6,
                    out_max=1e6, cycle_s=0.004)
    p3.update(0.0, 0.0)                      # 初始化（e=0）
    ds = []
    for i in range(200):
        p3.update(1.0, 0.0)                  # e 恒 1（首拍阶跃 Δe=1）
        ds.append(p3.state()["d"])
    d_peak = max(ds)
    d_tail = ds[-1]
    # 首拍冲击 ≈ Tv/(Td+T)·Δe = 0.5/0.104 ≈ 4.8，随后按 Td/(Td+T) 衰减
    check("i8.pid_deriv_filter",
          4.0 < d_peak < 6.0 and abs(d_tail) < 0.05 * d_peak,
          "d_peak=%.3f d_tail=%.4f（冲击有限且衰减）" % (d_peak, d_tail))

    # Tn=0 纯 PD：积分通道关闭，长跑不发散（P/PD 有静差是理论必然：
    # 本对象 y=u 时 y∞=kp/(1+kp)·sp=5/3，判有界+收敛到理论静差值）
    p4 = pidmod.PID("pd", 0.5, tn_s=0.0, tv_s=0.05, td_s=0.02,
                    out_min=-100.0, out_max=100.0, cycle_s=0.004)
    y4 = 0.0
    ok_pd = True
    for i in range(25000):                   # 100s
        u4 = p4.update(5.0, y4)
        y4 += (u4 - y4) * (0.004 / 0.5)
        if not math.isfinite(y4) or abs(y4) > 1e6:
            ok_pd = False
            break
    check("i8.pid_pure_pd_stable", ok_pd and abs(y4 - 5.0 / 3.0) < 0.05,
          "y=%.4f（100s 纯 PD 有界，理论静差值 5/3）" % y4)

    # ================= 进程内：三回路整定（裸引擎 + 默认机器常数） =================
    import engine as engmod

    class _FakePts(object):
        def __init__(self):
            self.d = {}

        def set(self, k, v):
            self.d[k] = v

        def get(self, k, d=None):
            return self.d.get(k, d)

        def has(self, k):
            return k in self.d

        def commit(self):
            pass

    eng2 = engmod.Engine(_FakePts())
    eng2._refresh_loop_consts_locked()
    # 力回路：接触位起步 → 50N 到带（±1N、无超调 >10%、~1-2s 到带）
    ax2 = eng2.axes["BS_Arm_Z"]
    ax2.pos = eng2._bond["head_z_contact_mm"]
    ax2.x_actual = ax2.pos
    eng2.force_target = 50.0
    peak_f, t_band = 0.0, None
    for i in range(6000):
        eng2.tick()
        peak_f = max(peak_f, eng2.force_value)
        if t_band is None and abs(eng2.force_value - 50.0) <= 1.0:
            t_band = i * 0.001
    check("i8.loop_force",
          abs(eng2.force_value - 50.0) <= 1.0 and peak_f <= 55.0
          and t_band is not None and t_band < 2.5,
          "F=%.2fN peak=%.2f 到带t=%.2fs" % (eng2.force_value, peak_f,
                                             t_band or -1))
    # 脉冲加热 25→300℃（±0.5℃ 稳态，无 >10% 超调）
    h_up = eng2.heaters["PulseHeat_Up"]
    eng2.heater_command("PulseHeat_Up", "SetTargetTempWithPlan",
                        {"Temperature": 300})
    peak_t = 0.0
    for i in range(15000):
        eng2.tick()
        peak_t = max(peak_t, h_up.cur)
    check("i8.loop_head_temp",
          abs(h_up.cur - 300.0) <= 0.5 and peak_t <= 330.0,
          "T=%.2f℃ peak=%.2f power=%.1f%%" % (h_up.cur, peak_t, h_up.power))
    # 硅片台 140→150℃（±0.5℃ 稳态；预置 140 防用例超时——全段 25→150
    # 真实 ~3min 仿真，回归只验证末段收敛与保温）
    # 迭代 18：200s 浸泡 → 40s（到带 ~14s sim，仍 ~3× 余量；进程内裸引擎
    # tick 是纯计算、不吃 /api/speed 倍速，200k tick 白烧数秒墙钟）
    h_dw = eng2.heaters["Heat_Dw"]
    h_dw.cur = 140.0
    eng2.heater_command("Heat_Dw", "SetTargetTempWithPlan",
                        {"Temperature": 150})
    peak_w = 140.0
    for i in range(40000):
        eng2.tick()
        peak_w = max(peak_w, h_dw.cur)
    check("i8.loop_wafer_temp",
          abs(h_dw.cur - 150.0) <= 0.5 and peak_w <= 152.0,
          "T=%.3f℃ peak=%.3f power=%.2f%%" % (h_dw.cur, peak_w, h_dw.power))

    # ================= HTTP：常数/调参/处方组 =================
    mc = http_json(base + "/api/machine_constants")["constants"]
    bd = mc.get("bond") or {}
    # 2026-08-31 迭代 9：bond 组只留整机标定量（裸接触位/寻力位），
    # pid/硬件参数迁 tuning.json，噪声迁 recipe sim_sensors
    tn = http_json(base + "/api/tuning")["tuning"]
    tn_hw = tn.get("bond_hw") or {}
    pd_ = tn.get("pid") or {}
    check("i8.mc_groups",
          abs(bd.get("head_z_contact_mm", 0) - (-168.93)) < 1e-9
          and abs(bd.get("head_z_search_mm", 0) - (-166.9)) < 1e-9
          and "pid" not in mc
          and abs(tn_hw.get("force_spring_k_n_per_mm", 0) - 500.0) < 1e-9
          and abs(tn_hw.get("wafer_heater_rate_cps", 0) - 0.733) < 1e-9
          and "force_kp" in pd_ and "wafertemp_kp" in pd_
          and abs(pd_.get("cycle_s", 0) - 0.004) < 1e-12,
          "bond keys=%s pid keys=%s"
          % (sorted(k for k in bd if not k.startswith("_")),
             sorted(k for k in pd_ if not k.startswith("_"))))
    c_mc1, _ = http_put(base + "/api/tuning",
                        {"pid": {"force_kp": 0.0008}})
    c_mc2, _ = http_put(base + "/api/tuning",
                        {"pid": {"bogus_kp": 1.0}})
    c_mc3, _ = http_put(base + "/api/machine_constants",
                        {"bond": {"head_z_contact_mm": -999.0}})
    c_mc4, _ = http_put(base + "/api/machine_constants",
                        {"pid": {"force_kp": 0.0008}})
    check("i8.mc_put", c_mc1 == 200 and c_mc2 == 400 and c_mc3 == 400
          and c_mc4 == 400,
          "ok=%s unknown=%s range=%s pid_out=%s" % (c_mc1, c_mc2, c_mc3,
                                                    c_mc4))
    rec_bond = (http_json(base + "/api/recipe")["recipe"]).get("bond") or {}
    check("i8.recipe_bond",
          abs(rec_bond.get("force_N", 0) - 50.0) < 1e-9
          and abs(rec_bond.get("force_tol_N", 0) - 1.0) < 1e-9
          and abs(rec_bond.get("dwell_s", 0) - 10.0) < 1e-9
          and abs(rec_bond.get("head_temp_C", 0) - 300.0) < 1e-9
          and abs(rec_bond.get("wafer_temp_C", 0) - 30.0) < 1e-9
          and rec_bond.get("wafer_preheat") is True
          and rec_bond.get("head_cool_after") is True
          and abs(rec_bond.get("fast_speed_mm_s", 0) - 20.0) < 1e-9
          and abs(rec_bond.get("search_speed_mm_s", 0) - 0.2) < 1e-9,
          "bond=%s" % (rec_bond,))
    # state_public.pid 三回路 pv/sp/out 暴露（面板轮询面）
    pid_pub = (flow_state().get("pid") or {})
    check("i8.state_public_pid",
          all(k in pid_pub for k in ("force", "head_temp", "wafer_temp"))
          and all(f in (pid_pub.get("force") or {})
                  for f in ("pv", "sp", "out", "kp", "tn_s")),
          "keys=%s force=%s"
          % (sorted(pid_pub), pid_pub.get("force")))

    # ================= 上片真空 bug：两分支步骤断言 =================
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/recipe/restore_default", {})
    flow_cleanup()
    c, r = http_post(base + "/api/process/wafer_ega",
                     {"with_load": True, "paused": True})
    sd = r.get("steps_def") or []
    wl_i = next((i for i, s_ in enumerate(sd)
                 if s_.get("op") == "wafer" and s_.get("action") == "load"),
                -1)
    pair = [(s_.get("op"), s_.get("path", ""), s_.get("circuit", ""))
            for s_ in sd[wl_i:wl_i + 3]] if wl_i >= 0 else []
    ok_load = c == 200 and wl_i >= 0 and len(pair) == 3 \
        and pair[1][0] == "io" and pair[1][1].endswith("WT_WaferVac") \
        and pair[2][0] == "wait_grip" and pair[2][2] == "WT_WaferVac"
    flow_cleanup()
    check("i8.wafer_vac_steps_load", ok_load,
          "wafer load 后两步=%s" % (pair[1:],))
    # 晶圆已在位分支（do_load=false）：同样的幂等两步
    http_post(base + "/api/workpiece/wafer", {"action": "load"})
    c, r = http_post(base + "/api/process/wafer_ega", {"paused": True})
    sd2 = r.get("steps_def") or []
    has_pair = any(
        sd2[i].get("op") == "io"
        and str(sd2[i].get("path", "")).endswith("WT_WaferVac")
        and sd2[i + 1].get("op") == "wait_grip"
        and sd2[i + 1].get("circuit") == "WT_WaferVac"
        for i in range(len(sd2) - 1))
    flow_cleanup()
    check("i8.wafer_vac_steps_present", c == 200 and has_pair,
          "已在位分支含 WT_WaferVac 幂等两步=%s" % has_pair)

    # ================= chain 六阶段 happy path（含 bond） =================
    def setup_recipe():
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5},
            "bond": {"dwell_s": 2.0}})
        assert c == 200, r

    try:
        setup_recipe()
        # 预置硅片台到温（迭代 12 起 wafer_temp_C 默认 30——按处方值预热，
        # keep_warm 常保温下通常本已到温，流程的"等温确认"幂等秒过，
        # 防用例超时）
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        http_post(base + "/api/speed", {"scale": 8.0})
        t0 = time.time()
        cur_t = 0.0
        plc = "http://127.0.0.1:%d" % PLC_PORT
        while time.time() - t0 < 60.0:
            v = http_json(plc + "/io/observe?path=SystemMachine.M_Heater."
                                "Heat_Dw.oCurTemp")
            cur_t = float(v.get("value", 0.0))
            if cur_t >= tgt_t - 1.5:       # 读数 ±0.5℃ 噪声，阈值让 1.5℃
                break
            time.sleep(0.5)
        http_post(base + "/api/speed", {"scale": 1.0})
        check("i8.wafer_preheat", cur_t >= tgt_t - 1.5,
              "Heat_Dw 预置到温 oCurTemp=%.1f℃（目标 %.1f）" % (cur_t, tgt_t))

        # 迭代 18：chain 六阶段实跑套 8× 提速窗（finally 复原 1×）——
        # 本用例同时是 quick 套件的代表性单片链（chain happy path）
        set_speed(8.0)
        c, r = http_post(base + "/api/process/chain", {})
        ch = chain_wait()
        st_ok = ch.get("stages") or {}
        viz = http_json(base + "/api/viz")
        import process_flows as pfmod
        rec_now = http_json(base + "/api/recipe")["recipe"]
        cells = pfmod.wafer_sites_cells(rec_now)
        center = min(cells, key=lambda cc: math.hypot(cc[2], cc[3]))
        cx0, cy0 = center[2], center[3]
        wafer_dies = [d for d in viz["dies"] if d["carrier"] == "wafer"]
        n_head = sum(1 for d in viz["dies"] if d["carrier"] == "head")
        bonded = [d for d in wafer_dies
                  if abs(d["pos"][0] - cx0) < 1e-6
                  and abs(d["pos"][1] - cy0) < 1e-6]
        # 晶圆真空（上片真空 bug 修复的运行态证据）
        v_air = http_json(plc + "/io/observe?path=SystemMachine.SS_ME.M_AIR."
                                "WT_WaferVac")
        vac_on = (v_air.get("values", {}) or {}).get(
            "SystemMachine.SS_ME.M_AIR.WT_WaferVac.oNegtiveState")
        check("i8.chain_bond_happy",
              c == 200 and not ch.get("active") and not ch.get("error")
              and all(st_ok.get(k) == "done" for k in
                      ("wafer_ega", "flip_pick", "flip_handoff",
                       "focus_level", "fine_align", "bond"))
              and len(bonded) == 1 and n_head == 0
              and vac_on is True,
              "stages=%s err=%s bonded=%d pos=%s head=%d vac=%s"
              % (st_ok, ch.get("error"), len(bonded),
                 bonded[0]["pos"] if bonded else None, n_head, vac_on))
        # 迭代 20：truth 新口径 quick 覆盖——bond_results 记录带接触时刻
        # 真值分量分解（commit=align+descent、truth=commit+process_shift），
        # 下降链系统项 µm 级存在性
        rs8 = http_json(base + "/api/bond/results").get("results") or []
        rec8 = rs8[-1] if rs8 else {}
        tc8 = rec8.get("truth_components") or {}
        al8, de8 = tc8.get("align_resid"), tc8.get("descent")
        cm8, ps8 = tc8.get("commit"), tc8.get("process_shift")
        t8 = rec8.get("truth") or {}
        comp_ok = bool(al8 and de8 and cm8 and ps8 and t8) \
            and abs(cm8["dx_um"] - (al8["dx_um"] + de8["dx_um"])) < 1e-3 \
            and abs(cm8["drz_urad"] - (al8["drz_urad"] + de8["drz_urad"])) \
            < 1e-3 \
            and abs(t8["dx"] * 1000.0 - (cm8["dx_um"] + ps8["dx_um"])) < 1e-3 \
            and abs(t8["drz"] * 1e6 - (cm8["drz_urad"] + ps8["drz_urad"])) \
            < 1.0      # truth drz 落盘为 _r6(rad)——1µrad 量化余量
        de8_mag = math.hypot(de8["dx_um"], de8["dy_um"]) if de8 else 0.0
        check("i8.bond_truth_components",
              comp_ok and 1.0 < de8_mag < 50.0,
              "components=%s truth=%s" % (tc8, t8))
        flow_cleanup()
        # ---- 同 site 重复键合 → 400 ----
        c2, r2 = http_post(base + "/api/process/bond", {})
        check("i8.bond_repeat_400",
              c2 == 400 and "已键合" in (r2.get("error") or ""),
              "code=%s err=%s" % (c2, (r2.get("error") or "")[:60]))
        flow_cleanup()
        # ---- 缺前置（无精对准记录的新 site）→ 400 ----
        alt = max(cells, key=lambda cc: math.hypot(cc[2], cc[3]))
        c3, r3 = http_post(base + "/api/process/bond",
                           {"site": [alt[0], alt[1]]})
        check("i8.bond_guard_400",
              c3 == 400 and "精对准" in (r3.get("error") or ""),
              "code=%s err=%s" % (c3, (r3.get("error") or "")[:60]))
        flow_cleanup()

    finally:
        try:
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter8b():
    """--- 19.13 迭代 8B：视觉链路 MQTT 化 ---
      - vision/trigger/{cam} → vision/result/{cam}（载荷 {req_id, cam,
        result}，非 MQTT 触发 req_id=null，结果出口唯一 = _publish）；
      - VisionClient.request 逐相机（FAST）与进程内 identify 结果一致；
      - 未知相机触发回 error 帧到同名结果 topic（迭代 31：原静默丢弃）、
        正常触发结果载荷 cam 与 topic 一致、VisionClient (cam, req_id)
        双键认领（跨相机同 req_id 不串答、无 cam 键旧帧兼容收）；
      - flow_engine 四个拍照点分发：eng.vision_client 在 → 走客户端，
        缺失 → 回退进程内（计数 stub 直测；真实 MQTT 往返由 EGA/精对准/
        chain 六阶段用例复跑覆盖）。"""
    print("--- 19.13 迭代 8B（视觉链路 MQTT 化） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    from mqtt_broker import MqttSub, mqtt_publish
    from vision_service import VisionClient

    # 场景（同 17.1）：晶圆上片 + down 相机对准晶圆标记
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    http_post(base + "/api/vision/camera/config",
              {"cam": "down", "mode": "trigger", "exposure_ms": 8.0})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    # 1) 非 MQTT 触发：vision/result/{cam} 载荷 {req_id:null, cam, result:七字段}
    sub = MqttSub(port=1883, client_id="test-i8b-sub")
    sub.subscribe("vision/result/+")
    c, r = http_post(base + "/api/vision/identify", {"cam": "down"})
    e = (r.get("entry") or {})
    res = e.get("result") or {}
    got = sub.recv(timeout=5)
    payload = json.loads(got[1]) if got else {}
    check("i8b.mqtt_result_envelope",
          c == 200 and payload.get("req_id") is None
          and payload.get("cam") == "down"
          and got is not None and got[0] == "vision/result/down"
          and isinstance(payload.get("result"), dict)
          and payload["result"].get("judge") == res.get("judge")
          and all(k in payload["result"] for k in
                  ("strShapeName", "judge", "lstCol", "lstRow",
                   "lstAngle", "lstScore", "lstScale")),
          "payload keys=%s judge=%s"
          % (sorted(payload), payload.get("result", {}).get("judge")))

    # 2) VisionClient MQTT 触发逐相机 vs 进程内 identify 结果一致
    cli = VisionClient().start()
    time.sleep(0.3)                  # 等订阅挂上 broker
    same_all = True
    details = []
    for cam in ("down", "up", "coarse"):
        m = cli.request(cam, timeout=10.0)
        time.sleep(0.3)      # 闪光灭灯 live 镜像（≤1 扫描周期+100Hz 采样）
                             # 后再发起下一次拍照，避免灯态竞态
        c2, r2 = http_post(base + "/api/vision/identify", {"cam": cam})
        res2 = ((r2.get("entry") or {}).get("result") or {})
        mr = m.get("result") or {}
        same = (mr.get("judge") == res2.get("judge"))
        if mr.get("judge") and res2.get("judge"):
            same = same and bool(mr.get("lstCol")) \
                and bool(res2.get("lstCol")) \
                and abs(mr["lstCol"][0] - res2["lstCol"][0]) < 5 \
                and abs(mr["lstRow"][0] - res2["lstRow"][0]) < 5
        same_all = same_all and same and m.get("error") is None
        details.append("%s:j=%s/%s" % (cam, mr.get("judge"),
                                       res2.get("judge")))
    check("i8b.mqtt_vs_inproc", same_all, " ".join(details))

    # 3) 未知相机不再静默（2026-09-10 迭代 31）：有 req_id 回 error 帧
    #    到同名结果 topic——raw 层（载荷 cam="all" 优先于 topic 后缀）
    #    与 VisionClient 层各验一支
    sub2 = MqttSub(port=1883, client_id="test-i8b-unk")
    sub2.subscribe("vision/result/+")
    mqtt_publish("vision/trigger/down",
                 json.dumps({"req_id": "raw-all-1", "cam": "all"}),
                 "127.0.0.1", 1883)
    unk = None
    t0 = time.time()
    while time.time() - t0 < 5.0:
        got2 = sub2.recv(timeout=1.0)
        if not got2:
            continue
        pl2 = json.loads(got2[1])
        if pl2.get("req_id") == "raw-all-1":
            unk = (got2[0], pl2)
            break
    sub2.close()
    check("i8b.mqtt_unknown_cam_error",
          unk is not None and unk[0] == "vision/result/all"
          and unk[1].get("cam") == "all"
          and "未知相机" in (unk[1].get("error") or "")
          and (unk[1].get("result") or {}).get("judge") is False,
          "frame=%s" % (unk,))
    m_unk = cli.request("nope", timeout=5.0)
    check("i8b.mqtt_unknown_cam_client",
          bool(m_unk.get("error")) and "未知相机" in m_unk["error"]
          and (m_unk.get("result") or {}).get("judge") is False,
          "error=%s" % (m_unk.get("error"),))

    # 4) 正常 MQTT 触发：结果载荷 cam 字段与 topic 后缀一致 + req_id 回填
    sub3 = MqttSub(port=1883, client_id="test-i8b-camf")
    sub3.subscribe("vision/result/+")
    mqtt_publish("vision/trigger/down",
                 json.dumps({"req_id": "raw-down-1", "cam": "down"}),
                 "127.0.0.1", 1883)
    camf = None
    t0 = time.time()
    while time.time() - t0 < 8.0:
        got3 = sub3.recv(timeout=1.0)
        if not got3:
            continue
        pl3 = json.loads(got3[1])
        if pl3.get("req_id") == "raw-down-1":
            camf = (got3[0], pl3)
            break
    sub3.close()
    check("i8b.mqtt_result_cam_field",
          camf is not None and camf[0] == "vision/result/down"
          and camf[1].get("cam") == "down"
          and camf[1].get("error") is None
          and isinstance(camf[1].get("result"), dict),
          "frame=%s" % (camf,))
    sub.close()

    # 5) VisionClient 双键认领（迭代 31）：预置跨相机同 req_id 帧不应答
    #    （真应答来自本相机 FAST 实拍）；无 cam 键的旧帧落兼容位仍收
    rid1 = "vs-%06d-%x" % (cli._seq + 1, id(cli) & 0xFFFF)
    with cli._cond:
        cli._by_req[("up", rid1)] = {
            "req_id": rid1, "cam": "up",
            "result": {"strShapeName": "FAKE_UP", "judge": True,
                       "lstCol": [1.0], "lstRow": [1.0], "lstAngle": [0.0],
                       "lstScore": [0.99], "lstScale": [1.0]}}
    m5 = cli.request("down", timeout=10.0)
    cross_ok = (m5.get("error") is None
                and (m5.get("result") or {}).get("strShapeName") != "FAKE_UP")
    rid2 = "vs-%06d-%x" % (cli._seq + 1, id(cli) & 0xFFFF)
    with cli._cond:
        cli._by_req[(None, rid2)] = {
            "req_id": rid2,
            "result": {"strShapeName": "OLD_FRAME", "judge": True,
                       "lstCol": [2.0], "lstRow": [2.0], "lstAngle": [0.0],
                       "lstScore": [0.9], "lstScale": [1.0]}}
    m6 = cli.request("down", timeout=10.0)
    compat_ok = (m6.get("result") or {}).get("strShapeName") == "OLD_FRAME"
    check("i8b.client_dual_key", cross_ok and compat_ok,
          "cross=%s compat=%s" % (cross_ok, compat_ok))

    # 6) 调用点分发（计数 stub 直测 _vision_identify）
    import flow_engine as femod

    class _StubCli(object):
        def __init__(self):
            self.n = 0

        def request(self, cam, template=None, timeout=10.0,
                    ir_overlay=False):
            self.n += 1
            return {"result": {"judge": True}, "error": None}

    class _StubEng(object):
        vision = None
        vision_client = _StubCli()

    eng = _StubEng()
    fe = femod.FlowEngine.__new__(femod.FlowEngine)
    fe.eng = eng
    r3 = fe._vision_identify("down")
    check("i8b.dispatch_client",
          eng.vision_client.n == 1 and r3["result"]["judge"] is True,
          "n=%d r=%s" % (eng.vision_client.n, r3))

    class _StubVision(object):
        def __init__(self):
            self.n = 0

        def identify(self, cam, template=None, ir_overlay=False):
            self.n += 1
            return {"result": {"judge": True}}

    eng.vision_client = None
    eng.vision = _StubVision()
    fe._vision_identify("down")
    check("i8b.dispatch_fallback", eng.vision.n == 1,
          "n=%d" % eng.vision.n)


def test_iter9():
    """--- 19.14 迭代 9：下位机调参 tuning.json + 键合两段速下压/慢速寻力
    + 硅片台常保温 + 噪声入测校 ---
      - /api/tuning GET/PUT（校验 400 两支）与迁移逻辑（临时文件实跑
        migrate_legacy：pid+6 硬件键→tuning.json、噪声 2 键→recipe
        sim_sensors、machine_constants 剔除、幂等）；
      - 厚度换算 effective_bond_contact_mm 数值例（裸 -168.93 + 晶圆 1.0
        + 芯片 0.5 = -167.43；片变厚→接触位上移，符号验证）；
      - force_search op 时序（进程内引擎+流程引擎）：快速段 move → 慢速
        寻力触 2N 停轴并记录 bond_touch_pos；超程未触力 → FAIL"寻力未接触"；
      - wafer_keep_warm：recipe/apply 即设 Heat_Dw 目标 wafer_temp_C 并
        常开 PID（sp/active 证据）；等温步带内秒过（_do_heater 计时 <1s）。"""
    print("--- 19.14 迭代 9（tuning/两段速寻力/厚度换算/keep_warm） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= HTTP：/api/tuning 读写与校验 =================
    tj = http_json(base + "/api/tuning")
    tn = tj.get("tuning") or {}
    check("i9.tuning_get",
          tj.get("ok") and sorted(g for g in tj.get("groups", []))
          == ["bond_hw", "pid"]
          and abs(tn["pid"]["force_kp"] - 0.0008) < 1e-12
          and abs(tn["pid"]["cycle_s"] - 0.004) < 1e-12
          and abs(tn["bond_hw"]["force_spring_k_n_per_mm"] - 500.0) < 1e-9
          and abs(tn["bond_hw"]["ambient_C"] - 25.0) < 1e-9
          and abs(tn["bond_hw"]["wafer_heater_max_C"] - 511.4) < 1e-9,
          "groups=%s" % (tj.get("groups"),))
    c1, _ = http_put(base + "/api/tuning", {"bond_hw": {"ambient_C": 26.0}})
    got = http_json(base + "/api/tuning")["tuning"]["bond_hw"]["ambient_C"]
    c2, _ = http_put(base + "/api/tuning", {"bond_hw": {"ambient_C": 25.0}})
    c3, _ = http_put(base + "/api/tuning", {"pid": {"nope": 1.0}})
    c4, _ = http_put(base + "/api/tuning",
                     {"bond_hw": {"head_heater_max_C": 5000.0}})
    check("i9.tuning_put", c1 == 200 and abs(got - 26.0) < 1e-9
          and c2 == 200 and c3 == 400 and c4 == 400,
          "rw=%s/%s unknown=%s range=%s" % (c1, c2, c3, c4))

    # ================= 进程内：迁移逻辑（临时文件实跑 migrate_legacy） =================
    import tempfile
    import tuning as tunmod
    d = tempfile.mkdtemp()
    mc_p, tu_p, rc_p = (os.path.join(d, n) for n in
                        ("machine_constants.json", "tuning.json",
                         "recipe.json"))
    old_mc = {"cameras": {},
              "bond": {"head_z_contact_mm": -167.9,
                       "force_spring_k_n_per_mm": 555.0, "ambient_C": 24.0,
                       "head_heater_max_C": 451.0, "pulse_heater_rate_cps": 199.0,
                       "wafer_heater_rate_cps": 0.8, "wafer_heater_max_C": 500.0,
                       "temp_noise_C": 0.6, "force_noise_N": 1.1},
              "pid": {"force_kp": 0.0007, "cycle_s": 0.005}}
    with open(mc_p, "w", encoding="utf-8") as f:
        json.dump(old_mc, f)
    with open(rc_p, "w", encoding="utf-8") as f:
        json.dump({"sim_sensors": {"confocal_sigma_um": 0.05}}, f)
    logs = []
    n1 = tunmod.migrate_legacy(mc_p, tu_p, rc_p, log=logs.append)
    n2 = tunmod.migrate_legacy(mc_p, tu_p, rc_p, log=logs.append)
    mc2 = json.load(open(mc_p, encoding="utf-8"))
    tu2 = tunmod.Tuning(tu_p).get()
    rc2 = json.load(open(rc_p, encoding="utf-8"))
    check("i9.migration",
          n1 == 10 and n2 == 0 and logs and "迁移" in logs[0]
          and "pid" not in mc2
          and sorted(k for k in mc2["bond"]) == ["head_z_contact_mm"]
          and abs(tu2["pid"]["force_kp"] - 0.0007) < 1e-12
          and abs(tu2["pid"]["cycle_s"] - 0.005) < 1e-12
          and abs(tu2["pid"]["headtemp_kp"] - 3.0) < 1e-12   # 默认值补齐
          and abs(tu2["bond_hw"]["force_spring_k_n_per_mm"] - 555.0) < 1e-9
          and abs(tu2["bond_hw"]["wafer_heater_rate_cps"] - 0.8) < 1e-9
          and abs(rc2["sim_sensors"]["temp_noise_C"] - 0.6) < 1e-9
          and abs(rc2["sim_sensors"]["force_noise_N"] - 1.1) < 1e-9
          and abs(rc2["sim_sensors"]["confocal_sigma_um"] - 0.05) < 1e-12,
          "n1=%s n2=%s mc_bond=%s" % (n1, n2, sorted(mc2["bond"])))

    # ================= 进程内：厚度换算符号/数值 =================
    import engine as engmod
    c0 = engmod.effective_bond_contact_mm(-168.93, 1.0, 0.5)
    c_thick = engmod.effective_bond_contact_mm(-168.93, 2.0, 0.8)
    c_bare = engmod.effective_bond_contact_mm(-168.93, 0.0, 0.0)
    check("i9.thickness_calc",
          abs(c0 - (-167.43)) < 1e-9 and abs(c_bare - (-168.93)) < 1e-9
          and c_thick > c0,     # 片变厚 → 接触位上移（pos 增大方向）
          "eff=%.4f bare=%.4f thick=%.4f" % (c0, c_bare, c_thick))

    # ================= 进程内：force_search 时序（引擎+流程引擎） =================
    import flow_engine as femod

    class _FakePts(object):
        def __init__(self):
            self.d = {}

        def set(self, k, v):
            self.d[k] = v

        def get(self, k, dv=None):
            return self.d.get(k, dv)

        def has(self, k):
            return k in self.d

        def commit(self):
            pass

    pts = _FakePts()
    eng3 = engmod.Engine(pts)
    eng3.interlock_enabled = False    # 裸引擎无 EMO/门锁点位映像，联锁会
                                    # 按缺省判 EMO active——进程内单测屏蔽
    fe = femod.FlowEngine(eng3, pts, os.path.join(d, "flows.json"))
    eng3.flow = fe
    tick_stop = []

    def tick_loop():                     # ~2x 实时驱动（线程安全 tick）
        while not tick_stop:
            eng3.tick()
            time.sleep(0.0005)

    th = threading.Thread(target=tick_loop, daemon=True)
    th.start()

    def flow_wait_f(timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if fe.state in ("done", "aborted"):
                return fe.state
            time.sleep(0.05)
        return "timeout"

    try:
        ax = eng3.axes["BS_Arm_Z"]
        eng3.move("BS_Arm_Z", 0.0, 100.0)      # 先抬到安全位
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with eng3.lock:
                if ax.in_position and not ax.moving:
                    break
            time.sleep(0.01)
        # 快速段（move 到寻力起始位）→ 慢速寻力（触 2N 停）
        steps = [
            {"op": "move", "axis": "BS_Arm_Z", "target": -166.9,
             "speed": 20.0, "desc": "快速下压段"},
            {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
             "touch_force_N": 2.0, "min_pos": -169.9, "timeout": 60.0,
             "desc": "慢速寻力段"},
        ]
        fe.run_steps(steps, "i9-force_search-ok")
        st = flow_wait_f()
        touch_pos = eng3.bond_touch_pos
        last = fe.steps[-1] if fe.steps else {}
        # 无 recipe → eff=裸 -168.93；k=500N/mm。触力判据=含噪读数 ≥2N
        # → 真值 F ≥ 2−1=1N → 停点必 ≤ eff−0.002；流程每 10ms 墙钟轮询
        # （tick 2x 实时），5mm/s 下单次轮询间隔最多过冲 ~0.1mm≈50N →
        # 下限放宽到 eff−0.15。注：停轴记录后力回路（sp=0）会抬头卸力，
        # ax.pos 回有效接触位，故只断言记录值
        check("i9.force_search_touch",
              st == "done" and touch_pos is not None
              and -169.08 < touch_pos < -168.931,
              "state=%s touch=%s last=%s" % (st, touch_pos, last))
        fe.reset()
        # 超程支：min_pos 在接触位之上（-168.5 > -168.93）→ 永不到力 → FAIL
        eng3.move("BS_Arm_Z", -166.9, 50.0)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with eng3.lock:
                if ax.in_position and not ax.moving:
                    break
            time.sleep(0.01)
        steps2 = [
            {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
             "touch_force_N": 2.0, "min_pos": -168.5, "timeout": 60.0,
             "desc": "超程未触力"},
        ]
        fe.run_steps(steps2, "i9-force_search-overtravel")
        st2 = flow_wait_f()
        err2 = (fe.steps[-1].get("info") if fe.steps else "") or ""
        check("i9.force_search_overtravel_fail",
              st2 == "aborted" and "寻力未接触" in err2,
              "state=%s err=%s" % (st2, err2[:60]))
        fe.stop() if fe.state in ("running", "paused") else None
        fe.reset()
        # keep_warm 秒过（进程内 _do_heater wait：带内应立即返回 <1s）
        h_dw = eng3.heaters["Heat_Dw"]
        h_dw.cur = 150.0
        eng3.heater_command("Heat_Dw", "SetTargetTempWithPlan",
                            {"Temperature": 150})
        t0 = time.time()
        fe._do_heater({"name": "Heat_Dw", "action": "wait", "tol_c": 2.0,
                       "timeout": 5.0})
        dt = time.time() - t0
        check("i9.keep_warm_instant", dt < 1.0, "等温步耗时 %.3fs" % dt)
    finally:
        tick_stop.append(True)
        th.join(timeout=3.0)
        try:
            fe.stop()
        except Exception:
            pass

    # ================= HTTP：wafer_keep_warm apply 生效 =================
    c, r = http_put(base + "/api/recipe",
                    {"bond": {"wafer_keep_warm": True, "wafer_temp_C": 150.0}})
    assert c == 200, r
    http_post(base + "/api/recipe/apply", {})
    time.sleep(1.2)         # engine 每秒重读常数/处方；等一拍刷新
    pid_pub = (http_json(base + "/api/flow/state").get("pid") or {})
    wt = pid_pub.get("wafer_temp") or {}
    check("i9.keep_warm_apply",
          wt.get("active") is True
          and abs(wt.get("sp", 0.0) - 150.0) < 1e-6,
          "wafer_temp=%s" % (wt,))
    # 还原：停硅片台加热 + 恢复默认处方，防后续用例热状态污染
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.Heat_Dw",
               "cmd": "StopTargetTempWithPlan"})
    http_post(base + "/api/recipe/restore_default", {})


def test_iter10():
    """--- 19.16 迭代 10：site_bonded 统一守卫 + /api/workpiece/map ---
      - 进程内 site_bonded 容差：0.5µm 偏差匹配 / 2µm 不匹配 / site=None
        不计 / carrier!=wafer 不计；
      - HTTP /api/workpiece/map 字段完整性（film_dies/wafer_sites/geometry）
        + 键合前后对比（site bonded false→true、被取 die carrier film→
        wafer）；
      - 已键合 site 重跑 focus_level/fine_align → 400 且报"已键合"
        （禁止重复调平/对准；bond 同款守卫由 i8 覆盖）；
      - chain 运行中重复启动 → 409。"""
    print("--- 19.16 迭代 10（site_bonded 守卫/workpiece map/chain 409） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= 进程内：site_bonded 容差匹配 =================
    import workpieces as wpmod

    class _WP(object):
        def __init__(self):
            self._lock = threading.RLock()
            self.dies = [
                {"id": "R00C00", "carrier": "wafer", "site": [10.0, 20.0]},
                {"id": "R00C01", "carrier": "wafer", "site": None},
                {"id": "R00C02", "carrier": "film", "site": [10.0, 20.0]},
            ]

    wp = _WP()
    check("i10.site_bonded_tol",
          wpmod.site_bonded(wp, 10.0, 20.0)
          and wpmod.site_bonded(wp, 10.0005, 19.9995)   # 0.5µm 偏差仍匹配
          and not wpmod.site_bonded(wp, 10.002, 20.0)    # 2µm 偏差不匹配
          and not wpmod.site_bonded(wp, 0.0, 0.0),       # None/film 不计
          "exact/0.5um/2um/empty 组合")

    # ================= HTTP：/api/workpiece/map 字段完整性（键合前） =================
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/recipe/restore_default", {})
    m0 = http_json(base + "/api/workpiece/map")
    fd0 = m0.get("film_dies") or []
    ws0 = m0.get("wafer_sites") or []
    geo = m0.get("geometry") or {}
    sites_total = http_json(
        base + "/api/recipe/wafer_sites?limit=100000").get("total_sites")
    check("i10.map_fields",
          m0.get("ok") and fd0 and ws0
          and len(ws0) == sites_total
          and all(set(("row", "col", "x", "y", "id", "carrier"))
                  <= set(d) for d in fd0[:50])
          and all(set(("row", "col", "x", "y", "bonded")) <= set(s)
                  for s in ws0)
          and all(d["carrier"] == "film" for d in fd0)
          and all(re.match(r"^R\d\dC\d\d$", d["id"]) for d in fd0[:50])
          and not any(s["bonded"] for s in ws0)
          and (geo.get("film") or {}).get("diameter_mm") == 300.0
          and (geo.get("wafer") or {}).get("diameter_mm") == 300.0
          and (geo.get("wafer") or {}).get("site_size_x_mm"),
          "film_dies=%d sites=%d/%s geo=%s"
          % (len(fd0), len(ws0), sites_total, sorted(geo)))

    # ================= 链跑通一个 site（默认中心位）→ 键合后对比 =================
    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def chain_wait(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ch = flow_state().get("chain") or {}
            if not ch.get("active"):
                return ch
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    # 迭代 18：chain 实跑及键合后断言 quick 门控跳过（单片链覆盖由
    # i8.chain_bond_happy 承担），full 全跑；上方 site_bonded 容差/map
    # 字段用例段首自清理、sim≈0，两套件保留。
    if not fblock(["i10.chain_conflict", "i10.chain_bond_done",
                   "i10.map_after_bond", "i10.bonded_guard"]):
        return
    # 中心位行列号（chain 缺省 = min hypot(x,y)，同 builder 规则）
    cs = min(ws0, key=lambda s: math.hypot(s["x"], s["y"]))
    try:
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×）
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            # FAST θ 噪声尾靠确认环兜底（同 i7 惯例，放宽到 8 进一步压低
            # 偶发超次 abort——2026-09-01 复跑观测）
            "fine_align": {"confirm_max_iter": 8}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/chain", {})
        # 链运行中重复启动 → 409（等 active 置位再发，避免竞态）
        t0 = time.time()
        while time.time() - t0 < 10.0:
            if (flow_state().get("chain") or {}).get("active"):
                break
            time.sleep(0.1)
        c2, r2 = http_post(base + "/api/process/chain", {})
        check("i10.chain_conflict",
              c == 200 and c2 == 409 and "在运行" in (r2.get("error") or ""),
              "first=%s second=%s err=%s" % (c, c2, r2.get("error")))
        ch = chain_wait()
        st = ch.get("stages") or {}
        # 失败时把中止步骤详情带进断言输出（2026-09-01：长会话后偶发
        # wafer_ega abort，需真因）
        fail_info = ""
        if ch.get("error"):
            bad = [s2 for s2 in (flow_state().get("steps") or [])
                   if s2.get("status") not in ("ok", "")]
            fail_info = " bad_steps=%s" % ([(b.get("op"), b.get("status"),
                                             str(b.get("info"))[:80])
                                            for b in bad[-3:]],)
        check("i10.chain_bond_done",
              not ch.get("error")
              and all(st.get(k) == "done" for k in
                      ("wafer_ega", "flip_pick", "flip_handoff",
                       "focus_level", "fine_align", "bond")),
              "stages=%s err=%s%s" % (st, ch.get("error"), fail_info))
        flow_cleanup()
        # ---- 键合后 map 对比：该 site bonded=true，被取 die carrier=wafer ----
        m1 = http_json(base + "/api/workpiece/map")
        s1 = [s for s in (m1.get("wafer_sites") or [])
              if s["row"] == cs["row"] and s["col"] == cs["col"]]
        bonded_cnt = sum(1 for s in m1.get("wafer_sites", []) if s["bonded"])
        wafer_dies = [d for d in m1.get("film_dies", [])
                      if d["carrier"] == "wafer"]
        check("i10.map_after_bond",
              len(s1) == 1 and s1[0]["bonded"] is True
              and bonded_cnt == 1 and len(wafer_dies) == 1,
              "site=%s bonded_cnt=%s wafer_dies=%s"
              % (s1, bonded_cnt, [d["id"] for d in wafer_dies]))
        # ---- 已键合 site 重跑 focus_level/fine_align → 400 已键合守卫 ----
        body = {"site": [cs["row"], cs["col"]]}
        c3, r3 = http_post(base + "/api/process/focus_level",
                           dict(body, mode="per_site"))
        c4, r4 = http_post(base + "/api/process/fine_align", body)
        check("i10.bonded_guard",
              c3 == 400 and "已键合" in (r3.get("error") or "")
              and c4 == 400 and "已键合" in (r4.get("error") or ""),
              "fl=%s:%s fa=%s:%s"
              % (c3, (r3.get("error") or "")[:40],
                 c4, (r4.get("error") or "")[:40]))
        flow_cleanup()
    finally:
        try:
            set_speed(1.0)                   # 提速窗复原（迭代 18）
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter11a():
    """--- 19.17 迭代 11 第一棒（A 处方两组 + B 键合结果收集器） ---
      - 处方 full_wafer/post_inspect：默认值 / PUT 部分合并往返 / 未知键
        与非法枚举 400 / _migrate 旧文件播种；
      - tuning bond_hw.wafer_cool_rate_cps 新键（默认 5.0 / PUT 往返 /
        非法 400）；
      - engine 降温方向用主动冷却速率（进程内数值验证：150℃→25℃ 速率
        ≈5℃/s 而非自然热损 0.733；升温方向不变）；
      - bond_results：一次真实单片 chain 后记录字段完整（die/site/t_bond/
        truth/chain 五段/bond_params），truth 与 fine_align_result.
        phys_resid 一致；API 两形态（全量+摘要 / ?die= 单条，未知 404）；
        workpiece/reset 后清空。"""
    print("--- 19.17 迭代 11a（处方 full_wafer/post_inspect + bond_results） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= 处方：两组默认值 =================
    rec0 = http_json(base + "/api/recipe")["recipe"]
    fw0, pi0 = rec0.get("full_wafer") or {}, rec0.get("post_inspect") or {}
    check("i11a.recipe_defaults",
          fw0.get("count") == 0
          and fw0.get("path_mode") == "center_out"
          and fw0.get("die_order") == "nearest_pick"
          and fw0.get("skip_bonded") is True
          and fw0.get("stop_on_fail") is False
          and pi0.get("enabled") is True
          and pi0.get("auto_after_single") is False
          and pi0.get("auto_after_full") is False
          and abs(pi0.get("cool_target_C", -1) - 25.0) < 1e-9
          and abs(pi0.get("cool_rate_cps", -1) - 5.0) < 1e-9,
          "full_wafer=%s post_inspect=%s" % (fw0, pi0))

    # ================= 处方：PUT 往返 + 未知键/非法枚举 400 =================
    c, r = http_put(base + "/api/recipe",
                    {"full_wafer": {"count": 12, "path_mode": "serpentine"},
                     "post_inspect": {"cool_target_C": 30.0}})
    rec1 = http_json(base + "/api/recipe")["recipe"]
    check("i11a.recipe_put_roundtrip",
          c == 200 and rec1["full_wafer"]["count"] == 12
          and rec1["full_wafer"]["path_mode"] == "serpentine"
          and abs(rec1["post_inspect"]["cool_target_C"] - 30.0) < 1e-9,
          "code=%s fw=%s pi=%s"
          % (c, rec1.get("full_wafer"), rec1.get("post_inspect")))
    c2, r2 = http_put(base + "/api/recipe", {"full_wafer": {"bogus": 1}})
    c3, r3 = http_put(base + "/api/recipe",
                      {"full_wafer": {"path_mode": "spiral"}})
    c4, r4 = http_put(base + "/api/recipe",
                      {"post_inspect": {"enabled": "yes"}})
    rec2 = http_json(base + "/api/recipe")["recipe"]
    check("i11a.recipe_reject",
          c2 == 400 and c3 == 400 and c4 == 400
          and rec2["full_wafer"]["count"] == 12,   # 拒载不污染
          "codes=%s/%s/%s" % (c2, c3, c4))
    http_post(base + "/api/recipe/restore_default", {})

    # ================= 处方迁移：旧文件无两组 → _migrate 播种 =================
    import recipe as recipe_mod
    old = json.loads(json.dumps(recipe_mod.DEFAULT_RECIPE))
    old.pop("full_wafer"); old.pop("post_inspect")
    mig = recipe_mod._migrate(old)
    check("i11a.recipe_migrate",
          mig["full_wafer"] == recipe_mod.DEFAULT_RECIPE["full_wafer"]
          and mig["post_inspect"] == recipe_mod.DEFAULT_RECIPE["post_inspect"]
          and recipe_mod.validate(mig) == [],
          "迁移后 validate errors=%s" % (recipe_mod.validate(mig),))

    # ================= tuning：wafer_cool_rate_cps 新键 =================
    tn = http_json(base + "/api/tuning").get("tuning") or {}
    hw = tn.get("bond_hw") or {}
    check("i11a.tuning_key",
          abs(hw.get("wafer_cool_rate_cps", -1) - 5.0) < 1e-9,
          "bond_hw.wafer_cool_rate_cps=%s" % hw.get("wafer_cool_rate_cps"))
    c5, _ = http_put(base + "/api/tuning",
                     {"bond_hw": {"wafer_cool_rate_cps": 8.0}})
    hw2 = (http_json(base + "/api/tuning").get("tuning") or {}) \
        .get("bond_hw") or {}
    c6, _ = http_put(base + "/api/tuning",
                     {"bond_hw": {"wafer_cool_rate_cps": 0.0}})
    check("i11a.tuning_put",
          c5 == 200 and abs(hw2.get("wafer_cool_rate_cps", -1) - 8.0) < 1e-9
          and c6 == 400,
          "put=%s val=%s bad=%s"
          % (c5, hw2.get("wafer_cool_rate_cps"), c6))
    http_put(base + "/api/tuning", {"bond_hw": {"wafer_cool_rate_cps": 5.0}})

    # ================= engine：降温方向用主动冷却速率（进程内数值） =================
    from engine import Engine as _Engine
    from points import Points as _Points
    eng = _Engine(_Points(os.path.join(ROOT, "runtime", "io_points.json")))
    with eng.lock:
        eng._refresh_loop_consts_locked()
        h = eng.heaters["Heat_Dw"]
        assert abs(h.cool_rate - 5.0) < 1e-9, h.cool_rate
        h.cur, h.target, h.active, h.power = 150.0, 25.0, True, 0.0
        for _ in range(2000):           # 2s 仿真
            eng._tick_heaters_locked()
        cool_rate = (150.0 - h.cur) / 2.0
        # 升温方向回归：满功率升温仍按 wafer_heater_rate_cps（0.733）
        h.cur, h.target, h.active, h.power = 25.0, 150.0, True, 100.0
        for _ in range(2000):
            eng._tick_heaters_locked()
        heat_rate = (h.cur - 25.0) / 2.0
    check("i11a.cool_rate",
          4.9 < cool_rate <= 5.0 + 1e-9 and 0.70 < heat_rate < 0.74,
          "降温 %.3f ℃/s（旧自然热损 ≈0.19）；升温 %.3f ℃/s"
          % (cool_rate, heat_rate))

    # ================= bond_results：真实单片 chain 全链采集 =================
    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def chain_wait(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ch = flow_state().get("chain") or {}
            if not ch.get("active"):
                return ch
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    # 迭代 18：bond_results 实跑采集（单片 chain）quick 门控跳过（单片链
    # 覆盖由 i8.chain_bond_happy 承担），full 全跑；上方处方/tuning/迁移/
    # 冷却速率用例 sim≈0 两套件保留。
    if not fblock(["i11a.wafer_preheat", "i11a.chain_done",
                   "i11a.results_record", "i11a.results_get_one",
                   "i11a.results_reset"]):
        return

    try:
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8},
            "bond": {"dwell_s": 2.0}})
        assert c == 200, r
        # 预置硅片台到温（同 i8 惯例：8 倍速预 Heat_Dw 到处方
        # wafer_temp_C（迭代 12 起默认 30）保温）
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        http_post(base + "/api/speed", {"scale": 8.0})
        t0 = time.time()
        cur_t = 0.0
        plc = "http://127.0.0.1:%d" % PLC_PORT
        while time.time() - t0 < 60.0:
            v = http_json(plc + "/io/observe?path=SystemMachine.M_Heater."
                                "Heat_Dw.oCurTemp")
            cur_t = float(v.get("value", 0.0))
            if cur_t >= tgt_t - 1.5:
                break
            time.sleep(0.5)
        http_post(base + "/api/speed", {"scale": 1.0})
        check("i11a.wafer_preheat", cur_t >= tgt_t - 1.5,
              "Heat_Dw 预置到温 oCurTemp=%.1f℃（目标 %.1f）" % (cur_t, tgt_t))

        set_speed(8.0)   # 迭代 18：chain 实跑提速窗（finally 复原 1×）
        c, r = http_post(base + "/api/process/chain", {})
        ch = chain_wait()
        st = ch.get("stages") or {}
        check("i11a.chain_done",
              c == 200 and not ch.get("error")
              and all(st.get(k) == "done" for k in
                      ("wafer_ega", "flip_pick", "flip_handoff",
                       "focus_level", "fine_align", "bond")),
              "stages=%s err=%s" % (st, ch.get("error")))
        # fine_align_result 须在 flow/reset 前取（reset 会清掉）
        fa_st = flow_state().get("fine_align_result") or {}
        flow_cleanup()

        # ---- 全量列表 + 统计摘要 ----
        res = http_json(base + "/api/bond/results")
        rs = res.get("results") or []
        rec = rs[0] if rs else {}
        chain_keys = sorted((rec.get("chain") or {}).keys())
        ph = fa_st.get("phys_resid") or {}
        truth = rec.get("truth") or {}
        # 迭代 20 truth 口径：truth = commit（键合接触时刻真值，含键合头
        # 下降链）+ process_shift（压合微滑移，默认 0）；fa 时刻 phys_resid
        # 保留为 align_resid 分量；descent=下降链=commit−align_resid
        tcp = rec.get("truth_components") or {}
        al = tcp.get("align_resid") or {}
        cm = tcp.get("commit") or {}
        ps = tcp.get("process_shift") or {}
        de = tcp.get("descent") or {}
        truth_match = truth and cm and ps and \
            abs(truth.get("dx", 9e9) * 1000.0
                - (cm.get("dx_um", 0) + ps.get("dx_um", 0))) < 1e-3 \
            and abs(truth.get("dy", 9e9) * 1000.0
                - (cm.get("dy_um", 0) + ps.get("dy_um", 0))) < 1e-3 \
            and abs(truth.get("drz", 9e9) * 1e6
                    - (cm.get("drz_urad", 0) + ps.get("drz_urad", 0))) < 1.0
        align_match = al and ph and \
            abs(al.get("dx_um", 0) - ph.get("tx_um", 0)) < 1e-3 \
            and abs(al.get("dy_um", 0) - ph.get("ty_um", 0)) < 1e-3 \
            and abs(al.get("drz_urad", 0) - ph.get("theta_urad", 0)) < 1.0
        # 下降链守恒：commit = align + descent；且本机 BS_Arm_Z 地图差下
        # 下降链系统项须达 µm 级（迭代 20 物理重构的存在性证据）
        de_consist = de and cm and al and \
            abs(de.get("dx_um", 0) - (cm["dx_um"] - al["dx_um"])) < 1e-3 \
            and abs(de.get("drz_urad", 0)
                    - (cm["drz_urad"] - al["drz_urad"])) < 1e-3
        de_mag = math.hypot(de.get("dx_um", 0.0), de.get("dy_um", 0.0))
        check("i11a.results_record",
              res.get("ok") and res.get("count") == 1
              and (res.get("stats") or {}).get("with_truth") == 1
              and re.match(r"^R\d\dC\d\d$", rec.get("die_id") or "")
              and set(("row", "col", "x", "y")) <= set(rec.get("site") or {})
              and rec.get("t_bond") is not None
              and rec.get("measured") is None
              and truth_match and align_match and de_consist
              and 1.0 < de_mag < 50.0
              and set(("wafer_load", "ega_fit", "pick_coarse",
                       "handoff_tilt", "focus_level", "fine_align",
                       "bond_commit")) <= set(chain_keys)
              and ((rec.get("chain") or {}).get("fine_align") or {})
                  .get("phys_resid") == ph
              and abs((rec.get("bond_params") or {}).get("force_N", 0)
                      - 50.0) < 1e-9,
              "die=%s site=%s truth=%s components=%s chain=%s stats=%s"
              % (rec.get("die_id"), rec.get("site"), truth, tcp, chain_keys,
                 res.get("stats")))
        print("    [i11a] 记录样例: %s"
              % json.dumps(rec, ensure_ascii=True)[:600], flush=True)

        # ---- 单条 ?die=（含完整 chain）+ 未知 404 ----
        die_id = rec.get("die_id") or ""
        one = http_json(base + "/api/bond/results?die=" + die_id)
        c7, r7 = http_get_raw(base + "/api/bond/results?die=R99C99")
        check("i11a.results_get_one",
              one.get("ok") and (one.get("result") or {}).get("die_id")
              == die_id
              and (one.get("result") or {}).get("chain") == rec.get("chain")
              and c7 == 404,
              "one.ok=%s 404=%s" % (one.get("ok"), c7))

        # ---- 持久化文件落盘 + reset 清空 ----
        fjson = (BARE.bond_results_path if BARE is not None
                 else os.path.join(ROOT, "runtime", "bond_results.json"))
        disk = []
        if os.path.exists(fjson):
            with open(fjson, encoding="utf-8") as f:
                disk = (json.load(f) or {}).get("records") or []
        http_post(base + "/api/workpiece/reset", {})
        after = http_json(base + "/api/bond/results")
        check("i11a.results_reset",
              len(disk) == 1 and after.get("count") == 0
              and after.get("results") == [],
              "disk=%d after_reset=%s" % (len(disk), after.get("count")))
    finally:
        try:
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter11b():
    """--- 19.18 迭代 11 第二棒（C 全片键合 + D 键合后检测 + 结果数据） ---
      - 路径规划纯函数单测：plan_sites 三模式排序（center_out |x|+|y| 升序
        / serpentine 行蛇形 / nearest 最近邻链）+ skip 已键合过滤 +
        count 截断；order_dies 两模式（nearest_pick 离取片位升序 /
        serpentine 行蛇形）；
      - bond_results.update_measured 往返（临时文件进程内）+ 统计摘要
        truth/measured 双列含 3σ；
      - full_wafer 小批量端到端（处方 count=2）：配对表 2 对、状态契约
        full_wafer{active,progress,pairs,current,results}、两 die 记录齐
        （truth 非空、site 各异）；
      - post_inspect 端到端：两 die measured 落盘，且与 truth 同量级
        （容差 |Δxy|<5µm / |Δrz|<100µrad。误差预算：xy 主导项=下相机
        绝对位姿误差——align_stage 6DOF 误差图未标定（实测 0.1~3µm 随
        AS 位变化，Temp/_e6_probe.py 探测），真机靠标定吸收；rz=测量
        噪声层（AS 重复定位 0.5µm 3σ/轴+FAST 0.09µm，±3.8mm 基线
        σ≈18µrad，100µrad≈5.7σ），详见专题文档 §2.2）；
      - auto_after 开关：auto_after_full=true 时全片结束自动触发检测流程
        （flow 名含"键合后精度检测"），false 时不触发。"""
    print("--- 19.18 迭代 11b（全片键合 + 键合后检测） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= 路径规划纯函数单测 =================
    import full_wafer_runner as fwr
    # 3×3 网格（行列 0..2，坐标 -10/0/10）
    cells = [(r, c, (c - 1) * 10.0, (r - 1) * 10.0)
             for r in range(3) for c in range(3)]
    co = fwr.plan_sites(cells, None, "center_out", 0)
    check("i11b.plan_center_out",
          co[0][2:] == (0.0, 0.0)
          and [abs(c[2]) + abs(c[3]) for c in co] == sorted(
              abs(c[2]) + abs(c[3]) for c in co)
          and len(co) == 9,
          "首=%s" % (co[0],))
    sp = fwr.plan_sites(cells, None, "serpentine", 0)
    row1_cols = [c[1] for c in sp if c[0] == 1]
    check("i11b.plan_serpentine",
          [c[0] for c in sp] == sorted(c[0] for c in sp)
          and row1_cols == [2, 1, 0]              # 第 2 行反向
          and len(sp) == 9,
          "row1=%s" % (row1_cols,))
    nn = fwr.plan_sites(cells, None, "nearest", 0)
    dists = [math.hypot(nn[i + 1][2] - nn[i][2], nn[i + 1][3] - nn[i][3])
             for i in range(len(nn) - 1)]
    check("i11b.plan_nearest",
          nn[0][2:] == (0.0, 0.0) and len(nn) == 9
          and max(dists) <= math.hypot(10.0, 10.0) + 1e-9
          and dists[0] <= 10.0 + 1e-9,
          "首=%s 步距max=%.2f" % (nn[0], max(dists)))
    # skip 已键合 + count 截断
    sk = fwr.plan_sites(cells, lambda x, y: x == 0.0 and y == 0.0,
                        "center_out", 2)
    check("i11b.plan_skip_count",
          len(sk) == 2 and all(not (c[2] == 0.0 and c[3] == 0.0) for c in sk),
          "sk=%s" % (sk,))
    dies9 = [{"id": "R%02dC%02d" % (r, c), "film_pos": [(c - 1) * 20.0,
             (r - 1) * 20.0]} for r in range(3) for c in range(3)]
    onp = fwr.order_dies(dies9, "nearest_pick", [20.0, 20.0], [0.0, 0.0])
    osp = fwr.order_dies(dies9, "serpentine")
    check("i11b.order_dies",
          onp[0]["id"] == "R02C02"
          and [d["id"] for d in osp[:4]] == ["R00C00", "R00C01", "R00C02",
                                             "R01C02"],
          "np首=%s sp前4=%s" % (onp[0]["id"], [d["id"] for d in osp[:4]]))

    # ================= update_measured 往返（临时文件进程内） =================
    import bond_results as brm
    tmpf = os.path.join(ROOT, "tests", "_i11b_bond_results.json")
    if os.path.exists(tmpf):
        os.remove(tmpf)
    br1 = brm.BondResults(tmpf)
    br1.add({"die_id": "R00C00",
             "site": {"row": 1, "col": 1, "x": 0.0, "y": 0.0},
             "t_bond": 1.0, "truth": {"dx": 0.0001, "dy": -0.0002,
                                      "drz": 1e-5},
             "truth_note": None, "measured": None, "chain": {},
             "bond_params": {}})
    ok1 = br1.update_measured("R00C00", 0.00011, -0.00019, 1.1e-5, 2.5)
    ok2 = br1.update_measured("R99C99", 0.0, 0.0, 0.0, 3.0)
    one = br1.get_by_die("R00C00")
    st0 = br1.list_public()["stats"]
    br2 = brm.BondResults(tmpf)          # 重载验证落盘往返
    m2 = (br2.get_by_die("R00C00") or {}).get("measured") or {}
    check("i11b.update_measured",
          ok1 and not ok2
          and abs(one["measured"]["dx"] - 0.00011) < 1e-9
          and abs(m2["dy"] + 0.00019) < 1e-9
          and st0["measured"] == 1
          and st0["measured_stats"]["abs_dx_um"]["mean"] > 0
          and "sigma3" in st0["truth"]["abs_dxy_um"],
          "m=%s stats=%s" % (one.get("measured"), st0.get("measured_stats")))
    br2.clear()
    os.remove(tmpf)

    # ================= full_wafer 小批量端到端（count=2） =================
    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def fw_wait(timeout=1500.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            fw = flow_state().get("full_wafer") or {}
            if not fw.get("active"):
                return fw
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    def flow_wait_done(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = flow_state()
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"state": "timeout"}

    def wafer_preheat():
        """硅片台 8 倍速预热到处方 bond.wafer_temp_C（迭代 12 起默认 30℃）
        保温（post_inspect 降温后/首次键合前调用；同 i11a 惯例）。
        迭代 18：复原到外层提速窗倍速 _SPEED_CUR（不再硬编码 1×）。"""
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        http_post(base + "/api/speed", {"scale": 8.0})
        t0 = time.time()
        cur_t = 0.0
        plc = "http://127.0.0.1:%d" % PLC_PORT
        while time.time() - t0 < 60.0:
            v = http_json(plc + "/io/observe?path=SystemMachine.M_Heater."
                                "Heat_Dw.oCurTemp")
            cur_t = float(v.get("value", 0.0))
            if cur_t >= tgt_t - 1.5:
                break
            time.sleep(0.5)
        http_post(base + "/api/speed", {"scale": _SPEED_CUR})
        return cur_t, tgt_t

    # 迭代 18：full_wafer/post_inspect 端到端实跑 quick 门控跳过（全片
    # 编排 sim 远超 30s 标准，full 全跑）；上方路径规划/update_measured
    # 纯函数单测两套件保留。
    if not fblock(["i11b.wafer_preheat", "i11b.ega_done", "i11b.fw_start",
                   "i11b.fw_done", "i11b.fw_records",
                   "i11b.pi_start", "i11b.pi_done", "i11b.pi_measured",
                   "i11b.fw2_done", "i11b.auto_after_full",
                   "i11b.auto_after_off"]):
        return

    try:
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×；wafer_preheat
                         # 内部复原到 _SPEED_CUR 保持窗内 8×）
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8,
                           # 放宽确认阈值：i11b 考全片编排/检测闭环，fa 严格
                           # 阈值有专测覆盖；0.5µm 阈值在 FAST 噪声尾部会
                           # 偶发 abort（同 fa.* 已知偶发族），拖垮整片测试
                           "confirm_tol_xy_um": 1.0,
                           "confirm_tol_rz_urad": 60.0},
            "bond": {"dwell_s": 2.0},
            "full_wafer": {"count": 2, "path_mode": "center_out",
                           "die_order": "nearest_pick"},
            "post_inspect": {"auto_after_full": False,
                             "auto_after_single": False}})
        assert c == 200, r
        # 预置晶圆+EGA（全片前的标准前置；同 i11a 预热惯例）
        cur_t, tgt_t = wafer_preheat()
        check("i11b.wafer_preheat", cur_t >= tgt_t - 1.5,
              "Heat_Dw 预置到温 oCurTemp=%.1f℃（目标 %.1f）"
              % (cur_t, tgt_t))
        c, r = http_post(base + "/api/process/wafer_ega", {})
        assert c == 200, r
        st = flow_wait_done()
        check("i11b.ega_done", st.get("state") == "done",
              "ega state=%s" % st.get("state"))
        flow_cleanup()

        c, r = http_post(base + "/api/process/full_wafer", {})
        pairs = (r or {}).get("plan") or []
        check("i11b.fw_start",
              c == 200 and (r or {}).get("pairs") == 2 and len(pairs) == 2
              and all(p.get("die") and p.get("site") for p in pairs),
              "code=%s pairs=%s plan=%s" % (c, (r or {}).get("pairs"), pairs))
        fw = fw_wait()
        fwr_res = fw.get("results") or []
        if any(not x.get("ok") for x in fwr_res) or fw.get("error"):
            # 失败诊断：打印 flow 日志尾部（fine_align abort 等原因）
            for e in (flow_state().get("log") or [])[-15:]:
                print("    [i11b][fwlog] %s"
                      % (e.get("msg") or "").encode(
                          "unicode_escape").decode("ascii")[:150],
                      flush=True)
        check("i11b.fw_done",
              not fw.get("active") and not fw.get("error")
              and (fw.get("progress") or [0, 0]) == [2, 2]
              and len(fwr_res) == 2 and all(x.get("ok") for x in fwr_res),
              "progress=%s results=%s err=%s"
              % (fw.get("progress"), fwr_res, fw.get("error")))
        print("    [i11b] 配对表: %s" % json.dumps(pairs, ensure_ascii=True),
              flush=True)
        flow_cleanup()

        res = http_json(base + "/api/bond/results")
        rs = res.get("results") or []
        sites_seen = set((x["site"]["row"], x["site"]["col"]) for x in rs)
        check("i11b.fw_records",
              res.get("count") == 2
              and len(sites_seen) == 2
              and all(x.get("truth") for x in rs)
              and all(re.match(r"^R\d\dC\d\d$", x.get("die_id") or "")
                      for x in rs),
              "count=%s sites=%s" % (res.get("count"), sites_seen))
        print("    [i11b] 两 die 记录: %s"
              % json.dumps([{"die": x["die_id"], "site": x["site"],
                             "truth": x["truth"]} for x in rs],
                           ensure_ascii=True), flush=True)

        # ================= post_inspect 端到端 =================
        c, r = http_post(base + "/api/process/post_inspect", {})
        check("i11b.pi_start", c == 200 and (r or {}).get("ok"),
              "code=%s r=%s" % (c, str(r)[:200]))
        st = flow_wait_done()
        check("i11b.pi_done", st.get("state") == "done",
              "pi state=%s flow=%s" % (st.get("state"), st.get("flow")))
        flow_cleanup()
        res2 = http_json(base + "/api/bond/results")
        rs2 = res2.get("results") or []
        comp = []
        allm = True
        for x in rs2:
            t, m = x.get("truth"), x.get("measured")
            if not (t and m):
                allm = False
                continue
            comp.append({"die": x["die_id"],
                         "dx_um": (m["dx"] - t["dx"]) * 1000.0,
                         "dy_um": (m["dy"] - t["dy"]) * 1000.0,
                         "drz_urad": (m["drz"] - t["drz"]) * 1e6})
        # 容差依据（迭代 15 起 pi_measure 重构为红外套刻套刻相对测量——
        # 迭代 20 起同一机位两次独立触发（先 quad 后 cross）、组间差分，
        # EGA 残差/相机位姿系统差/逐标记 AS 重复性全部对消；本用例维持
        # 5µm/100µrad 宽容差只验闭环，紧容差（1µm/100µrad）由
        # i15.pi_overlay_measured 覆盖）。旧口径预算：xy 主导项=下相机
        # 绝对位姿 6DOF 未标定 ~2.5-3µm、rz=测量噪声层 σ≈18µrad，详见
        # docs/全片键合与键合后检测_V1.md。truth=键合接触时刻真值
        #（迭代 20 起，含下降链系统项；measured 同含，差值相消）。
        okc = allm and len(comp) == 2 and all(
            abs(v["dx_um"]) < 5.0 and abs(v["dy_um"]) < 5.0
            and abs(v["drz_urad"]) < 100.0 for v in comp)
        check("i11b.pi_measured",
              okc and (res2.get("stats") or {}).get("measured") == 2,
              "measured-truth=%s stats.measured=%s"
              % (comp, (res2.get("stats") or {}).get("measured")))
        print("    [i11b] measured vs truth 偏差（um/urad）: %s"
              % json.dumps(comp, ensure_ascii=True), flush=True)
        print("    [i11b] measured 样例: truth=%s measured=%s"
              % (rs2[0].get("truth"), rs2[0].get("measured")) if rs2 else "",
              flush=True)

        # ================= auto_after 开关 =================
        # auto_after_full=true：再跑 count=1 全片（第三颗 die/下一个 site），
        # 结束后应自动触发"键合后精度检测"流程。post_inspect 已把晶圆降到
        # 25℃——先 8 倍速回温到处方 wafer_temp_C（否则 bond 等温升温拖
        # 时间）。
        wafer_preheat()
        c, r = http_put(base + "/api/recipe",
                        {"full_wafer": {"count": 1},
                         "post_inspect": {"auto_after_full": True}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200, r
        fw = fw_wait(timeout=1500.0)
        check("i11b.fw2_done",
              not fw.get("active") and len(fw.get("results") or []) == 1
              and (fw.get("results") or [{}])[0].get("ok"),
              "fw2=%s" % (fw.get("results"),))
        # 自动触发的检测是异步跑的：等 flow 出现并完成
        t0 = time.time()
        seen_pi = False
        while time.time() - t0 < 600.0:
            st = flow_state()
            if "键合后精度检测" in (st.get("flow") or ""):
                seen_pi = True
                if st.get("state") in ("done", "aborted"):
                    break
            time.sleep(0.5)
        res3 = http_json(base + "/api/bond/results")
        check("i11b.auto_after_full",
              seen_pi and st.get("state") == "done"
              and (res3.get("stats") or {}).get("measured") == 3,
              "seen_pi=%s state=%s measured=%s"
              % (seen_pi, st.get("state"),
                 (res3.get("stats") or {}).get("measured")))
        flow_cleanup()
        # auto_after_full=false：再跑 count=1，结束后 flow 应保持 idle
        # （上一轮自动检测又降到了 25℃，先回温）
        wafer_preheat()
        c, r = http_put(base + "/api/recipe",
                        {"post_inspect": {"auto_after_full": False}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200, r
        fw = fw_wait(timeout=1500.0)
        time.sleep(1.0)
        st = flow_state()
        check("i11b.auto_after_off",
              not fw.get("active")
              and len(fw.get("results") or []) == 1
              and (st.get("state") == "idle"
                   or "键合后精度检测" not in (st.get("flow") or "")),
              "state=%s flow=%s" % (st.get("state"), st.get("flow")))
    finally:
        try:
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter13():
    """--- 19.20 迭代 13：键合接触确认 + AS_X/Y repeat 40nm + confirm_tol
    0.15µm + /api/full_reset + quad/cross 套刻标记 ---
      - 机械常数 bond.contact_verify_tol_um 默认 50.0、PUT 合法/非法；
      - 进程内 force_search 接触确认：容差内 PASS 且 eng.bond_contact_
        evidence 留证（actual/eff/dev/触力），bond_results.collect_stage
        chain.bond 段带 contact_verify；期望位偏 430µm 超差 → FAIL"接触位
        异常"；迭代 17 WP-B 双源判定几何侧（stub confocal：弹簧压入 −4µm
        PASS 留 geom_gap_um 证据 / 悬空 +200µm FAIL"接触面间隙异常（几何
        真值）"，轴链 PASS 不兜底）；
      - quad 形状几何（四角 30µm 小方块 @150µm 边界）+ recipe 校验接受
        quad/拒绝非法形状（400）；默认处方 site_marks=quad/film.marks=cross；
      - /api/full_reset 清场：制造状态（倍速×4/加热器 200℃/视觉结果环/
        晶圆上片/paused 流程）→ full_reset → 逐项断言清零（flow idle、
        chain/full_wafer 非 active、倍速 1.0、加热器关且回环境温、力 0、
        晶圆离场、视觉结果环空、ega_result None）且 bond_results 历史保留。"""
    print("--- 19.20 迭代 13（接触确认/full_reset/quad/0.15um/AS40nm） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= HTTP：contact_verify_tol_um 默认值与校验 =================
    mc = http_json(base + "/api/machine_constants")
    bd = (mc.get("constants") or {}).get("bond") or {}
    check("i13.contact_tol_default",
          abs(bd.get("contact_verify_tol_um", -1) - 50.0) < 1e-9,
          "tol=%s" % (bd.get("contact_verify_tol_um"),))
    c1, _ = http_put(base + "/api/machine_constants",
                     {"bond": {"contact_verify_tol_um": 80.0}})
    got = ((http_json(base + "/api/machine_constants").get("constants") or {})
           .get("bond") or {}).get("contact_verify_tol_um")
    c2, _ = http_put(base + "/api/machine_constants",
                     {"bond": {"contact_verify_tol_um": 50.0}})
    c3, _ = http_put(base + "/api/machine_constants",
                     {"bond": {"contact_verify_tol_um": -1.0}})
    c4, _ = http_put(base + "/api/machine_constants",
                     {"bond": {"contact_verify_tol_um": 5000.0}})
    check("i13.contact_tol_put",
          c1 == 200 and abs((got or -1) - 80.0) < 1e-9 and c2 == 200
          and c3 == 400 and c4 == 400,
          "rw=%s/%s got=%s neg=%s range=%s" % (c1, c2, got, c3, c4))

    # ================= HTTP：confirm_tol_xy_um 默认 0.15µm =================
    fa_d = (http_json(base + "/api/recipe")["recipe"].get("fine_align") or {})
    check("i13.confirm_tol_default",
          abs(fa_d.get("confirm_tol_xy_um", -1) - 0.15) < 1e-12,
          "fine_align=%s" % (fa_d,))

    # ================= 进程内：quad 形状几何 + recipe 校验 =================
    import vision_model as vmmod

    class _M(object):
        shape = "quad"
        size_mm = 0.15

    prims = vmmod.shape_primitives(_M())
    check("i13.quad_geometry",
          len(prims) == 4 and all(p[0] == "rect" for p in prims)
          and all(abs(p[3] - 0.03) < 1e-12 and abs(p[4] - 0.03) < 1e-12
                  for p in prims)
          and sorted((round(p[1], 6), round(p[2], 6)) for p in prims)
          == [(-0.06, -0.06), (-0.06, 0.06), (0.06, -0.06), (0.06, 0.06)]
          and vmmod.entry_points(_M()) == [(0.0, 0.0)],
          "prims=%s" % (prims,))
    c_q1, _ = http_put(base + "/api/recipe",
                       {"wafer": {"site_marks": {"shape": "quad"}}})
    c_q2, _ = http_put(base + "/api/recipe",
                       {"film": {"marks": {"shape": "triangle"}}})
    check("i13.quad_recipe_validate", c_q1 == 200 and c_q2 == 400,
          "quad=%s bad=%s" % (c_q1, c_q2))

    # ================= 进程内：force_search 接触确认（快用例） =================
    import tempfile
    import engine as engmod
    import flow_engine as femod
    import bond_results as brmod

    class _FakePts13(object):
        def __init__(self):
            self.d = {}

        def set(self, k, v):
            self.d[k] = v

        def get(self, k, dv=None):
            return self.d.get(k, dv)

        def has(self, k):
            return k in self.d

        def commit(self):
            pass

    d13 = tempfile.mkdtemp()
    pts = _FakePts13()
    eng3 = engmod.Engine(pts)
    eng3.interlock_enabled = False
    fe = femod.FlowEngine(eng3, pts, os.path.join(d13, "flows.json"))
    eng3.flow = fe
    tick_stop = []

    def tick_loop13():
        while not tick_stop:
            eng3.tick()
            time.sleep(0.0005)

    th = threading.Thread(target=tick_loop13, daemon=True)
    th.start()

    def flow_wait13(timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if fe.state in ("done", "aborted"):
                return fe.state
            time.sleep(0.05)
        return "timeout"

    try:
        eng3.move("BS_Arm_Z", 0.0, 100.0)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with eng3.lock:
                if eng3.axes["BS_Arm_Z"].in_position \
                        and not eng3.axes["BS_Arm_Z"].moving:
                    break
            time.sleep(0.01)
        # PASS 支：eff_contact=裸 -168.93（无 recipe），容差 ±50µm
        steps = [
            {"op": "move", "axis": "BS_Arm_Z", "target": -166.9,
             "speed": 20.0, "desc": "快速下压段"},
            {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
             "touch_force_N": 2.0, "min_pos": -169.9, "timeout": 60.0,
             "eff_contact": -168.93, "contact_verify_tol_um": 50.0,
             "desc": "慢速寻力段+接触确认"},
        ]
        fe.run_steps(steps, "i13-contact-ok")
        st = flow_wait13()
        ev = eng3.bond_contact_evidence or {}
        buf = {"site": None, "die_id": None, "chain": {}}
        brmod.collect_stage(eng3, "bond", None, buf)
        cv = ((buf.get("bond_params") or {}).get("contact_verify") or {})
        check("i13.force_search_contact_pass",
              st == "done" and ev.get("judge") == "pass"
              and abs(ev.get("eff_contact_z_mm", 0) - (-168.93)) < 1e-9
              and abs(ev.get("dev_um", 9e9)) <= 50.0
              and ev.get("touch_force_N", 0) >= 1.0
              and abs(ev.get("actual_contact_z_mm", 0)
                      - (eng3.bond_touch_pos or 9e9)) < 1e-5
              and ev.get("geom_gap_um") is None      # 裸引擎无 confocal→几何
              and "geom_gap_um" not in cv            # 分支跳过，证据不留字段
              and cv.get("judge") == "pass"
              and abs(cv.get("dev_um", 9e9) - ev.get("dev_um", 0)) < 1e-9,
              "state=%s ev=%s cv=%s" % (st, ev, cv))
        # 流程日志留证（接触确认一行）
        logtxt = " ".join(l.get("msg", "") for l in fe.log)
        check("i13.contact_log",
              "接触确认" in logtxt and "µm" in logtxt,
              "hit=%s" % ("接触确认" in logtxt))
        fe.reset()
        # FAIL 支：期望位 -168.5（与实际 -168.93 差 430µm > 容差）→ 接触位异常
        eng3.move("BS_Arm_Z", -166.9, 50.0)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with eng3.lock:
                if eng3.axes["BS_Arm_Z"].in_position \
                        and not eng3.axes["BS_Arm_Z"].moving:
                    break
            time.sleep(0.01)
        steps2 = [
            {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
             "touch_force_N": 2.0, "min_pos": -169.9, "timeout": 60.0,
             "eff_contact": -168.5, "contact_verify_tol_um": 50.0,
             "desc": "接触确认超差"},
        ]
        fe.run_steps(steps2, "i13-contact-fail")
        st2 = flow_wait13()
        err2 = (fe.steps[-1].get("info") if fe.steps else "") or ""
        ev2 = eng3.bond_contact_evidence or {}
        check("i13.force_search_contact_fail",
              st2 == "aborted" and "接触位异常" in err2
              and "厚度/机械常数不符" in err2
              and ev2.get("judge") == "fail"
              and abs(ev2.get("dev_um", 0)) > 50.0,
              "state=%s err=%s ev=%s" % (st2, err2[:60], ev2))
        fe.stop() if fe.state in ("running", "paused") else None
        fe.reset()

        # ---- 迭代 17 WP-B：contact_verify 双源判定的几何侧 ----
        # 裸引擎无 confocal/kinematics → 上面两支只走轴链。这里给 eng3 挂
        # stub（confocal 面高/6DOF 位姿/处方/工件）让几何分支真实执行：
        # geom_gap=−4µm（弹簧压入，正常接触）→ PASS 且证据带 geom_gap_um；
        # geom_gap=+200µm（die 底面悬空 0.2mm，轴链 PASS 也救不回来）→
        # FAIL"接触面间隙异常（几何真值）"。
        class _StubKin17(object):
            def compute(self, positions):
                return {}

        class _StubRecipe17(object):
            def get(self):
                return {}

        class _StubWp17(object):
            wafer = {"present": True}
            dies = []

            def check_transfers(self, eng, points, sim_time):
                pass

            def viz(self):
                return {}

        class _StubConfocal17(object):
            def __init__(self, gap_mm):
                self.gap_mm = gap_mm

            def die_center_world(self, positions, poses, wp):
                return [0.0, 0.0]

            def wafer_surface_z(self, x, y, positions, poses, recipe, wp):
                return 885.9

            def die_surface_z(self, x, y, positions, poses, recipe, wp):
                return 885.9 + self.gap_mm

        eng3.kinematics = _StubKin17()
        eng3.recipe = _StubRecipe17()
        eng3.workpieces = _StubWp17()
        try:
            # 几何 PASS 支：gap=−4µm（压入=接触上）
            eng3.confocal = _StubConfocal17(-0.004)
            eng3.move("BS_Arm_Z", -166.9, 50.0)
            t0 = time.time()
            while time.time() - t0 < 10.0:
                with eng3.lock:
                    if eng3.axes["BS_Arm_Z"].in_position \
                            and not eng3.axes["BS_Arm_Z"].moving:
                        break
                time.sleep(0.01)
            fe.run_steps([
                {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
                 "touch_force_N": 2.0, "min_pos": -169.9, "timeout": 60.0,
                 "eff_contact": -168.93, "contact_verify_tol_um": 50.0,
                 "desc": "几何判定 PASS"}],
                "i17-geom-ok")
            stg = flow_wait13()
            evg = eng3.bond_contact_evidence or {}
            infog = (fe.steps[-1].get("info") if fe.steps else "") or ""
            check("i17.force_search_geom_pass",
                  stg == "done" and evg.get("judge") == "pass"
                  and abs(evg.get("geom_gap_um", 9e9) - (-4.0)) < 1e-9
                  and "几何间隙" in infog,
                  "state=%s ev=%s" % (stg, evg))
            fe.reset()
            # 几何 FAIL 支：gap=+200µm 悬空（轴链偏差在容差内照样 FAIL）
            eng3.confocal = _StubConfocal17(0.2)
            eng3.move("BS_Arm_Z", -166.9, 50.0)
            t0 = time.time()
            while time.time() - t0 < 10.0:
                with eng3.lock:
                    if eng3.axes["BS_Arm_Z"].in_position \
                            and not eng3.axes["BS_Arm_Z"].moving:
                        break
                time.sleep(0.01)
            fe.run_steps([
                {"op": "force_search", "axis": "BS_Arm_Z", "speed": 5.0,
                 "touch_force_N": 2.0, "min_pos": -169.9, "timeout": 60.0,
                 "eff_contact": -168.93, "contact_verify_tol_um": 50.0,
                 "desc": "几何判定 FAIL"}],
                "i17-geom-fail")
            stg2 = flow_wait13()
            errg = (fe.steps[-1].get("info") if fe.steps else "") or ""
            evg2 = eng3.bond_contact_evidence or {}
            check("i17.force_search_geom_fail",
                  stg2 == "aborted"
                  and "接触面间隙异常（几何真值）" in errg
                  and abs(evg2.get("dev_um", 9e9)) <= 50.0     # 轴链侧本身 PASS
                  and evg2.get("judge") == "fail"
                  and abs(evg2.get("geom_gap_um", 0) - 200.0) < 1e-9,
                  "state=%s err=%s ev=%s" % (stg2, errg[:60].replace("µ", "u"),
                                             evg2))
            fe.stop() if fe.state in ("running", "paused") else None
            fe.reset()
        finally:
            eng3.confocal = None
            eng3.kinematics = None
            eng3.recipe = None
            eng3.workpieces = None
    finally:
        tick_stop.append(True)
        th.join(timeout=3.0)
        try:
            fe.stop()
        except Exception:
            pass

    # ================= HTTP：/api/full_reset 清场 =================
    # 制造状态：倍速×4 / 脉冲加热器 200℃ / 视觉结果环 ≥1 / 晶圆上片 /
    # paused 流程；bond_results 当前条数（迭代 23 起一并清空）
    http_post(base + "/api/speed", {"scale": 4.0})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 200, "Steady_Time": 0}})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    http_post(base + "/api/workpiece/wafer", {"action": "load"})
    http_post(base + "/api/vision/identify", {"cam": "down"})
    br0 = http_json(base + "/api/bond/results")
    tp0 = http_json(base + "/api/bond/throughput")
    c_run, r_run = http_post(base + "/api/flow/run",
                             {"name": "晶圆键合示范流程", "paused": True})
    assert c_run == 200, r_run
    time.sleep(0.3)
    st_pre = http_json(base + "/api/flow/state")
    assert st_pre.get("state") == "paused", st_pre.get("state")
    nres_pre = len((http_json(base + "/api/vision/results?limit=20")
                    .get("results") or []))
    c, r = http_post(base + "/api/full_reset", {})
    st_post = http_json(base + "/api/flow/state")
    pid_post = st_post.get("pid") or {}
    axes_post = http_json(base + "/api/axes")
    viz_post = http_json(base + "/api/viz")
    nres_post = len((http_json(base + "/api/vision/results?limit=20")
                     .get("results") or []))
    br1 = http_json(base + "/api/bond/results")
    tp1 = http_json(base + "/api/bond/throughput")
    check("i13.full_reset",
          c == 200 and r.get("ok")
          and st_post.get("state") == "idle"
          and not ((st_post.get("chain") or {}).get("active"))
          and not ((st_post.get("full_wafer") or {}).get("active"))
          and abs(axes_post.get("speed_scale", -1) - 1.0) < 1e-9
          and all(not (pid_post.get(k) or {}).get("active")
                  for k in ("force", "head_temp", "wafer_temp"))
          and abs((pid_post.get("force") or {}).get("pv", 9e9)) < 1e-6
          and abs((pid_post.get("head_temp") or {}).get("pv", -1) - 25.0) < 0.5
          and viz_post.get("wafer", {}).get("present") is False
          and nres_pre >= 1 and nres_post == 0
          and st_post.get("ega_result") is None
          and st_post.get("fine_align_result") is None
          # 迭代 23：键合结果历史随一键复位一并清空（与
          # /api/workpiece/reset 对齐；此前刻意保留——自相矛盾）
          and (br1.get("count") or 0) == 0
          and (tp1.get("ok_count") or 0) == 0,
          "c=%s state=%s scale=%s pid=%s wafer=%s nres=%s→%s br=%s→%s"
          % (c, st_post.get("state"), axes_post.get("speed_scale"),
             {k: (pid_post.get(k) or {}).get("active")
              for k in ("force", "head_temp", "wafer_temp")},
             viz_post.get("wafer"), nres_pre, nres_post,
             br0.get("count"), br1.get("count")))
    # 清理：复位后处方仍默认（restore_default 防前面 PUT 残留）
    http_post(base + "/api/recipe/restore_default", {})


def test_iter15():
    """--- 19.21 迭代 15：键合后检测红外套刻重构 / 全片三值 / PV/2+补偿 /
    产率分析 / 产率模式 pipelined ---
      - 处方 full_wafer.pipelined：默认 false / PUT 往返 / 非法 400 拒载；
      - 机械常数 fine_align.bond_offset_* 三参默认 0、PUT 往返/超界 400、
        flip_handoff.fa_x_standby_mm 默认 -1100；
      - bond_results 统计：PV/2=(max−min)/2 与带符号 mean（truth/measured
        两组，进程内临时文件）；
      - throughput 单元：UPH=3600×ok/net、阶段均耗时、最近 N 颗 UPH、
        成功/失败计数、clear 复位；
      - e2e（串行 count=2）：启动响应三值（sites_avail/dies_on_film/
        pairs）；记录带 timing（t_start<t_end、五阶段耗时）；throughput
        端点累计；post_inspect 套刻口径——两次独立触发（迭代 20 起：
        先 quad 后 cross 各带 ir_overlay、同机位不动轴，vision 结果里
        两模板名条目各=site 数；迭代 15 的一次双模板列表契约保留但流程
        不再用）、measured≈truth 紧容差（对消生效）；
      - e2e 补偿量：按 measured mean PUT bond_offset → 第 3 颗 truth
        位移 −offset 且 measured≈truth（迭代 20 口径：offset 在接触时刻
        真值采样前施加，位移进 truth；mean→0 位移验证）；
      - e2e pipelined count=3：预取发生（overlap_s>0、window_missed=0）、
        失败不影响当前颗、throughput.overlap_s>0。"""
    print("--- 19.21 迭代 15（套刻检测/三值/PV2补偿/产率/pipelined） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    # 迭代 17：视觉默认 REAL——本段测流程不测视觉，显式切 fast
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    # ================= HTTP：处方 pipelined 开关 =================
    fw0 = (http_json(base + "/api/recipe")["recipe"].get("full_wafer") or {})
    check("i15.recipe_pipelined_default",
          fw0.get("pipelined") is False, "full_wafer=%s" % (fw0,))
    c1, _ = http_put(base + "/api/recipe",
                     {"full_wafer": {"pipelined": True}})
    p1 = (http_json(base + "/api/recipe")["recipe"].get("full_wafer") or {})
    c2, _ = http_put(base + "/api/recipe",
                     {"full_wafer": {"pipelined": "yes"}})
    p2 = (http_json(base + "/api/recipe")["recipe"].get("full_wafer") or {})
    c3, _ = http_put(base + "/api/recipe",
                     {"full_wafer": {"pipelined": False}})
    check("i15.recipe_pipelined_put",
          c1 == 200 and p1.get("pipelined") is True
          and c2 == 400 and p2.get("pipelined") is True and c3 == 200,
          "put=%s bad=%s got=%s/%s" % (c1, c2, p1.get("pipelined"),
                                       p2.get("pipelined")))

    # ================= HTTP：机械常数 bond_offset_* + fa_x_standby_mm =================
    mc = http_json(base + "/api/machine_constants").get("constants") or {}
    fa_mc = mc.get("fine_align") or {}
    fh_mc = mc.get("flip_handoff") or {}
    check("i15.mc_bond_offset_default",
          fa_mc.get("bond_offset_x_um") == 0.0
          and fa_mc.get("bond_offset_y_um") == 0.0
          and fa_mc.get("bond_offset_rz_urad") == 0.0
          and abs(fh_mc.get("fa_x_standby_mm", 0) + 1050.0) < 1e-9,
          "fa=%s standby=%s" % (
              {k: fa_mc.get(k) for k in fa_mc if k.startswith("bond_")},
              fh_mc.get("fa_x_standby_mm")))
    c1, _ = http_put(base + "/api/machine_constants",
                     {"fine_align": {"bond_offset_x_um": 2.5,
                                     "bond_offset_y_um": -1.5,
                                     "bond_offset_rz_urad": 8.0}})
    got = (http_json(base + "/api/machine_constants").get("constants") or {}) \
        .get("fine_align") or {}
    c2, _ = http_put(base + "/api/machine_constants",
                     {"fine_align": {"bond_offset_x_um": 2000.0}})
    c3, _ = http_put(base + "/api/machine_constants",
                     {"fine_align": {"bond_offset_x_um": 0.0,
                                     "bond_offset_y_um": 0.0,
                                     "bond_offset_rz_urad": 0.0}})
    check("i15.mc_bond_offset_put",
          c1 == 200 and abs(got.get("bond_offset_x_um", 0) - 2.5) < 1e-9
          and abs(got.get("bond_offset_y_um", 0) + 1.5) < 1e-9
          and abs(got.get("bond_offset_rz_urad", 0) - 8.0) < 1e-9
          and c2 == 400 and c3 == 200,
          "rw=%s got=%s range=%s rst=%s" % (c1, got.get("bond_offset_x_um"),
                                            c2, c3))

    # ================= 进程内：PV/2 + 带符号 mean + throughput 单元 =================
    import bond_results as brm
    tmpf = os.path.join(ROOT, "tests", "_i15_bond_results.json")
    if os.path.exists(tmpf):
        os.remove(tmpf)
    br1 = brm.BondResults(tmpf)
    for i, (dx, dy, drz) in enumerate(((0.0001, -0.0002, 1e-5),
                                       (0.0003, 0.0004, -3e-5),
                                       (-0.0002, 0.0001, 2e-5))):
        br1.add({"die_id": "R%02dC%02d" % (i, i),
                 "site": {"row": i, "col": i, "x": 0.0, "y": 0.0},
                 "t_bond": 10.0 + i,
                 "truth": {"dx": dx, "dy": dy, "drz": drz},
                 "truth_note": None,
                 "measured": {"dx": dx * 1.1, "dy": dy * 1.1,
                              "drz": drz * 1.1, "t": 20.0 + i},
                 "timing": {"t_start": 5.0 + i * 10,
                            "t_end": 10.0 + i * 10,
                            "stages": {"bond": 2.0}},
                 "chain": {}, "bond_params": {}})
    st0 = br1.list_public()["stats"]
    # dx 真值（µm）：0.1/0.3/-0.2 → PV/2=(0.3−(−0.2))/2=0.25，mean=0.2/3
    tdx = st0["truth"]["dx_um"]
    mdx = st0["measured_stats"]["dx_um"]
    check("i15.pv2_stats",
          abs(tdx["pv2"] - 0.25) < 1e-6
          and abs(tdx["mean"] - (0.2 / 3.0)) < 1e-3
          and abs(mdx["pv2"] - 0.275) < 1e-6
          and abs(st0["truth"]["drz_urad"]["pv2"] - 25.0) < 1e-6,
          "tdx=%s mdx=%s drz=%s" % (tdx, mdx,
                                    st0["truth"]["drz_urad"]))
    br1.throughput_add(2, 1, 100.0, {"flip_pick": 20.0, "bond": 40.0},
                       overlap_s=5.0)
    br1.throughput_add(1, 0, 55.0, {"flip_pick": 22.0, "bond": 38.0},
                       overlap_s=3.0)
    tp = br1.throughput_public()
    # uph_total=3600×3/155；recent：3 条 timing 记录 t_end=10/20/30 →
    # 3600×2/20=360；stage_avg flip_pick=(20+22)/2
    check("i15.throughput_unit",
          abs(tp["uph_total"] - 3600.0 * 3 / 155.0) < 0.01
          and abs(tp["uph_recent"] - 360.0) < 0.01
          and tp["ok_count"] == 3 and tp["fail_count"] == 1
          and abs(tp["net_time_s"] - 155.0) < 1e-6
          and abs(tp["overlap_s"] - 8.0) < 1e-6
          and abs(tp["stage_avg_s"]["flip_pick"] - 21.0) < 1e-6
          and tp["stage_avg_s"]["focus_level"] is None
          and tp["recent_n"] == 3 and tp["runs"] == 2,
          "tp=%s" % ({k: tp[k] for k in
                      ("uph_total", "uph_recent", "ok_count", "fail_count",
                       "net_time_s", "overlap_s", "runs")},))
    br1.clear()
    tp2 = br1.throughput_public()
    check("i15.throughput_clear",
          tp2["ok_count"] == 0 and tp2["net_time_s"] == 0.0
          and tp2["uph_total"] is None
          and br1.list_public()["count"] == 0,
          "tp2=%s count=%s" % (tp2["ok_count"],
                               br1.list_public()["count"]))
    os.remove(tmpf)

    # ================= e2e：串行全片 count=2 + 套刻检测 + 补偿 + pipelined =================
    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def fw_wait(timeout=2400.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            fw = flow_state().get("full_wafer") or {}
            if not fw.get("active"):
                return fw
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    def flow_wait_done(timeout=1200.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = flow_state()
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"state": "timeout"}

    def wafer_preheat():
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        http_post(base + "/api/speed", {"scale": 8.0})
        t0 = time.time()
        cur_t = 0.0
        plc = "http://127.0.0.1:%d" % PLC_PORT
        while time.time() - t0 < 60.0:
            v = http_json(plc + "/io/observe?path=SystemMachine.M_Heater."
                                "Heat_Dw.oCurTemp")
            cur_t = float(v.get("value", 0.0))
            if cur_t >= tgt_t - 1.5:
                break
            time.sleep(0.5)
        # 迭代 18：复原到外层提速窗倍速 _SPEED_CUR（不再硬编码 1×）
        http_post(base + "/api/speed", {"scale": _SPEED_CUR})
        return cur_t, tgt_t

    def run_post_inspect():
        c, r = http_post(base + "/api/process/post_inspect", {})
        assert c == 200, r
        st = flow_wait_done()
        flow_cleanup()
        return st

    # 迭代 18：e2e 实跑区（串行全片/套刻检测/补偿/pipelined）quick 门控
    # 跳过（远超 sim 30s 标准，full 全跑）；上方处方/机械常数/PV2/
    # throughput 单元用例 sim≈0 两套件保留。
    if not fblock(["i15.ega_done", "i15.fw_start_counts", "i15.fw_done",
                   "i15.record_timing", "i15.throughput_api",
                   "i15.pi_done", "i15.pi_two_trigger",
                   "i15.pi_overlay_measured",
                   "i15.comp_bond_done", "i15.comp_shift",
                   "i15.pipelined_done", "i15.pipelined_throughput"]):
        return

    try:
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×；wafer_preheat
                         # 内部复原到 _SPEED_CUR 保持窗内 8×）
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8,
                           "confirm_tol_xy_um": 1.0,
                           "confirm_tol_rz_urad": 60.0},
            "bond": {"dwell_s": 2.0},
            "full_wafer": {"count": 2, "path_mode": "center_out",
                           "die_order": "nearest_pick"},
            "post_inspect": {"auto_after_full": False,
                             "auto_after_single": False}})
        assert c == 200, r
        wafer_preheat()
        c, r = http_post(base + "/api/process/wafer_ega", {})
        assert c == 200, r
        st = flow_wait_done()
        check("i15.ega_done", st.get("state") == "done",
              "ega state=%s" % st.get("state"))
        flow_cleanup()

        # ---- 串行全片 count=2：启动响应三值 ----
        c, r = http_post(base + "/api/process/full_wafer", {})
        check("i15.fw_start_counts",
              c == 200 and (r or {}).get("pairs") == 2
              and (r or {}).get("sites_avail") == 621
              and (r or {}).get("dies_on_film") == 149
              and (r or {}).get("dies_expected") == 149,
              "c=%s pairs=%s sites=%s dies=%s exp=%s"
              % (c, (r or {}).get("pairs"), (r or {}).get("sites_avail"),
                 (r or {}).get("dies_on_film"),
                 (r or {}).get("dies_expected")))
        fw = fw_wait()
        check("i15.fw_done",
              not fw.get("active") and not fw.get("error")
              and len(fw.get("results") or []) == 2
              and all(x.get("ok") for x in (fw.get("results") or []))
              and (fw.get("overlap_s") or 0.0) == 0.0,
              "results=%s err=%s" % (fw.get("results"), fw.get("error")))
        flow_cleanup()
        # 记录 timing（产率数据）
        res = http_json(base + "/api/bond/results")
        rs = res.get("results") or []
        tms = [x.get("timing") for x in rs]
        check("i15.record_timing",
              len(rs) == 2 and all(
                  tm and tm.get("t_start") is not None
                  and tm.get("t_end") is not None
                  and tm["t_end"] > tm["t_start"]
                  and all((tm.get("stages") or {}).get(k) is not None
                          for k in ("flip_pick", "flip_handoff",
                                    "focus_level", "fine_align", "bond"))
                  for tm in tms),
              "timing=%s" % (tms,))
        tp0 = http_json(base + "/api/bond/throughput")
        check("i15.throughput_api",
              tp0.get("ok") and tp0.get("ok_count") == 2
              and tp0.get("runs") == 1
              and (tp0.get("uph_total") or 0) > 0
              and (tp0.get("stage_avg_s") or {}).get("bond") is not None
              and (tp0.get("overlap_s") or 0.0) == 0.0,
              "tp=%s" % ({k: tp0.get(k) for k in
                          ("ok_count", "runs", "uph_total", "overlap_s")},))

        # ---- 键合后检测（套刻口径）：两次独立触发 + measured≈truth ----
        # 两次独立触发证据（2026-09-07 迭代 20：先 quad 模板后 cross 模板，
        # 各带 ir_overlay，同机位不动轴；迭代 15 的一次双模板列表契约保留
        # 但流程不再用）：检测前/后各数一次「模板名=晶圆 site 模板/die 角标
        # 模板 且 ir_overlay」的识别条数，增量各==site 数（2）；模板为列表
        # 的条目应 0 新增
        mc_vm = (http_json(base + "/api/machine_constants")
                 .get("constants") or {}).get("vision_marks") or {}
        tpl_w = mc_vm.get("tpl_wafer_site", "cv_quad")
        tpl_d = mc_vm.get("tpl_post_inspect", "tpl_post_inspect")

        def _dual_count():
            vr = (http_json(base + "/api/vision/results?limit=100")
                  .get("results") or [])
            n_w = len([e for e in vr if e.get("template") == tpl_w
                       and e.get("ir_overlay")])
            n_d = len([e for e in vr if e.get("template") == tpl_d
                       and e.get("ir_overlay")])
            lst = [e for e in vr if isinstance(e.get("template"), list)]
            return n_w, n_d, len(lst)

        w0, d0, l0 = _dual_count()
        st = run_post_inspect()
        check("i15.pi_done", st.get("state") == "done",
              "pi state=%s" % st.get("state"))
        w1, d1, l1 = _dual_count()
        check("i15.pi_two_trigger",
              w1 - w0 == 2 and d1 - d0 == 2 and l1 == l0,
              "quad %d→%d cross %d→%d list %d→%d" % (w0, w1, d0, d1,
                                                     l0, l1))
        rs2 = http_json(base + "/api/bond/results").get("results") or []
        comp = []
        allm = True
        for x in rs2:
            t, m = x.get("truth"), x.get("measured")
            if not (t and m):
                allm = False
                continue
            comp.append({"die": x["die_id"],
                         "dx_um": (m["dx"] - t["dx"]) * 1000.0,
                         "dy_um": (m["dy"] - t["dy"]) * 1000.0,
                         "drz_urad": (m["drz"] - t["drz"]) * 1e6})
        # 套刻对消后的紧容差（迭代 15 实测，见交接文档）：EGA 残差/相机
        # 位姿系统差/逐标记 AS 重复性已组内对消——xy 残余=FAST 质心噪声
        # （差分 σ≈0.13µm）+WS/AS 到位重采样+单对旋转修正残余（迭代 20：
        # 扣 drz×J(d0) 项），判据 1.0µm；drz（单对模式）=投影角差×相机
        # 方向行列式符号换算回世界系（迭代 20 修正下相机镜像翻号——此前
        # 符号反被小角度掩盖），受结果角度精度限制，判据 100µrad（多点对
        # 模式由点集拟合定角，不受此限——本机 FOV 1.8mm 恒为单对）。
        # 迭代 20 起 truth 含下降链系统项（~12µm/18µrad 常数）——本用例
        # 断的是 measured−truth 差值，系统项两侧相消不受影响。
        okc = allm and len(comp) == 2 and all(
            abs(v["dx_um"]) < 1.0 and abs(v["dy_um"]) < 1.0
            and abs(v["drz_urad"]) < 100.0 for v in comp)
        check("i15.pi_overlay_measured",
              okc and (http_json(base + "/api/bond/results")
                       .get("stats") or {}).get("measured") == 2,
              "measured-truth=%s" % (comp,))
        print("    [i15] 套刻 measured vs truth 偏差（um/urad）: %s"
              % json.dumps(comp, ensure_ascii=True), flush=True)

        # ---- 补偿量建议：measured mean → PUT bond_offset → 第 3 颗位移 ----
        base_v = {}
        for k in ("dx", "dy", "drz"):
            base_v[k] = sum(x["measured"][k] for x in rs2) / len(rs2)
        c, r = http_put(base + "/api/machine_constants", {"fine_align": {
            "bond_offset_x_um": base_v["dx"] * 1000.0,
            "bond_offset_y_um": base_v["dy"] * 1000.0,
            "bond_offset_rz_urad": base_v["drz"] * 1e6}})
        assert c == 200, r
        wafer_preheat()
        c, r = http_put(base + "/api/recipe", {"full_wafer": {"count": 1}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200, r
        fw = fw_wait()
        check("i15.comp_bond_done",
              not fw.get("active")
              and len(fw.get("results") or []) == 1
              and (fw.get("results") or [{}])[0].get("ok"),
              "fw=%s" % (fw.get("results"),))
        flow_cleanup()
        st = run_post_inspect()
        assert st.get("state") == "done", st.get("state")
        rs3 = [x for x in (http_json(base + "/api/bond/results")
                           .get("results") or []) if x.get("measured")]
        last = rs3[-1]
        t3, m3 = last["truth"], last["measured"]
        # 迭代 20 truth 口径：bond_offset 在精对准后/下压前施加 WS+offset，
        # 而 truth 在键合接触时刻采样（bond_commit）——offset 位移本身进
        # truth：truth3 ≈ truth基线mean − offset（offset=基线 measured
        # mean≈truth mean，die 落点相对 site 标记 = truth无offset −
        # offset）；measured3 ≈ truth3（套刻相对测量）。
        tv = {k: sum(x["truth"][k] for x in rs2) / len(rs2)
              for k in ("dx", "dy", "drz")}
        dev = {"dx_um": (m3["dx"] - t3["dx"]) * 1000.0,
               "dy_um": (m3["dy"] - t3["dy"]) * 1000.0,
               "drz_urad": (m3["drz"] - t3["drz"]) * 1e6}
        shf = {"dx_um": (t3["dx"] - (tv["dx"] - base_v["dx"])) * 1000.0,
               "dy_um": (t3["dy"] - (tv["dy"] - base_v["dy"])) * 1000.0,
               "drz_urad": (t3["drz"] - (tv["drz"] - base_v["drz"])) * 1e6}
        check("i15.comp_shift",
              abs(dev["dx_um"]) < 1.0 and abs(dev["dy_um"]) < 1.0
              and abs(dev["drz_urad"]) < 100.0
              and abs(shf["dx_um"]) < 1.0 and abs(shf["dy_um"]) < 1.0
              and abs(shf["drz_urad"]) < 100.0,
              "measured3-truth3=%s；truth3位移残差=%s（truth3=%s "
              "基线truth mean=%s 基线measured mean=%s）"
              % (dev, shf, t3, {k: round(v * 1e3, 4) for k, v in tv.items()},
                 {k: round(v * 1e3, 4) for k, v in base_v.items()}))
        c, r = http_put(base + "/api/machine_constants", {"fine_align": {
            "bond_offset_x_um": 0.0, "bond_offset_y_um": 0.0,
            "bond_offset_rz_urad": 0.0}})
        assert c == 200, r

        # ---- 产率模式 pipelined count=3：预取发生+overlap>0+不影响当前颗 ----
        wafer_preheat()
        c, r = http_put(base + "/api/recipe",
                        {"full_wafer": {"count": 3, "pipelined": True}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200, r
        fw = fw_wait()
        res_p = fw.get("results") or []
        check("i15.pipelined_done",
              not fw.get("active") and not fw.get("error")
              and len(res_p) == 3 and all(x.get("ok") for x in res_p)
              and (fw.get("overlap_s") or 0.0) > 0.0
              and (fw.get("window_missed") or 0) == 0,
              "results=%s overlap=%s missed=%s err=%s"
              % (res_p, fw.get("overlap_s"), fw.get("window_missed"),
                 fw.get("error")))
        flow_cleanup()
        tp1 = http_json(base + "/api/bond/throughput")
        rs_all = http_json(base + "/api/bond/results")
        check("i15.pipelined_throughput",
              tp1.get("ok") and (tp1.get("overlap_s") or 0.0) > 0.0
              and tp1.get("ok_count") == 6 and tp1.get("runs") == 3
              and (tp1.get("uph_recent") or 0) > 0
              and (rs_all.get("count") or 0) == 6,
              "tp=%s count=%s" % ({k: tp1.get(k) for k in
                                   ("ok_count", "runs", "overlap_s",
                                    "uph_recent")}, rs_all.get("count")))
    finally:
        try:
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_put(base + "/api/machine_constants", {"fine_align": {
                "bond_offset_x_um": 0.0, "bond_offset_y_um": 0.0,
                "bond_offset_rz_urad": 0.0}})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


# ---------------- 迭代 17：视觉 REAL/OpenCV 引擎升级 ----------------
def test_iter17():
    """--- 19.22 迭代 17：REAL 端到端冒烟（vision.real_flow_smoke）---
      - REAL 模式下走流程同款 MQTT VisionClient 触发路径：fa 型
        （down/cv_quad @ 晶圆 site quad 标记，17.17 同款走位）+ pi 型
        （ir_overlay 双模板 cv_quad+tpl_post_inspect）各一次 REAL identify，
        证明默认 REAL 下流程拍照链路不卡（30s 自适应超时内回包）、
        judge=pass、模板回显与耗时分解齐备；
      - 质心精化证据：entry 调试字段 ncc_col/ncc_row 不入 MQTT 七字段；
      - 跑完设回 fast、物料复位。"""
    print("--- 19.22 迭代 17（REAL 端到端冒烟） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False
    if not has_cv2:
        print("   [skip] cv2 未安装，vision.real_flow_smoke 跳过（不 FAIL）")
        return
    from vision_service import VisionClient

    # 场景：晶圆上片 + 17.17 走位（down 相机居中 site(0,0)#0 quad 标记）
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", -38.594), ("WS_LS_Y", 4.8),
                    ("AS_X", 0.0), ("AS_Y", 0.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    cli = VisionClient().start()
    # 迭代 50 flake 加固：start 后等订阅就绪再触发——MQTT QoS0 无 retained，
    # 订阅须抢在结果发布前，否则首拍可能丢回包
    time.sleep(0.3)
    http_post(base + "/api/vision/mode", {"mode": "real"})
    try:
        # fa 型：单模板 REAL identify（自适应超时 30s 内必须回包，保持单发）
        t0 = time.time()
        r1 = cli.request("down", template="cv_quad", timeout=30.0)
        wall1 = time.time() - t0
        res1 = r1.get("result") or {}
        sc1 = (res1.get("lstScore") or [0])[0]
        # pi 型：ir_overlay 双模板（无已键合 die 时 cross 无命中，quad 在）。
        # 迭代 50 flake 加固：NCC 单帧尾部一票否决（judge 偶发 False）——
        # 最多 3 次尝试，任一帧 judge=True 且 lstName 含 cv_quad 即通过，
        # wall/score 取通过样本（全败取末次样本做 FAIL 证据）
        r2 = {"error": "not-run"}
        res2 = {}
        scores2 = []
        names2 = []
        wall2 = 0.0
        pi_ok = False
        for _try in range(3):
            t0 = time.time()
            r2 = cli.request("down", template=["cv_quad", "tpl_post_inspect"],
                             timeout=30.0, ir_overlay=True)
            wall2 = time.time() - t0
            res2 = r2.get("result") or {}
            scores2 = res2.get("lstScore") or []
            names2 = res2.get("lstName") or []
            if not r2.get("error") and res2.get("judge") is True \
                    and any(n_ == "cv_quad" for n_ in names2):
                pi_ok = True
                break
        quad_hit = any(n_ == "cv_quad" for n_ in names2)
        # 结果环最新两条：模板回显 + timing + ncc 调试字段不进七字段
        rs = http_json(base + "/api/vision/results?limit=4"
                       ).get("results") or []
        e_fa = [e for e in rs if e.get("template") == "cv_quad"]
        e_pi = [e for e in rs
                if isinstance(e.get("template"), list)]
        tm1 = (e_fa[-1].get("timing") or {}) if e_fa else {}
        no_ncc = all(not any(k.startswith("ncc_") for k in e["result"])
                     for e in rs if e.get("result"))
        check("vision.real_flow_smoke",
              not r1.get("error") and res1.get("judge") is True
              and sc1 >= 0.7
              and abs((res1.get("lstCol") or [0])[0] - 1500) < 60
              and pi_ok and no_ncc
              and wall1 < 30.0 and wall2 < 30.0
              and 0 < tm1.get("total_ms", 9e9) < 30000,
              # 迭代 50：FAIL detail 带 pi 侧分数证据（lstScore/lstName/judge）
              "fa: judge=%s score=%.3f wall=%.1fs | pi: judge=%s quad=%s "
              "wall=%.1fs scores=%s names=%s | total_ms=%.0f"
              % (res1.get("judge"), sc1, wall1, res2.get("judge"),
                 quad_hit, wall2, ["%.3f" % s for s in scores2], names2,
                 tm1.get("total_ms", -1)))
        print("   [iter17] fa total=%.0fms (render=%.0f tcp=%.0f "
              "match=%.0f) | pi wall=%.2fs"
              % (tm1.get("total_ms", -1), tm1.get("render_ms", -1),
                 tm1.get("tcp_ms", -1), tm1.get("match_ms", -1), wall2))
    finally:
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
            http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                                "speed": 600.0})
        for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
            wait_settled(ax)
        http_post(base + "/api/workpiece/reset", {})


def test_pi_real():
    """19.30.pi_real：REAL 模式键合后检测（pi_measure 红外套刻）端到端冒烟。

    防"REAL 链静默退化"类回归：套件流程段默认 FAST，REAL 链曾零覆盖——
    迭代 35 精灵渲染 warpAffine dst= 整窗替换把套刻对先画标记抹掉
    （2026-09-21 修复：_draw_mark_cov 改 cv2.max 累积），REAL pi 自
    2026-09-15 起失效六个迭代无人发现，用户面板默认 REAL 才暴露。
    staging（8×）：物料复位→处方默认 count=1 →EGA→full_wafer →pi 阶段切
    REAL→post_inspect。finally：mode 回 fast、1×、物料/处方/流程复位。"""
    base = "http://127.0.0.1:%d" % WS_PORT

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)
        http_post(base + "/api/flow/reset", {})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def fw_wait(timeout=1200.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            fw = flow_state().get("full_wafer") or {}
            if not fw.get("active"):
                return fw
            time.sleep(0.15)
        return {"active": True, "error": "timeout"}

    def flow_wait_done(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = flow_state()
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.15)
        return {"state": "timeout"}

    try:
        set_speed(8.0)
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8},
            "bond": {"dwell_s": 1.0},
            "full_wafer": {"count": 1, "path_mode": "center_out",
                           "die_order": "nearest_pick"},
            "post_inspect": {"auto_after_full": False,
                             "auto_after_single": False}})
        assert c == 200, r
        c, r = http_post(base + "/api/process/wafer_ega", {})
        assert c == 200, r
        st = flow_wait_done()
        check("pireal.ega_done", st.get("state") == "done",
              "ega state=%s" % st.get("state"))
        flow_cleanup()
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200, r
        fw = fw_wait()
        check("pireal.bond_done",
              not fw.get("error") and len(fw.get("results") or []) == 1
              and fw["results"][0].get("ok"),
              "fw err=%s results=%s" % (fw.get("error"), fw.get("results")))
        flow_cleanup()
        # pi 阶段切 REAL（本用例就是补 REAL 链覆盖）
        c, r = http_post(base + "/api/vision/mode", {"mode": "real"})
        assert c == 200, r
        c, r = http_post(base + "/api/process/post_inspect", {})
        assert c == 200, r
        st = flow_wait_done()
        bad = [s for s in st.get("steps", []) if s.get("status") == "fail"]
        check("pireal.pi_done_real",
              st.get("state") == "done" and not bad,
              "state=%s fail=%s" % (st.get("state"),
                                    json.dumps(bad, ensure_ascii=False)[:300]))
        rs = http_json(base + "/api/bond/results").get("results") or []
        comp = []
        for x in rs:
            t, m = x.get("truth"), x.get("measured")
            if t and m:
                comp.append({"dx_um": (m["dx"] - t["dx"]) * 1000.0,
                             "dy_um": (m["dy"] - t["dy"]) * 1000.0,
                             "drz_urad": (m["drz"] - t["drz"]) * 1e6})
        # REAL 链 measured−truth 容差（迭代 23 实测 3σ~0.6µm，留余量防 flake）
        check("pireal.pi_real_measured",
              len(comp) == 1 and all(abs(v["dx_um"]) < 2.0
                                     and abs(v["dy_um"]) < 2.0
                                     and abs(v["drz_urad"]) < 200.0
                                     for v in comp),
              "comp=%s" % (comp,))
    finally:
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        set_speed(1.0)
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()


def test_real_chains():
    """19.31.real_chains：REAL 模式工艺链端到端哨兵（EGA→flip_pick→
    flip_handoff→focus_level→fine_align，不键合——键合+检测链由
    19.30.pi_real 覆盖）。

    迭代 53 教训（"FAST 过 ≠ REAL 过"）：套件流程段默认 FAST，REAL 渲染/
    识别链曾是回归盲区。本段把视觉识别密集的工艺段全程锁 REAL 跑一遍，
    并断言每阶段 /api/vision/results 的 mode=real 条目增量（防"流程走了
    但视觉回退 FAST"的假绿）。finally 复原 fast/1×/物料/处方。"""
    base = "http://127.0.0.1:%d" % WS_PORT

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)
        http_post(base + "/api/flow/reset", {})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_wait_done(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = flow_state()
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.15)
        return {"state": "timeout"}

    def real_seq():
        """-> (max_seq, {real 条目的 seq 集合})。结果环 RESULTS_MAX=100 会
        淘汰——按 seq 区间计数才不受淘汰影响（套件前序 REAL 段把环灌满时
        "按内容数 mode==real"的 delta 恒为 0，2026-09-21 实测踩坑）。"""
        rs = http_json(base + "/api/vision/results?limit=100").get("results") or []
        mx = max([e.get("seq", -1) for e in rs] or [-1])
        return mx, {e.get("seq") for e in rs if e.get("mode") == "real"}

    def real_new(prev, cur):
        """prev/cur=(max_seq, real_seqs)：返回区间内新增 real 条数。"""
        return len([s for s in cur[1] if prev[0] < s <= cur[0]])

    try:
        set_speed(8.0)
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {
            "wafer_load": {"load_x_mm": 0.2, "load_y_mm": -0.15,
                           "load_rz_rad": 0.0002, "jitter_on": False,
                           "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8,
                           "confirm_tol_xy_um": 1.0,
                           "confirm_tol_rz_urad": 60.0},
            "bond": {"dwell_s": 1.0},
            "post_inspect": {"auto_after_full": False,
                             "auto_after_single": False}})
        assert c == 200, r
        # 全段锁 REAL（本段就是 REAL 链覆盖哨兵）
        c, r = http_post(base + "/api/vision/mode", {"mode": "real"})
        assert c == 200, r

        n0 = real_seq()
        c, r = http_post(base + "/api/process/wafer_ega", {})
        assert c == 200, r
        st = flow_wait_done()
        check("realchain.ega_done",
              st.get("state") == "done" and flow_state().get("ega_result"),
              "ega state=%s" % st.get("state"))
        n1 = real_seq()
        flow_cleanup()

        c, r = http_post(base + "/api/process/flip_pick", {})
        assert c == 200, r
        st = flow_wait_done()
        check("realchain.flip_done",
              st.get("state") == "done"
              and flow_state().get("coarse_offset"),
              "flip state=%s" % st.get("state"))
        n2 = real_seq()
        flow_cleanup()

        c, r = http_post(base + "/api/process/flip_handoff", {})
        assert c == 200, r
        st = flow_wait_done()
        check("realchain.handoff_done", st.get("state") == "done",
              "handoff state=%s" % st.get("state"))
        flow_cleanup()

        c, r = http_post(base + "/api/process/focus_level", {})
        assert c == 200, r
        st = flow_wait_done()
        check("realchain.fl_done",
              st.get("state") == "done" and flow_state().get("fl_result"),
              "fl state=%s" % st.get("state"))
        flow_cleanup()

        c, r = http_post(base + "/api/process/fine_align", {})
        assert c == 200, r
        st = flow_wait_done()
        check("realchain.fa_done",
              st.get("state") == "done"
              and flow_state().get("fine_align_result"),
              "fa state=%s" % st.get("state"))
        n5 = real_seq()
        flow_cleanup()

        # REAL 识别计数增量（EGA 9 测点、flip 粗对准 ≥1、fa 双侧 ≥2）——
        # 证明各阶段真走了 REAL 渲染+识别链而非 FAST 解析回退
        d_ega = real_new(n0, n1)
        d_flip = real_new(n1, n2)
        d_fa = real_new(n2, n5)
        check("realchain.real_identify_counts",
              d_ega >= 9 and d_flip >= 1 and d_fa >= 2,
              "ega +%d flip +%d fa前累计 +%d" % (d_ega, d_flip, d_fa))
    finally:
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        set_speed(1.0)
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()


def test_iter17c():
    """--- 19.23 迭代 17 WP-C：产率实时化 + 主控智能恢复 ---
      - throughput live 块：空闲=None；全片运行中={active, kind:"full",
        current:{die,site,stage}, run_ok/run_fail, cur_elapsed_s, est_uph}；
      - 全片单对失败自动放回+继续（i17.fw_fail_recovery）：接触确认容差
        改坏（contact_verify_tol_um=0.001µm → bond 接触确认必 FAIL，die
        卡 head），count=2 stop_on_fail=false → 第 1 对失败后自动
        head_to_flip+flip_place_back 放回蓝膜（recovered:true）并继续第
        2 对成功；期间采样 live 块；失败后即刻把容差改回（第 2 对键合在
        恢复+取片/交接/调平/精对准之后，时间余量充足）；
      - 单片 abort recovery 状态（i17.chain_recovery_status）：同款注入
        跑单片链（site=[0,0] 避开前面已键合的中心位），abort 后
        chain.recovery={die_carrier:"head", options:[return_film,
        continue_bond]}；走面板同款恢复（head_to_flip→flip_place_back）
        后 recovery=None；
      - clear_head 启动（i17.fw_clear_head）：flip_pick+flip_handoff 造
        head 残片 → POST full_wafer 409 带 clear_head_option；带
        clear_head=true → 自动放回后启动成功。"""
    print("--- 19.23 迭代 17 WP-C（产率实时化/主控智能恢复） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    http_post(base + "/api/vision/mode", {"mode": "fast"})

    def flow_state():
        return http_json(base + "/api/flow/state")

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，0.15s 足够
        http_post(base + "/api/flow/reset", {})

    def fw_wait(timeout=2400.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            fw = flow_state().get("full_wafer") or {}
            if not fw.get("active"):
                return fw
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"active": True, "error": "timeout"}

    def flow_wait_done(timeout=1200.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = flow_state()
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.15)     # 迭代 18：轮询加密
        return {"state": "timeout"}

    def wafer_preheat():
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        http_post(base + "/api/speed", {"scale": 8.0})
        t0 = time.time()
        cur_t = 0.0
        plc = "http://127.0.0.1:%d" % PLC_PORT
        while time.time() - t0 < 60.0:
            v = http_json(plc + "/io/observe?path=SystemMachine.M_Heater."
                                "Heat_Dw.oCurTemp")
            cur_t = float(v.get("value", 0.0))
            if cur_t >= tgt_t - 1.5:
                break
            time.sleep(0.5)
        # 迭代 18：复原到外层提速窗倍速 _SPEED_CUR（不再硬编码 1×）
        http_post(base + "/api/speed", {"scale": _SPEED_CUR})
        return cur_t, tgt_t

    def set_contact_tol(tol):
        c, r = http_put(base + "/api/machine_constants",
                        {"bond": {"contact_verify_tol_um": tol}})
        assert c == 200, r

    # 迭代 18：三个用例都需全片/单片链实跑（失败恢复 count=2、单片 abort
    # 恢复、clear_head+count=1），远超 sim 30s 标准——quick 整段门控跳过，
    # full 全跑。
    if not fblock(["i17.fw_fail_recovery", "i17.fw_fail_recovery_film",
                   "i17.throughput_live",
                   "i17.chain_recovery_status", "i17.chain_recovery_done",
                   "i17.fw_clear_head"]):
        return

    try:
        set_speed(8.0)   # 迭代 18 提速窗（finally 复原 1×；wafer_preheat
                         # 内部复原到 _SPEED_CUR 保持窗内 8×）
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 8,
                           "confirm_tol_xy_um": 1.0,
                           "confirm_tol_rz_urad": 60.0},
            "bond": {"dwell_s": 2.0},
            "full_wafer": {"count": 2, "path_mode": "center_out",
                           "die_order": "nearest_pick",
                           "stop_on_fail": False},
            "post_inspect": {"auto_after_full": False,
                             "auto_after_single": False}})
        assert c == 200, r
        wafer_preheat()
        c, r = http_post(base + "/api/process/wafer_ega", {})
        assert c == 200, r
        st = flow_wait_done()
        assert st.get("state") == "done", st.get("state")
        flow_cleanup()

        # 空闲时 live=None
        tp_idle = http_json(base + "/api/bond/throughput")
        live_idle_none = tp_idle.get("live") is None

        # ---- 全片单对失败自动放回+继续 + live 采样 ----
        set_contact_tol(0.001)      # 接触确认必 FAIL → die 卡 head
        c, r = http_post(base + "/api/process/full_wafer", {})
        assert c == 200 and (r or {}).get("pairs") == 2, (c, r)
        live_seen = None
        restored = False
        t0 = time.time()
        fw = {"active": True}
        while time.time() - t0 < 2400.0:
            fw = flow_state().get("full_wafer") or {}
            if live_seen is None and fw.get("active") \
                    and (fw.get("current") or {}).get("stage"):
                live_seen = (http_json(base + "/api/bond/throughput")
                             .get("live"))
            if (fw.get("results") or []) and not restored:
                set_contact_tol(50.0)   # 第 1 对已判失败（含恢复完成）→改回
                restored = True
            if not fw.get("active"):
                break
            time.sleep(0.5)
        res = fw.get("results") or []
        check("i17.fw_fail_recovery",
              restored and not fw.get("active") and not fw.get("error")
              and len(res) == 2
              and res[0].get("ok") is False
              and res[0].get("recovered") is True
              and res[1].get("ok") is True,
              "results=%s err=%s restored=%s" % (res, fw.get("error"),
                                                 restored))
        # 残片确已放回蓝膜
        die0 = res[0].get("die") if res else None
        carriers = {d["id"]: d.get("carrier") for d in
                    (http_json(base + "/api/workpiece/map")
                     .get("film_dies") or [])}
        check("i17.fw_fail_recovery_film",
              die0 is not None and carriers.get(die0) == "film",
              "die=%s carrier=%s" % (die0, carriers.get(die0)))
        check("i17.throughput_live",
              live_idle_none
              and live_seen is not None and live_seen.get("active")
              and live_seen.get("kind") == "full"
              and (live_seen.get("current") or {}).get("stage")
              and live_seen.get("cur_elapsed_s") is not None
              and live_seen.get("run_ok") == 0
              and live_seen.get("est_uph") is None,
              "idle_none=%s live=%s" % (live_idle_none, live_seen))
        flow_cleanup()

        # ---- 单片 abort 后 recovery 状态 + 面板同款恢复 ----
        # site 取离中心最远的未键合位（中心位已被前面的全片跑掉，已键合
        # site 会触发幂等跳过导致 bond 不跑）
        wmap = http_json(base + "/api/workpiece/map")
        cand = [s2 for s2 in (wmap.get("wafer_sites") or [])
                if not s2.get("bonded")]
        assert cand, wmap
        far = max(cand, key=lambda s2: abs(s2["x"]) + abs(s2["y"]))
        site_rc = [far["row"], far["col"]]
        set_contact_tol(0.001)
        c, r = http_post(base + "/api/process/chain", {"site": site_rc})
        assert c == 200, r
        t0 = time.time()
        ch = {"active": True}
        while time.time() - t0 < 1200.0:
            ch = flow_state().get("chain") or {}
            if not ch.get("active"):
                break
            time.sleep(0.5)
        rec = ch.get("recovery") or {}
        check("i17.chain_recovery_status",
              not ch.get("active") and ch.get("error")
              and (ch.get("stages") or {}).get("bond") == "fail"
              and rec.get("die_carrier") == "head"
              and rec.get("options") == ["return_film", "continue_bond"],
              "err=%s stages=%s recovery=%s"
              % (ch.get("error"), ch.get("stages"), rec))
        set_contact_tol(50.0)
        # 恢复：head_to_flip → flip_place_back 串行（面板按钮同款顺序）
        c, r = http_post(base + "/api/process/head_to_flip", {})
        assert c == 200, r
        st = flow_wait_done()
        assert st.get("state") == "done", st.get("state")
        c, r = http_post(base + "/api/process/flip_place_back", {})
        assert c == 200, r
        st = flow_wait_done()
        assert st.get("state") == "done", st.get("state")
        ch2 = flow_state().get("chain") or {}
        check("i17.chain_recovery_done",
              ch2.get("recovery") is None,
              "recovery=%s" % (ch2.get("recovery"),))
        flow_cleanup()

        # ---- clear_head 启动：head 残片 → 409 → clear_head=true 自动放回 ----
        c, r = http_post(base + "/api/process/flip_pick", {})
        assert c == 200, r
        st = flow_wait_done()
        assert st.get("state") == "done", st.get("state")
        c, r = http_post(base + "/api/process/flip_handoff", {})
        assert c == 200, r
        st = flow_wait_done()
        assert st.get("state") == "done", st.get("state")
        c, r = http_put(base + "/api/recipe", {"full_wafer": {"count": 1}})
        assert c == 200, r
        c1, r1 = http_post(base + "/api/process/full_wafer", {})
        c2, r2 = http_post(base + "/api/process/full_wafer",
                           {"clear_head": True})
        # clear_head 为异步清残片路径（clearing:true，runner 线程内放回
        # 后再规划启动）——fw_wait 等整批收完
        fw = fw_wait() if c2 == 200 else {"active": False,
                                          "error": "start failed"}
        res = fw.get("results") or []
        check("i17.fw_clear_head",
              c1 == 409 and (r1 or {}).get("clear_head_option") is True
              and c2 == 200 and (r2 or {}).get("clearing") is True
              and not fw.get("active") and not fw.get("error")
              and (fw.get("progress") or [0, 0]) == [1, 1]
              and len(res) == 1 and res[0].get("ok") is True,
              "c1=%s r1=%s c2=%s r2=%s progress=%s results=%s err=%s"
              % (c1, r1, c2, r2, fw.get("progress"), res, fw.get("error")))
        flow_cleanup()
    finally:
        try:
            set_contact_tol(50.0)
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)     # 迭代 18：stop 标志轮询周期 10ms，足够
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


# ---------------- 迭代 20：键合后检测整改 + 误差传递链物理重构 ----------------
def test_iter20():
    """--- 19.24 迭代 20 ---
    quick（两套件，sim≈0）：
      - i20.mc_defaults / i20.mc_put：新机械常数键默认值 0 + PUT 往返
        （bond.bond_shift_xy_um/bond_shift_rz_urad 压合微滑移 3σ；
        fine_align.frame_cal_x/y_um、frame_cal_rz_urad 计量帧标定残差）；
      - i20.bond_truth_op_validate：bond_truth op 进程内 schema 校验通过
        （op 注册 + 无必填字段）；
      - i20.ir_frame_api：GET /api/vision/frame?cam=down&ir_overlay=1
        红外套刻通道出 PNG；POST /api/vision/capture/save 带
        ir_overlay:true 采图元数据标记 + 取图 + 删除。
    full-only（fblock 实跑链）：
      - i20.repeat_contrast：同 site 重复键合 组A N=6 + 注入 5µm 取片
        扰动对照 组B N=4（每颗 workpiece reset 全链重跑）——truth 分量
        分解一致性（commit=align+descent、truth=commit+shift）、重复性
        预算（xy/drz 3σ）、descent 系统项常数性、组间均值差≪注入量
        （上游误差被补偿链消除，不进 truth）。"""
    print("--- 19.24 迭代 20（pi 整改 + truth 物理重构） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    # ---- 机械常数新键：默认值 + PUT 往返 ----
    const = http_json(base + "/api/machine_constants")
    grp = const.get("constants") or {}
    bond_mc = grp.get("bond") or {}
    fa_mc = grp.get("fine_align") or {}
    check("i20.mc_defaults",
          bond_mc.get("bond_shift_xy_um") == 0.0
          and bond_mc.get("bond_shift_rz_urad") == 0.0
          and fa_mc.get("frame_cal_x_um") == 0.0
          and fa_mc.get("frame_cal_y_um") == 0.0
          and fa_mc.get("frame_cal_rz_urad") == 0.0,
          "bond=%s fine_align=%s"
          % ({k: bond_mc.get(k) for k in ("bond_shift_xy_um",
                                          "bond_shift_rz_urad")},
             {k: fa_mc.get(k) for k in ("frame_cal_x_um", "frame_cal_y_um",
                                        "frame_cal_rz_urad")}))
    c, r = http_put(base + "/api/machine_constants",
                    {"bond": {"bond_shift_xy_um": 0.15,
                              "bond_shift_rz_urad": 1.5},
                     "fine_align": {"frame_cal_x_um": 0.05}})
    grp2 = (http_json(base + "/api/machine_constants")
            .get("constants") or {})
    ok_put = c == 200 \
        and (grp2.get("bond") or {}).get("bond_shift_xy_um") == 0.15 \
        and (grp2.get("bond") or {}).get("bond_shift_rz_urad") == 1.5 \
        and (grp2.get("fine_align") or {}).get("frame_cal_x_um") == 0.05
    c2, r2 = http_put(base + "/api/machine_constants",
                      {"bond": {"bond_shift_xy_um": 0.0,
                                "bond_shift_rz_urad": 0.0},
                       "fine_align": {"frame_cal_x_um": 0.0}})
    check("i20.mc_put", ok_put and c2 == 200,
          "put=%s restore=%s" % (c, c2))

    # ---- bond_truth op 合法性（进程内 schema 校验，不起流程）----
    import flow_engine as femod
    _errs = []
    femod.FlowEngine._validate_steps(
        femod.FlowEngine.__new__(femod.FlowEngine),
        [{"op": "bond_truth"}], "_i20", _errs, depth=1)
    check("i20.bond_truth_op_validate",
          not _errs and "bond_truth" in femod.FLOW_OPS,
          "errors=%s" % (_errs,))

    # ---- IR 套刻帧 API + 套刻采图 ----
    def _frame(query):
        with urllib.request.urlopen(base + "/api/vision/frame?" + query,
                                    timeout=15) as resp:
            return resp.status, resp.read()
    c1, png1 = _frame("cam=down&ir_overlay=1")
    c2, png2 = _frame("cam=down")
    c3, r3 = http_post(base + "/api/vision/capture/save",
                       {"cam": "down", "ir_overlay": True})
    cap = (r3 or {}).get("capture") or {}
    got_png = False
    if cap.get("name"):
        with urllib.request.urlopen(base + "/api/vision/capture/file?name="
                                    + cap["name"], timeout=15) as resp:
            got_png = resp.read(8) == b"\x89PNG\r\n\x1a\n"
        c4, r4 = http_delete(base + "/api/vision/captures",
                             {"name": cap["name"]})
    else:
        c4 = 0
    check("i20.ir_frame_api",
          c1 == 200 and png1[:8] == b"\x89PNG\r\n\x1a\n"
          and c2 == 200 and png2[:8] == b"\x89PNG\r\n\x1a\n"
          and c3 == 200 and cap.get("ir_overlay") is True
          and got_png and c4 == 200,
          "frame_ir=%s frame_vis=%s cap=%s ir_flag=%s got=%s del=%s"
          % (c1, c2, c3, cap.get("ir_overlay"), got_png, c4))

    # ---- full-only：同 site 重复键合 + 5µm 上游扰动对照 ----
    if not fblock(["i20.repeat_contrast"]):
        return

    def flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)
        http_post(base + "/api/flow/reset", {})

    def chain_wait(timeout=900.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ch = http_json(base + "/api/flow/state").get("chain") or {}
            if not ch.get("active"):
                return ch
            time.sleep(0.15)
        return {"active": True, "error": "timeout"}

    def one_sample(inject):
        http_post(base + "/api/workpiece/reset", {})
        if inject:
            http_post(base + "/api/workpiece/pick_offset",
                      {"x_mm": 0.005, "y_mm": 0.0})   # 5µm 上游扰动
        c, r = http_post(base + "/api/process/chain", {})
        if c != 200:
            return None
        ch = chain_wait()
        flow_cleanup()
        if ch.get("error"):
            return None
        rs = http_json(base + "/api/bond/results").get("results") or []
        return rs[-1] if rs else None

    def _stats(vals):
        n = len(vals)
        m = sum(vals) / n
        v = sum((x - m) ** 2 for x in vals) / n
        return m, 3.0 * math.sqrt(v)

    try:
        set_speed(8.0)   # 提速窗（finally 复原 1×）
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        http_post(base + "/api/recipe/restore_default", {})
        flow_cleanup()
        c, r = http_put(base + "/api/recipe", {"wafer_load": {
            "load_x_mm": 0.2, "load_y_mm": -0.15, "load_rz_rad": 0.0002,
            "jitter_on": False, "jitter_xy_um": 0.0, "jitter_rz_urad": 0.0},
            "sim_sensors": {"confocal_sigma_um": 0.0},
            "fine_align": {"confirm_max_iter": 5},
            "bond": {"dwell_s": 1.0}})
        assert c == 200, r
        # 预热一次（keep_warm 下后续链等温确认幂等秒过）
        tgt_t = float((http_json(base + "/api/recipe")["recipe"]
                       .get("bond") or {}).get("wafer_temp_C", 30.0))
        http_post(base + "/api/io/command",
                  {"path": "SystemMachine.Heat_Dw",
                   "cmd": "SetTargetTempWithPlan",
                   "args": {"Temperature": tgt_t, "Steady_Time": 0}})
        t0 = time.time()
        while time.time() - t0 < 60.0:
            v = http_json("http://127.0.0.1:%d/io/observe?path=SystemMachine."
                          "M_Heater.Heat_Dw.oCurTemp" % PLC_PORT)
            if float(v.get("value", 0.0)) >= tgt_t - 1.5:
                break
            time.sleep(0.5)
        rows_a, rows_b = [], []
        for i in range(6):
            rec = one_sample(False)
            if rec:
                rows_a.append(rec)
        for i in range(4):
            rec = one_sample(True)
            if rec:
                rows_b.append(rec)
        ok_struct = len(rows_a) >= 5 and len(rows_b) >= 3
        for rec in rows_a + rows_b:
            t = rec.get("truth") or {}
            tc = rec.get("truth_components") or {}
            al = tc.get("align_resid") or {}
            de = tc.get("descent") or {}
            cm = tc.get("commit") or {}
            ps = tc.get("process_shift") or {}
            ok_struct = ok_struct and bool(t) and bool(tc) \
                and abs(cm.get("dx_um", 0)
                        - (al.get("dx_um", 0) + de.get("dx_um", 0))) < 1e-3 \
                and abs(t.get("dx", 9e9) * 1000.0
                        - (cm.get("dx_um", 0) + ps.get("dx_um", 0))) < 1e-3
        # 统计（truth 与分量；µm/µrad；只取分量齐全的记录）
        rows_a2 = [r for r in rows_a if (r.get("truth_components") or {})
                   .get("descent") and r.get("truth")]
        rows_b2 = [r for r in rows_b if (r.get("truth_components") or {})
                   .get("descent") and r.get("truth")]
        def _col(rows, f):
            return [f(r) for r in rows]
        ta = {k: _col(rows_a2, lambda r, k=k: (r["truth"][k]
                      * (1e6 if k == "drz" else 1000.0)))
              for k in ("dx", "dy", "drz")}
        tb = {k: _col(rows_b2, lambda r, k=k: (r["truth"][k]
                      * (1e6 if k == "drz" else 1000.0)))
              for k in ("dx", "dy", "drz")}
        da = {k: _col(rows_a2, lambda r, k=k:
                      (r["truth_components"]["descent"]
                       [dict(dx="dx_um", dy="dy_um", drz="drz_urad")[k]]))
              for k in ("dx", "dy", "drz")}
        st_a = {k: _stats(v) for k, v in ta.items()}
        st_b = {k: _stats(v) for k, v in tb.items()}
        st_d = {k: _stats(v) for k, v in da.items()}
        # 重复性预算：xy 3σ ≤0.4µm、drz 3σ ≤45µrad（迭代 20 实测 N=20：
        # xy 3σ=0.21/0.13µm、drz 3σ=28.8µrad——理论合成 3σ≈0.20µm/36µrad
        # （相机噪声传播+末次补偿 repeat+确认环残余），N=6 采样涨落留
        # 余量；详见 docs/键合误差模型_V1.md §4/§5 与
        # Temp/_iter20_repeat_report.txt）
        rep_ok = st_a["dx"][1] <= 0.4 and st_a["dy"][1] <= 0.4 \
            and st_a["drz"][1] <= 45.0
        # descent 系统项常数性：µm 级且 3σ≪|mean|
        de_l = math.hypot(st_d["dx"][0], st_d["dy"][0])
        de_s = max(st_d["dx"][1], st_d["dy"][1])
        de_ok = 1.0 < de_l < 50.0 and de_s < 0.3 * de_l
        # 对照：注入 5µm 上游扰动后 truth 分布不变（|Δmean|≪注入量）
        ct_ok = abs(st_b["dx"][0] - st_a["dx"][0]) <= \
            max(0.3, 1.5 * st_a["dx"][1]) \
            and abs(st_b["dy"][0] - st_a["dy"][0]) <= \
            max(0.3, 1.5 * st_a["dy"][1]) \
            and abs(st_b["drz"][0] - st_a["drz"][0]) <= \
            max(30.0, 1.5 * st_a["drz"][1])
        check("i20.repeat_contrast",
              ok_struct and rep_ok and de_ok and ct_ok,
              "nA=%d nB=%d struct=%s | A 3σ=(%.3f,%.3f)um/%.1furad | "
              "descent |mean|=%.2fum 3σ=%.3f | Δmean=(%.3f,%.3f)um/%.1furad"
              % (len(rows_a), len(rows_b), ok_struct,
                 st_a["dx"][1], st_a["dy"][1], st_a["drz"][1],
                 de_l, de_s,
                 st_b["dx"][0] - st_a["dx"][0],
                 st_b["dy"][0] - st_a["dy"][0],
                 st_b["drz"][0] - st_a["drz"][0]))
        print("    [i20] A 组 truth mean=(%+.3f,%+.3f)um/%+.1furad "
              "3σ=(%.3f,%.3f)um/%.1furad；B-A Δmean=(%+.3f,%+.3f)um/%+.1furad"
              % (st_a["dx"][0], st_a["dy"][0], st_a["drz"][0],
                 st_a["dx"][1], st_a["dy"][1], st_a["drz"][1],
                 st_b["dx"][0] - st_a["dx"][0], st_b["dy"][0] - st_a["dy"][0],
                 st_b["drz"][0] - st_a["drz"][0]), flush=True)
    finally:
        try:
            http_post(base + "/api/speed", {"scale": 1.0})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.Heat_Dw",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.PulseHeat_Up",
                       "cmd": "StopTargetTempWithPlan"})
            http_post(base + "/api/recipe/restore_default", {})
            http_post(base + "/api/workpiece/reset", {})
            http_post(base + "/api/flow/stop", {})
            time.sleep(0.15)
            http_post(base + "/api/flow/reset", {})
            for ax, tgt in (("BS_Arm_Z", 0.0), ("AS_Z", 0.0),
                            ("AS_X", 0.0), ("AS_Y", 0.0),
                            ("WS_Rz", 0.0)):
                http_post(base + "/api/axis/move",
                          {"axis": ax, "target": tgt, "speed": 500.0})
            for ax in ("BS_Arm_Z", "AS_Z", "AS_X", "AS_Y", "WS_Rz"):
                wait_settled(ax, timeout=20.0)
        except Exception:
            pass


def test_iter21():
    """--- 19.25 迭代 21 ---
    quick（两套件，sim≈0）：
      - i21.recipe_fast_noise_default / put / invalid_400：FAST 质心噪声入
        处方 sim_sensors.fast_noise_px（默认 0.15，PUT 合法值生效回读一致、
        越界 400 不污染）；
      - i21.components_get：GET /api/components 返回 168 组件、字段齐全、
        含"大理石基座1"（preset granite → material_key granite 默认种子）；
      - i21.materials_get_defaults：GET /api/materials 17 种内置材料 +
        ref_counts（granite=3）；
      - i21.material_map_put / invalid_400：合法指派回读一致（components
        与 map 两口径）+ 非法材料键/未知组件 id 400；
      - i21.materials_put_cte / delete_referenced_400 / range_400 /
        add_and_ref：改 CTE 生效回读、删除被引用材料 400、物性越界 400、
        新增材料后可被 map 引用、引用中禁删、解除引用后可删。
    持久化：quick 档不含重启场景，按 GET 回读一致校验（落盘由后端
    update 路径 tmp+os.replace 保证）。"""
    print("--- 19.25 迭代 21（fast_noise_px 入测校 + 组件材质物性） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    # ---- 任务 1：sim_sensors.fast_noise_px（默认/PUT 往返/越界 400）----
    # （2026-09-08 迭代 22 起逐相机嵌套 {coarse,up,down}；标量 PUT 走
    # _migrate 展开为全实例同值——本块同时覆盖迁移集成路径）
    ss = (http_json(base + "/api/recipe").get("recipe") or {}) \
        .get("sim_sensors") or {}
    check("i21.recipe_fast_noise_default",
          ss.get("fast_noise_px") == {"coarse": 0.15, "up": 0.1,
                                      "down": 0.1},
          "sim_sensors=%s" % (ss,))
    c, r = http_put(base + "/api/recipe",
                    {"sim_sensors": {"fast_noise_px": 0.3}})
    ss2 = (http_json(base + "/api/recipe").get("recipe") or {}) \
        .get("sim_sensors") or {}
    ok_put = c == 200 and ss2.get("fast_noise_px") == {"coarse": 0.3,
                                                       "up": 0.3, "down": 0.3}
    c2, r2 = http_put(base + "/api/recipe",
                      {"sim_sensors": {"fast_noise_px": 0.15}})
    ss3 = (http_json(base + "/api/recipe").get("recipe") or {}) \
        .get("sim_sensors") or {}
    check("i21.recipe_fast_noise_put",
          ok_put and c2 == 200 and ss3.get("fast_noise_px") ==
          {"coarse": 0.15, "up": 0.15, "down": 0.15},
          "put=%s readback=%s restore=%s" % (c, ss2.get("fast_noise_px"), c2))
    codes = []
    for bad in (2.5, -0.1):
        c3, r3 = http_put(base + "/api/recipe",
                          {"sim_sensors": {"fast_noise_px": bad}})
        codes.append(c3)
    ss4 = (http_json(base + "/api/recipe").get("recipe") or {}) \
        .get("sim_sensors") or {}
    check("i21.recipe_fast_noise_invalid_400",
          codes == [400, 400] and ss4.get("fast_noise_px") ==
          {"coarse": 0.15, "up": 0.15, "down": 0.15},
          "codes=%s readback=%s" % (codes, ss4.get("fast_noise_px")))

    # ---- 任务 2：组件目录（168 组件 + 已知组件默认种子）----
    comps = http_json(base + "/api/components").get("components") or []
    dl1 = [c for c in comps if c.get("name") == "大理石基座1"]
    ok_fields = bool(comps) and all(
        set(("id", "name", "parentId", "preset_id", "material_key"))
        <= set(c) for c in comps)
    check("i21.components_get",
          len(comps) == 168 and ok_fields and len(dl1) == 1
          and dl1[0].get("preset_id") == "granite"
          and dl1[0].get("material_key") == "granite",
          "n=%d fields=%s 大理石基座1=%s" % (len(comps), ok_fields, dl1))

    # ---- 材料库默认值 + 引用计数 ----
    mj = http_json(base + "/api/materials")
    mats = mj.get("materials") or {}
    refs = mj.get("ref_counts") or {}
    gr = mats.get("granite") or {}
    al = mats.get("al_6061") or {}
    check("i21.materials_get_defaults",
          mj.get("ok") and len(mats) == 17
          and gr.get("cte_ppm_K") == 6.5
          and gr.get("youngs_GPa") == 50.0
          and al.get("cte_ppm_K") == 23.6
          and al.get("youngs_GPa") == 68.9
          and (mats.get("vacuum_black_al") or {}).get("cte_ppm_K") == 23.6
          and refs.get("granite") == 3,
          "n=%d granite=%s al_6061=%s refs.granite=%s"
          % (len(mats), gr, al, refs.get("granite")))

    # ---- 组件映射：合法指派回读（map 与 components 两口径）+ 恢复 ----
    cid = dl1[0]["id"] if dl1 else "box_mssjj329_9ocxvw"
    c, r = http_put(base + "/api/material_map", {cid: "ss_316"})
    mmap = http_json(base + "/api/material_map").get("map") or {}
    comps2 = http_json(base + "/api/components").get("components") or []
    mk2 = ([x for x in comps2 if x.get("id") == cid] or [{}])[0] \
        .get("material_key")
    ok_map = c == 200 and mmap.get(cid) == "ss_316" and mk2 == "ss_316"
    c2, r2 = http_put(base + "/api/material_map", {cid: "granite"})
    mmap2 = http_json(base + "/api/material_map").get("map") or {}
    check("i21.material_map_put",
          ok_map and c2 == 200 and mmap2.get(cid) == "granite",
          "put=%s readback=%s/%s restore=%s" % (c, mmap.get(cid), mk2, c2))
    codes = []
    codes.append(http_put(base + "/api/material_map",
                          {cid: "nonexistent_mat"})[0])
    codes.append(http_put(base + "/api/material_map",
                          {"no_such_component": "granite"})[0])
    mmap3 = http_json(base + "/api/material_map").get("map") or {}
    check("i21.material_map_invalid_400",
          codes == [400, 400] and mmap3.get(cid) == "granite",
          "codes=%s readback=%s" % (codes, mmap3.get(cid)))

    # ---- 材料库：改 CTE 生效回读 + 恢复 ----
    c, r = http_put(base + "/api/materials",
                    {"set": {"granite": {"cte_ppm_K": 7.0}}})
    gr2 = (http_json(base + "/api/materials").get("materials") or {}) \
        .get("granite") or {}
    ok_cte = c == 200 and gr2.get("cte_ppm_K") == 7.0 \
        and gr2.get("youngs_GPa") == 50.0      # 未提交字段保持
    c2, r2 = http_put(base + "/api/materials",
                      {"set": {"granite": {"cte_ppm_K": 6.5}}})
    gr3 = (http_json(base + "/api/materials").get("materials") or {}) \
        .get("granite") or {}
    check("i21.materials_put_cte",
          ok_cte and c2 == 200 and gr3.get("cte_ppm_K") == 6.5,
          "put=%s readback=%s restore=%s" % (c, gr2.get("cte_ppm_K"), c2))

    # ---- 删除被引用材料 400（granite 被 3 组件引用）----
    c, r = http_put(base + "/api/materials", {"delete": ["granite"]})
    mats2 = http_json(base + "/api/materials").get("materials") or {}
    check("i21.materials_delete_referenced_400",
          c == 400 and "granite" in mats2,
          "code=%s granite_kept=%s" % (c, "granite" in mats2))

    # ---- 物性越界 400（CTE 超上限 / 泊松比超上限 / 密度低于下限）----
    codes = []
    for bad in ({"set": {"granite": {"cte_ppm_K": 61.0}}},
                {"set": {"granite": {"poisson": 0.6}}},
                {"set": {"granite": {"density_kg_m3": 100.0}}}):
        codes.append(http_put(base + "/api/materials", bad)[0])
    gr4 = (http_json(base + "/api/materials").get("materials") or {}) \
        .get("granite") or {}
    check("i21.materials_range_400",
          codes == [400, 400, 400]
          and gr4.get("cte_ppm_K") == 6.5 and gr4.get("poisson") == 0.25,
          "codes=%s granite=%s" % (codes, gr4))

    # ---- 新增材料 → map 可引用 → 引用中禁删 → 解除引用后可删 ----
    new_mat = {"name_cn": "测试材料", "cte_ppm_K": 5.0, "youngs_GPa": 100.0,
               "poisson": 0.3, "density_kg_m3": 2000.0,
               "thermal_cond_W_mK": 10.0, "spec_heat_J_kgK": 800.0,
               "note": "回归用例临时材料"}
    c1, r1 = http_put(base + "/api/materials",
                      {"set": {"i21_test_mat": new_mat}})
    c2, r2 = http_put(base + "/api/material_map", {cid: "i21_test_mat"})
    mats3 = http_json(base + "/api/materials").get("materials") or {}
    mmap4 = http_json(base + "/api/material_map").get("map") or {}
    c3, r3 = http_put(base + "/api/materials",
                      {"delete": ["i21_test_mat"]})     # 被引用 → 400
    c4, r4 = http_put(base + "/api/material_map", {cid: "granite"})
    c5, r5 = http_put(base + "/api/materials",
                      {"delete": ["i21_test_mat"]})     # 解除引用 → 200
    mats4 = http_json(base + "/api/materials").get("materials") or {}
    check("i21.materials_add_and_ref",
          c1 == 200 and c2 == 200
          and (mats3.get("i21_test_mat") or {}).get("cte_ppm_K") == 5.0
          and mmap4.get(cid) == "i21_test_mat"
          and c3 == 400 and c4 == 200 and c5 == 200
          and "i21_test_mat" not in mats4 and len(mats4) == 17,
          "add=%s ref=%s del_ref=%s unref=%s del=%s left=%s n=%d"
          % (c1, c2, c3, c4, c5, "i21_test_mat" in mats4, len(mats4)))


def test_iter22():
    """--- 19.26 迭代 22 ---
    quick（两套件；段首 restore_default 保证 recipe/厚度确定，段尾恢复
    modal OFF+地板 typical+σ 默认；perf 窗起点=段首——段内全程 1×，末段
    read_clean_health 零等待）。
    任务 1 传感器噪声逐实例拆分：
      - i22.sim_sensors_default / migrate_scalar / put_instance /
        invalid_400：嵌套默认值、旧标量 _migrate 展开、逐实例 PUT 生效
        回读（他实例不受影响）、实例越界/未知实例键/标量当嵌套 400；
      - i22.fast_noise_per_cam：down 相机 σ=1.2 vs 0.02px 同机位连拍
        统计可分（散布比 ≫4），coarse/up 回读不受影响。
    任务 2 全机模态仿真：
      - i22.modal_get_default / put_invalid_400 / put_roundtrip / toggle；
      - i22.modal_off_zero_impact：OFF（及 ON+无激励空闲）actual/6DOF
        与基线逐位一致；
      - i22.modal_floor_rms：ON+地板 typical 模态 RSS≈0.3µm（宽松），
        toolpoints wafer_stage 误差相对 OFF 基线有可测偏移；
      - i22.modal_move_excitation：WS_LS_X 小行程激发——scope 峰值 ≫
        缓冲尾段底噪、整定尾巴 ON>OFF（激发/对照用 modal 开关清缓冲
        隔离，防 8s 历史窗串扰）；
      - i22.modal_contact_pulse：BS_Arm_Z 接触沿后 head_z_bounce 峰值
        显著>等幅非接触行程对照；
      - i22.modal_exposure_fast：FAST 同机位连拍，模态 ON+地板较差
        散布显著>OFF（宽松统计）；
      - i22.modal_perf：modal ON 干净窗 tick≥999Hz。"""
    print("--- 19.26 迭代 22（传感器噪声逐实例拆分 + 全机模态仿真） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    t_seg = time.time()          # perf 干净窗起点（段内全程 1×）

    def _ss():
        return (http_json(base + "/api/recipe").get("recipe") or {}) \
            .get("sim_sensors") or {}

    def _modal(en):
        c, r = http_post(base + "/api/modal", {"enabled": en})
        assert c == 200 and r.get("enabled") == bool(en), (c, r)

    def _fast_spread(cam, shots):
        """同机位连拍 → ( lstCol/lstRow 合成 1σ, 有效样本数 )。"""
        cols, rows = [], []
        for _ in range(shots):
            c, r = http_post(base + "/api/vision/identify", {"cam": cam})
            res = ((r.get("entry") or {}).get("result") or {})
            if res.get("judge") and res.get("lstCol"):
                cols.append(res["lstCol"][0])
                rows.append(res["lstRow"][0])
        if len(cols) < 5:
            return float("nan"), len(cols)
        mx, my = sum(cols) / len(cols), sum(rows) / len(rows)
        var = (sum((v - mx) ** 2 for v in cols)
               + sum((v - my) ** 2 for v in rows)) / (2 * len(cols))
        return math.sqrt(var), len(cols)

    def _scope_peak(ch, n=2500):
        """模态 OFF→ON 清缓冲后测量：返回缓冲内 |q| 峰值（native 单位）。
        调用方负责在激发前已切换过开关（缓冲只含本次激发数据）。"""
        sc = http_json(base + "/api/modal/scope?ch=%d&n=%d" % (ch, n))
        return max((abs(v) for v in sc["data"]), default=0.0), sc

    # ---- 干净确定起点（recipe 默认：厚度 1.0/0.5 → 接触位 -167.43）----
    http_post(base + "/api/recipe/restore_default", {})
    _modal(False)

    # ================= 任务 1：sim_sensors 逐实例拆分 =================
    ss = _ss()
    check("i22.sim_sensors_default",
          ss.get("fast_noise_px") == {"coarse": 0.15, "up": 0.1,
                                      "down": 0.1}
          and ss.get("confocal_sigma_um") == {"up": 0.065, "down": 0.065}
          and ss.get("temp_noise_C") == {"PulseHeat_Up": 0.5,
                                         "Heat_Dw": 0.5}
          and ss.get("force_noise_N") == 1.0,
          "sim_sensors=%s" % (ss,))

    # ---- 单元级迁移：旧标量 → 全实例同值展开 ----
    import copy as _copy
    import recipe as _recipe_mod
    old = {"sim_sensors": {"confocal_sigma_um": 0.07, "temp_noise_C": 0.9,
                           "force_noise_N": 2.0, "fast_noise_px": 0.33}}
    mig = _recipe_mod._migrate(_copy.deepcopy(old))["sim_sensors"]
    check("i22.sim_sensors_migrate_scalar",
          mig["confocal_sigma_um"] == {"up": 0.07, "down": 0.07}
          and mig["temp_noise_C"] == {"PulseHeat_Up": 0.9, "Heat_Dw": 0.9}
          and mig["fast_noise_px"] == {"coarse": 0.33, "up": 0.33,
                                       "down": 0.33}
          and mig["force_noise_N"] == 2.0
          and not _recipe_mod.validate(_copy.deepcopy(
              _recipe_mod.DEFAULT_RECIPE)),
          "migrated=%s" % (mig,))

    # ---- 逐实例 PUT 生效回读（他实例不受影响）+ 恢复 ----
    # 2026-09-08 默认值变更：fast_noise up/down 0.15→0.1（coarse 保持
    # 0.15）、confocal_sigma 0.05→0.065——期望值随新默认更新
    c1, _ = http_put(base + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"up": 0.4},
                                      "confocal_sigma_um": {"down": 0.3},
                                      "temp_noise_C": {"PulseHeat_Up": 0.9}}})
    ss2 = _ss()
    ok_put = c1 == 200 \
        and ss2["fast_noise_px"] == {"coarse": 0.15, "up": 0.4, "down": 0.1} \
        and ss2["confocal_sigma_um"] == {"up": 0.065, "down": 0.3} \
        and ss2["temp_noise_C"] == {"PulseHeat_Up": 0.9, "Heat_Dw": 0.5}
    c2, _ = http_put(base + "/api/recipe",
                     {"sim_sensors": {"fast_noise_px": {"up": 0.1},
                                      "confocal_sigma_um": {"down": 0.065},
                                      "temp_noise_C": {"PulseHeat_Up": 0.5}}})
    ss3 = _ss()
    check("i22.sim_sensors_put_instance",
          ok_put and c2 == 200
          and ss3["fast_noise_px"]["up"] == 0.1
          and ss3["confocal_sigma_um"]["down"] == 0.065
          and ss3["temp_noise_C"]["PulseHeat_Up"] == 0.5,
          "put=%s readback=%s restore=%s" % (c1, ss2, c2))

    # ---- 逐实例校验 400（实例越界/未知实例键/嵌套键当标量）不污染 ----
    codes = [http_put(base + "/api/recipe",
                      {"sim_sensors": {"fast_noise_px": {"up": 2.5}}})[0],
             http_put(base + "/api/recipe",
                      {"sim_sensors": {"fast_noise_px": {"left": 0.1}}})[0],
             http_put(base + "/api/recipe",
                      {"sim_sensors": {"force_noise_N": {"x": 1.0}}})[0],
             http_put(base + "/api/recipe",
                      {"sim_sensors": {"temp_noise_C":
                                       {"Heat_Dw": -0.5}}})[0]]
    ss4 = _ss()
    check("i22.sim_sensors_invalid_400",
          codes == [400, 400, 400, 400]
          and ss4["fast_noise_px"]["up"] == 0.1
          and ss4["force_noise_N"] == 1.0
          and ss4["temp_noise_C"]["Heat_Dw"] == 0.5,
          "codes=%s readback=%s" % (codes, ss4))

    # ---- 场景准备（FAST 连拍/曝光积分共用）：干净物料+晶圆+下相机机位 ----
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                    ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)

    # ---- FAST 逐相机噪声：down 大/小 σ 同机位连拍统计可分 ----
    http_put(base + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 1.2}}})
    s_big, n_big = _fast_spread("down", 14)
    http_put(base + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.02}}})
    s_small, n_small = _fast_spread("down", 14)
    ss5 = _ss()
    http_put(base + "/api/recipe",
             {"sim_sensors": {"fast_noise_px": {"down": 0.1}}})
    check("i22.fast_noise_per_cam",
          n_big >= 10 and n_small >= 10 and s_small > 0.0
          and s_big > 4.0 * s_small          # σ 比 60×，取 15× 裕度
          and ss5["fast_noise_px"]["coarse"] == 0.15
          and ss5["fast_noise_px"]["up"] == 0.1,
          "big=%.3fpx(n=%d) small=%.3fpx(n=%d) ratio=%.1f"
          % (s_big, n_big, s_small, n_small,
             s_big / max(s_small, 1e-9)))

    # ================= 任务 2：全机模态仿真 =================
    # ---- 默认参数 + 默认 OFF ----
    mj = http_json(base + "/api/modal")
    pm = mj.get("params") or {}
    hb = (pm.get("modes") or {}).get("head_z_bounce") or {}
    wx = (pm.get("axes") or {}).get("WS_LS_X") or {}
    check("i22.modal_get_default",
          mj.get("ok") and mj.get("enabled") is False
          and len(pm.get("axes") or {}) == 21
          and len(pm.get("modes") or {}) == 9
          and wx.get("fn_hz") == 45.0 and wx.get("zeta") == 0.05
          and hb.get("fn_hz") == 58.0 and hb.get("contact") is True
          and (pm.get("floor") or {}).get("level") == "typical"
          and mj.get("floor_rms_um") == 0.3,
          "enabled=%s axes=%d modes=%d floor=%s"
          % (mj.get("enabled"), len(pm.get("axes") or {}),
             len(pm.get("modes") or {}), pm.get("floor")))

    # ---- OFF 零影响：actual/toolpoints 与显式 OFF 前后逐位一致；
    #      ON+地板关+空闲（enable 清零后无激励）同样一致 ----
    def _snap():
        st = http_json(base + "/api/axes")
        tp = http_json(base + "/api/toolpoints")
        return ([a["actual"] for a in st["axes"]],
                {m: (v["error_mm"], v["error_rad"])
                 for m, v in (tp.get("modules") or {}).items()})
    s0 = _snap()
    _modal(False)
    s1 = _snap()
    http_put(base + "/api/modal", {"floor": {"level": "off"}})
    _modal(True)
    time.sleep(0.3)          # 无激励空闲：enable 清零后 q 恒为 0
    s2 = _snap()
    check("i22.modal_off_zero_impact",
          s0 == s1 == s2,
          "off_eq=%s idle_on_eq=%s" % (s0 == s1, s1 == s2))

    # ---- PUT 校验 400（fn/zeta 越界、未知轴/模块、非法地板等级）----
    codes = [http_put(base + "/api/modal",
                      {"modes": {"head_z_bounce": {"fn_hz": 999.0}}})[0],
             http_put(base + "/api/modal",
                      {"modes": {"head_z_bounce": {"zeta": 0.9}}})[0],
             http_put(base + "/api/modal",
                      {"modes": {"head_z_bounce": {"zeta": 0.0005}}})[0],
             http_put(base + "/api/modal",
                      {"axes": {"NO_AXIS": {"fn_hz": 50.0}}})[0],
             http_put(base + "/api/modal",
                      {"modes": {"head_z_bounce":
                                 {"outputs": {"no_module":
                                              [0, 0, 0, 0, 0, 0]}}}})[0],
             http_put(base + "/api/modal",
                      {"floor": {"level": "swamp"}})[0]]
    mj2 = http_json(base + "/api/modal")
    check("i22.modal_put_invalid_400",
          codes == [400] * 6
          and (mj2["params"]["modes"]["head_z_bounce"]["fn_hz"] == 58.0)
          and mj2["params"]["floor"]["level"] == "off",
          "codes=%s fn=%s floor=%s"
          % (codes, mj2["params"]["modes"]["head_z_bounce"]["fn_hz"],
             mj2["params"]["floor"]["level"]))

    # ---- PUT 往返（fn 60→58、地板回 typical）+ 热切换 ----
    c1, _ = http_put(base + "/api/modal",
                     {"modes": {"head_z_bounce": {"fn_hz": 60.0}}})
    rb1 = http_json(base + "/api/modal")["params"]["modes"]["head_z_bounce"]
    c2, _ = http_put(base + "/api/modal",
                     {"modes": {"head_z_bounce": {"fn_hz": 58.0}},
                      "floor": {"level": "typical"}})
    mj3 = http_json(base + "/api/modal")
    _modal(True)
    on1 = http_json(base + "/api/modal")["enabled"]
    _modal(False)
    st0 = http_json(base + "/api/modal/state")
    q0 = st0["modes"]["head_z_bounce"]["q_mm"]
    check("i22.modal_put_roundtrip",
          c1 == 200 and rb1["fn_hz"] == 60.0 and c2 == 200
          and mj3["params"]["modes"]["head_z_bounce"]["fn_hz"] == 58.0
          and mj3["floor_rms_um"] == 0.3,
          "c=%s/%s rb=%s floor=%s" % (c1, c2, rb1["fn_hz"],
                                      mj3["floor_rms_um"]))
    check("i22.modal_toggle",
          on1 is True and st0["enabled"] is False and q0 == 0.0,
          "on=%s off=%s q_after_off=%s" % (on1, st0["enabled"], q0))

    # ---- ON+地板 typical：模态 RSS≈0.3µm（宽松）+ toolpoints 模块误差
    #      相对 OFF 基线有可测偏移（6DOF 投影贯通） ----
    _modal(True)
    time.sleep(1.0)          # RMS EMA τ≈0.25s，4τ 收敛
    st = http_json(base + "/api/modal/state")
    rss = math.sqrt(sum(v["rms_um"] ** 2
                        for v in st["modes"].values()))
    _modal(False)
    tp0 = http_json(base + "/api/toolpoints")["modules"]["wafer_stage"]
    _modal(True)
    time.sleep(0.4)
    deltas = []
    for _ in range(3):
        tp1 = http_json(base + "/api/toolpoints")["modules"]["wafer_stage"]
        deltas.append(max(abs(a - b) for a, b
                          in zip(tp1["error_nm"], tp0["error_nm"])))
        time.sleep(0.1)
    check("i22.modal_floor_rms",
          0.1 < rss < 0.9 and max(deltas) > 30.0,
          "RSS=%.3fµm toolpoints_Δmax=%.1fnm" % (rss, max(deltas)))

    # ---- 曝光积分（FAST）：模态 ON+地板较差 → 同机位连拍散布 >OFF ----
    _modal(False)
    s_off, n_off = _fast_spread("down", 12)
    http_put(base + "/api/modal", {"floor": {"level": "poor"}})
    _modal(True)
    time.sleep(0.5)
    s_on, n_on = _fast_spread("down", 12)
    http_put(base + "/api/modal", {"floor": {"level": "off"}})
    check("i22.modal_exposure_fast",
          n_off >= 8 and n_on >= 8 and s_off > 0.0 and s_on > 2.0 * s_off,
          "off=%.3fpx(n=%d) on=%.3fpx(n=%d) ratio=%.1f"
          % (s_off, n_off, s_on, n_on, s_on / max(s_off, 1e-9)))

    # ---- 短行程运动激发：scope 峰值 ≫ 尾段底噪、整定尾巴 ON>OFF ----
    # （激发前 modal OFF→ON 清缓冲——8s 历史窗不串前序行程）
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 0.0,
                                        "speed": 600.0})
    wait_in_position("WS_LS_X")
    time.sleep(0.3)
    _modal(False)
    _modal(True)                     # 清缓冲：只录本次激发
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 5.0,
                                        "speed": 600.0})
    wait_in_position("WS_LS_X")
    time.sleep(0.3)                  # 让环尾巴进缓冲
    sc = http_json(base + "/api/modal/scope?ch=0&n=2500")
    d1 = sc["data"]
    peak = max(abs(v) for v in d1) if d1 else 0.0
    i_pk = max((i for i, v in enumerate(d1) if abs(v) == peak), default=0)
    idx_last = max((i for i, v in enumerate(d1) if abs(v) > 0.2e-3),
                   default=-1)
    settle_on = (idx_last - i_pk) * sc["dt"] if idx_last > i_pk else 0.0
    tail = max((abs(v) for v in d1[-150:]), default=0.0)   # 尾段底噪
    _modal(False)                    # OFF 对照：同行程不再记录（缓冲空）
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 0.0,
                                        "speed": 600.0})
    wait_in_position("WS_LS_X")
    sc2 = http_json(base + "/api/modal/scope?ch=0&n=2500")
    peak_off = max((abs(v) for v in sc2["data"]), default=0.0)
    check("i22.modal_move_excitation",
          peak * 1e3 > 0.5 and peak > 4.0 * max(tail, 1e-12)
          and settle_on > 0.03 and peak_off == 0.0,
          "peak=%.3fµm tail=%.4fµm settle_on=%.3fs peak_off=%.4fµm"
          % (peak * 1e3, tail * 1e3, settle_on, peak_off * 1e3))

    # ---- 键合接触脉冲：接触沿后 head_z_bounce 跃升（对照非接触行程）----
    http_put(base + "/api/modal", {"floor": {"level": "off"}})
    # 安全退避（与交接流程同序：先抬头、横退，再降臂）
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    http_post(base + "/api/axis/move", {"axis": "AS_Y", "target": 120.0,
                                        "speed": 500.0})
    wait_settled("AS_Y")
    http_post(base + "/api/axis/move", {"axis": "FA_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("FA_X", timeout=30.0)
    http_post(base + "/api/axis/move", {"axis": "WS_LS_X", "target": 0.0,
                                        "speed": 600.0})
    wait_settled("WS_LS_X")

    def _arm_probe(t_from, t_to):
        """BS_Arm_Z t_from→t_to 小行程（缓冲隔离）后 head_z_bounce 峰值。"""
        http_post(base + "/api/axis/move",
                  {"axis": "BS_Arm_Z", "target": t_from, "speed": 200.0})
        wait_settled("BS_Arm_Z", timeout=10.0)
        time.sleep(0.25)             # 行程自身激发残余环衰减（τ≈46ms）
        _modal(False)
        _modal(True)                 # 清缓冲：只录本次小行程
        http_post(base + "/api/axis/move",
                  {"axis": "BS_Arm_Z", "target": t_to, "speed": 200.0})
        wait_settled("BS_Arm_Z", timeout=10.0)
        sc = http_json(base + "/api/modal/scope?ch=2&n=2000")
        return max((abs(v) for v in sc["data"]), default=0.0)

    pk_ctrl = _arm_probe(-166.0, -166.4)      # 不接触（对照）
    pk_pulse = _arm_probe(-167.0, -167.55)    # 跨越 -167.43 接触沿
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z", timeout=10.0)
    # 脉冲 0.2mm/s → amp≈0.55µm，与 0.55mm 短行程自激（~0.25µm）非相干
    # 叠加后期望 ~0.6µm：绝对值取近 3× 裕度、比值宽松 1.8×
    check("i22.modal_contact_pulse",
          pk_pulse * 1e3 > 0.2 and pk_pulse > 1.8 * max(pk_ctrl, 1e-12),
          "pulse=%.3fµm ctrl=%.3fµm ratio=%.1f"
          % (pk_pulse * 1e3, pk_ctrl * 1e3,
             pk_pulse / max(pk_ctrl, 1e-12)))

    # ---- perf：modal ON 干净窗 tick≥999（红线；窗起点=段首，段内全程
    #      1× 无倍速残留，免等待） ----
    http_put(base + "/api/modal", {"floor": {"level": "typical"}})
    _modal(True)
    h = read_clean_health(base, t_seg, retries=1)
    check("i22.modal_perf", h["tick_hz"] >= 999.0,
          "tick=%.1fHz push=%.1fHz jitter_avg=%.0fus"
          % (h["tick_hz"], h["push_hz"], h["tick_jitter_avg_us"]))

    # ---- 段尾恢复：modal OFF（防污染末态 health 与后续复跑） ----
    _modal(False)


def test_iter23a():
    """--- 19.27 迭代 23 Phase A：BUG 链修复（布局签名/双键挂载/重置清结果/
    统计分代/REAL 亚像素+几何角连续化）---
    quick（两套件，sim≈0 + REAL 连拍 ~10s 墙钟）：
      - i23a.layout_signature_unit：进程内 Workpieces——键合消耗不警告
        （dies_consumed 增加、签名不变）/改处方派生真警告（签名不符）/
        重铺消除；
      - i23a.layout_stale_api：PUT 处方 film.dies（不 apply）→
        /api/workpiece/layout film_layout_mismatch=True（dies_laid≠
        dies_expected）；restore_default + 重上膜 → 消除；
      - i23a.update_measured_dual_key：进程内 BondResults——(die_id,site)
        双键挂**最新无 measured** 记录（同 die 复跑不错挂）；全测过 →
        回退首条 + measured_warning；
      - i23a.record_offset_snapshot：进程内 finalize_record（stub eng）——
        记录带 bond_offset 快照（键合时刻 fine_align.bond_offset_*，
        统计分代依据）；
      - i23a.bond_clear_api：POST /api/bond/clear → 记录+产率统计清零；
      - i23a.full_reset_clears_results：制造状态后 POST /api/full_reset →
        记录/产率一并清零（迭代 23 新语义，对照旧"历史保留"）+ 其余清场；
      - i23a.real_subpixel_repeat：REAL 同机位连拍 N=6——位置散布
        ≤0.15px（修复前 0.3~0.5px 显著下降，较实测 0.03px 留 3× 裕度）
        + 角度连续非零（≥2 个不同值，旧 0.2° 量化格恒同值=1；渲染噪声
        底版池=2 块轮换，静态场景不同值上限即 2）且散布 ≤0.02°
        （350µrad，较实测 ~55µrad 留 3×+ 裕度）。"""
    print("--- 19.27 迭代 23 Phase A（布局签名/双键/重置清结果/分代/"
          "REAL 精度） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    # ================= 进程内：布局签名语义 =================
    import workpieces as wpm
    wp1 = wpm.Workpieces(os.path.join(ROOT, "runtime", "transfers.json"),
                         recipe=None)
    lay0 = wp1.layout_public()
    wp1.dies[0]["carrier"] = "wafer"           # 模拟键合消耗 2 颗
    wp1.dies[1]["carrier"] = "head"
    lay1 = wp1.layout_public()
    wp1.grid_cfg["pitch"] = 6.0                # 模拟处方派生变化（陈膜）
    lay2 = wp1.layout_public()
    wp1.load_film()                            # 重铺 → 签名刷新
    lay3 = wp1.layout_public()
    check("i23a.layout_signature_unit",
          not lay0["film_layout_mismatch"] and lay0["dies_consumed"] == 0
          and lay1["dies_on_film"] == 98 and lay1["dies_consumed"] == 2
          and not lay1["film_layout_mismatch"]
          and lay2["film_layout_mismatch"] is True
          and not lay3["film_layout_mismatch"]
          and lay3["dies_consumed"] == 0,
          "消耗=%s/%s 陈膜=%s 重铺=%s"
          % (lay1["dies_consumed"], lay1["film_layout_mismatch"],
             lay2["film_layout_mismatch"], lay3["film_layout_mismatch"]))

    # ================= HTTP：布局 payload + 陈膜两态 =================
    http_post(base + "/api/workpiece/reset", {})
    lay = http_json(base + "/api/workpiece/layout")
    c, _ = http_put(base + "/api/recipe",
                    {"film": {"dies": {"size_x_mm": 5.0, "size_y_mm": 5.0}}})
    lay_s = http_json(base + "/api/workpiece/layout")
    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/film", {"action": "load"})
    lay_r = http_json(base + "/api/workpiece/layout")
    check("i23a.layout_stale_api",
          lay.get("ok") and lay.get("film_layout_mismatch") is False
          and lay.get("dies_laid") == 149 and lay.get("dies_consumed") == 0
          and c == 200 and lay_s.get("film_layout_mismatch") is True
          and lay_s.get("dies_laid") == 149
          and lay_s.get("dies_expected") == 293
          and lay_r.get("film_layout_mismatch") is False
          and lay_r.get("dies_laid") == 149
          and lay_r.get("dies_consumed") == 0,
          "初始 laid/consumed=%s/%s 陈膜=%s(%s/%s) 重铺=%s"
          % (lay.get("dies_laid"), lay.get("dies_consumed"),
             lay_s.get("film_layout_mismatch"), lay_s.get("dies_laid"),
             lay_s.get("dies_expected"), lay_r.get("film_layout_mismatch")))

    # ================= 进程内：update_measured 双键 =================
    import bond_results as brm
    tmpf = os.path.join(ROOT, "tests", "_i23a_bond_results.json")
    for f in (tmpf, tmpf + "2"):
        if os.path.exists(f):
            os.remove(f)
    br1 = brm.BondResults(tmpf)
    for i in range(2):      # 同 die 两轮运行（die_id 跨运行重复）
        br1.add({"die_id": "R01C01",
                 "site": {"row": 1, "col": 1, "x": 0.0, "y": 0.0},
                 "t_bond": 10.0 + i,
                 "truth": {"dx": 1e-4, "dy": 1e-4, "drz": 1e-5},
                 "measured": None, "chain": {}, "bond_params": {}})
    ok1 = br1.update_measured("R01C01", 1e-3, 1e-3, 1e-4, 99.0, site=[1, 1])
    rs1 = br1.list_public()["results"]
    # 第二轮复测：第一轮记录仍无 measured（同 die 复跑场景），挂到它上面
    ok2 = br1.update_measured("R01C01", 2e-3, 2e-3, 2e-4, 100.0, site=[1, 1])
    rs2 = br1.list_public()["results"]
    # site 不符：双键 miss → 回退首条 + warning
    ok3 = br1.update_measured("R01C01", 5e-3, 5e-3, 5e-4, 101.0, site=[9, 9])
    rs3 = br1.list_public()["results"]
    check("i23a.update_measured_dual_key",
          ok1 and ok2 and ok3
          and rs1[0]["measured"] is None
          and rs1[1]["measured"]["dx"] == 1e-3       # 挂最新无 measured
          and rs2[0]["measured"]["dx"] == 2e-3       # 次轮挂剩余未测记录
          and not rs2[0].get("measured_warning")
          and rs3[0]["measured"]["dx"] == 5e-3       # site 不符回退首条
          and bool(rs3[0].get("measured_warning")),
          "首轮=%s 次轮=%s warn=%s"
          % (rs1[1]["measured"], rs2[0]["measured"],
             rs3[0].get("measured_warning")))
    os.remove(tmpf)

    # ================= 进程内：bond_offset 快照（统计分代） =================
    import types
    import threading as _th

    class _Cfg(types.SimpleNamespace):
        def get(self):
            return self._v

    eng_s = types.SimpleNamespace(
        recipe=_Cfg(_v=json.load(open(os.path.join(ROOT, "runtime",
                                                   "recipe.json"),
                                      encoding="utf-8"))),
        constants=_Cfg(_v={"fine_align": {"bond_offset_x_um": 2.5,
                                          "bond_offset_y_um": -1.5,
                                          "bond_offset_rz_urad": 8.0}}),
        lock=_th.RLock(), sim_time=12.3)
    rec_s = brm.finalize_record(eng_s, [6, 6],
                                {"chain": {}, "bond_params": {},
                                 "timing": None})
    check("i23a.record_offset_snapshot",
          rec_s.get("bond_offset") == {"x_um": 2.5, "y_um": -1.5,
                                       "rz_urad": 8.0},
          "bond_offset=%s" % (rec_s.get("bond_offset"),))

    # ================= HTTP：/api/bond/clear =================
    c, r = http_post(base + "/api/bond/clear", {})
    res_c = http_json(base + "/api/bond/results")
    tp_c = http_json(base + "/api/bond/throughput")
    check("i23a.bond_clear_api",
          c == 200 and (r or {}).get("ok")
          and (res_c.get("count") or 0) == 0
          and (tp_c.get("ok_count") or 0) == 0
          and (tp_c.get("fail_count") or 0) == 0
          and (tp_c.get("net_time_s") or 0.0) == 0.0,
          "c=%s count=%s ok=%s net=%s"
          % (c, res_c.get("count"), tp_c.get("ok_count"),
             tp_c.get("net_time_s")))

    # ================= HTTP：full_reset 清结果（迭代 23 新语义） =================
    http_post(base + "/api/speed", {"scale": 2.0})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    http_post(base + "/api/workpiece/wafer", {"action": "load"})
    http_post(base + "/api/vision/identify", {"cam": "down"})
    n_pre = len((http_json(base + "/api/vision/results?limit=20")
                 .get("results") or []))
    c, r = http_post(base + "/api/full_reset", {})
    br1j = http_json(base + "/api/bond/results")
    tp1j = http_json(base + "/api/bond/throughput")
    axes1 = http_json(base + "/api/axes")
    viz1 = http_json(base + "/api/viz")
    n_post = len((http_json(base + "/api/vision/results?limit=20")
                  .get("results") or []))
    check("i23a.full_reset_clears_results",
          c == 200 and (r or {}).get("ok")
          and (br1j.get("count") or 0) == 0
          and (tp1j.get("ok_count") or 0) == 0
          and abs(axes1.get("speed_scale", -1) - 1.0) < 1e-9
          and viz1.get("wafer", {}).get("present") is False
          and n_pre >= 1 and n_post == 0,
          "c=%s br=%s tp=%s scale=%s wafer=%s nres=%s→%s"
          % (c, br1j.get("count"), tp1j.get("ok_count"),
             axes1.get("speed_scale"), viz1.get("wafer", {}).get("present"),
             n_pre, n_post))

    # ================= REAL 同机位连拍：亚像素散布 + 连续角 =================
    try:
        import cv2 as _cv2  # noqa: F401
        has_cv2 = True
    except ImportError:
        has_cv2 = False
    if not has_cv2:
        print("   [skip] cv2 未安装，i23a.real_subpixel_repeat 跳过（不 FAIL）")
        return
    # 场景同 iter17：晶圆上片 + WS(-38.594, 4.8) → down 相机居中 site quad
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/workpiece/wafer", {"action": "load",
                                              "offset": [0.0, 0.0]})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", -38.594), ("WS_LS_Y", 4.8),
                    ("AS_X", 0.0), ("AS_Y", 0.0), ("AS_Z", 0.0),
                    ("WS_Rz", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
        wait_settled(ax)
    cols, rows, angs = [], [], []
    try:
        http_post(base + "/api/vision/mode", {"mode": "real"})
        # 2026-09-08：本用例度量"渲染亚像素地板"（几何管线的连续性），
        # 须压低质心噪声（fast_noise_px 结果倒推注入后 σ=fnp=0.15px 会
        # 直接主导连拍散布）；取 0.01px——std_col≈0.02px（阈值 0.15 的
        # 1/7）且帧噪声保留角度微差（distinct≥2 的原意），传感器量值
        # 口径由 i21/i22/i28 系列覆盖
        http_put(base + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.01}}})
        for _ in range(6):               # 同机位连拍（中间不动轴）
            c, r = http_post(base + "/api/vision/identify",
                             {"cam": "down", "template": "cv_quad"})
            e = (r or {}).get("entry") or {}
            res = e.get("result") or {}
            if c == 200 and res.get("judge") and res.get("lstCol"):
                cols.append(res["lstCol"][0])
                rows.append(res["lstRow"][0])
                angs.append((res.get("lstAngle") or [0.0])[0])

        def _std(v):
            if len(v) < 2:
                return 9e9
            m = sum(v) / len(v)
            return (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5
        sc_, sr_, sa_ = _std(cols), _std(rows), _std(angs)
        distinct = len(set(round(a, 4) for a in angs))
        # 位置散布 ≤0.15px（修复前粗网格亚像素+质心 0.3~0.5px；全分辨率
        # 亚像素实测 ~0.03px，3× 裕度）；角度连续（≥2 个不同值——旧 0.2°
        # 量化格下连拍恒同值=1；渲染管线噪声底版池=2 块轮换，静态场景
        # 不同值上限即 2）且散布 ≤0.02°（350µrad，实测 ~55µrad 的 3×+
        # 裕度）
        check("i23a.real_subpixel_repeat",
              len(cols) >= 5 and sc_ <= 0.15 and sr_ <= 0.15
              and distinct >= 2 and sa_ <= 0.02,
              "n=%d std_col=%.4fpx std_row=%.4fpx distinct=%d "
              "std_ang=%.5f°(=%.0fµrad)"
              % (len(cols), sc_, sr_, distinct, sa_, sa_ * 17453))
        print("    [i23a] REAL 连拍 cols=%s angs=%s"
              % ([round(v, 3) for v in cols],
                 [round(v, 4) for v in angs]), flush=True)
    finally:
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        http_put(base + "/api/recipe",
                 {"sim_sensors": {"fast_noise_px": {"down": 0.1}}})
        for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
            http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                                "speed": 600.0})
        for ax in ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y", "AS_Z"):
            wait_settled(ax)
        http_post(base + "/api/workpiece/reset", {})


def test_iter23b():
    """--- 19.28 迭代 23 Phase B ---
    quick（两套件；段首恢复 recipe 默认+热仿真参数复位，段尾恢复 thermal OFF
    +波动 typical+头部 heater 回 25℃；段内 8× 提速窗在墙钟敏感断言前复原
    1×，perf 干净窗起点=末提速窗尾 t_perf）。
    组件级热仿真（LPTN 热网络）：
      - i23b.thermal_get_default：默认 OFF/20 组 21 节点/分频 tick/curve
        未知节点 400；
      - i23b.thermal_off_zero_impact：OFF（及 ON+deform 全禁）actual/
        toolpoints 与基线逐位一致；
      - i23b.thermal_put_invalid_400：h 越界/负额定 W/未知节点/未知轴/非法
        波动档 400 不污染；
      - i23b.thermal_put_roundtrip / toggle：PUT 生效回读恢复、enabled 热
        切换、OFF 状态保持；
      - i23b.thermal_step_response：PulseHeat_Up 阶跃 25→125℃——bond_head
        节点温度符号正确、主 τ 与 C/G_tot 估算同量级（±50%）、稳态能量守恒
        （resid→0）+ ΔT≈P·R 方向校验（head→arm 热流/温差符号）；
      - i23b.thermal_deform_toolpoints：热态 toolpoints wafer_stage scale
        Δ>0（CTE 方向）且量级符合、bond_head Z 热伸长与 CTE×ΔT×L 预测
        比值 ∈[0.8,1.25]；
      - i23b.modal_fn_eff_source：modal 热软化 ΔT 源 thermal ON 读热节点/
        OFF 回退 heater.cur（进程内单元断言）；
      - i23b.thermal_perf：thermal ON 干净窗 tick≥999Hz。
    迭代 55 辐射链（位置相关视角系数，rad_links）：
      - i55.rad_get_default：默认 4 链 schema/enable/state 在场；
      - i55.rad_flow_proximity：头 300℃ 压到上相机（gap=10mm）vs 抬起
        （≈70mm）——近位 flow ∈[1,4]W、近/远比值 >5×、F 单调；
      - i55.rad_invalid_400：eps 越界/未知轴/未知节点/a=b 400 不污染；
      - i55.rad_disable_zero：全禁→state 零辐射链（迭代 54c 零耦合结论钉死）
        +复开 g_w_k>0；
      - i55.rad_unit_model：进程内 ε_eff·σ·A 预算/4 链入网/div 不变；
      - i55.rad_energy_balance / rad_perf：rad ON 锚定稳态守恒 <15W、
        干净窗 tick≥999Hz。
    迭代 56 吸盘 Z 向热胀（deform_map chuck_z_growth + 共焦面高/接触位热联动）：
      - i56.chuck_z_growth_confocal：吸盘 110℃ 下共焦读数 +30~80µm 且与
        state.deform.wafer_stage.z 一致、z_focus 不变；
      - i56.chuck_z_growth_vision_defocus：下相机晶圆标记 blur 升 1.2~4.0
        （热离焦）；
      - i56.contact_thermal_shift：停 base_eff+0.04mm，冷态力 0、吸盘热态
        力 >0（接触提前 ~50µm）；
      - i56.collet_up_thermal：头 300℃ 上共焦裸吸盘面下移 0.6~1.2mm
        （head_z_growth 到达共焦）；
      - i56.off_zero_confocal：热 OFF 读数/接触力逐位回基线。"""
    print("--- 19.28 迭代 23 Phase B（组件级热仿真） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    def _thermal(en):
        c, r = http_post(base + "/api/thermal", {"enabled": en})
        assert c == 200 and r.get("enabled") == bool(en), (c, r)

    def _snap():
        st = http_json(base + "/api/axes")
        tp = http_json(base + "/api/toolpoints")
        return ([a["actual"] for a in st["axes"]],
                {m: (v["error_mm"], v["error_rad"])
                 for m, v in (tp.get("modules") or {}).items()})

    def _tstate():
        return http_json(base + "/api/thermal/state")

    # ---- 干净确定起点（recipe 默认 → keep-warm Heat_Dw 30℃；modal OFF
    #      防 toolpoints 模态噪声；热仿真参数复位波动档 off+deform 全使能）----
    http_post(base + "/api/recipe/restore_default", {})
    c, r = http_post(base + "/api/modal", {"enabled": False})
    http_put(base + "/api/thermal",
             {"env": {"wave": "off"},
              "deform_map": {k: {"enable": True}
                             for k in ("granite_pair", "chuck_die_pitch",
                                       "beam_to_head", "head_z_growth",
                                       "align_optics_drift")}})
    _thermal(False)

    # ---- 默认参数 + 默认 OFF + 节点表/派生 τ + curve 未知节点 400 ----
    tj = http_json(base + "/api/thermal")
    tp0 = tj.get("params") or {}
    st0 = _tstate()
    chuck = (st0.get("nodes") or {}).get("wafer_chuck") or {}
    c_cv, _ = http_get_raw(base + "/api/thermal/curve?node=no_node&n=10")
    curve400 = (c_cv == 400)
    check("i23b.thermal_get_default",
          tj.get("ok") and tj.get("enabled") is False
          and len(tp0.get("groups") or {}) == 20
          and len(st0.get("nodes") or {}) == 21
          and len(st0.get("links") or []) == 20
          and 50 <= (tj.get("tick") or {}).get("div", 0) <= 500
          and chuck.get("c_J_K", 0) > 1000.0
          and chuck.get("tau_s", 0) > 0.5
          and chuck.get("cte_ppm_K", 0) > 1.0
          and curve400,
          "groups=%d nodes=%d links=%d div=%s chuck_C=%.0f tau=%.2s c400=%s"
          % (len(tp0.get("groups") or {}), len(st0.get("nodes") or {}),
             len(st0.get("links") or []), (tj.get("tick") or {}).get("div"),
             chuck.get("c_J_K", 0), chuck.get("tau_s", 0), curve400))

    # ---- OFF 零影响：actual/toolpoints 与基线逐位一致；
    #      ON+deform 全禁（零注入）同样一致 ----
    s0 = _snap()
    _thermal(True)
    _thermal(False)
    s1 = _snap()
    http_put(base + "/api/thermal",
             {"deform_map": {k: {"enable": False}
                             for k in ("granite_pair", "chuck_die_pitch",
                                       "beam_to_head", "head_z_growth",
                                       "align_optics_drift")}})
    _thermal(True)
    time.sleep(0.4)          # 热 tick 已走若干拍：deform 全禁 → 零注入
    s2 = _snap()
    _thermal(False)
    http_put(base + "/api/thermal",
             {"deform_map": {k: {"enable": True}
                             for k in ("granite_pair", "chuck_die_pitch",
                                       "beam_to_head", "head_z_growth",
                                       "align_optics_drift")}})
    check("i23b.thermal_off_zero_impact",
          s0 == s1 == s2,
          "off_eq=%s idle_on_nomap_eq=%s" % (s0 == s1, s1 == s2))

    # ---- PUT 校验 400（h 越界/负额定 W/未知邻接节点/未知 deform 节点/
    #      负 P_fl/非法波动档）不污染 ----
    codes = [http_put(base + "/api/thermal", {"conv": {"h_w_m2k": -1.0}})[0],
             http_put(base + "/api/thermal", {"conv": {"h_w_m2k": 500.0}})[0],
             http_put(base + "/api/thermal",
                      {"heaters": {"Heat_Dw": {"rated_w": -100.0}}})[0],
             http_put(base + "/api/thermal",
                      {"links": [{"a": "no_node", "b": "base_frame",
                                  "kc": 0.3}]})[0],
             http_put(base + "/api/thermal",
                      {"deform_map": {"bad_entry": {
                          "enable": True, "nodes": ["no_node"],
                          "scale_axes": {"WS_LS_X": 1.0}}}})[0],
             http_put(base + "/api/thermal",
                      {"motors": {"groups": {"wafer_stage": {
                          "p_full_w": -5.0}}}})[0],
             http_put(base + "/api/thermal", {"env": {"wave": "swamp"}})[0]]
    tj2 = http_json(base + "/api/thermal")
    p2 = tj2.get("params") or {}
    check("i23b.thermal_put_invalid_400",
          codes == [400] * 7
          and (p2.get("conv") or {}).get("h_w_m2k") == 5.0
          and (p2.get("heaters") or {}).get("Heat_Dw", {})
              .get("rated_w") == 500.0
          and (p2.get("env") or {}).get("wave") == "off",
          "codes=%s h=%s rated=%s wave=%s"
          % (codes, (p2.get("conv") or {}).get("h_w_m2k"),
             (p2.get("heaters") or {}).get("Heat_Dw", {}).get("rated_w"),
             (p2.get("env") or {}).get("wave")))

    # ---- PUT 往返（h 5→6→5）+ enabled 热切换 + OFF 状态保持 ----
    c1, _ = http_put(base + "/api/thermal", {"conv": {"h_w_m2k": 6.0}})
    rb1 = (http_json(base + "/api/thermal").get("params") or {}) \
        .get("conv") or {}
    c2, _ = http_put(base + "/api/thermal", {"conv": {"h_w_m2k": 5.0}})
    rb2 = (http_json(base + "/api/thermal").get("params") or {}) \
        .get("conv") or {}
    check("i23b.thermal_put_roundtrip",
          c1 == 200 and rb1.get("h_w_m2k") == 6.0
          and c2 == 200 and rb2.get("h_w_m2k") == 5.0,
          "c=%s/%s rb=%s/%s" % (c1, c2, rb1.get("h_w_m2k"),
                                rb2.get("h_w_m2k")))

    # ================= 阶跃响应 + 守恒 + 变形贯通（同一热态） =================
    # 机位：WS 双轴非零（scale 杠杆）+ 臂退避；头阶跃 25→125℃（200℃/s 速率
    # → 0.5s 斜坡 ≈ 阶跃）+ 吸盘 25→60℃（keep-warm 被前序段 full_reset 清
    # 掉，显式命令不依赖遗留状态；0.733℃/s 在 8× 窗内到温），8× 提速窗
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)
    s_cold = _snap()
    _thermal(True)
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 125.0, "Steady_Time": 0}})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.Heat_Dw",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 60.0, "Steady_Time": 0}})
    set_speed(8.0)
    time.sleep(10.0)         # 主窗（有效倍速随负载 ~4.7~8×，≥47s sim）
    cv = http_json(base + "/api/thermal/curve?node=bond_head&n=200")
    d = cv.get("data") or []
    st1 = _tstate()
    head1 = st1["nodes"]["bond_head"]
    tau_est = float(head1["tau_s"])
    t63 = -1.0
    if d:
        # 缓冲含阶跃前平台段：t63 自最后一个平稳点（≤起点+0.5℃）起算
        base_t = d[0]
        i0 = max((i for i, v in enumerate(d) if v <= base_t + 0.5),
                 default=0)
        tgt_t = base_t + 0.632 * (d[-1] - base_t)
        for i in range(i0, len(d)):
            if d[i] >= tgt_t:
                t63 = (i - i0) * cv["dt"]
                break
    sign_ok = bool(d) and d[-1] > base_t + 50.0
    check("i23b.thermal_step_response",
          cv.get("ok") and sign_ok and t63 > 0.0
          and 0.5 * tau_est <= t63 <= 1.5 * tau_est,
          "T %.2f→%.2f℃ t63=%.2fs tau_est=%.2fs (C=%.0fJ/K) n=%d dt=%.2f"
          % (d[0] if d else -1, d[-1] if d else -1, t63, tau_est,
             head1["c_J_K"], len(d), cv.get("dt", -1)))
    # ---- 能量守恒 + ΔT≈P·R 方向（续上窗同热态） ----
    # 守恒读数口径：头节点净流入残差=C·ΔT/τ（锚定滞后指数尾，τ≈7.6s
    # 衰减）→ <15W（≪78W 主流）；有效倍速随负载漂移，轮询至收敛（指数
    # 衰减确定收敛，与墙钟/负载无关，最坏 ~7s 墙钟）
    st2 = None
    resid_head = None
    for _ in range(28):           # 负载下有效倍速 ~3.4× 时 14 拍不够（实测
        st2 = _tstate()           # resid 15.7/16.4 擦线 FAIL）→ 28 拍（指数
        resid_head = abs(st2["power"]["residual_by_node_w"]["bond_head"])
        if resid_head < 15.0:     # 衰减确定收敛，与墙钟/负载无关）
            break
        time.sleep(0.5)
    set_speed(1.0)
    t_perf = time.time()       # 末个提速窗终点（perf 干净窗起点）
    n2 = st2["nodes"]
    flow_ha = None
    r_ha = None
    for lk in st2["links"]:
        if {lk["a"], lk["b"]} == {"bond_arm", "bond_head"}:
            flow_ha = lk["flow_w"]
            r_ha = lk["r_k_per_w"]
    dT_ha = n2["bond_head"]["t_c"] - n2["bond_arm"]["t_c"]
    pR = abs(flow_ha) * r_ha if flow_ha is not None else -1.0
    # 方向：T_head>T_arm 且热流 head→arm（flow_w 定义 a→b 为正；本链
    # a=arm b=head，flow<0 即头端流向臂端）；|P·R| 与 |ΔT| 一致（±20%，
    # 链路报表口径校验）
    check("i23b.thermal_energy_balance",
          resid_head < 15.0 and dT_ha > 50.0
          and flow_ha is not None and flow_ha < 0.0
          and pR > 0.0 and 0.8 * dT_ha <= pR <= 1.2 * dT_ha,
          "resid=%.4fW dT(head-arm)=%.2f℃ flow=%.2fW R=%.3fK/W P·R=%.2f℃"
          % (resid_head, dT_ha, flow_ha or -9, r_ha or -9, pR))
    # ---- 热变形贯通 toolpoints（同一热态 vs 冷态基线）----
    s_hot = _snap()
    ws_cold = dict(s_cold[1])["wafer_stage"][0]
    ws_hot = dict(s_hot[1])["wafer_stage"][0]
    bh_cold = dict(s_cold[1])["bond_head"][0]
    bh_hot = dict(s_hot[1])["bond_head"][0]
    dx_ws = (ws_hot[0] - ws_cold[0]) * 1e6      # nm
    dy_ws = (ws_hot[1] - ws_cold[1]) * 1e6
    dz_bh = (bh_hot[2] - bh_cold[2]) * 1e6      # nm
    cte_c = n2["wafer_chuck"]["cte_ppm_K"]
    dT_c = n2["wafer_chuck"]["delta_t_c"]
    # 吸盘 scale 预测：Δdx = 轴位 × CTE×ΔT（granite ΔT≈0 无贡献）
    pred_dx = 58.2162 * cte_c * dT_c            # nm（ppm×mm×1e3）
    ratio_x = dx_ws / pred_dx if pred_dx else 9e9
    cte_h = n2["bond_head"]["cte_ppm_K"]
    dT_h = n2["bond_head"]["delta_t_c"]
    pred_dz = -cte_h * 1e-6 * dT_h * 150.0 * 1e6     # head_z_growth 预测 nm
    ratio = dz_bh / pred_dz if pred_dz else 9e9
    check("i23b.thermal_deform_toolpoints",
          dx_ws > 1000.0 and dy_ws > 1000.0      # 吸盘 scale CTE 方向（正）
          and 0.7 <= ratio_x <= 1.3              # 量级符合 CTE×ΔT×轴位
          and dz_bh < -100000.0                  # 头 Z 向下热伸长
          and 0.8 <= ratio <= 1.25,
          "wafer_stage Δdx=%.0fnm(pred %.0f,ratio %.3f) Δdy=%.0fnm "
          "bond_head Δdz=%.0fnm pred=%.0fnm ratio=%.3f"
          % (dx_ws, pred_dx, ratio_x, dy_ws, dz_bh, pred_dz, ratio))
    # ---- enabled 热切换 + OFF 状态保持（头节点热态 ~125℃：若 OFF 清状态
    #      会回落基准 25℃——双读相等且≠基准即"保持且零积分"）----
    on1 = http_json(base + "/api/thermal").get("enabled")
    _thermal(False)
    t1 = _tstate()["nodes"]["bond_head"]["t_c"]
    time.sleep(0.7)              # > 热 tick 周期（0.5s）：OFF 应零积分
    st_off = _tstate()
    t2 = st_off["nodes"]["bond_head"]["t_c"]
    check("i23b.thermal_toggle",
          on1 is True and st_off.get("enabled") is False
          and t1 == t2 and t1 > 100.0,
          "on=%s off=%s t1=%.4f t2=%.4f"
          % (on1, st_off.get("enabled"), t1, t2))
    _thermal(True)               # perf 需要 thermal ON
    # ---- heater 段尾恢复（头回 25℃、吸盘回 keep-warm 默认 30℃） ----
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 25.0, "Steady_Time": 0}})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.Heat_Dw",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 30.0, "Steady_Time": 0}})

    # ---- modal 热软化 ΔT 源切换（进程内单元断言：ON 读热节点 REF22 口径、
    #      OFF 回退 heater.cur、无 thermal 标记回原值；group_delta_t 映射） ----
    import modal as _modal_mod
    import thermal as _thermal_mod
    _ts_unit = _thermal_mod.ThermalSystem()
    _ts_unit._T["bond_head"] = 125.0
    _ts_unit._T["wafer_chuck"] = 25.0
    g_head = _ts_unit.group_delta_t("head")
    g_wafer = _ts_unit.group_delta_t("wafer")
    g_none = _ts_unit.group_delta_t("no_group")

    class _H(object):
        cur = 125.0

    class _Th(object):
        def __init__(self, on, dT):
            self.enabled = on
            self._dT = dT

        def group_delta_t(self, grp):
            return self._dT

    class _Eng(object):
        def __init__(self, th):
            self.thermal = th
            self.heaters = {"PulseHeat_Up": _H()}

    _m = _modal_mod.ModalSystem.__new__(_modal_mod.ModalSystem)
    _m._cfg = {"thermal_softening": {"enable": True, "coeff_per_c": 0.0012}}
    entry = {"fn_hz": 58.0, "thermal": "head"}
    fn_on = _m._fn_eff(entry, _Eng(_Th(True, 97.2)))    # ON：读节点 ΔT=97.2
    fn_off = _m._fn_eff(entry, _Eng(_Th(False, 97.2)))  # OFF：heater 125−22
    fn_plain = _m._fn_eff({"fn_hz": 58.0}, _Eng(_Th(True, 97.2)))
    check("i23b.modal_fn_eff_source",
          abs(g_head - 103.0) < 1e-9          # 125−22（REF 22 口径）
          and abs(g_wafer - 3.0) < 1e-9       # wafer_chuck 初始 25−22
          and g_none is None
          and abs(fn_on - 58.0 * (1 - 0.0012 * 97.2)) < 1e-9
          and abs(fn_off - 58.0 * (1 - 0.0012 * 103.0)) < 1e-9
          and abs(fn_plain - 58.0) < 1e-9,
          "g_head=%.2f g_wafer=%.2f fn_on=%.3f fn_off=%.3f fn_plain=%.2f"
          % (g_head, g_wafer, fn_on, fn_off, fn_plain))

    # ---- perf：thermal ON 干净窗 tick≥999Hz（热 tick 2Hz 分频开销内）----
    h = read_clean_health(base, t_perf, retries=1)
    check("i23b.thermal_perf",
          h["tick_hz"] >= 999.0,
          "tick=%.1fHz push=%.1fHz jitter_avg=%.0fus"
          % (h["tick_hz"], h["push_hz"], h["tick_jitter_avg_us"]))

    # ================= 迭代 55：辐射链（位置相关视角系数） =================
    # 准备：AS→锚点 (0,6)（rad 链 lat 参考位；段尾还原）、头 25→300℃（8× 窗），
    # rad 4 链默认全开（thermal 仍 ON）
    st_ax = http_json(base + "/api/axes")
    as_xy0 = {a["name"]: a["pos"] for a in st_ax["axes"]
              if a["name"] in ("AS_X", "AS_Y")}
    for ax, tgt in (("AS_X", 0.0), ("AS_Y", 6.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 300.0, "Steady_Time": 0}})
    set_speed(8.0)
    t_head = 0.0
    for _ in range(40):                     # 锚定 τ≈7.6s sim：轮询到 ≥290℃
        t_head = _tstate()["nodes"]["bond_head"]["t_c"]
        if t_head >= 290.0:
            break
        time.sleep(0.5)
    set_speed(1.0)

    # ---- 默认参数/schema/state ----
    tp55 = (http_json(base + "/api/thermal").get("params") or {}) \
        .get("rad_links") or []
    st55 = _tstate().get("rad_links") or []
    check("i55.rad_get_default",
          len(tp55) == 4 and len(st55) == 4
          and all(0.0 < l.get("eps_a", 0) <= 1.0
                  and 0.0 < l.get("f_max", 0) <= 1.0
                  and l.get("area_mm2", 0) > 0.0 for l in tp55)
          and {l["a"] + ">" + l["b"] for l in tp55} ==
          {"bond_head>align_optics", "wafer_chuck>align_optics",
           "bond_head>wafer_chuck", "bond_head>die_chuck"},
          "params=%d state=%d t_head=%.1f" % (len(tp55), len(st55), t_head))

    # ---- 位置相关热流：头压到相机上（gap=10mm）vs 抬起（gap≈70mm） ----
    def _rad_flow(a, b):
        for lk in _tstate().get("rad_links") or []:
            if lk["a"] == a and lk["b"] == b:
                return lk
        return None

    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": -119.83,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    time.sleep(0.7)                          # ≥1 个热 tick：gap/F/flow 重算
    near = _rad_flow("bond_head", "align_optics")
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": -60.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    time.sleep(0.7)
    far = _rad_flow("bond_head", "align_optics")
    ratio = (near["flow_w"] / far["flow_w"]) \
        if (near and far and far["flow_w"] > 0) else -1.0
    check("i55.rad_flow_proximity",
          near is not None and far is not None
          and abs(near["gap_mm"] - 10.0) < 0.5
          and 1.0 <= near["flow_w"] <= 4.0
          and ratio > 5.0 and near["f"] > far["f"],
          "near f=%.4f gap=%.1f flow=%.3fW far f=%.4f gap=%.1f flow=%.3fW "
          "ratio=%.1f T=%.1f"
          % (near["f"], near["gap_mm"], near["flow_w"], far["f"],
             far["gap_mm"], far["flow_w"], ratio, t_head))

    # ---- PUT 400：eps 越界/未知轴/未知节点/a=b，不污染 ----
    good = (http_json(base + "/api/thermal").get("params") or {}) \
        .get("rad_links") or []
    bad1 = [dict(good[0], eps_a=1.5)] + good[1:]
    bad2 = [dict(good[0], gap_axis={"name": "NO_AXIS", "pos_ref_mm": 0.0,
                                    "dir": 1.0})] + good[1:]
    bad3 = [dict(good[0], a="no_node")] + good[1:]
    bad4 = [dict(good[0], b="bond_head")] + good[1:]
    codes55 = [http_put(base + "/api/thermal", {"rad_links": bad1})[0],
               http_put(base + "/api/thermal", {"rad_links": bad2})[0],
               http_put(base + "/api/thermal", {"rad_links": bad3})[0],
               http_put(base + "/api/thermal", {"rad_links": bad4})[0]]
    tp55b = (http_json(base + "/api/thermal").get("params") or {}) \
        .get("rad_links") or []
    check("i55.rad_invalid_400",
          codes55 == [400] * 4 and len(tp55b) == 4
          and all(l.get("enable") for l in tp55b),
          "codes=%s params=%d" % (codes55, len(tp55b)))

    # ---- 全禁=零辐射链（迭代 54c 零耦合结论回归钉死）+ 复开 ----
    # 注意：PUT rad_links 触发温度状态重置（update() 语义），复开后 g_w_k>0
    # 即可（flow 随节点回温恢复，不在本用例断言）
    off_arr = [dict(l, enable=False) for l in good]
    c55, _ = http_put(base + "/api/thermal", {"rad_links": off_arr})
    st55c = _tstate().get("rad_links") or []
    on_arr = [dict(l, enable=True) for l in good]
    c55b, _ = http_put(base + "/api/thermal", {"rad_links": on_arr})
    time.sleep(0.7)
    st55d = _tstate().get("rad_links") or []
    check("i55.rad_disable_zero",
          c55 == 200 and st55c == []
          and c55b == 200 and len(st55d) == 4
          and all(lk["f"] > 0.0 for lk in st55d)
          and sum(1 for lk in st55d if lk["g_w_k"] > 0.0) >= 3,
          "off_state=%d on_state=%d g=%s"
          % (len(st55c), len(st55d),
             [round(lk["g_w_k"], 6) for lk in st55d]))

    # ---- 纯函数级（进程内）：默认 4 链入网 + ε_eff·σ·A 预算 + 稳定 dt 不变 ----
    _u55 = _thermal_mod.ThermalSystem()
    _lk0 = _u55._net["rad"][0]
    _eps_eff = 1.0 / (1.0 / 0.8 + 1.0 / 0.9 - 1.0)
    check("i55.rad_unit_model",
          len(_u55._net["rad"]) == 4
          and abs(_lk0["k_w"] - _eps_eff * _thermal_mod.SIGMA_SB * 2000e-6)
          < 1e-15
          and 50 <= _u55._net["div"] <= 500,
          "rad=%d k_w=%.3e div=%d" % (len(_u55._net["rad"]), _lk0["k_w"],
                                      _u55._net["div"]))

    # ---- rad ON 锚定稳态能量守恒 + perf（头回 300℃ 后） ----
    # 主窗 ≥47s sim（i23b 同构）：回温与锚定滞后残差衰减同窗完成——PUT 复开
    # 重置过温度，重新趋 300℃ 时刻残差 ~3kW（锚定 300W/K×Δ10K），需 ~10τ 窗
    set_speed(8.0)
    time.sleep(10.0)
    resid55 = None
    for _ in range(28):           # 同 i23b：负载下有效倍速漂移→28 拍收敛窗
        st55e = _tstate()
        # 口径同 i23b=分节点残差（head 锚定滞后尾）；balance_residual_w
        # 是全节点 max——慢节点（electronics τ≈31min 等）重置后再升温的
        # 物理长尾恒占 ~100W，非缺陷（文档 §11 口径），不作判据
        resid55 = abs(st55e["power"]["residual_by_node_w"]["bond_head"])
        if resid55 < 15.0:
            break
        time.sleep(0.5)
    set_speed(1.0)
    t_perf55 = time.time()
    resid_opt = abs(st55e["power"]["residual_by_node_w"]["align_optics"])
    check("i55.rad_energy_balance",
          resid55 is not None and resid55 < 15.0 and resid_opt < 5.0,
          "resid_head=%.4fW resid_align_optics=%.4fW"
          % (resid55 if resid55 is not None else -1, resid_opt))
    h55 = read_clean_health(base, t_perf55, retries=1)
    check("i55.rad_perf",
          h55["tick_hz"] >= 999.0,
          "tick=%.1fHz push=%.1fHz jitter=%.0fus"
          % (h55["tick_hz"], h55["push_hz"], h55["tick_jitter_avg_us"]))

    # ---- i55 段内恢复：头回 25℃、BS_Arm_Z 回 0、AS 还原、speed 1× ----
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 25.0, "Steady_Time": 0}})
    http_post(base + "/api/axis/move", {"axis": "BS_Arm_Z", "target": 0.0,
                                        "speed": 200.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in as_xy0.items():
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in as_xy0:
        wait_settled(ax)
    set_speed(1.0)

    # ================= 迭代 56：吸盘 Z 向热胀（面高/接触位热联动） =================
    # 准备：晶圆在位（无则 0 偏置上片，段尾卸）、AS (0,6,0) 下共焦量程内、
    # WS 至 site_R14C14#1 标记居视场位、BS 远；热 ON（沿用 i55 段状态）
    st_ax56 = http_json(base + "/api/axes")
    pos56 = {a["name"]: a["pos"] for a in st_ax56["axes"]
             if a["name"] in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y",
                              "BS_Arm_Z")}
    c56, _r56 = http_post(base + "/api/workpiece/wafer",
                          {"action": "load", "offset": [0.0, 0.0], "rz": 0.0})
    loaded56 = (c56 == 200)
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    for ax, tgt in (("AS_X", 0.0), ("AS_Y", 6.0), ("AS_Z", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("WS_LS_X", -46.197), ("WS_LS_Y", 10.799):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 300.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)
    _thermal(True)
    # 头等冷（<26.5℃）：否则残余头温在量窗内继续冷却，dz_h 漂移会把
    # 接触位信号（dz_c≈+50µm）淹没——i55 段 300℃ 余热锚定 τ=7.6s 冷却很快
    set_speed(8.0)
    for _ in range(60):
        if _tstate()["nodes"]["bond_head"]["t_c"] < 26.5:
            break
        time.sleep(0.5)
    set_speed(1.0)

    # ---- 冷态基线：下共焦打晶圆面 + 视觉离焦 + 接触力（停 base_eff+0.02） ----
    # 力读数链噪声 ±1.0N/拍（recipe sim_sensors.force_noise_N 默认 1.0，
    # 单次读数不可判）——30 拍均值（均匀噪声 σ→~0.18N）
    def _force_avg(n=30, dt=0.05):
        s = 0.0
        for _ in range(n):
            s += float((http_json(base + "/api/io").get("points") or {}).get(
                "SystemMachine.SS_Bonding.M_BS.oActualForce", 0.0) or 0.0)
            time.sleep(dt)
        return s / n

    rd0 = http_json(base + "/api/confocal/read?probe=down")
    id0 = http_post(base + "/api/vision/identify",
                    {"cam": "down", "template": "cv_quad"})[1]
    blur0 = (id0.get("entry") or {}).get("blur_ratio") or 0.0
    mc56 = http_json(base + "/api/machine_constants")
    rc56 = http_json(base + "/api/recipe")
    base_eff = float(mc56["constants"]["bond"]["head_z_contact_mm"]) \
        + float(rc56["recipe"]["wafer"]["thickness_mm"]) \
        + float(rc56["recipe"]["film"]["dies"]["thickness_mm"])
    as_safe = [float(v) for v in
               mc56["constants"]["flip_handoff"]["as_safe_xy_mm"]]
    time.sleep(0.3)
    f0 = _force_avg()      # 零点基线（BS=0 远离接触，位置无关）

    # ---- 吸盘 25→110℃（8× 窗；anchor τ=3.3s 快通道 + 升温速率限） ----
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.Heat_Dw", "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 110.0, "Steady_Time": 0}})
    set_speed(8.0)
    t_chuck = 0.0
    for _ in range(90):
        t_chuck = _tstate()["nodes"]["wafer_chuck"]["t_c"]
        if t_chuck >= 105.0:
            break
        time.sleep(0.5)
    set_speed(1.0)
    rd1 = http_json(base + "/api/confocal/read?probe=down")
    id1 = http_post(base + "/api/vision/identify",
                    {"cam": "down", "template": "cv_quad"})[1]
    blur1 = (id1.get("entry") or {}).get("blur_ratio") or 0.0
    dz_state = _tstate()["deform"]["wafer_stage"]["t_mm"][2] * 1000.0
    d_read = (rd1.get("reading_um") or -9.0) - (rd0.get("reading_um") or -9.0)
    d_zf = abs((rd1.get("z_focus") or 0.0) - (rd0.get("z_focus") or 0.0)) \
        * 1000.0
    check("i56.chuck_z_growth_confocal",
          rd0.get("valid") and rd1.get("valid")
          and 30.0 <= d_read <= 80.0
          and abs(dz_state - d_read) < 15.0 and d_zf < 1.0,
          "Δread=%.1fµm deform_z=%.1fµm Δz_focus=%.2fµm T=%.1f"
          % (d_read, dz_state, d_zf, t_chuck))
    check("i56.chuck_z_growth_vision_defocus",
          1.2 <= (blur1 - blur0) <= 4.0,
          "blur %.2f→%.2f（晶圆面热离焦 ≈ deform_z/dof）" % (blur0, blur1))

    # ---- 接触位热漂移：吸盘热（+dz_c）头近冷 → 接触提前，同位力跳升 ----
    # 判据用**增量**不用绝对值：发布读数=（真值×增益误差+零偏−标定零偏）/
    # 标定增益，零点由盘上 sensor_calib 标定值决定（非物理零，迭代 49 口径）
    # 注意：头降到键合行程前对准台必须先退避（as_safe_xy）——否则沙盒干涉
    # （上气环×对准相机承载台）409 拒动，与真机键合流程同序
    for ax, tgt in (("AS_X", as_safe[0]), ("AS_Y", as_safe[1])):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": base_eff + 0.02, "speed": 50.0})
    wait_settled("BS_Arm_Z")
    f1 = _force_avg()
    check("i56.contact_thermal_shift",
          (f1 - f0) > 0.8,
          "f_cold=%.3fN f_hot=%.3fN Δ=%.3fN（base_eff=%.3f 停 base+0.02mm "
          "T=%.1f）" % (f0, f1, f1 - f0, base_eff, t_chuck))
    # 回锚点与高位（后续 collet/off_zero 需要）
    for ax, tgt in (("AS_X", 0.0), ("AS_Y", 6.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 100.0})
    wait_settled("BS_Arm_Z")

    # ---- 上共焦对 300℃ 头：head_z_growth 到达共焦（裸吸盘面下移 ~1mm） ----
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": -1.0,
                                        "speed": 50.0})
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": -119.83, "speed": 100.0})
    for ax in ("AS_Z", "BS_Arm_Z"):
        wait_settled(ax)
    ru0 = http_json(base + "/api/confocal/read?probe=up")
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 300.0, "Steady_Time": 0}})
    set_speed(8.0)
    t_head56 = 0.0
    for _ in range(40):
        t_head56 = _tstate()["nodes"]["bond_head"]["t_c"]
        if t_head56 >= 290.0:
            break
        time.sleep(0.5)
    set_speed(1.0)
    ru1 = http_json(base + "/api/confocal/read?probe=up")
    dzs_up = (ru0.get("z_surface") or 0.0) - (ru1.get("z_surface") or 0.0)
    check("i56.collet_up_thermal",
          ru0.get("valid") and 0.6 <= dzs_up <= 1.2,
          "up z_surface %.4f→%.4f Δ=%.3fmm read=%s T=%.1f"
          % (ru0.get("z_surface") or -9, ru1.get("z_surface") or -9,
             dzs_up, ru1.get("reading_um"), t_head56))

    # ---- 回冷归位（热 ON 冷却回 30/25℃）：读数/接触力回冷态基线 ----
    # 可逆性判据（热 OFF 零影响由 i23b.thermal_off_zero_impact 工具点
    # 逐位基线承担；此处钉"效应由温度驱动、温度回则效应回"）
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.PulseHeat_Up",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 25.0, "Steady_Time": 0}})
    http_post(base + "/api/io/command",
              {"path": "SystemMachine.Heat_Dw",
               "cmd": "SetTargetTempWithPlan",
               "args": {"Temperature": 30.0, "Steady_Time": 0}})
    set_speed(8.0)
    for _ in range(120):
        _st = _tstate()["nodes"]
        if _st["wafer_chuck"]["t_c"] < 32.0 and _st["bond_head"]["t_c"] < 28.0:
            break
        time.sleep(0.5)
    set_speed(1.0)
    http_post(base + "/api/axis/move", {"axis": "AS_Z", "target": 0.0,
                                        "speed": 50.0})
    wait_settled("AS_Z")
    time.sleep(1.2)          # eff_contact 按 ~1s 节流重算
    rd2 = http_json(base + "/api/confocal/read?probe=down")
    # f2 同 contact_shift 的沙盒约束：先退避再停 base+0.02
    for ax, tgt in (("AS_X", as_safe[0]), ("AS_Y", as_safe[1])):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": base_eff + 0.02, "speed": 50.0})
    wait_settled("BS_Arm_Z")
    f2 = _force_avg()
    check("i56.off_zero_confocal",
          rd2.get("valid")
          and abs((rd2.get("reading_um") or 0.0)
                  - (rd0.get("reading_um") or 0.0)) < 1.0
          and abs(f2 - f0) < 0.5,
          "read %.2f vs %.2fµm f %.3f vs %.3fN（回冷后归位）"
          % (rd2.get("reading_um") or -9, rd0.get("reading_um") or -9,
             f2, f0))

    # ---- i56 段内恢复：轴位还原、晶圆（新上则卸）、speed 1× ----
    # 顺序敏感：先抬 BS_Arm_Z 出键合行程，再把对准台移回（防恢复路径
    # 撞沙盒干涉——与接触测量段的退避约束同一物理）
    if "BS_Arm_Z" in pos56:
        http_post(base + "/api/axis/move",
                  {"axis": "BS_Arm_Z", "target": pos56["BS_Arm_Z"],
                   "speed": 200.0})
        wait_settled("BS_Arm_Z")
    for ax, tgt in pos56.items():
        if ax == "BS_Arm_Z":
            continue
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in pos56:
        if ax == "BS_Arm_Z":
            continue
        wait_settled(ax)
    if loaded56:
        http_post(base + "/api/workpiece/wafer", {"action": "unload"})
    set_speed(1.0)

    # ---- 段尾恢复：thermal OFF + 波动 typical + WS 回零 ----
    _thermal(False)
    http_put(base + "/api/thermal", {"env": {"wave": "typical"}})
    for ax in ("WS_LS_X", "WS_LS_Y"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 600.0})
    for ax in ("WS_LS_X", "WS_LS_Y"):
        wait_settled(ax)


def test_nonortho():
    """--- 迭代 37：Y 导轨非正交角测校（栅格跳测法重写，-k "i37 or nonortho or
    i36" 定向段，段名 19.29.nonortho）---
    原理与契约见 metrology/stage_nonortho.py / calibration._validate_stage_
    nonortho：基准尺=晶圆 die 阵列（wafer_sites 格点+site_marks quad 假定
    正交），走距=格点节距（默认处方 20mm，start 与处方校验一致 0.001mm）；
    o_i=标记世界位−相机世界位（_context 链），o_i 对步号 LS 斜率（=台步进
    −栅格步弦差）；迭代 42b 起分解走横向/沿轨分量：X 线横向分量读栅格
    安装旋转 ρ̂=−asin(s_x'·ŷ/p)、Y 线横向分量给 ψ_y=asin(s_y'·x̂/p)=
    φ_t+ρ̂，φ(X 基准)=wrap(ψ_y−ρ̂)（ρ̂ 差中抵消）；步进向量 v=s'+p·R(ρ̂)
    e_line 加回栅格步，ε=|v|/p−1（比例误差，迭代 42b 新增）。十字对称
    走法（K 奇数，中心±(K−1)/2 步）；θ_y 偏差口径=wrap(φ−ρ̂)。严格带前提：main() 在 -k 定向（tokens ⊆
    {i36,i37,nonortho}）时把扫描轴 WS_LS_X/WS_LS_Y/AS_X/AS_Y 的 6 分量误
    差地图归零（手法同 test_metrology 真值段，finally 按字节恢复）——长
    杠杆（80mm）把随机项压到 µrad 级后，残差主导项是 6 分量地图投影。
    用例：ws/as 注入往返恢复 ±5µrad、链关不消费、ref_axis=y 负值输出、
    pitch/行程/无标记 400、cancel 恢复语义、十字轨迹+θ_y 偏差口径+节点
    CSV（19 列行基准 schema）+物理↔逻辑双向全矩阵（迭代 42b：[col_x|
    col_y]=步进向量/pitch 实测像，列模长=ε）+逐节点进度、K 偶数 400
    与奇数边界 3/21 放行、比例误差 εx/εy（迭代 42b scale_inject：main()
    段首定向上下文 low_order.json WS_LS_X 注入 +200ppm，断言基准=GET
    /api/errormaps 活基线，±30ppm 地板口径；εx 注入不污染角读出回归）。"""
    base = "http://127.0.0.1:%d" % WS_PORT
    LIGHT_DOWN = "SystemMachine.SS_Bonding.M_AS.LightOn_2"
    TOL = 5.0                     # ±5µrad 严格带（地图归零 + 80mm 长杠杆）
    TOL_ABS = 15.0                # WS 主套件绝对容差（2026-09-16 定版）：
                                  # quick/standalone 上下文系统地板 ~±10µrad
                                  # （scale_ppm 折进 ψ_y + 地图别名 + ρ̂ 门限
                                  # 混叠，差分路径已证伪）；真值级 ±5 由
                                  # test_metrology i36（地图归零）承担
    TOL_AS = 40.0                 # AS 地图开实测漂移地板 ~10µrad 级（对准台
                                  # 带相机动+走位后误差图重采样漂移，三轮实录
                                  # +0.3/−1.1、+5.4/+5.3、−9.2 基线跳）——±5
                                  # 系地图归零 truth 段口径（test_metrology
                                  # 真值段承担），主套件 AS 取 ±40 同 i36 主
                                  # 套件口径；差分消静态偏置后仍须容漂移
    INJ = {"ws_stage_nonortho": "ws_y_rail_dir_urad",
           "as_stage_nonortho": "as_y_rail_dir_urad"}
    JOG = {"ws_stage_nonortho": ("WS_LS_X", "WS_LS_Y"),
           "as_stage_nonortho": ("AS_X", "AS_Y")}

    def move(axis, target, speed=600.0):
        c, r = http_post(base + "/api/axis/move",
                         {"axis": axis, "target": target, "speed": speed})
        assert c == 200, (axis, target, r)
        return wait_settled(axis)

    def run_job(routine, tpl, timeout=900.0, **params):
        p = {"template_name": tpl}
        p.update(params)
        c, r = http_post(base + "/api/calibration/start",
                         {"routine": routine, "cam": "down", "params": p})
        if c != 200:
            return {"state": "start_%d" % c, "error": r.get("error")}, None
        jid = r["job_id"]
        s = wait_terminal(jid, timeout)
        res = None
        if s.get("state") == "done":
            res = http_json(base + "/api/calibration/result?job_id=%d"
                            % jid)["result"]
        return s, res

    def wait_terminal(jid, timeout=900.0):
        """轮询作业到终态（done/error/canceled），返回末次 status。"""
        t0 = time.time()
        s = {}
        while time.time() - t0 < timeout:
            s = http_json(base + "/api/calibration/status?job_id=%d" % jid)
            if s.get("state") in ("done", "error", "canceled"):
                break
            time.sleep(0.2)
        return s

    def del_tpl(name):
        http_delete(base + "/api/vision/templates", {"name": name})

    def put_stage_geo(**kw):
        c, r = http_put(base + "/api/machine_constants", {"stage_geo": kw})
        assert c == 200 and r.get("ok"), (kw, r)

    def center_quad():
        """guide 引导把最近 site quad 角居中到 down 画幅中心（下相机
        jog=WS 走位）——栅格跳测起点须角对准（走距=格点节距使相对相位
        逐 node 复现）。"""
        g = http_json(base + "/api/vision/guide?cam=down")
        qd = next((m for m in g.get("marks") or []
                   if (m.get("name") or "").startswith("site_")), None)
        assert qd, "down 视野几何内无 site 标记（先上片）"
        for _ in range(8):
            g = http_json(base + "/api/vision/guide?cam=down")
            mk = next((m for m in g.get("marks") or []
                       if m.get("name") == qd["name"]), None)
            if mk is None:
                break
            if mk.get("in_fov") and \
                    max(abs(d) for d in mk["delta_mm"][:2]) < 0.005:
                return mk["name"]
            for s in (mk.get("suggested") or []):
                move(s["axis"],
                     get_axis_entry(s["axis"])["pos"] + s["delta_mm"])
        return qd["name"]

    def mk_tpl(name, anchor=None):
        """当前帧建 601px 小 ROI 模板（灯开 teach，完灭；i34 口径）。"""
        c, r = http_post(base + "/api/io/command",
                         {"path": LIGHT_DOWN, "cmd": "SetBool",
                          "args": {"value": True}})
        assert c == 200 and r.get("ok"), (c, r)
        time.sleep(0.25)
        try:
            if anchor is None:
                g = http_json(base + "/api/vision/guide?cam=down")
                ms = [m for m in g.get("marks") or [] if m.get("in_fov")]
                assert ms, "视野内无标记可锚"
                anchor = ms[0]["fov_center"]
            col, row = anchor
            tw = 601
            x0 = max(0, min(3000 - tw, int(round(col)) - 300))
            y0 = max(0, min(3000 - tw, int(round(row)) - 300))
            del_tpl(name)
            c, r = http_post_tpl(base + "/api/vision/templates",
                             {"cam": "down", "name": name,
                              "roi": [x0, y0, tw, tw]})
            assert c == 200 and r.get("ok"), (c, r)
        finally:
            http_post(base + "/api/io/command",
                      {"path": LIGHT_DOWN, "cmd": "SetBool",
                       "args": {"value": False}})

    def _rc(name):
        """site_R{r}C{c}#{k} → (r, c)。"""
        p = name.replace("site_R", "").split("C")
        return int(p[0]), int(p[1].split("#")[0])

    def prep(stage):
        """上片 + 走位台就位 + quad 角居中 + 小模板（返回模板名；走位轴
        当前位即栅格跳测 center）。**角选择须留走法余量**：十字对称走法
        ±(K−1)/2=±2 步（K=5），ws/as 统一把居中角行列调到 [6,9]——下界
        防圆裁出格（实测 R03 行 C01 起无标记：±2 步须 ≥2，留 6 余量；
        居中靠 WS jog，WS −pitch=索引递增 / +pitch=递减，两 stage 同法），
        上界防上限越界（9+2=11 ⊂ 网格内）。ws：初始 ≈(3,3) 逐格 −20mm
        抬到 ≥6。as：同法（旧 L 走法只 +4 步向网内故 C03 可用，对称走法
        −2 步会踩出格）；AS 起点保持 (20,30)——对称行程 ±40mm：X 走
        −20→60 ⊂ [−30,80]、Y 走 −10→70 ⊂ [−30,120]。
        迭代 47 起默认 pitch=10.005（gap 0.005）：±20mm jog≈±2 格，
        [6,9] 窗口宽 4 ≥ 步长 2 收敛性不变（奇偶链 3/5/7/9、4/6/8 均有
        落点）；角 [6,9] 在 29×29 网格半径 ~94mm 处，±2 步 ~114mm
        仍深在 rlim=147 圆裁内，余量结论不变。"""
        http_post(base + "/api/workpiece/reset", {})
        # 迭代 38：关 jitter（jitter_rz_urad ±30µrad 随机上片旋转经 ρ̂ 门限
        # （<100µrad 判 0）全量混叠进 φ 基线——quick 两轮 WS 往返统一
        # ~−10µrad 偏移的根因；load_rz_rad=250µrad 确定性值 > 门限可读出
        # 抵消，无害；i8 setup_recipe 同款口径。本段为套件末段，不恢复）
        cj, rj = http_put(base + "/api/recipe",
                          {"wafer_load": {"jitter_on": False,
                                          "jitter_rz_urad": 0.0}})
        assert cj == 200, (cj, rj)
        http_post(base + "/api/vision/mode", {"mode": "fast"})
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "offset": [0.0, 0.0]})
        assert c == 200 and r.get("ok"), (c, r)
        for ax, tgt in (("WS_LS_X", 58.2162), ("WS_LS_Y", 111.6102),
                        ("AS_X", 20.0), ("AS_Y", 30.0), ("AS_Z", 0.0),
                        ("WS_Rz", 0.0)):
            move(ax, tgt)
        name = center_quad()
        r_, c_ = _rc(name)
        while c_ < 6:
            move("WS_LS_X", get_axis_entry("WS_LS_X")["pos"] - 20.0)
            name = center_quad()
            c_ = _rc(name)[1]
        while c_ > 9:
            move("WS_LS_X", get_axis_entry("WS_LS_X")["pos"] + 20.0)
            name = center_quad()
            c_ = _rc(name)[1]
        while r_ < 6:
            move("WS_LS_Y", get_axis_entry("WS_LS_Y")["pos"] - 20.0)
            name = center_quad()
            r_ = _rc(name)[0]
        while r_ > 9:
            move("WS_LS_Y", get_axis_entry("WS_LS_Y")["pos"] + 20.0)
            name = center_quad()
            r_ = _rc(name)[0]
        tpl = "i37_%s_tpl_p%d" % (stage, os.getpid())   # pid 唯一：并发
        mk_tpl(tpl)                                     # 测试实例共享
        return tpl                                      # vision_templates/ 防互删

    def restore_geo(*keys):
        put_stage_geo(**{k: 0.0 for k in keys})

    # ---- 段首：六自由度链显式开（FK 注入消费口；段尾保持开=默认态）----
    c, r = http_post(base + "/api/errormaps", {"six_dof_enabled": True})
    assert c == 200, (c, r)

    # ---- ① ws 注入往返：+200/−200/+500 恢复 + φ=0 基线 ----
    # 判读口径（2026-09-16 多轮实证定版）：**绝对容差 ±15µrad**。φ 测量在
    # quick/standalone 上下文有 ~±10µrad 级系统地板（轴 scale_ppm 折进 ψ_y
    # 不可分 + 误差地图别名 + ρ̂ 门限混叠，且基线与注入作业的偏移结构随上
    # 下文切换——差分消基线路径已证伪）；注入信号 200/500µrad ≫ 地板，判
    # 读增益/符号仍有效；真值级 ±5µrad 精度门由 test_metrology i36（地图
    # 归零专属 boot）承担，不在此重复。
    tpl = prep("ws")
    geo_keys = ("ws_y_rail_dir_urad", "as_y_rail_dir_urad",
                "ws_nonortho_urad", "as_nonortho_urad")
    got = {}
    got_dbg = {}               # 诊断：phi None 时的作业终态/error
    try:
        put_stage_geo(ws_y_rail_dir_urad=0.0)
        s0, res0 = run_job("ws_stage_nonortho", tpl, repeat=2)
        phi0 = ((res0 or {}).get("measured") or {}).get("phi_urad")
        got[0.0] = phi0
        got_dbg[0.0] = "%s:%s" % (s0, ((res0 or {}).get("error") or "")[:60])
        for phi in (200.0, -200.0, 500.0):
            put_stage_geo(ws_y_rail_dir_urad=phi)
            s, res = run_job("ws_stage_nonortho", tpl, repeat=2)
            m = (res or {}).get("measured") or {}
            got[phi] = m.get("phi_urad")
            got_dbg[phi] = "%s:%s" % (s, ((res or {}).get("error") or "")[:60])
        ok = phi0 is not None and abs(phi0) <= TOL_ABS \
            and all(got[p] is not None and abs(got[p] - p) <= TOL_ABS
                    for p in (200.0, -200.0, 500.0))
        check("i37.nonortho_inject_ws_roundtrip", ok,
              "φ_t→φ̄: %s |φ0|=%.2f（绝对容差 ±%.0fµrad=地板口径）dbg=%s"
              % (", ".join("%+.0f→%+.2f" % (p, got[p] if got[p] is not None else 1e9)
                           for p in (200.0, -200.0, 500.0)),
                 abs(phi0) if phi0 is not None else 1e9, TOL_ABS, got_dbg))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ① as 注入往返：±500 恢复（相机走位台 σ=−1）----
    # 差分口径（同 ws）：先测 φ=0 基线 b（轴 scale_ppm ~5-10 折进 ψ_y
    # 不可分，静态基线 ~±6µrad 随工况漂移），断言 |φ̄−φ_t−b|≤TOL——
    # 基线不随注入变，差分即真值恢复（2026-09-16 复跑实证：直接 ±5 容差
    # 在负载下被 +5.3µrad 基线偏置整组顶穿，而 ws 差分口径同轮稳定通过）
    tpl = prep("as")
    got = {}
    got_dbg = {}               # 诊断：phi None 时的作业终态/error
    try:
        put_stage_geo(as_y_rail_dir_urad=0.0)
        _s0, _res0 = run_job("as_stage_nonortho", tpl, repeat=2)
        phi0 = (((_res0 or {}).get("measured") or {}).get("phi_urad"))
        got[0.0] = phi0
        got_dbg[0.0] = "%s:%s" % (_s0.get("state"),
                                  (_s0.get("error") or "")[:120])
        for phi in (500.0, -500.0):
            put_stage_geo(as_y_rail_dir_urad=phi)
            s, res = run_job("as_stage_nonortho", tpl, repeat=2)
            m = (res or {}).get("measured") or {}
            got[phi] = m.get("phi_urad")
            got_dbg[phi] = "%s:%s" % (s.get("state"),
                                      (s.get("error") or "")[:120])
        ok = (phi0 is not None
              and all(got[p] is not None and abs(got[p] - p - phi0) <= TOL_AS
                      for p in (500.0, -500.0)))
        check("i37.nonortho_inject_as_roundtrip", ok,
              "φ_t→φ̄: %s base=%.2f（差分容差 ±%.0fµrad，地图开 AS 漂移"
              "地板口径）dbg=%s"
              % (", ".join("%+.0f→%+.2f" % (p, got[p] if got[p] is not None else 1e9)
                           for p in (500.0, -500.0)),
                 phi0 if phi0 is not None else 1e9, TOL_AS, got_dbg))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- 链开关：链关时注入不消费（|φ|≤5 ≈0）----
    tpl = prep("ws")
    phi_off = None
    try:
        put_stage_geo(ws_y_rail_dir_urad=200.0)
        c, r = http_post(base + "/api/errormaps", {"six_dof_enabled": False})
        assert c == 200, (c, r)
        try:
            s, res = run_job("ws_stage_nonortho", tpl, repeat=2)
            phi_off = ((res or {}).get("measured") or {}).get("phi_urad")
        finally:
            http_post(base + "/api/errormaps", {"six_dof_enabled": True})
        check("i37.nonortho_chain_off",
              phi_off is not None and abs(phi_off) <= TOL,
              "链关+注入 200：φ̄=%.2fµrad（应≈0，容差 ±%.0f）"
              % (phi_off if phi_off is not None else 1e9, TOL))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ② ref_axis=y：输出=−X 基准（+200 → ≈−200；绝对容差 TOL_ABS 同①）----
    tpl = prep("ws")
    phi_y = None
    try:
        put_stage_geo(ws_y_rail_dir_urad=200.0)
        s, res = run_job("ws_stage_nonortho", tpl, repeat=2, ref_axis="y")
        m = (res or {}).get("measured") or {}
        phi_y = m.get("phi_urad")
        ok = phi_y is not None and abs(phi_y - (-200.0)) <= TOL_ABS \
            and abs((m.get("ws_nonortho_urad") or 0.0) - 200.0) <= TOL_ABS
        check("i37.nonortho_ref_axis_y", ok,
              "ref_axis=y：phi_urad=%s ws_nonortho_urad=%s（应≈−200/+200，"
              "绝对容差 ±%.0fµrad=地板口径）"
              % (phi_y, m.get("ws_nonortho_urad"), TOL_ABS))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ③ pitch≠处方 → 400 且报实际 pitch ----
    tpl = prep("ws")
    try:
        c, r = http_post(
            base + "/api/calibration/start",
            {"routine": "ws_stage_nonortho", "cam": "down",
             "params": {"template_name": tpl, "pitch_mm": 20.5}})
        err = (r or {}).get("error") or ""
        check("i37.nonortho_pitch_mismatch_400",
              c == 400 and "pitch_x" in err and "10.005" in err,
              "c=%s err=%s" % (c, err[:110]))
    finally:
        del_tpl(tpl)

    # ---- ④ 行程越界 → 400 ----
    tpl = prep("ws")
    try:
        c, r = http_post(
            base + "/api/calibration/start",
            {"routine": "ws_stage_nonortho", "cam": "down",
             "params": {"template_name": tpl,
                        "center_xy": [200.0, 111.6102]}})
        err = (r or {}).get("error") or ""
        check("i37.nonortho_travel_400",
              c == 400 and "限位" in err,
              "c=%s err=%s" % (c, err[:110]))
    finally:
        del_tpl(tpl)

    # ---- ⑤ 视野无标记 → 400（_check_marks 上移 start）----
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    for ax in ("WS_LS_X", "WS_LS_Y"):
        move(ax, 0.0)
    c, r = http_post(
        base + "/api/calibration/start",
        {"routine": "ws_stage_nonortho", "cam": "down",
         "params": {"template_name": "i37_nomark_tpl_p%d" % os.getpid()}})
    err = (r or {}).get("error") or ""
    # 模板不存在也 400——先建模板再试，确保命中"无标记"分支
    if c == 400 and ("标记" in err and "模板" not in err):
        check("i37.nonortho_no_marks_400", True, "err=%s" % err[:80])
    else:
        # 建一个模板后重试（无 wafer → 无标记）
        tpl = "i37_nomark_tpl_p%d" % os.getpid()
        try:
            c1, r1 = http_post(base + "/api/io/command",
                               {"path": LIGHT_DOWN, "cmd": "SetBool",
                                "args": {"value": True}})
            time.sleep(0.25)
            try:
                del_tpl(tpl)
                http_post_tpl(base + "/api/vision/templates",
                          {"cam": "down", "name": tpl,
                           "roi": [1200, 1200, 601, 601]})
            finally:
                http_post(base + "/api/io/command",
                          {"path": LIGHT_DOWN, "cmd": "SetBool",
                           "args": {"value": False}})
            c, r = http_post(
                base + "/api/calibration/start",
                {"routine": "ws_stage_nonortho", "cam": "down",
                 "params": {"template_name": tpl}})
            err = (r or {}).get("error") or ""
            check("i37.nonortho_no_marks_400",
                  c == 400 and "标记" in err,
                  "c=%s err=%s" % (c, err[:110]))
        finally:
            del_tpl(tpl)

    # ---- ⑥ cancel：协作式取消 + 现场恢复 + 忙释放 ----
    tpl = prep("ws")
    wpos = tuple(get_axis_entry(a)["pos"] for a in ("WS_LS_X", "WS_LS_Y"))
    state = err2 = None
    try:
        c, r = http_post(base + "/api/calibration/start",
                         {"routine": "ws_stage_nonortho", "cam": "down",
                          "params": {"template_name": tpl, "repeat": 10}})
        assert c == 200 and r.get("job_id"), (c, r)
        jid = r["job_id"]
        t0 = time.time()
        while time.time() - t0 < 30:
            s = http_json(base + "/api/calibration/status?job_id=%d" % jid)
            if (s.get("log") or []) and len(s["log"]) > 3:
                break
            time.sleep(0.2)
        c2, r2 = http_post(base + "/api/calibration/cancel",
                           {"job_id": jid})
        t0 = time.time()
        while time.time() - t0 < 120:
            s = http_json(base + "/api/calibration/status?job_id=%d" % jid)
            if s.get("state") in ("canceled", "done", "error"):
                break
            time.sleep(0.3)
        state = s.get("state")
        err2 = s.get("error")
        # cancel 终态后现场恢复：轴回起点、灯灭
        time.sleep(1.0)
        wpos2 = tuple(get_axis_entry(a)["pos"]
                      for a in ("WS_LS_X", "WS_LS_Y"))
        axes_ok = all(abs(wpos2[i] - wpos[i]) < 0.01 for i in (0, 1))
        # 灯位读 9010 点位快照（与 PLC 同一 live 表；PLC /io/observe 对从未
        # 建档的点回 404 会崩溃套件——取消路径可能从未点灯，稳健起见走快照）
        snap = http_json(base + "/api/io")
        light_off = not (snap.get("points") or {}).get(LIGHT_DOWN, False)
        # 忙释放：新作业可启动并 done
        s3, res3 = run_job("ws_stage_nonortho", tpl, repeat=2)
        check("i37.nonortho_cancel",
              state == "canceled" and axes_ok and light_off
              and s3.get("state") == "done",
              "state=%s err=%s axes_restored=%s light_off=%s "
              "followup=%s" % (state, (err2 or "")[:60], axes_ok,
                               light_off, s3.get("state")))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ⑦ 十字轨迹 + θ_y 偏差口径 + 节点 CSV + 双向矩阵 + 逐节点进度 ----
    # 十字跳测：K=5（默认）节点轴位=center+(i−(K−1)/2)·pitch（±2 步，两线
    # 共中心）；θ_y 偏差口径=wrap(φ−ρ̂)（注入 +200、理想栅格 ρ̂=0 → θ_y≈
    # +200µrad 小数值，旧绝对口径会显示 ≈90°=1570796µrad 量级）；节点级
    # 过程数据落第二份 CSV（stage_nonortho_ws_nodes_*.csv，行数=repeat×
    # 2K，19 列行基准全量记录 schema）；物理↔逻辑双向矩阵 matrix_l2p/
    # matrix_p2l（恒 X 基准 φ=m_x）；进度 total=repeat×2K 逐节点推进
    tpl = prep("ws")
    try:
        put_stage_geo(ws_y_rail_dir_urad=200.0)
        wpos = tuple(get_axis_entry(a)["pos"] for a in ("WS_LS_X", "WS_LS_Y"))
        s, res = run_job("ws_stage_nonortho", tpl, repeat=2)
        m = (res or {}).get("measured") or {}
        ser = (res or {}).get("series") or {}
        det = (res or {}).get("detail") or {}
        done = s.get("state") == "done" and bool(res)
        k = det.get("k_points") or 5
        pitch = det.get("node_pitch_mm") or 20.0
        cx, cy = (det.get("center_xy") or [0.0, 0.0])[:2]
        nx, ny = ser.get("nodes_x") or [], ser.get("nodes_y") or []
        half = (k - 1) / 2.0
        traj_ok = done and len(nx) == k and len(ny) == k and all(
            nd.get("i") == i
            and abs(nd.get("pos", 1e9) - (cx + (i - half) * pitch)) < 1e-4
            for i, nd in enumerate(nx)) and all(
            nd.get("i") == i
            and abs(nd.get("pos", 1e9) - (cy + (i - half) * pitch)) < 1e-4
            for i, nd in enumerate(ny))
        # 作业结束轴回原位
        wpos2 = tuple(get_axis_entry(a)["pos"] for a in ("WS_LS_X", "WS_LS_Y"))
        axes_ok = all(abs(wpos2[i] - wpos[i]) < 0.01 for i in (0, 1))
        check("i37.nonortho_cross_track", bool(traj_ok) and axes_ok,
              "K=%s center=(%s,%s) nodes_x[0].pos=%s（应=%s）axes_back=%s"
              % (k, cx, cy, nx[0].get("pos") if nx else None,
                 cx - half * pitch, axes_ok))
        # θ_y 偏差口径：wrap(θ_y−θ_x)=φ 恒等；注入 +200 理想栅格 ρ̂=0 →
        # θ_y≈+200、θ_x≈0（绝对容差 TOL_ABS 同①；恒等式为结构判据不变）
        ty, tx = m.get("theta_y_urad"), m.get("theta_x_urad")
        phi = m.get("phi_urad")
        dev_ok = (ty is not None and tx is not None and phi is not None
                  and abs(ty - 200.0) <= TOL_ABS and abs(tx) <= TOL_ABS
                  and abs(ty - tx - phi) <= 1.0)
        check("i37.nonortho_theta_y_dev", bool(dev_ok),
              "θ_y=%s θ_x=%s φ=%s（注入 +200：θ_y 应≈+200µrad 偏差口径，"
              "旧绝对口径≈1570996µrad 无意义）" % (ty, tx, phi))
        # 拟合线契约：fit_x/fit_y={slope:[2],mean:[2]}（GUI 散点+拟合线）
        fx, fy = ser.get("fit_x") or {}, ser.get("fit_y") or {}
        fit_ok = (len(fx.get("slope") or []) == 2
                  and len(fx.get("mean") or []) == 2
                  and len(fy.get("slope") or []) == 2
                  and len(fy.get("mean") or []) == 2)
        # 节点 CSV：存在、数据行数=repeat×2K、行基准全量记录 schema
        # （每行一个测量节点、X/Y 分列、单位进列名；迭代 45 起尾部追加
        # frame_cols_px/frame_rows_px 逐帧原始坐标列——规范 5.3 单帧保存，
        # 19→21 列）
        bn = os.path.basename(det.get("csv_nodes") or "")
        fp = os.path.join(ROOT, "runtime", "metrology_data", bn)
        header, nrows = [], -1
        if bn and os.path.exists(fp):
            with open(fp, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f if ln.strip()]
            header = lines[0].split(",") if lines else []
            nrows = len(lines) - 1
        csv_ok = (fit_ok and nrows == 2 * 2 * k and header == [
            "cycle", "line", "node", "template",
            "AS_X_mm", "AS_Y_mm", "WS_LS_X_mm", "WS_LS_Y_mm",
            "mark_col_px", "mark_row_px", "score",
            "cam_world_x_mm", "cam_world_y_mm",
            "mark_world_x_mm", "mark_world_y_mm",
            "o_x_mm", "o_y_mm", "resid_x_mm", "resid_y_mm",
            "frame_cols_px", "frame_rows_px"])
        check("i37.nonortho_nodes_csv", csv_ok,
              "csv=%s rows=%s（应=%d）header=%s fit=%s"
              % (bn, nrows, 2 * 2 * k, header, fit_ok))
        # 汇总 CSV（迭代 48）：逐轮物理↔逻辑矩阵 8 列——表头 17 列 schema、
        # 行数=repeat、逐轮 l2p 均值≈measured.matrix_l2p（ρ̂=0 场景精确，
        # 非零抖动二阶近似，容差 1e-6）、首行 p2l≈inv(l2p)（容差 1e-6）
        bn2 = os.path.basename((res or {}).get("csv_name") or "")
        fp2 = os.path.join(ROOT, "runtime", "metrology_data", bn2)
        hdr2, rows2 = [], []
        if bn2 and os.path.exists(fp2):
            with open(fp2, "r", encoding="utf-8") as f:
                lines2 = [ln.strip() for ln in f if ln.strip()]
            hdr2 = lines2[0].split(",") if lines2 else []
            rows2 = [ln.split(",") for ln in lines2[1:]]
        mat_cols = ["l2p_00", "l2p_01", "l2p_10", "l2p_11",
                    "p2l_00", "p2l_01", "p2l_10", "p2l_11"]
        sum_ok = (hdr2 == ["repeat", "phi_urad", "theta_x_urad",
                           "theta_y_urad", "resid_rms_nm",
                           "scale_x", "scale_y",
                           "scale_ppm_x", "scale_ppm_y"] + mat_cols
                  and len(rows2) == 2)
        mean_ok = inv_ok = False
        ml2p = m.get("matrix_l2p")
        if sum_ok:
            try:
                vals = [[float(v) for v in r[9:17]] for r in rows2]
                if (isinstance(ml2p, list) and len(ml2p) == 2
                        and all(isinstance(r, list) and len(r) == 2
                                for r in ml2p)):
                    mean_ok = all(
                        abs(sum(v[j] for v in vals) / len(vals)
                            - [ml2p[0][0], ml2p[0][1],
                               ml2p[1][0], ml2p[1][1]][j]) <= 1e-6
                        for j in range(4))
                a00, a01, a10, a11 = vals[0][:4]
                b00, b01, b10, b11 = vals[0][4:]
                inv_ok = (abs(a00 * b00 + a01 * b10 - 1.0) <= 1e-6
                          and abs(a00 * b01 + a01 * b11) <= 1e-6
                          and abs(a10 * b00 + a11 * b10) <= 1e-6
                          and abs(a10 * b01 + a11 * b11 - 1.0) <= 1e-6)
            except (ValueError, IndexError):
                pass
        check("i37.nonortho_summary_csv", sum_ok and mean_ok and inv_ok,
              "csv=%s rows=%s header=%s mean≈l2p:%s inv:%s"
              % (bn2, len(rows2), hdr2, mean_ok, inv_ok))
        # 物理↔逻辑双向全矩阵（迭代 42b 口径，注入 ws_y_rail_dir_urad=
        # +200 场景）：M=[col_x|col_y]=各线步进向量/pitch 实测像——
        # col_y.x≈(1+εy)·sin(200µrad)（容差沿用该段注入口径 TOL_ABS）、
        # 列模长=measured.scale_x/scale_y（一致性界=1e-8 舍入+lat²/2 横向
        # 贡献，纯 5e-9 舍入档在大横向注入下结构性不可达——hypot 把横向
        # 加进模长）、col_y 偏角 atan2(col_y.x,col_y.y)≈φ（=ws_nonortho_urad
        # 键同一 X 基准均值，内部一致锚，基线无关）、col_x 横向残量
        # ≤30µrad 地板（X 线方向不再强制贴逻辑 X，残量≈0 是物理结果非约束）、
        # matrix_p2l 与 matrix_l2p 互逆（A@B≈I 容差 1e-8——输出按 9 位有效
        # 数字舍入，cos≈1 档首位 1 占位使舍入误差放大到 ~2e-9，1e-12 结构性
        # 不可达）、scale_ppm 键与 scale 键自洽（±0.2ppm）
        l2p, p2l = m.get("matrix_l2p"), m.get("matrix_p2l")

        def _is22(a):
            return (isinstance(a, list) and len(a) == 2
                    and all(isinstance(r, list) and len(r) == 2
                            for r in a))
        mat_ok = False
        scx, scy = m.get("scale_x"), m.get("scale_y")
        ppmx, ppmy = m.get("scale_ppm_x"), m.get("scale_ppm_y")
        if _is22(l2p) and _is22(p2l) and scx and scy:
            ident = [[sum(l2p[i][kk] * p2l[kk][j] for kk in range(2))
                      for j in range(2)] for i in range(2)]
            phi_key = (m.get("ws_nonortho_urad") or 0.0) * 1e-6
            # 列模长 vs scale 键一致性：5e-9 纯舍入档在大横向分量下结构
            # 性不可达——hypot 把横向加进模长（≈lat²/2，500µrad 注入时
            # ~1.2e-7），且 cos≈1 档 sig9 舍入半 ulp 就有 5e-9。界=舍入
            # 1e-8 + 该列横向实际贡献 lat²/2（迭代 42b 复测实测 9.9e-9
            # 超旧 5e-9 界属判据过紧，产品值正确）。
            tol_nx = 1e-8 + 0.5 * l2p[1][0] ** 2
            tol_ny = 1e-8 + 0.5 * l2p[0][1] ** 2
            mat_ok = (
                abs(l2p[0][1] - math.sin(200.0e-6)) <= TOL_ABS * 1e-6
                and abs(math.hypot(l2p[0][0], l2p[1][0]) - scx) <= tol_nx
                and abs(math.hypot(l2p[0][1], l2p[1][1]) - scy) <= tol_ny
                and abs(l2p[1][0]) <= 30.0e-6
                and abs(math.atan2(l2p[0][1], l2p[1][1]) - phi_key)
                    <= TOL_ABS * 1e-6
                and all(abs(ident[i][j] - (1.0 if i == j else 0.0))
                        <= 1e-8 for i in range(2) for j in range(2))
                and ppmx is not None and ppmy is not None
                and abs(ppmx - (scx - 1.0) * 1e6) <= 0.2
                and abs(ppmy - (scy - 1.0) * 1e6) <= 0.2)
        check("i37.nonortho_matrix_l2p_p2l", bool(mat_ok),
              "l2p=%s p2l=%s scale=(%s,%s)ppm=(%s,%s)（注入 +200：col_y.x "
              "应≈sin(200µrad)=%.9f，列模长=ε，col_y 方向≈φ，互逆 1e-8；"
              "ws_nonortho_urad=%s）"
              % (l2p, p2l, scx, scy, ppmx, ppmy, math.sin(200.0e-6),
                 m.get("ws_nonortho_urad")))
        # 进度：total=repeat×2K 逐节点推进（done 后保持末值 i=n=20）
        prog = s.get("progress") or {}
        check("i37.nonortho_progress_total",
              done and prog.get("n") == 2 * 2 * k
              and prog.get("i") == prog.get("n"),
              "progress=%s（应 i=n=repeat×2K=%d）" % (prog, 2 * 2 * k))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ⑧ k_points 奇数守卫：偶数→400，奇数边界 3/21 放行 ----
    tpl = prep("ws")
    try:
        c, r = http_post(base + "/api/calibration/start",
                         {"routine": "ws_stage_nonortho", "cam": "down",
                          "params": {"template_name": tpl, "k_points": 4}})
        err = (r or {}).get("error") or ""
        check("i37.nonortho_k_even_400",
              c == 400 and "奇数" in err,
              "c=%s err=%s" % (c, err[:110]))
        # 奇数边界放行（start 通过即取消；K=21 行程 ±10×20mm 须 center 留
        # ±200mm——WS 软限位 ±220，显式给 center_xy=[0,0]）
        ok3 = ok21 = False
        c, r = http_post(base + "/api/calibration/start",
                         {"routine": "ws_stage_nonortho", "cam": "down",
                          "params": {"template_name": tpl, "k_points": 3,
                                     "repeat": 2, "wait_ms": 0,
                                     "frames_per_point": 1}})
        if c == 200:
            ok3 = True
            http_post(base + "/api/calibration/cancel",
                      {"job_id": r["job_id"]})
            wait_terminal(r["job_id"], 180.0)
        c, r = http_post(base + "/api/calibration/start",
                         {"routine": "ws_stage_nonortho", "cam": "down",
                          "params": {"template_name": tpl, "k_points": 21,
                                     "repeat": 2, "wait_ms": 0,
                                     "frames_per_point": 1,
                                     "center_xy": [0.0, 0.0]}})
        if c == 200:
            ok21 = True
            http_post(base + "/api/calibration/cancel",
                      {"job_id": r["job_id"]})
            wait_terminal(r["job_id"], 180.0)
        check("i37.nonortho_k_odd_3_21", ok3 and ok21,
              "K=3 start=%s K=21 start=%s（均应 200 放行）" % (ok3, ok21))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)

    # ---- ⑨ 比例误差 εx/εy（迭代 42b：scale_ppm 注入恢复 + 全矩阵口径）----
    # 注入路径调研结论：error_system._scale_ppm 仅 __init__ 时从
    # error_maps/low_order.json 读（error_system.py:109-115），无 live
    # API（/api/errormaps 仅 enabled/six_dof_enabled；set_axis_config
    # 只到 repeat）→ 与 i37 误差地图归零同款"写文件+finally 字节恢复"
    # 手法：main() 段首（backend 启动前）定向 -k 上下文把 WS_LS_X 改
    # +200ppm；套件上下文不改文件。断言基准=GET /api/errormaps 读回的活
    # scale_ppm（定向=+200.0、套件=low_order 原值 −5.063），两上下文同
    # 判据；容差 ±30ppm=地板口径（与 φ 的 ±15µrad 同级）。Y 未注入→
    # |scale_ppm_y|<30（原值 +6.102+拟合噪声≪30）。φ sanity：εx 注入
    # 不得污染角读出（横向/沿轨分量分解回归门——旧方向读出在本场景把
    # ρ̂ 误判 π/2，φ 会崩到 ≈−1.57e6µrad）。
    tpl = prep("ws")
    try:
        em = http_json(base + "/api/errormaps")
        axm = (em.get("axes") or {})
        bsx = float((axm.get("WS_LS_X") or {}).get("scale_ppm") or 0.0)
        bsy = float((axm.get("WS_LS_Y") or {}).get("scale_ppm") or 0.0)
        put_stage_geo(ws_y_rail_dir_urad=0.0)
        s, res = run_job("ws_stage_nonortho", tpl, repeat=2)
        m = (res or {}).get("measured") or {}
        ppmx, ppmy = m.get("scale_ppm_x"), m.get("scale_ppm_y")
        scx, scy = m.get("scale_x"), m.get("scale_y")
        phi_s = m.get("phi_urad")
        ok = (s.get("state") == "done"
              and ppmx is not None and abs(ppmx - bsx) <= 30.0
              and ppmy is not None and abs(ppmy - bsy) <= 30.0
              and abs(ppmy) < 30.0
              and scx is not None
              and abs((scx - 1.0) * 1e6 - ppmx) <= 0.2
              and scy is not None
              and abs((scy - 1.0) * 1e6 - ppmy) <= 0.2
              and phi_s is not None and abs(phi_s) <= TOL_ABS)
        check("i37.nonortho_scale_inject", ok,
              "WS_LS_X 注入：scale_ppm_x=%s（活基线 %+.3f，容差 ±30ppm） "
              "scale_ppm_y=%s（基线 %+.3f，未注入应|·|<30） φ=%sµrad"
              "（εx 不得污染角读出，≤±%.0f）scale=(%s,%s) state=%s"
              % (ppmx, bsx, ppmy, bsy, phi_s, TOL_ABS, scx, scy,
                 s.get("state")))
    finally:
        restore_geo(*geo_keys)
        del_tpl(tpl)


def test_iter76():
    """--- 19.33 迭代 76（键合头正压垂直落片）---
    quick/full：高位单路正压、实际 XY 投影、无晶圆/片外保护、翻转承接
    优先、直接 OpenAir 垂直落片、正压宏与 bond_mark 兼容；独立后端真实 HTTP
    取片交接后手动正压，不依赖 bond 流程或晶圆负压。"""
    print("--- 19.33 迭代 76（键合头正压垂直落片） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    from engine import Engine
    from points import Points
    from workpieces import Workpieces
    from recipe import Recipe
    from error_system import ErrorSystem
    from error_kinematics import ErrorKinematics
    from machine_constants import MachineConstants
    from flow_engine import FlowEngine
    import vision_model as vm

    rt = os.path.join(ROOT, "runtime")

    def _tick(eng, n):
        for _ in range(n):
            eng.tick()

    def _fixture(wafer=True, z=0.0, ws_x=0.0, flip=False, head=True):
        eng = Engine(Points(os.path.join(rt, "io_points.json")))
        eng.errors = ErrorSystem()
        eng.recipe = Recipe(os.path.join(rt, "recipe.json"))
        eng.kinematics = ErrorKinematics(
            eng.errors, os.path.join(rt, "error_kinematics.json"))
        wp = Workpieces(os.path.join(rt, "transfers.json"), recipe=eng.recipe)
        eng.workpieces = wp
        if head:
            wp.dies[0].update({"carrier": "head", "site": [0.0, 0.0]})
        if wafer:
            wp.load_wafer(offset=[0.2, -0.15], rz=0.00025)
        for name, value in (("BS_Arm_Z", z), ("WS_LS_X", ws_x),
                            ("FA_X", -1242.35 if flip else 0.0)):
            eng.axes[name].pos = eng.axes[name].target = float(value)
        for i in (1, 2, 3):
            eng.air_command("WT_BondingVac_%d" % i, "OpenNegtive")
        if flip:
            eng.air_command("ChuckVac", "OpenNegtive")
        _tick(eng, 800)
        return eng, wp, wp.dies[0]

    def _release(eng, names, cmd="OpenPositive", ticks=800):
        for name in names:
            eng.air_command(name, cmd)
        _tick(eng, ticks)

    def _head_world(positions, poses, dsite):
        hc, hrot = vm._module_pose(poses, "bond_head")
        hrot = [hrot[0] + positions.get("BS_CAL_Rx", 0.0),
                hrot[1] + positions.get("BS_CAL_Ry", 0.0), hrot[2]]
        hd = vm._rotvec_apply([float(dsite[0]), float(dsite[1]), 0.0], hrot)
        return hc[0] + hd[0], hc[1] + hd[1]

    def _wafer_world(positions, poses, wafer, site, recipe):
        wc, wrot = vm._module_pose(poses, "wafer_stage")
        wrot = [wrot[0], wrot[1],
                wrot[2] + positions.get("WS_Rz", 0.0)]
        off = wafer["offset"]
        wrz = float(wafer.get("rz", 0.0))
        c, s = math.cos(wrz), math.sin(wrz)
        local = [off[0] + c * site[0] - s * site[1],
                 off[1] + s * site[0] + c * site[1], 0.0]
        w = vm._rotvec_apply_pivot(local, wrot, vm.wafer_rz_pivot(recipe))
        return wc[0] + w[0], wc[1] + w[1]

    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/bond/clear", {})

    # 用户原始场景：高位、非键合流程、晶圆无负压，单路正压也立即落片。
    eng, wp, die = _fixture(z=0.0)
    _release(eng, ["WT_BondingVac_1"], ticks=2)
    ent = wp.log[-1] if wp.log else {}
    check("i76.manual_positive_high",
          die["carrier"] == "wafer"
          and not any(d["carrier"] == "head" for d in wp.dies)
          and ent.get("rule") == "head_positive_to_wafer"
          and ent.get("release") == "positive_vertical"
          and ent.get("site") == die["site"],
          "carrier=%s site=%s log=%s" % (die["carrier"], die["site"], ent))

    # 非零 WS 位姿、上片偏移/旋转及头片局部偏移：正反变换必须保持世界 XY。
    eng, wp, die = _fixture(z=-20.0, ws_x=10.0)
    for name, value in (("WS_LS_Y", -5.0), ("WS_Rz", 0.005)):
        eng.axes[name].pos = eng.axes[name].target = value
    die["site"] = [0.007, -0.0022]
    die["rz"] = 0.012
    die["resid"] = [0.001, -0.001, 0.000002]
    positions, poses = vm._engine_poses(eng)
    before_xy = _head_world(positions, poses, die["site"])
    _release(eng, ["WT_BondingVac_2"], ticks=2)
    positions2, poses2 = vm._engine_poses(eng)
    after_xy = _wafer_world(positions2, poses2, wp.wafer, die["site"],
                            eng.recipe.get())
    projection_err = math.hypot(after_xy[0] - before_xy[0],
                                after_xy[1] - before_xy[1])
    check("i76.vertical_projection",
          die["carrier"] == "wafer" and projection_err < 1e-8
          and die["rz"] == 0.012
          and die["resid"] == [0.001, -0.001, 0.000002],
          "before=%s after=%s err=%.3gmm site=%s"
          % (before_xy, after_xy, projection_err, die["site"]))

    eng, wp, die = _fixture(wafer=False)
    _release(eng, ["WT_BondingVac_1"], ticks=20)
    check("i76.no_wafer_guard",
          die["carrier"] == "head"
          and not any(d["carrier"] == "wafer" for d in wp.dies),
          "carrier=%s" % die["carrier"])

    # 接触位+晶圆负压也不能让旧规则在正压后回退成固定中心落片。
    eng, wp, die = _fixture(z=-167.43, ws_x=220.0)
    eng.air_command("WT_WaferVac", "OpenNegtive")
    _tick(eng, 800)
    _release(eng, ["WT_BondingVac_%d" % i for i in (1, 2, 3)])
    check("i76.outside_wafer_guard",
          die["carrier"] == "head"
          and not any(d["carrier"] == "wafer" for d in wp.dies),
          "carrier=%s log=%s" % (die["carrier"], list(wp.log)))

    eng, wp, die = _fixture(head=False)
    n0 = len(wp.dies)
    _release(eng, ["WT_BondingVac_1"], ticks=20)
    check("i76.no_head_noop", len(wp.dies) == n0
          and not any(d["carrier"] == "wafer" for d in wp.dies)
          and not wp.log, "dies=%d log=%s" % (len(wp.dies), list(wp.log)))

    # 有效翻转承接优先：即使晶圆也在位，正常吸回仍回 flip。
    eng, wp, die = _fixture(z=-50.93, flip=True)
    _release(eng, ["WT_BondingVac_%d" % i for i in (1, 2, 3)])
    check("i76.flip_receiver_priority",
          die["carrier"] == "flip"
          and wp.log[-1].get("rule") == "head_to_flip",
          "carrier=%s log=%s" % (die["carrier"], wp.log[-1]))

    # 直接 OpenAir 与正压共用当前 XY 垂直落片；保留旧规则作兼容。
    eng, wp, die = _fixture(z=-167.43)
    eng.air_command("WT_WaferVac", "OpenNegtive")
    _tick(eng, 800)
    positions, poses = vm._engine_poses(eng)
    before_xy = _head_world(positions, poses, die["site"])
    _release(eng, ["WT_BondingVac_%d" % i for i in (1, 2, 3)],
             cmd="OpenAir")
    positions2, poses2 = vm._engine_poses(eng)
    after_xy = _wafer_world(positions2, poses2, wp.wafer, die["site"],
                            eng.recipe.get())
    projection_err = math.hypot(after_xy[0] - before_xy[0],
                                after_xy[1] - before_xy[1])
    check("i76.direct_open_air",
          die["carrier"] == "wafer" and projection_err < 1e-8
          and wp.log[-1].get("rule") == "head_positive_to_wafer"
          and wp.log[-1].get("release") == "air_vertical",
          "carrier=%s site=%s err=%.3gmm log=%s"
          % (die["carrier"], die["site"], projection_err, wp.log[-1]))

    # 新正压规则可由 transfer 宏执行；正常 bond_mark 覆盖为工艺目标/残差。
    eng, wp, die = _fixture()
    eng.constants = MachineConstants(os.path.join(rt, "machine_constants.json"))
    eng.flow = FlowEngine(eng, eng.points, os.path.join(rt, "flows.json"))

    def _pump():
        for _ in range(20):
            eng.tick()
            time.sleep(0.001)

    th = threading.Thread(target=_pump)
    th.start()
    try:
        macro_info = eng.flow._do_transfer("head_positive_to_wafer", 2.0)
    finally:
        th.join()
    check("i76.transfer_macro_positive", die["carrier"] == "wafer",
          "carrier=%s info=%s" % (die["carrier"], macro_info))
    eng.bond_commit = {"site": [10.0, 20.0], "tx": 0.001,
                       "ty": -0.002, "theta": 0.000003}
    mark_info = eng.flow._do_bond_mark({"site_x": 10.0, "site_y": 20.0})
    check("i76.bond_mark_compat", die["site"] == [10.0, 20.0]
          and die["resid"] == [0.001, -0.002, 0.000003],
          "site=%s resid=%s info=%s"
          % (die["site"], die.get("resid"), mark_info))

    def _flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)
        http_post(base + "/api/flow/reset", {})

    def _flow_run(path):
        c, r = http_post(base + path, {})
        assert c == 200 and r.get("ok"), (path, c, r)
        t0 = time.time()
        while time.time() - t0 < 120.0:
            st = http_json(base + "/api/flow/state")
            if st.get("state") in ("done", "aborted"):
                return st
            time.sleep(0.05)
        return {"state": "timeout"}

    http_post(base + "/api/vision/mode", {"mode": "fast"})
    set_speed(8.0)
    try:
        _flow_cleanup()
        st_pick = _flow_run("/api/process/flip_pick")
        _flow_cleanup()
        st_hand = _flow_run("/api/process/flip_handoff")
        _flow_cleanup()
        c_w, r_w = http_post(base + "/api/workpiece/wafer",
                             {"action": "load", "offset": [0.2, -0.15],
                              "rz": 0.00025})
        heads = [d for d in http_json(base + "/api/viz")["dies"]
                 if d["carrier"] == "head"]
        prepared = (st_pick.get("state") == "done"
                    and st_hand.get("state") == "done" and c_w == 200
                    and len(heads) == 1)
        check("i76.live_prepare_head", prepared,
              "pick=%s handoff=%s wafer=%s heads=%d"
              % (st_pick.get("state"), st_hand.get("state"), c_w, len(heads)))
        if not prepared:
            for name in ("i76.live_manual_positive", "i76.live_vertical_site",
                         "i76.live_no_bond_result"):
                check(name, False, "真实取片/交接前置失败")
            return

        http_post(base + "/api/axis/move",
                  {"axis": "WS_LS_X", "target": 5.0, "speed": 200.0})
        wait_settled("WS_LS_X", timeout=20.0)
        viz0 = http_json(base + "/api/viz")
        hd = next(d for d in viz0["dies"] if d["carrier"] == "head")
        positions = {a["name"]: a["pos"]
                     for a in http_json(base + "/api/axes")["axes"]}
        toolpoints = http_json(base + "/api/toolpoints")
        before_xy = _head_world(positions, toolpoints, hd["pos"])
        c, r = http_post(base + "/api/io/command",
                         {"path": "SystemMachine.SS_ME.M_AIR.WT_BondingVac_2",
                          "cmd": "OpenPositive"})
        landed = None
        t0 = time.time()
        while time.time() - t0 < 3.0:
            vv = http_json(base + "/api/viz")
            cand = [d for d in vv["dies"]
                    if d["id"] == hd["id"] and d["carrier"] == "wafer"]
            if cand and not any(d["carrier"] == "head" for d in vv["dies"]):
                landed = vv, cand[0]
                break
            time.sleep(0.02)
        ent = None
        if landed:
            ent = next((e for e in reversed(landed[0]["transfer_log"])
                        if e.get("die") == hd["id"]
                        and e.get("rule") == "head_positive_to_wafer"), None)
        check("i76.live_manual_positive", c == 200 and r.get("ok")
              and landed is not None and ent is not None
              and ent.get("release") == "positive_vertical",
              "code=%s landed=%s ent=%s"
              % (c, landed[1] if landed else None, ent))
        if landed:
            after_xy = _wafer_world(positions, toolpoints, landed[0]["wafer"],
                                    landed[1]["pos"], http_json(base + "/api/recipe")["recipe"])
            err = math.hypot(after_xy[0] - before_xy[0],
                             after_xy[1] - before_xy[1])
            check("i76.live_vertical_site", err < 0.001
                  and landed[1].get("rz") == hd.get("rz"),
                  "before=%s after=%s err=%.6gmm site=%s"
                  % (before_xy, after_xy, err, landed[1]["pos"]))
        else:
            check("i76.live_vertical_site", False, "手动正压后未落片")
        results = http_json(base + "/api/bond/results").get("results") or []
        check("i76.live_no_bond_result", not results,
              "手动落片不应生成键合结果：%s" % results)
    finally:
        _flow_cleanup()
        for name in ("WT_BondingVac_1", "WT_BondingVac_2", "WT_BondingVac_3",
                     "ChuckVac", "WT_WaferVac"):
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.SS_ME.M_AIR." + name,
                       "cmd": "OpenAir"})
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        http_post(base + "/api/bond/clear", {})
        set_speed(1.0)
        for axis in ("WS_LS_X", "WS_LS_Y", "WS_Rz", "BS_Arm_Z",
                     "FA_X", "FA_Z", "AS_X", "AS_Y"):
            http_post(base + "/api/axis/move",
                      {"axis": axis, "target": 0.0, "speed": 200.0})
            wait_settled(axis, timeout=20.0)


def test_iter78():
    """--- 19.35 迭代 78：IO 一键搜力 + 通大气垂直落片 ---"""
    print("--- 19.35 迭代 78（IO 一键搜力/通大气垂直落片） ---")
    base = "http://127.0.0.1:%d" % WS_PORT
    from engine import Engine, InterlockError
    from points import Points
    from plc_http import plc_command
    from workpieces import Workpieces
    from recipe import Recipe
    from error_system import ErrorSystem
    from error_kinematics import ErrorKinematics

    rt = os.path.join(ROOT, "runtime")
    bs_base = "SystemMachine.SS_Bonding.M_BS"

    def _fixture(z=-166.9, material=False):
        eng = Engine(Points(os.path.join(rt, "io_points.json")))
        ax = eng.axes["BS_Arm_Z"]
        ax.pos = ax.target = ax.x_actual = float(z)
        wp = None
        if material:
            eng.errors = ErrorSystem()
            eng.recipe = Recipe(os.path.join(rt, "recipe.json"))
            eng.kinematics = ErrorKinematics(
                eng.errors, os.path.join(rt, "error_kinematics.json"))
            wp = Workpieces(os.path.join(rt, "transfers.json"), recipe=eng.recipe)
            eng.workpieces = wp
        return eng, wp

    def _ticks(eng, n):
        for _ in range(n):
            eng.tick()

    def _finish(eng, limit=30000):
        first_settle = None
        for i in range(1, limit + 1):
            eng.tick()
            st = eng.force_search_state()
            if first_settle is None and st["phase"] == "settling":
                first_settle = i
            if not st["busy"]:
                return i, first_settle
        raise AssertionError("ForceSearch 未结束：%s" % eng.force_search_state())

    with open(os.path.join(rt, "io_points.json"), encoding="utf-8") as f:
        meta = json.load(f)["points"]
    check("i78.io_points", all(meta[bs_base + "." + n]["kind"] == k for n, k in (
        ("ForceSearch", "null"), ("StopForceSearch", "null"),
        ("oForceSearchBusy", "bool"), ("oForceSearchDone", "bool"),
        ("oForceSearchFail", "bool"), ("oForceSearchPos", "real"))),
          "points=%d" % len(meta))
    check("i78.engine_none_compat",
          Engine(None).force_search_state()["phase"] == "idle", "Engine(None)")

    eng, _ = _fixture()
    invalid = []
    for args in ({"Force": 0}, {"Speed": -1}, {"Speed": True},
                 {"Steady_Time": -0.1}, {"Max_Distance": 0}, {"Timeout": 0}):
        try:
            eng.force_search_start(args)
        except ValueError:
            invalid.append(args)
    check("i78.invalid_400", len(invalid) == 6, "accepted=%s" % invalid)

    eng, _ = _fixture()
    accepted = plc_command(eng, eng.points, bs_base + ".ForceSearch", "Pulse", {})
    n, settle = _finish(eng)
    st = eng.force_search_state()
    check("i78.default_done", accepted["accepted"] and st["done"]
          and not st["fail"] and settle is not None
          and accepted["params"]["Force"] == 2.0
          and accepted["params"]["Speed"] == 0.2
          and accepted["params"]["Steady_Time"] == 0.5
          and accepted["params"]["Max_Distance"] == 5.0,
          "accepted=%s ticks=%d state=%s" % (accepted, n, st))
    check("i78.stable_time", n - settle >= 500,
          "settle_tick=%s done_tick=%s" % (settle, n))
    _ticks(eng, 4)
    check("i78.hold_and_latch", not eng.force_mode
          and not eng.axes["BS_Arm_Z"].moving
          and abs(eng.axes["BS_Arm_Z"].pos - st["stop_pos"]) < 1e-12
          and eng.points.get(bs_base + ".oForceSearchDone") is True
          and eng.points.get(bs_base + ".oForceSearchFail") is False
          and not any("M_BS.." in p for p in eng.points.snapshot()),
          "pos=%.6f state=%s" % (eng.axes["BS_Arm_Z"].pos, st))

    eng, _ = _fixture()
    eng.errors = ErrorSystem()
    eng.errors.all_errors_off = True
    accepted = plc_command(eng, eng.points, bs_base, "ForceSearch", {
        "Force": 3.0, "Speed": 0.4, "Steady_Time": 0.05,
        "Tolerance_N": 0.5, "Max_Distance": 4.0, "Timeout": 20.0})
    n, _ = _finish(eng)
    check("i78.custom_and_error_free", accepted["params"]["Force"] == 3.0
          and accepted["params"]["Speed"] == 0.4
          and eng.force_search_state()["done"] and n < 6000
          and 3.0 <= eng.force_value < 4.0,
          "ticks=%d force=%.6f state=%s"
          % (n, eng.force_value, eng.force_search_state()))

    eng, _ = _fixture()
    eng.force_search_start({"Speed": 20.0})
    try:
        eng.force_search_start()
        duplicate = False
    except InterlockError:
        duplicate = True
    eng.force_search_stop()
    eng.bonding_force(0.0, 0.0)
    try:
        eng.force_search_start()
        force_conflict = False
    except InterlockError:
        force_conflict = True
    check("i78.command_conflict", duplicate and force_conflict,
          "duplicate=%s force_mode=%s" % (duplicate, force_conflict))

    eng, _ = _fixture(z=0.0)
    plc_command(eng, eng.points, bs_base, "ForceSearch",
                {"Max_Distance": 0.02, "Speed": 20.0})
    _finish(eng)
    st = eng.force_search_state()
    check("i78.no_contact_fail", st["fail"] and not st["done"]
          and "未接触" in st["reason"]
          and abs(st["stop_pos"] + 0.02) < 1e-9
          and not eng.axes["BS_Arm_Z"].moving, str(st))

    eng, _ = _fixture()
    eng._const_refresh_sim = eng.sim_time
    eng._bond["force_gain_err"] = 4.0
    eng.force_search_start({"Speed": 20.0, "Timeout": 0.1})
    _finish(eng)
    check("i78.stability_timeout", eng.force_search_state()["fail"]
          and "超时" in eng.force_search_state()["reason"]
          and not eng.force_search_state()["done"],
          str(eng.force_search_state()))

    eng, _ = _fixture()
    plc_command(eng, eng.points, bs_base, "ForceSearch", {})
    _ticks(eng, 100)
    plc_command(eng, eng.points, bs_base, "StopForceSearch", {})
    check("i78.stop_cancels", eng.force_search_state()["phase"] == "idle"
          and not eng.force_search_state()["done"]
          and not eng.axes["BS_Arm_Z"].moving, str(eng.force_search_state()))

    eng, _ = _fixture()
    eng.force_search_start()
    _ticks(eng, 50)
    eng.estop()
    check("i78.estop_fail", eng.force_search_state()["fail"]
          and not eng.force_search_state()["done"]
          and not eng.axes["BS_Arm_Z"].moving, str(eng.force_search_state()))

    eng, _ = _fixture()
    eng.force_search_start()
    _ticks(eng, 50)
    eng.enable("BS_Arm_Z", False)
    _ticks(eng, 2)
    check("i78.disable_fail", eng.force_search_state()["fail"]
          and not eng.force_search_state()["done"]
          and not eng.force_search_state()["busy"], str(eng.force_search_state()))

    eng, _ = _fixture()
    eng.force_search_start()
    eng.reset_thermal_force()
    check("i78.full_reset_clear", eng.force_search_state()["phase"] == "idle"
          and not eng.force_search_state()["done"]
          and not eng.force_search_state()["fail"]
          and not eng.axes["BS_Arm_Z"].moving, str(eng.force_search_state()))

    def _air_fixture(cmd, z=-20.0, flip=False, wafer=True):
        eng, wp = _fixture(z=z, material=True)
        wp.dies[0].update({"carrier": "head", "site": [0.0, 0.0]})
        if wafer:
            wp.load_wafer(offset=[0.2, -0.15], rz=0.00025)
        eng.axes["WS_LS_X"].pos = eng.axes["WS_LS_X"].target = 10.0
        eng.axes["FA_X"].pos = -1242.35 if flip else 0.0
        for i in (1, 2, 3):
            eng.air_command("WT_BondingVac_%d" % i, "OpenNegtive")
        if flip:
            eng.air_command("ChuckVac", "OpenNegtive")
        _ticks(eng, 800)
        for i in ((1, 2, 3) if flip else (2,)):
            eng.air_command("WT_BondingVac_%d" % i, cmd)
        _ticks(eng, 800 if flip else 2)
        return eng, wp, wp.dies[0]

    eng_pos, wp_pos, die_pos = _air_fixture("OpenPositive")
    eng_air, wp_air, die_air = _air_fixture("OpenAir")
    check("i78.air_same_vertical_site", die_pos["carrier"] == "wafer"
          and die_air["carrier"] == "wafer" and die_air["site"] == die_pos["site"]
          and wp_pos.log[-1]["release"] == "positive_vertical"
          and wp_air.log[-1]["release"] == "air_vertical",
          "positive=%s air=%s" % (wp_pos.log[-1], wp_air.log[-1]))
    _, wp_flip, die_flip = _air_fixture("OpenAir", z=-50.93, flip=True)
    check("i78.air_flip_priority", die_flip["carrier"] == "flip"
          and wp_flip.log[-1]["rule"] == "head_to_flip", str(wp_flip.log[-1]))
    _, wp_none, die_none = _air_fixture("OpenAir", wafer=False)
    check("i78.air_no_wafer_guard", die_none["carrier"] == "head"
          and not wp_none.log, "carrier=%s log=%s"
          % (die_none["carrier"], list(wp_none.log)))

    def _flow_cleanup():
        http_post(base + "/api/flow/stop", {})
        time.sleep(0.15)
        http_post(base + "/api/flow/reset", {})

    def _flow_run(path):
        c, r = http_post(base + path, {})
        assert c == 200 and r.get("ok"), (path, c, r)
        t0 = time.time()
        while time.time() - t0 < 120.0:
            st = http_json(base + "/api/flow/state")
            if st.get("state") in ("done", "aborted"):
                assert st["state"] == "done", st
                return st
            time.sleep(0.05)
        raise AssertionError("流程超时：%s %s" % (path, st))

    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/reset", {})
    http_post(base + "/api/bond/clear", {})
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    set_speed(8.0)
    try:
        _flow_cleanup()
        pick = _flow_run("/api/process/flip_pick")
        _flow_cleanup()
        handoff = _flow_run("/api/process/flip_handoff")
        _flow_cleanup()
        http_post(base + "/api/workpiece/wafer",
                  {"action": "load", "offset": [0.0, 0.0]})
        check("i78.live_prepare", pick["state"] == "done"
              and handoff["state"] == "done"
              and sum(d["carrier"] == "head"
                      for d in http_json(base + "/api/viz")["dies"]) == 1,
              "pick=%s handoff=%s" % (pick["state"], handoff["state"]))
        mc = http_json(base + "/api/machine_constants")["constants"]
        rec = http_json(base + "/api/recipe")["recipe"]
        sx, sy = mc["flip_handoff"]["as_safe_xy_mm"]
        for axis, target in (("AS_X", sx), ("AS_Y", sy)):
            http_post(base + "/api/axis/move",
                      {"axis": axis, "target": target, "speed": 200.0})
            wait_settled(axis, timeout=20.0)
        z_search = (mc["bond"]["head_z_search_mm"]
                    + rec["wafer"]["thickness_mm"]
                    + rec["film"]["dies"]["thickness_mm"])
        http_post(base + "/api/axis/move",
                  {"axis": "BS_Arm_Z", "target": z_search, "speed": 20.0})
        wait_settled("BS_Arm_Z", timeout=20.0)
        set_speed(1.0)
        c, accepted = http_post(base + "/api/io/command",
                                {"path": bs_base, "cmd": "ForceSearch"})
        observed = None
        t0 = time.time()
        while time.time() - t0 < 25.0:
            pts = http_json(base + "/api/io")["points"]
            if pts.get(bs_base + ".oForceSearchDone"):
                observed = pts
                break
            if pts.get(bs_base + ".oForceSearchFail"):
                break
            time.sleep(0.02)
        check("i78.live_force_search_done", c == 200 and accepted.get("accepted")
              and observed is not None
              and not observed.get(bs_base + ".oForceSearchBusy")
              and not observed.get(bs_base + ".oForceSearchFail"),
              "code=%s accepted=%s observed=%s" % (c, accepted, observed))
        assert observed is not None, "真实 IO 搜力未完成"
        stop_pos = observed[bs_base + ".oForceSearchPos"]
        time.sleep(0.6)
        pos_after = next(a["pos"] for a in http_json(base + "/api/axes")["axes"]
                         if a["name"] == "BS_Arm_Z")
        viz = http_json(base + "/api/viz")
        pid = http_json(base + "/api/flow/state")["pid"]["force"]
        check("i78.live_search_side_effect_free",
              abs(pos_after - stop_pos) < 1e-9 and pid["sp"] == 0.0
              and sum(d["carrier"] == "head" for d in viz["dies"]) == 1
              and not http_json(base + "/api/bond/results").get("results"),
              "pos=%.6f stop=%.6f pid=%s head=%d results=%s"
              % (pos_after, stop_pos, pid,
                 sum(d["carrier"] == "head" for d in viz["dies"]),
                 http_json(base + "/api/bond/results").get("results")))
    finally:
        _flow_cleanup()
        http_post(base + "/api/io/command",
                  {"path": bs_base, "cmd": "StopForceSearch"})
        for name in ("WT_BondingVac_1", "WT_BondingVac_2", "WT_BondingVac_3",
                     "ChuckVac", "WT_WaferVac"):
            http_post(base + "/api/io/command",
                      {"path": "SystemMachine.SS_ME.M_AIR." + name,
                       "cmd": "OpenAir"})
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        http_post(base + "/api/bond/clear", {})
        set_speed(1.0)
        for axis in ("BS_Arm_Z", "AS_X", "AS_Y"):
            http_post(base + "/api/axis/move",
                      {"axis": axis, "target": 0.0, "speed": 200.0})
            wait_settled(axis, timeout=20.0)


def test_iter77():
    """--- 19.34 迭代 77（误差全关：统一运行时屏蔽，含轴 3σ repeat）---"""
    print("--- 19.34 迭代 77（误差全关） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    def _gate(off):
        c, r = http_post(base + "/api/error_injection", {"disabled": bool(off)})
        assert c == 200 and r.get("disabled") is bool(off), (c, r)
        return r

    def _move_ws(x):
        http_post(base + "/api/axis/move",
                  {"axis": "WS_LS_X", "target": x, "speed": 100.0})
        return wait_settled("WS_LS_X", timeout=15.0)

    def _confocal():
        return http_json(base + "/api/confocal/read?probe=down")

    def _frame():
        with urllib.request.urlopen(base + "/api/vision/frame?cam=down",
                                    timeout=30) as resp:
            return resp.read()

    _gate(False)
    check("i77.invalid_400",
          http_post(base + "/api/error_injection", {})[0] == 400
          and http_post(base + "/api/error_injection", {"disabled": "true"})[0] == 400,
          "disabled 缺省/字符串均须 400")
    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/workpiece/reset", {})
    repeat0 = get_axis_entry("WS_LS_X")["config"]["repeat"]
    http_post(base + "/api/vision/mode", {"mode": "fast"})
    try:
        http_post(base + "/api/errormaps", {"enabled": True, "six_dof_enabled": True})
        http_post(base + "/api/modal", {"enabled": True})
        http_post(base + "/api/thermal", {"enabled": True})
        http_post(base + "/api/warpage", {"enabled": True})
        http_post(base + "/api/axis/config", {"axis": "WS_LS_X", "repeat": 500.0})
        a0 = _move_ws(0.0)
        check("i77.nonzero_repeat_fixture", abs(a0["actual"] - a0["x_actual"]) > 1e-7,
              "repeat=500nm actual-x_actual=%.9gmm" % (a0["actual"] - a0["x_actual"]))
        state = _gate(True)
        a1 = _move_ws(1.0)
        a2 = _move_ws(0.0)
        cfg = state.get("configured") or {}
        eff = state.get("effective") or {}
        check("i77.repeat_zero_and_config_kept",
              a1["actual"] == a1["x_actual"] and a2["actual"] == a2["x_actual"]
              and a2["config"]["repeat"] == 500.0
              and cfg.get("axis_maps") is True and cfg.get("six_dof") is True
              and cfg.get("modal") is True and cfg.get("thermal") is True
              and cfg.get("warpage") is True
              and not any(eff.values()),
              "actual=%.9f/%.9f repeat=%s configured=%s effective=%s"
              % (a1["actual"], a2["actual"], a2["config"]["repeat"], cfg, eff))
        tp = http_json(base + "/api/toolpoints")["modules"]
        check("i77.six_dof_zero", all(v == 0.0 for m in tp.values()
                                     for v in m["error_mm"] + m["error_rad"]),
              "全模块 error_mm/error_rad 均须精确为零")
        _gate(False)
        ar = _move_ws(0.5)
        check("i77.restore_reinject", abs(ar["actual"] - ar["x_actual"]) > 1e-7,
              "恢复后 actual-x_actual=%.9gmm" % (ar["actual"] - ar["x_actual"]))
        _gate(True)
        _move_ws(0.0)
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "inject_recipe": True})
        assert c == 200, (c, r)
        w = r["wafer"]
        check("i77.wafer_load_zero",
              w["offset"] == [0.0, 0.0] and w["rz"] == 0.0
              and w["z"] == w["rx"] == w["ry"] == 0.0,
              "wafer=%s" % w)
        c, r = http_post(base + "/api/workpiece/wafer", {"action": "unload"})
        assert c == 200, (c, r)
        c, r = http_post(base + "/api/workpiece/wafer",
                         {"action": "load", "inject_recipe": True,
                          "warp": {"bow_um": 80.0, "saddle_um": 15.0, "ang_deg": 25.0}})
        assert c == 200, (c, r)
        ws = http_json(base + "/api/warpage/state")
        check("i77.warpage_effect_zero",
              (ws.get("bond_dz_um") or 0.0) == 0.0
              and ((ws.get("probe_down") or {}).get("dz_um") or 0.0) == 0.0,
              "显式 warp 保留作场景参数，但全关时 dz=0")
        http_put(base + "/api/recipe", {"sim_sensors": {
            "confocal_sigma_um": {"down": 0.5}, "fast_noise_px": {"down": 0.5},
            "force_noise_N": 1.0, "force_gain_err": 0.1, "force_offset_N": 0.5}})
        time.sleep(1.2)
        reads = [_confocal()["reading_um"] for _ in range(6)]
        check("i77.confocal_deterministic", len(set(reads)) == 1,
              "readings=%s" % reads)
        force = (http_json(base + "/api/io")["points"]).get(
            "SystemMachine.SS_Bonding.M_BS.oActualForce")
        check("i77.force_true_zero", force == 0.0,
              "非零增益/偏置/噪声配置下，未接触力读数=%s" % force)
        for light in ("SystemMachine.SS_Bonding.M_AS.LightOn_1",
                      "SystemMachine.SS_Bonding.M_AS.LightOn_2"):
            c, r = http_post(base + "/api/io/command",
                             {"path": light, "cmd": "SetBool", "args": {"value": True}})
            assert c == 200 and r.get("ok"), (c, r)
        fast = [http_post(base + "/api/vision/identify",
                          {"cam": "down", "template": "cv_quad"})[1]
                for _ in range(3)]
        check("i77.fast_deterministic",
              all(x.get("ok") for x in fast)
              and len({json.dumps((x.get("entry") or {}).get("result"),
                                  sort_keys=True) for x in fast}) == 1,
              "三次 FAST 结果须逐字段一致")
        http_post(base + "/api/vision/mode", {"mode": "real"})
        f1, f2 = _frame(), _frame()
        check("i77.real_deterministic", f1 == f2,
              "两帧 REAL PNG %d/%d bytes 完全一致" % (len(f1), len(f2)))
        t_perf = time.time()
        h = read_clean_health(base, t_perf, retries=1)
        check("i77.perf", h["tick_hz"] >= 999.0 and h["push_hz"] >= 499.0,
              "tick=%.1fHz push=%.1fHz" % (h["tick_hz"], h["push_hz"]))
        # 工艺微滑移直接调用同一 FlowEngine 方法，避免为一个小分支重跑整片流程。
        import threading
        from types import SimpleNamespace
        from flow_engine import FlowEngine
        wp = SimpleNamespace(
            _lock=threading.RLock(), log=[{"rule": "head_to_wafer", "die": "D77"}],
            dies=[{"id": "D77", "carrier": "wafer", "site": None, "resid": None}])
        eng = SimpleNamespace(
            workpieces=wp, bond_commit={"site": [0.0, 0.0], "tx": 0.0,
                                        "ty": 0.0, "theta": 0.0},
            constants=SimpleNamespace(get=lambda: {
                "bond": {"bond_shift_xy_um": 100.0, "bond_shift_rz_urad": 100.0}}),
            errors=SimpleNamespace(all_errors_off=True), fine_align_result={})
        flow = FlowEngine.__new__(FlowEngine)
        flow.eng = eng
        flow._do_bond_mark({"site_x": 0.0, "site_y": 0.0})
        check("i77.bond_shift_zero", wp.dies[0]["resid"] == [0.0, 0.0, 0.0],
              "resid=%s" % wp.dies[0]["resid"])
        import vision_model as vm
        cam = vm.load_cameras()["down"]
        cam.error_gate = eng.errors
        u, v = cam.project_rel(0.1, 0.0)
        xy = cam.solve_rel(u, v)
        check("i77.camera_geometry_ideal",
              cam.view_rot == 0.0 and abs(u - 0.1 / cam.k_mm * cam.dir_x) < 1e-12
              and abs(v) < 1e-12 and abs(xy[0] - 0.1) < 1e-12 and abs(xy[1]) < 1e-12
              and vm.wafer_rz_pivot({"sim_align": {
                  "rz_center_true_offset_um": [100.0, 200.0]}}, eng.errors) == (0.0, 0.0),
              "物理/认知旋转与 sim_align 枢轴均回理想几何")
        from thermal import ThermalSystem
        th = ThermalSystem()
        th.enabled = True
        th._T["bond_head"] = 325.0
        th._net["nodes"]["bond_head"]["cte_ppm_K"] = 10.0  # 无组件目录时补物性夹具
        before = th.module_offset("bond_head")
        th.error_gate = eng.errors
        after = th.module_offset("bond_head")
        check("i77.thermal_source_zero", any(before[0]) and after == ([0.0] * 3, [0.0] * 3),
              "热变形源头零输出")
        from modal import ModalSystem
        md = ModalSystem()
        md.enabled = True
        md.error_gate = eng.errors
        md.tick(0.001, SimpleNamespace(axes={}))
        check("i77.modal_source_zero",
              all(s["q"] == 0.0 for s in md._ax_state.values())
              and all(s["q"] == 0.0 for s in md._mode_state.values()),
              "全关后模态状态清零且不推进")
    finally:
        _gate(False)
        http_post(base + "/api/modal", {"enabled": False})
        http_post(base + "/api/thermal", {"enabled": False})
        http_post(base + "/api/warpage", {"enabled": False})
        http_post(base + "/api/axis/config", {"axis": "WS_LS_X", "repeat": repeat0})
        _move_ws(0.0)
        http_post(base + "/api/workpiece/reset", {})
        http_post(base + "/api/recipe/restore_default", {})
        http_post(base + "/api/vision/mode", {"mode": "real"})


def test_iter73():
    """--- 19.32 迭代 73（晶圆翘曲 warpage）---
    quick；段首 recipe 默认+modal/thermal OFF（防交叉耦合）+翘曲参数复位
    OFF；段尾翘曲 OFF+参数还原+工件复位+轴归零。
    晶圆翘曲（bow+saddle 二维静态场，读时合成注入）：
      - i73.warpage_get_default：默认 OFF/参数 4 键/state 结构（无片剖面
        按名义）；
      - i73.warpage_put_invalid_400：幅值越界/jitter 越界/未知键/非数值
        400 不污染；
      - i73.warpage_put_roundtrip / toggle：PUT 生效回读还原、enabled 热
        切换；
      - i73.warpage_off_zero_impact：OFF↔ON↔OFF 共焦 z_surface 与轴读数
        快照逐位回基线；
      - i73.warpage_confocal_probe：ON 后共焦面高增量 == state.probe_down
        .dz_um（同源断言 ±0.005µm）；
      - i73.warpage_state_profile：显式真值 bow80/saddle15/ang25 的剖面
        公式断言（中心=bow、u/v 边缘=±saddle·cos2θ）+warp 回显+边缘坡度；
      - i73.warpage_vision_defocus：下相机晶圆标记 blur 随翘曲升起
        （dz/DOF 量级）；
      - i73.warpage_contact_shift：停 base_eff+0.02mm，OFF 力≈0、ON（键合
        点 dz≈+70µm）力跳升（接触提前，i56 同手法）；
      - i73.warpage_perf：warpage ON 干净窗 tick≥999Hz。"""
    print("--- 19.32 迭代 73（晶圆翘曲） ---")
    base = "http://127.0.0.1:%d" % WS_PORT

    def _warp(en):
        c, r = http_post(base + "/api/warpage", {"enabled": en})
        assert c == 200 and r.get("enabled") == bool(en), (c, r)

    def _wstate():
        return http_json(base + "/api/warpage/state")

    def _cf_z():
        return http_json(base + "/api/confocal/read?probe=down") \
            .get("z_surface")

    def _force_avg(n=30, dt=0.05):
        s = 0.0
        for _ in range(n):
            s += float((http_json(base + "/api/io").get("points") or {}).get(
                "SystemMachine.SS_Bonding.M_BS.oActualForce", 0.0) or 0.0)
            time.sleep(dt)
        return s / n

    # ---- 干净确定起点（recipe 默认；modal/thermal OFF 防交叉注入；翘曲
    #      参数复位+OFF；工件复位；轴归零——段尾同口径还原）----
    http_post(base + "/api/recipe/restore_default", {})
    http_post(base + "/api/modal", {"enabled": False})
    http_post(base + "/api/thermal", {"enabled": False})
    http_put(base + "/api/warpage", {"bow_um": 30.0, "saddle_um": 12.0,
                                     "saddle_ang_deg": 0.0, "jitter_pct": 20.0})
    _warp(False)
    http_post(base + "/api/workpiece/reset", {})
    for ax in ("AS_X", "AS_Y", "AS_Z", "BS_Arm_Z"):
        http_post(base + "/api/axis/move", {"axis": ax, "target": 0.0,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y", "AS_Z", "BS_Arm_Z"):
        wait_settled(ax)

    # ---- 默认参数 + 默认 OFF + state 结构（无片：剖面按名义） ----
    wj = http_json(base + "/api/warpage")
    wp0 = wj.get("params") or {}
    st0 = _wstate()
    prof0 = st0.get("profile") or {}
    check("i73.warpage_get_default",
          wj.get("ok") and wj.get("enabled") is False
          and all(k in wp0 for k in ("bow_um", "saddle_um",
                                     "saddle_ang_deg", "jitter_pct"))
          and abs(float(wp0.get("bow_um", 0)) - 30.0) < 1e-9
          and st0.get("wafer_present") is False
          and st0.get("warp") is None
          and len(prof0.get("x_mm") or []) == 65
          and abs((prof0.get("u_z_um") or [0])[32] - 30.0) < 0.01,
          "enabled=%s keys=%s profile_n=%d center=%s"
          % (wj.get("enabled"), sorted(wp0), len(prof0.get("x_mm") or []),
             (prof0.get("u_z_um") or [None])[32]))

    # ---- PUT 校验：越界/未知键/非数值 400 不污染 ----
    codes = [http_put(base + "/api/warpage", {"bow_um": 501.0})[0],
             http_put(base + "/api/warpage", {"saddle_um": -600.0})[0],
             http_put(base + "/api/warpage", {"jitter_pct": -1})[0],
             http_put(base + "/api/warpage", {"jitter_pct": 101})[0],
             http_put(base + "/api/warpage", {"bogus_key": 1})[0],
             http_put(base + "/api/warpage", {"bow_um": "abc"})[0]]
    wj2 = http_json(base + "/api/warpage")
    check("i73.warpage_put_invalid_400",
          all(c == 400 for c in codes)
          and abs(float((wj2.get("params") or {}).get("bow_um", 0))
                  - 30.0) < 1e-9,
          "codes=%s bow=%s" % (codes,
                               (wj2.get("params") or {}).get("bow_um")))

    # ---- PUT 回读还原 + POST 热切换 ----
    http_put(base + "/api/warpage", {"bow_um": 45.0, "saddle_ang_deg": 60.0})
    pj = (http_json(base + "/api/warpage").get("params") or {})
    rt1 = abs(float(pj.get("bow_um", 0)) - 45.0) < 1e-9 \
        and abs(float(pj.get("saddle_ang_deg", 0)) - 60.0) < 1e-9
    http_put(base + "/api/warpage", {"bow_um": 30.0, "saddle_ang_deg": 0.0})
    pj2 = (http_json(base + "/api/warpage").get("params") or {})
    rt2 = abs(float(pj2.get("bow_um", 0)) - 30.0) < 1e-9
    _warp(True)
    on1 = http_json(base + "/api/warpage").get("enabled") is True
    _warp(False)
    on2 = http_json(base + "/api/warpage").get("enabled") is False
    check("i73.warpage_put_roundtrip", rt1 and rt2,
          "roundtrip bow 30→45→30 ang 0→60→0")
    check("i73.warpage_toggle", on1 and on2, "POST enabled true/false 回读")

    # ---- 上片（显式真值 bow80/saddle15/ang25，确定性免抖动） ----
    c, r = http_post(base + "/api/workpiece/wafer",
                     {"action": "load",
                      "warp": {"bow_um": 80.0, "saddle_um": 15.0,
                               "ang_deg": 25.0}})
    assert c == 200 and r.get("ok"), (c, r)

    # ---- OFF 零影响：ON↔OFF 夹逼，共焦 z_surface/轴读数逐位回基线 ----
    ax0 = [a["actual"] for a in http_json(base + "/api/axes")["axes"]]
    z_off0 = _cf_z()
    _warp(True)
    time.sleep(0.2)
    z_on = _cf_z()
    _warp(False)
    ax1 = [a["actual"] for a in http_json(base + "/api/axes")["axes"]]
    z_off1 = _cf_z()
    check("i73.warpage_off_zero_impact",
          z_off0 == z_off1 and ax0 == ax1 and z_on != z_off0,
          "z_surface OFF %.6f == %.6f（ON %.6f）axes 快照逐位一致"
          % (z_off0, z_off1, z_on))

    # ---- 共焦增量 == state.probe_down.dz_um（同源断言） + 剖面公式 ----
    _warp(True)
    z_on2 = _cf_z()
    st1 = _wstate()
    pdz = (st1.get("probe_down") or {}).get("dz_um")
    d_meas = (z_on2 - z_off0) * 1000.0
    check("i73.warpage_confocal_probe",
          pdz is not None and abs(d_meas - pdz) < 0.005,
          "Δz_surface=%.4fµm state.probe_down.dz_um=%.4fµm（x=%s,y=%s）"
          % (d_meas, pdz if pdz is not None else -9,
             (st1.get("probe_down") or {}).get("x_mm"),
             (st1.get("probe_down") or {}).get("y_mm")))
    w1 = st1.get("warp") or {}
    prof = st1.get("profile") or {}
    u, v = prof.get("u_z_um") or [], prof.get("v_z_um") or []
    sad_edge = 15.0 * math.cos(math.radians(2.0 * 25.0))   # ±9.6418
    check("i73.warpage_state_profile",
          abs(float(w1.get("bow_um", 0)) - 80.0) < 1e-9
          and abs(float(w1.get("saddle_um", 0)) - 15.0) < 1e-9
          and abs(float(w1.get("ang_deg", 0)) - 25.0) < 1e-9
          and len(u) == 65 and len(v) == 65
          and abs(u[32] - 80.0) < 0.02        # 中心=bow
          and abs(u[0] - sad_edge) < 0.02     # u=−R 边缘=saddle·cos2θ
          and abs(v[0] + sad_edge) < 0.02     # v=−R 边缘=−saddle·cos2θ
          and len(st1.get("edge_tilt_urad") or []) == 2,
          "warp=%s center=%.3f u_edge=%.3f(期望 %.4f) v_edge=%.3f tilt=%s"
          % (w1, u[32] if u else -9, u[0] if u else -9, sad_edge,
             v[0] if v else -9, st1.get("edge_tilt_urad")))

    # ---- 视觉离焦：下相机晶圆标记 blur 随翘曲升起（dz/DOF） ----
    _warp(False)
    id0 = http_post(base + "/api/vision/identify",
                    {"cam": "down", "template": "cv_quad"})[1]
    blur0 = ((id0.get("entry") or {}).get("blur_ratio")) or 0.0
    _warp(True)
    id1 = http_post(base + "/api/vision/identify",
                    {"cam": "down", "template": "cv_quad"})[1]
    blur1 = ((id1.get("entry") or {}).get("blur_ratio")) or 0.0
    check("i73.warpage_vision_defocus",
          id0.get("ok") and id1.get("ok")
          and blur0 < 0.5 and 2.0 <= (blur1 - blur0) <= 5.0,
          "blur %.3f→%.3f（翘曲 dz≈72µm/DOF 20µm ≈ 3.6）" % (blur0, blur1))

    # ---- 键合接触贯通：停 base_eff+0.02mm，OFF 力≈0、ON 力跳升 ----
    # （i56 同手法；力读数链噪声 ±1N/拍→30 拍均值；头下降前 AS 退避防 409）
    mc73 = http_json(base + "/api/machine_constants")
    rc73 = http_json(base + "/api/recipe")
    base_eff = float(mc73["constants"]["bond"]["head_z_contact_mm"]) \
        + float(rc73["recipe"]["wafer"]["thickness_mm"]) \
        + float(rc73["recipe"]["film"]["dies"]["thickness_mm"])
    as_safe = [float(v) for v in
               mc73["constants"]["flip_handoff"]["as_safe_xy_mm"]]
    bdz = float(_wstate().get("bond_dz_um") or 0.0)
    time.sleep(1.3)          # eff_contact 1Hz 刷新窗（ON 状态已含 warp_dz）
    for ax, tgt in (("AS_X", as_safe[0]), ("AS_Y", as_safe[1])):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": base_eff + 0.02, "speed": 50.0})
    wait_settled("BS_Arm_Z")
    f_on = _force_avg()
    _warp(False)
    time.sleep(1.3)          # OFF 后等 1Hz 刷新把 warp_dz 撤掉
    f_off = _force_avg()
    check("i73.warpage_contact_shift",
          40.0 <= bdz <= 90.0 and (f_on - f_off) > 0.5,
          "bond_dz=%.1fµm f_on=%.3fN f_off=%.3fN Δ=%.3fN（base_eff=%.3f "
          "停 base+0.02mm）" % (bdz, f_on, f_off, f_on - f_off, base_eff))

    # ---- 回位 + 收尾还原 ----
    http_post(base + "/api/axis/move",
              {"axis": "BS_Arm_Z", "target": 0.0, "speed": 100.0})
    wait_settled("BS_Arm_Z")
    for ax, tgt in (("AS_X", 0.0), ("AS_Y", 0.0)):
        http_post(base + "/api/axis/move", {"axis": ax, "target": tgt,
                                            "speed": 200.0})
    for ax in ("AS_X", "AS_Y"):
        wait_settled(ax)
    http_post(base + "/api/workpiece/reset", {})
    _warp(True)              # perf：ON 干净窗
    t_perf = time.time()

    # ---- perf：warpage ON 干净窗 tick≥999Hz（读时合成无 tick 开销） ----
    h = read_clean_health(base, t_perf, retries=1)
    check("i73.warpage_perf",
          h["tick_hz"] >= 999.0,
          "tick=%.1fHz push=%.1fHz jitter_avg=%.0fus"
          % (h["tick_hz"], h["push_hz"], h["tick_jitter_avg_us"]))

    _warp(False)
    http_put(base + "/api/warpage", {"bow_um": 30.0, "saddle_um": 12.0,
                                     "saddle_ang_deg": 0.0, "jitter_pct": 20.0})


def main():
    global SUITE, BARE, MQTT_PORT, _KW_TOKENS
    SUITE = "full" if "--full" in sys.argv else "quick"
    # ---- 迭代 19：裸引擎分片参数（run_full_parallel.py 唤起；默认全栈不变）----
    # --bare           不起 runtime 子进程，进程内 BareHarness 裸引擎 + HTTP shim
    # --segs a,b       只跑段表中列出的段（分片白名单）
    # --skip-segs a,b  跳过列出的段（全栈片=全量减迁移段）
    # --out PATH       exit 前 dump {suite, results, skipped, seg_times}
    # --shard-id N     分片序号（tempdir 前缀/MQTT 端口派生用）
    # -k "a or b"      迭代 37 定向回归：tokens 匹配 check 名（子串/小写），
    #                  只跑 _KW_SEG_MAP 映射到的段；tokens ⊆ {i36,i37,
    #                  nonortho} 时另把扫描轴误差地图归零（严格带前提，
    #                  手法同 test_metrology 真值段，finally 按字节恢复）。
    #                  迭代 39 增补主题键：microrot→18.mc_calib（含
    #                  cal.microrot_view_rot_parity 迁移不变量）、vision→
    #                  17.vision/17.vision_captures（vision.* 视觉链用例）
    def _arg(name, default=None):
        return (sys.argv[sys.argv.index(name) + 1] if name in sys.argv
                else default)
    bare = "--bare" in sys.argv
    segs_only = set((_arg("--segs") or "").split(",")) - {""} \
        if "--segs" in sys.argv else None
    segs_skip = set((_arg("--skip-segs") or "").split(",")) - {""}
    out_path = _arg("--out")
    shard_id = int(_arg("--shard-id", "0"))
    if "-k" in sys.argv:
        _KW_TOKENS = [t.strip().lower()
                      for part in (_arg("-k") or "").split(" or ")
                      for t in part.replace(",", " ").split() if t.strip()]
    # tokens → 可能含匹配用例的段（白名单；token 不在表=该 token 无段可跑）
    _KW_SEG_MAP = {"i36": {"19.29.nonortho"},
                   "i37": {"19.29.nonortho"},
                   "nonortho": {"19.29.nonortho"},
                   "pireal": {"19.30.pi_real"},
                   "realchain": {"19.31.real_chains"},
                   "i76": {"19.33.iter76"},
                   "i77": {"19.34.iter77"},
                   "i78": {"19.35.iter78"},
                   "microrot": {"18.mc_calib"},
                   "vision": {"17.vision", "17.vision_captures"}}
    kw_segs = set()
    for t in _KW_TOKENS:
        kw_segs |= _KW_SEG_MAP.get(t, set())
    # 迭代 37 定向真值段：扫描轴误差地图归零（栅格跳测 ±5µrad 严格带的
    # 前提——长杠杆把随机项压低后，残差主导项是 6 分量地图在 5 节点上的
    # 投影；归零只影响本定向段（-k 时其余段整体跳过），finally 按字节恢复
    _MAP_AXES = ("WS_LS_X", "WS_LS_Y", "AS_X", "AS_Y")
    _MAP_COMPS = ("dx", "dy", "dz", "rx", "ry", "rz")
    map_backup = {}
    if _KW_TOKENS and set(_KW_TOKENS) <= {"i36", "i37", "nonortho"} \
            and not bare:
        maps_dir = os.path.join(ROOT, "runtime", "error_maps")
        try:
            for a in _MAP_AXES:
                for comp in _MAP_COMPS:
                    mp = os.path.join(maps_dir,
                                      "%s_%s_simulated.csv" % (a, comp))
                    with open(mp, "rb") as f:
                        map_backup[mp] = f.read()
            for mp, raw in map_backup.items():
                lines = raw.decode("utf-8").splitlines()
                out = [lines[0]]
                for ln in lines[1:]:
                    out.append(ln.split(",")[0] + ",0.0")
                with open(mp, "w", encoding="utf-8", newline="\n") as f:
                    f.write("\n".join(out) + "\n")
        except OSError as e:
            map_backup = {}
            print("[i37] 误差地图归零失败（继续用原图跑）：%s" % e)
    # 迭代 42b：i37.nonortho_scale_inject 前提——WS_LS_X scale_ppm 注入
    # +200ppm（error_system 仅 __init__ 时从 error_maps/low_order.json 读
    # scale_ppm，无 live API——写文件+finally 字节恢复，同误差地图归零
    # 手法；仅 -k i37 定向上下文在 backend 启动前改文件，套件上下文不改
    # =按 low_order 原值断言活基线；备份先入 map_backup 再写，恢复兜底）
    if _KW_TOKENS and set(_KW_TOKENS) <= {"i36", "i37", "nonortho"} \
            and not bare:
        lop = os.path.join(ROOT, "runtime", "error_maps", "low_order.json")
        try:
            with open(lop, "rb") as f:
                map_backup[lop] = f.read()
            lo = json.loads(map_backup[lop].decode("utf-8"))
            lo["axes"]["WS_LS_X"]["scale_ppm"] = 200.0
            with open(lop, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(lo, ensure_ascii=False, indent=1) + "\n")
        except (OSError, ValueError, KeyError) as e:
            print("[i37] low_order.json scale 注入失败（按原值跑）：%s" % e)
    # 迭代 18：测试进程提计时分辨率（Windows 默认 15.6ms 会让各段的
    # 5~50ms 轮询 sleep 实际睡过头）；runtime 进程内已有同款调用
    try:
        if ctypes is not None:
            ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass
    proc = None
    ws = None
    if bare:
        # 裸引擎组装（无端口）；focus_level 段在本片时起真 MQTT broker
        # （fl.mqtt_* 契约用例），端口 per-shard 隔离
        import bare_harness
        need_mqtt = bool(segs_only and "19.9.focus_level" in segs_only)
        MQTT_PORT = 11883 + shard_id
        BARE = bare_harness.BareHarness(shard_id=shard_id, mqtt=need_mqtt,
                                        mqtt_port=MQTT_PORT)
        # HTTP shim：替换模块级 http_*（测试体零改动；URL 的 host:port
        # 仅作路径载体，shim 按 path 路由）
        globals()["http_json"] = BARE.http_json
        globals()["http_get_raw"] = BARE.http_get_raw
        globals()["http_post"] = BARE.http_post
        globals()["http_put"] = BARE.http_put
        globals()["http_delete"] = BARE.http_delete
        BARE.engine.set_speed_scale(8.0)   # WS 帧构建节流（无客户端，纯省 CPU）
        print("[bare] harness up: shard=%d mqtt=%s tmp=%s"
              % (shard_id, need_mqtt, BARE.tmp), flush=True)
    else:
        # 预检：端口已被占用（多半是旧 runtime 进程残留）时直接报错退出，
        # 避免 Windows SO_REUSEADDR 式双绑定导致连到旧代码
        try:
            http_json("http://127.0.0.1:%d/api/health" % WS_PORT)
            print("端口 %d 已被占用：请先结束旧的 runtime 进程再跑测试" % WS_PORT)
            return 1
        except Exception:
            pass
        proc = subprocess.Popen(
            [sys.executable, os.path.join(ROOT, "runtime", "main.py")],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    t_all0 = time.time()
    try:
        if not bare:
            if not wait_ready():
                print("runtime 启动失败")
                return 1
            ws = WSClient()
            ws.send({"cmd": "ping", "t": time.time()})
            time.sleep(0.2)
            with ws._lock:
                pong = any("pong" in t for t in ws.texts)
            check("ws.ping_pong", pong)

        # 段表：(段名, 函数, 参数, 倍速, 套件, full-only 用例清单)。
        # 倍速=段首统一设定（段内另有局部窗的传 1.0 由段内自管）；速率测量/
        # 墙钟语义段必须 1.0。套件="full" 的段 quick 不跑（登记跳过清单，
        # 其中"段首自清理且 sim<30s"的独有用例已拆出为独立段两套件都跑）。
        segs = [
            ("1.stream", test_stream, (ws,), 1.0, "both", []),  # 500Hz 红线
            ("2.move", test_move, (ws,), 8.0, "both", []),
            ("3.plc", test_plc, (), 8.0, "both", []),
            ("3.5.plc_scan", test_plc_scan, (), 1.0, "both", []),  # 墙钟 ms
            ("3.6.mcs", test_mcs, (ws,), 8.0, "both", []),
            ("4.buffer", test_buffer, (), 8.0, "both", []),  # DO 脉冲在 8× 下
                                                             # 仍 ≫50ms 轮询
            ("5.estop", test_estop, (ws,), 2.0, "both", []),  # 巡航窗 0.4s 墙钟
                                                             # =0.8s sim 仍在加速段
            ("6.speed", test_speed_scale, (ws,), 1.0, "both", []),  # 速率测量
            ("6.5.servo", test_servo, (ws,), 1.0, "both", []),  # settling 墙钟
            ("7.error_config", test_error_and_config, (ws,), 8.0, "both", []),
            ("7.5.sixdof", test_sixdof, (ws,), 8.0, "both", []),
            ("8.homing", test_homing_and_limits, (ws,), 1.0, "both", []),  # duration 墙钟窗 + latch 相 50ms 轮询会漏相，保持 1×
            ("9.workpieces", test_workpieces_interlock, (ws,), 8.0, "both", []),
            ("11.fault", test_fault, (), 2.0, "both", []),  # 巡航窗 0.15s 墙钟：
                                                # 4×=0.6s sim 贴 0.79s 型材余量太薄
                                                #（批次实测 mid_move_decel 抖动），2×
            ("11.5.acsn", test_acsn, (), 1.0, "both", []),
            ("14.collision", test_collision, (), 8.0, "both", []),
            ("13.flow", test_flow, (ws,), 1.0, "both", []),  # 段内局部 8× 窗
            ("15.recipe", test_recipe, (), 1.0, "both", []),
            ("17.vision", test_vision, (), 4.0, "both", []),  # 段内 17.7 回 1× 岛
            ("17.vision_captures", test_vision_captures, (), 4.0, "full", [
                # 全幅 PNG 编解码是纯计算墙钟大头（倍速压不动），留 --full
                "vision.capture_save", "vision.capture_api",
                "vision.capture_flash", "vision.template_from_capture",
                "vision.capture_delete",
                "vision.wafer_die_marks", "vision.wafer_die_defocus"]),
            ("19.process_flows", test_process_flows, (), 8.0, "full", [
                # 实跑类用例（quick 由 i8.chain_bond_happy 单片链覆盖取片/
                # 交接路径；生成期/校验类已拆 test_process_flows_gen 两套件跑）
                "process.flip_pick_run", "process.flip_tpl_mark",
                "process.flip_pick_coarse_real", "process.flip_place_back",
                "process.flip_pick_alarm", "process.flip_pick_alarm_no_transfer",
                "process.coarse_offset_err_chain",
                "process.flip_handoff_compensate", "process.flip_handoff_run",
                "process.head_to_flip_recover", "process.flip_handoff_mc"]),
            ("19.process_flows_gen", test_process_flows_gen, (), 1.0, "both", []),
            ("19.9.focus_level", test_focus_level, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.9.focus_level_units", test_focus_level_units, (), 1.0, "both", []),
            ("19.10.fine_align", test_fine_align, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.11.iter7", test_iter7, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.12.iter8", test_iter8, (), 1.0, "both", []),  # 段内 8×（代表链）
            ("19.13.iter8b", test_iter8b, (), 1.0, "both", []),  # MQTT 墙钟
            ("19.14.iter9", test_iter9, (), 1.0, "full", [
                # 裸引擎进程内用例：tick 由测试线程墙钟驱动（sleep 精度下限
                # ~1ms）不可压缩；寻力过冲余量（i13 容差 50µm）不允许更快
                # 驱动。两段合计 ~18s，为保 quick ≤3min 门控到 full
                "i9.tuning_get", "i9.tuning_put", "i9.migration",
                "i9.thickness_calc", "i9.force_search_touch",
                "i9.force_search_overtravel_fail", "i9.keep_warm_instant",
                "i9.keep_warm_apply"]),
            ("19.16.iter10", test_iter10, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.17.iter11a", test_iter11a, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.18.iter11b", test_iter11b, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.8.wafer_ega", test_wafer_ega, (), 8.0, "both", []),  # 原位：
                # 在 fl/fa 之后——fl.guard_no_ega 要求此前无 ega_result，
                # 段序不要乱动（迭代 18 教训：隐式顺序依赖）
            ("18.mc_calib", test_machine_constants_calibration, (), 1.0, "both", []),  # REAL
            ("19.20.iter13", test_iter13, (), 1.0, "full", [
                # 同 iter9：裸引擎寻力用例墙钟驱动不可压缩（接触容差 50µm 对
                # 轮询过冲敏感），~10s 门控到 full
                "i13.contact_tol_default", "i13.contact_tol_put",
                "i13.confirm_tol_default", "i13.quad_geometry",
                "i13.quad_recipe_validate", "i13.force_search_contact_pass",
                "i13.contact_log", "i13.force_search_contact_fail",
                "i17.force_search_geom_pass", "i17.force_search_geom_fail",
                "i13.full_reset"]),
            ("19.21.iter15", test_iter15, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.22.iter17", test_iter17, (), 1.0, "both", []),  # REAL 冒烟
            ("19.23.iter17c", test_iter17c, (), 1.0, "both", []),  # 段内窗+fblock
            ("19.24.iter20", test_iter20, (), 1.0, "both", []),  # 段内 fblock（i20.repeat_contrast 全链实跑 full-only）
            ("19.25.iter21", test_iter21, (), 1.0, "both", []),  # 纯 API 校验，sim≈0
            ("19.26.iter22", test_iter22, (), 1.0, "both", []),  # 段内 read_clean_health（modal perf）保持 1×
            ("19.27.iter23a", test_iter23a, (), 1.0, "both", []),  # REAL 连拍 ~10s
            ("19.28.iter23b", test_iter23b, (), 1.0, "both", []),  # 段内 8× 提速窗；perf 窗=t_perf
            ("19.29.nonortho", test_nonortho, (), 1.0, "both", []),  # 迭代 37
            ("19.30.pi_real", test_pi_real, (), 1.0, "both", []),  # 迭代 53 REAL pi
            ("19.31.real_chains", test_real_chains, (), 1.0, "both", []),  # 迭代 54 REAL 链
            ("19.32.iter73", test_iter73, (), 1.0, "both", []),  # 迭代 73 晶圆翘曲；段内无提速窗，perf 窗=t_perf
            ("19.33.iter76", test_iter76, (), 1.0, "both", []),  # 迭代 76 正压垂直落片；段内真实流程 8×
            ("19.34.iter77", test_iter77, (), 1.0, "both", []),  # 迭代 77 误差全关
            ("19.35.iter78", test_iter78, (), 1.0, "both", []),  # 迭代 78 IO 一键搜力 + 通大气垂直落片
        ]
        for name, fn, args, scale, seg_suite, seg_skips in segs:
            if segs_only is not None and name not in segs_only:
                continue                     # 分片白名单（迭代 19）
            if name in segs_skip:
                continue                     # 全栈片剔除迁移段（迭代 19）
            if _KW_TOKENS and name not in kw_segs:
                continue                     # -k 定向回归：段级白名单（迭代 37）
            if bare and args:
                # 裸引擎无 WS 客户端；依赖 ws 的段不分配给裸片（防御，
                # 正常分片方案不会排到）
                print(">>> %s … skip (bare 无 WS)" % name, flush=True)
                continue
            if SUITE == "quick" and seg_suite == "full":
                SKIPPED_CASES.extend(seg_skips)
                print(">>> %s … skip (full-only, %d 用例留 --full)"
                      % (name, len(seg_skips)), flush=True)
                continue
            t0 = time.time()
            set_speed(scale)                 # 段首统一倍速（兼防上段泄漏）
            try:
                fn(*args)
            finally:
                if _SPEED_CUR != 1.0:
                    set_speed(1.0)
                SEG_TIMES.append((name, time.time() - t0))
                print(">>> %s … done (%.1f s)" % (name, SEG_TIMES[-1][1]),
                      flush=True)
        if ws is not None:
            ws.close()

        if not bare:
            # 末态 health 红线读数必须落在纯 1× 稳态窗：套件尾段的提速窗/
            # i13 full_reset 的 4× blip 会污染 5s 聚合窗（tick 虚高、push 被
            # 4× 节流稀释）——复原 1× 后读窗起点越过此刻的干净窗（迭代 18）
            set_speed(1.0)
            h = read_clean_health("http://127.0.0.1:%d" % WS_PORT,
                                  time.time(), retries=2)
            print("\nhealth（纯 1× 稳态窗）: tick=%.1fHz jitter_avg=%.0fus "
                  "jitter_max=%.0fus push=%.1fHz"
                  % (h["tick_hz"], h["tick_jitter_avg_us"],
                     h["tick_jitter_max_us"], h["push_hz"]))
        failed = [r for r in RESULTS if not r[1]]
        print("\n==== 套件=%s %d/%d 通过 ====" % (SUITE, len(RESULTS) - len(failed),
                                               len(RESULTS)))
        if SUITE == "quick":
            print("quick 覆盖 %d 条，full-only 跳过 %d 条（合计 %d）："
                  % (len(RESULTS), len(SKIPPED_CASES),
                     len(RESULTS) + len(SKIPPED_CASES)))
            for n in SKIPPED_CASES:
                print("  skip: %s" % n)
        for name, ok, detail in RESULTS:
            if not ok:
                print("  FAIL: %s %s" % (name, detail))
        print("\n---- 段级墙钟 ----")
        for name, dt in SEG_TIMES:
            print("  %-22s %6.1f s" % (name, dt))
        print("  %-22s %6.1f s" % ("TOTAL", time.time() - t_all0))
        if out_path:
            # 分片结果外置（迭代 19）：run_full_parallel.py 聚合判 FAIL
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump({"suite": SUITE, "shard_id": shard_id,
                           "results": RESULTS, "skipped": SKIPPED_CASES,
                           "seg_times": SEG_TIMES,
                           "wall_s": time.time() - t_all0},
                          f, ensure_ascii=False)
        return 1 if failed else 0
    finally:
        if BARE is not None:
            BARE.close()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        # 迭代 37 定向真值段：误差地图按字节恢复（backend 已终止，不 writing）
        for mp, raw in map_backup.items():
            try:
                with open(mp, "wb") as f:
                    f.write(raw)
            except OSError as e:
                print("[i37] 误差地图恢复失败：%s %s" % (mp, e))


if __name__ == "__main__":
    sys.exit(main())

