#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simcore.py — D2W 测校离线仿真通用核心库
========================================
设计目标: 每个测校项 = "参数Schema + 误差源组合" 的声明, 其余全部复用.

分层:
  1. ErrorSource 族   — 可组合的误差源构件 (采样 + 解析sigma + 系统偏置)
  2. SimContext       — 时间基/光学换算等公共上下文 (帧数,帧周期,像素尺寸...)
  3. Engine           — 通用引擎: run() 逐帧合成 / analytical() RSS / monte_carlo()
  4. diagnose_series  — 误差特征识别 (漂移/随机游走/周期/反向间隙/离群),
                        对仿真数据与实测CSV同样适用 → 为现实排查提供指导
  5. build_report     — 统一文本报告

物理建模约定 (相对旧版的修正):
  * 漂移类误差以 [时间] 为基底 (nm/min, nm/sqrt(min)), 通过帧周期换算到帧,
    使 帧数/帧率 的改变能正确反映到结果 (旧版 step/sqrt(n) 使热漂移与n无关, 不物理).
  * 去均值随机游走的解析sigma: E[s^2] = step^2*(n+1)/6  → s = step*sqrt((n+1)/6)
    (严格推导, 旧版 sqrt(n/3) 偏大约 sqrt(2) 倍).
  * 标记偏离视场中心 r 参与建模:
      - 重复性: 边缘像差使亚像素定位精度退化 subpix_eff = subpix*(1+k_edge*(r/r_fov)^2)
      - 准确度: 微旋残差 phi*r 与畸变 k_dist*r 记为系统偏置 bias (不入3sigma, 单独报告)
  * 反向间隙区分 双向到位/定向逼近 两种工况.

单位约定: 内部一律 nm / 帧序列; 时间参数显式标注 min 或 s.
"""

import numpy as np

__version__ = "1.0.0"


# ===================================================================
#  上下文
# ===================================================================

class SimContext:
    """公共上下文: 帧数, 帧周期, 像素尺寸等. 误差源通过 ctx 取得换算基底."""

    def __init__(self, n, dt_s=0.05, pixel_size_um=None, extra=None):
        self.n = int(n)                  # 采样帧/循环数
        self.dt_s = float(dt_s)          # 相邻采样间隔 (秒)
        self.dt_min = self.dt_s / 60.0
        self.duration_min = self.n * self.dt_min
        self.pixel_size_um = pixel_size_um   # 等效像素尺寸 um/px (可为None)
        self.extra = extra or {}

    @property
    def pixel_size_nm(self):
        return None if self.pixel_size_um is None else self.pixel_size_um * 1000.0


# ===================================================================
#  误差源构件族
# ===================================================================

class ErrorSource:
    """误差源基类.

    sample(rng, ctx) -> (x, y)  逐帧误差序列, 单位nm
    sigma(ctx)       -> (sx, sy) 解析单轴sigma, 单位nm (对帧内统计而言)
    bias(ctx)        -> (bx, by) 系统偏置 (影响准确度而非重复性), 默认0
    """
    group = "未分组"

    def __init__(self, name):
        self.name = name

    def sample(self, rng, ctx):
        raise NotImplementedError

    def sigma(self, ctx):
        raise NotImplementedError

    def bias(self, ctx):
        return 0.0, 0.0


class GaussianNoise(ErrorSource):
    """白噪声 (高斯). 例: 相机离线重复性, 运动台定位重复性."""

    def __init__(self, name, sigma_x_nm, sigma_y_nm=None, group="随机噪声"):
        super().__init__(name)
        self.sx = float(sigma_x_nm)
        self.sy = float(sigma_x_nm if sigma_y_nm is None else sigma_y_nm)
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        x = rng.normal(0, self.sx, n) if self.sx > 0 else np.zeros(n)
        y = rng.normal(0, self.sy, n) if self.sy > 0 else np.zeros(n)
        return x, y

    def sigma(self, ctx):
        return self.sx, self.sy


class UniformQuantization(ErrorSource):
    """均匀量化噪声. 例: 亚像素定位量化. half_range -> sigma = h/sqrt(3)."""

    def __init__(self, name, half_range_nm, group="量化"):
        super().__init__(name)
        self.h = float(half_range_nm)
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        if self.h <= 0:
            return np.zeros(n), np.zeros(n)
        return rng.uniform(-self.h, self.h, n), rng.uniform(-self.h, self.h, n)

    def sigma(self, ctx):
        s = self.h / np.sqrt(3.0)
        return s, s


class RandomWalkDrift(ErrorSource):
    """随机游走漂移 (去均值). 例: 热漂移.

    以扩散系数 D (nm/sqrt(min)) 为物理参数: step_sigma = D*sqrt(dt_min).
    解析: 去均值cumsum的期望样本方差 = step^2*(n+1)/6.
    """

    def __init__(self, name, diffusion_nm_per_sqrt_min, group="慢漂移"):
        super().__init__(name)
        self.D = float(diffusion_nm_per_sqrt_min)
        self.group = group

    def _step(self, ctx):
        return self.D * np.sqrt(max(ctx.dt_min, 1e-12))

    def sample(self, rng, ctx):
        n = ctx.n
        st = self._step(ctx)
        if st <= 0:
            return np.zeros(n), np.zeros(n)
        x = np.cumsum(rng.normal(0, st, n)); x -= x.mean()
        y = np.cumsum(rng.normal(0, st, n)); y -= y.mean()
        return x, y

    def sigma(self, ctx):
        s = self._step(ctx) * np.sqrt((ctx.n + 1) / 6.0)
        return s, s


class LinearDrift(ErrorSource):
    """线性漂移 (确定性). 例: 预热不充分. 参数: nm/min.
    序列 slope_f*[0..n-1] 的标准差 = |slope_f|*sqrt((n^2-1)/12).
    注意: 确定性趋势计入std会抬高3sigma — 可配合去趋势选项评估纯随机分量.
    """

    def __init__(self, name, slope_x_nm_per_min, slope_y_nm_per_min, group="慢漂移"):
        super().__init__(name)
        self.kx = float(slope_x_nm_per_min)
        self.ky = float(slope_y_nm_per_min)
        self.group = group

    def sample(self, rng, ctx):
        t = np.arange(ctx.n, dtype=float) * ctx.dt_min
        return self.kx * t, self.ky * t

    def sigma(self, ctx):
        c = ctx.dt_min * np.sqrt((ctx.n ** 2 - 1) / 12.0)
        return abs(self.kx) * c, abs(self.ky) * c


class Vibration(ErrorSource):
    """机械振动 = 随机RMS分量 + 可选正弦分量 (随机相位).
    正弦解析sigma = A/sqrt(2). 若 freq*dt 接近整数会出现混叠(采样致), 属真实现象.
    """

    def __init__(self, name, rms_nm, sine_amp_nm=0.0, sine_freq_hz=0.0, group="振动"):
        super().__init__(name)
        self.rms = float(rms_nm)
        self.amp = float(sine_amp_nm)
        self.freq = float(sine_freq_hz)
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        x = rng.normal(0, self.rms, n) if self.rms > 0 else np.zeros(n)
        y = rng.normal(0, self.rms, n) if self.rms > 0 else np.zeros(n)
        if self.amp > 0 and self.freq > 0:
            t = np.arange(n) * ctx.dt_s
            phx, phy = rng.uniform(0, 2 * np.pi, 2)
            x = x + self.amp * np.sin(2 * np.pi * self.freq * t + phx)
            y = y + self.amp * np.sin(2 * np.pi * self.freq * t + phy)
        return x, y

    def sigma(self, ctx):
        s = np.sqrt(self.rms ** 2 + (self.amp ** 2) / 2.0)
        return s, s


class Backlash(ErrorSource):
    """反向间隙 (往返运动).

    mode='bidirectional': 双向到位, 奇偶循环 ±h/2 交替 (std=h/2) + 抖动
    mode='unidirectional': 定向逼近, 间隙表现为常值偏置 (入bias) + 抖动
    """

    def __init__(self, name, h_x_nm, h_y_nm, mode="bidirectional",
                 jitter_frac=0.1, group="运动台"):
        super().__init__(name)
        self.hx, self.hy = float(h_x_nm), float(h_y_nm)
        self.mode = mode
        self.jf = float(jitter_frac)
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        jx = rng.normal(0, self.hx * self.jf, n) if self.hx > 0 else np.zeros(n)
        jy = rng.normal(0, self.hy * self.jf, n) if self.hy > 0 else np.zeros(n)
        if self.mode == "bidirectional":
            pat = np.where(np.arange(n) % 2 == 0, 1.0, -1.0)
            return pat * self.hx / 2 + jx, pat * self.hy / 2 + jy
        return jx, jy   # 定向逼近: 常值偏置不进入序列波动

    def sigma(self, ctx):
        if self.mode == "bidirectional":
            return (np.sqrt((self.hx / 2) ** 2 + (self.hx * self.jf) ** 2),
                    np.sqrt((self.hy / 2) ** 2 + (self.hy * self.jf) ** 2))
        return self.hx * self.jf, self.hy * self.jf

    def bias(self, ctx):
        if self.mode == "unidirectional":
            return self.hx / 2.0, self.hy / 2.0
        return 0.0, 0.0


class RailStraightness(ErrorSource):
    """导轨直线度交叉耦合: 沿X运动→Y向误差, 沿Y运动→X向误差.
    参数为 nm/100mm 行程的等效高斯RMS, 按实际行程线性缩放."""

    def __init__(self, name, rail_x_nm_per_100mm, rail_y_nm_per_100mm,
                 travel_x_mm, travel_y_mm, group="运动台"):
        super().__init__(name)
        # X导轨直线度 → 影响Y; Y导轨 → 影响X
        self.sy = float(rail_x_nm_per_100mm) * float(travel_x_mm) / 100.0
        self.sx = float(rail_y_nm_per_100mm) * float(travel_y_mm) / 100.0
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        x = rng.normal(0, self.sx, n) if self.sx > 0 else np.zeros(n)
        y = rng.normal(0, self.sy, n) if self.sy > 0 else np.zeros(n)
        return x, y

    def sigma(self, ctx):
        return self.sx, self.sy


class SettlingResidual(ErrorSource):
    """到位稳定时间不足的残余振荡: A_res = A0 * exp(-t_settle/tau)."""

    def __init__(self, name, amp0_nm, settle_time_s, tau_s, group="运动台"):
        super().__init__(name)
        tau = max(float(tau_s), 1e-9)
        self.a = float(amp0_nm) * np.exp(-float(settle_time_s) / tau)
        self.group = group

    def sample(self, rng, ctx):
        n = ctx.n
        if self.a <= 0:
            return np.zeros(n), np.zeros(n)
        return rng.normal(0, self.a, n), rng.normal(0, self.a, n)

    def sigma(self, ctx):
        return self.a, self.a


class ConstantBias(ErrorSource):
    """纯系统偏置 (准确度项): 不进入逐帧波动, 仅记录于bias.
    例: 相机微旋残差 phi*r, 畸变 k*r — 静态下为常值, 不贡献重复性."""

    def __init__(self, name, bias_x_nm, bias_y_nm, group="系统偏置"):
        super().__init__(name)
        self.bx, self.by = float(bias_x_nm), float(bias_y_nm)
        self.group = group

    def sample(self, rng, ctx):
        return np.zeros(ctx.n), np.zeros(ctx.n)

    def sigma(self, ctx):
        return 0.0, 0.0

    def bias(self, ctx):
        return self.bx, self.by


# ===================================================================
#  通用引擎
# ===================================================================

class Engine:
    """通用仿真引擎. 由 (ctx构造器, 误差源列表构造器, spec) 驱动.

    model: 对象/命名空间, 需提供:
        model.context(p)  -> SimContext
        model.sources(p, ctx) -> [ErrorSource, ...]
    p: 参数字典 (含 'spec_3sigma', 可选 'seed', 'remove_slope')
    """

    def __init__(self, model, p):
        self.model = model
        self.p = dict(p)

    # ---------- 单次运行 ----------
    def run(self, seed=None):
        p = self.p
        ctx = self.model.context(p)
        sources = self.model.sources(p, ctx)
        rng = np.random.default_rng(seed if seed is not None else p.get("seed"))

        n = ctx.n
        tot_x = np.zeros(n); tot_y = np.zeros(n)
        budget = {}
        bias_x = bias_y = 0.0
        per_source = {}
        for s in sources:
            x, y = s.sample(rng, ctx)
            tot_x += x; tot_y += y
            bx, by = s.bias(ctx)
            bias_x += bx; bias_y += by
            rx = float(np.sqrt(np.mean(x ** 2)))
            ry = float(np.sqrt(np.mean(y ** 2)))
            budget[s.name] = {"group": s.group, "rms_x": rx, "rms_y": ry,
                              "rms_t": float(np.sqrt(rx ** 2 + ry ** 2))}
            per_source[s.name] = (x, y)

        if p.get("remove_slope", False):
            px, py = detrend(tot_x), detrend(tot_y)
        else:
            px, py = tot_x.copy(), tot_y.copy()

        sr, sp = series_stats(tot_x, tot_y), series_stats(px, py)
        spec = float(p.get("spec_3sigma", 0) or 0)
        return {
            "params": dict(p), "ctx": ctx,
            "frames": np.arange(1, n + 1),
            "raw": {"x": tot_x, "y": tot_y}, "proc": {"x": px, "y": py},
            "per_source": per_source,
            "sr": sr, "sp": sp, "budget": budget,
            "bias": {"x": bias_x, "y": bias_y},
            "pass_r": (sr["s3x"] <= spec and sr["s3y"] <= spec) if spec else None,
            "pass_p": (sp["s3x"] <= spec and sp["s3y"] <= spec) if spec else None,
            "detrend": bool(p.get("remove_slope", False)), "spec": spec,
        }

    # ---------- 解析RSS ----------
    def analytical(self):
        p = self.p
        ctx = self.model.context(p)
        sources = self.model.sources(p, ctx)
        comp = []
        vx = vy = 0.0
        bias_x = bias_y = 0.0
        for s in sources:
            sx, sy = s.sigma(ctx)
            bx, by = s.bias(ctx)
            bias_x += bx; bias_y += by
            comp.append({"name": s.name, "group": s.group,
                         "sigma_x": float(sx), "sigma_y": float(sy)})
            vx += sx ** 2; vy += sy ** 2
        sx_t, sy_t = float(np.sqrt(vx)), float(np.sqrt(vy))
        return {
            "components": comp,
            "sigma_total_x": sx_t, "sigma_total_y": sy_t,
            "three_sigma_x": 3 * sx_t, "three_sigma_y": 3 * sy_t,
            "bias_x": bias_x, "bias_y": bias_y,
            "n": ctx.n, "dt_s": ctx.dt_s,
        }

    # ---------- 蒙特卡洛 ----------
    def monte_carlo(self, mc_runs=500, seed0=10000):
        key = "sp" if self.p.get("remove_slope", False) else "sr"
        s3x = np.empty(mc_runs); s3y = np.empty(mc_runs)
        for i in range(mc_runs):
            r = self.run(seed=seed0 + i)
            s3x[i] = r[key]["s3x"]; s3y[i] = r[key]["s3y"]
        out = {"s3x": s3x, "s3y": s3y, "mc_runs": mc_runs}
        for ax, a in (("x", s3x), ("y", s3y)):
            out[f"s3{ax}_mean"] = float(np.mean(a))
            out[f"s3{ax}_std"] = float(np.std(a, ddof=1))
            for q, tag in ((2.5, "p2_5"), (97.5, "p97_5"), (5, "p5"), (95, "p95")):
                out[f"s3{ax}_{tag}"] = float(np.percentile(a, q))
            out[f"s3{ax}_min"] = float(np.min(a)); out[f"s3{ax}_max"] = float(np.max(a))
        spec = float(self.p.get("spec_3sigma", 0) or 0)
        if spec:
            out["pass_prob"] = float(np.mean((s3x <= spec) & (s3y <= spec)))
        return out


# ===================================================================
#  统计工具
# ===================================================================

def detrend(a):
    n = len(a); t = np.arange(n, dtype=float)
    A = np.vstack([t, np.ones(n)]).T
    k, c = np.linalg.lstsq(A, a, rcond=None)[0]
    return a - (k * t + c)


def series_stats(x, y):
    sx = float(np.std(x, ddof=1)); sy = float(np.std(y, ddof=1))
    t = np.arange(len(x), dtype=float)
    kx = float(np.polyfit(t, x, 1)[0]) if len(x) > 1 else 0.0
    ky = float(np.polyfit(t, y, 1)[0]) if len(y) > 1 else 0.0
    return {"mx": float(np.mean(x)), "my": float(np.mean(y)),
            "sx": sx, "sy": sy, "s3x": 3 * sx, "s3y": 3 * sy,
            "absx": float(np.max(np.abs(x))), "absy": float(np.max(np.abs(y))),
            "kx": kx, "ky": ky, "rx": float(np.ptp(x)), "ry": float(np.ptp(y))}


# ===================================================================
#  误差特征诊断 (对仿真序列与实测序列同样适用)
# ===================================================================

def _diag_axis(a, dt_s, label):
    """单轴序列特征识别. 返回 findings 列表 [(severity, title, detail, advice)]."""
    f = []
    n = len(a)
    if n < 10:
        return [("info", f"{label}: 样本量过少", f"n={n} < 10, 诊断能力受限", "增加采样次数")]
    t = np.arange(n, dtype=float)
    s = np.std(a, ddof=1)
    if s <= 0:
        return f

    # 1) 线性漂移显著性 (回归斜率 t 检验)
    A = np.vstack([t, np.ones(n)]).T
    coef, res, _, _ = np.linalg.lstsq(A, a, rcond=None)
    k = coef[0]
    resid = a - A @ coef
    se = np.sqrt(np.sum(resid ** 2) / (n - 2) / np.sum((t - t.mean()) ** 2))
    tstat = abs(k) / se if se > 0 else 0.0
    drift_total = abs(k) * (n - 1)
    if tstat > 4 and drift_total > 0.5 * s:
        f.append(("warn", f"{label}: 显著线性漂移",
                  f"斜率={k:.3f} nm/帧 (t={tstat:.1f}), 全程漂移≈{drift_total:.1f} nm",
                  "疑似预热不足或环境温度单调变化: 延长预热(≥30min), 检查空调周期; "
                  "若漂移不可避免, 评估去趋势后的纯随机分量"))

    # 2) 慢变/随机游走特征 (去趋势后 lag-1 自相关)
    d = detrend(a)
    sd = np.std(d, ddof=1)
    if sd > 0:
        rho1 = float(np.corrcoef(d[:-1], d[1:])[0, 1])
        if rho1 > 0.5:
            f.append(("warn", f"{label}: 强正自相关 (rho1={rho1:.2f})",
                      "相邻帧高度相关, 呈随机游走/慢漂移特征, 非白噪声",
                      "疑似热漂移或气流扰动: 检查温控波动与设备罩内气流; "
                      "白噪声假设下的3sigma判据对该分量偏乐观"))

    # 3) 周期性 (FFT 主峰占比)
    if n >= 32:
        dd = d - d.mean()
        ps = np.abs(np.fft.rfft(dd)) ** 2
        ps[0] = 0.0
        tot = ps.sum()
        if tot > 0:
            ipk = int(np.argmax(ps))
            frac = ps[ipk] / tot
            if frac > 0.30 and ipk > 0:
                cyc = ipk / n  # 周期/帧
                msg = f"主峰占功率{frac*100:.0f}%, 周期≈{1/cyc:.1f}帧"
                if dt_s:
                    msg += f" (≈{cyc/dt_s:.2f} Hz)"
                adv = ("疑似机械振动/风机/泵或电源工频: 用频率定位振源, "
                       "检查隔振器状态, 必要时错开采样频率")
                if ipk >= n // 2 - 1:
                    adv = ("周期≈2帧, 若为往返测试则高度疑似反向间隙交替特征: "
                           "优先检查间隙补偿或改定向逼近; 否则按高频振动排查")
                f.append(("warn", f"{label}: 显著周期分量", msg, adv))

    # 4) 奇偶分裂 (反向间隙/双向到位特征) — 在去趋势序列上检验, 避免漂移污染
    odd, even = d[1::2], d[0::2]
    if len(odd) >= 5 and len(even) >= 5:
        gap = abs(np.mean(odd) - np.mean(even))
        pooled = np.sqrt((np.var(odd, ddof=1) + np.var(even, ddof=1)) / 2)
        if pooled > 0 and gap > 1.2 * pooled:
            f.append(("warn", f"{label}: 奇偶循环均值分裂",
                      f"|奇-偶|={gap:.1f} nm vs 组内sigma={pooled:.1f} nm",
                      "双向到位的反向间隙特征: 检查间隙补偿参数, "
                      "或改为定向逼近(单向到位)消除该分量"))

    # 5) 离群点 (>4 sigma, 去趋势后)
    if sd > 0:
        nout = int(np.sum(np.abs(d) > 4 * sd))
        if nout > 0:
            f.append(("warn", f"{label}: 检测到{nout}个离群点(>4sigma)",
                      "可能为瞬时扰动/丢帧/误识别",
                      "检查log对应时刻事件(开门/行车/邻机), 核对图像识别得分"))
    return f


def diagnose_series(x, y, dt_s=None, budget=None, analytical=None, spec=None):
    """综合诊断: 序列特征识别 + 误差预算主导项排名 + 整改建议.

    x, y     : 误差序列 (nm), 可来自仿真或实测CSV
    budget   : run() 输出的budget (可选; 实测数据没有)
    analytical: analytical() 输出 (可选, 用于主导项排名)
    返回 dict(findings=[(sev,title,detail,advice)], ranking=[(name,share_x,share_y)], summary=str)
    """
    findings = []
    findings += _diag_axis(np.asarray(x, float), dt_s, "X")
    findings += _diag_axis(np.asarray(y, float), dt_s, "Y")

    ranking = []
    if analytical:
        vx = analytical["sigma_total_x"] ** 2 or 1e-30
        vy = analytical["sigma_total_y"] ** 2 or 1e-30
        for c in analytical["components"]:
            ranking.append((c["name"], c["group"],
                            c["sigma_x"] ** 2 / vx, c["sigma_y"] ** 2 / vy))
        ranking.sort(key=lambda r: max(r[2], r[3]), reverse=True)

    sx3, sy3 = 3 * np.std(x, ddof=1), 3 * np.std(y, ddof=1)
    lines = [f"实际序列 3sigma: X={sx3:.1f} nm, Y={sy3:.1f} nm"]
    if spec:
        ok = sx3 <= spec and sy3 <= spec
        lines.append(f"指标 ±{spec:.0f} nm → {'满足' if ok else '超标'}")
        if not ok and ranking:
            top = [r[0] for r in ranking[:3]]
            lines.append("超标时优先排查 (按方差贡献): " + " > ".join(top))
    elif ranking:
        top = [r[0] for r in ranking[:3]]
        lines.append("主导误差源 (按方差贡献): " + " > ".join(top))
    if not findings:
        findings.append(("ok", "未检测到异常特征",
                         "序列接近白噪声, 无显著漂移/周期/间隙特征",
                         "若3sigma仍超标, 说明随机噪声基底过高: "
                         "按方差贡献排名逐项压减"))
    return {"findings": findings, "ranking": ranking, "summary": "\n".join(lines)}


# ===================================================================
#  报告
# ===================================================================

def build_report(title, result, ana, mc, diag=None, extra_lines=None):
    from datetime import datetime
    r, p = result, result["params"]
    st = r["sp"] if r["detrend"] else r["sr"]
    spec = r["spec"]
    in_ci = (mc["s3x_p2_5"] <= st["s3x"] <= mc["s3x_p97_5"] and
             mc["s3y_p2_5"] <= st["s3y"] <= mc["s3y_p97_5"])
    if spec:
        ok = r["pass_p"] if r["detrend"] else r["pass_r"]
        verdict = "PASS" if ok else "FAIL"
        spec_line = f"  Spec:     3sigma <= +/-{spec:.0f} nm"
    else:
        verdict = "正常 (位于验收参考区间)" if in_ci else "超出验收参考区间 → 见DIAGNOSIS"
        spec_line = (f"  建议验收区间: 3sigX ∈ [{mc['s3x_p2_5']:.1f}, {mc['s3x_p97_5']:.1f}] nm, "
                     f"3sigY ∈ [{mc['s3y_p2_5']:.1f}, {mc['s3y_p97_5']:.1f}] nm")
    L = ["=" * 76, f"  {title}", "=" * 76,
         f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}   simcore v{__version__}"]
    if extra_lines:
        L += [f"  {s}" for s in extra_lines]
    L += ["", "-" * 76, "  RESULT", "-" * 76,
          f"  Verdict:  {verdict}",
          spec_line,
          f"  3sig X =  {st['s3x']:.2f} nm    3sig Y = {st['s3y']:.2f} nm",
          f"  Bias  X = {r['bias']['x']:.2f} nm    Bias Y = {r['bias']['y']:.2f} nm  (系统偏置, 准确度项)",
          f"  Detrend:  {'Yes' if r['detrend'] else 'No'}", ""]
    L += ["-" * 76, "  ACCEPTANCE REFERENCE (for inspectors)", "-" * 76,
          f"  Analytical 3sig:  X={ana['three_sigma_x']:.2f}  Y={ana['three_sigma_y']:.2f} nm",
          f"  MC 95% CI X: [{mc['s3x_p2_5']:.2f}, {mc['s3x_p97_5']:.2f}] nm  (N={mc['mc_runs']})",
          f"  MC 95% CI Y: [{mc['s3y_p2_5']:.2f}, {mc['s3y_p97_5']:.2f}] nm",
          f"  MC Mean:     X={mc['s3x_mean']:.2f}  Y={mc['s3y_mean']:.2f} nm",
          f"  建议验收上限 (P97.5): X<={mc['s3x_p97_5']:.1f} nm, Y<={mc['s3y_p97_5']:.1f} nm "
          f"(工程裕量可上浮10~20%)"]
    if "pass_prob" in mc:
        L.append(f"  通过概率 (按当前参数, n有限抽样): {mc['pass_prob']*100:.1f}%")
    L += ["", "  判读: 实测3sig ∈ 95%CI → 正常 | 超出区间 → 按DIAGNOSIS排查", ""]
    L += ["-" * 76, "  ANALYTICAL SIGMA BREAKDOWN (单轴)", "-" * 76]
    for c in ana["components"]:
        L.append(f"  {c['name']:<22s} [{c['group']}]  sx={c['sigma_x']:.2f}  sy={c['sigma_y']:.2f} nm")
    L += [f"  {'TOTAL (RSS)':<22s}  sx={ana['sigma_total_x']:.2f}  sy={ana['sigma_total_y']:.2f} nm", ""]
    if diag:
        L += ["-" * 76, "  DIAGNOSIS", "-" * 76, "  " + diag["summary"].replace("\n", "\n  "), ""]
        for sev, t, d, adv in diag["findings"]:
            L += [f"  [{sev.upper()}] {t}", f"      现象: {d}", f"      建议: {adv}"]
        if diag["ranking"]:
            L += ["", "  方差贡献排名:"]
            for nm, grp, fx, fy in diag["ranking"]:
                L.append(f"    {nm:<22s} X:{fx*100:5.1f}%  Y:{fy*100:5.1f}%")
        L.append("")
    L += ["-" * 76, "  PARAMETERS", "-" * 76]
    for k, v in p.items():
        L.append(f"  {k}: {v}")
    L += ["", "-" * 76, "  ERROR BUDGET (single-run RMS)", "-" * 76]
    for nm, vl in r["budget"].items():
        L.append(f"  {nm:<22s} X={vl['rms_x']:.2f}  Y={vl['rms_y']:.2f}  T={vl['rms_t']:.2f} nm")
    L += ["", "=" * 76]
    return "\n".join(L)


# ===================================================================
#  实测CSV导入 (供诊断模块复用): 列 idx,x_nm,y_nm 或 x_nm,y_nm
# ===================================================================

def load_measured_csv(path):
    import csv
    xs, ys = [], []
    with open(path, "r", encoding="utf-8-sig") as fh:
        rd = csv.reader(fh)
        rows = [row for row in rd if row and any(c.strip() for c in row)]
    start = 0
    try:
        float(rows[0][-1])
    except (ValueError, IndexError):
        start = 1   # 含表头
    for row in rows[start:]:
        vals = [float(c) for c in row if c.strip() != ""]
        if len(vals) >= 3:
            xs.append(vals[-2]); ys.append(vals[-1])
        elif len(vals) == 2:
            xs.append(vals[0]); ys.append(vals[1])
    return np.array(xs), np.array(ys)
