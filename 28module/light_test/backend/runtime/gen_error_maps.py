#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_error_maps.py — 为本机 21 轴生成合成 6 分量几何误差地图数据集。

输出 runtime/error_maps/：
  {axis}_{comp}_simulated.csv   21 轴 × 6 分量 = 126 份两列 CSV
    直线轴表头 position_mm,{comp}_um；旋转轴表头 position_rad,{comp}_urad
  manifest.json                 逐图 axis/component/node_count/coverage/
                                error_min/max/sha256/seed
  low_order.json                每轴 scale_ppm（精密 ±5~10、辅助 ±20）+
                                direction_rx/ry/rz_urad（暂留 0）

合成模型（参照 812PPT汇报/backend 的 generate_maps.py）：
  常值 + 线性 + 二次 + 双频正弦 + 局部高斯包 + 纹理噪声；固定 seed 可复现；
  节点 15~40 个不等距；两端各裁 1%~8%（部分覆盖，区间外视为未测量）。
"""
import csv
import hashlib
import json
import math
import os
import random

import axes_config as ac

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "error_maps")
DATASET_ID = "d2w-1928-all-axes-synthetic-v1"
COMPONENTS = ("dx", "dy", "dz", "rx", "ry", "rz")
TRANS = ("dx", "dy", "dz")


def stable_seed(*parts):
    raw = "|".join(map(str, parts)).encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:8], "big")


def axis_class(name):
    return "precision" if name in ac.PRECISION_AXES else "auxiliary"


def amplitude_range(name, comp):
    """(幅值范围, 纹理上限)。平移 µm、旋转 µrad。"""
    prec = name in ac.PRECISION_AXES
    if comp in TRANS:
        return ((0.2, 1.0), 0.03) if prec else ((1.0, 5.0), 0.2)
    return ((1.0, 5.0), 0.2) if prec else ((10.0, 30.0), 2.0)


def partial_coverage(lo, hi, seed, decimals):
    rng = random.Random(seed)
    travel = hi - lo
    left = travel * rng.uniform(0.01, 0.08)
    right = travel * rng.uniform(0.01, 0.08)
    return [round(lo + left, decimals), round(hi - right, decimals)]


def irregular_positions(low, high, node_count, seed, decimals):
    rng = random.Random(seed)
    weights = [rng.uniform(0.32, 1.85) for _ in range(node_count - 1)]
    total = sum(weights)
    cumulative = 0.0
    positions = [float(low)]
    for w in weights[:-1]:
        cumulative += w
        positions.append(round(low + (high - low) * cumulative / total, decimals))
    positions.append(float(high))
    if len(set(positions)) != len(positions):
        raise RuntimeError("位置精度不足导致重复节点")
    return positions


def simulated_value(position, coverage, name, comp, seed):
    """常值+线性+二次+双频正弦+局部高斯包+纹理噪声。"""
    rng = random.Random(seed + int(abs(position) * 1_000_003))
    low, high = coverage
    center = (low + high) / 2.0
    span = high - low
    u = (position - center) / (span / 2.0)
    (amp_lo, amp_hi), tex_hi = amplitude_range(name, comp)
    base = stable_seed(name, comp, "amplitude")
    r = random.Random(base)
    amplitude = r.uniform(amp_lo, amp_hi)
    texture = r.uniform(tex_hi * 0.15, tex_hi)
    phase = (base % 6283) / 1000.0
    second_phase = ((base // 13) % 6283) / 1000.0
    local_at = center + span * (((base // 101) % 1200) / 1200.0 - 0.5) * 0.65
    local_width = span * (0.055 + ((base // 37) % 90) / 1000.0)
    sign = -1.0 if base & 1 else 1.0
    value = amplitude * (
        0.34 * sign
        + 0.42 * u
        + 0.31 * sign * (u * u - 1.0 / 3.0)
        + 0.68 * math.sin(2.0 * math.pi * (position - low) / (span * 0.73) + phase)
        + 0.23 * math.cos(2.0 * math.pi * (position - low) / (span * 0.19)
                          + second_phase)
        + 0.38 * sign * math.exp(-((position - local_at) / local_width) ** 2)
    ) + rng.gauss(0.0, texture)
    return value


def generate():
    os.makedirs(OUT, exist_ok=True)
    maps = []
    low_order_axes = {}
    for name, lo, hi, vmax, unit, ctl, home, fid in ac.AXES:
        rotary = unit == "rad"
        decimals = 9 if rotary else 6
        pos_header = "position_rad" if rotary else "position_mm"
        # low_order：精密 ±5~10 ppm（带符号随机），辅助 ±20 ppm
        rng_lo = random.Random(stable_seed(DATASET_ID, name, "low-order"))
        if name in ac.PRECISION_AXES:
            scale = round(rng_lo.uniform(5.0, 10.0) * rng_lo.choice((-1, 1)), 3)
        else:
            scale = round(rng_lo.uniform(-20.0, 20.0), 3)
        low_order_axes[name] = {
            "scale_ppm": scale,
            "direction_rx_urad": 0.0,
            "direction_ry_urad": 0.0,
            "direction_rz_urad": 0.0,
        }
        for comp in COMPONENTS:
            seed = stable_seed(DATASET_ID, name, comp)
            coverage = partial_coverage(lo, hi, seed, decimals)
            node_count = 15 + seed % 26          # 15~40
            positions = irregular_positions(
                coverage[0], coverage[1], node_count, seed + 17, decimals)
            values = [simulated_value(p, coverage, name, comp, seed + 91)
                      for p in positions]
            value_unit = "um" if comp in TRANS else "urad"
            value_digits = 4 if comp in TRANS else 3
            filename = "%s_%s_simulated.csv" % (name, comp)
            path = os.path.join(OUT, filename)
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f, lineterminator="\n")
                w.writerow([pos_header, "%s_%s" % (comp, value_unit)])
                for p, v in zip(positions, values):
                    w.writerow(["%.*f" % (decimals, p),
                                "%.*f" % (value_digits, v)])
            with open(path, "rb") as f:
                raw = f.read()
            maps.append({
                "filename": filename, "axis": name, "component": comp,
                "axis_class": axis_class(name), "controller": ctl,
                "command_unit": unit,
                "value_unit": "um" if comp in TRANS else "urad",
                "node_count": node_count, "coverage": coverage,
                "error_min": round(min(values), value_digits),
                "error_max": round(max(values), value_digits),
                "sha256": hashlib.sha256(raw).hexdigest().upper(),
                "generation_seed": seed,
            })
    manifest = {
        "schema_version": 1, "dataset_id": DATASET_ID,
        "source_type": "synthetic_engineering_representative",
        "measured_data": False,
        "warning": "仿真与接口验证数据，不得作为实机标定或验收结论。",
        "coordinate_frame": "axis_parent_component_frame",
        "sign_convention": "actual_minus_nominal",
        "interpolation": "piecewise_linear_inside_coverage",
        "outside_coverage": "ideal_zero_unmeasured",
        "axis_count": len(ac.AXES), "map_count": len(maps), "maps": maps,
    }
    with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
        f.write("\n")
    with open(os.path.join(OUT, "low_order.json"), "w", encoding="utf-8") as f:
        json.dump({"enabled": True,
                   "note": "scale_ppm 按轴分级：精密 ±5~10、辅助 ±20；"
                           "direction_*_urad 暂不用留 0",
                   "axes": low_order_axes}, f, ensure_ascii=False, indent=1)
        f.write("\n")
    print("error_maps: %d maps for %d axes -> %s"
          % (len(maps), len(ac.AXES), OUT))
    return manifest


if __name__ == "__main__":
    generate()
