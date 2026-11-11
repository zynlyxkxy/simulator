# -*- coding: utf-8 -*-
"""gen_vision_templates.py — 合成标准对准标记模板（2026-09-05 迭代 17）。

重写 tpl_die_head / tpl_head_chuck（up 相机）与 tpl_post_inspect（down 相机）
三个占位模板（原 PNG 是 tpl_cv_box.png 的拷贝，与场景标记完全对不上，
REAL 识别必败），并把 cv_quad 的 shape 字段补成 "quad"。

画法：复用 vision_model.shape_primitives（quad/cross 图元定义单一事实源），
按相机 pixel_um 换算像素尺寸（up/down 均 0.6µm/px → 150µm 标记=250px），
灰度/模糊/噪声模仿真实渲染帧（BG_LIGHT 底 + MARK_DARK 暗标记 + 轻高斯 +
轻噪声），画布 400×400（标记居中，四周留背景过渡带——NCC 需要边缘/背景
对比，纯标记填满画幅会零方差退化）。

用法：cd backend && python tools/gen_vision_templates.py   （幂等重写）
"""
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
RUNTIME = os.path.join(ROOT, "..", "runtime")
sys.path.insert(0, os.path.abspath(RUNTIME))

import vision_model as vm  # noqa: E402

TEMPLATES_JSON = os.path.join(RUNTIME, "vision_templates.json")
TEMPLATES_DIR = os.path.join(RUNTIME, "vision_templates")
CREATED = 1788100000.0          # 固定时间戳（生成产物，幂等 diff 稳定）
CANVAS = 400                    # 画布边长 px
K_MM_PX = 0.0006                # up/down 相机 0.6µm/px（machine_constants
                                # cameras up/down: pixel_um 6.0 / mag 10.0）
BLUR_SIGMA = 0.8                # 轻高斯（模仿渲染帧合焦层的像素级软化）
NOISE_SIGMA = 2.1               # 轻噪声 ≈ K_NOISE·√gain/√exposure（8ms/1×）

# 三个占位模板：形状按机械常数 vision_marks 语义（die 角标/吸盘标记/键合后
# 检测 die 角标都是 cross 150µm，见 recipe.film.marks）
# cv_quad（2026-09-07 迭代 20 加入）：旧 PNG 是 08-24 建库时的实拍裁图，
# 边界几何 ~112µm 与当前 quad 标记 150µm 不符（REAL 套刻帧上分数 0.48
# 不过阈）——按真几何重生成
TARGETS = [
    {"name": "tpl_die_head", "cam": "up", "shape": "cross", "size_um": 150,
     "_note": "合成模板（2026-09-05 迭代 17，tools/gen_vision_templates.py）："
              "键合头上 die 角标 cross 150µm，供机械常数 vision_marks."
              "tpl_die_head 默认引用；真机标定后在视觉页按实测帧重建替换"},
    {"name": "tpl_head_chuck", "cam": "up", "shape": "cross", "size_um": 150,
     "_note": "合成模板（2026-09-05 迭代 17，tools/gen_vision_templates.py）："
              "键合头吸盘标记 cross 150µm，供机械常数 vision_marks."
              "tpl_head_chuck 默认引用（as_head_center 测校预留）"},
    {"name": "tpl_post_inspect", "cam": "down", "shape": "cross",
     "size_um": 150,
     "_note": "合成模板（2026-09-05 迭代 17，tools/gen_vision_templates.py）："
              "键合后红外透视检测 die 角标 cross 150µm，供机械常数 "
              "vision_marks.tpl_post_inspect 默认引用"},
    {"name": "cv_quad", "cam": "down", "shape": "quad", "size_um": 150,
     "_note": "合成模板（2026-09-07 迭代 20，tools/gen_vision_templates.py）："
              "晶圆 site 对准标记 quad 150µm（150µm 边界四角各 30µm 小方块"
              "），供机械常数 vision_marks.tpl_wafer_site 默认引用（迭代 20 起"
              "替代占位期 cross 图 cv_cross_0824——后者 REAL 套刻同帧会把 "
              "die cross 当最佳峰）；真机标定后在视觉页按实测帧重建替换"},
]


def synth_template(shape, size_um):
    """合成一张模板图 -> numpy uint8 (CANVAS, CANVAS)，标记居中。"""
    n = vm.np()
    cv2 = vm.cv2_or_none()
    if cv2 is None:
        raise RuntimeError("gen_vision_templates 需要 cv2（opencv-python）")
    mark = vm.Mark("gen", shape, 0.0, 0.0, 0.0, 0.0, size_um / 1000.0)
    img = n.full((CANVAS, CANVAS), vm.BG_LIGHT, dtype=n.float32)
    gray = float(vm.BG_LIGHT * vm.MARK_DARK)
    cc = CANVAS / 2.0
    for prim in vm.shape_primitives(mark):
        kind = prim[0]
        if kind == "rect":
            _k, rcx, rcy, rw, rh = prim
            # mark 局部 mm → 画布 px（yaw=0 轴对齐，中心居中）
            x0 = cc + (rcx - rw / 2.0) / K_MM_PX
            y0 = cc + (rcy - rh / 2.0) / K_MM_PX
            x1 = cc + (rcx + rw / 2.0) / K_MM_PX
            y1 = cc + (rcy + rh / 2.0) / K_MM_PX
            pts = n.array([[[int(round(x0)), int(round(y0))],
                            [int(round(x1)), int(round(y0))],
                            [int(round(x1)), int(round(y1))],
                            [int(round(x0)), int(round(y1))]]],
                          dtype=n.int32)
            cv2.fillPoly(img, [pts], gray)
        elif kind == "ring":
            _k, rcx, rcy, r_out, r_in = prim
            c = (int(round(cc + rcx / K_MM_PX)),
                 int(round(cc + rcy / K_MM_PX)))
            cv2.circle(img, c, int(round(r_out / K_MM_PX)), gray, -1)
            cv2.circle(img, c, int(round(r_in / K_MM_PX)),
                       float(vm.BG_LIGHT), -1)
        else:
            raise RuntimeError("未支持的图元: %r" % (prim,))
    img = cv2.GaussianBlur(img, (0, 0), BLUR_SIGMA)
    rng = n.random.default_rng(20260905)    # 固定种子：产物幂等
    img = img + rng.normal(0.0, NOISE_SIGMA, img.shape).astype(n.float32)
    return n.clip(img, 0, 255).astype(n.uint8)


def main():
    with open(TEMPLATES_JSON, encoding="utf-8") as f:
        doc = json.load(f)
    tpls = doc["templates"]
    for spec in TARGETS:
        name = spec["name"]
        if name not in tpls:
            raise RuntimeError("vision_templates.json 缺模板条目 %r" % name)
        img = synth_template(spec["shape"], spec["size_um"])
        fname = "tpl_%s.png" % name
        os.makedirs(TEMPLATES_DIR, exist_ok=True)
        with open(os.path.join(TEMPLATES_DIR, fname), "wb") as f:
            f.write(vm.png_encode_gray(img))
        meta = tpls[name]
        meta.update({
            "file": fname, "shape": spec["shape"],
            "roi": [0, 0, CANVAS, CANVAS], "w": CANVAS, "h": CANVAS,
            "threshold": 0.6, "created": CREATED,
            "source": "gen_vision_templates",
            "_note": spec["_note"],
            "light_on_when_created": True,
        })
        print("[gen] %s: %s %dum -> %s (%dx%d)"
              % (name, spec["shape"], spec["size_um"], fname, CANVAS, CANVAS))
    # cv_quad shape 补正（占位期遗留空字符串）
    if tpls.get("cv_quad", {}).get("shape") != "quad":
        tpls["cv_quad"]["shape"] = "quad"
        print("[gen] cv_quad shape -> quad")
    tmp = TEMPLATES_JSON + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
        f.write("\n")
    os.replace(tmp, TEMPLATES_JSON)
    print("[gen] vision_templates.json 已更新")

    # 自检：合成帧（同画法把标记放到偏移位）上自匹配，score 应 ≈1
    import vision_service as vs
    matcher = vs.Matcher()
    for spec in TARGETS:
        meta = tpls[spec["name"]]
        tpl = matcher.load_template_img(meta)
        n = vm.np()
        frame = n.full((1200, 1200), vm.BG_LIGHT, dtype=n.float32)
        off = 137                      # 任意偏移，验证质心修正后的定位精度
        frame[off:off + CANVAS, off:off + CANVAS] = \
            matcher.load_template_img(meta).astype(n.float32)
        entries, ms = matcher.match(frame.astype(n.uint8), [meta], 0.6)
        if not entries:
            raise RuntimeError("自检失败：%s 无命中" % spec["name"])
        e = max(entries, key=lambda x: x["score"])
        # 质心修正后命中点应 = off + CANVAS/2
        exp = off + CANVAS / 2.0
        ok = e["score"] >= 0.95 and abs(e["col"] - exp) < 2.0 \
            and abs(e["row"] - exp) < 2.0
        print("[selftest] %s: score=%.3f col=%.1f row=%.1f (期 %.1f) %s"
              % (spec["name"], e["score"], e["col"], e["row"], exp,
                 "OK" if ok else "FAIL"))
        if not ok:
            raise RuntimeError("自检失败：%s" % spec["name"])
    print("[gen] done")


if __name__ == "__main__":
    main()
