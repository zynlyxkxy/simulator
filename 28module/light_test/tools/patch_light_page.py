#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_light_page.py — 给细化前端联机产物打「几何校正」补丁（只改产物，不改源）。

用法：
    python light_test/tools/patch_light_page.py            # 应用到产物
    python light_test/tools/patch_light_page.py --check     # 只检查是否已应用

背景（2026-10-28 实测）：
  1. 硅片坐面：页面注入把晶圆体抬了 2.54mm——那是**旧模型**的标定
     （旧吸盘网格顶 882.28 比当时认定的物理陶瓷面 884.9 低 2.6mm）。
     细化模型的吸盘网格顶实测就是 900.00，与键合接触链自洽（头底 900 +
     片厚 1.0 = 901 = 晶圆上表面），再抬 2.54 会让晶圆悬在陶瓷面之上。
  2. E_Pin 压紧零位：细化模型 E_Pin 组在轴 0 时针尖世界 Z=903.5，比陶瓷面
     900 高 3.5mm；而轴名标注「0=压紧位」。晶圆坐面 900 + 片厚 1.0 = 901，
     所以模型值要整体偏 −2.5（903.5 → 901）才是"压在晶圆上表面"。
     负值由注入自带的滑块下限扩展逻辑承接（与 FA_X 同机制）。

两处都是幂等的；重复运行只会打印「已应用」。
"""
import argparse
import hashlib
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LT = os.path.dirname(HERE)
PAGE = os.path.join(LT, "1928_front_light_online.html")
BAK = PAGE + ".pre_patch"

# ---- 补丁 1：晶圆坐面 ----
WAFER_OLD_COMMENT = """      // 以吸盘当前包围盒顶面为承载面，世界坐标转吸盘局部后挂为子节点（随轴运动）。
      // 浮起量 2.54mm（2026-09-06 迭代 17 标定，旧值 0.5）：TwinDOM 晶圆吸盘
      // 网格顶 882.28 / 陶瓷面网格顶 881.9 比后端物理陶瓷面 CHUCK_TOP_Z=884.9
      // 低 ~2.6mm，而键合头 z 链是轴驱动物理准确的（有效接触位 -167.43 时
      // head die 底面世界 z=885.82≈物理 885.9）——为让键合时 head 上的 die
      // 底面贴合晶圆面（此前视觉缝 ~2.0mm，"上芯片没接触上"的显示侧），
      // 晶圆顶面对齐头侧真值链：浮起=885.82−882.28−T(1.0)≈2.54。
      // die map/标记/已键合 die 全是晶圆体子节点或 waferBase 派生，
      // 一并随动，内部一致性不变。
"""
WAFER_NEW_COMMENT = """      // 以吸盘包围盒顶面为承载面（晶圆底面贴合陶瓷面），世界坐标转吸盘局部后
      // 挂为子节点随轴运动。
      // 【2026-10-28 校正】旧版此处额外抬 2.54mm，是上一代模型的标定（旧吸盘
      // 网格顶 882.28 比当时认定的陶瓷面 884.9 低 2.6mm）。细化模型实测吸盘网格
      // 顶=900.00 即真实陶瓷面，键合头底在 BS_Z=-147 时正好落在"陶瓷面+片厚"，
      // 因此浮起量归零，晶圆体底面直接坐在陶瓷面上；抬 2.54 会悬空并让 E_Pin
      // 看起来"从晶圆里穿出来"。
      // die map/标记/已键合 die 全是晶圆体子节点或 waferBase 派生，一并随动。
"""
WAFER_OLD_LINE = "b.max.z + T / 2 + 2.54"
WAFER_NEW_LINE = "b.max.z + T / 2"

# ---- 补丁 2：E_Pin 压紧零位偏置 ----
EPIN_OLD = "{ sim: 'E_EPIN_Z_1', model: 'axis_WS_EPIN_Z',        t: 'lin', drag: 'E_EPIN_Z' },"
EPIN_NEW = ("{ sim: 'E_EPIN_Z_1', model: 'axis_WS_EPIN_Z',        t: 'lin', drag: 'E_EPIN_Z',\n"
            "      offset: -2.5 },")


def sha(b):
    return hashlib.sha256(b).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只报告状态，不写文件")
    a = ap.parse_args()

    if not os.path.exists(PAGE):
        print("[error] 页面不存在: %s" % PAGE)
        return 1
    with open(PAGE, "rb") as f:
        raw = f.read()
    before = sha(raw)
    s = raw.decode("utf-8")
    changed = []

    # 补丁 1（注释与坐标行一起替换；已应用则都命中"新"串）
    if WAFER_OLD_LINE in s:
        assert s.count(WAFER_OLD_COMMENT) == 1, "晶圆坐面注释块不唯一"
        assert s.count(WAFER_OLD_LINE) == 1, "晶圆坐面坐标行不唯一"
        s = s.replace(WAFER_OLD_COMMENT, WAFER_NEW_COMMENT)
        s = s.replace(WAFER_OLD_LINE, WAFER_NEW_LINE)
        changed.append("晶圆坐面：去掉 +2.54mm 浮起（底面贴陶瓷面 900）")
    elif WAFER_NEW_LINE in s and "【2026-10-28 校正】" in s:
        print("[skip] 晶圆坐面已是校正后版本")
    else:
        print("[error] 找不到晶圆坐面代码（页面版本可能又变了）")
        return 2

    # 补丁 2
    if EPIN_OLD in s:
        assert s.count(EPIN_OLD) == 1, "E_Pin 映射行不唯一"
        s = s.replace(EPIN_OLD, EPIN_NEW)
        changed.append("E_Pin 压紧零位：offset -2.5（针尖 903.5 → 901 = 晶圆上表面）")
    elif "offset: -2.5 }" in s and "axis_WS_EPIN_Z" in s:
        print("[skip] E_Pin 偏置已应用")
    else:
        print("[error] 找不到 E_Pin 映射行")
        return 2

    if not changed:
        print("[ok] 全部补丁已应用，无需改动（sha %s）" % before)
        return 0
    if a.check:
        for c in changed:
            print("[would] " + c)
        return 0

    if not os.path.exists(BAK):
        shutil.copy2(PAGE, BAK)
        print("[bak] %s" % os.path.basename(BAK))
    with open(PAGE, "wb") as f:
        f.write(s.encode("utf-8"))
    for c in changed:
        print("[patch] " + c)
    print("[ok] sha %s → %s" % (before, sha(open(PAGE, "rb").read())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
