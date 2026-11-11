#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_light_online.py — 由细化前端源生成联机版（旁路，不动源文件）。

用法：
    python light_test/tools/build_light_online.py

产物 = 源 1928_front_light.html 逐字节内容，仅两处改动：
  1. 渲染循环起点 `function fe(L){requestAnimationFrame(fe),` 前置
     window.__TD_VIZ__ 场景钩子（scene/axes/controls/setAxis/obj3d/defs/Box3/V3）；
  2. 结尾 `</script></body></html>` → 追加内联 联机注入_light.js。

为什么内联而不是 <script src>：file:// 下外部脚本是 opaque origin，异常会被
洗成 "Script error."（无行号无消息），并触发查看器红色错误弹窗。

断言：
  1. 构建前后源文件 sha256 不变；
  2. 钩子锚点在源文件中唯一；
  3. 注入脚本内不得出现 "</script"；
  4. 产物 = 源前缀逐字节相同。
"""
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))        # light_test/tools
LT = os.path.dirname(HERE)                               # light_test

SRC_HTML = os.path.join(LT, "1928_front_light.html")
INJECT_JS = os.path.join(LT, "联机注入_light.js")
OUT_HTML = os.path.join(LT, "1928_front_light_online.html")

PROJECT_KEY = "window.__TWIN_DOM_PROJECT__="

HOOK_OLD = "function fe(L){requestAnimationFrame(fe),"
HOOK_NEW = ("window.__TD_VIZ__={scene:c,camera:u,renderer:a,obj3d:E,defs:R,"
            "axes:e.axes,controls:V,setAxis:je,project:e,Box3:ct,V3:C};"
            + HOOK_OLD)

TAIL = "</script></body></html>"

# 模型侧量程补丁（只作用于产物，源文件不动）：
#   axis_ISS_FA_X 源模型量程 0~1235（+ = 远离键合头），但 0=取片位、键合头在
#   X=-42.5，交接必须走 -1242.35 —— 源模型的正方向接反了，量程也差 7.35mm。
#   这里把下限放到 -1243，让仿真把翻转台送到键合头下方时模型能跟上。
AXIS_PATCH = {
    "axis_ISS_FA_X": {"min": -1243.0},
}


def patch_project(text):
    """改内联工程 JSON 的轴量程；返回 (新文本, 变更说明列表)。"""
    i = text.index(PROJECT_KEY) + len(PROJECT_KEY)
    obj, end = json.JSONDecoder().raw_decode(text, i)
    notes = []
    by_id = {a["id"]: a for a in obj.get("axes", [])}
    for aid, patch in AXIS_PATCH.items():
        ax = by_id.get(aid)
        if not ax:
            continue
        for k, v in patch.items():
            if ax.get(k) != v:
                notes.append("%s.%s %s → %s" % (aid, k, ax.get(k), v))
                ax[k] = v
    if not notes:
        return text, notes
    new = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    return text[:i] + new + text[end:], notes


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    before = sha256(SRC_HTML)
    with open(SRC_HTML, "rb") as f:
        html = f.read()
    with open(INJECT_JS, "rb") as f:
        js = f.read()

    text = html.decode("utf-8")

    # 新导出（portable-html 2.0 / TWZ 压缩）不走本构建器：
    #   - 工程数据不在内联 JSON 里，轴量程无法在字符串层打补丁；
    #   - 渲染循环内的变量名与 1.0.4 不同，钩子点位会失配；
    #   - 这类导出通常已自带 联机注入_细化.js（默认 9010），改由 URL 参数接旁路栈。
    if "__TWIN_DOM_TWZ_B64__" in text or PROJECT_KEY not in text:
        print("[skip] 这是 portable-html 2.0（TWZ 压缩）导出，本构建器不适用。")
        print("       处理办法二选一：")
        print("       a) 直接用页面自带的注入，打开时带端口参数：")
        print("          1928_front_light_online.html?ws=9030")
        print("       b) 若要换成 light_test/联机注入_light.js，需先把自带注入块剥掉再内联。")
        return 2

    assert b"</script" not in js.lower(), "联机注入_light.js 含 '</script'，会提前闭合标签"

    text, notes = patch_project(text)
    assert text.count(HOOK_OLD) == 1, "钩子锚点不唯一或不存在：%r" % HOOK_OLD
    text = text.replace(HOOK_OLD, HOOK_NEW)
    assert text.count(HOOK_NEW) == 1, "钩子注入后计数异常"
    patched = text.encode("utf-8")

    assert text.endswith(TAIL), "源文件结尾不是 %r" % TAIL
    body = patched[: -len(TAIL)]
    out = body + ("</script>\n<script>\n").encode("utf-8") + js + \
          ("\n</script>\n</body></html>").encode("utf-8")

    n = len(body)
    assert out[:n] == body, "产物前缀与源不一致"

    with open(OUT_HTML, "wb") as f:
        f.write(out)

    after = sha256(SRC_HTML)
    assert before == after, "源文件在构建过程中被改动！"

    print("[build] 源文件   %s (%d bytes, sha256 %.12s 未改动)" %
          (os.path.basename(SRC_HTML), len(html), before))
    if notes:
        print("[build] 模型量程补丁（仅产物）：" + "; ".join(notes))
    else:
        print("[build] 模型量程补丁：无")
    print("[build] 注入脚本 %s (%d bytes)" % (os.path.basename(INJECT_JS), len(js)))
    print("[build] 产物     %s (%.2f MB)" %
          (os.path.basename(OUT_HTML), len(out) / 1e6))
    print("[build] OK：钩子 + 内联注入，源文件逐字节保留")
    return 0


if __name__ == "__main__":
    sys.exit(main())
