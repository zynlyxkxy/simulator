#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_online_html.py — 一键重新生成 d2w_online.html。

用法（任意目录下均可）：
    python backend/tools/build_online_html.py

产物 = 原 d2w_front.html 逐字节内容，仅两处改动：
  1. 运行时尾部 `qf();})();` → 前置暴露 window.__TD_VIZ__ 场景钩子
     （scene/obj3d/defs/make/V3/Box3，供联机注入.js 真空/工件可视化）；
  2. 结尾 `</body></html>` → `</body><script>...联机注入.js 全文...</script></html>`。

为什么内联而不是 <script src>：file:// 页面通过 src 引用的外部脚本
被视为跨域（opaque origin），其中任何异常都会被浏览器洗成
"Script error."（无行号无消息），并触发 TwinDOM 查看器的红色错误弹窗；
内联后脚本与页面同源，window.onerror 能拿到真实 message 和行号。

断言：
  1. 构建前后原 html 的 SHA256 不变（本脚本只读它，绝不写它）；
  2. 产物 = 原字节串去尾 + 内联脚本 + 收尾（逐字节前缀校验）；
  3. 注入脚本内不得出现 "</script"（否则会提前闭合内联 script 标签）。
"""
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))          # backend/tools
ROOT = os.path.dirname(os.path.dirname(HERE))              # 28module
HTML_DIR = os.path.join(ROOT, "html_model")
SRC_HTML = os.path.join(HTML_DIR, "d2w_front.html")
INJECT_JS = os.path.join(HTML_DIR, "联机注入.js")
OUT_HTML = os.path.join(HTML_DIR, "d2w_online.html")

TAIL = b"</body></html>"

# 运行时场景钩子：查看器 IIFE 内的一切（scene/对象表/网格工厂）默认不外露。
# 在产物里把结尾启动渲染的 `qf();})();` 替换为带 window.__TD_VIZ__ 暴露的版本，
# 供联机注入.js 做真空/工件可视化。原 html 不改，补丁只作用于产物。
HOOK_OLD = "qf();})();"
HOOK_NEW = ("window.__TD_VIZ__={scene:Xt,make:If,obj3d:ls,defs:Oc,V3:R,Box3:Pt};"
            "qf();})();")
#   scene=THREE.Scene  obj3d=Map(对象id→Object3D)  defs=Map(对象id→工程定义)
#   make=If(对象定义JSON→Object3D 工厂)  V3=Vector3  Box3=Box3


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

    assert b"</script" not in js.lower(), \
        "联机注入.js 含有 '</script'，会提前闭合内联 script 标签"
    assert html.endswith(TAIL), \
        "原 html 不以 %r 结尾，无法定位注入点" % TAIL

    # 场景钩子补丁（作用于内存中的产物副本，原文件不动）
    text = html.decode("utf-8")
    assert text.count(HOOK_OLD) == 1, "运行时钩子点 %r 不唯一或不存在" % HOOK_OLD
    text = text.replace(HOOK_OLD, HOOK_NEW)
    html_patched = text.encode("utf-8")

    out = (html_patched[:-len(TAIL)] + b"</body>\n<script>\n" + js +
           b"\n</script>\n</html>")

    # 逐字节前缀校验：产物必须以前 N 字节 == 补丁版前 N-12 字节
    n = len(html_patched) - len(TAIL)
    assert out[:n] == html_patched[:n], "产物前缀与原文件不一致"

    with open(OUT_HTML, "wb") as f:
        f.write(out)

    after = sha256(SRC_HTML)
    assert before == after, "原文件在构建过程中被改动！"

    print("[build] 原文件   %s (%d bytes, sha256 %.12s 未改动)"
          % (SRC_HTML, len(html), before))
    print("[build] 注入脚本 %s (%d bytes)" % (INJECT_JS, len(js)))
    print("[build] 产物     %s (%d bytes)" % (OUT_HTML, len(out)))
    print("[build] OK：产物 = 原文件逐字节 + 内联 <script> 块")
    return 0


if __name__ == "__main__":
    sys.exit(main())
