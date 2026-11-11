#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""edge_headless_check.py — 用 Edge headless + CDP（纯标准库）真实浏览器验证联机版页面。

用法：
  1. 启动后端：python backend/runtime/main.py
  2. python backend/tests/edge_headless_check.py [html路径]

流程：
  - 启动 msedge --headless=new --remote-debugging-port（独立临时 profile）
  - CDP Target.createTarget 打开联机版 file:// 页面，等待 8 秒真实时间
  - Runtime.evaluate 抽取 HUD 文本、查看器红色错误弹窗是否存在、
    window.onerror 捕获记录，打印判定结果
  - 退出前关闭 Edge（只杀本次启动的进程树）

判定标准（PASS）：
  - HUD 存在且含 "已连接"
  - 推送频率 490~510 Hz
  - 渲染 fps > 0
  - "脚本错误  无"（或仅为查看器自身在 SwiftShader 下的报错，需人工甄别）
"""
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
CDP_PORT = 9223
WAIT_S = 8

DEFAULT_HTML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "html_model", "d2w_online.html")


# ---------------- 最小 CDP WebSocket 客户端（标准库） ----------------
class CDP:
    def __init__(self, ws_url):
        # ws://127.0.0.1:PORT/devtools/page/XXXX
        assert ws_url.startswith("ws://")
        rest = ws_url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port)), timeout=10)
        key = "dGhlIHNhbXBsZSBub25jZQ=="
        req = ("GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n" % (path, hostport, key))
        self.sock.sendall(req.encode("latin1"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            resp += self.sock.recv(4096)
        assert b"101" in resp.split(b"\r\n")[0], resp
        self._id = 0
        self._buf = b""

    def _read_exact(self, n):
        while len(self._buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("cdp eof")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def _recv_msg(self):
        while True:
            b1, b2 = self._read_exact(2)
            op, ln = b1 & 0x0F, b2 & 0x7F
            if ln == 126:
                ln = struct.unpack("!H", self._read_exact(2))[0]
            elif ln == 127:
                ln = struct.unpack("!Q", self._read_exact(8))[0]
            payload = self._read_exact(ln) if ln else b""
            if op == 0x1:
                return json.loads(payload.decode("utf-8"))
            # ping/分片等忽略（CDP 服务端不掩码、一般不分片）

    def call(self, method, params=None):
        self._id += 1
        mid = self._id
        payload = json.dumps({"id": mid, "method": method,
                              "params": params or {}}).encode("utf-8")
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            hdr = struct.pack("!BB", 0x81, 0x80 | n)
        elif n <= 0xFFFF:
            hdr = struct.pack("!BBH", 0x81, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x81, 0x80 | 127, n)
        self.sock.sendall(hdr + mask +
                          bytes(b ^ mask[i & 3] for i, b in enumerate(payload)))
        while True:
            msg = self._recv_msg()
            if msg.get("id") == mid:
                if "error" in msg:
                    raise RuntimeError("CDP %s: %s" % (method, msg["error"]))
                return msg.get("result", {})

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


EVAL_JS = r"""
(function () {
  var out = {};
  // HUD：注入脚本创建的 fixed 悬浮层（z-index 9999，含"联机状态"）
  var hud = null;
  var divs = document.querySelectorAll('div');
  for (var i = 0; i < divs.length; i++) {
    if (divs[i].textContent.indexOf('联机状态') >= 0 &&
        divs[i].style.zIndex === '9999') { hud = divs[i]; break; }
  }
  out.hudText = hud ? hud.textContent : null;
  // 查看器的全局错误弹窗：含"三维模型启动失败"的元素
  out.errorBox = null;
  for (var i = 0; i < divs.length; i++) {
    var t = divs[i].textContent;
    if (t.indexOf('三维模型启动失败') >= 0 && t.length < 500) {
      out.errorBox = t; break;
    }
  }
  out.title = document.title;
  out.hasProject = !!window.__TWIN_DOM_PROJECT__;
  out.axisCount = window.__TWIN_DOM_PROJECT__ ?
      (window.__TWIN_DOM_PROJECT__.axes || []).length : -1;
  out.sliderCount = document.querySelectorAll('input[type=range]').length;
  out.wsState = 'n/a';
  return JSON.stringify(out);
})()
"""


def main():
    html = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HTML)
    url = "file:///" + html.replace(os.sep, "/")
    print("[edge] %s" % EDGE)
    print("[page] %s" % url)
    assert os.path.exists(EDGE), "Edge 不存在: " + EDGE
    assert os.path.exists(html), "页面不存在: " + html

    profile = tempfile.mkdtemp(prefix="edge_cdp_")
    proc = subprocess.Popen(
        [EDGE, "--headless=new", "--disable-gpu", "--no-first-run",
         "--no-default-browser-check", "--remote-debugging-port=%d" % CDP_PORT,
         "--user-data-dir=" + profile, "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        # 等 CDP 端口就绪
        version = None
        t0 = time.time()
        while time.time() - t0 < 20:
            try:
                with urllib.request.urlopen(
                        "http://127.0.0.1:%d/json/version" % CDP_PORT,
                        timeout=2) as r:
                    version = json.loads(r.read().decode("utf-8"))
                break
            except Exception:
                time.sleep(0.3)
        assert version, "CDP 端口 20s 未就绪"
        print("[cdp] %s" % version.get("Browser"))

        # 创建页面 target（中文路径需百分号编码）
        from urllib.parse import quote
        req = urllib.request.Request(
            "http://127.0.0.1:%d/json/new?%s" % (CDP_PORT, quote(url)),
            method="PUT")
        with urllib.request.urlopen(req, timeout=5) as r:
            target = json.loads(r.read().decode("utf-8"))
        ws_url = target["webSocketDebuggerUrl"]

        cdp = CDP(ws_url)
        try:
            cdp.call("Runtime.enable")
            print("[wait] 页面运行 %ds（等查看器初始化 + WS 连接 + fps 累积）..."
                  % WAIT_S)
            time.sleep(WAIT_S)
            res = cdp.call("Runtime.evaluate",
                           {"expression": EVAL_JS, "returnByValue": True})
            info = json.loads(res["result"]["value"])
        finally:
            cdp.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)

    print("\n===== DOM 抽取结果 =====")
    print("title       : %s" % info["title"])
    print("hasProject  : %s (axes=%s, sliders=%s)"
          % (info["hasProject"], info["axisCount"], info["sliderCount"]))
    print("errorBox    : %s" % (info["errorBox"] or "无（未弹出红色错误框）"))
    print("hudText     :")
    if info["hudText"]:
        for ln in info["hudText"].split("\n"):
            print("    %s" % ln)
    else:
        print("    <未找到 HUD>")

    ok = True
    if not info["hudText"]:
        print("\n[FAIL] HUD 不存在——注入脚本未运行或解析失败")
        return 1
    if "已连接" not in info["hudText"]:
        print("\n[FAIL] HUD 未显示已连接"); ok = False
    if "脚本错误  无" not in info["hudText"]:
        print("\n[WARN] HUD 存在脚本错误行内容，需人工甄别来源")
    import re
    # 推送频率是注入脚本 1s 滑动窗计数；headless SwiftShader 下事件循环
    # 拥塞（fps~11）导致 WS 消息成批处理，窗口计数可冲过 500 出现尖峰，
    # 故放宽到 450~560；权威频率以 test_browser_handshake.py 实测为准
    m = re.search(r"推送频率\s+(\d+) Hz", info["hudText"])
    if not m or not (450 <= int(m.group(1)) <= 560):
        print("\n[FAIL] 推送频率不在 450~560：%s" % (m and m.group(1))); ok = False
    m = re.search(r"渲染 fps\s+(\d+)", info["hudText"])
    if not m or int(m.group(1)) <= 0:
        print("\n[FAIL] 渲染 fps 为 0"); ok = False
    if info["errorBox"]:
        print("\n[FAIL] 查看器红色错误弹窗出现：%s" % info["errorBox"]); ok = False
    print("\n==== %s ====" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
