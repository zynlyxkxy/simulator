#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""light_cdp.py — 细化前端联机版的浏览器侧驱动（Edge headless + CDP，纯标准库）。

用法：
    python light_test/tools/light_cdp.py check
    python light_test/tools/light_cdp.py eval "return window.__TD_VIZ__.axes.length"
    python light_test/tools/light_cdp.py eval --file expr.js --out result.json
    python light_test/tools/light_cdp.py shot --png out.png

参数：
    --page PATH    页面（默认 light_test/1928_front_light_online.html）
    --wait SEC     打开后等待秒数（默认 20；24MB 页面 + 871 个 GLB 首帧较慢）
    --gpu          启用 GPU（默认开；`--no-gpu` 退回 SwiftShader）
    --port N       CDP 端口（默认 9245）

约束：只用标准库；不联网；退出时只杀本次启动的 Edge 进程树。
"""
import argparse
import base64
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
from urllib.parse import quote

HERE = os.path.dirname(os.path.abspath(__file__))
LT = os.path.dirname(HERE)
DEFAULT_PAGE = os.path.join(LT, "1928_front_light_online.html")

EDGE_CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]


# ---------------- 最小 CDP WebSocket 客户端 ----------------
class CDP:
    def __init__(self, ws_url):
        assert ws_url.startswith("ws://")
        rest = ws_url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port)), timeout=20)
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
            chunk = self.sock.recv(1 << 16)
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

    def call(self, method, params=None, timeout=180):
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
        self.sock.settimeout(timeout)
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


class Session:
    """with Session(page=..., wait=20) as s: s.eval_js(...), s.screenshot(...)"""

    def __init__(self, page, wait=20.0, port=9245, gpu=True, width=1600, height=1000):
        page, _, self.query = page.partition("?")
        self.page = os.path.abspath(page)
        self.wait = wait
        self.port = port
        self.gpu = gpu
        self.width, self.height = width, height
        self.proc = None
        self.profile = None
        self.cdp = None

    def __enter__(self):
        edge = next((p for p in EDGE_CANDIDATES if os.path.exists(p)), None)
        assert edge, "未找到 msedge.exe"
        assert os.path.exists(self.page), "页面不存在: %s" % self.page
        self.profile = tempfile.mkdtemp(prefix="edge_light_")
        args = [edge, "--headless=new", "--no-first-run",
                "--no-default-browser-check",
                "--remote-debugging-port=%d" % self.port,
                "--user-data-dir=" + self.profile,
                "--window-size=%d,%d" % (self.width, self.height)]
        if self.gpu:
            args += ["--use-angle=d3d11", "--enable-gpu"]
        else:
            args += ["--disable-gpu"]
        args.append("about:blank")
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        t0 = time.time()
        while time.time() - t0 < 25:
            try:
                urllib.request.urlopen(
                    "http://127.0.0.1:%d/json/version" % self.port, timeout=2).read()
                break
            except Exception:
                time.sleep(0.3)
        url = "file:///" + self.page.replace(os.sep, "/")
        if self.query:
            url += "?" + self.query
        req = urllib.request.Request(
            "http://127.0.0.1:%d/json/new?%s" % (self.port, quote(url)), method="PUT")
        with urllib.request.urlopen(req, timeout=8) as r:
            target = json.loads(r.read().decode("utf-8"))
        self.cdp = CDP(target["webSocketDebuggerUrl"])
        self.cdp.call("Runtime.enable")
        self.cdp.call("Page.enable")
        self.cdp.call("Emulation.setDeviceMetricsOverride",
                      {"width": self.width, "height": self.height,
                       "deviceScaleFactor": 1, "mobile": False})
        if self.wait:
            time.sleep(self.wait)
        return self

    def eval_js(self, expr, await_promise=False, timeout=180):
        res = self.cdp.call("Runtime.evaluate",
                            {"expression": expr, "returnByValue": True,
                             "awaitPromise": await_promise}, timeout=timeout)
        if res.get("exceptionDetails"):
            raise RuntimeError("JS 异常: %s" % json.dumps(
                res["exceptionDetails"], ensure_ascii=False)[:800])
        return res.get("result", {}).get("value")

    def screenshot(self, png_path):
        res = self.cdp.call("Page.captureScreenshot", {"format": "png"})
        data = base64.b64decode(res["data"])
        os.makedirs(os.path.dirname(os.path.abspath(png_path)), exist_ok=True)
        with open(png_path, "wb") as f:
            f.write(data)
        return len(data)

    def __exit__(self, *exc):
        if self.cdp:
            self.cdp.close()
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.profile:
            shutil.rmtree(self.profile, ignore_errors=True)
        return False


HUD_JS = r"""
(function(){
  var out={};
  var hud=document.getElementById('lightHud');
  out.hud=hud?hud.textContent:null;
  out.hasHook=!!window.__TD_VIZ__;
  if(window.__TD_VIZ__){
    out.axes=window.__TD_VIZ__.axes.length;
    out.hookKeys=Object.keys(window.__TD_VIZ__);
    out.controls=window.__TD_VIZ__.controls?window.__TD_VIZ__.controls.size:0;
  }
  out.sliders=document.querySelectorAll('input[type=range]').length;
  out.errBox=null;
  var divs=document.querySelectorAll('div');
  for(var i=0;i<divs.length;i++){
    var t=divs[i].textContent||'';
    if(t.indexOf('三维模型启动失败')>=0&&t.length<500){out.errBox=t;break;}
  }
  out.title=document.title;
  return JSON.stringify(out);
})()
"""


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["check", "eval", "shot"])
    ap.add_argument("expr", nargs="?", help="eval 的内联表达式")
    ap.add_argument("--file", help="从文件读表达式")
    ap.add_argument("--out", help="eval 结果写文件")
    ap.add_argument("--png", help="shot 输出路径")
    ap.add_argument("--page", default=DEFAULT_PAGE)
    ap.add_argument("--wait", type=float, default=20.0)
    ap.add_argument("--port", type=int, default=9245)
    ap.add_argument("--no-gpu", action="store_true")
    a = ap.parse_args(argv)

    with Session(a.page, wait=a.wait, port=a.port, gpu=not a.no_gpu) as s:
        if a.cmd == "check":
            info = json.loads(s.eval_js(HUD_JS))
            print(json.dumps(info, ensure_ascii=False, indent=1))
            ok = bool(info.get("hud")) and "已连接" in (info.get("hud") or "")
            print("\n==== %s ====" % ("PASS" if ok else "FAIL"))
            return 0 if ok else 1
        if a.cmd == "eval":
            expr = open(a.file, encoding="utf-8").read() if a.file else a.expr
            assert expr, "需要表达式（内联或 --file）"
            val = s.eval_js(expr, await_promise=True)
            text = val if isinstance(val, str) else json.dumps(val, ensure_ascii=False, indent=1)
            if a.out:
                with open(a.out, "w", encoding="utf-8") as f:
                    f.write(text)
                print("[out] %s (%d bytes)" % (a.out, len(text)))
            else:
                print(text)
            return 0
        if a.cmd == "shot":
            assert a.png, "shot 需要 --png"
            n = s.screenshot(a.png)
            print("[png] %s (%d bytes)" % (a.png, n))
            return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
