# -*- coding: utf-8 -*-
"""points.py — IO 点位注册表（线程安全），倍福式周期扫描映像表模型。

- live：输出映像（外部观测读到的值），get/subtree/snapshot/has 全部读 live；
- pending：输入映像，set() 只写 pending，由引擎按扫描周期（plc_scan_ms）
  调 commit() 合并进 live —— 写入最多晚一个扫描周期被外部观测到；
- 同周期多次 set 同点，后者覆盖前者（pending 即最新值）；
- 未知点位保持原有语义：set 时立即建档生效（dir=observe，不经 pending），
  便于引擎写点位表之外的合成观测点（气路压强/流量等）。
"""
import json
import threading


class Points:
    def __init__(self, path=None):
        self._lock = threading.Lock()
        self._d = {}
        self._pending = {}
        if path:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            self._d = data.get("points", {})

    def set(self, name, value):
        with self._lock:
            if name in self._d:
                self._pending[name] = value      # 注册点位：写输入映像
            else:
                kind = ("bool" if isinstance(value, bool)
                        else "int" if isinstance(value, int)
                        else "real")
                self._d[name] = {"dir": "observe", "kind": kind, "value": value}

    def commit(self):
        """扫描周期到：把输入映像合并进输出映像（返回合并条数）。"""
        with self._lock:
            if not self._pending:
                return 0
            n = 0
            for name, value in self._pending.items():
                rec = self._d.get(name)
                if rec is not None:
                    rec["value"] = value
                    n += 1
            self._pending.clear()
            return n

    def pending_count(self):
        with self._lock:
            return len(self._pending)

    def get(self, name, default=None):
        with self._lock:
            rec = self._d.get(name)
            return default if rec is None else rec["value"]

    def has(self, name):
        with self._lock:
            return name in self._d

    def snapshot(self):
        """{name: value} 一致快照（live 输出映像）。"""
        with self._lock:
            return {k: v["value"] for k, v in self._d.items()}

    def snapshot_meta(self):
        with self._lock:
            return json.loads(json.dumps(self._d))

    def subtree(self, prefix):
        with self._lock:
            return {k: v["value"] for k, v in self._d.items()
                    if k == prefix or k.startswith(prefix + ".")}

    def __len__(self):
        return len(self._d)
