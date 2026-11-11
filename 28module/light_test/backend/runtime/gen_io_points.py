#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_io_points.py — 从点位表 dump 提取 GVL_IO.* / SystemMachine.* 变量名，
生成 io_points.json（带合理默认值）。可重复运行（幂等）。
"""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
DUMP = os.path.normpath(os.path.join(
    HERE, "..", "..", "1928运动固件点位表_V6", "点位表_dump.txt"))
OUT = os.path.join(HERE, "io_points.json")

TOKEN = re.compile(r"(?:GVL_IO|SystemMachine)(?:\.[A-Za-z_]\w*)+")

# 函数调用/命令名（点位表里以 Name(Type arg, ...) 形式出现的 Command）
COMMAND_LEAVES = {
    "SetTargetTempWithPlan", "StopTargetTempWithPlan", "StartPreHeat",
    "BondingByForceManual", "UpdateMode", "ES_Start", "ES_Stop", "ES_Reset",
    "ES_EmergencyStop", "ES_GvieBackDie", "ES_MoveToNextChip",
    "SetBufferNumber", "RunBuffer", "StopBuffer", "ResumeBuffer",
}

REAL_HINTS = ("oValue", "PTC", "NTC", "PT100", "Temper", "CurrentPos",
              "CurentPos", "Force", "CurTemp", "PreHeatTime")


def classify(name):
    leaf = name.rsplit(".", 1)[-1]
    if leaf in COMMAND_LEAVES:
        return "command", "null", None
    if any(h in name for h in REAL_HINTS):
        return "observe", "real", 0.0
    if leaf == "oBufferNum":
        return "observe", "int", 0
    if leaf.startswith("i"):
        return "observe", "bool", False
    if leaf.startswith("o"):
        return "observe", "bool", False
    # 其余（GVL_IO.oXxx 的输出侧、SystemMachine 末段无 i/o 前缀的）视为可写命令点
    return "command", "bool", False


def default_value(name, kind, value):
    """给关键点合理默认值。"""
    if kind == "real":
        if "Facility_PosPressure" in name:
            return 700.0     # 正压总表 0.7 MPa（kPa）
        if "Facility_NegPressure" in name:
            return -85.0     # 负压总表 -85 kPa
        if "Facility_N2Pressure" in name:
            return 500.0     # 氮气总表 0.5 MPa
        if any(h in name for h in ("PTC", "NTC", "PT100", "Temper", "CurTemp")):
            return 25.0      # 室温
        return 0.0
    if kind == "bool":
        if name.endswith(".iSystemOn"):
            return True
        if "EMO" in name:
            return False
        if "DoorStatus" in name or "LockStatus" in name or "ockStatus" in name:
            return True      # 门关好、锁锁好
        if "PowerOnMonitor" in name:
            return True
        return False
    return value


def main():
    with open(DUMP, encoding="utf-8") as f:
        text = f.read()
    names = sorted(set(TOKEN.findall(text)))
    points = {}
    for name in names:
        direction, kind, value = classify(name)
        points[name] = {"dir": direction, "kind": kind,
                        "value": default_value(name, kind, value)}
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump({"source": os.path.basename(DUMP), "points": points},
                  f, ensure_ascii=False, indent=1)
    print("io_points.json: %d points -> %s" % (len(points), OUT))
    return len(points)


if __name__ == "__main__":
    main()
