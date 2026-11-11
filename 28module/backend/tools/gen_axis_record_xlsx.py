# -*- coding: utf-8 -*-
"""gen_axis_record_xlsx.py — 按当前仿真模型重建《整机轴记录表_V2》。

数据源：backend/runtime/axes_config.py（权威）+ docs/轴号对照表_V1.md（状态/备注口径）。
输出：
  轴名对应形成和速度加速度/整机轴记录表_V2.xlsx（纯标准库手写 zip+XML，真 xlsx）
  轴名对应形成和速度加速度/整机轴记录表_V2.csv （同内容，UTF-8 带 BOM）
用法：python backend/tools/gen_axis_record_xlsx.py
改参后重跑本脚本即可再生成；脚本末尾自带读回自验（行数/列数/抽查 3 行 + CSV BOM）。
"""
import csv
import io
import os
import sys
import zipfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # 项目根
sys.path.insert(0, os.path.join(ROOT, "backend", "runtime"))
import axes_config as ac                               # noqa: E402

OUT_DIR = os.path.join(ROOT, "轴名对应形成和速度加速度")
XLSX_PATH = os.path.join(OUT_DIR, "整机轴记录表_V2.xlsx")
CSV_PATH = os.path.join(OUT_DIR, "整机轴记录表_V2.csv")

HEADERS = ["模块", "轴名", "控制器", "ACS轴号", "类型", "单位", "行程min", "行程max",
           "默认速度", "默认加速度", "加加速度(jerk)", "伺服tau(s)", "到位窗口",
           "整定时间(s)", "重复精度", "回零类型", "回零buffer", "状态", "备注"]
META1 = "整机轴记录表 V2（按当前仿真模型重建 2026-08-20）"
META2 = "数值源：backend/runtime/axes_config.py 现行配置"

# ---- 轴号对照表_V1.md 复用列（模块/状态/备注，2026-08-20 已核对） ----
MODULE = {
    "WS_LS_X": "WS", "WS_LS_Y": "WS", "WS_Rz": "WS",
    "E_EPIN_Z_1": "WS", "E_EPIN_Z_2": "WS", "E_EPIN_Z_3": "WS",
    "BS_Arm_Z": "BS", "BS_CAL_Rx": "BS", "BS_CAL_Ry": "BS",
    "AS_X": "AS", "AS_Y": "AS", "AS_Z": "AS", "AS_Rx": "AS", "AS_Ry": "AS",
    "FA_X": "CH", "FA_Z": "CH", "FA_Rx": "CH", "CH_Rz": "CH",
    "ES_X": "ES", "ES_Y": "ES", "ES_PunchZ": "ES",
}
STATUS = {
    "WS_LS_X": "待确认", "WS_LS_Y": "待确认", "WS_Rz": "待确认",
    "E_EPIN_Z_1": "待确认", "E_EPIN_Z_2": "待确认", "E_EPIN_Z_3": "待确认",
    "BS_Arm_Z": "已对齐", "BS_CAL_Rx": "已对齐", "BS_CAL_Ry": "已对齐",
    "AS_X": "待确认", "AS_Y": "待确认", "AS_Z": "待确认",
    "AS_Rx": "待确认", "AS_Ry": "待确认",
    "FA_X": "待确认", "FA_Z": "已对齐", "FA_Rx": "已对齐", "CH_Rz": "已对齐",
    "ES_X": "已对齐", "ES_Y": "已对齐", "ES_PunchZ": "待确认",
}
REMARK = {
    "WS_LS_X": "行程冲突D1：轴记录表-162~280 vs 仿真±220；双驱龙门占轴号0+1",
    "WS_LS_Y": "行程冲突D1：轴记录表-162~280 vs 仿真±220；双驱龙门占轴号2+3",
    "WS_Rz": "行程冲突D1：轴记录表±0.005rad vs 协议口径±1°；双驱占轴号4+5",
    "E_EPIN_Z_1": "行程冲突D3：轴记录表±2mm vs 0~30；顶针三轴同步leader",
    "E_EPIN_Z_2": "行程冲突D3：轴记录表±2mm vs 0~30；随E_EPIN_Z_1同步回零",
    "E_EPIN_Z_3": "行程冲突D3：轴记录表±2mm vs 0~30；随E_EPIN_Z_1同步回零",
    "BS_Arm_Z": "各资料无冲突（轴记录表≤-150mm 接近）",
    "BS_CAL_Rx": "零位传感器回零；各资料无冲突",
    "BS_CAL_Ry": "零位传感器回零；各资料无冲突",
    "AS_X": "D2命名交叉：点位表'对准模块Y轴'名为AS_X；按功能名落轴号6待真机复核",
    "AS_Y": "D2命名交叉：点位表'对准模块X轴'名为AS_Y；按功能名落轴号7待真机复核",
    "AS_Z": "前端模型原无此轴（D5），axis_alignz_0001 为仿真侧占位id",
    "AS_Rx": "点位表轴名定义未列轴号与归属（D4）",
    "AS_Ry": "点位表轴名定义未列轴号与归属（D4）",
    "FA_X": "轴号9由点位表+config4双重佐证，待上位机对账",
    "FA_Z": "各资料无冲突",
    "FA_Rx": "各资料无冲突",
    "CH_Rz": "config4 PLC侧文件名为FA_Rz.json；各资料无冲突",
    "ES_X": "外包模块高层命令（D6），vendor_es 占位实现",
    "ES_Y": "外包模块高层命令（D6），vendor_es 占位实现",
    "ES_PunchZ": "轴号10+回零buffer17由config3实证，待上位机对账",
}
ACS_AXIS_DISPLAY = {
    "WS_LS_X": "0（双驱0+1）", "WS_LS_Y": "2（双驱2+3）", "WS_Rz": "4（双驱4+5）",
    "AS_X": "6", "AS_Y": "7", "AS_Z": "8",
    "AS_Rx": "待整理（D4）", "AS_Ry": "待整理（D4）",
    "FA_X": "9", "ES_PunchZ": "10",
}
# 回零 buffer 真值（TRUTH_HOME_BUFFER 展开到轴；PLC/外包轴无 ACS buffer）
HOME_BUFFER = {
    "WS_LS_X": 18, "WS_LS_Y": 18, "WS_Rz": 18,
    "AS_X": 18, "AS_Y": 18, "AS_Z": 18,
    "FA_X": 18, "ES_PunchZ": 17,
}
HOME_TYPE_CN = {
    "gantry_dual": "gantry_dual 双驱龙门",
    "plc_sync_trio": "plc_sync_trio 顶针三轴同步",
    "zero_sensor": "zero_sensor 零位传感器",
    "vendor_es": "vendor_es 外包模块",
    "acs_limit": "acs_limit 普通软限位",
}


def fmt(v):
    """数值显示：整数去小数点，其余原样保留精度。"""
    if isinstance(v, float) and v == int(v) and abs(v) < 1e15:
        return int(v)
    return v


def build_rows():
    rows = []
    for i, (name, lo, hi, vmax, unit, ctl, home, _fid) in enumerate(ac.AXES):
        rotary = ac.axis_is_rotary(name)
        sv = ac.servo_defaults(name)
        acs_no = ACS_AXIS_DISPLAY.get(name, "—（PLC轴不过ACS）")
        if name in ("ES_X", "ES_Y"):
            acs_no = "—（外包模块高层命令，D6）"
        ht = ac.home_type(name)
        home_cn = "%d %s回零（%s）" % (home, "正" if home == 18 else "负",
                                      HOME_TYPE_CN[ht])
        if name in PRECISION_EXTRA_NOTE:
            home_cn += PRECISION_EXTRA_NOTE[name]
        # 重复精度（error_system 分级，REPEAT_3SIGMA 3σ 原生单位）
        r3 = ac.REPEAT_3SIGMA[name]
        if name in ac.PRECISION_AXES:
            rep = ("精密轴 3σ=500nm（平移误差0.2~1µm级）" if unit == "mm"
                   else "精密轴 3σ=2µrad（旋转误差1~5µrad级）")
        else:
            rep = ("辅助轴 3σ=3µm（平移误差1~5µm级）" if unit == "mm"
                   else "辅助轴 3σ=10µrad（旋转误差10~30µrad级）")
        assert r3 > 0
        rows.append([
            MODULE[name], name, ("PLC(外包)" if ht == "vendor_es" else ctl),
            acs_no, ("旋转" if rotary else "直线"), unit,
            fmt(lo), fmt(hi), fmt(vmax),
            fmt(vmax * ac.ACCEL_FACTOR), fmt(vmax * ac.JERK_FACTOR),
            sv["tau"], sv["settle_window"], sv["settle_time"],
            rep, home_cn, HOME_BUFFER.get(name, "—"),
            STATUS[name], REMARK[name],
        ])
    assert len(rows) == 21
    return rows


# 回零类型附加说明（对照表口径：vendor_es 标"占位"）
PRECISION_EXTRA_NOTE = {"ES_X": "（占位）", "ES_Y": "（占位）"}


# ---------------- 最小 xlsx 写出（纯标准库） ----------------
def col_letter(n):  # 1 -> A
    s = ""
    while n:
        n, r = divmod(n - 1, 26)
        s = chr(65 + r) + s
    return s


def sheet_xml(all_rows):
    ns = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
    parts = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
             '<worksheet xmlns="%s"><sheetData>' % ns]
    for r, row in enumerate(all_rows, 1):
        parts.append('<row r="%d">' % r)
        for c, val in enumerate(row, 1):
            ref = "%s%d" % (col_letter(c), r)
            if val is None or val == "":
                continue
            if isinstance(val, (int, float)):
                parts.append('<c r="%s"><v>%r</v></c>' % (ref, val))
            else:
                esc = (str(val).replace("&", "&amp;").replace("<", "&lt;")
                       .replace(">", "&gt;"))
                parts.append('<c r="%s" t="inlineStr"><is><t xml:space='
                             '"preserve">%s</t></is></c>' % (ref, esc))
        parts.append("</row>")
    parts.append("</sheetData></worksheet>")
    return "".join(parts)


def write_xlsx(path, all_rows):
    content_types = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                     '<Types xmlns="http://schemas.openxmlformats.org/'
                     'package/2006/content-types">'
                     '<Default Extension="rels" ContentType="application/vnd.'
                     'openxmlformats-package.relationships+xml"/>'
                     '<Default Extension="xml" ContentType="application/xml"/>'
                     '<Override PartName="/xl/workbook.xml" ContentType="appl'
                     'ication/vnd.openxmlformats-officedocument.spreadsheetml'
                     '.sheet.main+xml"/>'
                     '<Override PartName="/xl/worksheets/sheet1.xml" ContentT'
                     'ype="application/vnd.openxmlformats-officedocument.spr'
                     'eadsheetml.worksheet+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package'
            '/2006/relationships"><Relationship Id="rId1" Type="http://schem'
            'as.openxmlformats.org/officeDocument/2006/relationships/officeD'
            'ocument" Target="xl/workbook.xml"/></Relationships>')
    workbook = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                '<workbook xmlns="http://schemas.openxmlformats.org/spreadsh'
                'eetml/2006/main" xmlns:r="http://schemas.openxmlformats.org'
                '/officeDocument/2006/relationships"><sheets><sheet name="整'
                '机轴记录表V2" sheetId="1" r:id="rId1"/></sheets></workbook>')
    wb_rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
               '<Relationships xmlns="http://schemas.openxmlformats.org/pac'
               'kage/2006/relationships"><Relationship Id="rId1" Type="http:'
               '//schemas.openxmlformats.org/officeDocument/2006/relationsh'
               'ips/worksheet" Target="worksheets/sheet1.xml"/></Relationshi'
               'ps>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        z.writestr("_rels/.rels", rels)
        z.writestr("xl/workbook.xml", workbook)
        z.writestr("xl/_rels/workbook.xml.rels", wb_rels)
        z.writestr("xl/worksheets/sheet1.xml", sheet_xml(all_rows))


def write_csv(path, all_rows):
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\r\n")
    for row in all_rows:
        w.writerow(["" if v is None else v for v in row])
    with open(path, "wb") as f:
        f.write(b"\xef\xbb\xbf" + buf.getvalue().encode("utf-8"))


# ---------------- 读回自验 ----------------
def readback_xlsx(path):
    ns = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    with zipfile.ZipFile(path) as z:
        root = ET.fromstring(z.read("xl/worksheets/sheet1.xml"))
    grid = []
    for row in root.find("m:sheetData", ns).findall("m:row", ns):
        vals = {}
        for c in row.findall("m:c", ns):
            ref = c.get("r")
            col = 0
            for ch in ref:
                if ch.isalpha():
                    col = col * 26 + ord(ch) - 64
            if c.get("t") == "inlineStr":
                t = c.find("m:is/m:t", ns)
                vals[col] = t.text if t is not None and t.text else ""
            else:
                v = c.find("m:v", ns)
                vals[col] = float(v.text) if v is not None else ""
        if vals:
            grid.append([vals.get(i, "") for i in range(1, max(vals) + 1)])
    return grid


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    all_rows = [[META1], [META2], HEADERS] + build_rows()
    write_xlsx(XLSX_PATH, all_rows)
    write_csv(CSV_PATH, all_rows)

    # 自验 1：xlsx 读回 —— 行数/列数
    grid = readback_xlsx(XLSX_PATH)
    assert len(grid) == 24, "行数应为 24（2 元信息 + 表头 + 21 轴），实际 %d" % len(grid)
    assert len(grid[2]) == 19, "列数应为 19，实际 %d" % len(grid[2])
    body = grid[3:]
    assert len(body) == 21 and all(len(r) == 19 for r in body)

    # 自验 2：抽查 3 行关键数值与 axes_config.py 一致
    def cfg(name):
        return next(r for r in ac.AXES if r[0] == name)
    checks = [
        # (轴名, 表内行号0起, 行程min, 行程max, 速度, 加速度, jerk, tau, 到位窗口)
        ("WS_LS_X", 0, -220.0, 220.0, 600.0, 6000.0, 60000.0, 0.005, 0.002),
        ("FA_X", 14, -1250.0, 0.0, 600.0, 6000.0, 60000.0, 0.005, 0.002),
        ("ES_PunchZ", 20, 0.0, 5.0, 10.0, 100.0, 1000.0, 0.005, 0.002),
    ]
    for name, i, lo, hi, vmax, amax, jerk, tau, win in checks:
        row = body[i]
        assert row[1] == name, (name, row[1])
        got = (float(row[6]), float(row[7]), float(row[8]), float(row[9]),
               float(row[10]), float(row[11]), float(row[12]))
        exp = (lo, hi, vmax, amax, jerk, tau, win)
        assert got == exp, "%s 数值不符: %s vs %s" % (name, got, exp)
        c = cfg(name)
        assert c[1] == lo and c[2] == hi and c[3] == vmax

    # 自验 3：CSV BOM
    with open(CSV_PATH, "rb") as f:
        assert f.read(3) == b"\xef\xbb\xbf", "CSV 缺 UTF-8 BOM"

    print("OK: %s" % XLSX_PATH)
    print("OK: %s" % CSV_PATH)
    print("自验通过：xlsx 24 行 x 19 列；抽查 WS_LS_X/FA_X/ES_PunchZ 数值一致；CSV 带 BOM。")


if __name__ == "__main__":
    main()
