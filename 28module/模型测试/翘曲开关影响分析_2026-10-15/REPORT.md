# 翘曲开关影响分析（对准 / 光谱共焦）—— 迭代 73 验证延伸（2026-10-15）

翘曲仿真（迭代 73，`backend/docs/晶圆翘曲_V1.md`）的开关对照实测：翘曲
OFF/ON 对**对准**（EGA 全局对准 XY + 调焦调平 Z 向）与**光谱共焦**高度链
的影响定量分析。

## 方法

- 独立后端实例（9310 端口组，D2W_PORT_* 覆盖），与 9010 面板演示互不干扰；
  采集脚本 `_warp_analysis_73.py`（可重跑；收尾自动还原处方/翘曲参数/工件/轴位）。
- 注入：显式翘曲真值 **bow=80µm 纯穹顶片**（saddle=0，便于抛物线独立回收）；
  `jitter_pct=0` 免抽取抖动。
- 传感器读数噪声置 0（confocal_sigma_um=0、fast_noise_px 三相机 0）→
  OFF/ON 差异即纯翘曲效应；EGA 残余运行间差来自轴重复性采样等物理噪声
  （实测噪声带 ~0.1µm/标记），与翘曲无关。
- modal/thermal 全程 OFF（防交叉耦合）。

## 结果 1：光谱共焦直径扫描（相位 A，warpage_confocal_scan_*.csv +  parity PNG）

WS 载物晶圆相对下共焦探头步进 −100~+100mm（步 20mm，11 点 ×OFF/ON 两趟）：

| 指标 | 实测 | 判读 |
|---|---|---|
| OFF 基线平直度 | **0.000 µm**（11 点逐位一致） | 翘曲 OFF 零影响的直接证据 |
| ON−OFF 抛物线拟合 bow 回收 | **80.00035 µm（回收率 100.0004%）** | 独立测量口径（非 state 同源断言），场方程到达共焦链路精确 |
| 拟合残差 | max 0.00036 µm（0.36nm） | 纯抛物线，无畸变项 |
| 顶点位置 | ws_x=−42.397mm | 与 EGA 锚点 ws_ega_x=−42.394mm 自洽（Δ≈3µm，枢轴/偏旋转角量级） |

## 结果 2：对准（EGA 全局对准，XY）——免疫验证（相位 B，warpage_ega_*.csv）

OFF×2 / ON×2 各跑完整 wafer_ega（with_load，9 标记拟合）：

| 参数 | OFF 均值 | ON 均值 | Δ(ON−OFF) | 运行间噪声带（OFF#1↔#2） |
|---|---|---|---|---|
| tx | 2.48728 µm | 2.49121 µm | **+3.9 nm** | ±60 nm（标记） |
| ty | 0.81103 µm | 0.81521 µm | +4.2 nm | ±23 nm |
| theta | −1.117 µrad | −0.951 µrad | +0.17 µrad | ±1.7 µrad |
| resid_3s | 2.306 nm | 2.279 nm | −0.03 nm | ±0.02 nm |

全部 ON−OFF 差**埋在运行间噪声带内**（theta 差仅为带的 1/10）→
**XY 对准对翘曲免疫**，与设计文档 §3 一致（远心投影 z 不进 (x,y)；翘曲
是 Z 向现象，XY 真值有意不消费——坡度×杠杆的间接 XY 效应 ~nm 级，
见 §11 量级论证）。

## 结果 3：Z 向对准（调焦调平，共焦消费链）（相位 C，warpage_fl_*.csv）

flip_pick→flip_handoff→wafer_ega→focus_level(global) 全链对照：

| | OFF | ON |
|---|---|---|
| 流程结局 | **done**（verify pass） | **aborted**（verify 判废） |
| verify 残余 | [+0.00000006, −0.00000049] µrad ≈ 0 | **[−64.59, +5.55] µrad 超容差** |
| 测得晶圆面坡度 wafer_grad | ≈0（~1e-9 µrad） | **[−5.55, +64.59] µrad** |

ON 时调焦调平**真实测到**翘曲场在测点处的局部坡度（|g|=64.8µrad ↔
bow=80µm 穹顶理论梯度对应测点质心半径 r≈9.1mm，量级自洽），拟平面吸不
走曲率 → 复测残余超容差判废、流程中止。**这是物理正确行为**：与真机
"翘曲片调平验证失败/需分区补偿"的故障模式一致，证明翘曲已贯通
共焦→调焦调平链。

## 结果 4：视觉离焦剂量响应（相位 D，warpage_blur_*.csv）

同一视野点换片 bow∈{0,30,80}µm，下相机晶圆标记 FAST 识别：

| bow (µm) | blur OFF | blur ON | 探头处 dz ON (µm) |
|---|---|---|---|
| 0 | 0.072 | 0.072 | 0.0 |
| 30 | 0.072 | 0.388 | 8.64 |
| 80 | 0.072 | 1.154 | 23.05 |

blur 随 bow 单调上升、与 dz 同比例（Δblur/dz≈0.04~0.05 ≈ 1/DOF(20µm)）——
翘曲经标记 z 进入 apply_focus 离焦链（FAST blur_ratio 与 REAL 模糊核共用）。
注：重离焦（blur>3 判废）下匹配器可能改选标记，剂量响应按单标记口径解读。

## 结果 5：键合接触位注入量（相位 E，warpage_bond_dz_*.csv）

bond_dz_um：OFF=0.0 → ON=+25.1µm（键合点局部面高注入 eff_contact，符号同
迭代 56 热胀 dz_c）；与 i73.warpage_contact_shift 回归的力实测（停
base_eff+0.02mm，ON/OFF 力差 27.2N）互证。

## 结论

1. **OFF 零影响**：共焦 11 点逐位平直（0.000µm），EGA/FL/视觉基线不变。
2. **ON 贯通符合设计**：共焦面高精确按场方程到达（bow 回收 100.0004%）、
   视觉离焦按 dz/DOF 升起、调焦调平测到真实局部坡度并（大翘曲下）判废、
   键合接触位按局部 dz 贯通。
3. **XY 对准免疫**（设计验证）：EGA 全部参数 ON−OFF 差 ≪ 物理噪声带。

## 文件清单

- `_warp_analysis_73.py`——采集脚本（连 9310；可重跑，自动还原）
- `warpage_confocal_scan_*.csv`——共焦扫描原始数据（22 点×2 趟）
- `warpage_scan_parity_*.png`——抛物线拟合对照图
- `warpage_ega_*.csv`——EGA 四次全量结果（含 9 标记逐点测量）
- `warpage_fl_*.csv`——调焦调平 OFF/ON 结果（fl_result/fl_verify 全文）
- `warpage_blur_*.csv` / `warpage_bond_dz_*.csv`——离焦剂量/接触注入
- `warpage_analysis_*.json`——汇总（含 scan_fit 数字）
- `run_console.log`——运行控制台留档（GBK 控制台中文显示乱码为显示层
  问题，数据文件全部 UTF-8 干净）
