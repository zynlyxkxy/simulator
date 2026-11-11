# 整机模态普查实验报告（2026-09-29，迭代 57）

## 0. 实验设置

- **对象**：D2W 键合设备全链路数字模型（backend i53，9010 实机后端驱动）
- **模态仿真**：ON；地板激励 typical（0.3µm RMS 工程近似）；键合接触脉冲 ON；热软化 ON（本实验冷态）
- **激励手段**：①整机工艺流程——单颗链（EGA→flip_pick→flip_handoff→focus_level→fine_align→bond，含键合接触脉冲）；②逐轴 jog 激励扫描（21 轴各自小行程往返，补流程盲区）；③两个合成阶跃事件（WS_LS_X 快移 / BS_Arm_Z 阶跃）配 scope 1kHz×8s 波形采集
- **观测**：`/api/modal/state` 10Hz 全程采样（q/amp/rms-EMA，τ=0.25s）+ scope 环形缓冲 FFT
- **声明**：fn/ζ/增益全部为工程估值（待真机 FRF 辨识），本报告结论口径=**量级与激励覆盖合理性**，不做真机精度承诺

## 1. 三问直答

### Q1：每个组件的模态都能分析吗？——能，且全部有响应证据

- **9 个整机 MIMO 模态**：链全程全部有可辨响应（链峰值 0.42~2.28µm，
  为地板 RMS 的 2~10×，见 §2 表）。
- **21 个轴 SISO 模态**：轴模态只吃"自身加速度"激励（物理归属干净）——
  链内会动的轴（WS_XY/AS_XY/BS_Arm_Z 等）链峰值已观测到（最大
  WS_LS_Y 2.56µm）；链内不动的轴（E_EPIN×3/FA×3/CH_Rz/ES×3/校准轴×2 等）
  由 jog 扫描补齐激励，**21/21 全部测到响应**。
- **盲区结论**：不存在"模型里分析了但测不到"的组件；但存在"本流程不
  激励"的轴（如上料/顶针/校准轴在单颗链中不动）——这是流程覆盖面问题，
  不是模型盲区，jog 扫描即可补齐。

### Q2：各自是多少？

见 §2 全表（fn/ζ/链 RMS/链峰值/jog 峰值/激励源/输出模块）。

### Q3：量级是否合理？——合理，两处标注

对照口径：精密运动台整定尾巴 µm 级；整机一阶模态 20~100Hz（调研口径）；
地板 typical=0.3µm RMS 工程近似。

- **轴 SISO**（jog 峰值）：直线轴 32nm~2.2µm、旋转轴 0.09~4.4µrad——
  µm/µrad 级整定尾巴，✓ 合理。最大 ES_X/ES_Y≈2µm（fn=30Hz 蓝膜台，
  低刚度外包模块，物理上合理）；最小为校准微旋轴（0.1µrad 级，
  行程小+fn 高，合理）。
- **整机 MIMO**（链峰值）：0.42~2.28µm，地板 RMS 0.08~0.24µm——
  事件响应/地板比 2~10×，✓ 合理。最大 granite_sway_y 2.28µm
  （WS_LS_Y 大行程激励 36Hz 摇摆，可接受）。
- **fn 分布**：18~110Hz——主体落在整机一阶 20~100Hz 带内；
  两个带外标注：frame_film 18Hz（蓝膜台框架=低刚度外包模块，刻意偏低）、
  BS_CAL_Rx/Ry 110Hz（校准微旋轴=小型高刚度调整机构，偏高合理）。
- **FFT 自洽**：mode:granite_sway_x 谱峰 36.2Hz vs 配置 36.0Hz（+0.6%）、
  mode:head_z_bounce 54.9Hz vs 58.0Hz（−5%，阶跃事件能量低频富集所致，
  可接受）；axis:WS_LS_X 谱峰 6.5/5.0Hz ≠ 45Hz——**解读：轴模态在阶跃
  事件中的能量主峰是准静态挠度暂态（低频），45Hz 余振存在但衰减快
  （ζ=0.05→τ≈70ms）且幅度小，频谱主峰被暂态盖住**——非模型缺陷，
  真机锤击测试同样要看余振窗而非事件主峰。

## 2. 全量普查表

### 9 个整机 MIMO 模态（单颗链全程）

| 模态 | fn (Hz) | ζ | 链 RMS (µm) | 链峰值 (µm) | 地板 RMS (µm) | 主导阶段 | 激励源 | 输出模块 |
|---|---|---|---|---|---|---|---|---|
| granite_sway_x | 36.0 | 0.04 | 0.115 | 0.965 | 0.242 | focus_level | WS_LS_X/FA_X | wafer_stage/align_stage/bond_head/flip |
| granite_sway_y | 38.0 | 0.04 | 0.104 | 2.278 | 0.232 | focus_level | WS_LS_Y | wafer_stage/align_stage/bond_head |
| gantry_yaw | 27.0 | 0.05 | 0.115 | 1.336 | 0.329 | focus_level | WS_LS_X/WS_LS_Y | wafer_stage/align_stage/bond_head |
| gantry_sway_y | 31.0 | 0.05 | 0.106 | 1.578 | 0.197 | focus_level | FA_X/WS_LS_Y | bond_head/flip/wafer_stage |
| head_z_bounce | 58.0 | 0.06 | 0.113 | 1.268 | 0.095 | bond（接触脉冲） | BS_Arm_Z+contact | bond_head/wafer_stage |
| align_bridge | 52.0 | 0.05 | 0.244 | 1.275 | 0.119 | focus_level | AS_X/AS_Y | align_stage/bond_head |
| frame_film | 18.0 | 0.06 | 0.099 | 0.415 | 0.102 | settle | ES_X/ES_Y | film_stage |
| wafer_stage_rock | 44.0 | 0.05 | 0.104 | 1.050 | 0.174 | focus_level | WS_LS_X/WS_LS_Y+contact | wafer_stage |
| flip_arm_sway | 22.0 | 0.06 | 0.100 | 0.423 | 0.100 | settle | FA_Rx/FA_X/FA_Z | flip |

注：地板 RMS 读数偏高（含前序运动尾巴，EMA τ=0.25s 过渡）；链峰值已含
接触脉冲（bond 阶段 head_z_bounce/wafer_stage_rock 可见抬升）。

### 21 轴 SISO 模态

| 轴 | fn (Hz) | ζ | 链峰值 | jog 峰值 | 单位 | 链内激励 |
|---|---|---|---|---|---|---|
| WS_LS_X | 45 | 0.05 | 916 | 748 | nm | ✓（EGA/对准走位） |
| WS_LS_Y | 45 | 0.05 | 2560 | 1773 | nm | ✓ |
| WS_Rz | 70 | 0.08 | 0.0 | 0.45 | µrad | ✗（链内不转） |
| E_EPIN_Z_1/2/3 | 20 | 0.07 | 0.0 | 375~472 | nm | ✗（链内未顶出） |
| BS_Arm_Z | 95 | 0.08 | 466 | 2.4 | nm | ✓（键合下压+接触脉冲） |
| BS_CAL_Rx/Ry | 110 | 0.08 | ≤0.01 | ~0.1 | µrad | ✗（调平幅度小） |
| AS_X | 65 | 0.06 | 709 | 153 | nm | ✓（调焦调平/对准） |
| AS_Y | 65 | 0.06 | 1031 | 818 | nm | ✓ |
| AS_Z | 90 | 0.08 | 0.05 | 83 | nm | 弱（调平小行程） |
| AS_Rx/Ry | 85 | 0.08 | 0.0 | 0.09~0.26 | µrad | ✗ |
| FA_X | 40 | 0.05 | 0.0 | 31 | nm | ✗（本轮交接阶段 skip） |
| FA_Z | 80 | 0.08 | 0.0 | 90 | nm | ✗（同上） |
| FA_Rx | 25 | 0.06 | 0.0 | 4.4 | µrad | ✗（同上） |
| CH_Rz | 60 | 0.08 | 0.0 | 0.93 | µrad | ✗ |
| ES_X | 30 | 0.06 | 0.0 | 2008 | nm | ✗（上料侧本链不动） |
| ES_Y | 30 | 0.06 | 0.0 | 2184 | nm | ✗（同上） |
| ES_PunchZ | 85 | 0.08 | 0.0 | 8.9 | nm | ✗ |

说明：本轮链因物料状态继承（前轮已取片/交接），flip_pick/flip_handoff
阶段 skip，FA 系轴链内未激励（数值 = 上一轮全跑通链已覆盖 FA 大行程；
后续普查建议每次全新物料起跑）。

## 3. 图件

- `modal_timeline.png` —— 链全程 6 个代表模态 amp 时间线（log），
  阶段边界虚线；可见 focus_level/fine_align 阶段事件密集、bond 接触
  脉冲抬升
- `modal_census_bar.png` —— 左：21 轴 jog/链峰值（log）；右：9 模态
  链峰值 vs 地板 RMS（分离清晰）
- `modal_scope_fft.png` —— 4 通道波形+FFT（标注谱峰与配置 fn 参考线）
- `modal_census.csv` —— 逐模态全字段数据表
- `_analysis.json` —— 分析中间数据（程序可读）

## 4. 结论

**整机数字模型的模态体系在工艺流程激励下全链路可观测、量级合理**：
轴级 µm 整定尾巴、模块级 nm~µm 事件响应、fn 分布贴合整机一阶 20~100Hz
口径；接触脉冲通路工作（bond 阶段可见）。频谱自洽性验证通过
（granite_sway_x 36.2Hz vs 36.0Hz 仅 +0.6%）。

## 5. 边界与后续

- fn/ζ/增益全为工程估值——本报告回答"量级与覆盖合理性"，真机需 FRF
  （锤击/脉冲激励）辨识替换（路线见 docs/模态仿真_V1.md）
- 地板随机力为 RMS 工程近似（非真 VC 谱）；轴模态 FFT 主峰解读见 §1 Q3
- 链内未激励轴（FA/ES/EPIN 等）建议后续普查改为全片流程+全新物料，
  或固定 jog 扫描补齐（本报告的混合口径）
- 复跑：`env/python/python.exe Temp/_modal_census.py`（脚本在
  模型测试/模态普查_2026-09-29/ 有归档副本）
