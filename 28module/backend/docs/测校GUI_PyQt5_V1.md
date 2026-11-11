# 测校GUI_PyQt5_V1 —— PyQt5 测校软件（复刻 HoriMetrologyScirptsFramework 标准）

> 2026-09-09 迭代 26 新增。独立进程、独立解释器，与 panel 测校工作台（HTTP 契约）并存。
> 2026-09-20 迭代 42/49 更新：拍照链直给化（删 warm-up / 15s 超时直给 /
> 接收线程断线自动重连，§5.3）+ 新页登记（迭代 42 两项、迭代 49 五项），
> 测校项总数 18 项（总表 §5.2，与 `main.py:82-101` SCRIPTS 注册表一致）。

## 1. 这是什么

`backend/metrology_gui/` 是一套 PyQt5 测校软件，架构与通讯**逐层复刻**参考框架
`C:\Users\e-hao.wu\Desktop\3000测校SOP_u\HoriMetrologyScirptsFramework`（另一台设备的
真机测校框架）——目的是真机交接时 GUI 代码零改动，只改 `metrology_config.py` 的
broker 地址与轴/相机表。

启动：双击项目根 `launch_metrology_gui.bat`（GBK+CRLF；锁定项目内置 env/python
解释器，先探测 9010 后端并提示）。也可直接双击 `metrology_gui/main.py`——导入
PyQt5 失败会自动重定到 env/python 解释器（`D2W_MET_GUI_REEXEC` 哨兵防循环）。

## 2. 运行环境

- 解释器：项目内置 `env/python/python.exe`（Python 3.11.0，PyQt5 5.15.11/Qt 5.15.2、
  matplotlib 3.7.2、pandas 2.0.3、cv2 4.11.0；迭代 41 起全部环境指向收口于此，
  bat 启动设 PYTHONNOUSERSITE=1；旧 `3000测校SOP_u` 下 sop_tools/_tools venv
  仅留作历史容错兜底）。
- **无 paho-mqtt**：`cmdjson/PahoMqttClient.py` 是同名同 API 薄封装，底层复用
  `runtime/mqtt_broker.py` 的标准库客户端（MqttSub/mqtt_publish），全离线零安装。
- 中文路径 → Qt 插件目录解析为空会弹 "no Qt platform plugin"：迭代 30 起由
  `_qt_bootstrap.py` 统一处理——`find_venv_python()`（迭代 41 起 env/python 居首，
  sop_tools→_tools→`3000测校SOP_u/*/.venv` glob 仅历史容错兜底）+ `setup_qt_env()`（
  QApplication 创建前强制 QT_QPA_PLATFORM_PLUGIN_PATH）；main.py、`launch_metrology_gui.bat`
  （env/python 直锁+缺失中文提示）、`ws_gateway.py` launch 端点同一解析顺序。
  offscreen 截图测试另需 `QT_QPA_FONTDIR=C:\Windows\Fonts`。

## 3. 架构分层（对照参考框架）

| 层 | 本软件 | 参考对应 | 说明 |
| --- | --- | --- | --- |
| 主窗口 | `main.py`（QListWidget+QStackedWidget 自建列表） | C++ 宿主（无源码） | 参考的列表在宿主侧，我们自建 |
| 基类 | `script_widget.py` ScriptWidget | ScriptWidget.py | 抽象 run/interrupt/param 序列化；worker 线程+信号；**去掉 MQTT 状态回传宿主**；UI 更新一律走信号（修参考 worker 直碰 UI 的不严谨写法） |
| 控件工厂/QSS | `qt_custom_ui.py` | qt_custom_ui.py | 蓝标题条 #4a90e2→#357abd 等逐条取自参考 |
| 命令拼装 | `cmdjson/MC_Motion_cmd.py`、`Camera_CMD.py` | 同名 | 信封 `{task, context:json.dumps(payload), result}` 逐字段复刻 |
| MQTT 客户端 | `cmdjson/PahoMqttClient.py`（标准库实现） | 同名（paho） | publish/subscribe/收消息队列同 API |
| 收包状态机 | `mqtt_receive_worker.py` | MqttReceiveWorker.py | 解析 CTL_DR_ACS_Motion/VSS_Com，wait_stage_stop/wait_image_process 阻塞原语；裁掉 PLC/干涉仪/FLS/base64 图像分支；parse_data 加容错（坏帧不杀线程） |
| 动作层 | `device_actuator.py` | device_actuator.py | move_stage/take_photo = publish+wait 组合 |
| 配置 | `metrology_config.py` | metrology_config.py | broker 127.0.0.1:1883、DATA_SAVE_ROOT=runtime/metrology_data/、相机表（nm_per_px 初值 600/600/9133，refresh_nm_per_px 可从 9010 刷新）、camera_stage_map |
| 测校项（MQTT 原语） | `SingleCameraAlignStability.py(+_ui.py)`、`AlignStabilityDynamic.py(+_ui.py)` | 同名 | 逻辑/UI 分离，参数区/结果区/Operation Log 三段式 |
| 测校项（作业 API，迭代 30 重做；迭代 31/32/36/42/49 扩展至 16 项，总表 §5.2） | `ConfocalRepeat.py(+_ui.py)`、`StageMoveRepeat.py(+_ui.py)`、`DieTransferRepeat.py(+_ui.py)`、`WsRotateRepeat.py(+_ui.py)`（迭代 30 四项） | 同参考封装标准 | 每项独立参数区/matplotlib 结果区/统计表/日志；公共机械层（jobs 占用查询→start→status 轮询→result→cancel）收在 `job_base.py`（类比参考共享 qt_custom_ui）；执行走后端作业 API（工艺流/多相机现场管理不适合 GUI 端 MQTT 原语重组），与相机两项的 MQTT 原语路径并存 |
| Qt 引导 | `_qt_bootstrap.py`（迭代 30） | 无 | GUI 解释器解析唯一事实源（迭代 41 起 env/python→sop_tools→_tools→glob 兜底）+ setup_qt_env 强制插件路径；main.py/bat/ws_gateway launch 端点同一解析顺序 |

## 4. 线协议（与参考兼容）

- 命令 topic：`GUI_DR_ACS_Motion`（运动 CMD_Motion_MoveAbs/MoveRel）、`VSC_Com`（视觉
  VisionCmdJson：camID + content["1"].lstShapeName）。
- 反馈 topic：`CTL_DR_ACS_Motion`（CMD_Motion_RunTimeInfo，10Hz，dic_Stage→
  inPos/moving/lst_Axis[i].coord.value）、`VSS_Com`（content["1"].lstRow/lstCol/
  lstAngle/lstScore/judge）。
- 轴名：直接用仿真轴名（WS_LS_X/Y、AS_X/Y、FA_X），后端适配器恒等透传+别名
  （`runtime/mqtt_cmd_map.json`：WaferStage_X→WS_LS_X 等）。相机 camID=up/down/coarse
  （也兼容参考 int 编码 0=down/1=up/2=coarse——**该 int 映射是仿真假设，
  与真机 camID 约定待同事确认**，2026-09-10 迭代 31 标注；camID 未知时
  适配器回 judge=false+空 lst+error 的 VSS_Com 错误帧，不再只 print 静默）。
- 仿真侧实现：`runtime/mqtt_cmd_adapter.py`（随 main.py 默认拉起，
  `D2W_MQTT_CMD_ADAPTER=0` 可关）。刻意偏差：VSS_Com 不回传图像（image 恒空）；
  stage 分组 1001+ 号段为仿真自定义（对参考解析透明）；错误回包
  `{task 原样, result:false, context:{error}}` 是自定义约定。

## 5. 测校项语义

与 panel 工作台同一套后端物理（REAL 视觉链路），但走 MQTT 命令链时序：
订阅反馈 → 循环{（动态：move 起始位→wait_stage_stop→等待）→ move 拍照位→等待
→ take_photo} → pandas 写 CSV（沿用参考命名 single_camera_stability_/
align_stability_dynamic_ + MMDD_HH_MM_SS）→ matplotlib 双子图（标题 3σ nm，
np.std ddof=0 × 3 × nm_per_px）。

**模板名必填**（迭代 34 起扩展到全部作业 API 测校项）：空模板名 GUI 侧
即拒（`get_saved_param` 抛"视觉模板名称必填"），须是视觉页已注册模板
（相机两项输入框缺省占位 `cv_cross_0824`，`SingleCameraAlignStability_
ui.py:66-67`）；后端 start 再校验一次（空/不存在 → 400 列可用名）。
作业不自建临时模板、绝不删用户模板（后端 focus_scan/pixel_equivalent/
rotation_direction 三旧标定项例外，无模板参数、不在本软件 18 项内）。

**spinbox 超精度行为**（迭代 34，`qt_custom_ui.py:39-96`）：
`MetrologyDoubleSpinBox.validate` 对小数位超 `decimals` 的输入判
Intermediate 逐键放行（不再丢键），提交（失焦/回车）时 `fixup` 经
`textFromValue` **四舍五入到设定小数位**——根治旧行为"整条弹到
range 下限"（如 -250mm/0.0001rad）；越界/低于下限仍夹取到边界。

### 5.1 作业 API 测校项（迭代 30 按参考标准重做，取代 GLM 的 job_routines 通用壳；迭代 31/32/36/42/49 扩展至 16 项，总表 §5.2）

- **光谱共焦重复性**（`confocal_repeat`，cam=up/down）：repeat/wait_ms；µm 趋势曲线
  （标题 3σ µm）+统计表。
- **运动台移动重复性**（晶圆台=`ws_move_repeat`+down / 对准台=`as_move_repeat`+
  up 或 down）：起始位/拍照位 x,y + repeat/wait + **模板名必填**（迭代 34，
  空 GUI 即拒）；**相机下拉**（迭代 34：晶圆台恒锁下相机——晶圆标记只有
  下相机可见；对准台可选 上相机/下相机 缺省 up，选下相机时自动附
  `jog_group="align"` 使走位轴=AS_X/Y，`StageMoveRepeat.py:52-65,112-125`）；
  row/col 双曲线（标题 3σ nm）+轴到位统计。
- **芯片交接重复性**（`die_transfer_repeat`）：repeat/wait + **模板两字段
  （迭代 64 补）**：粗对准相机模板（coarse_tpl，flip/back 段 die 背面特征）+
  上相机模板（up_tpl，head 段键合头上 die 角标），启动时从机器常数
  `vision_marks.tpl_coarse_die/tpl_die_head` 预填当前默认，留空=后端走同一
  机器常数缺省（非空才下发）；back_tpl 不暴露（后端缺省=coarse_tpl）。
  **迭代 30 起循环内
  不放回蓝膜**：首取一颗后 die 在 翻转↔键合头 往复（flip_handoff→上相机 head 段
  →head_to_flip→coarse back 段），仅作业结束 finally 归位一次；结果=flip/head/back
  分段世界坐标趋势 + 分段 3σ(µm) 柱状对比 + 统计表。**迭代 34 起 back 段用
  固定 `back_tpl`（缺省=coarse_tpl；存在性
  start 校验 400）**，回翻后先走 FA_Z 回粗对准调焦位再拍；全程不自建临时模板。
- **硅片台旋转重复性**（`ws_rotate_repeat` 示教模式，**唯一模式——迭代 34 起
  auto 全自动路径已删**，缺示教参数后端 start 400）：**多点 2~10**（迭代 31 起）——
  先选点数再逐点填 模板名（视觉库已注册，**必填**）+示教位（对准台 x/y+晶圆台 x/y，
  "取当前位"按钮回填，晶圆台尽量不动）；旋转角输入 **rot_rad（rad）**。
  **直接测角法**（迭代 32 起）：首循环逐点基线测世界坐标 → 每循环 WS_Rz
  走 rz0+rot_rad → 逐点预测走位（绕认知枢轴=吸盘心+rz_center_offset_mm
  标定值，Δ=(I−R)(P−c)）旋转态实测 → 跨点 Kabsch 拟合实际旋转角 θ_i →
  角度误差 e_i=θ_i−rot_rad（**stats.angle_err mean_urad=系统角偏差、
  s3_urad=旋转角重复性**）+ 逐点拟合残余(nm)。旧四键
  （tpl_before/after+pose_before/after）兼容映射 2 点；旧字段
  stats.dtheta/series.dtheta_urad 兼容保留。
- **硅片台旋转中心标定**（`ws_rotate_center`，迭代 32 新增）：同一示教几何
  + 角度档扫描（±step_rad×1..n_angles 对称），已知指令角直接线性最小二乘
  (I−R(θ_j))c=Q−R(θ)P 堆叠解中心 c；measured.rz_center_offset_mm（拟合
  中心−WS_Rz 机械轴心）+ 残差 RMS/3σ + 逐 repeat 中心趋势；
  **"写回 rz_center_offset_mm"按钮**（POST apply fields=...，写 fine_align 组）。
- **硅片台旋转角度增益校准**（`ws_rotate_gain`，迭代 32A 新增）：同一几何，
  逐角度档 Kabsch 测实际角 → 过原点 LS gain=Σθaθc/Σθc²（截距=固定角偏差
  诊断）；measured.rz_gain + stats.gain/offset_urad；
  **"写回 rz_gain"按钮**（写 fine_align 组）。
- **相机微小旋转角标定**（`camera_microrot`，迭代 31 新增，参考真机脚本
  AlignCameraRzCalibration/CameraTiltAngleCalibration 两点法）：三相机全支持
  （走位轴迭代 32 起 down→AS_X/Y 动相机=等效动标记、up→AS_X/Y、coarse→FA_X
  单轴；**迭代 34 起 down 相机"运动台"下拉可选 对准台 align（默认）/晶圆台
  wafer**——wafer 走下相机固定看晶圆标记动 rel=+1，仅 down 相机时使能，
  `CameraMicrorotCalib.py:75-91`）；**模板名必填**（迭代 34，空 GUI 即拒）；
  **拍照位1/2 四输入框常使能必填**（迭代 34 删"指定拍照位"勾选框与缺省拍照
  位路径，photo_pos1/2 总是下发，各配"取当前位"按钮按 相机+运动台 映射
  运动组回填）；每轮纯 X 两位各拍 → φ=−atan2(Δrow,Δcol)，repeat（默认 10）
  均值输出；纯 X 容差 0.01mm（y 分量相等）；**间距预校验已删**（迭代 34——
  原 coarse≥10mm、up/down 0.3~1.4mm 硬校验与模板可达窗口预估移除，杠杆
  可用性由用户模板与作业期识别承担）；结果 measured.rot_mrad
  （mean/std/3σ µrad+逐轮 φ 序列），**"写回 rot_mrad"按钮**（POST apply
  fields=["rot_mrad"]，落盘+热生效）。
- **对准台/硅片台非正交测校**（`as_stage_nonortho`/`ws_stage_nonortho`，
  迭代 36 新增两页 `AsStageNonortho.py(+_ui.py)`/`WsStageNonortho.py(+_ui.py)`，
  cam 恒 down）：栅格跳测法（口径经迭代 37/38/39/42/45 多轮修订——现行=
  全 2×2 实测像矩阵 + 比例误差 εx/εy 分离直测，弦模型横向给角/沿轨给
  比例）；GUI 结果区 2×2 矩阵 + εx/εy ppm 行；写回 as/ws_nonortho_urad
  →stage_geo 组。技术文档按迭代 45 口径另写（开发交接 §6 暂缓清单），
  本条仅登记页面。
- **上吸盘位置标定**（`as_head_center`，迭代 42 新增，
  `AsHeadCenterCalib.py(+_ui.py)`，cam 恒 up）：测"上相机认键合头吸盘
  （collet）中心时的对准台轴位"。前置走位（AS_X/Y→现锚点、AS_Z→
  as_z_chuck_focus、BS_Arm_Z→head_z_upfocus——两焦面常数由 best_focus
  生产，两测校闭环）后 N 次 ±1mm 动态再逼近（单侧逼近消反向间隙）
  identify(up, tpl_head_chuck)→solve_pixel 反算（含 dir/swap/rot_mrad
  认知链）世界偏移 δ→anchor=当前 AS 位+δ（相机随台动，**写回后复测
  残差≈0 符号自证**）；模板名留空=后端缺省机器常数
  vision_marks.tpl_head_chuck；结果 measured.as_anchor_xy_mm /
  as_head_center_xy_mm=[x,y]（**双键同值联动**——上/下相机同轴安装同一
  物理量）+ 逐 repeat 锚点散点（µm，相对均值）+ mean/3σ（nm）统计表；
  **"写回锚点"按钮**（fields 双键，组路由 wafer_ega/focus_level）。
- **相机最佳焦平面标定**（`best_focus`，迭代 42 新增，
  `BestFocusCalib.py(+_ui.py)`，cam=up/down）：参考真机 FW OptimalFocalCal
  区间收缩法（每轮峰点邻域收缩+步长减半，末轮三点抛物线拟合亚步距 z*），
  清晰度=匹配 score 代理（VSS_Com 不回图；离焦 blur_ratio=|focus_err|/dof
  双模式单调可测，真机 score 同得）；相机下拉 up/down——down 扫 AS_Z→
  写 as_z_chuck_focus_mm、up 预置锚位后扫 BS_Arm_Z→写 head_z_upfocus_mm；
  z_start/z_end/step_mm 留空=后端缺省（焦面常数 ±0.3mm/步 0.02）；
  结果 z-score 曲线（各轮不同配色 + 峰值竖线 + best_z 竖线）+ best_z
  大数字 + 统计表；**"写回焦面"按钮**（按相机 fields=
  ["as_z_chuck_focus_mm"]/["head_z_upfocus_mm"]→focus_level 组）。
- **键合头Rx/Ry旋转轴标定**（`bs_rxry_axis`，迭代 49 新增，
  `BsRxryAxisCalib.py(+_ui.py)`，cam 恒 up）：BS_Rx/BS_Ry 各三相
  （−a/0/+a，a=angle_mrad 默认 1.5，差分有效 Θ=2a）× die 4 角标逐点
  AS 走位 → identify → 共焦补偿位读 up 探头得 Z（每点 repeat 默认 3
  取均值）→ 纯 numpy Kabsch/SVD（无 scipy）拟合旋转轴：轴方向（符号
  归一：Rx 的 x 分量为正）+ 轴上点（lstsq(I−R,t) 最小范数解——**沿轴
  分量不可观测**）+ RMSE；方向残差 ∝1/Θ 放大（mrad 级，repeat 压 √R）。
  结果 Rx/Ry 双 subplot（4 标记 neg→pos 位移矢量 quiver 图，XY 投影
  mm）+ 统计表（measured vs current）+ **"写回旋转轴"按钮**（fields=
  rx/ry_axis_point_mm+rx/ry_axis_dir 四键→bs_rxry_axis 组）。
- **Flip与键合头承接高度标定**（`flip_handoff_height`，迭代 49 新增，
  `FlipHandoffHeightCalib.py(+_ui.py)`，cam 恒 up）：**前提 Flip 上已有
  芯片**（页顶橙底醒目提示；无 die 后端 start 400）；键合头从
  (head_z_touch−2mm) 以 search_speed（默认 0.2mm/s）慢降，20ms 采
  oActualForce+双真空压力，力≥force_threshold_n（默认 2N）判接触停轴
  →Z*；measured.head_z_touch_mm=Z*（候选写回 flip_handoff 组）、
  flip_surface_mm=Z*−t_chip（chip_thickness_mm 默认 0.5）；结果
  **力-位移曲线**（x=z_mm，y=force_n，标注接触点）+ 统计表
  （measured vs current）+ **"写回承接高度"按钮**。
- **键合头键合高度标定**（`bond_height_calib`，迭代 49 新增，
  `BondHeightCalib.py(+_ui.py)`，cam 恒 up）：**前提晶圆已上片、键合头
  已拾取芯片**（页顶提示；缺则 400"请先放入芯片和晶圆"）；同法从
  bond.head_z_search_mm 慢降晶圆面判接触→Z*；measured.
  head_z_contact_mm=**Z*−t_chip−t_wafer**（裸值，候选写回 bond 组；
  wafer_thickness_mm 默认 1.0）；力-位移曲线 + 接触点标注 + 统计表
  （裸接触位 measured vs current / 实测接触位 z_contact_mm）+
  **"写回键合高度"按钮**。
- **键合温度校准**（`bond_temp_calib`，迭代 49 新增，
  `BondTempCalib.py(+_ui.py)`，cam 恒 up）：heater 下拉 up=键合头脉冲
  加热 / wafer=硅片台加热；setpoints_c 2~8 点（默认 up "100,200,300" /
  wafer "50,80,110"）逐点控温（settle |PV−SP|<1℃ 稳 2s，超时 600s 可
  中断），每点 samples×sample_interval_ms 采**发布读数**序列求
  mean/3σ（重复性）+ 最小二乘拟合 mean vs setpoint → 增益/偏置；传感链
  模型 raw=真值×(1+g_err)+b_err+noise、发布=(raw−b_cal)/(1+g_cal)，
  写回=**绝对新常数口径**（gain_new=a·gain_cur、offset_new=b·gain_cur+
  offset_cur，apply 直写 sensor_calib 组后发布读数精确回真值；PID 控制
  仍用真值 PV 不动——误差只进发布读数链，刻意口径）；结果 **mean_c vs
  setpoint_c 散点+拟合直线**（标注 gain/offset）+ 每 setpoint 一行
  mean/3σ 统计表 + **"写回温度校准"按钮**（按 heater 路由
  head_temp_*/wafer_temp_* 四键）；测温升降温耗时较长（硅片台约
  0.73℃/s），可随时中断。
- **键合力校准**（`bond_force_calib`，迭代 49 新增，
  `BondForceCalib.py(+_ui.py)`，cam 恒 up）：**前提同键合高度**（页顶
  提示；校准将真实下压）；先寻力接触再按 setpoints_n（默认
  "5,10,20,30"）逐点 BondingByForceManual 稳态采样 → mean/3σ + 线性
  拟合 gain/offset_n；结果 **mean_n vs setpoint_n 散点+拟合直线** +
  每 setpoint 一行 mean/3σ 统计表 + **"写回力校准"按钮**
  （force_gain/force_offset_n→sensor_calib 组）。

### 5.2 测校项总表（18 项，与 `main.py:82-101` SCRIPTS 注册表逐项一致）

前 2 项走 MQTT 原语链（非作业 API，无 routine 名）；后 16 项走作业 API
（job_base 机械层）。cam 列为 GUI build_cam 实际下发值。

| # | 列表显示名 | 页文件 | 后端 routine / 链路 | cam | 引入迭代 |
| --- | --- | --- | --- | --- | --- |
| 1 | 相机对准重复性·静态 | `SingleCameraAlignStability.py(+_ui.py)` | MQTT 原语（take_photo 链） | 相机下拉 up/down/coarse | 26 |
| 2 | 相机对准重复性·动态 | `AlignStabilityDynamic.py(+_ui.py)` | MQTT 原语（move_stage+take_photo 链） | 相机下拉；down 附 jog_group wafer/align | 26 |
| 3 | 光谱共焦重复性 | `ConfocalRepeat.py(+_ui.py)` | `confocal_repeat` | 探头下拉 up/down | 30 |
| 4 | 运动台移动重复性 | `StageMoveRepeat.py(+_ui.py)` | `ws_move_repeat`（晶圆台）/ `as_move_repeat`（对准台） | 晶圆台恒 down；对准台 up 缺省/down 附 jog_group=align | 30 |
| 5 | 芯片交接重复性 | `DieTransferRepeat.py(+_ui.py)` | `die_transfer_repeat` | coarse | 30 |
| 6 | 硅片台旋转重复性 | `WsRotateRepeat.py(+_ui.py)` | `ws_rotate_repeat` | down | 30 |
| 7 | 硅片台旋转中心标定 | `WsRotateCenterCalib.py(+_ui.py)` | `ws_rotate_center` | down | 32 |
| 8 | 旋转角度校准 | `WsRotateGainCalib.py(+_ui.py)` | `ws_rotate_gain` | down | 32A |
| 9 | 对准台非正交测校 | `AsStageNonortho.py(+_ui.py)` | `as_stage_nonortho` | down | 36 |
| 10 | 硅片台非正交测校 | `WsStageNonortho.py(+_ui.py)` | `ws_stage_nonortho` | down | 36 |
| 11 | 相机微小旋转角标定 | `CameraMicrorotCalib.py(+_ui.py)` | `camera_microrot` | up/down/coarse 三相机全支持 | 31 |
| 12 | 上吸盘位置标定 | `AsHeadCenterCalib.py(+_ui.py)` | `as_head_center` | up | 42 |
| 13 | 相机最佳焦平面标定 | `BestFocusCalib.py(+_ui.py)` | `best_focus` | up/down 下拉 | 42 |
| 14 | 键合头Rx/Ry旋转轴标定 | `BsRxryAxisCalib.py(+_ui.py)` | `bs_rxry_axis` | up | 49 |
| 15 | Flip与键合头承接高度标定 | `FlipHandoffHeightCalib.py(+_ui.py)` | `flip_handoff_height` | up | 49 |
| 16 | 键合头键合高度标定 | `BondHeightCalib.py(+_ui.py)` | `bond_height_calib` | up | 49 |
| 17 | 键合温度校准 | `BondTempCalib.py(+_ui.py)` | `bond_temp_calib` | up（heater=up/wafer 与 cam 无关） | 49 |
| 18 | 键合力校准 | `BondForceCalib.py(+_ui.py)` | `bond_force_calib` | up | 49 |

后端侧 routine 注册见 `runtime/calibration.py:129-137`（ROUTINES，另含
focus_scan/pixel_equivalent/rotation_direction 三旧标定项与
align_repeat_static/dynamic 两 panel 项，不进本软件）与 ROUTINE_CAM
（`runtime/calibration.py:164-182`）。

### 5.3 拍照链直给化（迭代 42）

相机两项（MQTT 原语链）拍照链三项变更（用户拍板"直给"口径；e2e 实证：
真后端+offscreen GUI 静态对准 n=2 全链 3.4s finish、Operation Log 无
warm-up、判败链（无标记模板）两拍 3.2s 内直给报错不卡等）：

1. **warm-up 首拍删除**：迭代 27 曾"正式循环前先打 1 拍结果丢弃"
  （冷启动 REAL 缓存在统计外完成；迭代 29 起动态项移到首次走到拍照位
  之后）——后端 `_warmup` 启动已预热，GUI 侧 warm-up 本就冗余且是
  拍照链卡等重灾区；迭代 42 起两相机页删 warm-up，**首拍即测量拍**
  （`SingleCameraAlignStability.py:14-16`、`AlignStabilityDynamic.py:28-31`）。
2. **15s 不出结果即报错直给**：`take_photo`/`wait_image_process` 默认
  超时 30s→**15s**（`device_actuator.py:52-63`、
  `mqtt_receive_worker.py:211-230`）——口径=后端 REAL 帧上限 12s
  （REAL_FRAME_TIMEOUT_S）+ 3s 曝光/渲染余量；到点未回即判败
  （`拍照应答超时(>15s)`），由 take_photo 重拍 1 次兜底（仍败抛
  TakePhotoError 带原因），**不再为积压场景留 30s 长等傻等**。
  （迭代 48 配套加固：接收 worker 批量排空每轮 ≤50 条 + start 先清
  闲置积压 CTL 帧 + also_accept 收下同一 take_photo 内上一 attempt
  迟到帧——同机位物理等价，15s 窗口在积压/贴线场景不再误判双杀，
  `mqtt_receive_worker.py:66-110`、`device_actuator.py:66-76`。）
3. **接收线程断线自动重连**：`PahoMqttClient.__reader` 对 recv 异常
  不再静默 break——打原因后进自动重连循环（退避 1s 起步 ×2 封顶 5s：
  重建 MqttSub socket → 按 `_sub_topics` 全量重新订阅 → 继续 recv；
  锁内换边防泄漏、退避 0.2s 切片响应 close()、subscribe 握手在本
  线程内同步完成不与 recv 并发抢字节流）；仅 close()/停止标志可终止
  接收线程（`cmdjson/PahoMqttClient.py:13-17,81-160`）。发布侧
  mqtt_publish 一次性连接与订阅连接互不影响，重连期间发布照常。
  配套：陈旧帧/camID 不符丢弃从控制台 print 改走 log 回调进
  Operation Log（`mqtt_receive_worker.py:143-156`）；重连事件同路
  上日志（迭代 48）。

**与历史"拍照成功但软件干等"（相机测试偶发等待超时）遗留问题的关系**：
该症状的根因不是后端慢，而是 GUI 接收线程 recv 异常后**静默 break
死亡**——VSS_Com 回帧从此无人入队，wait_image_process 无论等多久都
收不到，超时只是被动兜底（且旧 30s 口径下表现为长等）。迭代 42 的
重连循环根治"线程死了无人知"这一根因；15s 直给把残余病态场景
（broker 断线/接收积压/重连窗口 QoS0 丢帧）收敛为**有日志、有报错、
有重拍**的有界失败。作业 API 测校项（运动台移动重复性等）无此问题——
走纯 HTTP 作业链（job_base），结构上无 MQTT 回帧等待。注意区分更早的
另一例"偶发超时"（2026-09-01 已根治）：那是后端 broker 120s 读超时
误杀纯订阅连接导致视觉链丢帧，属后端 mqtt_broker 修复，与本项 GUI 侧
修复是两个独立根因（见开发交接 2026-09-01 条）。

### 5.4 机器常数读写链（标定类 routine 的 measured → machine_constants）

**读链**：GUI 启动/刷新机器常数走 `GET /api/machine_constants`（响应含
constants 全量分组 + cameras 派生只读字段 k_um_px/fov），如
metrology_config 的 nm_per_px 刷新（`refresh_nm_per_px`）；后端侧由
`runtime/machine_constants.py` 的注册表（`MachineConstants._d`）统一
持有，机器常数页 PUT /api/machine_constants 与 apply 写回都经同一
`update()` 深合并→全量校验→原子落盘。

**写链**：标定作业 done 后，GUI「写回」按钮发
`POST /api/calibration/apply {"job_id": N, "fields": [...]}`
（`runtime/calibration.py:1448-1507`）：

1. fields 子集校验（空/非数组/字段不在 measured → 400；纯测量程序无
   measured → 400）；
2. **组路由**（`APPLY_GROUP_ROUTE`，`runtime/calibration.py:205-229`）：
   相机内参字段（rot_mrad/mag/wd_mm/dir_x/dir_y 等）写回
   `cameras[cam]`；非相机字段按下表路由到对应机器常数组——
   `rz_center_offset_mm/rz_center_ref_xy_mm/rz_gain → fine_align`
   （FINE_ALIGN_FIELDS 校验原生支持："xy" 二维向量 / ("num",1e-6,1e6)，
   `runtime/machine_constants.py:155-170`）、
   `as/ws_nonortho_urad → stage_geo`、
   `as_anchor_xy_mm → wafer_ega`、
   `as_head_center_xy_mm/as_z_chuck_focus_mm/head_z_upfocus_mm → focus_level`、
   `rx/ry_axis_point_mm+rx/ry_axis_dir → bs_rxry_axis`（"xyz" 向量校验）、
   `head_z_touch_mm → flip_handoff`、`head_z_contact_mm → bond`、
   `head_/wafer_temp_gain+offset_c、force_gain/force_offset_n → sensor_calib`
   （迭代 42/49 逐批新增）；
   **别名路由**（迭代 49）：bond_temp/bond_force_calib 的 measured 同带
   显示别名键（gain/offset_c/offset_n）与常数名同值键——panel 按显示键
   提交时映射到常数名键（从 job.result.measured **全量**建 alias，勿用
   fields 过滤后子集）；
3. `constants.update(patch)` 深合并 → 全量 schema 校验（失败
   ValueError → API 400）→ 原子落盘 machine_constants.json；
4. `vision.apply_constants()` 热生效（渲染/解析/对焦立即跟随），
   响应 {"ok", applied, persisted, derived}。

已有写回入口的 GUI 页：camera_microrot（rot_mrad→cameras[cam]）、
ws_rotate_center（rz_center_offset_mm→fine_align）、ws_rotate_gain
（rz_gain→fine_align）、as_head_center（as_anchor_xy_mm+
as_head_center_xy_mm 双键联动→wafer_ega/focus_level）、best_focus
（as_z_chuck_focus_mm/head_z_upfocus_mm 按相机→focus_level）、
bs_rxry_axis（rx/ry 轴点+方向四键→bs_rxry_axis）、flip_handoff_height
（head_z_touch_mm→flip_handoff）、bond_height_calib
（head_z_contact_mm→bond）、bond_temp_calib（head_/wafer_temp_gain+
offset_c 按 heater→sensor_calib）、bond_force_calib（force_gain+
force_offset_n→sensor_calib）。

## 6. 真机迁移

换真机时预期只改 `metrology_config.py`：BROKER 地址、相机表 camID 编码、
轴名表（配合 mqtt_cmd_map.json 换成真机命名）。协议信封/话题/时序不变。

## 7. 已知边界

- `PahoMqttClient.unsubscribe` 是伪实现（迷你 broker 无 UNSUBSCRIBE），无实际影响。
- 后端不在线时界面可开，点开始经 sig_error 落"MQTT broker 未连接"。
- 拍照光源由后端 trigger 链自动闪光（vision_service._do_trigger），GUI 无需手动点灯；
  但视野内须有已注册模板对应的标记（judge=false 会显式报错中止，不再吞零）。
- 动态对准重复性（迭代 29 起）：下相机可选运动台（晶圆台 WS_LS_X/Y / 对准台
  AS_X/Y，对齐后端 jog_group 契约）；warm-up 首拍迭代 42 起删除（首拍即
  测量拍，§5.3）；启动前查 /api/calibration/jobs 占用；
  中断按钮联动 POST /api/calibration/cancel（后端协作式取消，终态 canceled）。
- 失败语义（2026-09 迭代 27 起，刻意超越参考框架的 [0,0] 哨兵；迭代 42 起
  超时 15s 直给，§5.3）：
  wait_image_process 默认 timeout=15s（迭代 42，旧 30s 长等删除）失败/超时
  返回 None；take_photo 失败重拍 1 次仍败
  抛 TakePhotoError；VSC_Com 信封 task 带 IMG_NNNNNN 序号防陈旧帧；run()
  正式循环前的 1 拍 warm-up 迭代 42 起删除（后端 _warmup 启动已预热，
  首拍即测量拍）；接收线程断线自动重连（退避 1s×2 封顶 5s+全量重订阅）。
- REAL 图像噪声：渲染底版为大底版随机偏移视图（帧间去相关），强度由 recipe
  sim_sensors.real_noise_scale{up,down,coarse}（默认 1.0，(0,10]）调节。
- GUI 不进自动回归（Qt 需显示环境）；适配器链路由 tests/test_metrology.py i26 段覆盖；
  迭代 34 契约（模板必填 UI 拒空、StageMoveRepeat 相机下拉 + down 附
  jog_group=align、微旋运动台下拉/拍照位常使能、spinbox 四舍五入）由
  后端 i34 段回归覆盖（`tests/test_metrology.py:3046-3234`，
  `test_i34` 汇总）；迭代 42 两新页/拍照链变更与迭代 49 五新页由
  offscreen 冒烟覆盖（QT_QPA_FONTDIR 修中文丢字，Temp 脚本留档不进
  产品代码）。
