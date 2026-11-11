# 项目规则（AGENTS.md）

D2W 键合设备全链路仿真项目。接手工作前先读 `backend/开发交接.md`。

## 硬性规则

1. **每次迭代必须更新 `backend/开发交接.md`**（变更内容、新增/变更文件、当前状态、已知问题）；涉及接口/协议变化时同步更新 `backend/docs/` 下对应文档。
2. **离线运行**：不引入任何外部 URL / CDN / 在线服务依赖；Python 侧只用标准库（除非用户明确批准）。**例外（迭代 41 起）**：项目自带 Python 环境 `env/python/`（3.11，含 numpy/cv2/matplotlib/pandas/PyQt5/openpyxl/xlsxwriter/pytest；迭代 41 起全部环境指向收口于此，bat 启动均设 PYTHONNOUSERSITE=1）；runtime 仿真本体仍纯标准库。
3. **单位约定**：直线 mm、旋转 rad 全线统一；TwinDOM 前端模型内部是 deg，换算只发生在 `html_model/联机注入.js` 边界。
4. **不要直接改** `html_model/d2w_front.html` 和 `.json`（TwinDOM 工程原文件）；联机版由 `backend/tools/build_online_html.py` 生成。
5. **启动脚本 `start.bat`、`launch_metrology_gui.bat` 必须保持 GBK 编码 + CRLF 行尾**（cmd 按系统 ANSI 解析）。
6. 实时性红线：物理 tick 1000Hz、WS 推送 500Hz、本机单向延迟 p50<1ms，回归不许退化。
7. 日常迭代跑 `python backend/tests/test_client.py`（quick 套件，墙钟 ≤3min）；交付/接口变更前跑 `python backend/tests/test_client.py --full`（全量 457 条）。

## 事实源

- 下位机点位表：`1928运动固件点位表_V10/`（**迭代 69 起 V6→V10**；命名沿用其 Cordis 路径；变更明细与影响分析见 `backend/docs/点位表_V10_变更影响分析.md`，V6 目录保留作历史）
- 上位机接口事实源：同事模板工程 io_template / motionstage_template / motion config1~4——RPC 契约、Cordis MCS HTTP 五服务、ACS SDK 指令面、真实 stage 划分（2026-08-19 拿到；**2026-10-29 起 `部分接口信息/` 已从本仓库删除且不再入库**，需要时从 git 历史（提交 `4e23504` 之前）取，或直接 clone `bonder_series/icb3000/io_template.git`、`bonder_series/icb3000/motionstage_template.git`）
- 轴参数：`轴名对应形成和速度加速度/整机轴记录表_V2.xlsx`（2026-08-20 按当前模型重建，生成器 `backend/tools/gen_axis_record_xlsx.py`；原始记录表已不在磁盘，冲突数值以 `接口规范_V1.md` D1/D3 记载为准）
- 接口契约：`backend/docs/接口规范_V1.md`、`前后端数据协议_V1.md`、`视觉系统_V1.md`；键合误差模型（truth 分量分解/下降链/标定残差）：`backend/docs/键合误差模型_V1.md`（迭代 20 起）
- 组件材质工程物性（材料库 17 种×7 维物性 + 组件映射 + API + 热仿真预留）：`backend/docs/材质物性_V1.md`（迭代 21 起）
- 全机模态仿真（21 轴 SISO + 9 全机 MIMO + 三激励 + 曝光积分，默认 OFF 热切换）：`backend/docs/模态仿真_V1.md`（迭代 22 起）
- 组件级热仿真（LPTN 21 节点能量守恒真网络 + 加热器/电机/电子件功率源 + 环境波动档 + 热变形 6DOF 贯通对准/键合真值，默认 OFF 热切换且 OFF 状态保持）：`backend/docs/热仿真_V1.md`（迭代 23 起）
- 晶圆翘曲仿真（bow+saddle 二维静态场 × 逐片 jitter 抽取物料真值 + 共焦面高/视觉离焦/键合接触位贯通，默认 OFF 热切换）：`backend/docs/晶圆翘曲_V1.md`（迭代 73 起）
- 测校软件/SOP 参考（别设备 PyQt5+MQTT 测校框架、重构后 SOP 全套、本设备测校需求草稿、测校仿真）：`测校/`（迭代 24 起接入）；**本设备测校软件 SOP 产出：`测校/1928测校软件SOP/`（迭代 58 起，首篇相机静态对准重复性——docx+images，截图由数字模型自动采集，采集/生成脚本留档 `Temp/_sop_camstatic_*.py`；迭代 72 起目录名由 D2W测校软件SOP 改为 1928测校软件SOP）**；测校程序相对独立——核心在 `backend/metrology/`（独立脚本包，CLI 可连已启动后端单跑），经 `runtime/calibration.py` ROUTINES 注册进 panel 测校页；独立回归 `python backend/tests/test_metrology.py`（不进 test_client.py 全量）；SOP 合订 `backend/docs/测校SOP_相机对准重复性_V1.md`
- PyQt5 测校软件（迭代 26 起）：`backend/metrology_gui/`（复刻参考框架分层，走 MQTT 线协议）+ 仿真侧适配器 `runtime/mqtt_cmd_adapter.py`；架构/协议/真机迁移见 `backend/docs/测校GUI_PyQt5_V1.md`；参考框架原件另在 `C:\Users\e-hao.wu\Desktop\3000测校SOP_u\`（运行解释器迭代 41 起收口到项目 `env/python/`；其 `_tools\.venv`（原 `sop_tools` 已改名）仅留作历史容错兜底）
