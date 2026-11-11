# 工艺流程：晶圆上片与全局对准（EGA）V1

2026-08-26（第 1 版）。在硅片台上装载晶圆并按 recipe 注入上片误差，
然后通过下精对准相机测量晶圆边缘若干 site 标记，最后做六参数仿射拟合，
得到对准台坐标系 ↔ 晶圆设计坐标系的 placement 变换——为后续上位机
晶圆全局对准（EGA）流程打样。

- 生成器：`runtime/process_flows.py` → `build_wafer_ega_flow(engine, with_load=None)`，
  运行前实时读 `machine_constants.json` 的 `wafer_ega` 组与 `recipe.json` 的
  `wafer_load` 块，PUT 改参后下一次运行自动跟随。
- 执行：`FlowEngine.run_steps`（共用状态机 / pause / stop / step / 日志），
  面板"工艺流程"tab 直接可见。
- API：`POST /api/process/wafer_ega {with_load?: bool, paused?: bool}`
  - `with_load=true`：强制重新上片（用于标定/复测）；
  - `with_load=false`：不上片，要求晶圆已在位；
  - `with_load=null/省略`：无晶圆时自动上片，已在位则跳过。
- 拟合结果：`GET /api/flow/state` 的 `ega_result` 字段；同时落存
  `engine.ega_result`。

## 坐标系约定

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| 对准台坐标系 | AS | 原点=对准台机械零位；下精对准相机光轴随 AS_X/AS_Y 移动，
  z 向由 AS_Z 对焦。 |
| 硅片台坐标系 | WS | 原点=硅片台机械零位；晶圆放在 WS_Rz 旋转台上，
  随 WS_LS_X/WS_LS_Y/WS_Rz 移动。 |
| 晶圆设计坐标系 | W | 晶圆制造/处方定义：圆心在原点，site 按 recipe 规则分布。 |

本流程要标定的就是 **AS ↔ W** 的六参数映射（平移 tx/ty、旋转 θ、
正交误差 ortho、scale sx/sy）。硅片台坐标系到对准台的转换通过
机械常数 `as_anchor_xy_mm` / `ws_ega_xyz_mm` 完成（见"锚点标定策略"）。

## 工艺时序

| # | 步骤 | 轴/IO | 目标值来源 |
|---|------|-------|-----------|
| 0 | （可选）晶圆上片 | `wafer load offset=[load_x,load_y] rz=load_rz` | `recipe.wafer_load` |
| 0b | 晶圆真空建立（两分支均幂等执行，2026-08-28 迭代 8 修复） | `io WT_WaferVac OpenNegtive` + `wait_grip WT_WaferVac expect=true`（超时报警"晶圆真空未建立"） | transfers.json `head_to_wafer` 要求 `WT_WaferVac.oNegtiveState==True` |
| 1 | 安全确认 | AS_X/AS_Y、BS_Arm_Z、FA_X、FA_Z | `flip_handoff.as_safe_xy_mm`、`head_z_safe_mm`、`fa_x_retreat_mm`、`flip_pick.fa_z_safe_mm` |
| 2 | 硅片台到 EGA 测量位 | WS_LS_X/WS_LS_Y/WS_Rz | `wafer_ega.ws_ega_xyz_mm` |
| 3 | 对准台到下相机焦面/锚点 | AS_Z=0、AS_X/AS_Y | `wafer_ega.as_anchor_xy_mm` |
| 4 | 逐点测量 site 标记 | `ega_measure` × n | `wafer_ega.ega_n_points` + recipe site_marks |
| 5 | 六参数仿射拟合 | `ega_fit` | `runtime/ega.py` |
| 6 | 流程结束日志 | — | — |

`ega_measure` 内部：
1. 按 AS/WS 联合行程拆分 site 目标到 `AS_X/AS_Y/WS_LS_X/WS_LS_Y`
   （优先让 WS 承担大行程，AS 只走小偏移）。
2. 移动到位后触发下精对准相机拍照（REAL/FAST 均可）。
3. 视觉识别 site 标记像素 → `Camera.solve_pixel` 得标记世界坐标。
4. 用当前相机光轴位姿换算：
   `measured_rel = (wx - cam_x + site_x, wy - cam_y + site_y)`，
   得到**相对晶圆中心的实际 site 坐标**，写入 `engine.ega_measurements`。

**上片真空时序**（迭代 8 修复，原 bug：上片段只有 `wafer load` 纯工件层
步骤，全程无 WT_WaferVac 气路步骤，键合交接永不触发）：上片分支在
`wafer load` 后、晶圆已在位分支（do_load=false）在片段开头，均幂等执行
`io WT_WaferVac OpenNegtive` → `wait_grip WT_WaferVac expect=true`
（密封=晶圆在位，engine air_grip_ok 语义），超时报警"晶圆真空未建立"。

`ega_fit`：对累计测点做六参数仿射拟合，结果落存 `engine.ega_result`。

## 组合走位与实测坐标换算

测量时对准台（AS）和硅片台（WS）会同时动——看着复杂，数学上是透明的。
三件事说清楚：

### 1. 走位约束是一个等式（`flow_engine._ega_split_targets`）

```
(AS − as_anchor_xy_mm) − (WS − ws_ega_xyz_mm) = site_nominal
```

- `as_anchor_xy_mm=[42.394, 59.0]`：WS 在 EGA 测量位时，AS 光轴对准硅片台
  中心（≈晶圆心）的 AS 坐标（机械常数，真机用气道交叉标定）。
  来源：迭代 4 零位纠错——TwinDOM 轴语义 `position=authored−current`，
  AS_X current=17、AS_Y current=55.4，此前把 authored 当零位导致 AS 世界坐标
  多算 (17,55.4)；纠正后相机真实零位 (-42.5,1.0)，硅片台吸盘中心
  (−0.106,60.0)（WS=[0,60] 时），故 AS 光轴对准吸盘中心需
  AS=(−0.106−(−42.5), 60.0−1.0)=(42.394,59.0)。
- 满足该等式时，相机光轴正对"晶圆心 + site 名义坐标"那个点；上片误差
  tx/ty 让标记偏出光轴中心，但远小于视场半宽（下相机 k=0.6µm/px ×
  3000px，半视场 0.9mm），标记仍在视场内。
- 等式有无穷多解 → 用拆分系数 f∈[0,1]（步长 0.05）在各自软限位内选
  总位移最小的解，大行程 WS（±220mm）优先、小行程 AS（X:-30~80 /
  Y:-30~120）只补差。**两台同动只是行程分配策略，与测量无关。**

### 2. 实测坐标：相机侧单端解算，WS 自然消掉（`_do_ega_measure`）

```
像素(col,row) ─solve_pixel→ 标记世界坐标 w = cam_pos + 像素残差×当量
                            （逆方向性 dir/swap、逆安装微旋 rot）
实测 = 名义 site + (w − cam_pos)
```

- `cam_pos` = 相机光轴世界坐标 = AS 模块 6DOF 位姿（AS 编码器名义值 +
  6DOF 误差）+ 安装偏移（`vision_model.camera_pose`）。
- 像素残差 (w − cam_pos) 是"标记偏离相机光轴多少"，只取决于标记与相机
  的**相对位置**。加回名义 site 后：实测 = 名义 + 残差，残差里就含着
  上片误差对该 site 的作用量，n 个点拟合即解出六参数。
- 关键：w 和 cam_pos 都是绝对世界坐标，相减后 WS 位置、拆分系数 f 全部
  消去——f=0.3 和 f=1.0 测出的"实测"一致（只差伺服/视觉噪声）。

### 3. WS 任意位置时只动 AS：等式的 f=1 特例

```
AS_target = as_anchor + (WS_current − ws_ega) + site_nominal
```

WS 位置编码器永远已知，代入即可——不需要 WS 回测量位。算例
（site=(116.2, −43.8)，anchor=[42.394, 59.0]，ws_ega=[0, 60]）：

| WS 实际位置 | AS_X 目标 | 可达性 |
|---|---|---|
| (−100, 0) | 42.394 + (−100−0) + 116.2 = **58.594** | ✓ 在 [−30, 80] 内，只动 AS 即可找到 |
| (0, 60)（测量位） | 42.394 + 0 + 116.2 = **158.594** | ✗ 超程（上限 80），必须让 WS 分担 |

所以"只动对准台找标记"在数学上永远成立，物理上受 AS 行程限制；EGA
拆分走位就是为了对全片所有 site 都够得着。这件事之所以成立，依赖两条
真机语义：①双台位置闭环已知（编码器）；②AS↔WS 的机械关系已用锚点
常数标定。仿真按同样语义建模，行为与真机一致。

## EGA 六参数拟合

实现：`runtime/ega.py` → `fit_similarity(nominal, measured)`。

模型：
```
[ mx ] = [ sx·cosθ − sx·sinθ + ortho·sinθ   ] [ nx ] + [ tx ]
[ my ] = [ sy·sinθ + sy·cosθ − ortho·cosθ   ] [ ny ] + [ ty ]
```
其中 `(nx, ny)` 为名义 site 坐标，`(mx, my)` 为实测坐标。
最小二乘解出 `tx, ty, θ, sx, sy, ortho`，并返回残差 3σ。

注入的 `wafer_load` 误差即为：
```
W' = R(load_rz) · W + T(load_x, load_y)
```
因此拟合结果应找回 `tx≈load_x`、`ty≈load_y`、`θ≈load_rz`，
scale≈1、ortho≈0、残差 3σ 在 µm 级。

## 锚点标定策略

真机没有"对准台坐标系 = 硅片台坐标系"的物理保证，必须用一个机械常数
描述两者关系。

- `ws_ega_xyz_mm`：硅片台 EGA 测量位。选择该位置时，对准台在其自身
  行程范围内能够扫过整片晶圆所有 site 标记。
- `as_anchor_xy_mm`：当 WS 处于 `ws_ega_xyz_mm` 时，把 AS 移动到
  `as_anchor_xy_mm`，下相机光轴正好对准硅片台吸盘中心（可用硅片台
  气道交叉或陶瓷面中心标定）。

二者合起来构成一个**锚点**：EGA 测得的 (site_x, site_y) 名义坐标以
晶圆中心为原点，实测坐标是在 AS 坐标系下、相对该锚点的偏移；拟合结果
因此就是 AS ↔ W 的 placement，不含 WS 的绝对位置。

当前默认值（迭代 4 零位纠错后）：
- `ws_ega_xyz_mm = [0.0, 60.0, 0.0]`
- `as_anchor_xy_mm = [42.5, 59.0]`

均为占位值，待真机标定后回填。

## 机械常数表（machine_constants.json `wafer_ega` 组）

| 键 | 默认 | 含义 / 来源 | 备注 |
|---|---|---|---|
| `ws_ega_xyz_mm` | [0.0, 60.0, 0.0] | 硅片台 EGA 测量位（WS_X/WS_Y/WS_Rz） | 占位，需保证对准台能看到全片 site |
| `as_anchor_xy_mm` | [42.394, 59.0] | WS 在测量位时，AS 光轴对准硅片台中心的坐标 | 迭代 4 零位纠错后默认值；真机用气道交叉标定 |
| `ega_n_points` | 9 | EGA 测点数 | 沿晶圆边缘均匀分布 |
| `ega_measure_timeout_s` | 10.0 | 单点视觉测量等待超时 | 待调 |

## 处方参数表（recipe.json `wafer_load` 块）

2026-08-27 起，这组参数在面板**「测校」页**的「晶圆 · 上片误差注入」
分组修改（由处方页迁入；保存与处方页"保存改动"同一入口，data-p 绑定
全局生效）。

| 键 | 默认 | 含义 |
|---|---|---|
| `load_x_mm` | 0.25 | 晶圆中心相对硅片台中心的 X 偏移（mm） |
| `load_y_mm` | -0.18 | 晶圆中心相对硅片台中心的 Y 偏移（mm） |
| `load_rz_rad` | 0.00025 | 晶圆相对硅片台的旋转（rad） |
| `jitter_xy_um` | 5.0 | 随机落片抖动 σ（µm），`jitter_on` 控制 |
| `jitter_rz_urad` | 30.0 | 随机旋转抖动 σ（µrad） |
| `jitter_on` | true | 是否启用随机抖动 |

这些参数模拟上片机械手/托盘的重复误差。关闭抖动后可用确定性注入值
直接验证 EGA 拟合能否找回（见测试锚点）。

## 流程守卫与错误

- 视觉系统未挂载 → 生成器抛 `RuntimeError`（main.py 启动顺序保证挂载）。
- 晶圆键合位不足 `ega_n_points` 个 → 生成器抛 `ValueError`。
- 上片后晶圆不在位（气压检查失败）→ 流程第 0 步继续，但后续测量无标记
  → `ega_measure` 超时 abort。
- 测点 move 撞沙盒 / 轴未使能 / 联锁触发 → 对应步骤 fail，流程 abort。
- 测点 < 3 个时调用 `ega_fit` → `RuntimeError`，abort。

## 测试锚点

`tests/test_client.py` 19.8 节 `test_wafer_ega`：
- 关闭 jitter，注入 `load_x=0.2mm`、`load_y=-0.15mm`、`load_rz=200µrad`。
- 跑 `POST /api/process/wafer_ega {with_load:true}`。
- 断言：流程 done、n=9、拟合 tx/ty/θ 与注入值偏差 <10µm/<10µm/<50µrad、
  残差 3σ<20µm。
- 实测（2026-08-26）：tx≈+201µm、ty≈-149µm、θ≈+200µrad、resid_3s≈1.3µm。

无头浏览器验证：`tests/ega_headless_verify.py`
- 启动 Edge headless + CDP，打开联机版页面；
- EGA 完成后拍两张图：WS_Rz=0 与 WS_Rz=0.01rad（0.573°）；
- 判定：HUD 已连接、`__TD_VIZ__` 钩子存在、无查看器报错、EGA 拟合找回
  注入值。

## 已知限制

1. `ws_ega_xyz_mm` / `as_anchor_xy_mm` 为占位值，真机需用标准标定片
   或气道交叉重新测定。
2. 当前只测每个 site 的第 1 个 mark；需要多点/mark 时，可增大
   `ega_n_points` 或在 `build_wafer_ega_flow` 内扩展 mark 索引。
3. 拟合结果是 AS ↔ W 的映射，尚未回写到工件层影响后续键合寻址
   （下一步：把 `ega_result` 喂给键合流程的 die 寻址）。
4. 硅片台坐标系与对准台坐标系的 z 向关系当前未参与拟合（远心相机
   焦面平面假设）。
