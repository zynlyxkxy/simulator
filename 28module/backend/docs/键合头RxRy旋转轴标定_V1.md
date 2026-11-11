# 键合头RxRy旋转轴标定_V1

测校 routine：`bs_rxry_axis`（迭代 49 新增，cam 恒 `"up"`，
ROUTINE_CAM 约束 `runtime/calibration.py:178,281-285`）。实现
`backend/metrology/bs_rxry_axis.py`（`run_bs_rxry_axis`，
`bs_rxry_axis.py:166-466`）；参数/前置校验
`runtime/calibration.py:1131-1172`（`_validate_bs_rxry_axis`）。
算法原型为用户提供的纯 numpy 参考仿真
`测校/一些测校仿真/RxRy旋转轴标定/rxry_axis_sim.py`（SVD 逆解忠实
移植，移植关系见 §7）。

本文面向设备工程师逐行核对算法：所有公式、字段名、阈值均对准代码
实际实现（引用格式 `文件:行号`）。单位约定：直线 mm、旋转 rad
（显示层 µm/µrad/mrad 处注明换算）。

---

## 1. 测的是什么

**定义**：键合头两根调平旋转轴 BS_CAL_Rx / BS_CAL_Ry 的**实际旋转
轴线空间表示**——方向单位向量 `dir` + 轴线上一点 `point`（世界
坐标，mm），每根轴一对：

| measured 键 | 内容 |
|---|---|
| `rx_axis_point_mm` / `rx_axis_dir` | BS_CAL_Rx 轴线上一点 [x,y,z] / 方向单位向量 |
| `ry_axis_point_mm` / `ry_axis_dir` | BS_CAL_Ry 轴线上一点 [x,y,z] / 方向单位向量 |

用途：apply 写回机器常数 `bs_rxry_axis` 组（`machine_constants.json
:151-173`）。**注意：写回当前仅为认知侧存储**——全 backend 尚无
运行时消费方（6DOF 误差链的 R 链元 dir/center 只读
`error_kinematics.json:170-199`，不读本组；调平链亦无引用），与
stage_geo.*_nonortho_urad 同性质：先解决"测得出、存得下"，消费链
（误差链/调平换算接入）属后续特性。默认值与
error_kinematics 真值对齐（point=[−42.5, 7.0, 1215.25]、Rx
dir=[1,0,0]、Ry dir=[0,1,0]——RxRy 调整球座 AABB 中心，占位值，
**待真机标定**，`error_kinematics.json:148`）。

**原理**：die 上 4 个角标为刚体点集。旋转前（phase=−a）测一遍各
标记三维空间位置 P_k——XY 由上相机测（identify→质心→反投影），Z
由上光谱共焦测；绕 BS_CAL_Rx（或 BS_CAL_Ry）转 ±angle_mrad 两
相位后（phase=+a）再测 Q_k；同一根轴的 −a/+a 两组三维点对做 SVD
刚体拟合，解出轴方向（Rodrigues 逆）与轴上点（⊥dir 平面 2D 闭式
旋转中心）。两根轴独立采集、独立拟合。差分有效转角 **Θ=2a**
（−a↔+a），是全部精度预算的核心杠杆（§5.1）。

---

## 2. 算法推导

### 2.1 标记布局与测量链

标记 = die 4 角标（`_mark_nominals`，`bs_rxry_axis.py:133-163`）：
recipe `film.marks.ratio`（±0.76，`recipe.json:175-192`）× die 半宽
（`film.dies.size_x/y_mm` 默认 10mm → 半宽 5mm → 四角 **±3.8mm**
杠杆），叠加键合头吸盘心名义位 `_HEAD_CENTER_XY=(−42.5, 7.0)`
（`bs_rxry_axis.py:130`，error_kinematics bond_head 工具点）与 die
site 偏移。`per_die<3` → CalibError（拟合至少 3 点，
`bs_rxry_axis.py:146`）。

测量链两个传感器：
- **XY：上相机**。相机零位世界 = align_stage 工具点 (−42.5,23.5) +
  mount (0,−22.5) = **(−42.5, 1.0)**（`_CAM_ZERO_XY`，
  `bs_rxry_axis.py:126-128`）；AS_X/Y 目标 = 标记世界 − 相机零位
  （`bs_rxry_axis.py:206-207`）。质心经 `_context`
  （`metrology/repeatability.py:99`）→ `cam.solve_pixel` 反投影得
  世界 XY（含 dir/swap/rot_mrad 认知），与 as_head_center 同口径。
- **Z：上光谱共焦**。探头相对相机偏移 `focus_level.
  confocal_cam_dx/dy_mm` = (0, −25)（`machine_constants.json:94-99`），
  AS 走位该偏移量把探头落到标记点上（`bs_rxry_axis.py:229-231`）。

### 2.2 作业时序（伪码）

```
保存现场 restore_axes = (BS_CAL_Rx, BS_CAL_Ry, BS_Arm_Z,
                         AS_Z, AS_X, AS_Y)        （:185-190）
点灯（上相机光源，:260-261）
前置位姿（照 extra_routines._pose_for_head_shot 口径，
         extra_routines.py:317-324）：
    AS_Y=6.0、AS_Z=0.0、BS_Arm_Z=−118.33（die 底面进上相机焦面）、
    BS_CAL_Rx/Ry→0                                （:262-268）
走到角标 0 名义位后 _check_marks 视野预检          （:269-275；
    头心位距四角 ±3.8mm 超 up 相机 1.8mm FOV，预检须在角标位做）
bs_z_meas = BS_Arm_Z 当前位（轴点 z 参考系修正用，:278）
对 axis ∈ (BS_CAL_Rx→"RX", BS_CAL_Ry→"RY")：       （:279）
  对 phase ∈ (−a→"neg", 0→"zero", +a→"pos")：      （:280-281；
      a = angle_mrad/1000 rad；每相首 _check_cancel :282）
    走 axis → phase
    逐标记 k（4 角标；每标记首 _check_cancel :287）：
      repeat 次 _measure_point → 均值 (x̄,ȳ,z̄)     （:288-301）
  axis → 0（每轴测完回零再测下一轴，:304）
SVD 拟合（每轴：phase=−a 组 ↔ phase=+a 组，:305-380，见 2.4/2.5）
finally：恢复六轴轴位 + 光源（:381-390）
落盘双 CSV（nodes + summary，:391-421）→ 组装结果（:429-466）
```

注意 **repeat 语义**：本 routine 的 repeat 是**单点重复测量取均值**
的次数（`bs_rxry_axis.py:288-301`，`_mean_std` 取 mean，
`repeatability.py:186`），不是 repeatability 族的统计循环数——
均值只能压随机噪声（∝1/√R），压不动系统项（§5.1）。

**zero 相不进拟合**：phase=0 组测量数据只进结果 `points` 键与
nodes CSV（供三段位移目视诊断），SVD 只用 −a/+a 两组（差分转角
Θ=2a 最大，`bs_rxry_axis.py:307-309`）。

### 2.3 单点三维测量（_measure_point，`bs_rxry_axis.py:201-257`）

**XY（相机，迭代精找一次再取值，:210-225）**：

```
AS_X/Y → 标记名义世界 − (−42.5, 1.0) → wait_ms
拍 1：identify(template_name) → _centroid → _context
      → mark_world mw、cam_world cw
δ = mw − cw；若 |δ| > 0.02mm：AS_X/Y += δ（相机随对准台动，
      新位=当前 AS 位+δ）→ wait → 拍 2 取值
```

迭代精找不是冗余：绕轴旋转时角标 y 向位移 = Θ×(z_标记−z_轴) 可达
亚 mm 级（§4.1），首拍质心偏离画幅中心，反投影含透视/微旋残差，
精找一次后取值。

**Z（共焦，:229-239）**：AS 走位共焦偏移（探头随台动，
`AS_new = AS_now − (conf_dx, conf_dy)`）把探头落到标记点，读
`confocal.read("up")`（`runtime/confocal.py:178-236`）：

```
reading = conf_mid_up + (z_focus − z_surface)×1000 + noise   〔共焦模型
          raw = mid + (zf−zs)×1000，confocal.py:213,225——朝上看〕
⟹ z_surface = z_focus − (reading − conf_mid_up)/1000   （:239）
```

`conf_mid_up = focus_level.confocal_mid_up_um` = 350µm（中点共面
约定：读数 350µm ↔ 被测面正好在焦面，`confocal.py:11-18`）。读数
无效（ok=False / reading None / valid=False——面高超出 [0,700]µm
量程）→ CalibError「共焦读数无效（probe=up 量程外——…检查
BS_Arm_Z 焦面位/片厚）」（`:233-237`，§5.3）。

每点记录一条 nodes CSV 行（轴位快照 + 世界坐标 + 共焦读数 +
相机世界，`:247-253`）。

### 2.4 SVD 刚体拟合：轴方向与有效转角

代码 `_estimate_rotation_axis_svd`（`bs_rxry_axis.py:94-114`）+
`_rotvec_from_R`（`:82-91`），与参考脚本逐步对应（§7）。输入
phase=−a 组 `P={P_k}`、phase=+a 组 `Q={Q_k}`（各 4 点三维坐标，
`:307-309`）：

```
质心      cP = mean(P)              cQ = mean(Q)
协方差    H = (P−cP)ᵀ(Q−cQ)
SVD       [U,~,Vt] = svd(H)；R = V·Uᵀ
          若 det(R)<0：V 第 3 列取反后重算 R（去反射，:107-109）
平移      t = cQ − R·cP                                  （:110）
RMSE      sqrt(mean(|P·Rᵀ + t − Q|²))                    （:111）
方向/角   w = [R₃₂−R₂₃, R₁₃−R₃₁, R₂₁−R₁₂]
          sinΘ = |w|/2，cosΘ = (tr(R)−1)/2，Θ = atan2(sinΘ, cosΘ)
          dir = w/(2·sinΘ)；Θ<1e-9 → ValueError          （:85-91）
```

即 3D Kabsch/正交 Procrustes 闭式解：只允许刚体旋转+平移下使
`Σ|R·P_k + t − Q_k|²` 最小。Kabsch **平移自由**：旋转带来的整体
位移（4 点共同的 y 向摆动，§4.1）进 t 不进 R，只有逐点**差分**
分量约束轴——这就是 ±3.8mm 杠杆上的 ±11.4µm 级 Z 信号决定方向
精度的原因（§5.1）。

**方向符号归一化**（`:313-315`）：SVD 给出的轴方向有整体正负
简并，契约定 **Rx 的 dir x 分量强制为正、Ry 的 dir y 分量强制为
正**（为负则整体取反），保证 measured/写回值符号稳定。

### 2.5 轴上点：⊥dir 平面 2D 闭式旋转中心（lstsq 病态弃用）

参考脚本原式 `lstsq(I−R, t)` 解轴点在小转角下**病态**：det(I−R)
~Θ²，测量噪声按 ~1/Θ² 放大成天文数字（Θ=3mrad 时放大 ~10⁵ 级）。
实现侧弃用该通道（`bs_rxry_axis.py:316-322` 注释），改走**垂直于
拟合方向的平面内 2D 闭式旋转中心**（与 extra_routines._fit_rot
同口径，`:323-355`）：

```
k = dir（单位向量）
e = 与 k 最不平行的坐标轴单位向量（argmin|k_i|，:328）
u = norm(k×e)；v = k×u                # ⊥k 平面正交基（:329-331）
p2 = P·[u,v]，q2 = Q·[u,v]            # 点集投影到 ⊥k 平面（:332-333）
p̄,q̄ = 投影面重心
th2 = atan2(Σcross, Σdot)             # 2D Kabsch 拟合面内转角（:336-342）
M = [[1−cos th2, sin th2],[−sin th2, 1−cos th2]]，det = 2(1−cos th2)
det < 1e-18 → CalibError「角度档退化」 （:347-350，angle_mrad 过小）
c2 = M⁻¹·(q̄ − R2·p̄)                   # 面内旋转中心闭式解（:351-354）
point = k·(cP·k) + c2_u·u + c2_v·v    # 组装回 3D（:355）
```

**沿轴分量不可观测**（旋转不变性：绕轴旋转对轴线上点不产生位移，
(I−R) 沿 k 秩亏）：`c2` 只定垂直分量；沿轴分量取**测量重心 cP 在
k 上的投影**作锚（`:355`），只保量级语义（与默认常数球座心
x=−42.5 同口径）。判读与回归断言都只看垂直分量（§5.2）。

### 2.6 参考系修正与逐标记残差

**BS_Arm_Z 参考系修正**（`:357-361`）：error_kinematics 的 R 链元
枢轴随 BS_Arm_Z 联动（rodrigues center 按 T 链元平移），实测的是
「当前测量位（BS_Arm_Z=−118.33）的有效轴线」——世界 z = 常数语义
z + BS_Arm_Z。机器常数定义在 **BS_Arm_Z=0 系**，报告值
`point[2] −= bs_z_meas` 折回（测量位 −118.33 ⟹ 报告 z 比实测
z 高 118.33mm）。

**逐标记残差**（`:362-372`）：复用 SVD 的 R/t 重算
`|R·P_k + t − Q_k|`（mm），进 summary CSV（`mark{i}_resid_mm`）；
整体 RMSE 进 measured.rx/ry_rmse_mm——异常大提示非刚体运动/夹持
滑移/标记误识别（参考脚本判据 >10µm 报警，`rxry_axis_sim.py:705`）。

### 2.7 CSV 列定义

统一落盘 `runtime/metrology_data/`，命名 `<prefix>_MMDD_HH_MM_SS.csv`
（重名自动 `_2`/`_3` 后缀，`_csv_write`，`extra_routines.py:92-105`）。
本 routine 双 CSV：

**nodes**（`bs_rxry_axis_nodes_*.csv`，表头 `:391-397`；每
「轴×相×标记×rep」一行，共 2×3×4×repeat 行）：

```
axis, phase, mark_id, rep, col, row,
AS_X_mm, AS_Y_mm, AS_Z_mm, BS_Arm_Z_mm, BS_CAL_Rx_rad, BS_CAL_Ry_rad,
mark_world_x_mm, mark_world_y_mm, mark_world_z_mm,
confocal_reading_um, cam_world_x_mm, cam_world_y_mm
```

**summary**（`bs_rxry_axis_summary_*.csv`，表头 `:419-421`；行
`:400-418`）：

```
axis, item, v0, v1, v2, _r0, _r1
```

行项：每轴 `axis_point_mm`（v0..v2=x,y,z）、`axis_dir`、
`angle_rad`（拟合有效转角 Θ）、`rmse_mm`、`mark{i}_resid_mm`×4，
末尾 meta 行 `angle_mrad`、`repeat`。

结果回执：`csv`/`csv_name` = nodes 路径/文件名，`csv_summary` =
summary 路径（`:449-450`）。

---

## 3. 详细测试步骤（SOP 级）

### 3.1 前提条件

1. **键合头上有 die**（start 同步守卫，`calibration.py:1168-1170`；
   无 → 400「键合头无芯片（bs_rxry_axis 需要 head 上有 die——先做
   交接或参照 prep_up）」）：先走完整交接链把 die 交到键合头
   （晶圆上片 → flip 取片 → 交接上头；回归侧对应 `prep_up`）。
2. **模板已注册**：默认模板名 = 机器常数 `vision_marks.
   tpl_die_head`（上相机认 die 角标，`machine_constants.json:132`）；
   作业不自建不删模板，未注册 → 400 并列出模板库可用名
   （`calibration.py:1152-1157`）。
3. 视觉链路可用 cv2（REAL 模式必需；start 统一拦
   `calibration.py:289-291`）；共焦链已挂（`eng.confocal` 非空，
   否则作业内 CalibError，`bs_rxry_axis.py:180-181`）。
4. **无工艺流程在跑**（流程 running/paused、全片键合、一键串联
   → 400，`_check_flow_idle`，`calibration.py:1097-1112,1171`）。
5. recipe 已挂且 `film.marks.per_die ≥ 3`（默认 4 角标，
   `recipe.json:172-194`）。

### 3.2 参数选取

| 参数 | 缺省 | 选取说明 |
|---|---|---|
| `angle_mrad` | 1.5 | [0.2, 1.7]，且 ≤ BS_CAL 软限位 ±0.00174533 rad（±0.1°，`axes_config.py:21-22`；start 双保险校验 `calibration.py:1160-1167`）。**行程允许时尽量取大**——全部残差 ∝1/Θ（Θ=2·angle，§5.1），1.5→1.7 残差再降 ~12% |
| `repeat` | 3 | [1,10] 整数；单点重复取均值，只压随机噪声（∝1/√R），系统项压不动 |
| `wait_ms` | 10 | [0, 60000]；走位后整定等待 |
| `template_name` | 机器常数 tpl_die_head | 一般不传；传了须已注册 |

### 3.3 启动 / 中断 / 判读

```
POST /api/calibration/start
  {"routine": "bs_rxry_axis", "cam": "up",
   "params": {"angle_mrad": 1.5, "repeat": 3, "wait_ms": 10}}
→ job_id；GET /api/calibration/status 轮询；done 后
  GET /api/calibration/result。
POST /api/calibration/cancel {"job_id": N}
  （协作式取消：每相首/每标记首检查点生效 → canceled；
   finally 恢复六轴轴位/光源照跑，bs_rxry_axis.py:282,287,381-390）
```

判读要点：
- `measured.rx_axis_dir` / `ry_axis_dir`：方向单位向量，理想 ≈
  [1,0,0] / [0,1,0]；非主分量即轴线安装偏差（mrad 级读法：
  分量×1000）。**仿真合成环境实测偏差 ~16mrad 级**（系统项主导，
  §5.1，待真机标定）；
- `measured.rx_axis_point_mm` / `ry_axis_point_mm`：**只判读垂直
  分量**（Rx 看 y/z、Ry 看 x/z），沿轴分量是锚定值不具测量意义
  （§5.2）；
- `measured.rx_rmse_mm` / `ry_rmse_mm`：SVD 拟合 RMSE（只读），
  回归口径 <0.1mm（仿真实测 ~8e-5mm 级）；异常大 → 非刚体/误识别；
- `points.RX.neg/zero/pos`：三相逐标记坐标（GUI 画位移 quiver）；
- 写回：`POST /api/calibration/apply {"job_id": N, "fields":
  ["rx_axis_point_mm","rx_axis_dir","ry_axis_point_mm","ry_axis_dir"]}`
  → 机器常数 `bs_rxry_axis` 组 4 键原子落盘+热生效（§6.3）。
  `rx_rmse_mm`/`ry_rmse_mm`/`angle_mrad` 只读不写回。

面板：CAL_ITEMS `panel/index.html:5847`（cam=null）；参数块
`:1365,5792-5794`；start 分支 `:6291-6292`（三参数可空=后端默认）。
PyQt5 测校 GUI 页 `backend/metrology_gui/BsRxryAxisCalib.py`
（双 subplot 标记位移 quiver + 轴点/方向/RMSE 表）。

---

## 4. 计算示例（自洽数字手算复核）

**示例数据为示意**（合成几何，非某次作业记录）：设 Rx 真值
dir=[1,0,0]、轴线 BS_Arm_Z=0 系过 (y_c, z_c)=(7.0, 1215.25)mm；
angle_mrad=1.5（a=0.0015 rad，差分 Θ=2a=0.003 rad）、die 10mm
（4 角标 y=7.0±3.8，即 y∈{3.2, 10.8}）。

### 4.1 相位差分信号量级

测量位 BS_Arm_Z=−118.33 → 测量位有效轴 z_c = 1215.25−118.33 =
**1096.92**。设角标测量位世界 z≈935.0（die 面在焦面邻域，示意），
则相对轴线的杠杆：dy = ±3.8、dz = 935.0−1096.92 = −161.92（4 点
同）。绕 x 轴从 −a 转到 +a 的一阶位移（Q−P）：

```
ΔΔx ≈ 0（绕 x 轴转不动 x）
ΔΔy = −Θ·dz = −0.003×(−161.92) = +0.48576 mm   （4 点共同 → 进 t）
ΔΔz = +Θ·dy = 0.003×(±3.8)    = ±0.0114 mm = ±11.4 µm（逐点差分 → 约束轴）
```

共同 y 摆 0.486mm ≫ 0.02mm 阈值 → 每点必触发一次相机迭代精找
（2.3）；逐点 Z 信号 ±11.4µm 对应共焦读数变化 ∓11.4µm，默认噪声
档 0.065µm（recipe `sim_sensors.confocal_sigma_um.up`，
`recipe.json:82-85`）下单点 SNR≈175。

### 4.2 2D 闭式旋转中心手算（y-z 投影面）

neg 相 z(−a)=935.0+(−0.0015)·dy、pos 相 z(+a)=935.0+0.0015·dy；
y(−a)=7.0−0.24288=6.75712、y(+a)=7.24288（4 点 y 逐点差一阶为 0）：

```
u_k = P_k−p̄ = (±3.8, ∓0.0057)     v_k = Q_k−q̄ = (±3.8, ±0.0057)
Σcross = Σ 2a·dy² = 0.003×(4×14.44) = 0.17328
Σdot   = Σ dy²·(1−a²) = 57.76×(1−2.25e-6) = 57.75987
th2 = atan2(0.17328, 57.75987) ≈ 3.0001e-3 rad ≈ Θ=2a ✓
```

旋转中心（M⁻¹(q̄−R2·p̄)，cosΘ=0.9999955、sinΘ≈0.003、
det=2(1−cosΘ)=9e-6）：

```
R2·p̄ = (0.9999955×6.75712 − 0.003×935.0,
         0.003×6.75712 + 0.9999955×935.0) = (3.95209, 935.01606)
r = q̄ − R2·p̄ = (7.24288−3.95209, 935.0−935.01606)
             = (3.29079, −0.01606)
c2_y = ((1−c)·r_y − s·r_z)/det = (1.4809e-5 + 4.8192e-5)/9e-6 ≈ 7.000
c2_z = (s·r_y + (1−c)·r_z)/det = (9.87230e-3)/9e-6          ≈ 1096.92
→ 测量位轴点 (y,z) ≈ (7.000, 1096.92) ✓ 真值 (7.0, 1096.92)
报告值（BS_Arm_Z=0 系）：z = 1096.92 − (−118.33) = 1215.25 ✓
```

### 4.3 1/Θ 放大量级链（方向残差预算）

方向（面内偏摆）由 Z 信号主导，参考脚本解析式（§7）按本例参数
（Sxx=Σdy²=57.76mm²、√Sxx=7.6mm、R=3）：

```
σ_yaw ≈ √2·σ_z / (Θ·√R·√Sxx)
      = 1.414×6.5e-5 mm / (0.003×1.732×7.6 mm) ≈ 2.3 mrad（1σ）
```

仿真合成环境**实测**方向偏差大于该随机噪声极限约 5 倍
（rx ~13~16mrad，系统项主导，见 §5.1），回归容差按 30mrad 定。
**以上为仿真合成值，待真机标定**；真机验收建议以多次标定解算轴
互差判读（§7）。

---

## 5. 误差来源与注意事项

1. **方向角估计偏差 ∝ 1/Θ（小角度杠杆放大，主项）**：BS_CAL 行程
   ±1.74533mrad 封顶 → a≤1.7mrad、差分有效转角 Θ=2a 仅 ~3mrad
   （1/Θ≈330×——对照参考脚本 Θ=35mrad 档的 29×，这是本实现残差
   mrad 级而参考仿真 µrad 级的根源）。方向信号是 ±3.8mm 杠杆上的
   ±11.4µm 逐点差分 Z 位移，任何 θ 相关的测量系统差按 ~1/Θ 放大
   进方向。仿真合成环境实测：`rx_dir=[0.99987, 0.00173, 0.01607]`
   对真值 [1,0,0] 夹角 **~16mrad**（另一次实现 rx ~13mrad、
   ry ~3mrad——vision 层局部旋转×链条旋转复合系统项主导，repeat
   间稳定，repeat↑ 压不动）；回归容差定 **30mrad**
   （`test_metrology.py:5101-5109`）。**合成值，待真机标定**。
2. **轴点沿轴分量不可观测（旋转不变性/秩亏）**：绕轴旋转对轴线上
   点不产生位移，(I−R) 沿轴秩亏；lstsq(I−R,t) 通道病态弃用
   （det~Θ²，2.5）。垂直分量由 ⊥dir 平面 2D 闭式解定（良性，噪声
   ~σ/Θ），仿真实测垂直分量对真值偏差 **<0.6mm**（回归容差
   1.5mm，`test_metrology.py:5110-5113`）；沿轴分量取测量重心
   投影锚定（回归只查 5mm 量级）——**判读只看垂直分量**。
3. **共焦量程与遮挡约束**：量程 [0,700]µm
   （`focus_level.confocal_range_um`），中点共面约定
   （350µm↔焦面）；BS_Arm_Z=−118.33 是「die 底面进焦面」的装配
   位，焦面常数/片厚错 → 读数越程 → valid=False → CalibError
   （`:233-237`）。差分 Δz 信号 ±11.4µm ≪ 量程——量程约束的是
   **绝对面高**不是信号。探头落点依赖 `confocal_cam_dx/dy_mm`
   =(0,−25) 常数正确（改错→测点偏移）。仿真无遮挡模型；真机需
   确认共焦光路（自上而下看 die 面）在走位行程内不被键合头结构
   遮挡。
4. **噪声各向异性与提升路径**（参考脚本解析/MC 结论，
   `rxry_axis_sim.py:20-25,243-247`）：方向残差的「面内偏摆」与
   位置残差的「横向」分量由共焦 Z 决定，「俯仰」分量由相机 XY
   决定；真机共焦噪声通常远大于相机 → **提升 Z 测量是第一杠杆**
   （多次平均/更高档传感器/改善反射一致性）；其余路径：增大 Θ
   （全分量 ∝1/Θ，受行程封顶）、增大标记展布（方向分量，受 die
   尺寸限）、增加 repeat（随机分量 ∝1/√R，系统项无效）。
5. **die 尺寸依赖**：杠杆 = ratio×die 半宽（默认 ±3.8mm）；die
   越小杠杆越短、方向精度越差（σ_yaw ∝1/√Sxx）。`per_die<3`
   → CalibError（`:146`）。
6. **恒定系统分量相消**：相机常数（k/rot/dir）与链条姿态误差的
   恒定分量同进 neg/pos 两相的反投影，SVD 差分后相消；但 vision
   层**旋转姿态相关**系统差（渐晕/模板在转角下的质心偏移）不
   相消——方向残差主项之一（第 1 条）。
7. **cancel/恢复语义**：检查点每相首/每标记首（`:282,287`）；
   finally 一定恢复 BS_CAL_Rx/Ry、BS_Arm_Z、AS_Z/X/Y 作业前轴位
   与上相机光源（`:381-390`）；每轴测完回零再测下一轴（`:304`）。

---

## 6. API / 参数 / 结果字段表

### 6.1 start 参数（`POST /api/calibration/start`，cam 恒 `"up"`）

校验入口 `_validate_bs_rxry_axis`（`runtime/calibration.py:1131-
1172`）；同步校验失败 → 400。

| 参数 | 类型 | 缺省 | 约束 |
|---|---|---|---|
| template_name | str | 机器常数 `vision_marks.tpl_die_head`（再兜底 `"tpl_die_head"`） | 非空且已注册，否则 400 列可用名（`calibration.py:1141-1158`） |
| angle_mrad | num | 1.5 | [0.2, 1.7]（`_num_field` 闭区间，`calibration.py:1086-1095,1159`）且 ≤ BS_CAL_Rx/Ry 软限位 hi=0.00174533 rad（`calibration.py:1160-1167`） |
| repeat | int | 3 | [1, 10]（`calibration.py:1138-1139`） |
| wait_ms | num | 10 | [0, 60000]（`calibration.py:1140`） |

前置守卫（均 start 同步 400）：cam 非 up（ROUTINE_CAM，
`calibration.py:178,281-285`）；cv2 缺失（`:289-291`）；head 无
die（`:1168-1170`，文案引导先做交接）；工艺流程忙（`:1171`）。

### 6.2 结果字段（组装 `bs_rxry_axis.py:429-466`）

| 字段 | 内容 |
|---|---|
| routine / camera / template_name / repeat / wait_ms | `"bs_rxry_axis"` / `"up"` / 回显 |
| **measured.rx_axis_point_mm / rx_axis_dir** | Rx 轴点 [x,y,z] mm（round 6）/ 方向单位向量（round 8）——**写回键** |
| **measured.ry_axis_point_mm / ry_axis_dir** | Ry 轴点 / 方向——**写回键** |
| measured.rx_rmse_mm / ry_rmse_mm / angle_mrad | SVD 拟合 RMSE（mm）与回读摆角——**只读不写回**（`:433-440`） |
| current.{4 键} | 机器常数 bs_rxry_axis 组现值（去 `_note`，`:441-442`） |
| points.RX/RY.{neg,zero,pos} | 三相逐标记均值坐标 [[x,y,z]×4]（`:443-445`） |
| stats.{rx_rmse_mm, ry_rmse_mm} | RMSE 汇总（`:447-448`）；series=None |
| detail.{angle_mrad, n_marks, confocal_offset_mm, confocal_mid_up_um, note} | 规范化参数与算法摘要（`:451-462`） |
| csv / csv_name | nodes CSV 路径/文件名（`:449`） |
| csv_summary | summary CSV 路径（`:450`） |

### 6.3 apply 写回

`POST /api/calibration/apply {"job_id": N, "fields": [...]}` → 经
APPLY_GROUP_ROUTE 路由 `bs_rxry_axis` 组（`calibration.py:218-221`），
可写字段仅 `rx_axis_point_mm` / `rx_axis_dir` /
`ry_axis_point_mm` / `ry_axis_dir` 4 键；machine_constants
`BS_RXRY_AXIS_FIELDS` 对该 4 键均为 **"xyz" 三维向量校验**
（`runtime/machine_constants.py:212-217`），原子落盘+热生效。

---

## 7. 历史对照（移植自 rxry_axis_sim.py）

算法原型 = 用户提供的参考仿真
`测校/一些测校仿真/RxRy旋转轴标定/rxry_axis_sim.py`（tkinter+
matplotlib GUI：正向 Rodrigues 生成 → SVD 逆解 → 对真值残差 +
解析误差传播 + 蒙特卡洛验收区间；其算法源头注释指 SDS §4.4 与
`Rx_Ry旋转轴标定.py`）。移植关系：

- **忠实移植**：`estimate_rotation_axis_svd` / `rotvec_from_R`
  逐步对应（`bs_rxry_axis.py:82-114` ↔ `rxry_axis_sim.py:40-70`），
  纯 numpy、lazy-import（`_np()`，`bs_rxry_axis.py:71-78`，照
  vision_model.np() 先例），不用 scipy（runtime 纯标准库约束的
  例外通道同口径）。
- **差异 ①（轴点算法升级）**：参考脚本轴点用 `lstsq(I−R, t)`
  （`rxry_axis_sim.py:69`）；实现侧因小转角病态（det~Θ²）弃用，
  改 ⊥dir 平面 2D 闭式旋转中心 + 沿轴重心投影锚定（2.5）。
- **差异 ②（标记布局）**：参考脚本 G×G 网格（默认 3×3、span
  20mm）；实现侧绑定 die 4 角标（2×2、±3.8mm，recipe 决定）。
- **差异 ③（转角量级）**：参考脚本默认 Θ=35mrad（1/Θ≈29×）；
  实现侧受 BS_CAL 行程 ±1.74533mrad 封顶，Θ=2a≤3.4mrad
  （1/Θ≈330×）——残差量级 mrad 级 vs 参考 µrad 级的根源（§5.1）。
- **差异 ④（符号契约）**：方向符号归一化（Rx 的 x / Ry 的 y
  分量强制为正）为实现侧新增，保证写回值符号稳定。
- **判读口径沿用**：参考脚本验收页原话——真机无真值，以**多次
  标定的解算轴互差（一致性）**判读，互差 3σ ≤ MC 验收区间宽度；
  RMSE 异常大（>10µm）→ 非刚体运动/夹持滑移/标记误识别
  （`rxry_axis_sim.py:583-585,705`）。

---

## 附：回归用例索引

- `tests/test_metrology.py:5062`（`test_i49_rxry_400`）：5 个
  start 400 分支——head 无 die（文案含「芯片」）、angle_mrad
  0.1/2.0 越界、cam=down、模板不存在列可用名。
- `tests/test_metrology.py:5082`（`test_i49_rxry_axis`）：prep_up
  真实交接链 die 上头 → angle 1.5/repeat 3/wait 5 跑通 → 方向
  （rx/ry 主分量 ±0.03 rad 容差）+ 轴点垂直分量（y/z±1.5mm、沿轴
  x±5mm）+ RMSE<0.1mm + CSV 落盘 → apply 写回 bs_rxry_axis 组 4 键
  → GET 一致 → finally 还原常数；容差实测依据见用例注释
  `:5101-5104`。
- `tests/test_metrology.py:5436`（`test_i49`）：迭代 49 段汇总
  入口（含上两条）。
- `tests/test_client.py:5810-5840`（18.3b）：机器常数 bs_rxry_axis
  新组在列 + 向量维数错 400 + 未知字段 400。
