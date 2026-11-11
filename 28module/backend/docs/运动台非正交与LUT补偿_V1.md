# 运动台非正交与 LUT 补偿方案（迭代 38 重写）

> 本文是**设计方案**（不是测校项技术文档）。参考事实源：
> `测校/一些测校仿真/运动台非正交与LUT补偿/`（stage_orthogonality_sim.py
> 算法在环仿真 + README）、`non_orthogonal_model_PCA.py`、
> `grating_and_interferometer_parallel_NO_error_PCA.py`（光栅+干涉仪原始
> 口径，测量端在本方案中按 **die 阵列格点跳测** 替换）、真机 SOP（重构后
> SOP：微动台像素轨迹法 / 粗动台 2×2 补偿矩阵）。
> 迭代 36 落地一阶 M 法 → 迭代 37 重写为**栅格跳测法**（基准尺=晶圆 die
> 阵列）→ 迭代 38 改**十字对称轨迹 + 节点级带符号残差输出**（LUT 建表
> 原材料就位）。LUT 的 2D 测量 routine 与补偿消费仍未实现，按本文执行。

---

## 1. 误差模型与"什么时候栅格跳测法不够"

一阶真值矩阵（X 基准轴约定，`stage_orthogonality_sim.py:93-125`）：

```
A = [[1+εx,        sinφ     ],
     [0,     (1+εy)·cosφ   ]]
```

- φ = XY 非正交角（µrad 级）——**栅格跳测法已测已补**（`stage_geo.*_nonortho_urad`，
  FK 注入键 `*_y_rail_dir_urad`）；
- εx/εy = 轴比例误差（ppm）——跳测法残差里的**线性斜坡**就是它；**迭代 42
  起已从节点链分离直测**（measured 增 `scale_x/scale_y`/`scale_ppm_x/y`，
  双向矩阵升级为全 2×2 实测像 M=[col_x|col_y]=各线步进向量/pitch，ε 即列
  模长；横向分量给角、沿轨分量给比例的弦模型分解），LUT 消费时 corr 表只
  承剩余高阶项；
- **高阶场**（一阶矩阵表达不了，是 LUT 的存在理由）：
  - 导轨直线度**弓形**（正弦，PV nm × 拱数）；
  - **偏摆耦合** ex = −ψ(x)·y（yaw ψ(x) 沿 X 正弦，µrad×mm=nm，阿贝型）；
- 干扰量：**晶圆上片旋转 ρ**（load_rz，处方默认 250µrad）——跳测法 X 线斜率
  方向已读出 ρ̂ 并在 φ 差中消去；LUT 生成时由 Kabsch 刚体拟合统一去除
  （§3），大倾角工况残留二阶项 ~φ·ρ·行程需留意。

**触发判据（跳测法节点残差诊断，迭代 38 起逐节点带符号输出）**：
X/Y 线法向残差对节点号做二次项 t 检验 t>4 且 PV>3σ，或常数补偿后独立
网格残差 RMS 仍超预算 → 建 LUT。

## 2. 测量方案（die 阵列 2D 格点跳测）

| 项 | 口径 |
|---|---|
| 基准尺 | 晶圆 die 阵列（wafer_sites 格点 + site_marks quad，**假定严格正交**，与非正交测校同一把尺）——真机对应"标准标记硅片/玻璃片" |
| 几何 | 与非正交测校同款：WS 台测=静止相机（AS 锚位）看 WS 带动的标记（σ=+1）；AS 台测=WS 静止看 AS 带相机反向扫（σ=−1，o=标记−相机，相对运动同式） |
| 轨迹 | **2D 栅格跳测**：非正交十字（两线×K 点）的推广——以中心 site 为原点，逐行 j：Y 走 j·pitch 后 X 线跳 K 点，共 G 行（K=G 为奇数，单调升序单侧逼近）。覆盖 = (G−1)×pitch 每轴（处方 pitch=10.005mm（2026-09-18 起 gap 默认 0.005）、G=9 → 80mm，旧 pitch=20mm 时 160mm；G 受晶圆 site 数与行程限位约束）；相位复现保证逐节点标记以同一相对角进 1.8mm 画幅（与非正交同款约束：起点须对准某 quad 角） |
| 每节点 | m 帧识别取质心均值（默认 m=3）；o_i=标记世界位−相机世界位（`_context` 链 solve_pixel，含 dir/微旋反投影）；σ_meas=RSS(相机σ, 到位重复性σ)/√m |
| 数据形态 | 逐节点记录 (q_cmd=(x_i,y_j), o_i, 节点像素, 循环号)——迭代 38 的 `stage_nonortho_*_nodes_*.csv` 即其 1D 两线特例（2026-09-16 起重构为行基准全量记录：[cycle,line,node,template,AS_X_mm,AS_Y_mm,WS_LS_X_mm,WS_LS_Y_mm,mark_col_px,mark_row_px,score,cam_world_x_mm,cam_world_y_mm,mark_world_x_mm,mark_world_y_mm,o_x_mm,o_y_mm,resid_x_mm,resid_y_mm]），2D routine 直接复用该记录格式 |

**网格密度权衡（关键设计式，参考 sim :250-265 与其诊断 :317-324）**：
LUT 残差 ≈ RSS( √2·0.667·σ/√m （节点噪声烘入）, 0.4·曲率·h²/8 （截断） )。
**噪声受限时加密网格反而更差**——pitch 被处方锁死（格点物理约束），实际
可调的是 G（覆盖）与 m（每节点帧数）：噪声项大就加 m，截断项大（曲率大）
才需要更密的基准尺（更小 pitch 的处方/真机换细光栅标准片）。

## 3. 生成方案

1. 汇总 2D 跳测节点：名义指令 q_cmd（轴位）与实测 o_i 反算的实际位置 q_meas；
2. **2D Kabsch 刚体拟合**去整体旋转/平移（上片 ρ + 锚位偏移；stdlib 闭式
   ——质心 + 2×2 协方差极分解，复用旋转三件套 `_fit_rot` 的数学件）→ R,t；
   **只去刚体**：比例斜坡（ε）与高阶场全部留在表里，不被拟合吸收；
3. 节点误差表 **corr[G,G,2]**（mm，规则名义网格上 q_meas−(q_cmd@R.T+t)）；
4. 存储：`runtime/stage_lut.json`（每台：gax[G] 轴 + corr 表 + R,t + 元数据
   {G,pitch,覆盖,m,σ_meas,ρ̂,生成时间,job_id}）；机器常数 stage_geo 组只存
   引用与开关（类比 vision_templates 存在性校验模式；纯数值字段模板放不下
   大表）。
5. 建表前提：**常数补偿先生效**（`*_y_rail_dir_urad` 已写回）——LUT 只吃
   常数补偿后的残余，与常数项正交叠加，不重复补偿。

## 4. 补偿施加层

- **主消费点 = EGA map 层**（`runtime/ega.py` `apply_map`）：LUT 本质是高阶
  EGA map。flow_engine 全部走位寻址（pi_measure/align_first/fl_ready/
  fl_measure/fa_measure/_fa_apply_comp）都经 `apply_map`——在 fit 六参数
  仿射基础上加节点残差表双线性前馈一处改动即全覆盖：
  `q_comp = affine(q_cmd) + bilinear(corr, q_cmd)`；逆换算处（世界→设计系）
  保持线性部 A⁻¹ 不动，残差表只做前馈不参与逆。
- **轴控下发兜底 = engine.move**：单轴调用拿不到 (x,y) 对，只适合施加单轴
  残余项（along-track）；XY 耦合项不放这层。
- **不碰测量/反算链**（solve_pixel/_context/视觉 identify）：补偿只动指令，
  不动测量——否则标定与验证失去真值基准。
- **真机**：补偿矩阵/LUT 下发控制器前馈（待上位机 Motion 层接口确认后映射；
  ACS 侧对应 "constant correction table"/运动学前馈机制，接口对齐后实现）。

## 5. 验证方案

- 独立 **V×V 验证网格**（V=21，**内缩半格防外插**），与标定网格不相交；
- 三向对比：未补偿 / 常数（仅 φ）/ 常数+LUT；
- 仿真内真值对照：LUT 表值与同 (x,y) 处 FK 真值（error_maps sample6 沿链
  合成 + y_rail_dir 注入）之差 = 补偿残余，可直接对拍；
- 验收口径：校准后残差 RMS 的 MC P97.5 上限（参考仿真解析预测交叉校验，
  比值应 ≈1.00）；
- 回归挂 tests/test_metrology.py（实现迭代排）。

## 6. 现状 / 下一步边界

- **已落地**：栅格跳测法非正交两 routine（迭代 37）；十字对称轨迹 + θ_y
  偏差口径 + **逐节点带符号残差 CSV/series**（迭代 38，LUT 原材料就位）；
  stage_geo 常数组（y_rail_dir 注入 + nonortho_urad 写回）；FK 导轨方向
  注入与补偿。
- **未做（下一步迭代）**：2D 栅格跳测测量 routine（G×G 逐行）、Kabsch 去
  刚体生成、stage_lut.json 存储与 stage_geo 引用键、ega.apply_map 的
  bilinear 前馈消费、大上片倾角 ρ 工况专项、εx/εy 拆分口径、真机标准片/
  光栅尺/干涉仪口径适配。
- 触发下一步的信号：非正交 routine 的节点残差二次项诊断持续显著（§1）。
