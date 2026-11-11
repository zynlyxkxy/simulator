# Cordis MCS 协议 V1（PLC HTTP 通讯报文规范）

> 状态：**V1（依据同事模板工程代码还原，待真机联调核对）**
> 日期：2026-08-19
> 事实来源（按置信度排序）：
> 1. **代码实证**：`部分接口信息/io_template/io_template/mainproject/plc_module/PlcClient/PlcClient.cpp`（+ motionstage_template 同名文件，两者报文完全一致）——URL、信封、五+1 个服务的请求/响应字段均逐行还原自该文件；
> 2. **代码实证**：`PlcIoModule.cpp`（addPath 自动归类规则）、`motionstage_template/KNOWLEDGE.md`（machinePart 三字段约束、Setting 走 getSettings/setCurrentValueForSetting）；
> 3. **配置实证**：`部分接口信息/motion/motion/config1~4/plc/*.json`（stage fullPath / 命令绑定 / 观测绑定）、`io_template/config/plc/io_config.json`（真空命令面）；
> 4. **推断**（文中以 ⚠推断 标注）：响应 FAULT 的内部字段、getSettings 的请求 schema、machinePart.internalPath 的取值规则、executeCommand 成功时 value 的内容。这些在客户端代码里没有被消费（PlcClient 只判"非 null"），真机行为需联调确认。

## 1. 连接与传输

| 项 | 值 | 来源 |
|---|---|---|
| 方法 | HTTP POST | 代码实证 |
| URL | `http://<ip>:<port>/JsonServices/com.cordis.explorer.connectors.MachinePart.MachinePartConnector/<ServiceName>` | 代码实证（`MCS_SERVICE_PATH` 常量） |
| 真机 port | **90**（config1~4 plc/*.json） | 配置实证 |
| Content-Type | `application/json` | 代码实证 |
| 客户端超时 | 2000ms（`HTTP_TIMEOUT_MS`），超时 abort 并视为失败 | 代码实证 |
| 仿真适配层 | `http://127.0.0.1:9190` + 同一 MCS_SERVICE_PATH（runtime/mcs_http.py） | 本仿真 |

服务名直接拼在路径尾部，共 6 个（客户端实际发出的）：

`getMachinePartProperties` / `executeCommand` / `getObserverValuesFromObserverPaths` / `setCurrentValueForSetting` / `saveCurrentValueForSetting` / `getSettings`

> 注：交接文档 §7 Q4 原口径"5 服务（含 getSettings）"；代码实证显示 **io_template 客户端实际发的是 `saveCurrentValueForSetting`**（set 之后紧跟一次 save），`getSettings` 只在 motionstage_template/KNOWLEDGE.md 文字里出现（"Setting 走 getSettings / setCurrentValueForSetting"），两个模板的 PlcClient 都未实现它。仿真侧 6 个全部实现。

## 2. 请求/响应信封（所有服务共用）

### 2.1 请求体（JSON 数组，固定两元素）

```json
[
  {"SecurityToken": "ADMIN"},
  {"InputArgs": [ <服务专属参数对象> ]}
]
```

- 元素 0：安全令牌对象，键名 `SecurityToken`，值恒为字符串 `"ADMIN"`（代码实证；仿真侧不校验）。
- 元素 1：对象，键名 `InputArgs`，值为数组；数组内容因服务而异（见 §3）。
- 序列化为 Compact JSON（无空白），UTF-8。

### 2.2 响应体（JSON 对象）

```json
{"type": "SUCCESS", "value": <服务专属>}
```

- `type`：`"SUCCESS"` 或 `"FAULT"`（代码实证只判断这两种；SUCCESS 之外按 FAULT 处理的只有显式 `"FAULT"` 串）。
- `value`：服务专属载荷。客户端对 executeCommand / setCurrentValueForSetting / saveCurrentValueForSetting **只判 value 非 null**，不看内容。
- FAULT 时客户端整体视为失败（`sendRequest` 返回 undefined）。⚠推断：FAULT 的完整形态按 JSON-RPC 惯例还应有错误描述字段，真机字段名未知；仿真侧返回 `{"type":"FAULT","value":null,"error":{"code":<字符串码>,"message":<中文描述>}}`，HTTP 状态仍 200（客户端只解析 body，不看 HTTP 码——QNetworkReply finished 即读 body）。

### 2.3 客户端失败语义（代码实证，联调上位机时的预期行为）

| 情形 | 客户端行为 |
|---|---|
| HTTP 超时（2s） | abort，返回失败；日志打 `[PLC RSP] TIMEOUT` |
| 响应体为空 | 返回失败；`[PLC RSP] EMPTY` |
| JSON 解析失败 | 返回失败；`[PLC RSP] PARSE ERROR` |
| `type=="FAULT"` | 返回失败；`[PLC RSP] FAULT` |
| 成功 | 取 `value` 返回 |

## 3. 五个+一个服务逐字段 schema

### 3.1 getMachinePartProperties — 取节点模板

请求 InputArgs（1 个元素）：

```json
[{"machinePartFullPath": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z"}]
```

| 字段 | 类型 | 说明 |
|---|---|---|
| machinePartFullPath | string | 节点全路径（fullPath，分隔符 `/` 与 `.` 混用，见 §4） |

响应 value（对象；客户端逐一消费以下四键）：

```json
{
  "machinePart": {
    "fullPath": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z",
    "internalPath": "SystemMachine/SS_Bonding/M_WS/E_EPIN_Z",
    "name": "E_EPIN_Z"
  },
  "commands":  [ {"name": "MoveAbs", "arguments": [ <argument 模板> ]}, ... ],
  "settings":  [ {"name": "MoveVelocity", "type": "FLOAT_TYPED_VALUE", ...}, ... ],
  "observers": [ {"name": "oCurrentPos_GC", "type": "FLOAT_OBSERVER"}, ... ]
}
```

- `machinePart`：**必须三字段齐备** `fullPath` / `internalPath` / `name`（KNOWLEDGE.md 明示"必须包含完整的三字段"，与 PLC Driver 严格一致）。`name` 为节点末段名。⚠推断：`internalPath` 为控制器内部路径（去掉 `3000_1_Controller/` 前缀）；客户端只把它原样回传给 executeCommand，不解析内容。
- `commands[]`：命令模板。字段：`name`（命令名）、`arguments`（参数模板数组，可空）。客户端只消费这两键；真机可能还有更多字段（KNOWLEDGE.md："命令 arguments 必须包含 JSON 里的所有原始字段"——即回传时不得裁剪模板字段）。
- argument 模板（executeCommand 回传时逐字段保留 + 填值）：⚠推断 至少含 `name` / `type`；客户端回传时**就地添加** `currentValue` 与 `savedValue`（同值，见 3.2）。`type` 判定规则（PlcIoModule.cpp addPath，代码实证）：含 `"FLOAT"` → AO 类；含 `"BOOLEAN"` → DO 类。模板里出现的具体串推定为 `FLOAT_TYPED_VALUE` / `BOOLEAN_TYPED_VALUE`（与 setting 的 type 命名一族）。
- `settings[]`：字段 `name` + `type`；type ∈ `FLOAT_TYPED_VALUE` / `INT_TYPED_VALUE` / `BOOLEAN_TYPED_VALUE`（代码实证，addPath 归类规则：BOOLEAN 单独一类，FLOAT/INT 同归数值类）。
- `observers[]`：字段 `name` + `type`；type ∈ `FLOAT_OBSERVER`（AI）/ `BOOLEAN_OBSERVER`（DI）（代码实证，addPath 按 `type.contains("FLOAT"/"BOOLEAN")` 归类）。
- 客户端判空规则：value 非对象 → 失败；`machinePart` 为空对象 → 失败。

### 3.2 executeCommand — 执行命令

客户端流程（代码实证）：从 3.1 缓存的命令模板取 `arguments`，**参数个数必须等于实参个数否则本地报错不发请求**；然后对每个 argument 模板就地填 `currentValue` 与 `savedValue`（均为实参值，double；bool 也转成 1.0/0.0 发）。

请求 InputArgs（1 个元素）：

```json
[{
  "command": {
    "name": "OpenNegative",
    "machinePart": {
      "fullPath": "3000_1_Controller/SystemMachine/SS_ME/M_MachineAirCabinet/BH_Chip_YN_XN_SuckerVac",
      "internalPath": "SystemMachine/SS_ME/M_MachineAirCabinet/BH_Chip_YN_XN_SuckerVac",
      "name": "BH_Chip_YN_XN_SuckerVac"
    },
    "arguments": [
      {"name": "Value", "type": "FLOAT_TYPED_VALUE", "currentValue": 12.5, "savedValue": 12.5}
    ]
  }
}]
```

| 字段 | 类型 | 说明 |
|---|---|---|
| command.name | string | 命令名（来自模板） |
| command.machinePart | object | 3.1 返回的 machinePart 原样回传（三字段齐备） |
| command.arguments | array | 模板 arguments 原样 + 每项填 `currentValue`/`savedValue`（double） |

响应 value：⚠推断 内容不重要（客户端只判非 null；LOCAL_DEBUG 桩返回空数组 `[]`）。仿真侧返回 `[]`。失败（超时/空/FAULT）→ 客户端 `executeCommand` 返回 1，成功返回 0。

### 3.3 getObserverValuesFromObserverPaths — 批量读观测

请求 InputArgs（1 个元素）：

```json
[{
  "observerPaths": [
    {"path": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z/oCurrentPos_GC"},
    {"path": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z/oInPos"}
  ]
}]
```

- `observerPaths[]` 每项一个对象，键 `path` = `<machinePartFullPath>/<observer名>`（代码实证：`mpFullPath + "/" + name` 拼接）。

响应 value（数组，**与请求同序同长**）：

```json
[{"currentValue": 12.345}, {"currentValue": true}]
```

- 每项对象至少含 `currentValue`；FLOAT_OBSERVER 为数值，BOOLEAN_OBSERVER 为 bool。
- 客户端只取每项的 `currentValue` 按键序消费；响应不是数组 → 整体失败。

### 3.4 getSettings — 读设置（⚠推断）

两个模板的 PlcClient 均未实现此服务，仅 KNOWLEDGE.md 提及"Setting 走 getSettings / setCurrentValueForSetting"。仿真侧按 3.1 同构实现（**schema 为推断，待真机核对**）：

请求 InputArgs（推断）：

```json
[{"machinePartFullPath": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z"}]
```

响应 value（推断）：settings 数组，每项在模板字段基础上带当前值：

```json
[{"name": "MoveVelocity", "type": "FLOAT_TYPED_VALUE", "floatValue": 25.0}]
```

### 3.5 setCurrentValueForSetting — 写设置当前值

请求 InputArgs（1 个元素）：

```json
[{
  "settingKey": {
    "machinePartFullPath": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z",
    "commandName": "",
    "settingName": "MoveVelocity"
  },
  "value": {
    "dummyKey": "",
    "type": "FLOAT_TYPED_VALUE",
    "floatValue": 10.0
  }
}]
```

| 字段 | 说明 |
|---|---|
| settingKey.machinePartFullPath | 节点 fullPath |
| settingKey.commandName | 恒为 `""`（代码实证；疑似供"命令级 setting"预留） |
| settingKey.settingName | 设置名 |
| value.dummyKey | 恒为 `""`（代码实证，占位键） |
| value.type | `FLOAT_TYPED_VALUE` / `INT_TYPED_VALUE` / `BOOLEAN_TYPED_VALUE` |
| value.floatValue / intValue / booleanValue | 随 type 带值键（代码实证：客户端把 typedValue 除 type 外的字段原样复制；setSettingDouble 发 `floatValue`，setSettingBool 发 `booleanValue`） |

响应 value：客户端只判非 null（LOCAL_DEBUG 桩为 `[]`）。

### 3.6 saveCurrentValueForSetting — 保存设置（持久化）

io_template 客户端在每次 set 成功后紧跟调用（代码实证）。请求 InputArgs：

```json
[{
  "settingKey": {
    "machinePartFullPath": "3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z",
    "settingName": "MoveVelocity"
  }
}]
```

（注意：比 set 少了 `commandName` 键。）响应 value 同样只判非 null。motionstage_template 的客户端不调用 save（只 set）。

## 4. machinePart fullPath 形态（重要）

- 前缀恒为 `3000_1_Controller/`（**源自同事 3000 硅片台项目模板，1928 D2W 机台真机前缀待复核**；配置实证：motion config1~4 与 io_config.json 全部沿用此前缀）。
- **分隔符 `/` 与 `.` 混用**（配置实证）：
  - io_config.json 全用 `/`：`3000_1_Controller/SystemMachine/SS_ME/M_MachineAirCabinet/BH_Chip_YN_XN_SuckerVac`
  - motion plc stage 配置首段 `/` 后全用 `.`：`3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z`
- 观测 path = fullPath + `/` + observer 名（永远用 `/` 拼，代码实证）。
- 仿真适配层（9190）对 fullPath 做 `/`→`.` 归一化后查映射表（宽容行为，⚠推断真机不做归一化，联调时以真机为准）。

### 同事模板中的路径轮转错位（配置实证，已按文件名归位，待同事确认）

motion config2/config4 的 plc stage JSON 里 `machinePartFullPath` 与文件名轮转错位一格：

| 文件名 | 文件内写的 fullPath 末段 | 归位后应采用 |
|---|---|---|
| config2/plc/Arm_Z.json | `M_BS.E_CAL.Ry` | `M_BS.Arm_Z` |
| config2/plc/CAL_Rx.json | `M_BS.Arm_Z` | `M_BS.E_CAL.Rx` |
| config2/plc/CAL_Ry.json | `M_BS.E_CAL.Rx` | `M_BS.E_CAL.Ry` |
| config4/plc/FA_Rx.json | `SS_CH.M_CH.Rz` | `SS_CH.M_FA.Rx` |
| config4/plc/FA_Rz.json | `SS_CH.M_FA.Rx` | `SS_CH.M_CH.Rz` |
| config4/plc/FA_Z.json | `SS_CH.M_FA.Z` | `SS_CH.M_FA.Z`（仅此条不错位） |

归位后与点位表口径（接口规范_V1.md §3.4：`M_BS.Arm_Z` / `M_BS.E_CAL.Rx/Ry` / `M_FA.Rx` / `M_FA.Z` / `M_CH.Rz`）完全一致，故判定为复制粘贴错位而非有意映射。

## 5. 命令/观测归类规则（PlcIoModule.addPath，代码实证）

上位机拿到节点模板后的自动归类（决定它怎么用这棵树）：

- command：`arguments` 为空 → 无参命令（cmd）；1 个参数且 type 含 `FLOAT` → AO；1 个参数且含 `BOOLEAN` → DO（DO 实参也按 double 1.0/0.0 发）。
- observer：type 含 `FLOAT` → AI；含 `BOOLEAN` → DI。
- setting：`BOOLEAN_TYPED_VALUE` → bool 设置；`FLOAT/INT_TYPED_VALUE` → 数值设置。
- 观测读取有缓存轮询（周期可配，io 侧默认由 IoController 注册后统一轮询；motion 侧 observerPeriodMs=500），`forceRefresh=true` 时绕过缓存直发 HTTP。

## 6. 真机节点模板实例（配置实证）

### 6.1 真空吸盘节点（io_config.json）

```
3000_1_Controller/SystemMachine/SS_ME/M_MachineAirCabinet/BH_Chip_YN_XN_SuckerVac
```

- commands：`Close` / `OpenNegative` / `OpenPositive`（均无参）
- observers：`AirRouteClose` / `AirRouteNegative` / `AirRoutePositive`（BOOLEAN_OBSERVER；io_config 以 `observer` 键给出 DO↔观测绑定）
- 同族节点：`BH_Chip_YP_XN_SuckerVac`、`FA_ChipSuckerVacuum`
- 气控箱级 DI（节点 `.../M_MachineAirCabinet`）：`oYN_XN_WithChip` / `oYP_XN_WithChip` / `oFA_WithChip` / `oWT_WithWafer`
- 顶层 DI（节点 `3000_1_Controller/SystemMachine`）：`iPhoto_Finish_First`

### 6.2 PLC 运动 stage 节点（motion config1~4 plc/*.json）

以 `3000_1_Controller/SystemMachine.SS_Bonding.M_WS.E_EPIN_Z` 为例，命令绑定（cmd_bindings，固定 API 名 → MCS 命令名，空串=不支持）：

| 固定 API | MCS 命令 |
|---|---|
| Enable / Disable | Enable / Disable |
| Home | Home |
| MoveAbs / MoveRel | MoveAbs / MoveRel |
| Stop | **Halt** |
| ResetError | **Reset Error**（带空格） |
| SetVelAcc | **SetMoveParam** |
| Jog / SetDec / SetJerk / SetKillDec | （空=不支持） |

观测绑定（obs_bindings）：

| 固定 API | MCS observer |
|---|---|
| Pos | `oCurrentPos_GC` |
| Vel | `obMoveSpd`（注意是 b 不是 o） |
| Acc | （空=不支持） |
| Enable | `oMotorEnabled` |
| Moving | `oMoving` |
| InPos | `oInPos` |

Setting 实例（KNOWLEDGE.md）：`CommandTimeOut`（get/set_CommandTimeOut）。

### 6.3 3000 项目遗留节点（io_config.json 有、1928 机无对应物）

- `.../SS_Bonding/M_WS_STAGE/Interferometer`：AI `oUI_Xg/oUI_Yg/oUI_Rxg/oUI_Ryg/oUI_Rzg`（干涉仪五分量，仿真无对应物）
- `.../SS_Bonding/M_WS_STAGE/E_WS_STAGE_Horizontal`：AO `X_Move/Y_Move/Rz_Move` + 观测 `oX_TargetPosition/oY_TargetPosition/oRz_TargetPosition`（3000 项目晶圆台水平向走 PLC；1928 的 WS_LS 走 ACS 直驱）

## 7. 仿真适配层行为（9190，runtime/mcs_http.py + mcs_map.json）

- 端口 9190，与语义层 9102 并存互不影响；URL 路径与真机相同（MCS_SERVICE_PATH + 服务名）。
- 节点库 = `runtime/mcs_map.json`（每条注明来源文件；fullPath 归一化 `/`→`.` 后匹配）。
- 未注册 fullPath → FAULT `PathNotFound`；注册了但标 `unmapped`（真机侧无对应物）→ FAULT `Unmapped`；节点内未知命令/观测/设置 → FAULT `CmdNotFound`/`ObserverNotFound`/`SettingNotFound`；setting 类型不符 → FAULT `TypeMismatch`；引擎联锁拒绝 → FAULT `Interlock`。
- executeCommand 翻译：真空三命令 → engine.air_command；stage 命令 → engine.move/home/stop/enable/reset_error/set_axis_config（SetMoveParam→vmax/amax，不持久化）。
- **W2（2026-08-19）使能语义收紧**：stage `Disable` 落到引擎使能状态机——静止仅去使能；**运动中掉使能 = 自由减速（3×amax 合成减速度）到静止后进 fault 态**（真机掉使能即报警）；`Reset Error` 从全局 reset 改为**按 stage 复位**（engine.reset_error，清 fault 与回零 fault 态，不再动全轴/estop）；对 disabled/fault 轴的 MoveAbs/MoveRel/Home 返回 FAULT `Interlock`（message "axis not enabled"/"axis fault"）。`oMotorEnabled` 观测对应引擎 enabled 运行时标志（默认 True，不持久化）。9190 整机沉默/断线可用 `POST 9010 /api/fault`（target "mcs"）注入：沉默期所有请求挂起（60s 兜底 503），断线期端口拒连。
- 观测读取：轴观测读 engine 轴状态；真空 AirRoute* 读点位表 `M_AIR.<回路>.oAirState/oNegtiveState/oPositiveState`（继承 PLC 扫描周期语义）；WithChip/WithWafer 读 workpieces 层。
- setting 映射：`MoveVelocity→vmax` / `MoveAcc→amax` / `CommandTimeOut→内存暂存`（最小集；saveCurrentValueForSetting 对绑轴参数的设置做 persist=True 落盘）。
- HTTP 层异常（非 JSON / 信封结构错）→ HTTP 400 + FAULT body；服务内错误一律 HTTP 200 + FAULT body（客户端只看 body）。
