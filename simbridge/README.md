# MctrlTo28Module — 1927 主控 ↔ 28 模型 中转层(relay)

在不修改 `1927project/` 与 `28module/` 源码的前提下,让 1927 主控层(mctrl/GUI)
**零改动、无感知**地控制 28 模型运动、拍照与共焦(cf)测高。

详细设计见同目录 `设计方案.md`。

## 目录结构(已精简)

```
1927project\simulator\simbridge\      ← 本中转层在 simulator 子仓库里
├─ relay\                  ← 全部源码(C++)
│  ├─ include\relay\*.h      (json/http/mqtt/28客户端/配置)
│  ├─ src\*.cpp              (motion×4 / io / bridge 服务端 + probe)
│  └─ CMakeLists.txt
├─ deploy\                 ← 运行包(脚本;bin 由 rebuild_bin.bat 生成,不入库)
│  ├─ start_relay.bat / stop_relay.bat
│  ├─ rebuild_bin.bat         (编译 + 从 1927project 拷贝 dll 重建 bin)
│  └─ prep_scene.py           (备场:晶圆上片 + 一颗芯片到键合头)
├─ README.md / 设计方案.md
└─ .gitignore              (build/ bin/ 不入库)
```

配置不再本地保存副本,启动时**只读引用 1927project 唯一配置源**(相对路径):
`..\..\..\config\mw_config.yaml` 与 `..\..\..\config\motion\configN\config.json`(relay 启动参数指向它们)。

## 构建与部署

前置:Visual Studio 2019(msvc x64)、cmake;`1927project` 根 = 本目录上三级(相对引用,
换机器不用改路径),且 `module\motion` / `module\io` 已编过(relay 链接它们
`output\interface\rpc\lib\release` 的 dll+import 库)。

```bat
deploy\rebuild_bin.bat     @echo 一键:cmake 编译 relay + 重建 deploy\bin(含运行 dll)
..\..\scripts\build_exe.bat simbridge   @echo 同上,从 1927project 顶层入口走
```

顶层 `scripts\build_exe.bat` 已把本中转层纳入编译流程:排在 motion 之后
(先 io→motion),无参数全量构建时自动带上;只支持 release(relay 链的是
platform lib 的 release 版),`debug` 全量构建时会跳过并打印一行 [skip]。
relay 在跑时 `deploy\bin` 被锁,`rebuild_bin.bat` 会明确报错——先
`simulator\simbridge\deploy\stop_relay.bat`(或 `exe\stop_all.bat`)再编。

## 运行

前置:28 模型已启动(28module\runtime\main.py,127.0.0.1:9010/9101/9102/1883;
建议用带 numpy 的 Python,相机预览依赖 numpy;REAL 模式另需 opencv)。

```bat
deploy\start_relay.bat     @echo 起 relay_motion×4 + relay_io + relay_bridge
                           (配置只读引用 ..\..\..\config 唯一源)
```
然后把 `start_all.bat` 里真机驱动 5 行(CTL_MDL_IO + CTL_MDL_Motion×4)停掉/替换,正常起
mctrl/GUI 即可。中转层替代的是这 5 个驱动进程的位置,主控代码与配置零改动。

## 已完成与验证(摘要)

- M1: 5 个 ISTAR RPC 服务端(端口/协议与真驱动一致)+ 心跳/Observer/GetServerInfo;
- M2: motion Enable/MoveAbs/Rel/Home/Stop 转发;到位=引擎标志 + **自比对
  |当前−目标|≤0.01(请求 tolerances 优先)**才推 CmdCompleted;
- M3: io 相机触发(6009/6010→28 AlignCameraTigger_1/2);视觉结果 1:1 翻译回 VSS_Com;
- M4: cf 测量(CTL_WM_CF_ControlManager→28 confocal→CTL_DR_CF_FLS);
- 搜力: 主控 `SearchBondPos`(10519)/`SearchBondPosDone`(10520) → 28 独立搜力组合动作
  `M_BS.ForceSearch` / `StopForceSearch`;参数取主控 `SetSearchForce`/`SetSearchPosSpeed`
  (4000/4001,28 侧无点位,桥接缓存);DI `oInBondingPos`(2506) 由桥接按
  `oForceSearchBusy/Done/Fail` 合成。28 搜力停在接触位,后面 `BondingByForceManual` 直接接力;
- 破真空落片: 键合真空 `OpenAir`/`OpenPositive` 直接转发,28 侧按释放瞬间实际位姿垂直落片;
- bridge 每 1s 向 VSS_Com / CTL_DR_CF_FLS 发空心跳(主控据此判 vision/cf 在线);
- 端到端:真 mctrl+GUI 全栈移动/回环验收通过。

## 依赖的 28 模型侧改动(已登记其 开发交接.md)

`后端模型\runtime\mqtt_broker.py` 补齐 QoS2(PUBLISH→PUBREC→暂存→PUBREL→转发→PUBCOMP);
主控平台 MQTT 客户端按 QoS2 发布,原 mini broker 不支持会断连。

## 备注

- 中转层只负责**拉起自己的 6 个进程**(start_relay.bat);主控(mctrl/GUI/EH/MC/RP/RM)
  由你们从 `1927project\exe\start_all.bat` 正常启动(把其中 5 行真机驱动换掉即可),
  relay 不需要也不代为启动主控。
- `master/`(1927project/exe 拷贝)与 `sim28/`(28 后端拷贝)已从本仓库移除;
  它们仅在我需要**隔离自测/验收**时临时使用,日常运行不需要。
- 轴映射(config2 键合头 ArmHead_*↔BS_*、config4 FA_Rz↔CH_Rz 等)为推断种子,
  见设计方案 §7,联调时请主控/28 同事确认。
