// relay_cfg.h - 配置加载(纯 stdlib)
// 1) 主控 motion configN/config.json 的 stage 目录(GetServerInfo 素材)
// 2) io 变量目录(由主控唯一配置源 config/io/{plc,acs}/io_config.json 构建)
#pragma once
#include <string>
#include <vector>
#include <memory>
#include "relay/relay_json.h"

namespace relay {

struct StageDef {
    std::string name;
    std::string stageType;             // "acs" / "plc"
    std::vector<std::string> axisNames;
};

// motion 模块目录(configN/config.json 的 acs_stages/plc_stages)
struct MotionCatalog {
    std::string systemName;
    std::vector<StageDef> stages;
};

// ===== 台/轴 → 28 模型轴 映射 =====
struct SimAxisMap {
    std::string module;    // motion1..4
    std::string stage;     // 主控 stage 名
    int idx = 0;           // 该 stage 内轴序(0-based,与 axisNames 对齐)
    std::string simAxis;   // 28 模型轴名(或别名,如 E_EPIN_Z)
    double unitScale = 1.0;
};

// 内置种子映射(依据两侧 ACS 轴号/物理位置核对;存疑项已注释)
const std::vector<SimAxisMap>& builtinAxisMap();
// 查 module+stage+idx 的映射;找不到返回 nullptr
const SimAxisMap* findAxisMap(const std::string& module, const std::string& stage, int idx);

// ===== io 变量目录 =====
// 一条 = io_define.h 的一个 IoVarInfo/IoDoVarInfo/IoCmdVarInfo 条目,
// 外加它到 28 模型 PLC 命令/观测面的翻译信息(见 relay_cfg.cpp 的推导规则)。
struct IoVarDef {
    int id = 0;
    std::string ioType;                // AI/DI/AO/DO/SettingDouble/SettingBool/Cmd/Buffer
    std::string label;                 // 显示名(GetServerInfo 报给主控的 name)
    std::string observer;              // 主控侧绑定的观测名(DO/AO 有)
    std::string path;                  // 28 侧命令/观测目标路径(点号分隔,已去控制器前缀)
    std::string cmd;                   // 28 命令名(SetBool/OpenNegtive/ES_Start/...;空=只读点)
    std::string readPath;              // 28 侧回读路径(Observer 取真实值用;可空)
    std::string probePath;             // 28 点位表里判定"该点是否已建模"的键
    std::string simHook;               // 特殊动作钩子:28 侧没有对应 IO、实际是面板按钮/组合动作的点
                                       // (目前 "film_load"/"film_unload"=蓝膜上下片、
                                       //  "search_force_set"/"search_speed_set"=搜力参数(只缓存)、
                                       //  "bond_force_search"/"bond_force_search_stop"=搜力起停、
                                       //  "force_release"=受控卸力)
    bool synthDi = false;              // 28 侧没有这个反馈点,值由桥接自己合成
                                       // (目前只有 M_BS.oInBondingPos=搜力中标志)
    std::vector<std::string> args;     // Cmd 参数名(与 ExecCmd 的 values 顺序对应)
    bool supported = true;             // 28 侧是否存在(按点位快照刷新;false → 读写回 errorCode=1)
};

struct IoCatalog {
    std::vector<IoVarDef> ai, di, ao, do_, settingD, settingB, cmd, buffer;

    // 全表按 id 查找;找不到返回 nullptr
    const IoVarDef* find(int id) const;
};

// 从 <ioConfigDir>/{plc,acs}/io_config.json 构建目录(读不到时回 defaultIoCatalog)
IoCatalog loadIoCatalogFromConfigDir(const std::string& ioConfigDir);

// 兜底:只登记相机触发两条(主控枚举 6000/6001)
IoCatalog defaultIoCatalog();

// 从 configN/config.json 文本加载 motion 目录
MotionCatalog loadMotionCatalog(const std::string& jsonText);

// 便捷:查找
const StageDef* findStage(const MotionCatalog& c, const std::string& name);

} // namespace relay
