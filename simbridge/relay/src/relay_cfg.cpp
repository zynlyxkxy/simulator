#include "relay/relay_cfg.h"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace relay {

// ---------- 小工具 ----------
static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return s;
}

static std::vector<std::string> strVecOf(const Json& j) {
    std::vector<std::string> out;
    for (const auto& e : j.arr()) out.push_back(e.asString());
    return out;
}

// ---------- motion ----------
MotionCatalog loadMotionCatalog(const std::string& jsonText) {
    MotionCatalog c;
    Json root = Json::parse(jsonText);
    c.systemName = jstr(root, "systemName", "");
    auto parseStages = [](const Json& list, const char* type, std::vector<StageDef>& out) {
        for (const auto& s : list.arr()) {
            StageDef d;
            d.name = jstr(s, "name");
            d.stageType = type;
            if (s.at("axisNames").isArray()) d.axisNames = strVecOf(s.at("axisNames"));
            out.push_back(std::move(d));
        }
    };
    parseStages(root.at("acs_stages"), "acs", c.stages);
    parseStages(root.at("plc_stages"), "plc", c.stages);
    return c;
}

const StageDef* findStage(const MotionCatalog& c, const std::string& name) {
    for (const auto& s : c.stages) if (s.name == name) return &s;
    return nullptr;
}

// ---------- io 目录 ----------
// 28 plc_http 的气路命令全集(与 28 侧 AIR_OPS 一致)
static const char* kAirOps[] = {"OpenPositive", "OpenNegtive", "OpenAir"};
static bool isAirOp(const std::string& name) {
    for (const char* s : kAirOps) if (name == s) return true;
    return false;
}

// 主控 config 的 fullPath 带控制器前缀且用 '/' 分段,28 点位表从 SystemMachine 起且用 '.' 分段:
//   "3000_1_Controller/SystemMachine/SS_ME/M_AIR/WT_WaferVac" → "SystemMachine.SS_ME.M_AIR.WT_WaferVac"
static std::string toSimPath(const std::string& fullPath) {
    std::string s = fullPath;
    size_t slash = s.find('/');
    if (slash != std::string::npos &&
        s.substr(0, slash).find("Controller") != std::string::npos) {
        s = s.substr(slash + 1);
    }
    for (char& ch : s) if (ch == '/') ch = '.';
    return s;
}

// AI/DI/Setting/Buffer:观测点全路径 = 宿主路径 + "." + name
static IoVarDef makeReadPoint(const std::string& type, const std::string& fp, const Json& e) {
    IoVarDef d;
    d.id = (int)jnum(e, "id");
    d.ioType = type;
    d.label = jstr(e, "label", jstr(e, "name"));
    d.path = fp + "." + jstr(e, "name");
    d.readPath = d.path;
    d.probePath = d.path;
    // 28 侧没有 M_BS.oInBondingPos(搜力中反馈)这个点:值由桥接合成(见
    // io_relay_server 的 forceSearchWorker),这里只登记;supported 判定因 synthDi 放行。
    if (type == "DI" && jstr(e, "name") == "oInBondingPos") d.synthDi = true;
    return d;
}

// DO:三条翻译规则(与 28 plc_http 的命令路由一一对应)
static IoVarDef makeDoPoint(const std::string& fp, const Json& e) {
    IoVarDef d;
    std::string name = jstr(e, "name");
    d.id = (int)jnum(e, "id");
    d.ioType = "DO";
    d.label = jstr(e, "label", name);
    d.observer = jstr(e, "observer");
    if (isAirOp(name)) {
        // 气路:28 按 cmd ∈ {OpenPositive,OpenNegtive,OpenAir} + 回路路径路由
        d.path = fp;
        d.cmd = name;
        d.probePath = fp;
    } else if (fp.find(".E_SignalTower") != std::string::npos) {
        // 灯塔/蜂鸣器:28 按 path 前缀路由,SetBool 到具体色名/蜂鸣器
        d.path = fp;
        d.cmd = "SetBool";
        d.probePath = fp + ".UpdateMode";
    } else {
        // 相机/光源/ES 等叶名式点位:path 落到叶名,SetBool 触发
        d.path = fp + "." + name;
        d.cmd = "SetBool";
        d.probePath = d.path;
    }
    if (!d.observer.empty()) d.readPath = fp + "." + d.observer;

    // 28 侧没有 M_ES 的 IO 行为:ES_Start/ES_Stop 实际就是 28 面板上的"蓝膜上片/下片"按钮
    // (面板 index.html 的 wqFilmLoad/wqFilmUnload 发 /api/workpiece/film load|unload)。
    // 这两个点因此不走 PLC 命令面,由 io_relay_server 的 IOSetDo 直接调工件接口。
    if (name == "ES_Start") d.simHook = "film_load";
    else if (name == "ES_Stop") d.simHook = "film_unload";

    return d;
}

static IoVarDef makeAoPoint(const std::string& fp, const Json& e) {
    IoVarDef d;
    std::string name = jstr(e, "name");
    d.id = (int)jnum(e, "id");
    d.ioType = "AO";
    d.label = jstr(e, "label", name);
    d.observer = jstr(e, "observer");
    d.path = fp + "." + name;
    d.cmd = "SetValue";
    d.probePath = d.path;
    if (!d.observer.empty()) d.readPath = fp + "." + d.observer;
    // 搜力参数两条在 28 侧没有点位:28 的 ForceSearch 是"一次下发整组参数"的组合动作,
    // 所以桥接不改写模型点位,只把主控下发过的值缓存起来,收到 SearchBondPos 时
    // 组装成 ForceSearch 的 args(见 io_relay_server::IOSetAo / IOExecCmd)。
    if (name == "SetSearchForce") d.simHook = "search_force_set";
    else if (name == "SetSearchPosSpeed") d.simHook = "search_speed_set";
    return d;
}

static IoVarDef makeCmdPoint(const std::string& fp, const Json& e) {
    IoVarDef d;
    d.id = (int)jnum(e, "id");
    d.ioType = "Cmd";
    d.label = jstr(e, "label", jstr(e, "name"));
    d.path = fp;                                  // 命令宿主路径
    d.cmd = jstr(e, "name");                      // 28 命令名
    d.probePath = fp + "." + jstr(e, "name");     // 28 侧真实点位(命令点)
    // 28 侧没有 SearchBondPos/SearchBondPosDone 这两条命令,但有等价的独立搜力
    // 组合动作 ForceSearch/StopForceSearch(iter78 模型扩展点):
    //   SearchBondPos     → ForceSearch(从当前 BS_Arm_Z 下行,到力停轴并稳定确认,
    //                       观测 oForceSearchBusy/Done/Fail/Pos;停在接触位不动)
    //   SearchBondPosDone → StopForceSearch(主控搜位超时/取消时发)
    // 见 io_relay_server::IOExecCmd 与 forceSearchWorker。
    if (d.cmd == "SearchBondPos") d.simHook = "bond_force_search";
    else if (d.cmd == "SearchBondPosDone") d.simHook = "bond_force_search_stop";
    // StopForceManualBonding 在 28 侧也没有,但它必须"真的把力控关掉":
    // 力的 PID 在 force_mode 下按住 BS_Arm_Z,不卸力的话 PostBond+100 的抬臂会被顶住
    // → moveTimeOut(5s) 超时 → 报警"键合头移动失败"(PostBond 是无条件兜底发的这条命令)。
    // 翻译成 BondingByForceManual{Force:0} —— 模型的"受控卸力",sp=0 后自动退出力控。
    if (d.cmd == "StopForceManualBonding") d.simHook = "force_release";
    for (const auto& a : e.at("args").arr()) d.args.push_back(a.asString());
    return d;
}

IoCatalog defaultIoCatalog() {
    IoCatalog c;
    // 兜底:只登记相机触发两条(与主控枚举 6000/6001 对齐)
    Json e1 = Json::object();
    e1["id"] = Json::number(6000);
    e1["name"] = Json::string("AlignCameraTigger_1");
    e1["observer"] = Json::string("oAlignCameraTigger_1");
    Json e2 = Json::object();
    e2["id"] = Json::number(6001);
    e2["name"] = Json::string("AlignCameraTigger_2");
    e2["observer"] = Json::string("oAlignCameraTigger_2");
    c.do_.push_back(makeDoPoint("SystemMachine.SS_Bonding.M_AS", e1));
    c.do_.push_back(makeDoPoint("SystemMachine.SS_Bonding.M_AS", e2));
    return c;
}

static void loadOneIoConfig(const std::string& file, IoCatalog& c) {
    std::string text;
    try { text = readFile(file); } catch (...) { return; }
    Json root;
    try { root = Json::parse(text); } catch (...) { return; }
    for (const auto& mp : root.at("machineParts").arr()) {
        std::string fp = toSimPath(jstr(mp, "fullPath"));
        for (const auto& e : mp.at("AI").arr()) c.ai.push_back(makeReadPoint("AI", fp, e));
        for (const auto& e : mp.at("DI").arr()) c.di.push_back(makeReadPoint("DI", fp, e));
        for (const auto& e : mp.at("AO").arr()) c.ao.push_back(makeAoPoint(fp, e));
        for (const auto& e : mp.at("DO").arr()) c.do_.push_back(makeDoPoint(fp, e));
        for (const auto& e : mp.at("SettingDouble").arr()) c.settingD.push_back(makeReadPoint("SettingDouble", fp, e));
        for (const auto& e : mp.at("SettingBool").arr()) c.settingB.push_back(makeReadPoint("SettingBool", fp, e));
        for (const auto& e : mp.at("Buffer").arr()) c.buffer.push_back(makeReadPoint("Buffer", fp, e));
        for (const auto& e : mp.at("Cmd").arr()) c.cmd.push_back(makeCmdPoint(fp, e));
    }
}

IoCatalog loadIoCatalogFromConfigDir(const std::string& dir) {
    IoCatalog c;
    loadOneIoConfig(dir + "/plc/io_config.json", c);
    loadOneIoConfig(dir + "/acs/io_config.json", c);
    if (c.ai.empty() && c.di.empty() && c.ao.empty() && c.do_.empty() && c.cmd.empty()) {
        std::cout << "[relay-cfg] no io_config.json under " << dir
                  << ", fallback to default catalog" << std::endl;
        return defaultIoCatalog();
    }
    return c;
}

const IoVarDef* IoCatalog::find(int id) const {
    auto scan = [id](const std::vector<IoVarDef>& list) -> const IoVarDef* {
        for (const auto& v : list) if (v.id == id) return &v;
        return nullptr;
    };
    if (const IoVarDef* p = scan(ai)) return p;
    if (const IoVarDef* p = scan(di)) return p;
    if (const IoVarDef* p = scan(ao)) return p;
    if (const IoVarDef* p = scan(do_)) return p;
    if (const IoVarDef* p = scan(settingD)) return p;
    if (const IoVarDef* p = scan(settingB)) return p;
    if (const IoVarDef* p = scan(cmd)) return p;
    return scan(buffer);
}

// ---------- 台/轴 → 28 轴映射(内置种子) ----------
const std::vector<SimAxisMap>& builtinAxisMap() {
    static const std::vector<SimAxisMap> table = {
        // config1 硅片台(ACS 轴号完全一致:0/2/4)
        {"motion1", "WaferStage_XY", 0, "WS_LS_X", 1.0},
        {"motion1", "WaferStage_XY", 1, "WS_LS_Y", 1.0},
        {"motion1", "WaferStage_Rz", 0, "WS_Rz", 1.0},
        {"motion1", "EPIN_Z", 0, "E_EPIN_Z", 1.0},   // 别名整组(3 针同步)
        // config2 键合头(名不同:ArmHead_* ↔ 28 BS_*;已确认:真机 config2 路径已修正为名实相符)
        {"motion2", "ArmHead_Z", 0, "BS_Arm_Z", 1.0},
        {"motion2", "ArmHead_Rx", 0, "BS_CAL_Rx", 1.0},
        {"motion2", "ArmHead_Ry", 0, "BS_CAL_Ry", 1.0},
        // config3 对准(ACS 轴号一致:6/7/8/10)
        {"motion3", "AlignStage_XY", 0, "AS_X", 1.0},
        {"motion3", "AlignStage_XY", 1, "AS_Y", 1.0},
        {"motion3", "AlignStage_Z", 0, "AS_Z", 1.0},
        {"motion3", "PunchZ", 0, "ES_PunchZ", 1.0},
        // config4 芯片传输(FA_X ACS 9 一致;PLC 轴名相近;FA_Rz ↔ CH_Rz 已确认:真机 M_CH.Rz)
        {"motion4", "FA_X", 0, "FA_X", 1.0},
        {"motion4", "FA_Rx", 0, "FA_Rx", 1.0},
        {"motion4", "FA_Z", 0, "FA_Z", 1.0},
        {"motion4", "FA_Rz", 0, "CH_Rz", 1.0},
    };
    return table;
}

const SimAxisMap* findAxisMap(const std::string& module, const std::string& stage, int idx) {
    for (const auto& m : builtinAxisMap())
        if (m.module == module && m.stage == stage && m.idx == idx) return &m;
    return nullptr;
}

} // namespace relay
