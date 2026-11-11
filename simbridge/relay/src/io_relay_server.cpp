#include "io_relay_server.h"
#include "io_define.h"
#include "relay/relay_util.h"
#include <algorithm>
#include <cstdio>
#include <iostream>

using namespace ::IOModule;
using namespace ::ISTAR;

namespace relay {

// 响应 errorCode:0=成功 1=查无此变量 2=读失败 3=写失败(与真 io 服务端一致)
static const int kErrNone  = 0;
static const int kErrNoVar = 1;
static const int kErrRead  = 2;
static const int kErrWrite = 3;
// SetCompleted.errorCode:0=通过 1=超时 2=下发失败(见 ioc_data.h 注释)
static const int kSetPass       = 0;
static const int kSetSendFailed = 2;

static void ilog(const char* tag, const std::string& msg) {
    std::cout << "[" << timeStr(nowMs()) << "][io-relay][" << tag << "] " << msg << std::endl;
}

static bool jsonToBool(const Json& j) {
    if (j.isBool()) return j.asBool();
    if (j.isNumber()) return j.asNumber() != 0.0;
    if (j.isString()) { const std::string& s = j.asString(); return s == "true" || s == "1"; }
    return false;
}

static double jsonToNum(const Json& j) {
    if (j.isNumber()) return j.asNumber();
    if (j.isBool()) return j.asBool() ? 1.0 : 0.0;
    return 0.0;
}

IoRelayServer::IoRelayServer(IoCatalog* catalog, SimClient* sim)
    : catalog_(catalog), sim_(sim) {}

IoRelayServer::~IoRelayServer() {
    stop_ = true;
    // 搜力轮询线程也要等:它每 500ms 查一次 stop_,正常 ~1s 内退出
    // (只读点位缓存,不做 HTTP,不会被卡住;见 forceSearchWorker)
    if (forceSearchThread_.joinable()) forceSearchThread_.join();
    if (timerThread_.joinable()) timerThread_.join();
}

void IoRelayServer::startTimers() {
    refreshIoCache();   // 同步探测一次:GetServerInfo 一开始就能报出 28 侧真实支持面
    stop_ = false;
    timerThread_ = std::thread([this] { timerLoop(); });
}

void IoRelayServer::timerLoop() {
    while (!stop_) {
        long long now = nowMs();
        if (now - lastHeartbeat_ >= 1000) {
            lastHeartbeat_ = now;
            IOStatusUploadPublish();   // 平台无参心跳:1s 一发,与真 io 服务端一致
        }
        if (now - lastObserver_ >= 500) {
            lastObserver_ = now;
            refreshIoCache();
            publishObserver();
        }
        sleepMs(50);
    }
}

// 拉 28 全量点位快照(GET /api/io),据此判定每个目录项在 28 侧是否已建模
void IoRelayServer::refreshIoCache() {
    if (!sim_ || !sim_->running()) return;
    Json j = sim_->ioSnapshot();
    if (!j.isObject() || !j.at("points").isObject()) return;

    std::lock_guard<std::mutex> lk(mu_);
    pointCache_.clear();
    for (const auto& p : j.at("points").obj()) pointCache_[p.first] = p.second;
    cacheValid_ = true;

    // probePath 不在点位表里 = 28 侧没建模这个点(键合工艺那批),读写一律回 errorCode=1。
    // synthDi 的点是故意"没建模"、由桥接合成值的,所以直接算支持。
    auto mark = [this](std::vector<IoVarDef>& list) {
        for (auto& v : list) v.supported = v.synthDi || pointCache_.count(v.probePath) > 0;
    };
    mark(catalog_->ai);
    mark(catalog_->di);
    mark(catalog_->ao);
    mark(catalog_->do_);
    mark(catalog_->settingD);
    mark(catalog_->settingB);
    mark(catalog_->cmd);
    mark(catalog_->buffer);

    if (!probed_) {
        probed_ = true;
        std::string miss;
        auto collect = [&miss](const std::vector<IoVarDef>& list) {
            for (const auto& v : list)
                if (!v.supported && !v.synthDi) miss += std::to_string(v.id) + " ";
        };
        collect(catalog_->ai);
        collect(catalog_->di);
        collect(catalog_->ao);
        collect(catalog_->do_);
        collect(catalog_->cmd);
        if (!miss.empty()) {
            ilog("probe", "28 side has no point for varId(s): " + miss +
                          "→ their read/write replies errorCode=1" +
                          " (hooked/synthesized points excluded: 10519/10520->ForceSearch,"
                          " 10518->force_release, 4000/4001->cached, 2506->synth)");
        }
    }
}

// 用 28 的真实值推 Observer,让主控 GUI 显示正确数值
void IoRelayServer::publishObserver() {
    IoObserverData od;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!cacheValid_) return;
        auto numOf = [this](const std::string& path, double& out) {
            auto it = pointCache_.find(path);
            if (it == pointCache_.end()) return false;
            out = jsonToNum(it->second);
            return true;
        };
        auto boolOf = [this](const std::string& path, bool& out) {
            auto it = pointCache_.find(path);
            if (it == pointCache_.end()) return false;
            out = jsonToBool(it->second);
            return true;
        };
        for (const auto& v : catalog_->ai) {
            double val = 0.0;
            if (v.supported && numOf(v.readPath, val)) {
                ObserverDoubleEntry e; e.varId = v.id; e.value = val;
                od.plc.AI.push_back(e);
            }
        }
        for (const auto& v : catalog_->di) {
            bool val = false;
            if (v.synthDi) {
                // 合成 DI:28 侧没有这个反馈点,值由桥接自己维护(搜力中标志)
                val = diInBondingPos_.load();
            } else if (!(v.supported && boolOf(v.readPath, val))) {
                continue;
            }
            ObserverBoolEntry e; e.varId = v.id; e.value = val;
            od.plc.DI.push_back(e);
        }
        for (const auto& v : catalog_->ao) {
            double val = 0.0;
            if (v.supported && numOf(v.readPath, val)) {
                ObserverDoubleEntry e; e.varId = v.id; e.value = val;
                od.plc.AO.push_back(e);
            }
        }
        for (const auto& v : catalog_->do_) {
            bool val = false;
            if (v.supported && boolOf(v.readPath, val)) {
                ObserverBoolEntry e; e.varId = v.id; e.value = val;
                od.plc.DO.push_back(e);
            }
        }
        for (const auto& v : catalog_->buffer) {
            bool val = false;
            if (v.supported && boolOf(v.readPath, val)) {
                ObserverBoolEntry e; e.varId = v.id; e.value = val;
                od.acs.Buf.push_back(e);
            }
        }
    }
    IOObserverDataPublish(od);
}

// ---------------------------------------------------------------------------
//  搜力(28 侧没有 SearchBondPos 命令,也没有 oInBondingPos 反馈点)
//
//  主控那段的协议是"下发 10519 开始搜力 → 轮询 DI 2506,等它回落 = 搜力完成"。
//  28 侧等价的专用接口是独立搜力组合动作(iter78 模型扩展点):
//    POST /api/io/command {"path":"SystemMachine.SS_Bonding.M_BS",
//                          "cmd":"ForceSearch","args":{Force,Speed,...}}
//  引擎在 1kHz 里从当前 BS_Arm_Z 下行,到力停轴,50ms 平均力稳定确认后锁存
//  oForceSearchDone(期间 oForceSearchBusy=true)。它停在接触位不动 —— 正好是
//  后面 10510(BondingByForceManual)启力环要的位置,所以桥接不用再动轴。
//
//  桥接这一侧:
//    1) 收到 10519 → 先置 DI2506=1(与真机"下发后 DI 置位"顺序一致),再把
//       SetSearchForce/SetSearchPosSpeed 缓存值组装成 ForceSearch 下发;
//    2) 后台每 500ms 读一次 28 点位缓存(与 refreshIoCache 共用 /api/io 快照):
//       先确认见到过 Busy(防读到上一次搜索残留的 Done),再等 Busy 落下;
//       Done → DI2506=0(= 搜力完成,主控放行到下压);Fail → 记日志,DI 保持 1。
//  任何失败(下发被联锁拒绝 / Fail / 迟迟不见 Busy)都保持 DI=1 不动,让主控自己的
//  searchPosTimeoutMs 报 BondArmSearchPosTimeout —— 那条报警信息比桥接伪造的准,
//  且它的 RETRY 会回到 +300 重新设参数重搜。
//  停止:主控 10520(SearchBondPosDone)→ StopForceSearch,DI2506 立即回落。
// ---------------------------------------------------------------------------

// ForceSearch 里主控没给的两项参数。
// 主控配方只给"初始位"(re260915: -160mm),带片接触位约 -167.4mm(裸吸盘 -168.9mm),
// 所以 20mm 行程留了一倍余量;1mm/s 下最多 20s,远小于主控 searchPosTimeoutMs(300s)。
// 模型侧 ForceSearch 的 Timeout 范围 [0.01,3600],取 240s 收在缺陷路径上。
static const double kForceSearchMaxDistMm = 20.0;
static const double kForceSearchTimeoutS  = 240.0;

static const char* kBsPointBase = "SystemMachine.SS_Bonding.M_BS.";   // 兜底(正常走 10510 的 path)

bool IoRelayServer::cacheBool(const std::string& path, bool& out) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = pointCache_.find(path);
    if (it == pointCache_.end()) return false;
    out = jsonToBool(it->second);
    return true;
}

bool IoRelayServer::cacheNum(const std::string& path, double& out) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = pointCache_.find(path);
    if (it == pointCache_.end()) return false;
    out = jsonToNum(it->second);
    return true;
}

void IoRelayServer::forceSearchWorker() {
    if (!sim_) {
        forceSearchRunning_ = false;
        return;
    }
    const std::string base = kBsPointBase;
    bool seenBusy = false, busy = false, done = false, fail = false;
    const long long t0 = nowMs();

    for (int i = 0; i < 720 && !stop_; ++i) {   // 500ms 一轮,上限 6min
        sleepMs(500);
        cacheBool(base + "oForceSearchBusy", busy);
        cacheBool(base + "oForceSearchDone", done);
        cacheBool(base + "oForceSearchFail", fail);
        if (busy) {
            seenBusy = true;                    // 搜索真的跑起来了
            continue;
        }
        if (seenBusy) break;                    // Busy 落下:终态已定
        if (fail) break;
        // 还没见到 Busy:done 可能只是上一轮搜索的残留,不认;但等太久说明命令没生效
        if (nowMs() - t0 > 5000) break;
    }

    if (stop_) {                                // 进程退出中,不再改 DI
        forceSearchRunning_ = false;
        return;
    }
    if (seenBusy && done) {
        double pos = 0.0;
        cacheNum(base + "oForceSearchPos", pos);
        ilog("req", "force search: done, head at BS_Arm_Z=" + std::to_string(pos) +
                    " -> DI2506=0 (search finished)");
        diInBondingPos_ = false;
    } else if (fail) {
        ilog("req", "force search: 28 reported oForceSearchFail (no contact/timeout/"
                    "estop/fault) -> keep DI2506=1, let mctrl time out and retry");
    } else if (seenBusy) {
        ilog("req", "force search: stopped without Done (StopForceSearch or axis "
                    "moved) -> keep DI2506=1 unless the stop came from 10520");
    } else {
        ilog("req", "force search: 28 never reported Busy within 5s (model build "
                    "without ForceSearch?) -> keep DI2506=1, let mctrl time out");
    }
    forceSearchRunning_ = false;
}

bool IoRelayServer::writeSim(const IoVarDef& v, const Json& args, std::string& detail) {
    if (!sim_ || !sim_->running()) {
        detail = "sim not running";
        return false;
    }
    Json r = sim_->ioCommand(v.path, v.cmd, args);
    detail = "28=" + r.dump();
    return r.isObject() && jbool(r, "ok", false);
}

// ===================================================================
// 发现:只报 28 侧真实支持的变量(键合工艺未建模的那些不出现,免得 GUI 显示死点)
// ===================================================================
ISTARResultStatus IoRelayServer::IOGetServerInfo(ResponseMessageStruct<IoServerInfo>& response) {
    IoServerInfo info;
    {
        std::lock_guard<std::mutex> lk(mu_);
        for (const auto& v : catalog_->ai) if (v.supported) { IoVarInfo e; e.id = v.id; e.name = v.label; info.io.AI.push_back(e); }
        for (const auto& v : catalog_->di) if (v.supported) { IoVarInfo e; e.id = v.id; e.name = v.label; info.io.DI.push_back(e); }
        for (const auto& v : catalog_->ao) if (v.supported) { IoAoVarInfo e; e.id = v.id; e.name = v.label; e.observer = v.observer; info.io.AO.push_back(e); }
        for (const auto& v : catalog_->do_) if (v.supported) { IoDoVarInfo e; e.id = v.id; e.name = v.label; e.observer = v.observer; info.io.DO.push_back(e); }
        for (const auto& v : catalog_->settingD) if (v.supported) { IoVarInfo e; e.id = v.id; e.name = v.label; info.io.SettingDouble.push_back(e); }
        for (const auto& v : catalog_->settingB) if (v.supported) { IoVarInfo e; e.id = v.id; e.name = v.label; info.io.SettingBool.push_back(e); }
        for (const auto& v : catalog_->cmd) if (v.supported) {
            IoCmdVarInfo e; e.id = v.id; e.name = v.label;
            for (const auto& a : v.args) { IoArgInfo ai; ai.name = a; e.args.push_back(ai); }
            info.io.Cmd.push_back(e);
        }
        for (const auto& v : catalog_->buffer) if (v.supported) { IoVarInfo e; e.id = v.id; e.name = v.label; info.io.Buffer.push_back(e); }
    }
    ilog("req", "GetServerInfo AI=" + std::to_string(info.io.AI.size()) +
                " DI=" + std::to_string(info.io.DI.size()) +
                " AO=" + std::to_string(info.io.AO.size()) +
                " DO=" + std::to_string(info.io.DO.size()) +
                " Cmd=" + std::to_string(info.io.Cmd.size()));
    response.data = info;
    return ISTAR_RESULT_SUCCESS; // 与真 io 一致:GetServerInfo 不设 bSuccess/errorCode
}

// ===================================================================
// 读
// ===================================================================
ISTARResultStatus IoRelayServer::IOGetAi(IoReadInput input, ResponseMessageStruct<IoAiOutput>& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    IoAiOutput out{};
    bool ok = false;
    if (v && v->ioType == "AI" && v->supported) {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = pointCache_.find(v->path);
        if (it != pointCache_.end()) { out.value = jsonToNum(it->second); out.errorCode = kErrNone; ok = true; }
        else out.errorCode = kErrRead;
    } else {
        out.errorCode = kErrNoVar;
    }
    out.value = ok ? out.value : 0.0;
    response.data = out;
    response.bSuccess = ok;
    response.errorCode = out.errorCode;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOGetDi(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    IoDiOutput out{};
    // 合成 DI(28 侧没有这个反馈点):值由桥接自己维护,直接回成功
    if (v && v->synthDi) {
        out.value = diInBondingPos_.load();
        out.errorCode = kErrNone;
        response.data = out;
        response.bSuccess = true;
        response.errorCode = kErrNone;
        return ISTAR_RESULT_SUCCESS;
    }
    bool ok = false;
    if (v && v->ioType == "DI" && v->supported) {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = pointCache_.find(v->readPath);
        if (it != pointCache_.end()) { out.value = jsonToBool(it->second); out.errorCode = kErrNone; ok = true; }
        else out.errorCode = kErrRead;
    } else {
        out.errorCode = kErrNoVar;
    }
    out.value = ok ? out.value : false;
    response.data = out;
    response.bSuccess = ok;
    response.errorCode = out.errorCode;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOGetDo(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    IoDiOutput out{};
    bool ok = false;
    if (v && v->ioType == "DO" && v->supported && !v->readPath.empty()) {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = pointCache_.find(v->readPath);
        if (it != pointCache_.end()) { out.value = jsonToBool(it->second); out.errorCode = kErrNone; ok = true; }
        else out.errorCode = kErrRead;
    } else {
        out.errorCode = kErrNoVar;
    }
    out.value = ok ? out.value : false;
    response.data = out;
    response.bSuccess = ok;
    response.errorCode = out.errorCode;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOGetAo(IoReadInput input, ResponseMessageStruct<IoAiOutput>& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    IoAiOutput out{};
    bool ok = false;
    if (v && v->ioType == "AO" && v->supported && !v->readPath.empty()) {
        std::lock_guard<std::mutex> lk(mu_);
        auto it = pointCache_.find(v->readPath);
        if (it != pointCache_.end()) { out.value = jsonToNum(it->second); out.errorCode = kErrNone; ok = true; }
        else out.errorCode = kErrRead;
    } else {
        out.errorCode = kErrNoVar;
    }
    out.value = ok ? out.value : 0.0;
    response.data = out;
    response.bSuccess = ok;
    response.errorCode = out.errorCode;
    return ISTAR_RESULT_SUCCESS;
}

// ===================================================================
// 写
// ===================================================================
ISTARResultStatus IoRelayServer::IOSetDo(IoDoInput input, ResponseNoDataMessageStruct& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    if (!v || v->ioType != "DO" || !v->supported) {
        ilog("req", "SetDo unknown/unsupported varId=" + std::to_string(input.varId));
        response.bSuccess = false;
        response.errorCode = kErrNoVar;
        return ISTAR_RESULT_SUCCESS;
    }

    // 蓝膜上下片:28 侧没有对应的 IO 行为,这两个点实际是面板上的"蓝膜上片/下片"按钮。
    // 只在上升沿(value=true)触发一次工件接口;写 0 只是"松开",不做任何事 —— 否则
    // 写 0 会被当成再按一次命令(28 的 M_ES 路由不看 args)。
    if (!v->simHook.empty()) {
        if (!input.value) {
            ilog("req", "SetDo varId=" + std::to_string(v->id) + "(" + v->label +
                        ") value=0 -> no-op");
            response.bSuccess = true;
            response.errorCode = kErrNone;
            if (input.verify) pushSetCompleted(input.varId, true, kSetPass);
            return ISTAR_RESULT_SUCCESS;
        }
        const char* act = (v->simHook == "film_load") ? "load" : "unload";
        Json r = sim_ ? sim_->filmAction(act) : Json::null();
        bool ok = r.isObject() && jbool(r, "ok", false);
        ilog("req", "SetDo varId=" + std::to_string(v->id) + "(" + v->label + ") -> " +
                    v->simHook + " POST /api/workpiece/film action=" + act + " 28=" + r.dump());
        response.bSuccess = ok;
        response.errorCode = ok ? kErrNone : kErrWrite;
        if (input.verify) pushSetCompleted(input.varId, ok, ok ? kSetPass : kSetSendFailed);
        return ISTAR_RESULT_SUCCESS;
    }

    Json args = Json::object();
    args["value"] = Json::boolean(input.value);
    std::string detail;
    bool ok = writeSim(*v, args, detail);
    ilog("req", "SetDo varId=" + std::to_string(v->id) + "(" + v->label + ") value=" +
                (input.value ? "1" : "0") + " verify=" + (input.verify ? "1" : "0") + " " + detail);
    response.bSuccess = ok;
    response.errorCode = ok ? kErrNone : kErrWrite;
    // verify=true 才回 SetCompleted(与真 io 一致),模拟器下"28 接收即成功"
    if (input.verify) pushSetCompleted(input.varId, ok, ok ? kSetPass : kSetSendFailed);
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOSetAo(IoAoInput input, ResponseNoDataMessageStruct& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    // 搜力参数钩子要放在 supported 判定之前:28 侧点位表里没有这两个点,
    // 值只留在桥接这边,等 SearchBondPos 时组装成 ForceSearch 的 args。
    // (28 侧 ForceSearch 限 Force∈(0,2000]N、Speed∈(0,20]mm/s,越界就退回 MC 缺省,
    //  免得一条坏配方把搜力整条命令打成 400。)
    if (v && (v->simHook == "search_force_set" || v->simHook == "search_speed_set")) {
        const bool isForce = (v->simHook == "search_force_set");
        const double upper = isForce ? 2000.0 : 20.0;
        double value = input.value;
        if (!(value > 0.0 && value <= upper)) {
            const double fallback = isForce ? 5.0 : 1.0;
            ilog("req", "SetAo " + v->label + "=" + std::to_string(value) +
                        " out of (0," + std::to_string(upper) + "] -> fallback " +
                        std::to_string(fallback));
            value = fallback;
        }
        {
            std::lock_guard<std::mutex> lk(paramMu_);
            if (isForce) searchForceN_ = value;
            else         searchSpeedMmS_ = value;
        }
        ilog("req", "SetAo varId=" + std::to_string(v->id) + "(" + v->label + ") value=" +
                    std::to_string(value) + " -> cached for ForceSearch (28 has no such point)");
        response.bSuccess = true;
        response.errorCode = kErrNone;
        if (input.verify) pushSetCompleted(input.varId, true, kSetPass);
        return ISTAR_RESULT_SUCCESS;
    }
    if (!v || v->ioType != "AO" || !v->supported) {
        ilog("req", "SetAo unknown/unsupported varId=" + std::to_string(input.varId));
        response.bSuccess = false;
        response.errorCode = kErrNoVar;
        return ISTAR_RESULT_SUCCESS;
    }
    Json args = Json::object();
    args["value"] = Json::number(input.value);
    std::string detail;
    bool ok = writeSim(*v, args, detail);
    ilog("req", "SetAo varId=" + std::to_string(v->id) + "(" + v->label + ") value=" +
                std::to_string(input.value) + " verify=" + (input.verify ? "1" : "0") + " " + detail);
    response.bSuccess = ok;
    response.errorCode = ok ? kErrNone : kErrWrite;
    if (input.verify) pushSetCompleted(input.varId, ok, ok ? kSetPass : kSetSendFailed);
    return ISTAR_RESULT_SUCCESS;
}

// PLC 命令:input.values 按 config 里的 args 顺序翻译成 28 命令参数
ISTARResultStatus IoRelayServer::IOExecCmd(IoExecCmdInput input, ResponseNoDataMessageStruct& response) {
    const IoVarDef* v = catalog_->find(input.varId);
    // 特殊动作钩子要放在 supported 判定之前:这类点 28 侧本来就没有对应点位
    if (v && v->simHook == "force_release") {
        // StopForceManualBonding → 模型的"受控卸力":BondingByForceManual{Force:0}。
        // 不卸力的话,力环 PID 会把 BS_Arm_Z 按在接触位,PostBond+100 的抬臂被顶住,
        // moveTimeOut(5s) 超时 → 报警"键合头移动失败"。
        // path/cmd 复用 10510(BondingByForceManual)的目录项,免得在这里写死路径。
        const IoVarDef* force = catalog_->find(10510);
        bool ok = false;
        std::string detail = "catalog has no 10510(BondingByForceManual)";
        if (force != nullptr) {
            Json a = Json::object();
            a["Force"] = Json::number(0.0);
            a["Speed"] = Json::number(0.0);
            ok = writeSim(*force, a, detail);
        }
        ilog("req", "ExecCmd varId=" + std::to_string(v->id) +
                    "(StopForceManualBonding) -> force release (Force=0) " + detail);
        response.bSuccess = ok;
        response.errorCode = ok ? kErrNone : kErrWrite;
        return ISTAR_RESULT_SUCCESS;
    }
    if (v && v->simHook == "bond_force_search") {
        if (forceSearchRunning_.load()) {
            ilog("req", "ExecCmd varId=" + std::to_string(v->id) +
                        "(SearchBondPos) already searching, duplicate ignored");
            response.bSuccess = true;
            response.errorCode = kErrNone;
            return ISTAR_RESULT_SUCCESS;
        }

        double force = 5.0, speed = 1.0;
        {
            std::lock_guard<std::mutex> lk(paramMu_);
            force = searchForceN_;
            speed = searchSpeedMmS_;
        }
        // 主控只给 SetSearchForce/SetSearchPosSpeed(4000/4001),行程与总超时由桥接定
        Json args = Json::object();
        args["Force"] = Json::number(force);
        args["Speed"] = Json::number(speed);
        args["Max_Distance"] = Json::number(kForceSearchMaxDistMm);
        args["Timeout"] = Json::number(kForceSearchTimeoutS);

        // 命令宿主路径复用 10510(BondingByForceManual)的目录项 = M_BS,免得写死
        const IoVarDef* ms = catalog_->find(10510);
        const std::string base = ms ? ms->path : std::string("SystemMachine.SS_Bonding.M_BS");

        // 先置"搜力中"再下发 —— 与真机"下发后 DI 置位"的顺序一致
        diInBondingPos_ = true;
        Json r = sim_ ? sim_->ioCommand(base, "ForceSearch", args) : Json::null();
        const bool ok = r.isObject() && jbool(r, "ok", false);
        ilog("req", "ExecCmd varId=" + std::to_string(v->id) + "(SearchBondPos) -> ForceSearch " +
                    args.dump() + " 28=" + r.dump());

        if (ok) {
            forceSearchRunning_ = true;
            if (forceSearchThread_.joinable()) forceSearchThread_.join();   // 回收上一轮
            forceSearchThread_ = std::thread([this] { forceSearchWorker(); });
        } else {
            // 被 28 联锁拒绝(力控在跑/轴在动/模型流程在跑/急停):DI 保持 1,
            // 让主控自己的 searchPosTimeoutMs 报警并 RETRY
            ilog("req", "ForceSearch rejected by 28 -> keep DI2506=1, let mctrl time out");
        }
        response.bSuccess = ok;
        response.errorCode = ok ? kErrNone : kErrWrite;
        return ISTAR_RESULT_SUCCESS;
    }
    if (v && v->simHook == "bond_force_search_stop") {
        // SearchBondPosDone → StopForceSearch:主控搜位超时/取消时停掉搜力,
        // 停在当前位置且不置 Done。DI2506 立即回落(下一轮搜力会重新置位)。
        const IoVarDef* ms = catalog_->find(10510);
        const std::string base = ms ? ms->path : std::string("SystemMachine.SS_Bonding.M_BS");
        Json a = Json::object();
        Json r = sim_ ? sim_->ioCommand(base, "StopForceSearch", a) : Json::null();
        const bool ok = r.isObject() && jbool(r, "ok", false);
        diInBondingPos_ = false;
        ilog("req", "ExecCmd varId=" + std::to_string(v->id) +
                    "(SearchBondPosDone) -> StopForceSearch 28=" + r.dump() + " -> DI2506=0");
        response.bSuccess = ok;
        response.errorCode = ok ? kErrNone : kErrWrite;
        return ISTAR_RESULT_SUCCESS;
    }
    if (!v || v->ioType != "Cmd" || !v->supported) {
        ilog("req", "ExecCmd unknown/unsupported varId=" + std::to_string(input.varId));
        response.bSuccess = false;
        response.errorCode = kErrNoVar;
        return ISTAR_RESULT_SUCCESS;
    }
    Json args = Json::object();
    size_t n = std::min(input.values.size(), v->args.size());
    for (size_t i = 0; i < n; ++i) args[v->args[i]] = Json::number(input.values[i]);
    args["value"] = Json::boolean(true);   // 无参命令(ES 等)统一带上,28 侧按叶名/cmd 路由
    std::string detail;
    bool ok = writeSim(*v, args, detail);
    ilog("req", "ExecCmd varId=" + std::to_string(v->id) + "(" + v->label + ") args=" +
                args.dump() + " " + detail);
    response.bSuccess = ok;
    response.errorCode = ok ? kErrNone : kErrWrite;
    return ISTAR_RESULT_SUCCESS;
}

void IoRelayServer::pushSetCompleted(int varId, bool success, int errorCode) {
    SetCompletedData d;
    d.varId = varId;
    d.success = success;
    d.errorCode = errorCode;
    IOSetCompletedPublish(d);
}

// ===================================================================
// 未支持/预留(主控 io_config 里没有对应点位)
// ===================================================================
#define IO_UNSUPPORTED(name, inputType)                                             \
    ISTARResultStatus IoRelayServer::name(inputType input, ResponseNoDataMessageStruct& response) { \
        ilog("req", #name " (unsupported) id=" + std::to_string(input.varId));      \
        response.bSuccess = false; response.errorCode = kErrNoVar;                   \
        return ISTAR_RESULT_SUCCESS;                                                 \
    }

IO_UNSUPPORTED(IOStartBuffer, IoVarInput)
IO_UNSUPPORTED(IOStopBuffer, IoVarInput)

#undef IO_UNSUPPORTED

ISTARResultStatus IoRelayServer::IOGetBufferRunning(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) {
    IoDiOutput out{};
    out.errorCode = kErrNoVar;   // 主控配置里没有 buffer 点位
    out.value = false;
    response.data = out;
    response.bSuccess = false;
    response.errorCode = kErrNoVar;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOSetSettingDouble(IoSettingDoubleInput input, ResponseNoDataMessageStruct& response) {
    ilog("req", "SetSettingDouble (unsupported) id=" + std::to_string(input.varId));
    response.bSuccess = false; response.errorCode = kErrNoVar;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOSetSettingBool(IoSettingBoolInput input, ResponseNoDataMessageStruct& response) {
    ilog("req", "SetSettingBool (unsupported) id=" + std::to_string(input.varId));
    response.bSuccess = false; response.errorCode = kErrNoVar;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus IoRelayServer::IOCustomCommand(IoCustomCommand cmd, ResponseNoDataMessageStruct& response) {
    ilog("req", "CustomCommand count=" + std::to_string(cmd.json.size()));
    response.bSuccess = true; response.errorCode = kErrNone;
    return ISTAR_RESULT_SUCCESS;
}

} // namespace relay
