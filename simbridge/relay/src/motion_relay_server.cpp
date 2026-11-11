#include "motion_relay_server.h"
#include "motion_define.h"
#include "relay/relay_util.h"
#include <cstdio>
#include <iostream>

using namespace ::MotionModule;
using namespace ::ISTAR;

namespace relay {

static void mlog(const char* tag, const std::string& msg) {
    std::cout << "[" << timeStr(nowMs()) << "][motion-relay][" << tag << "] " << msg << std::endl;
}

static std::string vecStr(const std::vector<double>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) { if (i) s += ", "; s += std::to_string(v[i]); }
    return s + "]";
}

// 到位判定:除 28 引擎 moving/in_position 标志外,自比对 |当前-目标| 的默认容差(轴单位)
static const double kArriveTolDefault = 0.01;

// 取第 i 轴到位容差:请求 tolerance>0 用之,否则默认 0.01
static double arriveTolFor(const std::vector<double>& tols, size_t i) {
    if (i < tols.size() && tols[i] > 0) return tols[i];
    return kArriveTolDefault;
}

// ============================================================
// 构造 / 定时推送
// ============================================================
MotionRelayServer::MotionRelayServer(MotionCatalog* catalog, SimClient* sim,
                                     const std::string& moduleName)
    : catalog_(catalog), sim_(sim), moduleName_(moduleName) {}

MotionRelayServer::~MotionRelayServer() {
    stop_ = true;
    if (timerThread_.joinable()) timerThread_.join();
    for (auto& kv : busy_) delete kv.second;
}

void MotionRelayServer::startTimers(int observerMs) {
    stop_ = false;
    timerThread_ = std::thread([this, observerMs] {
        long long lastH = 0, lastO = 0;
        while (!stop_) {
            long long now = nowMs();
            if (now - lastH >= 1000) {
                lastH = now;
                MOTIONStatusUploadPublish();  // 平台无参心跳:1s 一发,与真 motion 服务端一致
            }
            if (observerMs > 0 && now - lastO >= observerMs) {
                lastO = now;
                StageObserverDataList list;
                for (const auto& s : catalog_->stages) {
                    StageObserverData d;
                    d.stageName = s.name;
                    bool enable = false, moving = false, inPos = false;
                    stageValues(s, d.pos, d.vel, d.acc, enable, moving, inPos);
                    d.enable = enable; d.moving = moving; d.inPos = inPos;
                    list.items.push_back(std::move(d));
                }
                MotionObserverDataPublish(list);
            }
            sleepMs(50);
        }
    });
}

void MotionRelayServer::pushCmdCompleted(const std::string& stageType, const std::string& stageName,
                                         const std::string& cmdName, bool success, int errorCode) {
    CmdCompletedData d;
    d.stageType = stageType;
    d.stageName = stageName;
    d.cmdName = cmdName;
    d.success = success;
    d.errorCode = errorCode;
    mlog("pub", "CmdCompleted " + stageName + " " + cmdName + " ok=" +
                std::to_string(success) + " err=" + std::to_string(errorCode));
    MotionCmdCompletedPublish(d);
}

// ============================================================
// 映射 / 状态
// ============================================================
std::string MotionRelayServer::simAxisOf(const std::string& stage, int idx) const {
    const SimAxisMap* m = findAxisMap(moduleName_, stage, idx);
    return m ? m->simAxis : "";
}

bool MotionRelayServer::resolveStageAxes(const StageDef& st, std::vector<std::string>& simAxes) const {
    simAxes.clear();
    for (size_t i = 0; i < st.axisNames.size(); i++) {
        std::string a = simAxisOf(st.name, (int)i);
        if (a.empty()) return false;
        simAxes.push_back(a);
    }
    return true;
}

std::vector<std::string> MotionRelayServer::pollNames(const std::string& axis) {
    if (axis == "E_EPIN_Z") return {"E_EPIN_Z_1", "E_EPIN_Z_2", "E_EPIN_Z_3"};
    return {axis};
}

std::vector<std::string> MotionRelayServer::pollNames(const std::vector<std::string>& axes) {
    std::vector<std::string> out;
    for (const auto& a : axes) {
        auto p = pollNames(a);
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
}

bool MotionRelayServer::stageValues(const StageDef& st, std::vector<double>& pos,
                                    std::vector<double>& vel, std::vector<double>& acc,
                                    bool& enable, bool& moving, bool& inPos) {
    pos.assign(st.axisNames.size(), 0.0);
    vel.assign(st.axisNames.size(), 0.0);
    acc.assign(st.axisNames.size(), 0.0);
    enable = false; moving = false; inPos = false;
    if (!sim_ || !sim_->running()) return false;
    bool any = false;
    for (size_t i = 0; i < st.axisNames.size(); i++) {
        std::string simAxis = simAxisOf(st.name, (int)i);
        if (simAxis.empty()) continue;
        auto pn = pollNames(simAxis);
        AxisSnap a0;
        if (!sim_->getAxis(pn[0], a0)) continue;
        any = true;
        pos[i] = a0.pos; vel[i] = a0.vel; acc[i] = 0.0;
        if (i == 0) { enable = a0.enabled; moving = a0.moving; inPos = a0.inPos; }
        moving = moving || a0.moving;
        inPos = inPos && a0.inPos;
        for (size_t k = 1; k < pn.size(); k++) {
            AxisSnap ak;
            if (!sim_->getAxis(pn[k], ak)) continue;
            enable = enable && ak.enabled;
            moving = moving || ak.moving;
            inPos = inPos && ak.inPos;
        }
    }
    return any;
}

// ============================================================
// busy / 异步动作
// ============================================================
bool MotionRelayServer::claimBusy(const std::string& stage) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = busy_.find(stage);
    if (it == busy_.end()) {
        auto* f = new std::atomic<bool>(false);
        busy_[stage] = f;
        it = busy_.find(stage);
    }
    bool expected = false;
    return it->second->compare_exchange_strong(expected, true);
}

void MotionRelayServer::releaseBusy(const std::string& stage) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = busy_.find(stage);
    if (it != busy_.end()) it->second->store(false);
}

bool MotionRelayServer::ackStageCommand(const std::string& stageName,
                                        ResponseNoDataMessageStruct& response,
                                        const StageDef** out) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (!st) {
        mlog("req", "unknown stage=" + stageName);
        response.bSuccess = false; response.errorCode = -1;
        return false;
    }
    std::vector<std::string> axes;
    if (!resolveStageAxes(*st, axes)) {
        mlog("req", "unmapped stage=" + stageName);
        response.bSuccess = false; response.errorCode = -1;
        return false;
    }
    if (!claimBusy(st->name)) {
        mlog("req", "busy stage=" + stageName);
        response.bSuccess = false; response.errorCode = 2;
        return false;
    }
    if (out) *out = st;
    return true;
}

void MotionRelayServer::runAction(const StageDef& st, const std::string& cmdName,
                                  long long timeoutMs, std::function<void()> act,
                                  std::function<CmdState()> pred) {
    std::thread([this, st, cmdName, timeoutMs, act = std::move(act),
                 pred = std::move(pred)]() mutable {
        long long to = (timeoutMs > 0 ? timeoutMs : 30000LL);
        long long t0 = nowMs();
        if (sim_ && sim_->running()) {
            if (act) act();
        } else {
            mlog("run", cmdName + " skip: sim not running");
        }
        CmdState s = CmdState::Pending;
        while (!stop_ && (nowMs() - t0 < to)) {
            if (!sim_ || !sim_->running()) { s = CmdState::DoneFault; break; }
            if (pred) s = pred();
            if (s != CmdState::Pending) break;
            sleepMs(50);
        }
        if (s == CmdState::Pending) s = CmdState::DoneTimeout;
        bool ok = (s == CmdState::DoneOk);
        int ec = ok ? 0 : (s == CmdState::DoneTimeout ? 1 : 2);
        pushCmdCompleted(st.stageType, st.name, cmdName, ok, ec);
        releaseBusy(st.name);
    }).detach();
}

// ============================================================
// 发现
// ============================================================
ISTARResultStatus MotionRelayServer::MotionGetServerInfo(ResponseMessageStruct<ServerInfo>& response) {
    mlog("req", "GetServerInfo");
    response.data.systemName = catalog_->systemName;
    for (const auto& s : catalog_->stages) {
        StageInfo si;
        si.stageType = s.stageType;
        si.stageName = s.name;
        si.axisNames = s.axisNames;
        response.data.stages.push_back(std::move(si));
    }
    response.bSuccess = true;
    mlog("rsp", "GetServerInfo stages=" + std::to_string(catalog_->stages.size()));
    return ISTAR_RESULT_SUCCESS;
}

// ============================================================
// Enable / Disable / Home / Stop(异步:下发+轮询+完成推送)
// ============================================================
static bool acsAxisCmd(SimClient* sim, const std::string& cmd, const std::string& axis) {
    Json req = Json::object();
    req["cmd"] = Json::string(cmd);
    req["axis"] = Json::string(axis);
    Json rsp;
    if (!sim->acsCommand(req, rsp)) return false;
    if (rsp.isObject() && rsp.has("ok")) return jbool(rsp, "ok", false);
    return true; // 无 ok 字段按成功
}

ISTARResultStatus MotionRelayServer::MotionCmdEnable(StageCmdOptions options, ResponseNoDataMessageStruct& response) {
    const StageDef* st = nullptr;
    if (!ackStageCommand(options.stageName, response, &st)) return ISTAR_RESULT_SUCCESS;
    std::vector<std::string> axes; resolveStageAxes(*st, axes);
    mlog("req", "Enable stage=" + st->name);
    response.bSuccess = true; response.errorCode = 0;
    auto pn = pollNames(axes);
    runAction(*st, "Enable", options.timeoutMs,
        [this, axes] { for (auto& a : axes) acsAxisCmd(sim_, "enable", a); },
        [this, pn]() -> CmdState {
            bool en = true;
            for (auto& a : pn) { AxisSnap ax; if (!sim_->getAxis(a, ax)) return CmdState::DoneFault; en = en && ax.enabled; }
            return en ? CmdState::DoneOk : CmdState::Pending;
        });
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdDisable(StageCmdOptions options, ResponseNoDataMessageStruct& response) {
    const StageDef* st = nullptr;
    if (!ackStageCommand(options.stageName, response, &st)) return ISTAR_RESULT_SUCCESS;
    std::vector<std::string> axes; resolveStageAxes(*st, axes);
    mlog("req", "Disable stage=" + st->name);
    response.bSuccess = true; response.errorCode = 0;
    auto pn = pollNames(axes);
    runAction(*st, "Disable", options.timeoutMs,
        [this, axes] { for (auto& a : axes) acsAxisCmd(sim_, "disable", a); },
        [this, pn]() -> CmdState {
            bool en = false;
            for (auto& a : pn) { AxisSnap ax; if (!sim_->getAxis(a, ax)) return CmdState::DoneFault; en = en || ax.enabled; }
            return (!en) ? CmdState::DoneOk : CmdState::Pending;
        });
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdHome(StageCmdOptions options, ResponseNoDataMessageStruct& response) {
    const StageDef* st = nullptr;
    if (!ackStageCommand(options.stageName, response, &st)) return ISTAR_RESULT_SUCCESS;
    std::vector<std::string> axes; resolveStageAxes(*st, axes);
    mlog("req", "Home stage=" + st->name);
    response.bSuccess = true; response.errorCode = 0;
    auto pn = pollNames(axes);
    runAction(*st, "Home", options.timeoutMs,
        [this, axes] { for (auto& a : axes) sim_->axisHome(a, 18); },
        [this, pn]() -> CmdState {
            bool done = true;
            for (auto& a : pn) {
                AxisSnap ax; if (!sim_->getAxis(a, ax)) return CmdState::DoneFault;
                done = done && ax.homeState == "done" && !ax.moving;
            }
            return done ? CmdState::DoneOk : CmdState::Pending;
        });
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdStop(StageCmdOptions options, ResponseNoDataMessageStruct& response) {
    // Stop 不占用 busy:允许打断进行中的命令;不推完成(被打断命令的完成由对应 watcher
    // 在到位/超时后推出,与真机行为近似)
    const StageDef* st = findStage(*catalog_, options.stageName);
    if (!st) { response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS; }
    std::vector<std::string> axes;
    if (!resolveStageAxes(*st, axes)) { response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS; }
    mlog("req", "Stop stage=" + st->name);
    response.bSuccess = true; response.errorCode = 0;
    if (sim_ && sim_->running()) {
        for (auto& a : axes) acsAxisCmd(sim_, "stop", a);
    }
    return ISTAR_RESULT_SUCCESS;
}

// ============================================================
// MoveAbs / MoveRel
// ============================================================
ISTARResultStatus MotionRelayServer::MotionCmdMoveAbs(StageMoveOptions move, ResponseNoDataMessageStruct& response) {
    const StageDef* st = nullptr;
    if (!ackStageCommand(move.stageName, response, &st)) return ISTAR_RESULT_SUCCESS;
    std::vector<std::string> axes; resolveStageAxes(*st, axes);
    if (move.data.size() != axes.size()) {
        releaseBusy(st->name);
        mlog("req", "MoveAbs arg mismatch stage=" + st->name);
        response.bSuccess = false; response.errorCode = -1;
        return ISTAR_RESULT_SUCCESS;
    }
    mlog("req", "MoveAbs stage=" + st->name + " data=" + vecStr(move.data));
    response.bSuccess = true; response.errorCode = 0;
    auto targets = move.data;
    auto tols = move.tolerances;
    auto pn = pollNames(axes);
    runAction(*st, "MoveAbs", move.timeoutMs,
        [this, axes, targets] {
            for (size_t i = 0; i < axes.size(); i++) sim_->axisMove(axes[i], targets[i], 0.0);
        },
        [this, axes, targets, tols, pn]() -> CmdState {
            bool moving = false, inPos = true, fault = false;
            for (auto& a : pn) {
                AxisSnap ax; if (!sim_->getAxis(a, ax)) return CmdState::DoneFault;
                moving = moving || ax.moving; fault = fault || ax.fault;
                inPos = inPos && ax.inPos;
            }
            if (fault) return CmdState::DoneFault;
            if (!(inPos && !moving)) return CmdState::Pending;
            // 自比对:目标与当前位置误差必须 <= 到位容差(默认 0.01)
            for (size_t i = 0; i < axes.size(); i++) {
                AxisSnap ax;
                if (!sim_->getAxis(pollNames(axes[i])[0], ax)) return CmdState::DoneFault;
                if (std::fabs(ax.pos - targets[i]) > arriveTolFor(tols, i)) {
                    return CmdState::Pending;
                }
            }
            return CmdState::DoneOk;
        });
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdMoveRel(StageMoveOptions move, ResponseNoDataMessageStruct& response) {
    const StageDef* st = nullptr;
    if (!ackStageCommand(move.stageName, response, &st)) return ISTAR_RESULT_SUCCESS;
    std::vector<std::string> axes; resolveStageAxes(*st, axes);
    if (move.data.size() != axes.size()) {
        releaseBusy(st->name);
        response.bSuccess = false; response.errorCode = -1;
        return ISTAR_RESULT_SUCCESS;
    }
    mlog("req", "MoveRel stage=" + st->name + " data=" + vecStr(move.data));
    response.bSuccess = true; response.errorCode = 0;
    auto deltas = move.data;
    auto tols = move.tolerances;
    auto pn = pollNames(axes);
    auto absTargets = std::make_shared<std::vector<double>>(axes.size(), 0.0);
    runAction(*st, "MoveRel", move.timeoutMs,
        [this, axes, deltas, absTargets] {
            for (size_t i = 0; i < axes.size(); i++) {
                AxisSnap cur;
                if (sim_->getAxis(pollNames(axes[i])[0], cur)) {
                    (*absTargets)[i] = cur.pos + deltas[i];
                    sim_->axisMove(axes[i], (*absTargets)[i], 0.0);
                }
            }
        },
        [this, axes, absTargets, tols, pn]() -> CmdState {
            bool moving = false, inPos = true, fault = false;
            for (auto& a : pn) {
                AxisSnap ax; if (!sim_->getAxis(a, ax)) return CmdState::DoneFault;
                moving = moving || ax.moving; fault = fault || ax.fault;
                inPos = inPos && ax.inPos;
            }
            if (fault) return CmdState::DoneFault;
            if (!(inPos && !moving)) return CmdState::Pending;
            // 自比对:目标与当前位置误差必须 <= 到位容差(默认 0.01)
            for (size_t i = 0; i < axes.size(); i++) {
                AxisSnap ax;
                if (!sim_->getAxis(pollNames(axes[i])[0], ax)) return CmdState::DoneFault;
                if (std::fabs(ax.pos - (*absTargets)[i]) > arriveTolFor(tols, i))
                    return CmdState::Pending;
            }
            return CmdState::DoneOk;
        });
    return ISTAR_RESULT_SUCCESS;
}

// ============================================================
// 其余即时命令
// ============================================================
ISTARResultStatus MotionRelayServer::MotionCmdResetError(std::string stageName, ResponseNoDataMessageStruct& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (!st) { response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS; }
    std::vector<std::string> axes;
    resolveStageAxes(*st, axes);
    if (sim_ && sim_->running())
        for (auto& a : axes) acsAxisCmd(sim_, "reset_error", a);
    mlog("req", "ResetError stage=" + stageName);
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdJog(StageVecDouble velocities, ResponseNoDataMessageStruct& response) {
    // 说明:真机 Jog 为"速度持续点动";中转按一次性短位移(0.5s×速度)近似,
    // 速度=0 视为停。如需持续 Jog 语义(按住前进/松开停)需另行适配。
    const StageDef* st = findStage(*catalog_, velocities.stageName);
    if (!st) { response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS; }
    std::vector<std::string> axes;
    if (!resolveStageAxes(*st, axes) || velocities.data.size() != axes.size()) {
        response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS;
    }
    mlog("req", "Jog stage=" + st->name + " vel=" + vecStr(velocities.data));
    if (sim_ && sim_->running()) {
        for (size_t i = 0; i < axes.size(); i++) {
            if (velocities.data[i] != 0.0)
                sim_->axisJog(axes[i], velocities.data[i] * 0.5, 0.0);
        }
    }
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdSetVelAcc(StageVelAcc params, ResponseNoDataMessageStruct& response) {
    const StageDef* st = findStage(*catalog_, params.stageName);
    if (!st) { response.bSuccess = false; response.errorCode = -1; return ISTAR_RESULT_SUCCESS; }
    // 【不要】把主控的工艺速度当 28 模型的轴上限(ax.vmax)：
    //   - 主控的 vel 是"本次工艺速度"，已经通过每次 /api/axis/move {speed} 生效；
    //   - 而 ax.vmax 是"轴最大速度"，且是【落盘配置】(config_overrides.json，
    //     重启模型仍在)，连回零速度(home_fast/home_slow)都由它派生；
    //   - 更致命的是 engine.move 里 vmax=min(speed, ax.vmax)：一旦被设小，之后
    //     任何移动都被夹住 —— 键合头 vel=0.5 时，167mm 的行程要 335 秒。
    //   所以这里只记录不改模型配置(模型用自己的默认 vmax，如 BS_Arm_Z=200mm/s)。
    mlog("req", "SetVelAcc stage=" + st->name + " vel=" + vecStr(params.vel) +
                " -> ignored (not pushed to 28 axis vmax)");
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCmdSetDec(StageVecDouble decs, ResponseNoDataMessageStruct& response) {
    mlog("req", "SetDec stage=" + decs.stageName + " (noop)");
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionCmdSetJerk(StageVecDouble jerks, ResponseNoDataMessageStruct& response) {
    mlog("req", "SetJerk stage=" + jerks.stageName + " (noop)");
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionCmdSetKillDec(StageVecDouble vals, ResponseNoDataMessageStruct& response) {
    mlog("req", "SetKillDec stage=" + vals.stageName + " (noop)");
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}

// ============================================================
// 观测
// ============================================================
static int stageRet(const StageDef* st) { return st ? 0 : -1; }

ISTARResultStatus MotionRelayServer::MotionGetPos(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (forceRefresh && sim_) sim_->refreshAxes();
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e, m, ip;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data.data = pos;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionGetVel(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (forceRefresh && sim_) sim_->refreshAxes();
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e, m, ip;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data.data = vel;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionGetAcc(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e, m, ip;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data.data = acc;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionGetEnable(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (forceRefresh && sim_) sim_->refreshAxes();
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e = false, m = false, ip = false;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data = e;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionGetMoving(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (forceRefresh && sim_) sim_->refreshAxes();
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e, m = false, ip;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data = m;
    return ISTAR_RESULT_SUCCESS;
}
ISTARResultStatus MotionRelayServer::MotionGetInPos(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) {
    const StageDef* st = findStage(*catalog_, stageName);
    if (forceRefresh && sim_) sim_->refreshAxes();
    response.bSuccess = (st != nullptr); response.errorCode = stageRet(st);
    if (!st) return ISTAR_RESULT_SUCCESS;
    std::vector<double> pos, vel, acc; bool e, m, ip = false;
    stageValues(*st, pos, vel, acc, e, m, ip);
    response.data = ip;
    return ISTAR_RESULT_SUCCESS;
}

ISTARResultStatus MotionRelayServer::MotionCustomCommand(CustomCommand cmd, ResponseNoDataMessageStruct& response) {
    mlog("req", "CustomCommand count=" + std::to_string(cmd.json.size()));
    response.bSuccess = true; response.errorCode = 0;
    return ISTAR_RESULT_SUCCESS;
}

} // namespace relay
