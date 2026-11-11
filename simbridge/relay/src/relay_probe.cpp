// relay_probe.cpp - 用平台客户端库(与 mctrl 同款)验证中转服务端:
//   motion: GetServerInfo/GetServerStatus/Enable/MoveAbs/GetPos + 订阅推送计数
//   io:     GetServerInfo/GetServerStatus/GetDo/SetDo(6009/6010)
// 用法:
//   relay_probe --mw mw_config.yaml --module motion1 [--stage WaferStage_XY]
//   relay_probe --mw mw_config.yaml --module io
#include <csignal>
#include <atomic>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <thread>

#ifdef PROBE_MOTION
#include "motion_client.h"
#include "motion_define.h"
#else
#include "io_client.h"
#include "io_define.h"
#endif
#include "relay/relay_util.h"

static std::atomic<bool> g_stop{false};
static std::atomic<int> g_heartbeat{0}, g_observer{0}, g_completed{0};

static void onSig(int) { g_stop = true; }

static std::string arg(const std::map<std::string, std::string>& a, const std::string& k,
                       const std::string& d = "") {
    auto it = a.find(k);
    return it == a.end() ? d : it->second;
}

int main(int argc, char** argv) {
    std::map<std::string, std::string> a;
    for (int i = 1; i + 1 < argc; i += 2) a[argv[i]] = argv[i + 1];
    std::string mw = arg(a, "--mw", "mw_config.yaml");
    std::string module = arg(a, "--module", "motion1");
    std::string stage = arg(a, "--stage", "");

    std::cout << "[probe] module=" << module << " mw=" << mw << std::endl;

#ifdef PROBE_MOTION
    using namespace ::MotionModule;
    Motion_Client_Inherit client;
    ISTARResultStatus ret = client.MotionClientInit(mw, module);
    std::cout << "[probe] MotionClientInit ret=" << ret << std::endl;
    if (ret != ISTAR_RESULT_SUCCESS) return 2;

    client.RegisterCallback(M_O_T_I_O_N_STATUS_UPLOAD_ID,
        []() -> ISTARResultStatus { g_heartbeat++; return ISTAR_RESULT_SUCCESS; });
    client.RegisterCallback(MOTION_OBSERVER_DATA_ID,
        [](StageObserverDataList&) -> ISTARResultStatus { g_observer++; return ISTAR_RESULT_SUCCESS; });
    client.RegisterCallback(MOTION_CMD_COMPLETED_ID,
        [](CmdCompletedData& d) -> ISTARResultStatus {
            std::cout << "[probe] CmdCompleted " << d.stageName << " " << d.cmdName
                      << " success=" << d.success << " err=" << d.errorCode << std::endl;
            g_completed++; return ISTAR_RESULT_SUCCESS; });

    ServerInfo info;
    ret = client.MotionGetServerInfo(info);
    std::cout << "[probe] GetServerInfo ret=" << ret << " system=" << info.systemName
              << " stages=" << info.stages.size() << std::endl;
    for (const auto& s : info.stages)
        std::cout << "    stage " << s.stageType << " " << s.stageName
                  << " axes=" << s.axisNames.size() << std::endl;

    std::string target = stage.empty() ? (info.stages.empty() ? "" : info.stages[0].stageName) : stage;
    if (!target.empty()) {
        std::cout << "[probe] enable stage=" << target << " ret=" << client.MotionCmdEnable({target})
                  << std::endl;
        relay::sleepMs(600);
        StageMoveOptions mv;
        mv.stageName = target;
        mv.data = {10.0, 20.0};
        mv.timeoutMs = 8000;
        std::cout << "[probe] MoveAbs stage=" << target << " ret=" << client.MotionCmdMoveAbs(mv) << std::endl;
        relay::sleepMs(500);
        VecDouble pos;
        ret = client.MotionGetPos(target, true, pos);
        std::cout << "[probe] GetPos(force) ret=" << ret << " n=" << pos.data.size();
        for (auto v : pos.data) std::cout << " " << v;
        std::cout << std::endl;
    }
    ret = client.GetServerStatus();
    std::cout << "[probe] GetServerStatus ret=" << ret << std::endl;
#else
    using namespace ::IOModule;
    IO_Client_Inherit client;
    ISTARResultStatus ret = client.IOClientInit(mw, module);
    std::cout << "[probe] IOClientInit ret=" << ret << std::endl;
    if (ret != ISTAR_RESULT_SUCCESS) return 2;

    client.RegisterCallback(I_O_STATUS_UPLOAD_ID,
        []() -> ISTARResultStatus { g_heartbeat++; return ISTAR_RESULT_SUCCESS; });
    client.RegisterCallback(I_O_OBSERVER_DATA_ID,
        [](IoObserverData&) -> ISTARResultStatus { g_observer++; return ISTAR_RESULT_SUCCESS; });
    client.RegisterCallback(I_O_SET_COMPLETED_ID,
        [](SetCompletedData& d) -> ISTARResultStatus {
            std::cout << "[probe] SetCompleted varId=" << d.varId
                      << " success=" << d.success << " err=" << d.errorCode << std::endl;
            g_completed++; return ISTAR_RESULT_SUCCESS; });

    IoServerInfo info;
    ret = client.IOGetServerInfo(info);
    std::cout << "[probe] GetServerInfo ret=" << ret << " DO=" << info.io.DO.size() << std::endl;
    for (const auto& v : info.io.DO)
        std::cout << "    DO id=" << v.id << " name=" << v.name << std::endl;

    IoDoInput wr;
    wr.varId = 6009;
    wr.value = true;
    wr.verify = true;
    wr.expectedValue = true;
    std::cout << "[probe] SetDo(6009,true,verify) ret=" << client.IOSetDo(wr) << std::endl;

    IoReadInput rd;
    rd.varId = 6009;
    rd.forceRefresh = false;
    IoDiOutput o;
    ret = client.IOGetDo(rd, o);
    std::cout << "[probe] GetDo(6009) ret=" << ret << " value=" << o.value << " err=" << o.errorCode << std::endl;

    IoReadInput unknown;
    unknown.varId = 12345;
    unknown.forceRefresh = false;
    ret = client.IOGetDo(unknown, o);
    std::cout << "[probe] GetDo(unknown=12345) ret=" << ret << " err=" << o.errorCode << std::endl;

    ret = client.GetServerStatus();
    std::cout << "[probe] GetServerStatus ret=" << ret << std::endl;
#endif

    signal(SIGINT, onSig);
    std::cout << "[probe] waiting 8s for pushes..." << std::endl;
    for (int i = 0; i < 80 && !g_stop; i++) relay::sleepMs(100);
    std::cout << "[probe] heartbeat=" << g_heartbeat << " observer=" << g_observer
              << " completed=" << g_completed << std::endl;
    client.ExitRun();
    std::cout << "[probe] done" << std::endl;
    return 0;
}
