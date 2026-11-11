// relay_m5_probe.cpp - M5 端到端探针:扮演 GUI,走 GUI→mctrl→motion relay→28 全链路
// 用法: relay_m5_probe --mw mw_config.yaml [--stage WaferStage_XY --pos 30,40]
#include <csignal>
#include <atomic>
#include <iostream>
#include <fstream>
#include <map>
#include "mctrl_client.h"
#include "mctrl_define.h"
#include "relay/relay_util.h"

static std::atomic<bool> g_stop{false};
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
    std::string stage = arg(a, "--stage", "WaferStage_XY");
    std::string posStr = arg(a, "--pos", "30,40");

    std::cout << "[m5] connect mctrl(" << mw << ") stage=" << stage << std::endl;
    MctrlModule::MCTRL_Client_Inherit client;
    ISTARResultStatus ret = client.MCTRLClientInit(mw, "mctrl");
    std::cout << "[m5] MCTRLClientInit ret=" << ret << std::endl;
    if (ret != ISTAR_RESULT_SUCCESS) return 2;

    std::atomic<int> hb{0};
    client.RegisterCallback(MCTRL_TO_GUI_SYSTEM_INFO_ID,
        [&](SystemInfo&) -> ISTARResultStatus { hb++; return ISTAR_RESULT_SUCCESS; });

    // 收 mctrl 转发的 motion observer(位置回显 = 主控 GUI 同源数据)
    std::mutex omu;
    std::map<std::string, MctrlModule::StageObserverData> lastObs;
    client.RegisterCallback(MCTRL_TO_GUI_COMMAND_ID, [&](MctrlModule::ProcessMessage&) -> ISTARResultStatus {
        return ISTAR_RESULT_SUCCESS;
    });
    // 注:GUI 的 observer 回调类型见 mctrl_client.h RegisterCallbackType2(StageObserverDataList)
    client.RegisterCallback(MOTION_OBSERVER_DATA_ID, [&](MctrlModule::StageObserverDataList& lst) -> ISTARResultStatus {
        std::lock_guard<std::mutex> lk(omu);
        for (auto& d : lst.items) lastObs[d.stageName] = d;
        return ISTAR_RESULT_SUCCESS;
    });

    relay::sleepMs(1500);
    std::vector<double> pos;
    {
        std::string cur;
        for (char c : posStr) { if (c == ',') { pos.push_back(atof(cur.c_str())); cur.clear(); } else cur.push_back(c); }
        pos.push_back(atof(cur.c_str()));
    }
    MctrlModule::StageVecDouble mv;
    mv.stageName = stage;
    mv.data = pos;
    std::cout << "[m5] MoveAbs via mctrl stage=" << stage;
    for (auto v : pos) std::cout << " " << v;
    std::cout << std::endl;
    ret = client.MotionCmdMoveAbs(mv);
    std::cout << "[m5] MoveAbs ret=" << ret << std::endl;

    signal(SIGINT, onSig);
    for (int i = 0; i < 100 && !g_stop; i++) {
        relay::sleepMs(100);
        {
            std::lock_guard<std::mutex> lk(omu);
            auto it = lastObs.find(stage);
            if (it != lastObs.end() && it->second.moving) {
                std::cout << "[m5] observing " << stage << " pos=";
                for (auto v : it->second.pos) std::cout << v << " ";
                std::cout << "moving=1" << std::endl;
            }
        }
    }
    std::cout << "[m5] summary hb=" << hb << std::endl;
    client.ExitRun();
    return 0;
}
