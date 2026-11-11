// relay_motion_main.cpp - 复刻 motion1~4 进程(中转层)
// 用法:
//   relay_motion --mw mw_config.yaml --module motion1 --cfg config1.json
//               [--simhost 127.0.0.1] [--simport 9010] [--observer 500]
#include <csignal>
#include <atomic>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include "motion_server.h"
#include "motion_relay_server.h"
#include "relay/relay_util.h"
#include "relay/relay_cfg.h"

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop = true; }

static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return "";
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

static std::string arg(const std::map<std::string, std::string>& args, const std::string& key,
                       const std::string& def = "") {
    auto it = args.find(key);
    return it == args.end() ? def : it->second;
}

int main(int argc, char** argv) {
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; i += 2) args[argv[i]] = argv[i + 1];

    std::string mw = arg(args, "--mw", "mw_config.yaml");
    std::string module = arg(args, "--module", "motion1");
    std::string cfgPath = arg(args, "--cfg", "");
    std::string simHost = arg(args, "--simhost", "");
    int simPort = atoi(arg(args, "--simport", "9010").c_str());
    int observerMs = atoi(arg(args, "--observer", "500").c_str());

    std::cout << "[relay-motion] module=" << module << " mw=" << mw
              << " cfg=" << cfgPath << (simHost.empty() ? "" : " sim=" + simHost)
              << std::endl;

    relay::MotionCatalog catalog;
    try {
        std::string text = readFile(cfgPath);
        if (text.empty()) throw std::runtime_error("cfg file empty/missing");
        catalog = relay::loadMotionCatalog(text);
    } catch (const std::exception& e) {
        std::cerr << "[relay-motion] cfg load failed: " << e.what() << std::endl;
        return 2;
    }
    std::cout << "[relay-motion] system=" << catalog.systemName
              << " stages=" << catalog.stages.size() << std::endl;
    for (const auto& s : catalog.stages)
        std::cout << "    stage " << s.stageType << " " << s.name
                  << " axes=" << s.axisNames.size() << std::endl;

    relay::SimClient sim;
    if (!simHost.empty()) sim.start(simHost, simPort, 200);

    relay::MotionRelayServer server(&catalog, &sim, module);
    ISTARResultStatus ret = server.MotionServerInit(mw, module);
    if (ret != ISTAR_RESULT_SUCCESS) {
        std::cerr << "[relay-motion] MotionServerInit failed ret=" << ret << std::endl;
        return 3;
    }
    ret = server.MotionServerRegisterCallback();
    if (ret != ISTAR_RESULT_SUCCESS) {
        std::cerr << "[relay-motion] MotionServerRegisterCallback failed ret=" << ret << std::endl;
        return 4;
    }
    server.startTimers(observerMs);
    server.MotionRun();
    std::cout << "[relay-motion] " << module << " up." << std::endl;

    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);
    while (!g_stop) relay::sleepMs(200);
    sim.stop();
    return 0;
}
