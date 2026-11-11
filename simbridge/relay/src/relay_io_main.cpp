// relay_io_main.cpp - 复刻 io 进程(中转层)
// 用法:
//   relay_io --mw mw_config.yaml [--simhost 127.0.0.1] [--simport 9010]
#include <csignal>
#include <atomic>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include "io_server.h"
#include "io_relay_server.h"
#include "relay/relay_util.h"
#include "relay/relay_cfg.h"

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop = true; }

static std::string arg(const std::map<std::string, std::string>& args, const std::string& key,
                       const std::string& def = "") {
    auto it = args.find(key);
    return it == args.end() ? def : it->second;
}

int main(int argc, char** argv) {
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; i += 2) args[argv[i]] = argv[i + 1];

    std::string mw = arg(args, "--mw", "mw_config.yaml");
    std::string simHost = arg(args, "--simhost", "");
    int simPort = atoi(arg(args, "--simport", "9010").c_str());
    // io 变量目录:默认取 mw_config.yaml 同级的 io/ 子目录(= 1927project/config/io)
    std::string ioCfg = arg(args, "--iocfg", "");
    if (ioCfg.empty()) {
        size_t p = mw.find_last_of("/\\");
        ioCfg = (p == std::string::npos ? std::string(".") : mw.substr(0, p)) + "/io";
    }

    std::cout << "[relay-io] mw=" << mw << " iocfg=" << ioCfg
              << (simHost.empty() ? "" : " sim=" + simHost) << std::endl;

    relay::IoCatalog catalog = relay::loadIoCatalogFromConfigDir(ioCfg);
    std::cout << "[relay-io] catalog AI=" << catalog.ai.size()
              << " DI=" << catalog.di.size()
              << " AO=" << catalog.ao.size()
              << " DO=" << catalog.do_.size()
              << " Setting=" << (catalog.settingD.size() + catalog.settingB.size())
              << " Cmd=" << catalog.cmd.size()
              << " Buffer=" << catalog.buffer.size() << std::endl;

    relay::SimClient sim;
    if (!simHost.empty()) sim.start(simHost, simPort, 200);

    relay::IoRelayServer server(&catalog, &sim);
    ISTARResultStatus ret = server.IOServerInit(mw, "io");
    if (ret != ISTAR_RESULT_SUCCESS) {
        std::cerr << "[relay-io] IOServerInit failed ret=" << ret << std::endl;
        return 3;
    }
    ret = server.IOServerRegisterCallback();
    if (ret != ISTAR_RESULT_SUCCESS) {
        std::cerr << "[relay-io] IOServerRegisterCallback failed ret=" << ret << std::endl;
        return 4;
    }
    server.startTimers();
    server.IORun();
    std::cout << "[relay-io] io up." << std::endl;

    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);
    while (!g_stop) relay::sleepMs(200);
    sim.stop();
    return 0;
}
