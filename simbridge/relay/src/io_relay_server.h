// io_relay_server.h - 复刻 io 服务端:把主控 io 调用翻译成 28 模型 PLC 命令/观测
#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <string>
#include <vector>
#include "io_server.h"
#include "relay/relay_cfg.h"
#include "relay/relay_sim.h"

namespace relay {

class IoRelayServer : public ::IOModule::UIIoControllerServerImpl {
public:
    IoRelayServer(IoCatalog* catalog, SimClient* sim);
    ~IoRelayServer();

    void startTimers();

    // ---- 发现 ----
    ISTARResultStatus IOGetServerInfo(ResponseMessageStruct<IoServerInfo>& response) override;

    // ---- 读 ----
    ISTARResultStatus IOGetAi(IoReadInput input, ResponseMessageStruct<IoAiOutput>& response) override;
    ISTARResultStatus IOGetDi(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) override;
    ISTARResultStatus IOGetAo(IoReadInput input, ResponseMessageStruct<IoAiOutput>& response) override;
    ISTARResultStatus IOGetDo(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) override;

    // ---- 写 ----
    ISTARResultStatus IOSetAo(IoAoInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOSetDo(IoDoInput input, ResponseNoDataMessageStruct& response) override;

    // ---- 其余:缓冲/命令/设置 ----
    ISTARResultStatus IOStartBuffer(IoVarInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOStopBuffer(IoVarInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOGetBufferRunning(IoReadInput input, ResponseMessageStruct<IoDiOutput>& response) override;
    ISTARResultStatus IOExecCmd(IoExecCmdInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOSetSettingDouble(IoSettingDoubleInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOSetSettingBool(IoSettingBoolInput input, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus IOCustomCommand(IoCustomCommand cmd, ResponseNoDataMessageStruct& response) override;

    void pushSetCompleted(int varId, bool success, int errorCode);

private:
    void timerLoop();
    void refreshIoCache();   // GET /api/io → pointCache_,并据此刷新各点 supported
    void publishObserver();  // 500ms 用 28 真实值推 AI/DI/AO/DO
    bool writeSim(const IoVarDef& v, const Json& args, std::string& detail);

    // 搜力(28 侧没有 SearchBondPos 命令与 oInBondingPos 反馈点):
    // 收到 SearchBondPos → 把缓存的搜力参数组装成 ForceSearch 下发给 28,
    // 后台轮询 oForceSearchBusy/Done/Fail,期间把合成 DI diInBondingPos_ 置真。
    // 详见实现处注释。
    void forceSearchWorker();
    // 读 28 点位缓存里的一个布尔/数值点(取不到返回 false)
    bool cacheBool(const std::string& path, bool& out);
    bool cacheNum(const std::string& path, double& out);

    IoCatalog* catalog_;
    SimClient* sim_;
    std::atomic<bool> stop_{false};
    std::thread timerThread_;

    // ---- 搜力状态 ----
    std::atomic<bool> forceSearchRunning_{false};  // 后台轮询线程是否在跑(防重复下发)
    std::atomic<bool> diInBondingPos_{false};      // 合成 DI 2506 的值(搜力中=true)
    std::thread forceSearchThread_;                // ~IoRelayServer 里 join
    // 主控 SetAo 4000/4001 下发的搜力参数(28 侧无对应点位,只缓存)
    std::mutex paramMu_;
    double searchForceN_ = 5.0;      // 缺省取 MC searchForce
    double searchSpeedMmS_ = 1.0;    // 缺省取 MC searchSpeed

    mutable std::mutex mu_;
    std::map<std::string, Json> pointCache_;   // 28 点位快照(path → value)
    bool cacheValid_ = false;
    bool probed_ = false;

    long long lastHeartbeat_ = 0;   // 仅 timer 线程读写
    long long lastObserver_ = 0;
};

} // namespace relay
