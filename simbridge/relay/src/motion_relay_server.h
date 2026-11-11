// motion_relay_server.h - 复刻 motion1~4 服务端(转发到 28 模型)
#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <map>
#include <functional>
#include "motion_server.h"
#include "relay/relay_cfg.h"
#include "relay/relay_sim.h"

namespace relay {

enum class CmdState { Pending, DoneOk, DoneTimeout, DoneStop, DoneFault };

class MotionRelayServer : public ::MotionModule::UIMotionControllerServerImpl {
public:
    MotionRelayServer(MotionCatalog* catalog, SimClient* sim, const std::string& moduleName);
    ~MotionRelayServer();

    void startTimers(int observerMs);

    // ---- 发现 ----
    ISTARResultStatus MotionGetServerInfo(::ResponseMessageStruct<ServerInfo>& response) override;

    // ---- 命令(覆写) ----
    ISTARResultStatus MotionCmdEnable(StageCmdOptions options, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdDisable(StageCmdOptions options, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdHome(StageCmdOptions options, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdMoveAbs(StageMoveOptions move, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdMoveRel(StageMoveOptions move, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdStop(StageCmdOptions options, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdResetError(std::string stageName, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdJog(StageVecDouble velocities, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdSetVelAcc(StageVelAcc params, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdSetDec(StageVecDouble decs, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdSetJerk(StageVecDouble jerks, ResponseNoDataMessageStruct& response) override;
    ISTARResultStatus MotionCmdSetKillDec(StageVecDouble vals, ResponseNoDataMessageStruct& response) override;

    // ---- 观测 ----
    ISTARResultStatus MotionGetPos(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) override;
    ISTARResultStatus MotionGetVel(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) override;
    ISTARResultStatus MotionGetAcc(std::string stageName, bool forceRefresh, ResponseMessageStruct<VecDouble>& response) override;
    ISTARResultStatus MotionGetEnable(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) override;
    ISTARResultStatus MotionGetMoving(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) override;
    ISTARResultStatus MotionGetInPos(std::string stageName, bool forceRefresh, ResponseMessageStruct<bool>& response) override;

    ISTARResultStatus MotionCustomCommand(CustomCommand cmd, ResponseNoDataMessageStruct& response) override;

    void pushCmdCompleted(const std::string& stageType, const std::string& stageName,
                          const std::string& cmdName, bool success, int errorCode);

private:
    // ---- 映射 ----
    // stage 第 idx 轴的 28 轴名(空=未映射)
    std::string simAxisOf(const std::string& stage, int idx) const;
    // stage 全部轴的 28 轴名列表(任一未映射则返回 false)
    bool resolveStageAxes(const StageDef& st, std::vector<std::string>& simAxes) const;
    // 别名组展开为轮询名(如 E_EPIN_Z → 3 针)
    static std::vector<std::string> pollNames(const std::string& axis);
    static std::vector<std::string> pollNames(const std::vector<std::string>& axes);

    // ---- 状态 ----
    bool axisOk(const std::string& simAxis);                       // 读缓存
    bool stageValues(const StageDef& st, std::vector<double>& pos, std::vector<double>& vel,
                     std::vector<double>& acc, bool& enable, bool& moving, bool& inPos);

    // ---- 命令执行 ----
    bool claimBusy(const std::string& stage);                      // false=busy
    void releaseBusy(const std::string& stage);
    // 通用异步动作:整个流程在独立线程执行(先 act 再轮询 pred,结束后推完成并释放 busy)
    void runAction(const StageDef& st, const std::string& cmdName, long long timeoutMs,
                   std::function<void()> act, std::function<CmdState()> pred);
    // 同步 ACK 前置检查:查台/查映射/占 busy;失败时填 response 返回 false
    bool ackStageCommand(const std::string& stageName, ResponseNoDataMessageStruct& response,
                         const StageDef** out);

    void heartbeatLoop();
    void observerLoop(int periodMs);

    MotionCatalog* catalog_;
    SimClient* sim_;
    std::string moduleName_;
    std::atomic<bool> stop_{false};
    std::thread timerThread_;
    std::mutex mu_;
    std::map<std::string, std::atomic<bool>*> busy_;
};

} // namespace relay
