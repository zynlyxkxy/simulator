// relay_sim.h - 28 模型客户端(HTTP)+ 轴状态缓存
#pragma once
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <cstdio>
#include <cstdlib>
#include "relay/relay_json.h"

namespace relay {

struct AxisSnap {
    std::string name;
    double pos = 0.0, vel = 0.0;
    bool enabled = false, moving = false, inPos = false, fault = false;
    std::string state = "idle";
    std::string homeState = "done";   // 回零状态(done/…)
};

// 对 28 模型 9010/9102 的访问。轮询线程缓存 /api/axes 供 Observer/GetPos 使用。
class SimClient {
public:
    SimClient() = default;
    ~SimClient() { stop(); }

    // host/port: 28 模型 WS 网关(9010)。start 后起轮询线程。
    void start(const std::string& host, int port, int pollMs = 200);
    void stop();

    bool running() const { return running_.load(); }
    std::string lastError() const { return lastError_; }

    // 缓存快照
    std::vector<AxisSnap> snapshotAxes() const;
    bool getAxis(const std::string& name, AxisSnap& out) const;
    long long lastPollAt() const { return lastPollAt_; }

    // ---- 同步原语(调用时阻塞) ----
    // GET /api/axes 强制刷新,返回解析结果(未开轮询时也可手动调用)
    bool refreshAxes();
    // POST /api/axis/move {axis,target,speed} -> 返回 {ok}
    bool axisMove(const std::string& axis, double target, double speed, double* durationMs = nullptr);
    // POST /api/axis/jog {axis,delta,speed}
    bool axisJog(const std::string& axis, double delta, double speed);
    // POST /api/axis/home {axis,mode}
    bool axisHome(const std::string& axis, int mode = 18);
    // POST /api/axis/config {axis,vmax,...} 预留
    bool axisSetSpeed(const std::string& axis, double vmax);
    // ACS TCP(9101)一行 JSON 指令,回一行 JSON
    bool acsCommand(const Json& req, Json& resp, int timeoutMs = 3000);
    // POST /api/io/command {path,cmd,args}
    Json ioCommand(const std::string& path, const std::string& cmd, const Json& args);
    // GET /io/observe?path=... (9102;仅供诊断)
    Json ioObserve(const std::string& path);
    // GET /api/io → {ok,count,points:{path:value}} 28 全量点位快照(9010)
    Json ioSnapshot();
    // POST /api/workpiece/film {action:"load"|"unload"} —— 28 面板的"蓝膜上片/下片"按钮
    Json filmAction(const std::string& action);
    // GET /api/axes 完整 json(供上层再处理)
    Json axesJson();
    // GET /api/confocal/read?probe=...
    Json confocalRead(const std::string& probe);

    // 通用 GET/POST(返回解析后的 json;错误时置 lastError_)
    Json httpGet(const std::string& pathAndQuery);
    Json httpPostJson(const std::string& path, const std::string& body);

private:
    void pollLoop();

    std::string host_ = "127.0.0.1";
    int port_ = 9010;
    int pollMs_ = 200;
    std::atomic<bool> running_{false};
    std::thread pollThread_;

    mutable std::mutex mu_;
    std::vector<AxisSnap> axes_;
    long long lastPollAt_ = 0;
    std::string lastError_;
};

} // namespace relay
