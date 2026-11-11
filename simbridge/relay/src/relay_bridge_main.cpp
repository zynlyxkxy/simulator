// relay_bridge.cpp - MQTT 桥:28 模型视觉结果 → 主控 VSS_Com;主控 CF 测量 → 28 共焦 → CTL_DR_CF_FLS
// 用法:
//   relay_bridge [--mqtt 127.0.0.1:1883] [--sim 127.0.0.1:9010]
#include <csignal>
#include <atomic>
#include <iostream>
#include <sstream>
#include <map>
#include "relay/relay_mqtt.h"
#include "relay/relay_sim.h"
#include "relay/relay_json.h"
#include "relay/relay_util.h"

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop = true; }

static std::string arg(const std::map<std::string, std::string>& a, const std::string& k,
                       const std::string& d = "") {
    auto it = a.find(k);
    return it == a.end() ? d : it->second;
}

namespace {

std::string g_mqttHost = "127.0.0.1";
int g_mqttPort = 1883;

// 心跳/应答共用信封:context 空(主控只 markSeen 不解析内容)
std::string buildEnvelope(const std::string& task) {
    relay::Json env = relay::Json::object();
    env["task"] = relay::Json::string(task);
    env["result"] = relay::Json::boolean(true);
    env["context"] = relay::Json::string("");
    env["errMsg"] = relay::Json::string("");
    env["uuid"] = relay::Json::string("");
    env["PublishType"] = relay::Json::number(0);
    return env.dump();
}

// 相机名 → 主控 camID(默认 up=0, down=1, coarse 未用)
int camIdOf(const std::string& cam) {
    if (cam == "up") return 0;
    if (cam == "down") return 1;
    return -1;
}

std::string jsonStrOf(const relay::Json& j) { return j.dump(); }

// ================= 视觉:28 result → 主控 VSS_Com =================
void handleVisionResult(relay::MqttClient& pub, const std::string& cam, const std::string& payload) {
    relay::Json j;
    try { j = relay::Json::parse(payload); } catch (...) { return; }
    int camId = camIdOf(cam);
    if (camId < 0) return;
    relay::Json result = j.at("result");
    bool judge = result.isObject() ? jbool(result, "judge", false) : false;
    // 抽数组
    auto arrOf = [&](const char* k) -> relay::Json {
        relay::Json a = relay::Json::array();
        if (result.isObject() && result.at(k).isArray()) {
            for (const auto& e : result.at(k).arr()) a.push_back(e);
        }
        return a;
    };
    relay::Json inner = relay::Json::object();
    inner["strShapeName"] = relay::Json::string(
        result.isObject() ? jstr(result, "strShapeName", "") : "");
    inner["judge"] = relay::Json::boolean(judge);
    inner["lstCol"] = arrOf("lstCol");
    inner["lstRow"] = arrOf("lstRow");
    inner["lstAngle"] = arrOf("lstAngle");
    inner["lstScore"] = arrOf("lstScore");
    inner["lstScale"] = arrOf("lstScale");
    if (result.isObject() && result.at("lstName").isArray())
        inner["lstName"] = arrOf("lstName");

    relay::Json contentMap = relay::Json::object();
    contentMap["1"] = inner;

    relay::Json rcp = relay::Json::object();
    rcp["cmdType"] = relay::Json::number(2);
    rcp["camID"] = relay::Json::number(camId);
    relay::Json lst = relay::Json::array();
    lst.push_back(relay::Json::number(0));
    lst.push_back(relay::Json::number(1));
    rcp["lstCellType"] = lst;
    rcp["content"] = relay::Json::string("");
    rcp["isImageSend"] = relay::Json::boolean(false);
    rcp["imagePath"] = relay::Json::string("");
    rcp["image"] = relay::Json::string("");
    rcp["imgType"] = relay::Json::number(-1);
    rcp["m_bRealtimeReturn"] = relay::Json::boolean(false);

    relay::Json ctx = relay::Json::object();
    ctx["rcp"] = rcp;
    ctx["totalJudge"] = relay::Json::boolean(judge);
    ctx["content"] = relay::Json::string(contentMap.dump());
    ctx["image"] = relay::Json::string("");

    relay::Json env = relay::Json::object();
    env["task"] = relay::Json::string("CamSnapOnce");
    env["result"] = relay::Json::boolean(true);
    env["context"] = relay::Json::string(ctx.dump());
    env["errMsg"] = relay::Json::string("");
    env["uuid"] = relay::Json::string("");
    env["PublishType"] = relay::Json::number(0);

    std::cout << "[" << relay::timeStr(relay::nowMs()) << "][bridge] vision/result/" << cam
              << " -> VSS_Com camID=" << camId << " judge=" << judge << std::endl;
    pub.publish("VSS_Com", env.dump());
}

// ================= CF:主控 MeasureOneChannel → 28 共焦 → CTL_DR_CF_FLS =================
void handleCfMeasure(relay::MqttClient& pub, relay::SimClient& sim, const std::string& payload) {
    relay::Json env;
    try { env = relay::Json::parse(payload); } catch (...) { return; }
    std::string ctxStr = jstr(env, "context", "");
    if (ctxStr.empty()) return;
    relay::Json ctx;
    try { ctx = relay::Json::parse(ctxStr); } catch (...) { return; }
    int sensorId = (int)ctx.at("sensorId").asInt64(0);
    int count = (int)ctx.at("collectionQuantity").asInt64(1);
    if (count < 1) count = 1;
    // sensorId → probe(默认:0=down,1=up;可配置化【待定稿】)
    std::string probe = (sensorId == 1) ? "up" : "down";
    std::cout << "[" << relay::timeStr(relay::nowMs()) << "][bridge] CF measure sensorId="
              << sensorId << " probe=" << probe << " count=" << count << std::endl;

    relay::Json data = relay::Json::array();
    for (int i = 0; i < count; i++) {
        double um = 0.0;
        relay::Json r = sim.confocalRead(probe);
        if (r.isObject() && jbool(r, "valid", false)) um = jnum(r, "reading_um", 0.0);
        else if (r.isObject() && r.at("reading_um").isNumber()) um = jnum(r, "reading_um", 0.0);
        data.push_back(relay::Json::number(um));
        if (count > 1) relay::sleepMs(30);
    }

    relay::Json res = relay::Json::object();
    res["id"] = relay::Json::number(0);
    res["count"] = relay::Json::number(count);
    res["data"] = data;
    relay::Json ra = relay::Json::array();
    res["unitRa"] = ra;
    res["peakItn"] = ra;

    relay::Json out = relay::Json::object();
    out["task"] = relay::Json::string("CMD_CF_MeasureOneResult");
    out["result"] = relay::Json::boolean(true);
    out["context"] = relay::Json::string(res.dump());
    out["errMsg"] = relay::Json::string("");
    out["uuid"] = relay::Json::string("");
    out["PublishType"] = relay::Json::number(0);
    pub.publish("CTL_DR_CF_FLS", out.dump());
    std::cout << "[" << relay::timeStr(relay::nowMs()) << "][bridge] -> CTL_DR_CF_FLS data="
              << data.dump() << std::endl;
}

} // namespace

int main(int argc, char** argv) {
    std::map<std::string, std::string> a;
    for (int i = 1; i + 1 < argc; i += 2) a[argv[i]] = argv[i + 1];
    std::string mqttAddr = arg(a, "--mqtt", "127.0.0.1:1883");
    std::string simAddr = arg(a, "--sim", "127.0.0.1:9010");
    size_t colon = mqttAddr.rfind(':');
    if (colon != std::string::npos) {
        g_mqttHost = mqttAddr.substr(0, colon);
        g_mqttPort = atoi(mqttAddr.substr(colon + 1).c_str());
    }
    colon = simAddr.rfind(':');
    std::string simHost = simAddr;
    int simPort = 9010;
    if (colon != std::string::npos) {
        simHost = simAddr.substr(0, colon);
        simPort = atoi(simAddr.substr(colon + 1).c_str());
    }
    std::cout << "[bridge] mqtt=" << g_mqttHost << ":" << g_mqttPort
              << " sim=" << simHost << ":" << simPort << std::endl;

    relay::SimClient sim;   // 不轮询,仅同步读 confocal
    relay::MqttClient mqtt;
    mqtt.setOnMessage([&](const std::string& topic, const std::string& payload) {
        if (topic.rfind("vision/result/", 0) == 0) {
            std::string cam = topic.substr(std::string("vision/result/").size());
            handleVisionResult(mqtt, cam, payload);
        } else if (topic == "CTL_WM_CF_ControlManager") {
            relay::Json j;
            try { j = relay::Json::parse(payload); } catch (...) { return; }
            std::string task = jstr(j, "task", "");
            if (task == "CMD_CF_MeasureOneChannel")
                handleCfMeasure(mqtt, sim, payload);
            else
                std::cout << "[bridge] cf task ignored: " << task << std::endl;
        } else if (topic == "VSC_Com") {
            // 主控下发配方/触发(调试宏构建)记录;拍照由 io DO 硬触发驱动
            relay::Json j;
            try { j = relay::Json::parse(payload); } catch (...) { return; }
            std::cout << "[bridge] VSC_Com task=" << jstr(j, "task", "?") << " (record only)"
                      << std::endl;
        }
    });
    mqtt.start(g_mqttHost, g_mqttPort, "relay-bridge",
               {"vision/result/#", "VSC_Com", "CTL_WM_CF_ControlManager"});

    // 1s 心跳:VSS_Com / CTL_DR_CF_FLS 各发一条空 context 信封,让主控 StatusManager
    // 判定视觉/共焦在线(mctrl 收任何消息即 markSeen,空 context 安全返回)
    bool hb = arg(a, "--hb", "1") != "0";
    std::thread hbThread;
    if (hb) {
        hbThread = std::thread([&] {
            long long last = 0, lastLog = 0;
            while (!g_stop) {
                long long now = relay::nowMs();
                if (now - last >= 1000) {
                    last = now;
                    bool ok1 = mqtt.publish("VSS_Com", buildEnvelope("Heartbeat"));
                    bool ok2 = mqtt.publish("CTL_DR_CF_FLS", buildEnvelope("Heartbeat"));
                    std::cout << "[" << relay::timeStr(now)
                              << "][bridge] hb vss=" << ok1 << " cf=" << ok2 << std::endl;
                    if (now - lastLog >= 60000) {
                        lastLog = now;
                    }
                }
                relay::sleepMs(100);
            }
        });
    }

    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);
    while (!g_stop) relay::sleepMs(200);
    if (hb && hbThread.joinable()) hbThread.join();
    mqtt.stop();
    std::cout << "[bridge] bye" << std::endl;
    return 0;
}
