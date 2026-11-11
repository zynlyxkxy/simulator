// relay_mqtt.h - 极简 MQTT 3.1.1 客户端(纯 WinSock,QoS0)
// 足够:CONNECT/SUBSCRIBE/PUBLISH(QoS0)/PINGREQ,自动重连+重订阅。
// 用于 28 模型 mini broker 与 EMQX(生产)。消息经回调分发。
#pragma once
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdint>
#include <map>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>

namespace relay {

// topic 过滤(支持 + #),用于订阅回调匹配
bool mqttTopicMatch(const std::string& filter, const std::string& topic);

class MqttClient {
public:
    using OnMessage = std::function<void(const std::string& topic, const std::string& payload)>;

    MqttClient() = default;
    ~MqttClient() { stop(); }

    void setOnMessage(OnMessage cb) { cb_ = std::move(cb); }
    // 连接参数;subscribeFilters 启动后自动订阅并重订阅
    void start(const std::string& host, int port, const std::string& clientId,
               const std::vector<std::string>& subscribeFilters);
    void stop();

    bool connected() const { return connected_.load(); }
    // QoS0 发布
    bool publish(const std::string& topic, const std::string& payload);

private:
    void runLoop();                 // 连接/保活/重连线程
    bool connectOnce(SOCKET sock);  // 发送 CONNECT/SUBSCRIBE
    bool pumpOnce(SOCKET sock);     // 读一条报文;false=需重连
    bool sendPacket(SOCKET sock, const std::vector<char>& pkt);

    std::string host_;
    int port_ = 1883;
    std::string clientId_;
    std::vector<std::string> filters_;
    OnMessage cb_;
    std::atomic<bool> run_{false};
    std::atomic<bool> connected_{false};
    std::thread thread_;
    std::mutex pubMu_;   // 发布串行
};

} // namespace relay
