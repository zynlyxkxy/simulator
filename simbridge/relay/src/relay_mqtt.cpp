#include "relay/relay_mqtt.h"
#include "relay/relay_util.h"
#include <ws2tcpip.h>
#include <iostream>
#include <sstream>

#pragma comment(lib, "ws2_32.lib")

namespace relay {

// ---------- topic 匹配 ----------
bool mqttTopicMatch(const std::string& filter, const std::string& topic) {
    if (filter == "#") return true;
    std::vector<std::string> fp, tp;
    auto split = [](const std::string& s, std::vector<std::string>& out) {
        std::string cur;
        for (char c : s) { if (c == '/') { out.push_back(cur); cur.clear(); } else cur.push_back(c); }
        out.push_back(cur);
    };
    split(filter, fp);
    split(topic, tp);
    for (size_t i = 0; i < fp.size(); i++) {
        if (fp[i] == "#") return i == fp.size() - 1;
        if (i >= tp.size()) return false;
        if (fp[i] != "+" && fp[i] != tp[i]) return false;
    }
    return fp.size() == tp.size();
}

// ---------- 编码 ----------
static std::vector<char> encStr(const std::string& s) {
    std::vector<char> out;
    uint16_t n = (uint16_t)s.size();
    out.push_back((char)(n >> 8)); out.push_back((char)(n & 0xFF));
    for (char c : s) out.push_back(c);
    return out;
}
static std::vector<char> encRemainingLen(size_t len) {
    std::vector<char> out;
    do {
        unsigned char d = (unsigned char)(len % 128);
        len /= 128;
        if (len) d |= 0x80;
        out.push_back((char)d);
    } while (len);
    return out;
}
static void append(std::vector<char>& a, const std::vector<char>& b) { a.insert(a.end(), b.begin(), b.end()); }
static void appendStr(std::vector<char>& a, const std::string& s) { auto e = encStr(s); append(a, e); }

static bool recvExact(SOCKET s, char* buf, size_t n, int timeoutMs) {
    DWORD t = (DWORD)timeoutMs;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&t, sizeof t);
    size_t got = 0;
    while (got < n) {
        int r = recv(s, buf + got, (int)(n - got), 0);
        if (r <= 0) return false;
        got += (size_t)r;
    }
    return true;
}

// ---------- 客户端 ----------
void MqttClient::start(const std::string& host, int port, const std::string& clientId,
                       const std::vector<std::string>& subscribeFilters) {
    host_ = host;
    port_ = port;
    clientId_ = clientId;
    filters_ = subscribeFilters;
    if (run_.exchange(true)) return;
    thread_ = std::thread([this] { runLoop(); });
}

void MqttClient::stop() {
    if (!run_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

bool MqttClient::sendPacket(SOCKET sock, const std::vector<char>& pkt) {
    size_t off = 0;
    while (off < pkt.size()) {
        int n = send(sock, pkt.data() + off, (int)(pkt.size() - off), 0);
        if (n <= 0) return false;
        off += (size_t)n;
    }
    return true;
}

bool MqttClient::connectOnce(SOCKET sock) {
    // CONNECT
    std::vector<char> pkt;
    pkt.push_back(0x10);
    std::vector<char> body;
    appendStr(body, "MQTT");            // protocol
    body.push_back(4);                  // level 4
    body.push_back(0x02);               // flags: clean session
    uint16_t keepalive = 30;
    body.push_back((char)(keepalive >> 8)); body.push_back((char)(keepalive & 0xFF));
    appendStr(body, clientId_);
    auto rl = encRemainingLen(body.size());
    pkt.push_back((char)rl[0]);
    append(pkt, body);
    if (!sendPacket(sock, pkt)) return false;

    char ack[4];
    if (!recvExact(sock, ack, 4, 5000)) return false;
    if ((ack[0] >> 4) != 2 || ack[3] != 0) return false;

    // SUBSCRIBE(每条 QoS0)
    for (const auto& f : filters_) {
        std::vector<char> sp;
        sp.push_back(0x82);
        std::vector<char> sb;
        uint16_t pid = 1;
        sb.push_back((char)(pid >> 8)); sb.push_back((char)(pid & 0xFF));
        appendStr(sb, f);
        sb.push_back(0);   // QoS0
        auto srl = encRemainingLen(sb.size());
        sp.push_back((char)srl[0]);
        append(sp, sb);
        if (!sendPacket(sock, sp)) return false;
        char suback[5];
        if (!recvExact(sock, suback, 5, 5000)) return false;
    }
    return true;
}

bool MqttClient::pumpOnce(SOCKET sock) {
    char hdr[2];
    if (!recvExact(sock, hdr, 2, 30000)) return false;
    int type = hdr[0] >> 4;
    int flags = hdr[0] & 0x0F;
    size_t remLen = 0, mul = 1;
    int bytes = 1;
    char c = hdr[1];
    while ((c & 0x80)) {
        remLen += (size_t)(c & 0x7F) * mul;
        mul *= 128;
        if (!recvExact(sock, &c, 1, 5000)) return false;
        bytes++;
    }
    remLen += (size_t)(c & 0x7F) * mul;
    if (remLen > (1 << 20)) return false;
    std::string body;
    body.resize(remLen);
    if (remLen && !recvExact(sock, &body[0], remLen, 5000)) return false;

    if (type == 3 && cb_) {   // PUBLISH
        size_t p = 0;
        if (body.size() < 2) return true;
        uint16_t tlen = (uint8_t)body[0] * 256 + (uint8_t)body[1];
        p = 2;
        if (p + tlen > body.size()) return true;
        std::string topic = body.substr(p, tlen);
        p += tlen;
        int qos = (flags >> 1) & 0x3;
        if (qos > 0 && p + 2 <= body.size()) p += 2;
        std::string payload = body.substr(p);
        cb_(topic, payload);
    } else if (type == 8) {
        // SUBACK 已由 connectOnce 消费,这里为订阅过程中若 broker 稍后回,忽略
    } else if (type == 12) {
        // PINGRESP 忽略
    }
    return true;
}

void MqttClient::runLoop() {
    WSADATA wd;
    WSAStartup(MAKEWORD(2, 2), &wd);
    while (run_) {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock != INVALID_SOCKET) {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons((unsigned short)port_);
            inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
            if (connect(sock, (sockaddr*)&addr, sizeof addr) == 0) {
                if (connectOnce(sock)) {
                    connected_ = true;
                    // 保活线程发 PINGREQ + 主循环收报文
                    long long lastPing = nowMs();
                    bool alive = true;
                    while (run_ && alive) {
                        if (!pumpOnce(sock)) { alive = false; break; }
                        if (nowMs() - lastPing >= 10000) {
                            std::vector<char> ping = {(char)0xC0, 0};
                            std::lock_guard<std::mutex> lk(pubMu_);
                            alive = sendPacket(sock, ping);
                            lastPing = nowMs();
                        }
                    }
                    connected_ = false;
                }
            }
            closesocket(sock);
        }
        for (int i = 0; i < 10 && run_; i++) sleepMs(100);   // 1s 重连
    }
}

bool MqttClient::publish(const std::string& topic, const std::string& payload) {
    if (!connected_) return false;
    std::vector<char> pkt;
    pkt.push_back(0x30);   // PUBLISH QoS0
    std::vector<char> body = encStr(topic);
    for (char c : payload) body.push_back(c);
    auto rl = encRemainingLen(body.size());
    pkt.insert(pkt.end(), rl.begin(), rl.end());
    append(pkt, body);
    // 通过 loop 线程持有的 socket 发送:我们使用短连接方式?为简单可靠:
    // 直接开一次性连接发布(每次 publish 一连接,低频率场景足够)
    WSADATA wd;
    WSAStartup(MAKEWORD(2, 2), &wd);
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
    bool ok = false;
    if (connect(sock, (sockaddr*)&addr, sizeof addr) == 0) {
        // CONNECT(短连接)
        std::vector<char> cp;
        cp.push_back(0x10);
        std::vector<char> body2;
        appendStr(body2, "MQTT");
        body2.push_back(4); body2.push_back(2);
        body2.push_back(0); body2.push_back(30);
        appendStr(body2, clientId_ + "-p");
        auto crl = encRemainingLen(body2.size());
        cp.push_back((char)crl[0]);
        append(cp, body2);
        if (sendPacket(sock, cp)) {
            char ack[4];
            if (recvExact(sock, ack, 4, 3000) && (ack[0] >> 4) == 2 && ack[3] == 0) {
                ok = sendPacket(sock, pkt);
            }
        }
    }
    closesocket(sock);
    return ok;
}

} // namespace relay
