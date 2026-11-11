#include "relay/relay_sim.h"
#include "relay/relay_http.h"
#include "relay/relay_util.h"
#include <sstream>

namespace relay {

void SimClient::start(const std::string& host, int port, int pollMs) {
    host_ = host;
    port_ = port;
    pollMs_ = pollMs;
    if (running_.exchange(true)) return;
    pollThread_ = std::thread([this] { pollLoop(); });
}

void SimClient::stop() {
    if (!running_.exchange(false)) return;
    if (pollThread_.joinable()) pollThread_.join();
}

void SimClient::pollLoop() {
    while (running_) {
        refreshAxes();
        int step = 50;
        int waited = 0;
        while (running_ && waited < pollMs_) { sleepMs(step); waited += step; }
    }
}

Json SimClient::httpGet(const std::string& pathAndQuery) {
    try {
        HttpClient cl(host_, port_, 3000);
        auto r = cl.get(pathAndQuery);
        if (r.status == 0) { lastError_ = "http err " + pathAndQuery; return Json::null(); }
        if (r.body.empty()) return Json::null();
        return Json::parse(r.body);
    } catch (const std::exception& e) {
        lastError_ = e.what();
        return Json::null();
    }
}

Json SimClient::httpPostJson(const std::string& path, const std::string& body) {
    try {
        HttpClient cl(host_, port_, 3000);
        auto r = cl.postJson(path, body);
        if (r.status == 0) { lastError_ = "http err " + path; return Json::null(); }
        if (r.body.empty()) return Json::null();
        return Json::parse(r.body);
    } catch (const std::exception& e) {
        lastError_ = e.what();
        return Json::null();
    }
}

bool SimClient::refreshAxes() {
    Json j = httpGet("/api/axes");
    if (!j.isObject() || !j.at("axes").isArray()) {
        lastError_ = "/api/axes bad resp";
        return false;
    }
    std::vector<AxisSnap> tmp;
    for (const auto& a : j.at("axes").arr()) {
        AxisSnap s;
        s.name = jstr(a, "name");
        s.pos = jnum(a, "pos");
        s.vel = jnum(a, "vel");
        s.enabled = jbool(a, "enabled", false);
        s.moving = jbool(a, "moving", false);
        s.inPos = jbool(a, "in_position", true);
        s.fault = jbool(a, "fault", false);
        s.state = jstr(a, "state", "idle");
        s.homeState = jstr(a, "home_state", "done");
        tmp.push_back(std::move(s));
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        axes_ = std::move(tmp);
        lastPollAt_ = nowMs();
    }
    lastError_.clear();
    return true;
}

std::vector<AxisSnap> SimClient::snapshotAxes() const {
    std::lock_guard<std::mutex> lk(mu_);
    return axes_;
}

bool SimClient::getAxis(const std::string& name, AxisSnap& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto& a : axes_) if (a.name == name) { out = a; return true; }
    return false;
}

Json SimClient::axesJson() {
    return httpGet("/api/axes");
}

bool SimClient::axisMove(const std::string& axis, double target, double speed, double* durationMs) {
    Json body = Json::object();
    body["axis"] = Json::string(axis);
    body["target"] = Json::number(target);
    if (speed > 0) body["speed"] = Json::number(speed);
    Json j = httpPostJson("/api/axis/move", body.dump());
    if (!j.isObject() || !jbool(j, "ok", false)) {
        lastError_ = "axisMove " + axis + " rejected: " + j.dump();
        return false;
    }
    if (durationMs && j.has("duration")) *durationMs = jnum(j, "duration") * 1000.0;
    return true;
}

bool SimClient::axisJog(const std::string& axis, double delta, double speed) {
    Json body = Json::object();
    body["axis"] = Json::string(axis);
    body["delta"] = Json::number(delta);
    if (speed > 0) body["speed"] = Json::number(speed);
    Json j = httpPostJson("/api/axis/jog", body.dump());
    if (!j.isObject() || !jbool(j, "ok", false)) {
        lastError_ = "axisJog " + axis + " rejected: " + j.dump();
        return false;
    }
    return true;
}

bool SimClient::axisHome(const std::string& axis, int mode) {
    Json body = Json::object();
    body["axis"] = Json::string(axis);
    if (mode > 0) body["mode"] = Json::number((double)mode);
    Json j = httpPostJson("/api/axis/home", body.dump());
    if (!j.isObject() || !jbool(j, "ok", false)) {
        lastError_ = "axisHome " + axis + " rejected: " + j.dump();
        return false;
    }
    return true;
}

bool SimClient::axisSetSpeed(const std::string& axis, double vmax) {
    Json body = Json::object();
    body["axis"] = Json::string(axis);
    body["vmax"] = Json::number(vmax);
    Json j = httpPostJson("/api/axis/config", body.dump());
    if (!j.isObject() || !jbool(j, "ok", false)) {
        lastError_ = "axisSetSpeed " + axis + " rejected: " + j.dump();
        return false;
    }
    return true;
}

bool SimClient::acsCommand(const Json& req, Json& resp, int timeoutMs) {
    std::string line = req.dump() + "\n";
    std::string out;
    try {
        // WSA
        WSADATA wd;
        if (WSAStartup(MAKEWORD(2, 2), &wd) != 0) { lastError_ = "WSAStartup"; return false; }
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) { lastError_ = "socket"; return false; }
        DWORD t = (DWORD)timeoutMs;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&t, sizeof t);
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char*)&t, sizeof t);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(9101);
        inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
        if (connect(s, (sockaddr*)&addr, sizeof addr) != 0) {
            closesocket(s);
            lastError_ = "acs connect fail " + std::to_string(WSAGetLastError());
            return false;
        }
        send(s, line.data(), (int)line.size(), 0);
        char buf[4096];
        int n = recv(s, buf, sizeof buf - 1, 0);
        closesocket(s);
        if (n <= 0) { lastError_ = "acs recv fail"; return false; }
        out.assign(buf, (size_t)n);
        if (out.back() == '\n') out.pop_back();
        resp = Json::parse(out);
        return true;
    } catch (const std::exception& e) {
        lastError_ = e.what();
        return false;
    }
}

Json SimClient::ioCommand(const std::string& path, const std::string& cmd, const Json& args) {
    Json body = Json::object();
    body["path"] = Json::string(path);
    body["cmd"] = Json::string(cmd);
    if (args.isObject()) body["args"] = args;
    Json j = httpPostJson("/api/io/command", body.dump());
    if (j.isNull()) lastError_ = "ioCommand failed: " + path;
    return j;
}

Json SimClient::ioObserve(const std::string& path) {
    try {
        HttpClient cl(host_, 9102, 3000);
        std::string q = path;
        std::string enc;
        // URL-encode 简单版(路径含 '.' '/' 保留)
        auto r = cl.get("/io/observe?path=" + path);
        if (r.status == 0 || r.body.empty()) { lastError_ = "ioObserve err " + path; return Json::null(); }
        return Json::parse(r.body);
    } catch (const std::exception& e) {
        lastError_ = e.what();
        return Json::null();
    }
}

Json SimClient::ioSnapshot() {
    // 走 9010 主网关(GET /api/io 与 9102 /io/snapshot 同源),避免再开一条到 9102 的连接
    return httpGet("/api/io");
}

Json SimClient::filmAction(const std::string& action) {
    Json body = Json::object();
    body["action"] = Json::string(action);
    Json j = httpPostJson("/api/workpiece/film", body.dump());
    if (j.isNull()) lastError_ = "filmAction " + action + " failed";
    return j;
}

Json SimClient::confocalRead(const std::string& probe) {
    return httpGet("/api/confocal/read?probe=" + probe);
}

} // namespace relay
