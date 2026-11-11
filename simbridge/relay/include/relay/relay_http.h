// relay_http.h - 极简 HTTP/1.1 客户端(纯 WinSock,循环请求)。
// 用途:调用 28 模型 REST(9010 /api/*,9102 /io/command)与 MCS(9190)。
// 只支持本机/局域网明文 HTTP;无分块编码(28 模型返回体均小且不 chunked)。
#pragma once
#include <string>
#include <cstdint>
#include <stdexcept>
#include <sstream>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

namespace relay {

class HttpError : public std::runtime_error {
public:
    explicit HttpError(const std::string& m) : std::runtime_error(m) {}
};

struct HttpResponse {
    int status = 0;
    std::string body;
    bool ok() const { return status >= 200 && status < 300; }
};

// 一次性 TCP 连接执行请求(简单、无连接复用;本机调用频率低,足够)
class HttpClient {
public:
    HttpClient(const std::string& host, int port, int timeoutMs = 3000)
        : host_(host), port_(port), timeoutMs_(timeoutMs) {
        ensureWsa();
    }

    HttpResponse get(const std::string& pathAndQuery) {
        return request("GET", pathAndQuery, "", {});
    }
    HttpResponse postJson(const std::string& path, const std::string& jsonBody) {
        std::map<std::string, std::string> h;
        h["Content-Type"] = "application/json";
        return request("POST", path, jsonBody, h);
    }

    HttpResponse request(const std::string& method, const std::string& path,
                         const std::string& body,
                         const std::map<std::string, std::string>& extraHeaders) {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) throw HttpError("socket failed");
        try {
            setRecvTimeout(sock, timeoutMs_);
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons((unsigned short)port_);
            if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
                // 允许域名
                hostent* he = gethostbyname(host_.c_str());
                if (!he) throw HttpError("dns fail: " + host_);
                memcpy(&addr.sin_addr, he->h_addr, he->h_length);
            }
            if (connect(sock, (sockaddr*)&addr, sizeof addr) != 0)
                throw HttpError("connect " + host_ + ":" + std::to_string(port_) + " failed err=" + std::to_string(WSAGetLastError()));

            std::string req;
            req += method + " " + path + " HTTP/1.1\r\n";
            req += "Host: " + host_ + ":" + std::to_string(port_) + "\r\n";
            req += "User-Agent: relay/0.1\r\n";
            req += "Accept: */*\r\n";
            req += "Connection: close\r\n";
            for (auto& kv : extraHeaders) req += kv.first + ": " + kv.second + "\r\n";
            if (!body.empty()) req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
            req += "\r\n";
            req += body;
            sendAll(sock, req);

            std::string raw = recvAll(sock);
            closesocket(sock);
            return parse(raw);
        } catch (...) {
            closesocket(sock);
            throw;
        }
    }

private:
    std::string host_;
    int port_;
    int timeoutMs_;

    static bool wsaReady_;
    static void ensureWsa() {
        if (wsaReady_) return;
        WSADATA wd;
        if (WSAStartup(MAKEWORD(2, 2), &wd) != 0) throw HttpError("WSAStartup failed");
        wsaReady_ = true;
    }
    static void setRecvTimeout(SOCKET s, int ms) {
        DWORD t = (DWORD)ms;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&t, sizeof t);
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char*)&t, sizeof t);
    }
    static void sendAll(SOCKET s, const std::string& data) {
        size_t off = 0;
        while (off < data.size()) {
            int n = send(s, data.data() + off, (int)(data.size() - off), 0);
            if (n <= 0) throw HttpError("send failed");
            off += (size_t)n;
        }
    }
    static std::string recvAll(SOCKET s) {
        std::string out;
        char buf[8192];
        while (true) {
            int n = recv(s, buf, sizeof buf, 0);
            if (n > 0) out.append(buf, (size_t)n);
            else if (n == 0) break;
            else {
                int e = WSAGetLastError();
                if (e == WSAETIMEDOUT) break; // 超时当读完(服务端应已 close)
                break;
            }
        }
        return out;
    }
    static HttpResponse parse(const std::string& raw) {
        HttpResponse r;
        size_t hEnd = raw.find("\r\n\r\n");
        if (hEnd == std::string::npos) { hEnd = raw.find("\n\n"); }
        std::string head = raw.substr(0, hEnd);
        std::string body;
        if (hEnd != std::string::npos) body = raw.substr(hEnd + 4);
        // 去 chunked(万一)
        if (head.find("Transfer-Encoding: chunked") != std::string::npos ||
            head.find("transfer-encoding: chunked") != std::string::npos) {
            std::string dec;
            size_t p = 0;
            while (p < body.size()) {
                size_t eol = body.find("\r\n", p);
                if (eol == std::string::npos) break;
                size_t len = strtoul(body.substr(p, eol - p).c_str(), nullptr, 16);
                if (len == 0) break;
                dec.append(body, eol + 2, len);
                p = eol + 2 + len + 2;
            }
            body = dec;
        }
        std::istringstream hs(head);
        std::string line;
        if (std::getline(hs, line)) {
            if (line.size() >= 9 && line.compare(0, 7, "HTTP/1.") == 0) {
                r.status = atoi(line.substr(9).c_str());
            }
        }
        r.body = body;
        return r;
    }
};
bool HttpClient::wsaReady_ = false;

} // namespace relay
