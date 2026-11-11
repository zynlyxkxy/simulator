// relay_json.h - 极简 JSON 解析/序列化(纯标准库,单头)。
// 用于:28 模型 HTTP 响应解析(/api/axes 等)、MQTT 载荷构造。
// 不保证性能,仅保证正确性。对象键保序(用 vector<pair>),便于按需输出。
#pragma once
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <algorithm>

namespace relay {

class Json {
public:
    enum class Type { Null, Bool, Num, Str, Arr, Obj };
    using Pair = std::pair<std::string, Json>;

    Json() : type_(Type::Null) {}
    static Json null() { return Json(); }
    static Json boolean(bool b) { Json j; j.type_ = Type::Bool; j.b_ = b; return j; }
    static Json number(double d) { Json j; j.type_ = Type::Num; j.n_ = d; return j; }
    static Json string(const std::string& s) { Json j; j.type_ = Type::Str; j.s_ = s; return j; }
    static Json array() { Json j; j.type_ = Type::Arr; return j; }
    static Json object() { Json j; j.type_ = Type::Obj; return j; }

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isNumber() const { return type_ == Type::Num; }
    bool isString() const { return type_ == Type::Str; }
    bool isArray() const { return type_ == Type::Arr; }
    bool isObject() const { return type_ == Type::Obj; }

    bool asBool(bool def = false) const { return type_ == Type::Bool ? b_ : def; }
    double asNumber(double def = 0.0) const { return type_ == Type::Num ? n_ : def; }
    long long asInt64(long long def = 0) const { return type_ == Type::Num ? (long long)n_ : def; }
    const std::string& asString() const { return s_; }
    std::string asString(const std::string& def) const { return type_ == Type::Str ? s_ : def; }

    std::vector<Json>& arr() { return a_; }
    const std::vector<Json>& arr() const { return a_; }
    std::vector<Pair>& obj() { return o_; }
    const std::vector<Pair>& obj() const { return o_; }

    size_t size() const {
        if (type_ == Type::Arr) return a_.size();
        if (type_ == Type::Obj) return o_.size();
        return 0;
    }

    // 数组下标 / 对象键访问(无则返回 null)
    const Json& at(size_t i) const {
        static const Json kNull;
        if (type_ == Type::Arr && i < a_.size()) return a_[i];
        return kNull;
    }
    const Json& at(const std::string& key) const {
        static const Json kNull;
        if (type_ != Type::Obj) return kNull;
        for (const auto& p : o_) if (p.first == key) return p.second;
        return kNull;
    }
    // 写访问
    Json& operator[](size_t i) { return a_[i]; }
    Json& operator[](const std::string& key) {
        for (auto& p : o_) if (p.first == key) return p.second;
        o_.emplace_back(key, Json());
        return o_.back().second;
    }
    void push_back(const Json& v) { a_.push_back(v); }
    bool has(const std::string& key) const {
        if (type_ != Type::Obj) return false;
        for (const auto& p : o_) if (p.first == key) return true;
        return false;
    }
    // 递归查找 first key (浅层惯例用)
    const Json* findPath(const std::vector<std::string>& keys) const {
        const Json* cur = this;
        for (const auto& k : keys) {
            if (!cur->isObject()) return nullptr;
            bool found = false;
            for (const auto& p : cur->o_) {
                if (p.first == k) { cur = &p.second; found = true; break; }
            }
            if (!found) return nullptr;
        }
        return cur;
    }

    // ---------- 解析 ----------
    static Json parse(const std::string& text) {
        Parser p(text);
        Json v = p.parseValue();
        p.skipWs();
        if (!p.eof()) throw std::runtime_error("JSON: trailing data");
        return v;
    }

    // ---------- 序列化 ----------
    std::string dump() const {
        std::string out;
        dumpTo(out);
        return out;
    }

private:
    Type type_;
    bool b_ = false;
    double n_ = 0.0;
    std::string s_;
    std::vector<Json> a_;
    std::vector<Pair> o_;

    static void escapeTo(const std::string& s, std::string& out) {
        out.push_back('"');
        for (char c : s) {
            switch (c) {
                case '"': out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                case '\b': out += "\\b"; break;
                case '\f': out += "\\f"; break;
                default:
                    if ((unsigned char)c < 0x20) {
                        char buf[8];
                        snprintf(buf, sizeof buf, "\\u%04x", (unsigned char)c);
                        out += buf;
                    } else out.push_back(c);
            }
        }
        out.push_back('"');
    }

    void dumpTo(std::string& out) const {
        switch (type_) {
            case Type::Null: out += "null"; break;
            case Type::Bool: out += (b_ ? "true" : "false"); break;
            case Type::Num: {
                if (n_ == (long long)n_ && std::fabs(n_) < 1e15) {
                    char buf[32];
                    snprintf(buf, sizeof buf, "%lld", (long long)n_);
                    out += buf;
                } else {
                    std::ostringstream oss;
                    oss.precision(15);
                    oss << n_;
                    out += oss.str();
                }
                break;
            }
            case Type::Str: escapeTo(s_, out); break;
            case Type::Arr: {
                out.push_back('[');
                for (size_t i = 0; i < a_.size(); i++) {
                    if (i) out.push_back(',');
                    a_[i].dumpTo(out);
                }
                out.push_back(']');
                break;
            }
            case Type::Obj: {
                out.push_back('{');
                for (size_t i = 0; i < o_.size(); i++) {
                    if (i) out.push_back(',');
                    escapeTo(o_[i].first, out);
                    out.push_back(':');
                    o_[i].second.dumpTo(out);
                }
                out.push_back('}');
                break;
            }
        }
    }

    struct Parser {
        const std::string& s;
        size_t i = 0;
        explicit Parser(const std::string& text) : s(text) {}
        bool eof() const { return i >= s.size(); }
        char peek() const { return i < s.size() ? s[i] : '\0'; }
        void skipWs() {
            while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;
        }
        bool consume(char c) {
            if (peek() == c) { i++; return true; }
            return false;
        }
        [[noreturn]] void fail(const std::string& msg) const {
            throw std::runtime_error("JSON parse error at " + std::to_string(i) + ": " + msg);
        }
        Json parseValue() {
            skipWs();
            if (eof()) fail("unexpected end");
            char c = peek();
            if (c == '{') return parseObject();
            if (c == '[') return parseArray();
            if (c == '"') return Json::string(parseString());
            if (c == 't') { expect("true"); return Json::boolean(true); }
            if (c == 'f') { expect("false"); return Json::boolean(false); }
            if (c == 'n') { expect("null"); return Json::null(); }
            if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
            fail("unexpected char");
        }
        void expect(const char* lit) {
            size_t n = 0;
            while (lit[n]) { if (i + n >= s.size() || s[i + n] != lit[n]) fail("bad literal"); n++; }
            i += n;
        }
        std::string parseString() {
            if (!consume('"')) fail("expect string");
            std::string out;
            while (true) {
                if (eof()) fail("unterminated string");
                char c = s[i++];
                if (c == '"') break;
                if (c == '\\') {
                    if (eof()) fail("bad escape");
                    char e = s[i++];
                    switch (e) {
                        case '"': out.push_back('"'); break;
                        case '\\': out.push_back('\\'); break;
                        case '/': out.push_back('/'); break;
                        case 'n': out.push_back('\n'); break;
                        case 'r': out.push_back('\r'); break;
                        case 't': out.push_back('\t'); break;
                        case 'b': out.push_back('\b'); break;
                        case 'f': out.push_back('\f'); break;
                        case 'u': {
                            if (i + 4 > s.size()) fail("bad \\u");
                            unsigned code = 0;
                            for (int k = 0; k < 4; k++) code = code * 16 + hexVal(s[i++]);
                            // 代理对(基本够用)
                            if (code >= 0xD800 && code <= 0xDBFF && i + 6 <= s.size() &&
                                s[i] == '\\' && s[i + 1] == 'u') {
                                i += 2;
                                unsigned lo = 0;
                                for (int k = 0; k < 4; k++) lo = lo * 16 + hexVal(s[i++]);
                                code = 0x10000 + ((code - 0xD800) << 10) + (lo - 0xDC00);
                            }
                            appendUtf8(out, code);
                            break;
                        }
                        default: fail("bad escape char");
                    }
                } else out.push_back(c);
            }
            return out;
        }
        static int hexVal(char c) {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            throw std::runtime_error("bad hex");
        }
        static void appendUtf8(std::string& out, unsigned code) {
            if (code < 0x80) out.push_back((char)code);
            else if (code < 0x800) {
                out.push_back((char)(0xC0 | (code >> 6)));
                out.push_back((char)(0x80 | (code & 0x3F)));
            } else if (code < 0x10000) {
                out.push_back((char)(0xE0 | (code >> 12)));
                out.push_back((char)(0x80 | ((code >> 6) & 0x3F)));
                out.push_back((char)(0x80 | (code & 0x3F)));
            } else {
                out.push_back((char)(0xF0 | (code >> 18)));
                out.push_back((char)(0x80 | ((code >> 12) & 0x3F)));
                out.push_back((char)(0x80 | ((code >> 6) & 0x3F)));
                out.push_back((char)(0x80 | (code & 0x3F)));
            }
        }
        Json parseNumber() {
            size_t start = i;
            if (peek() == '-') i++;
            while (i < s.size() && (isdigit((unsigned char)s[i]) || s[i] == '.' || s[i] == 'e' || s[i] == 'E' || s[i] == '+' || s[i] == '-')) i++;
            return Json::number(strtod(s.substr(start, i - start).c_str(), nullptr));
        }
        Json parseObject() {
            consume('{');
            Json obj = Json::object();
            skipWs();
            if (consume('}')) return obj;
            while (true) {
                skipWs();
                std::string key = parseString();
                skipWs();
                if (!consume(':')) fail("expect ':'");
                obj[key] = parseValue();
                skipWs();
                if (consume('}')) break;
                if (!consume(',')) fail("expect ',' or '}'");
            }
            return obj;
        }
        Json parseArray() {
            consume('[');
            Json arr = Json::array();
            skipWs();
            if (consume(']')) return arr;
            while (true) {
                arr.push_back(parseValue());
                skipWs();
                if (consume(']')) break;
                if (!consume(',')) fail("expect ',' or ']'");
            }
            return arr;
        }
    };
};

// 便捷:从 obj 取数值/字符串数组/字符串成员
inline double jnum(const Json& o, const char* key, double def = 0.0) {
    return o.at(key).asNumber(def);
}
inline bool jbool(const Json& o, const char* key, bool def = false) {
    return o.at(key).asBool(def);
}
inline std::string jstr(const Json& o, const char* key, const std::string& def = "") {
    return o.at(key).asString(def);
}

} // namespace relay
