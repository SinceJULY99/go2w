// 冒充原厂运控服务的 Move 接口,让外部程序(避障等)不改代码就能给 RL 策略下速度指令。
//
// 背景:跑本控制器必须先 ReleaseMode 关掉原厂运控,之后 SportClient::Move() 发到
// rt/api/sport/request 的请求就没人接了。这里订阅同一个 topic,只认两个 api:
//   1008 Move(vx, vy, vyaw)  参数 JSON {"x":..,"y":..,"z":..},机体系,和策略指令同号
//   1003 StopMove            立即归零
// 其余 api(StandUp / Damp 等)回 3203(UT_ROBOT_ERR_SERVER_API_NOT_IMPL)并打警告 ——
// 站起、急停仍走遥控器。请求要求回复(policy.noreply == false)时回 code 0,否则
// 客户端的 Move() 会一直阻塞到超时。
//
// 只负责收发和记录"最新一条指令 + 时间",新鲜度判定和与摇杆的仲裁在调用方。
#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <regex>
#include <set>
#include <string>

#include <spdlog/spdlog.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_api.hpp>
#include <unitree/robot/internal/internal_request_response.hpp>

namespace unitree
{

class SportMoveBridge
{
public:
    // timeout_s:超过这么久没收到 Move 就视为无外部指令(避障程序挂掉时自动停)
    explicit SportMoveBridge(double timeout_s = 0.5)
    : timeout_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_s)))
    {
        pub_ = std::make_shared<robot::ChannelPublisher<robot::Response>>("rt/api/sport/response");
        pub_->InitChannel();
        sub_ = std::make_shared<robot::ChannelSubscriber<robot::Request>>("rt/api/sport/request");
        sub_->InitChannel([this](const void* msg) { on_request(*(const robot::Request*)msg); });
        spdlog::info("SportMoveBridge: listening rt/api/sport/request (Move/StopMove), timeout {:.2f}s", timeout_s);
    }

    // 有未过期的外部指令时写入 cmd(vx, vy, wz)并返回 true
    bool get(std::array<float, 3>& cmd)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(std::chrono::steady_clock::now() - stamp_ > timeout_) return false;
        cmd = cmd_;
        return true;
    }

private:
    void on_request(const robot::Request& req)
    {
        const auto api_id = req.header().identity().api_id();
        int32_t code = 0;

        if(api_id == robot::go2::ROBOT_SPORT_API_ID_MOVE) {
            std::array<float, 3> v;
            if(parse_move(req.parameter(), v)) {
                set(v);
            } else {
                spdlog::warn("SportMoveBridge: bad Move parameter: {}", req.parameter());
                code = 3203;
            }
        } else if(api_id == robot::go2::ROBOT_SPORT_API_ID_STOPMOVE) {
            set({0.f, 0.f, 0.f});
        } else {
            code = 3203;
            if(warned_.insert(api_id).second) {  // 每种 api 只警告一次,免得刷屏
                spdlog::warn("SportMoveBridge: sport api {} not supported (only Move/StopMove)", api_id);
            }
        }

        if(!req.header().policy().noreply()) {
            robot::Response resp;
            resp.header().identity(req.header().identity());
            resp.header().status().code(code);
            pub_->Write(resp);
        }
    }

    void set(const std::array<float, 3>& v)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = v;
        stamp_ = std::chrono::steady_clock::now();
    }

    // SDK 发出的参数形如 {"x":0.3,"y":0,"z":-0.2};三个键缺一不可
    static bool parse_move(const std::string& s, std::array<float, 3>& v)
    {
        static const char* keys[3] = {"x", "y", "z"};
        for(int i = 0; i < 3; ++i) {
            std::smatch m;
            const std::regex re(std::string("\"") + keys[i] + "\"\\s*:\\s*([-+0-9.eE]+)");
            if(!std::regex_search(s, m, re)) return false;
            try { v[i] = std::stof(m[1].str()); } catch(...) { return false; }
        }
        return true;
    }

    robot::ChannelPublisherPtr<robot::Response> pub_;
    robot::ChannelSubscriberPtr<robot::Request> sub_;
    std::mutex mutex_;
    std::array<float, 3> cmd_{0.f, 0.f, 0.f};
    std::chrono::steady_clock::time_point stamp_{};  // 默认值远早于 now(),启动即"过期"
    std::chrono::steady_clock::duration timeout_;
    std::set<int64_t> warned_;  // 只在 DDS 回调线程里访问
};

}
