// 关闭 Go2-W 原厂运控服务(代替手机 App),之后才能用 go2w_ctrl 发 lowcmd。
//
// 流程照 unitree_sdk2 example/go2w/go2w_stand_example.cpp:
//   CheckMode -> (StandDown 趴下) -> ReleaseMode 循环直到 CheckMode 确认已关闭
// 运控一关所有关节立刻失力,站着关会直接摔 —— 所以先 StandDown,并要人确认
// 狗已趴下或吊起(-y 跳过确认)。原厂运控每次上电都会重新拉起,每次开机都要关一次。
//
// 用法: release_mode [网卡=eth0] [-y]
// 退出码: 0 = 运控已关闭, 1 = 失败
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using unitree::robot::b2::MotionSwitcherClient;

// 返回 1 = 运控在运行, 0 = 已关闭, -1 = 查询失败(超时等;此时 name 也为空,不能当成已关闭)
static int query(MotionSwitcherClient& msc)
{
    std::string form, name;
    int32_t ret = msc.CheckMode(form, name);
    if(ret != 0) {
        std::cout << "  CheckMode 失败, 错误码 " << ret << std::endl;
        return -1;
    }
    if(name.empty()) return 0;
    std::cout << "  运控运行中: form=" << form << " name=" << name << std::endl;
    return 1;
}

int main(int argc, char** argv)
{
    std::string iface = "eth0";
    bool yes = false;
    for(int i = 1; i < argc; ++i) {
        if(std::strcmp(argv[i], "-y") == 0) yes = true;
        else iface = argv[i];
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    MotionSwitcherClient msc;
    msc.SetTimeout(5.0f);
    msc.Init();

    int st = -1;
    for(int i = 0; i < 3 && st < 0; ++i) {
        st = query(msc);
        if(st < 0) sleep(1);
    }
    if(st < 0) {
        std::cout << "查询运控状态失败(网卡 " << iface << " 对吗?狗开机了吗?)" << std::endl;
        return 1;
    }
    if(st == 0) {
        std::cout << "运控已是关闭状态" << std::endl;
        return 0;
    }

    // 先趴下。Go2-W 的 StandDown 只在站立时成功,已经趴着会返回错误,忽略即可
    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(5.0f);
    sport.Init();
    std::cout << "StandDown -> " << sport.StandDown() << " (已趴着时非 0 属正常)" << std::endl;
    sleep(3);

    if(!yes) {
        std::cout << "运控关闭后关节立即失力。确认狗已趴下或吊起后按回车继续, Ctrl+C 取消" << std::endl;
        std::string line;
        std::getline(std::cin, line);
    }

    for(int i = 0; i < 6; ++i) {
        std::cout << "ReleaseMode -> " << msc.ReleaseMode() << std::endl;
        sleep(3);
        if(query(msc) == 0) {
            std::cout << "运控已关闭" << std::endl;
            return 0;
        }
    }
    std::cout << "关闭运控失败" << std::endl;
    return 1;
}
