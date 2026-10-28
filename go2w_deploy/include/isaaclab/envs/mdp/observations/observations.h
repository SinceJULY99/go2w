// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;

    std::vector<int> joint_ids;
    try {
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
    } catch(const std::exception& e) {
    }

    if(joint_ids.empty())
    {
        data.resize(asset->data.joint_pos.size());
        for(size_t i = 0; i < asset->data.joint_pos.size(); ++i)
        {
            data[i] = asset->data.joint_pos[i];
        }
    }
    else
    {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i)
        {
            data[i] = asset->data.joint_pos[joint_ids[i]];
        }
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    try {
        std::vector<int> joint_ids;
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
        if(!joint_ids.empty()) {
            std::vector<float> tmp_data;
            tmp_data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i){
                tmp_data[i] = data[joint_ids[i]];
            }
            data = tmp_data;
        }
    } catch(const std::exception& e) {
    
    }

    return data;
}

// Q1 (wheeled quadruped) was trained with joint_pos_rel_without_wheel: the wheel
// (hub) joints are continuous, so their absolute angle is meaningless and was
// forced to 0 in the observation. Q1's 4 wheels are the last 4 joints.
REGISTER_OBSERVATION(joint_pos_rel_without_wheel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    // zero the last 4 (wheel) entries
    size_t n = data.size();
    for(size_t i = (n >= 4 ? n - 4 : 0); i < n; ++i) {
        data[i] = 0.0f;
    }

    return data;
}

// 同 joint_pos_rel_without_wheel,但轮子索引可配置,而不是写死"最后 4 个"。
//
// 上面那个假设轮子排在末尾,对 robot_lab / IsaacLab 训练的策略成立。而
// legged_gym(DreamWaQ)按 URDF 里关节出现的顺序编号,轮子跟在每条腿后面 ——
// go2w 的策略序是 FL(髋,大腿,小腿,轮) FR(...) RL(...) RR(...),轮子在 3/7/11/15。
// 用错版本不会报错,只是有 4 维该清零的没清、4 维不该清的被清零了。
//
// 索引读 deploy.yaml 的顶层 wheel_ids(不放 params,避免和 State_RLBase 判定
// 位置/速度时用的那份重复;做法与 velocity_commands 读 env->cfg["commands"] 一致)。
REGISTER_OBSERVATION(joint_pos_rel_wheel_ids)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    if(!env->cfg["wheel_ids"]) {
        throw std::runtime_error(
            "observation 'joint_pos_rel_wheel_ids' requires a top-level "
            "'wheel_ids' list in deploy.yaml");
    }
    for(int id : env->cfg["wheel_ids"].as<std::vector<int>>()) {
        if(id < 0 || id >= (int)data.size()) {
            throw std::runtime_error("wheel_ids contains an out-of-range joint index");
        }
        data[id] = 0.0f;
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel_rel)
{
    auto & asset = env->robot;
    auto data = asset->data.joint_vel;

    try {
        const std::vector<int> joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();

        if(!joint_ids.empty()) {
            data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i) {
                data[i] = asset->data.joint_vel[joint_ids[i]];
            }
        }
    } catch(const std::exception& e) {
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(last_action)
{
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

// 指令来源仲裁:摇杆有输入(任一轴 > 0.05)时摇杆优先 —— 人随时能接管;
// 否则有未过期的外部指令(SportClient::Move)就用外部的;都没有就用摇杆(≈0)。
// 两种来源都夹到 deploy.yaml 的 ranges,外部程序给超了也出不了训练分布。
REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & data = env->robot->data;
    auto & joystick = data.joystick;

    std::array<float, 3> cmd = {joystick->ly(), -joystick->lx(), -joystick->rx()};
    const bool stick_active = std::fabs(cmd[0]) > 0.05f || std::fabs(cmd[1]) > 0.05f || std::fabs(cmd[2]) > 0.05f;
    if(!stick_active && data.external_cmd_valid) {
        cmd = data.external_cmd;
    }

    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    obs[0] = std::clamp(cmd[0], cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(cmd[1], cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(cmd[2], cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

REGISTER_OBSERVATION(gait_phase)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);
    return obs;
}

}
}