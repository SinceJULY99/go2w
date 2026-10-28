#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}
void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();

    // 哪几个动作是轮子(速度目标)、哪几个是腿(位置目标),按 deploy.yaml 顶层的
    // wheel_ids 判定,不再硬编码"前 12 位置、后 4 速度"。
    //
    // 原实现假设策略输出是"12 条腿 + 4 个轮",这对 robot_lab / IsaacLab 训练的策略
    // 成立(它的 joint_ids_map 是 identity,策略序 == 硬件序)。而 legged_gym
    // (DreamWaQ)按 URDF 关节顺序编号,策略序是 FL(髋,大腿,小腿,轮) FR(...)
    // RL(...) RR(...),轮子交错在 3/7/11/15 —— 按原逻辑会把轮子的速度目标当成
    // 髋关节角度发出去。
    //
    // 注意:joint_ids_map 只负责"策略序 -> 硬件序"的搬运,而位置/速度的语义判定
    // 必须按【策略索引】来。所以下面用 i 查 is_wheel,用 joint_ids_map[i] 定电机。
    //
    // 本进程只有一个 State_RLBase 实例,故用函数内 static 只解析一次配置。
    static const std::vector<bool> is_wheel = [this, &action] {
        std::vector<bool> mask(action.size(), false);
        if(!env->cfg["wheel_ids"]) {
            throw std::runtime_error("deploy.yaml is missing the top-level 'wheel_ids' list");
        }
        for(int id : env->cfg["wheel_ids"].as<std::vector<int>>()) {
            if(id < 0 || id >= (int)mask.size()) {
                throw std::runtime_error("wheel_ids contains an out-of-range action index");
            }
            mask[id] = true;
        }
        return mask;
    }();

    for(int i(0); i < (int)action.size(); i++) {
        auto & motor = lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]];
        if(is_wheel[i]) {
            motor.dq() = action[i];   // 轮:速度目标
        } else {
            motor.q()  = action[i];   // 腿:位置目标
        }
    }
}