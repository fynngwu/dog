#include "policy_runner.hpp"
#include "robot_config.hpp"
#include <algorithm>

PolicyRunner::PolicyRunner(const std::string& engine_path) {
    engine_ = std::make_unique<InferenceEngine>(
        engine_path, cfg::kPolicyInputDim, cfg::kPolicyOutputDim);
    std::cout << "[PolicyRunner] Loaded engine: " << engine_path << std::endl;
}

PolicyOutput PolicyRunner::Run(const std::vector<float>& obs_450) const {
    PolicyOutput out;
    out.raw_action.resize(cfg::kPolicyOutputDim, 0.0f);
    
    // 执行推理
    engine_->infer(obs_450, out.raw_action);

    out.desired_abs.resize(cfg::kPolicyOutputDim, 0.0f);
    out.clipped_abs.resize(cfg::kPolicyOutputDim, 0.0f);

    // 转换为绝对角度并裁剪
    for (int i = 0; i < cfg::kPolicyOutputDim; ++i) {
        float act = out.raw_action[i];

        // 膝关节减速比处理
        if (i >= 8 && i <= 11) {
            act *= cfg::kKneeRatio;
        }

        // 计算期望绝对角度: sign * action * scale + offset
        float sign = cfg::kJointDirection[i];
        float desired = sign * act * cfg::kActionScale + cfg::kJointOffsets[i];
        
        // 计算限位范围
        float lower = cfg::kJointOffsets[i] + cfg::kXmlMin[i];
        float upper = cfg::kJointOffsets[i] + cfg::kXmlMax[i];

        out.desired_abs[i] = desired;
        out.clipped_abs[i] = std::clamp(desired, lower, upper);
    }
    
    return out;
}