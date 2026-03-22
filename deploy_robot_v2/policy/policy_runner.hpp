#pragma once
#include <vector>
#include <memory>
#include <string>
#include "tensorrt_inference.hpp"

/**
 * @brief Policy 输出结构
 */
struct PolicyOutput {
    std::vector<float> raw_action;      // 原始网络输出 (12维)
    std::vector<float> desired_abs;     // 期望绝对角度 (12维)
    std::vector<float> clipped_abs;     // 裁剪后的绝对角度 (12维)
};

/**
 * @brief Policy 推理包装器
 * 
 * 封装 TensorRT 推理引擎，提供：
 * - 原始 action 输出
 * - 转换为绝对角度
 * - 关节限位裁剪
 * - 膝关节减速比处理
 */
class PolicyRunner {
public:
    /**
     * @brief 构造函数
     * @param engine_path TensorRT engine 文件路径
     */
    explicit PolicyRunner(const std::string& engine_path);
    ~PolicyRunner() = default;

    /**
     * @brief 运行推理
     * @param obs_450 450维观测输入 (10帧历史)
     * @return Policy 输出结构
     */
    PolicyOutput Run(const std::vector<float>& obs_450) const;

    /**
     * @brief 获取推理引擎指针 (用于调试)
     */
    InferenceEngine* GetEngine() { return engine_.get(); }

private:
    std::unique_ptr<InferenceEngine> engine_;
};