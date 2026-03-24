#pragma once

#include <vector>
#include <string>
#include <memory>

namespace minimal {

/**
 * @brief Policy network wrapper using TensorRT
 *
 * Handles model loading and inference for the legged locomotion policy.
 * Input: 450-dimensional observation vector (10 frames × 45 dims)
 * Output: 12-dimensional action vector
 */
class Policy {
public:
    static constexpr int kInputDim = 450;   // 10 frames × 45 dims
    static constexpr int kOutputDim = 12;

    /**
     * @brief Load policy from TensorRT engine file
     * @param engine_path Path to .engine file
     */
    explicit Policy(const std::string& engine_path);

    ~Policy();

    /**
     * @brief Run policy inference
     * @param obs_450 Input observation vector (must be 450 dimensions)
     * @return Action vector (12 dimensions), empty on error
     */
    std::vector<float> Run(const std::vector<float>& obs_450) const;

    /**
     * @brief Check if policy is valid (loaded successfully)
     */
    bool IsValid() const { return valid_; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    bool valid_;
};

}  // namespace minimal
