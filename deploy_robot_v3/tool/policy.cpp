#include "policy.hpp"
#include "tensorrt_inference.hpp"
#include <iostream>

namespace minimal {

// Pimpl implementation using v2's InferenceEngine
class Policy::Impl {
public:
    Impl(const std::string& engine_path, int input_dim, int output_dim)
        : engine_(engine_path, input_dim, output_dim) {}

    void infer(const std::vector<float>& input, std::vector<float>& output) {
        engine_.infer(input, output);
    }

private:
    InferenceEngine engine_;
};

Policy::Policy(const std::string& engine_path)
    : impl_(nullptr), valid_(false) {
    try {
        impl_ = std::make_unique<Impl>(engine_path, kInputDim, kOutputDim);
        valid_ = true;
        std::cout << "[Policy] Engine loaded: " << engine_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Policy] Failed to load engine: " << e.what() << std::endl;
        valid_ = false;
    }
}

Policy::~Policy() = default;

std::vector<float> Policy::Run(const std::vector<float>& obs_450,
                               PolicyStatus* status) const {
    if (!valid_) {
        std::cerr << "[Policy] Invalid policy, returning zeros" << std::endl;
        if (status) *status = PolicyStatus::kInvalid;
        return std::vector<float>(kOutputDim, 0.0f);
    }

    if (obs_450.size() != static_cast<size_t>(kInputDim)) {
        std::cerr << "[Policy] Input size mismatch: expected " << kInputDim
                  << ", got " << obs_450.size() << std::endl;
        if (status) *status = PolicyStatus::kInputError;
        return std::vector<float>(kOutputDim, 0.0f);
    }

    std::vector<float> output(kOutputDim, 0.0f);
    try {
        impl_->infer(obs_450, output);
        if (status) *status = PolicyStatus::kOk;
    } catch (const std::exception& e) {
        std::cerr << "[Policy] Inference failed: " << e.what() << std::endl;
        if (status) *status = PolicyStatus::kInferError;
        return std::vector<float>(kOutputDim, 0.0f);
    }

    return output;
}

}  // namespace minimal
