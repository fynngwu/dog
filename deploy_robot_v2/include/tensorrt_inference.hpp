#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <NvOnnxParser.h>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include "observations.hpp"

class InferenceEngine {
public:
    InferenceEngine(const std::string& model_path, int input_dim_, int output_dim_);
    ~InferenceEngine();
    void infer(const std::vector<float>& input, std::vector<float>& output);
private:
    int input_dim_;
    int output_dim_;
    
    std::vector<float> input_buffer_;
    std::vector<float> output_buffer_;

    void* device_buffers_[2];

    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;

    class TensorRTLogger : public nvinfer1::ILogger {
        void log(Severity severity, const char* msg) noexcept override {
            if (severity <= Severity::kWARNING)
                std::cout << msg << std::endl;
        }
    } logger_;
};