#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <cstdint>

/**
 * @brief 单帧记录结构
 */
struct FrameRecord {
    uint64_t tick = 0;              // 帧计数
    double t_sec = 0.0;             // 从启动开始的秒数
    double loop_dt_ms = 0.0;        // 循环周期 (毫秒)
    double infer_ms = 0.0;          // 推理耗时 (毫秒)
    int history_valid = 0;          // 有效历史帧数
    int history_full = 0;           // 历史是否已满
    int control_enabled = 0;        // 控制是否启用

    std::vector<float> single_obs;      // 单帧观测 (45维)
    std::vector<float> stacked_obs;     // 堆叠观测 (450维，可选)
    std::vector<float> raw_action;      // 原始动作 (12维)
    std::vector<float> desired_abs;     // 期望绝对角度 (12维)
    std::vector<float> clipped_abs;     // 裁剪后角度 (12维)

    // 新增：执行层诊断字段
    std::vector<float> cmd_sent_abs;    // 实际发送的命令 (12维)
    std::vector<float> cmd_minus_pos;   // 命令与实际位置差 (12维)
    std::vector<float> cmd_delta;       // 命令相对上一拍变化 (12维)
    float max_track_err = 0.0f;         // 最大跟踪误差
    int clamp_count = 0;                // clamp 次数
    int imu_fresh = 1;                  // IMU 是否新鲜
    int motors_fresh = 1;               // 电机是否新鲜

    std::vector<float> motor_pos_abs;   // 电机位置 (12维)
    std::vector<float> motor_vel_abs;   // 电机速度 (12维)
    std::vector<float> motor_torque;    // 电机力矩 (12维)
};

/**
 * @brief 会话日志记录器
 *
 * 每次运行生成一个 session 目录：
 * - meta.json: 元数据
 * - events.log: 事件日志
 * - frames.csv: 帧数据
 *
 * 定期 flush 确保数据不会因崩溃丢失
 */
class SessionLogger {
public:
    SessionLogger() = default;
    ~SessionLogger() { Close(); }

    /**
     * @brief 打开会话日志
     * @param session_dir 会话目录路径
     * @return true 成功, false 失败
     */
    bool Open(const std::string& session_dir);

    /**
     * @brief 记录元数据
     * @param text JSON 格式的元数据
     */
    void LogMeta(const std::string& text);

    /**
     * @brief 记录事件
     * @param text 事件文本
     */
    void LogEvent(const std::string& text);

    /**
     * @brief 记录帧数据
     * @param rec 帧记录
     */
    void LogFrame(const FrameRecord& rec);

    /**
     * @brief 关闭日志
     */
    void Close();

    /**
     * @brief 检查日志是否打开
     */
    bool IsOpen() const { return meta_.is_open() && events_.is_open() && csv_.is_open(); }

    /**
     * @brief 获取帧计数
     */
    uint64_t GetFrameCount() const { return frame_count_; }

    /**
     * @brief 设置 flush 间隔（帧数）
     * @param frames 每隔多少帧 flush 一次，默认 50
     */
    void SetFlushInterval(int frames) { flush_interval_ = frames; }

private:
    std::ofstream meta_;
    std::ofstream events_;
    std::ofstream csv_;
    bool header_written_ = false;
    uint64_t frame_count_ = 0;
    int flush_interval_ = 50;  // 每 50 帧 flush 一次
};