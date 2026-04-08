#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "log.hpp"
#include "motor_io.hpp"

namespace twin {

struct ReplaySample {
    double time_sec = 0.0;
    std::vector<float> targets_rel;
};

class TwinAgent {
public:
    explicit TwinAgent(int cmd_port = 47001, int state_port = 47002);
    ~TwinAgent();

    bool Start();
    void Stop();
    bool IsRunning() const { return running_; }

private:
    void CommandLoop();
    void HandleCommand(int client_fd);
    void StateLoop();
    std::string ProcessCommand(const std::string& cmd);
    std::string SnapshotToJson();

    bool SetupServerSocket(int port, int& out_fd, std::string& err);
    bool InitToOffset(float duration_sec, std::string& err, std::vector<std::string>& offline_motors);
    bool SetJointTargets(const std::vector<float>& joint_targets, std::string& err);
    bool SendCurrentReplayCursorFrame(std::string& err, bool advance_after_send);
    bool SendReplayFrameAtCursor(size_t cursor, std::string& err, bool update_cursor_after_send);
    bool ParseJointIndexList(const std::string& token, std::vector<int>& indices, std::string& err) const;

    bool MotionBusy() const;
    bool AbortMotion(const std::string& reason);
    bool StartReplayMotion(float speed_factor, std::string& err);
    bool LaunchMotionThread(const std::string& name,
                            float duration_sec,
                            const std::function<bool(double, std::string&)>& step_fn,
                            const std::function<void()>& on_success,
                            std::string& err);
    bool LoadReplayCsv(const std::string& csv_path, std::vector<ReplaySample>& samples, std::string& err) const;
    static std::string Base64Decode(const std::string& encoded);

    int cmd_port_;
    int state_port_;
    int cmd_server_fd_ = -1;
    int state_server_fd_ = -1;

    std::atomic<bool> running_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<uint64_t> seq_{0};

    minimal::MotorIO motor_io_;
    std::thread cmd_thread_;
    std::thread state_thread_;
    std::thread motion_thread_;

    std::mutex control_mutex_;
    std::vector<float> last_joint_targets_;

    std::atomic<bool> motion_active_{false};
    std::atomic<bool> motion_abort_{false};
    std::mutex motion_mutex_;
    std::string active_motion_name_;
    std::string last_motion_error_;
    std::chrono::steady_clock::time_point motion_last_tick_;

    std::mutex replay_mutex_;
    std::vector<ReplaySample> replay_samples_;
    std::string replay_csv_path_;
    size_t replay_cursor_ = 0;
    float replay_speed_factor_ = 1.0f;
    bool replay_loaded_ = false;

    MotorFault last_fault_;
    void SetFault(const MotorFault& f);
};

}  // namespace twin
