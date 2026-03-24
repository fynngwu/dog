#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "motor_io.hpp"

namespace twin {

class TwinAgent {
public:
    TwinAgent(int cmd_port = 47001, int state_port = 47002);
    ~TwinAgent();

    bool Start();
    void Stop();
    bool IsRunning() const { return running_; }

private:
    void CommandLoop();
    void StatePublishLoop();
    void HandleCommand(int client_fd);
    std::string ProcessCommand(const std::string& cmd);
    std::string SnapshotToJson();

    bool SetupServerSocket(int port, int& out_fd, std::string& err);
    bool InitToOffset(float duration_sec, std::string& err);
    bool SetJointTargets(const std::vector<float>& joint_targets, std::string& err);
    void CloseAllStateClients();

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

    std::mutex control_mutex_;
    std::vector<float> last_joint_targets_;

    std::mutex state_clients_mutex_;
    std::vector<int> state_clients_;
};

}  // namespace twin
