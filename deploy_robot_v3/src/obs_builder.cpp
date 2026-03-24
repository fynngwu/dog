#include "obs_builder.hpp"
#include <cstddef>

namespace minimal {

void ObsBuilder::Reset() {
    history_.clear();
}

std::vector<float> ObsBuilder::BuildSingle(const std::vector<float>& imu_obs,
                                           const std::array<float, 3>& command,
                                           const std::vector<float>& joint_obs) const {
    std::vector<float> obs;
    obs.reserve(kObsDim);
    obs.insert(obs.end(), imu_obs.begin(), imu_obs.end());
    obs.insert(obs.end(), command.begin(), command.end());
    obs.insert(obs.end(), joint_obs.begin(), joint_obs.end());
    obs.insert(obs.end(), 12, 0.0f);
    return obs;
}

void ObsBuilder::Push(const std::vector<float>& single_obs) {
    history_.push_back(single_obs);
    if (history_.size() > kHistoryLen) {
        history_.pop_front();
    }
}

void ObsBuilder::ClearHistory() {
    history_.clear();
}

size_t ObsBuilder::HistorySize() const {
    return history_.size();
}

std::vector<float> ObsBuilder::FlattenHistory() const {
    std::vector<float> flat;
    flat.reserve(kPolicyInputDim);

    const int missing = kHistoryLen - static_cast<int>(history_.size());
    for (int i = 0; i < missing; ++i) {
        flat.insert(flat.end(), kObsDim, 0.0f);
    }

    for (const auto& frame : history_) {
        flat.insert(flat.end(), frame.begin(), frame.end());
    }

    return flat;
}

}  // namespace minimal
