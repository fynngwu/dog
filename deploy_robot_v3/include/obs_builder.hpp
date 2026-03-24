#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace minimal {

class ObsBuilder {
public:
    static constexpr int kObsDim = 45;
    static constexpr int kHistoryLen = 10;
    static constexpr int kPolicyInputDim = 450;

    void Reset();
    std::vector<float> BuildSingle(const std::vector<float>& imu_obs,
                                   const std::array<float, 3>& command,
                                   const std::vector<float>& joint_obs) const;
    void Push(const std::vector<float>& single_obs);
    void ClearHistory();
    size_t HistorySize() const;
    std::vector<float> FlattenHistory() const;

private:
    std::deque<std::vector<float>> history_;
};

}  // namespace minimal
