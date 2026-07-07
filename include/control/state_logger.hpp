#pragma once

#include <array>
#include <mutex>
#include <vector>

namespace kist {

// In-memory ring buffer of robot state history for the his_* policy
// observations. Lean port of gear_sonic's StateLogger: no CSV sinks, no
// hand/temperature fields, and no stride resampling — the shipped
// observation config only ever samples the last N consecutive 50Hz ticks
// (step_size 1), so GetLatest(n) returns exactly that.
class StateLogger {
public:
    struct Entry {
        // base_quat defaults to all-zero (not identity) so zero-padded history
        // entries match gear_sonic's makeZeroEntry_ exactly.
        std::array<double, 4>  base_quat{0.0, 0.0, 0.0, 0.0};  // pelvis IMU, wxyz
        std::array<double, 3>  base_ang_vel{};                 // pelvis gyro (rad/s)
        std::array<double, 29> body_q{};        // IsaacLab order, default_angles subtracted
        std::array<double, 29> body_dq{};       // IsaacLab order
        std::array<double, 29> last_action{};   // raw policy output (IsaacLab order)
    };

    explicit StateLogger(size_t capacity = 100) : capacity_(capacity) {
        ring_.resize(capacity_);
    }

    void Log(const Entry& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        ring_[(start_ + size_) % capacity_] = e;
        if (size_ < capacity_) ++size_;
        else                   start_ = (start_ + 1) % capacity_;
    }

    // Latest n entries, oldest → newest. When fewer than n are available the
    // missing (oldest) slots are zero entries, matching gear_sonic's padding.
    std::vector<Entry> GetLatest(size_t n) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Entry> out(n);  // zero entries by default
        size_t have = std::min(n, size_);
        for (size_t j = 0; j < have; ++j) {
            size_t idx = (start_ + size_ - have + j) % capacity_;
            out[n - have + j] = ring_[idx];
        }
        return out;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

private:
    size_t             capacity_;
    mutable std::mutex mutex_;
    std::vector<Entry> ring_;
    size_t             start_{0};
    size_t             size_{0};
};

} // namespace kist
