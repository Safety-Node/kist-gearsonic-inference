#pragma once

#include <memory>
#include <chrono>
#include <mutex>
#include <shared_mutex>

template <typename T>
struct TimestampedData {
    std::shared_ptr<const T> data;
    std::chrono::steady_clock::time_point timestamp;

    TimestampedData()
        : data(nullptr), timestamp{} {}

    TimestampedData(std::shared_ptr<const T> d, std::chrono::steady_clock::time_point t)
        : data(d), timestamp(t) {}

    bool HasData() const { return data != nullptr; }

    double GetAgeMs() const {
        if (!HasData()) return -1.0;
        auto now = std::chrono::steady_clock::now();
        auto age = std::chrono::duration_cast<std::chrono::microseconds>(now - timestamp);
        return age.count() / 1000.0;
    }
};

template <typename T>
class DataBuffer {
public:
    void SetData(const T& newData) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = std::make_shared<T>(newData);
        last_update_time_ = std::chrono::steady_clock::now();
    }

    void SetData(T&& newData) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = std::make_shared<T>(std::move(newData));
        last_update_time_ = std::chrono::steady_clock::now();
    }

    std::shared_ptr<const T> GetData() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return data_;
    }

    TimestampedData<T> GetDataWithTime() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (data_) return TimestampedData<T>(data_, last_update_time_);
        return TimestampedData<T>();
    }

    void Clear() {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        data_ = nullptr;
        last_update_time_ = std::chrono::steady_clock::time_point{};
    }

private:
    std::shared_ptr<T> data_;
    std::chrono::steady_clock::time_point last_update_time_;
    mutable std::shared_mutex mutex_;
};
