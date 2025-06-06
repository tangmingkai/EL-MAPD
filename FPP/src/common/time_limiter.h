#pragma once
#include <chrono>
#include <iostream>

using std::chrono::steady_clock;

class TimeLimiter {
 public:
  TimeLimiter(const double &time_limit_s) : time_limit_s_(time_limit_s) {
    ResetStartTime();
  }

  inline void ResetStartTime() { start_time_ = steady_clock::now(); }

  inline double GetElapse() const {
    auto now = steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - start_time_);
    return duration.count();
  }

  inline bool Timeout() const {
    double elapse = GetElapse();
    return elapse >= time_limit_s_;
  }

  inline double GetRemainingTime() const { return time_limit_s_ - GetElapse(); }
 private:
  steady_clock::time_point start_time_;
  double time_limit_s_;
};