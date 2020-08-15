#ifndef PID_H
#define PID_H

#include <chrono>
#include <cmath>
#include <iostream>

using Clock = std::chrono::system_clock;
using TimePoint = std::chrono::time_point<Clock>;

class PID {
 public:
  PID() { last_time_ = std::chrono::system_clock::now(); }

  void SetCoefficients(float Kp, float Ki, float Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  }

  void SetLimits(float min, float max) {
    min_ = min;
    max_ = max;
  }

  void setError(float error) {
    float elapsed_s = getElapsedSeconds();

    // proportional
    p_value_ = error;

    // integral
    // TODO: reset on target speed change
    i_value_ += error * elapsed_s;

    // derivative
    d_value_ = (error - last_error_) / elapsed_s;

    // std::cout << "elapsed_s: " << elapsed_s << ", I: " << i_value_
    //           << ", D: " << d_value_ << std::endl;

    // last values
    last_error_ = error;
    last_time_ = Clock::now();
  }

  float getAction() const {
    // raw pid
    float action = Kp_ * p_value_ + Ki_ * i_value_ + Kd_ * d_value_;

    // anti winding up ignores I when outside limits
    if (action < min_ or max_ < action) {
      action = Kp_ * p_value_ + Kd_ * d_value_;
    }

    // action is always limited
    return fmax(min_, fmin(max_, action));
  }

 private:
  float getElapsedSeconds() {
    TimePoint now = Clock::now();
    std::chrono::duration<double> elapsed_seconds = now - last_time_;
    return elapsed_seconds.count();
  }

  // PID values
  float p_value_{0.0};
  float i_value_{0.0};
  float d_value_{0.0};

  // PID coefficients
  float Kp_;
  float Ki_;
  float Kd_;

  // limits
  float min_;
  float max_;

  // last values
  float last_error_{0.0};
  TimePoint last_time_;
};

#endif  // PID_H
