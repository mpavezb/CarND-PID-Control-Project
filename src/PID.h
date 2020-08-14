#ifndef PID_H
#define PID_H

class PID {
 public:
  void Init(float Kp, float Ki, float Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kp;
  }

  void setError(float error) {
    // proportional
    p_value_ = error;

    // integral
    i_value_ += error;

    // derivative
    d_value_ = error - previous_error_;
    previous_error_ = error;
  }

  float getAction() const {
    return -Kp_ * p_value_ - Ki_ * i_value_ - Kd_ * d_value_;
  }

 private:
  // PID values
  float p_value_{0.0};
  float i_value_{0.0};
  float d_value_{0.0};
  float previous_error_{0.0};

  // PID coefficients
  float Kp_;
  float Ki_;
  float Kd_;
};

#endif  // PID_H
