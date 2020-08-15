#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "PID.h"

class VehicleController {
 public:
  VehicleController() {
    steering_controller_.SetCoefficients(0.1, 0.00, 0.02);
    throttle_controller_.SetCoefficients(0.6, 0.01, 0.01);
    steering_controller_.SetLimits(-1.0, 1.0);
    throttle_controller_.SetLimits(-1.0, 1.0);
    target_speed_mph_ = min_speed_mph_;
  }

  // decrease target speed if CTE outside limits
  void updateTargetSpeed() {
    if (-1.0 * cte_speed_th_ < current_cte_ and current_cte_ < cte_speed_th_) {
      target_speed_mph_ += speed_delta_mph_;
    } else {
      target_speed_mph_ -= speed_delta_mph_;
    }
    target_speed_mph_ = fmin(max_speed_mph_, target_speed_mph_);
    target_speed_mph_ = fmax(min_speed_mph_, target_speed_mph_);
  }

  void step() {
    updateTargetSpeed();

    float speed_error = target_speed_mph_ - current_speed_mph_;
    throttle_controller_.setError(speed_error);
    steering_controller_.setError(-1.0 * current_cte_);
  }

  void setCrossTrackError(float cte) { current_cte_ = cte; }
  void setTelemetrySpeed(float speed_mph) { current_speed_mph_ = speed_mph; }

  float getActionSteeringAngle() const {
    return steering_controller_.getAction();
  }

  float getActionThrottle() const { return throttle_controller_.getAction(); }

 private:
  // telemetry
  float current_speed_mph_{0.0F};
  float current_cte_{0.0F};

  // controllers
  PID throttle_controller_;
  PID steering_controller_;

  // target
  float cte_speed_th_{0.6};
  float speed_delta_mph_{0.5};
  float target_speed_mph_;
  float max_speed_mph_{30.0};
  float min_speed_mph_{20.0};
};

#endif  // VEHICLE_CONTROLLER_H
