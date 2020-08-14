#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <iostream>

#include "PID.h"

class VehicleController {
 public:
  VehicleController(float target_speed_mph)
      : target_speed_mph_(target_speed_mph) {
    steering_controller_.Init(0.0, 0.0, 0.0);
    throttle_controller_.Init(0.2, 0.0, 0.0);
  }

  void step() {
    std::cout << "[telemetry] cte: " << current_cte_
              << ", steering_angle: " << current_steering_angle_
              << ", speed_mph: " << current_speed_mph_ << std::endl;

    float speed_error = target_speed_mph_ - current_speed_mph_;
    throttle_controller_.setError(speed_error);

    std::cout << "[controller] speed error: " << speed_error << std::endl;
  }

  void setCrossTrackError(float cte) { current_cte_ = cte; }
  void setTelemetrySpeed(float speed_mph) { current_speed_mph_ = speed_mph; }
  void setTelemetrySteeringAngle(float angle) {
    current_steering_angle_ = angle;
  }

  float getActionSteeringAngle() const {
    float action = 0.0;
    return fmax(-1, fmin(+1, action));
  }

  float getActionThrottle() const {
    float action = -1 * throttle_controller_.getAction();
    return fmax(-1, fmin(+1, action));
  }

 private:
  // telemetry
  float current_speed_mph_;
  float current_cte_;
  float current_steering_angle_;

  // controllers
  PID throttle_controller_;
  PID steering_controller_;

  // target
  float target_speed_mph_;
};

#endif  // VEHICLE_CONTROLLER_H
