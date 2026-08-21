#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace camera_follow {

struct Settings {
  std::array<double, 3> lookat_offset{0.0, 0.0, 0.0};
  double azimuth = 90.0;
  double smoothing_tau = 0.0;
  bool follow_yaw = false;
};

struct Pose {
  std::array<double, 3> position{};
  std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};  // wxyz
};

struct Output {
  std::array<double, 3> lookat{};
  double azimuth = 0.0;
};

class Filter {
 public:
  void Reset() { initialized_ = false; }

  Output Update(const Pose& pose, double dt, const Settings& settings) {
    const double yaw = Yaw(pose.quaternion);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    auto offset = settings.lookat_offset;
    if (settings.follow_yaw) {
      offset = {c * settings.lookat_offset[0] - s * settings.lookat_offset[1],
                s * settings.lookat_offset[0] + c * settings.lookat_offset[1],
                settings.lookat_offset[2]};
    }
    Output desired{{pose.position[0] + offset[0], pose.position[1] + offset[1],
                    pose.position[2] + offset[2]},
                   settings.azimuth + (settings.follow_yaw ? yaw * 180.0 / kPi : 0.0)};
    if (!initialized_ || settings.smoothing_tau <= 0.0 || dt <= 0.0) {
      output_ = desired;
      initialized_ = true;
      return output_;
    }
    const double alpha = 1.0 - std::exp(-dt / settings.smoothing_tau);
    for (int i = 0; i < 3; ++i) {
      output_.lookat[i] += alpha * (desired.lookat[i] - output_.lookat[i]);
    }
    output_.azimuth += alpha * WrapDegrees(desired.azimuth - output_.azimuth);
    return output_;
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;

  static double Yaw(const std::array<double, 4>& q) {
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                      1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  }

  static double WrapDegrees(double angle) {
    return std::fmod(angle + 540.0, 360.0) - 180.0;
  }

  bool initialized_ = false;
  Output output_{};
};

}  // namespace camera_follow
