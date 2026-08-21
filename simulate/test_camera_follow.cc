#include "src/camera_follow.h"

#include <cassert>
#include <cmath>

int main() {
  camera_follow::Filter filter;
  camera_follow::Settings settings;
  settings.lookat_offset = {1.0, 0.0, 0.2};
  camera_follow::Pose pose{{2.0, 3.0, 0.8}, {std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)}};

  auto fixed = filter.Update(pose, 0.02, settings);
  assert(std::abs(fixed.lookat[0] - 3.0) < 1e-12);
  assert(std::abs(fixed.lookat[1] - 3.0) < 1e-12);
  assert(std::abs(fixed.azimuth - 90.0) < 1e-12);

  filter.Reset();
  settings.follow_yaw = true;
  auto yaw = filter.Update(pose, 0.02, settings);
  assert(std::abs(yaw.lookat[0] - 2.0) < 1e-12);
  assert(std::abs(yaw.lookat[1] - 4.0) < 1e-12);
  assert(std::abs(yaw.azimuth - 180.0) < 1e-12);

  settings.follow_yaw = false;
  settings.smoothing_tau = 1.0;
  filter.Reset();
  pose.position = {0.0, 0.0, 0.0};
  filter.Update(pose, 0.1, settings);
  pose.position = {10.0, 0.0, 0.0};
  auto smooth = filter.Update(pose, 0.1, settings);
  assert(smooth.lookat[0] > 1.0 && smooth.lookat[0] < 11.0);
  filter.Reset();
  auto reset = filter.Update(pose, 0.1, settings);
  assert(std::abs(reset.lookat[0] - 11.0) < 1e-12);
}
