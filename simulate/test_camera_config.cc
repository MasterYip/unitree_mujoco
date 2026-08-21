#include "src/param.h"

#include <cassert>
#include <cmath>

int main() {
  param::config.load_from_yaml("config.yaml");
  assert(param::config.camera_follow == 0);
  assert(param::config.camera_lookat_offset.size() == 3);
  const char* args[] = {"unitree_mujoco", "--camera-follow", "--camera-follow-body", "pelvis",
                        "--camera-follow-yaw", "--camera-smoothing-tau", "0.25",
                        "--camera-lookat-z", "0.1"};
  param::helper(static_cast<int>(sizeof(args) / sizeof(args[0])), const_cast<char**>(args));
  assert(param::config.camera_follow == 1);
  assert(param::config.camera_follow_body == "pelvis");
  assert(param::config.camera_follow_yaw == 1);
  assert(std::abs(param::config.camera_smoothing_tau - 0.25) < 1e-12);
  assert(std::abs(param::config.camera_lookat_offset[2] - 0.1) < 1e-12);
}
