#pragma once

#include "ray_plugin.h"
#include "raycaster_src/RayCasterCamera.h"

namespace mujoco::plugin::sensor {

class RayCasterCameraPlugin : public RayPlugin {
public:
  static RayCasterCameraPlugin *Create(const mjModel *m, mjData *d,
                                       int instance);
  RayCasterCameraPlugin(RayCasterCameraPlugin &&) = default;
  ~RayCasterCameraPlugin() = default;
  RayCasterCameraCfg cfg;

  static void RegisterPlugin();

  static constexpr std::array<const char *, 5> ray_attributes = {
      "focal_length", "horizontal_aperture", "vertical_aperture", "size",
      "dis_range"};

private:
  RayCasterCameraPlugin(const mjModel *m, mjData *d, int instance);
  std::vector<mjtNum> distance_;
};

} // namespace mujoco::plugin::sensor
