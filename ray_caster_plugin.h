#pragma once

#include "ray_plugin.h"
#include "raycaster_src/RayCaster.h"

namespace mujoco::plugin::sensor {

class RayCasterPlugin : public RayPlugin {
public:
  static RayCasterPlugin *Create(const mjModel *m, mjData *d, int instance);
  RayCasterPlugin(RayCasterPlugin &&) = default;
  ~RayCasterPlugin() = default;

  static void RegisterPlugin();
  RayCasterCfg cfg;

  static constexpr std::array<const char *, 4> ray_attributes = {
      "resolution", "size", "dis_range", "type"};

private:
  RayCasterPlugin(const mjModel *m, mjData *d, int instance);
  std::vector<mjtNum> distance_;
};

} // namespace mujoco::plugin::sensor
