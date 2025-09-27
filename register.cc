#include <mujoco/mjplugin.h>
#include "ray_caster_plugin.h"
#include "ray_caster_camera_plugin.h"
#include "ray_caster_lidar_plugin.h"

namespace mujoco::plugin::sensor {

mjPLUGIN_LIB_INIT {
  RayCasterPlugin::RegisterPlugin();
  RayCasterCameraPlugin::RegisterPlugin();
  RayCasterLidarPlugin::RegisterPlugin();
}

} // namespace mujoco::plugin::sensor
