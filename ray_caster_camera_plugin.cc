#include "ray_caster_camera_plugin.h"
#include "engine/engine_name.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"
#include "ray_plugin.h"
#include "raycaster_src/RayCaster.h"
#include "raycaster_src/RayCasterCamera.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor {
// Creates a RayCasterCameraPlugin instance if all config attributes are defined
// and within their allowed bounds.
RayCasterCameraPlugin *RayCasterCameraPlugin::Create(const mjModel *m,
                                                     mjData *d, int instance) {

  return new RayCasterCameraPlugin(m, d, instance);
}

RayCasterCameraPlugin::RayCasterCameraPlugin(const mjModel *m, mjData *d,
                                             int instance) {
  cfg = RayCasterCameraCfg();
  getBaseCfg(m, d, instance);

  auto focal_length =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[0]), 1);
  cfg.focal_length = focal_length[0];

  auto horizontal_aperture =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[1]), 1);
  cfg.horizontal_aperture = horizontal_aperture[0];

  auto vertical_aperture =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[2]), 1);
  cfg.vertical_aperture = vertical_aperture[0];

  auto size =
      ReadVector<int>(mj_getPluginConfig(m, instance, ray_attributes[3]), 1);
  cfg.h_ray_num = size[0];
  cfg.v_ray_num = size[1];

  auto dis_range =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[4]), 2);
  cfg.dis_range = {dis_range[0], dis_range[1]};

  auto baseline =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[5]));
  cfg.baseline = baseline.empty() ? n_step_update : baseline[0];

  std::string name =
      std::string(mj_id2name(m, mjOBJ_CAMERA, m->sensor_objid[sensor_id]));
  cfg.cam_name = name;
  cfg.m = m;
  cfg.d = d;
  ray_caster = new RayCasterCamera(cfg);
  initSensor(m, d, instance, ray_caster->nray);
}

void RayCasterCameraPlugin::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sensor.ray_caster_camera";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  auto data = concat_arrays(ray_attributes);
  plugin.nattribute = data.size();
  plugin.attributes = data.data();

  // Stateless.
  plugin.nstate = +[](const mjModel *m, int instance) { // size + n_data_size
    auto sensor_data_types =
        ReadStringVector(mj_getPluginConfig(m, instance, "sensor_data_types"));
    int n_state = 2 + sensor_data_types.size() * 2;
    return n_state;
  };

  // Sensor dimension = nchannel * size[0] * size[1]
  plugin.nsensordata = +[](const mjModel *m, int instance, int sensor_id) {
    auto size = ReadVector<int>(mj_getPluginConfig(m, instance, "size"), 2);
    if (size[0] <= 0 || size[1] <= 0)
      mju_error("RayCasterPlugin: size must be positive");
    int nray = size[0] * size[1];
    int n_data = computeDateSize(m, instance, nray);
    return n_data;
  };

  // Can only run after forces have been computed.
  plugin.needstage = STAGE;

  // Initialization callback.
  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    auto *RayCasterCameraPlugin = RayCasterCameraPlugin::Create(m, d, instance);
    if (!RayCasterCameraPlugin) {
      return -1;
    }
    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(RayCasterCameraPlugin);
    return 0;
  };

  // Destruction callback.
  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<RayCasterCameraPlugin *>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset callback.
  plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                     int instance) {
    auto *RayCasterCameraPlugin =
        reinterpret_cast<class RayCasterCameraPlugin *>(plugin_data);
    RayCasterCameraPlugin->Reset(m, instance);
  };

  // Compute callback.
  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        auto *RayCasterCameraPlugin =
            reinterpret_cast<class RayCasterCameraPlugin *>(
                d->plugin_data[instance]);
        RayCasterCameraPlugin->Compute(m, d, instance);
      };

  // Visualization callback.
  plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *opt,
                         mjvScene *scn, int instance) {
    auto *RayCasterCameraPlugin =
        reinterpret_cast<class RayCasterCameraPlugin *>(
            d->plugin_data[instance]);
    RayCasterCameraPlugin->Visualize(m, d, opt, scn, instance);
  };

  // Register the plugin.
  mjp_registerPlugin(&plugin);
}

} // namespace mujoco::plugin::sensor
