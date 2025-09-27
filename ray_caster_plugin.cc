#include "ray_caster_plugin.h"
#include "engine/engine_name.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"
#include "ray_plugin.h"
#include "raycaster_src/RayCaster.h"

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
// Creates a RayCasterPlugin instance if all config attributes are defined and
// within their allowed bounds.
RayCasterPlugin *RayCasterPlugin::Create(const mjModel *m, mjData *d,
                                         int instance) {

  return new RayCasterPlugin(m, d, instance);
}

RayCasterPlugin::RayCasterPlugin(const mjModel *m, mjData *d, int instance) {
  cfg = RayCasterCfg();
  getBaseCfg(m, d, instance);

  auto resolution =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[0]), 1);
  cfg.resolution = resolution[0];

  auto size =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[1]), 2);
  cfg.size = {size[0], size[1]};

  auto dis_range =
      ReadVector<double>(mj_getPluginConfig(m, instance, ray_attributes[2]), 2);
  cfg.dis_range = {dis_range[0], dis_range[1]};

  std::string type = mj_getPluginConfig(m, instance, ray_attributes[3]);
  if (type == "base") {
    cfg.type = RayCasterType::base;
  } else if (type == "yaw") {
    cfg.type = RayCasterType::yaw;
  } else if (type == "world") {
    cfg.type = RayCasterType::world;
  } else {
    mju_error("RayCasterPlugin: type must be base, yaw or world");
  }

  std::string name =
      std::string(mj_id2name(m, mjOBJ_CAMERA, m->sensor_objid[sensor_id]));
  cfg.cam_name = name;
  cfg.m = m;
  cfg.d = d;
  ray_caster = new RayCaster(cfg);
  initSensor(m,d, instance, ray_caster->nray);
}

void RayCasterPlugin::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sensor.ray_caster";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  auto data = concat_arrays(ray_attributes);
  plugin.nattribute = data.size();
  plugin.attributes = data.data();

  // Stateless.
  plugin.nstate = +[](const mjModel *m, int instance) {
    // size + n_data_size
    auto sensor_data_types =
        ReadStringVector(mj_getPluginConfig(m, instance, "sensor_data_types"));
    int n_state = 2 + sensor_data_types.size() * 2;
    return n_state;
  };

  // Sensor dimension = nchannel * size[0] * size[1]
  plugin.nsensordata = +[](const mjModel *m, int instance, int sensor_id) {
    auto size = ReadVector<double>(mj_getPluginConfig(m, instance, "size"), 2);
    auto resolution =
        ReadVector<double>(mj_getPluginConfig(m, instance, "resolution"), 1);
    if (size[0] <= 0 || size[1] <= 0 || resolution[0] <= 0)
      mju_error("RayCasterPlugin: size and resolution must be positive");

    int nray = ((int)(size[0] / resolution[0]) + 1) *
               ((int)(size[1] / resolution[0]) + 1);
    int n_data = computeDateSize(m, instance, nray);
    return n_data;
  };

  // Can only run after forces have been computed.
  plugin.needstage = STAGE;

  // Initialization callback.
  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    auto *RayCasterPlugin = RayCasterPlugin::Create(m, d, instance);
    if (!RayCasterPlugin) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(RayCasterPlugin);
    return 0;
  };

  // Destruction callback.
  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<RayCasterPlugin *>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset callback.
  plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                     int instance) {
    auto *RayCasterPlugin =
        reinterpret_cast<class RayCasterPlugin *>(plugin_data);
    RayCasterPlugin->Reset(m, instance);
  };

  // Compute callback.
  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        auto *RayCasterPlugin =
            reinterpret_cast<class RayCasterPlugin *>(d->plugin_data[instance]);
        RayCasterPlugin->Compute(m, d, instance);
      };

  // Visualization callback.
  plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *opt,
                         mjvScene *scn, int instance) {
    auto *RayCasterPlugin =
        reinterpret_cast<class RayCasterPlugin *>(d->plugin_data[instance]);
    RayCasterPlugin->Visualize(m, d, opt, scn, instance);
  };

  // Register the plugin.
  mjp_registerPlugin(&plugin);
}

} // namespace mujoco::plugin::sensor
