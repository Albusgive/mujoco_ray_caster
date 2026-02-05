#if __cplusplus >= 202002L
    #define RAY_LAMBDA_CAPTURE [=, this]
#else
    #define RAY_LAMBDA_CAPTURE [=]
#endif
#include "ray_plugin.h"
#include "engine/engine_name.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_spatial.h"
#include "raycaster_src/Noise.hpp"
#include "raycaster_src/RayCaster.h"
#include "raycaster_src/RayNoise.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
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

// Checks that a plugin config attribute exists.
bool CheckAttr(const std::string &input) {
  char *end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

std::vector<std::string> ReadStringVector(const std::string &input) {
  std::vector<std::string> output;
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim)) {
    if (item.empty()) {
      continue;
    }
    CheckAttr(item);
    output.push_back(item);
  }
  return output;
}

int computeDateSize(const mjModel *m, int instance, int nray) {
  int n_data = 0;
  auto sensor_data_types =
      ReadStringVector(mj_getPluginConfig(m, instance, "sensor_data_types"));
  for (int i = 0; i < sensor_data_types.size(); i++) {
    if ((sensor_data_types[i].find("data") != std::string::npos) ||
        (sensor_data_types[i].find("image") != std::string::npos) ||
        (sensor_data_types[i].find("normal") != std::string::npos) ||
        (sensor_data_types[i].find("plane") != std::string::npos)) {
      n_data += nray;
    } else if ((sensor_data_types[i].find("pos_w") != std::string::npos) ||
               (sensor_data_types[i].find("pos_b") != std::string::npos)) {
      n_data += nray * 3;
    } else {
      mju_error("RayPlugin: sensor_data_types error: %s",
                sensor_data_types[i].c_str());
    }
  }
  return n_data;
}

RayPlugin::RayPlugin() {
  vis_cfg = VisCfg();
  ray_caster = nullptr;
  sensor_data_list = nullptr;
  n_sensor_data = 0;
}

void RayPlugin::Reset(const mjModel *m, int instance) {}

void RayPlugin::Compute(const mjModel *m, mjData *d, int instance) {
  n_step++;
  if (n_step < n_step_update)
    return;
  n_step = 0;
  if (compute_time_log) {
    start = std::chrono::high_resolution_clock::now();
  }
  mj_markStack(d);
  mjtNum *sensordata = d->sensordata + m->sensor_adr[sensor_id];
  ray_caster->compute_distance();
  for (int i = 0; i < n_sensor_data; i++) {
    SensorData &sensor_data = sensor_data_list[i];
    sensor_data.func(sensordata + sensor_data.data_point);
  }
  mj_freeStack(d);
  if (compute_time_log) {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << name << " compute: " << duration_ms.count() / 1000.0
              << " milliseconds" << std::endl;
  }
}

void RayPlugin::getBaseCfg(const mjModel *m, mjData *d, int instance) {
  // Get sensor id.
  int id;
  for (id = 0; id < m->nsensor; ++id) {
    if (m->sensor_type[id] == mjSENS_PLUGIN &&
        m->sensor_plugin[id] == instance) {
      break;
    }
  }
  sensor_id = id;
  name = std::string(m->names + m->name_sensoradr[sensor_id]);
  if (m->sensor_objtype[sensor_id] != mjOBJ_CAMERA) {
    mju_error("%s:the sensor objtype is err,must be camera", name.c_str());
  }
  /* ----- vis ----- */
  auto set_vis_default = [&](VisCfg::Default &vis_default, const char *attrib) {
    auto data = ReadVector<mjtNum>(mj_getPluginConfig(m, instance, attrib),
                                   vis_default.min_num_param);
    if (!data.empty()) {
      vis_default.is_draw = true;
      checkVisValue(vis_default.ratio, data[0]);
      checkVisValue(vis_default.width, data[1]);
      for (int i = 0; i < 4; i++) {
        checkVisValue(vis_default.color[i], data[i + 2]);
      }
    }
    std::vector<mjtNum> other;
    for (int i = VisCfg::Default().min_num_param; i < data.size(); i++) {
      other.push_back(data[i]);
    }
    return other;
  };

  auto edge = set_vis_default(vis_cfg.deep_ray, base_attributes[0]);
  if (!edge.empty())
    checkVisValue(vis_cfg.deep_ray.edge, edge[0]);

  auto draw_deep_ray_ids =
      set_vis_default(vis_cfg.deep_ray_ids, base_attributes[1]);
  for (int i = 0; i < draw_deep_ray_ids.size(); i++) {
    vis_cfg.deep_ray_ids.ids.push_back(static_cast<int>(draw_deep_ray_ids[i]));
  }

  set_vis_default(vis_cfg.deep, base_attributes[2]);

  set_vis_default(vis_cfg.hip_point, base_attributes[3]);

  // cfg.deep_ray.print();
  // cfg.deep_ray_ids.print();
  // cfg.deep.print();
  // cfg.hip_point.print();
  /* ----- vis ----- */

  /* ----- sensor data type  ----- */
  /* [data,image,inv_image,normal,inv_normal][inf_max,inf_zero][noise] */
  auto sensor_data_types =
      ReadStringVector(mj_getPluginConfig(m, instance, base_attributes[4]));
  n_sensor_data = sensor_data_types.size();
  if (sensor_data_list != nullptr) {
    delete[] sensor_data_list;
  }
  sensor_data_list = new SensorData[n_sensor_data];
  for (int i = 0; i < n_sensor_data; i++) {
    if (!sensor_data_list[i].fromStr(sensor_data_types[i]))
      mju_error("RayPlugin: sensor_data_types error: %s",
                sensor_data_types[i].c_str());
    // sensor_data_list[i].print();
  }
  /* ----- sensor data type  ----- */
  /* ----- log  ----- */
  auto compute_time =
      ReadVector<bool>(mj_getPluginConfig(m, instance, base_attributes[9]));
  compute_time_log = compute_time.empty() ? false : compute_time[0];
  /* ----- log  ----- */

  /* ----- update step  ----- */
  auto num_step =
      ReadVector<int>(mj_getPluginConfig(m, instance, base_attributes[10]));
  n_step_update = num_step.empty() ? n_step_update : num_step[0];
  /* ----- update step  ----- */
}

void RayPlugin::initSensor(const mjModel *m, mjData *d, int instance,
                           int nray) {
  int data_pos = 0;
  for (int i = 0; i < n_sensor_data; i++) {
    switch (sensor_data_list[i].type) {
    case DataType::data:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_data(data, sensor_data_list[i].is_noise,
                             sensor_data_list[i].is_inf_max);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::image:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_data_normalized(data, sensor_data_list[i].is_noise,
                                    sensor_data_list[i].is_inf_max,
                                    sensor_data_list[i].is_inv, 255.0);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::normal:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_data_normalized(data, sensor_data_list[i].is_noise,
                                    sensor_data_list[i].is_inf_max,
                                    sensor_data_list[i].is_inv, 1.0);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::distance_to_image_plane:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_distance_to_image_plane(
            data, sensor_data_list[i].is_noise, sensor_data_list[i].is_inf_max);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::image_plane_image:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_distance_to_image_plane_normalized(
            data, sensor_data_list[i].is_noise, sensor_data_list[i].is_inf_max,
            sensor_data_list[i].is_inv, 255.0);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::image_plane_normal:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_distance_to_image_plane_normalized(
            data, sensor_data_list[i].is_noise, sensor_data_list[i].is_inf_max,
            sensor_data_list[i].is_inv, 1.0);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray;
      data_pos += nray;
      break;
    case DataType::pos_w:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_data_pos_w(data);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray * 3;
      data_pos += nray * 3;
      break;
    case DataType::pos_b:
      sensor_data_list[i].func = RAY_LAMBDA_CAPTURE(mjtNum *data) {
        ray_caster->get_data_pos_b(data);
      };
      sensor_data_list[i].data_point = data_pos;
      sensor_data_list[i].data_size = nray * 3;
      data_pos += nray * 3;
      break;
    default:
      mju_error("RayPlugin: sensor_data_types error: %s",
                sensor_data_list[i].name.c_str());
      break;
    }
  }
  /*  --------------  noise ----------- */
  auto noise =
      ReadStringVector(mj_getPluginConfig(m, instance, base_attributes[5]));
  auto noise_cfg =
      ReadVector<mjtNum>(mj_getPluginConfig(m, instance, base_attributes[6]));
  if (!noise.empty() && !noise_cfg.empty())
  {
    int seed = 0;
    for (int i = 0; i < noise_attributes.size(); i++) {
      if (noise[0].find(noise_attributes[i].first) != std::string::npos) {
        if (noise_cfg.size() < noise_attributes[i].second - 1)
          mju_error("RayPlugin: noise_cfg error: %s",
                    mj_getPluginConfig(m, instance, base_attributes[6]));
        else if (noise_cfg.size() == noise_attributes[i].second)
          seed = noise_cfg[noise_attributes[i].second - 1];
        switch (i) {
        case 0: {
          ray_caster->setNoise(
              ray_noise::UniformNoise(noise_cfg[0], noise_cfg[1], seed));
        } break;
        case 1: {
          ray_caster->setNoise(
              ray_noise::GaussianNoise(noise_cfg[0], noise_cfg[1], seed));
        } break;
        case 2: {
          ray_caster->setNoise(ray_noise::RayNoise1(noise_cfg[0], noise_cfg[1],
                                                    noise_cfg[2], seed));
        } break;
        case 3: {
          ray_caster->setNoise(ray_noise::RayNoise2(
              noise_cfg[0], noise_cfg[1], noise_cfg[2], noise_cfg[3],
              noise_cfg[4], noise_cfg[5], noise_cfg[6], seed));
        } break;
        }
      }
    }
  }
  /*---------------- pluginstate ----------------  */
  int state_idx = m->plugin_stateadr[instance];
  d->plugin_state[state_idx] = ray_caster->h_ray_num;
  d->plugin_state[state_idx + 1] = ray_caster->v_ray_num;
  state_idx += 2;
  for (int i = 0; i < n_sensor_data; i++) {
    d->plugin_state[state_idx] = sensor_data_list[i].data_point;
    d->plugin_state[state_idx + 1] = sensor_data_list[i].data_size;
    state_idx += 2;
  }

  /*geomgroup*/
  auto geomgroup =
      ReadVector<bool>(mj_getPluginConfig(m, instance, base_attributes[7]));
  if (geomgroup.size() <= 6) {
    for (int i = 0; i < geomgroup.size(); i++) {
      ray_caster->geomgroup[i] = geomgroup[i];
    }
  }

  /* other cfg */
  auto detect_parentbody =
      ReadVector<bool>(mj_getPluginConfig(m, instance, base_attributes[8]));
  if (!detect_parentbody.empty()) {
    if (detect_parentbody[0]) {
      ray_caster->no_detect_body_id = m->cam_bodyid[ray_caster->cam_id];
    }
  }

  /* tuning */
  bool is_compute_hit = false;
  bool is_compute_hit_b = false;
  for (int i = 0; i < n_sensor_data; i++) {
    if (sensor_data_list[i].type == DataType::pos_w)
      is_compute_hit = true;
    if (sensor_data_list[i].type == DataType::pos_b)
      is_compute_hit_b = true;
  }
  ray_caster->is_compute_hit = is_compute_hit | vis_cfg.hip_point.is_draw;
  ray_caster->is_compute_hit_b = is_compute_hit_b;

  /* thread */
  auto num_thread =
      ReadVector<int>(mj_getPluginConfig(m, instance, base_attributes[11]));
  if (!num_thread.empty()) {
    ray_caster->set_num_thread(num_thread[0]);
  }
}

void RayPlugin::Visualize(const mjModel *m, mjData *d, const mjvOption *opt,
                          mjvScene *scn, int instance) {
  mj_markStack(d);

  if (vis_cfg.deep_ray.is_draw)
    ray_caster->draw_deep_ray(scn, vis_cfg.deep_ray.ratio,
                              vis_cfg.deep_ray.width, vis_cfg.deep_ray.edge,
                              vis_cfg.deep_ray.color);
  if (vis_cfg.deep_ray_ids.is_draw) {
    int len = vis_cfg.deep_ray_ids.ids.size();
    for (int i = 0; i < len; i++) {
      ray_caster->draw_deep_ray(scn, vis_cfg.deep_ray_ids.ids[i],
                                vis_cfg.deep_ray_ids.width,
                                vis_cfg.deep_ray_ids.color);
    }
  }
  if (vis_cfg.deep.is_draw)
    ray_caster->draw_deep(scn, vis_cfg.deep.ratio, vis_cfg.deep.width,
                          vis_cfg.deep.color);
  if (vis_cfg.hip_point.is_draw)
    ray_caster->draw_hip_point(scn, vis_cfg.hip_point.ratio,
                               vis_cfg.hip_point.width,
                               vis_cfg.hip_point.color);

  mj_freeStack(d);
}

} // namespace mujoco::plugin::sensor
