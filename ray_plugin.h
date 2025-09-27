#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "raycaster_src/RayCaster.h"
#include "raycaster_src/RayCasterCamera.h"
#include "raycaster_src/RayCasterLidar.h"
#include <iostream>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor {

#define STAGE mjSTAGE_POS

// Checks that a plugin config attribute exists.
bool CheckAttr(const std::string &input);

std::vector<std::string> ReadStringVector(const std::string &input);

int computeDateSize(const mjModel *m, int instance, int nray);

// Converts a string into a numeric vector
template <typename T> std::vector<T> ReadVector(const std::string &input) {
  std::vector<T> output;
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim)) {
    CheckAttr(item);
    output.push_back(strtod(item.c_str(), nullptr));
  }
  return output;
}

template <typename T>
std::vector<T> ReadVector(const std::string &input, int min_num) {
  std::vector<T> output;
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim)) {
    CheckAttr(item);
    output.push_back(strtod(item.c_str(), nullptr));
  }
  if (output.empty())
    return output;
  while (output.size() <= min_num) {
    output.push_back(-1);
  }
  return output;
}

template <typename T1, typename T2>
void checkVisValue(T1 &default_value, T2 &check_value) {
  if (check_value == -1)
    return;
  default_value = static_cast<T1>(check_value);
}

class VisCfg {
public:
  class Default {
  public:
    bool is_draw = false;
    int ratio = 1;
    mjtNum width = 5;
    float color[4] = {1.0, 1.0, 1.0, 0.5};
    int min_num_param = 6;
    std::string name;
    virtual void print() {
      std::cout << name << std::endl;
      std::cout << "-----is_draw: " << is_draw << std::endl;
      std::cout << "-----min_num_param: " << min_num_param << std::endl;
      std::cout << "-----ratio: " << ratio << std::endl;
      std::cout << "-----width: " << width << std::endl;
      std::cout << "-----color: " << color[0] << "  " << color[1] << "  "
                << color[2] << "  " << color[3] << std::endl;
    }
  };
  class DrawDeepRay : public Default {
  public:
    DrawDeepRay() {
      name = "DrawDeepRay";
      this->color[0] = 0.0;
      this->color[1] = 1.0;
      this->color[2] = 0.0;
      this->min_num_param = 7;
    }
    bool edge = true;
    void print() override {
      Default::print();
      std::cout << "-----edge: " << edge << std::endl;
    }
  };
  class DrawDeepRayIds : public Default {
  public:
    DrawDeepRayIds() {
      name = "DrawDeepRayIds";
      this->color[0] = 1.0;
      this->color[1] = 1.0;
      this->color[2] = 0.0;
    }
    std::vector<int> ids;
    void print() override {
      Default::print();
      std::cout << "-----ids: ";
      for (auto &id : ids) {
        std::cout << id << " ";
      }
      std::cout << std::endl;
    }
  };
  class DrawDeep : public Default {
  public:
    DrawDeep() {
      name = "DrawDeep";
      this->color[0] = 0.0;
      this->color[1] = 0.0;
      this->color[2] = 1.0;
    }
  };
  class DrawHipPoint : public Default {
  public:
    DrawHipPoint() {
      name = "DrawHipPoint";
      this->color[0] = 1.0;
      this->color[1] = 0.0;
      this->color[2] = 0.0;
      this->width = 0.02;
    }
  };

  DrawDeepRay deep_ray;
  DrawDeepRayIds deep_ray_ids;
  DrawDeep deep;
  DrawHipPoint hip_point;
};

enum class DataType { data, image, normal, pos_w, pos_b };

class SensorData {
public:
  DataType type;
  std::string name;
  bool is_noise = false;
  bool is_inf_max = true;
  bool is_inv = false;
  int data_point;
  int data_size;
  std::function<void(mjtNum *data)> func;
  bool fromStr(std::string data_type) {
    name = data_type;
    if (data_type.find("noise") != std::string::npos)
      is_noise = true;
    if (data_type.find("inf_zero") != std::string::npos)
      is_inf_max = false;
    if (data_type.find("inv") != std::string::npos)
      is_inv = true;
    if (data_type.find("data") != std::string::npos)
      type = DataType::data;
    else if (data_type.find("image") != std::string::npos)
      type = DataType::image;
    else if (data_type.find("normal") != std::string::npos)
      type = DataType::normal;
    else if (data_type.find("pos_w") != std::string::npos)
      type = DataType::pos_w;
    else if (data_type.find("pos_b") != std::string::npos)
      type = DataType::pos_b;
    else
      return false;

    return true;
  }
  void print() {
    std::cout << name << std::endl;
    std::cout << "-----is_noise: " << is_noise << std::endl;
    std::cout << "-----is_inf_max: " << is_inf_max << std::endl;
    std::cout << "-----is_inv: " << is_inv << std::endl;
  }
};

class RayPlugin {
public:
  RayPlugin();
  ~RayPlugin() = default;

  /*--------通用接口--------*/
  void Reset(const mjModel *m, int instance);
  void Compute(const mjModel *m, mjData *d, int instance);
  void getBaseCfg(const mjModel *m, mjData *d, int instance);
  void initSensor(const mjModel *m, mjData *d, int instance, int nray);
  void Visualize(const mjModel *m, mjData *d, const mjvOption *opt,
                 mjvScene *scn, int instance);
  template <typename T, std::size_t N1>
  static constexpr auto concat_arrays(const std::array<T, N1> &arr) {
    std::array<T, N1 + base_attributes.size()> result{};
    for (std::size_t i = 0; i < N1; ++i)
      result[i] = arr[i];
    for (std::size_t i = 0; i < base_attributes.size(); ++i)
      result[N1 + i] = base_attributes[i];
    return result;
  }

  VisCfg vis_cfg;
  RayCaster *ray_caster;
  SensorData *sensor_data_list;
  int n_sensor_data;

  static constexpr std::array<const char *, 9> base_attributes = {
      "draw_deep_ray",  "draw_deep_ray_ids", "draw_deep",
      "draw_hip_point", "sensor_data_types", "noise_type",
      "noise_cfg",      "geomgroup",         "detect_parentbody"};
  std::vector<std::pair<std::string_view, int>> noise_attributes = {
      {"uniform", 3}, {"gaussian", 3}, {"noise1", 4}, {"noise2", 8}};
  /*--------通用接口--------*/
};

} // namespace mujoco::plugin::sensor
