#pragma once
#include "RayCaster.h"
#include <cmath>
#include <mujoco/mujoco.h>
#include <string>

class RayCasterLidarCfg {
public:
  const mjModel *m = nullptr;
  mjData *d = nullptr;
  std::string cam_name = "";
  mjtNum fov_h = 360.0;
  mjtNum fov_v = 30.0;
  int h_ray_num = 160;
  int v_ray_num = 16;
  std::array<mjtNum, 2> dis_range = {0.1, 100.0};
  bool is_detect_self = false;
};

class RayCasterLidar : public RayCaster {
public:
  RayCasterLidar();
  // 仅使用 Cfg 初始化
  RayCasterLidar(const RayCasterLidarCfg &cfg);
  ~RayCasterLidar();

  void init(const RayCasterLidarCfg &cfg);

private:
  mjtNum fov_h = 50.0;
  mjtNum fov_v = 50.0;
  mjtNum h_res = 0.0; 
  mjtNum v_res = 0.0; 

  void create_rays() override;
};
