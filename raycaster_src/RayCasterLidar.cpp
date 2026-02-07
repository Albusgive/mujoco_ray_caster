#include "RayCasterLidar.h"
#include <mujoco/mujoco.h>

RayCasterLidar::RayCasterLidar() {}

RayCasterLidar::RayCasterLidar(const RayCasterLidarCfg &cfg) { init(cfg); }

RayCasterLidar::~RayCasterLidar() {}

void RayCasterLidar::init(const RayCasterLidarCfg &cfg) {
  this->fov_h = cfg.fov_h;
  this->fov_v = cfg.fov_v;

  // 防止除0
  if (cfg.h_ray_num > 1)
    this->h_res = this->fov_h / (cfg.h_ray_num - 1);
  else
    this->h_res = 0;

  if (cfg.v_ray_num > 1)
    this->v_res = this->fov_v / (cfg.v_ray_num - 1);
  else
    this->v_res = 0;

  // 调用基类初始化
  _init(cfg.m, cfg.d, cfg.cam_name, cfg.h_ray_num, cfg.v_ray_num, cfg.dis_range,
        cfg.is_detect_self, cfg.loss_angle);
}

void RayCasterLidar::create_rays() {
  mjtNum ref_vec[3] = {0.0, 0.0, -deep_max};
  mjtNum axis_x[3] = {1.0, 0.0, 0.0};
  mjtNum quat_x[4];
  mjtNum axis_y[3] = {0.0, 1.0, 0.0};
  mjtNum quat_y[4];
  mjtNum combined_quat[4];
  mjtNum res_vec[3];
  mjtNum start_h_angle = fov_h / 2.0;
  mjtNum start_v_angle = fov_v / 2.0;
  for (int i = 0; i < v_ray_num; i++) {
    mjtNum angle_x = ((start_v_angle - v_res * i) / 180.0) * mjPI;
    mju_axisAngle2Quat(quat_x, axis_x, angle_x);
    for (int j = 0; j < h_ray_num; j++) {
      mjtNum angle_y = ((start_h_angle - h_res * j) / 180.0) * mjPI;
      mju_axisAngle2Quat(quat_y, axis_y, angle_y);
      mju_mulQuat(combined_quat, quat_y, quat_x);
      // mju_mulQuat(combined_quat, quat_x, quat_y);
      mju_rotVecQuat(res_vec, ref_vec, combined_quat);
      int idx = _get_idx(i, j) * 3;
      _ray_vec[idx + 0] = res_vec[0];
      _ray_vec[idx + 1] = res_vec[1];
      _ray_vec[idx + 2] = res_vec[2];
    }
  }
}