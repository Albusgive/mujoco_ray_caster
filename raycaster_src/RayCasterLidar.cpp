#include "RayCasterLidar.h"
#include <mujoco/mujoco.h>

RayCasterLidar::RayCasterLidar() {}

RayCasterLidar::RayCasterLidar(RayCasterLidarCfg &cfg) {

  init(cfg.m, cfg.d, cfg.cam_name, cfg.fov_h, cfg.fov_v, cfg.h_ray_num,
       cfg.v_ray_num, cfg.dis_range, cfg.is_detect_self);
}

RayCasterLidar::RayCasterLidar(const mjModel *m, mjData *d, std::string cam_name,
                               mjtNum fov_h, mjtNum fov_v, int h_ray_num,
                               int v_ray_num,
                               const std::array<mjtNum, 2> &dis_range,
                               bool is_detect_self) {
  init(m, d, cam_name, fov_h, fov_v, h_ray_num, v_ray_num, dis_range,
       is_detect_self);
}

RayCasterLidar::~RayCasterLidar() {
  // delete[] _ray_vec;
  // delete[] ray_vec;
  // delete[] geomids;
  // delete[] dist;
  // delete[] dist_ratio;
}

void RayCasterLidar::init(const mjModel *m, mjData *d, std::string cam_name,
                          mjtNum fov_h, mjtNum fov_v, int h_ray_num,
                          int v_ray_num, const std::array<mjtNum, 2> &dis_range,
                          bool is_detect_self) {
  this->fov_h = fov_h;
  this->fov_v = fov_v;
  h_res = fov_h / (h_ray_num - 1);
  v_res = fov_v / (v_ray_num - 1);
  _init(m, d, cam_name, h_ray_num, v_ray_num, dis_range, is_detect_self);
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