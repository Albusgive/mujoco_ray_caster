#include "RayCasterCamera.h"
#include <mujoco/mujoco.h>

RayCasterCamera::RayCasterCamera() {}

RayCasterCamera::RayCasterCamera(RayCasterCameraCfg &cfg) {
  init(cfg.m, cfg.d, cfg.cam_name, cfg.focal_length, cfg.horizontal_aperture,
       cfg.h_ray_num, cfg.v_ray_num, cfg.dis_range, cfg.vertical_aperture,
       cfg.is_detect_parentbody);
}

RayCasterCamera::RayCasterCamera(const mjModel *m, mjData *d, std::string cam_name,
                                 mjtNum focal_length,
                                 mjtNum horizontal_aperture, int h_ray_num,
                                 int v_ray_num,
                                 const std::array<mjtNum, 2> &dis_range,
                                 mjtNum vertical_aperture,
                                 bool is_detect_parentbody) {
  init(m, d, cam_name, focal_length, horizontal_aperture, h_ray_num, v_ray_num,
       dis_range, vertical_aperture, is_detect_parentbody);
}

RayCasterCamera::~RayCasterCamera() {
  // delete[] _ray_vec;
  // delete[] ray_vec;
  // delete[] geomids;
  // delete[] dist;
  // delete[] dist_ratio;
}

void RayCasterCamera::init(const mjModel *m, mjData *d, std::string cam_name,
                           mjtNum focal_length, mjtNum horizontal_aperture,
                           int h_ray_num, int v_ray_num,
                           const std::array<mjtNum, 2> &dis_range,
                           mjtNum vertical_aperture,
                           bool is_detect_parentbody) {
  this->focal_length = focal_length;
  this->horizontal_aperture = horizontal_aperture;
  this->vertical_aperture = vertical_aperture;
  if (vertical_aperture == 0) {
    this->aspect_ratio = (double)h_ray_num / (double)v_ray_num;
    this->vertical_aperture = horizontal_aperture / aspect_ratio;
  } else
    this->aspect_ratio = horizontal_aperture / vertical_aperture;

  h_pixel_size = horizontal_aperture / h_ray_num;
  v_pixel_size = (horizontal_aperture / aspect_ratio) / v_ray_num;
  _init(m, d, cam_name, h_ray_num, v_ray_num, dis_range, is_detect_parentbody);
}

void RayCasterCamera::compute_ray_vec_virtual_plane() {
  // mjtNum vertical_aperture = horizontal_aperture / aspect_ratio;

  // mjtNum half_width = horizontal_aperture / 2.0;
  // mjtNum half_height = vertical_aperture / 2.0;

  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      mjtNum x = (j + 0.5 - h_ray_num / 2.0) * h_pixel_size;
      mjtNum y = (v_ray_num / 2.0 - i - 0.5) * v_pixel_size; // Y轴翻转
      mjtNum z = -focal_length;                              // 相机方向是-z

      mjtNum point_on_plane[3] = {x, y, z};

      mjtNum norm = mju_norm3(point_on_plane);
      mjtNum dir[3] = {point_on_plane[0] / norm, point_on_plane[1] / norm,
                       point_on_plane[2] / norm};

      int idx = _get_idx(i, j) * 3;
      _ray_vec[idx + 0] = dir[0] * deep_max;
      _ray_vec[idx + 1] = dir[1] * deep_max;
      _ray_vec[idx + 2] = dir[2] * deep_max;
    }
  }
}
