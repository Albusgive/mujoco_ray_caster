#include "RayCasterCamera.h"
#include "plugin/mujoco_ray_caster/raycaster_src/RayCaster.h"
#include <cmath>
#include <iostream>
#include <mujoco/mujoco.h>

RayCasterCamera::RayCasterCamera() {}
RayCasterCamera::RayCasterCamera(const RayCasterCameraCfg &cfg) {
  init(cfg);
}
RayCasterCamera::~RayCasterCamera() {
  // 资源释放由基类析构函数处理
}
void RayCasterCamera::init(const RayCasterCameraCfg &cfg) {
  this->baseline = cfg.baseline;
  this->focal_length = cfg.focal_length;
  this->horizontal_aperture = cfg.horizontal_aperture;
  this->vertical_aperture = cfg.vertical_aperture;
  
  // 计算相机参数
  if (this->vertical_aperture == 0) {
    this->aspect_ratio = (double)cfg.h_ray_num / (double)cfg.v_ray_num;
    this->vertical_aperture = this->horizontal_aperture / this->aspect_ratio;
  } else {
    this->aspect_ratio = this->horizontal_aperture / this->vertical_aperture;
  }
  this->h_pixel_size = this->horizontal_aperture / cfg.h_ray_num;
  this->v_pixel_size = (this->horizontal_aperture / this->aspect_ratio) / cfg.v_ray_num;
  
  if (this->baseline > 0.0)
    is_compute_hit = true;
  // 调用基类的 protected 初始化函数
  _init(cfg.m, cfg.d, cfg.cam_name, cfg.h_ray_num, cfg.v_ray_num, cfg.dis_range, cfg.is_detect_parentbody);
}

void RayCasterCamera::compute_ray_vec_virtual_plane() {
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

void RayCasterCamera::set_num_thread(int n) {
  RayCaster::set_num_thread(n);
  stereo_task_datas.clear();
  for(const auto& t : ray_task_datas)
  {
    StereoTaskData data;
    data.is_left = true;
    data.start = t.start;
    data.end = t.end;
    stereo_task_datas.push_back(data);
    data.is_left = false;
    stereo_task_datas.push_back(data);
  }
}

void RayCasterCamera::compute_stereo_ray(bool is_left, int start, int end) {
  int geomid[1];
  mjtNum stereo_ray[3];
  mjtNum pnt[3];
  if (is_left) {
    pnt[0] = left_pos_w[0];
    pnt[1] = left_pos_w[1];
    pnt[2] = left_pos_w[2];
  } else {
    pnt[0] = right_pos_w[0];
    pnt[1] = right_pos_w[1];
    pnt[2] = right_pos_w[2];
  }
  for (int idx = start; idx < end; idx++) {
    if (std::isnan(pos_w[idx * 3]) || geomids[idx] == -1 || dist[idx] == 0) {
      continue;
    }
    mju_sub3(stereo_ray, pos_w + idx * 3, pnt);
    mjtNum ratio =
        mj_ray(m, d, pnt, stereo_ray, geomgroup, 1, no_detect_body_id, geomid);
    bool is_valid = (geomid[0] != -1) && (geomid[0] == geomids[idx]) &&
                    (ratio >= 0.9999 && ratio <= 1.0001);
    if (!is_valid) {
      pos_w[idx * 3] = pos_w[idx * 3 + 1] = pos_w[idx * 3 + 2] = NAN;
      dist[idx] = 0;
      dist_ratio[idx] = 0;
    }
  }
}

void RayCasterCamera::compute_distance() {
  RayCaster::compute_distance();
  if (baseline == 0)
    return;

  // left and right
  mjtNum left_cam_pos_b[3] = {-baseline / 2, 0.0, 0.0};
  mjtNum right_cam_pos_b[3] = {baseline / 2, 0.0, 0.0};
  mju_mulMatVec(left_pos_w, mat, left_cam_pos_b, 3, 3);
  mju_addTo3(left_pos_w, pos);
  mju_mulMatVec(right_pos_w, mat, right_cam_pos_b, 3, 3);
  mju_addTo3(right_pos_w, pos);

  if (num_thread > 0) {
    int n_ = stereo_task_datas.size();
    std::vector<mjTask> tasks(n_);
    for (int i = 0; i < n_; i++) {
      mju_defaultTask(&tasks[i]);
      tasks[i].func = stereo_task_func;
      tasks[i].args = &stereo_task_datas[i];
      mju_threadPoolEnqueue(pool, &tasks[i]);
    }
    int first_start = stereo_task_datas[0].start;
    compute_stereo_ray(true, 0, first_start);
    compute_stereo_ray(false, 0, first_start);
    for (int i = 0; i < num_thread; i++) {
      mju_taskJoin(&tasks[i]);
    }
  } else {
    compute_stereo_ray(true, 0, nray);
    compute_stereo_ray(false, 0, nray);
  }
}