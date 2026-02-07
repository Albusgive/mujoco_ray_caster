#include "RayCasterCamera.h"
#include "engine/engine_util_blas.h"
#include "mujoco/mjtnum.h"
#include <cmath>
#include <iostream>
#include <mujoco/mujoco.h>

RayCasterCamera::RayCasterCamera() {}
RayCasterCamera::RayCasterCamera(const RayCasterCameraCfg &cfg) { init(cfg); }
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
  this->v_pixel_size =
      (this->horizontal_aperture / this->aspect_ratio) / cfg.v_ray_num;

  if (this->baseline > 0.0)
    is_compute_hit = true;
  // 调用基类的 protected 初始化函数
  _init(cfg.m, cfg.d, cfg.cam_name, cfg.h_ray_num, cfg.v_ray_num, cfg.dis_range,
        cfg.is_detect_parentbody,cfg.loss_angle);
#if mjVERSION_HEADER >= 341
  left_ray_normal = new mjtNum[h_ray_num * v_ray_num * 3];
  right_ray_normal = new mjtNum[h_ray_num * v_ray_num * 3];
#endif
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
  for (const auto &t : ray_task_datas) {
    StereoTaskData data;
    data.instance = this;
    data.start = t.start;
    data.end = t.end;
    stereo_task_datas.push_back(data);
  }
}

void RayCasterCamera::compute_stereo_ray(int start, int end) {
  int geomid[1];
  mjtNum left_stereo_ray[3];
  mjtNum right_stereo_ray[3];
  for (int idx = start; idx < end; idx++) {
    if (std::isnan(pos_w[idx * 3]) || geomids[idx] == -1 || dist[idx] == 0) {
      continue;
    }
    const mjtNum *target = pos_w + idx * 3;
    mju_sub3(left_stereo_ray, target, left_pos_w);
    mju_sub3(right_stereo_ray, target, right_pos_w);

#if mjVERSION_HEADER >= 341
    mjtNum left_ratio =
        mj_ray(m, d, left_pos_w, left_stereo_ray, geomgroup, 1,
               no_detect_body_id, geomid, left_ray_normal + idx * 3);
    mjtNum right_ratio =
        mj_ray(m, d, right_pos_w, right_stereo_ray, geomgroup, 1,
               no_detect_body_id, geomid, right_ray_normal + idx * 3);
    if (is_loss_angle) {
      mjtNum ldm_ray_normal[3] = {ray_normal[idx * 3],
                                  ray_normal[idx * 3 + 1],
                                  ray_normal[idx * 3 + 2]};
      mju_normalize3(ldm_ray_normal); // 确保物体表面法线是单位向量
      // (表面 -> LDM)
      const mjtNum *ldm_ray_ptr = ray_vec + idx * 3;
      mjtNum L[3] = {-ldm_ray_ptr[0], -ldm_ray_ptr[1], -ldm_ray_ptr[2]};
      mju_normalize3(L);
      // (表面 -> 相机)
      mjtNum V_left[3] = {-left_stereo_ray[0], -left_stereo_ray[1],
                          -left_stereo_ray[2]};
      mju_normalize3(V_left);
      mjtNum V_right[3] = {-right_stereo_ray[0], -right_stereo_ray[1],
                           -right_stereo_ray[2]};
      mju_normalize3(V_right);
      mjtNum cos_cam_left = mju_dot3(ldm_ray_normal, V_left);
      mjtNum cos_cam_right = mju_dot3(ldm_ray_normal, V_right);
      mjtNum cos_light = mju_dot3(ldm_ray_normal, L);
      if (cos_cam_left < loss_angle_cos ||
          cos_cam_right < loss_angle_cos ||
          cos_light < loss_angle_cos) {
        pos_w[idx * 3] = pos_w[idx * 3 + 1] = pos_w[idx * 3 + 2] = NAN;
        dist[idx] = 0;
        dist_ratio[idx] = 0;
        continue;
      }
    }
#else
    mjtNum left_ratio = mj_ray(m, d, left_pos_w, left_stereo_ray, geomgroup, 1,
                               no_detect_body_id, geomid);
    mjtNum right_ratio = mj_ray(m, d, right_pos_w, right_stereo_ray, geomgroup,
                                1, no_detect_body_id, geomid);
#endif
    bool is_valid = (left_ratio >= 0.9999 && left_ratio <= 1.0001) &&
                    (right_ratio >= 0.9999 && right_ratio <= 1.0001);
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
    compute_stereo_ray(0, first_start);
    for (int i = 0; i < num_thread; i++) {
      mju_taskJoin(&tasks[i]);
    }
  } else {
    compute_stereo_ray(0, nray);
  }
}