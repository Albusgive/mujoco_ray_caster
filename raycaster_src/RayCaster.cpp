#include "RayCaster.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <mujoco/mujoco.h>
#include <utility>

RayCaster::RayCaster() {}

RayCaster::RayCaster(RayCasterCfg &cfg) {
  init(cfg.m, cfg.d, cfg.cam_name, cfg.resolution, cfg.size, cfg.dis_range,
       cfg.type, cfg.is_detect_parentbody);
}

RayCaster::RayCaster(const mjModel *m, mjData *d, std::string cam_name,
                     mjtNum resolution, const std::array<mjtNum, 2> &size,
                     const std::array<mjtNum, 2> &dis_range, RayCasterType type,
                     bool is_detect_parentbody) {

  init(m, d, cam_name, resolution, size, dis_range, type, is_detect_parentbody);
}

void RayCaster::init(const mjModel *m, mjData *d, std::string cam_name,
                     mjtNum resolution, const std::array<mjtNum, 2> &size,
                     const std::array<mjtNum, 2> &dis_range, RayCasterType type,
                     bool is_detect_parentbody) {
  this->resolution = resolution;
  this->size[0] = size[1];
  this->size[1] = size[0];
  this->type = type;
  _init(m, d, cam_name, (this->size[0] / resolution) + 1,
        (this->size[1] / resolution) + 1, dis_range, is_detect_parentbody);
}

RayCaster::~RayCaster() {
  // delete[] _ray_vec;
  // delete[] ray_vec;
  // delete[] geomids;
  // delete[] dist;
  // delete[] dist_ratio;
}

void RayCaster::_init(const mjModel *m, mjData *d, std::string cam_name,
                      int h_ray_num, int v_ray_num,
                      const std::array<mjtNum, 2> &dis_range,
                      bool is_detect_parentbody) {
  this->m = m;
  this->d = d;
  this->cam_id = mj_name2id(m, mjOBJ_CAMERA, cam_name.c_str());
  if (cam_id == -1) {
    mju_error("RayCaster: no found camera name");
  }
  if (!is_detect_parentbody) {
    no_detect_body_id = m->cam_bodyid[cam_id];
    if (no_detect_body_id == 0)
      no_detect_body_id = -1;
  }
  this->h_ray_num = h_ray_num;
  this->v_ray_num = v_ray_num;
  deep_min = dis_range[0];
  deep_max = dis_range[1];

  nray = h_ray_num * v_ray_num;
  deep_min_ratio = deep_min / deep_max;
  deep_min_ratio_dif = 1.0 - deep_min_ratio;
  deep_min_dif = deep_max - deep_min;

  pos = d->cam_xpos + cam_id * 3;
  mat = d->cam_xmat + cam_id * 9;

  if (type == RayCasterType::none) {
    is_offert = false;
  }
  _ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  ray_vec = new mjtNum[h_ray_num * v_ray_num * 3];
  _ray_vec_offset = new mjtNum[h_ray_num * v_ray_num * 3];
  ray_vec_offset = new mjtNum[h_ray_num * v_ray_num * 3];
  pos_w = new mjtNum[h_ray_num * v_ray_num * 3];
  pos_b = new mjtNum[h_ray_num * v_ray_num * 3];

  geomids = new int[h_ray_num * v_ray_num];
  dist = new mjtNum[h_ray_num * v_ray_num];
  dist_ratio = new mjtNum[h_ray_num * v_ray_num];

  _noise = new ray_noise::Noise;
  create_rays();
}

int RayCaster::get_idx(int v, int h) {
  int idx = _get_idx(v, h);
  if (idx < 0 || idx >= nray) {
    mju_error("Index out of range");
    return -1;
  }
  return idx;
}

int RayCaster::_get_idx(int v, int h) { return v * h_ray_num + h; }

int RayCaster::get_nray(RayCasterCfg &cfg) {
  return ((int)(cfg.size[0] / cfg.resolution) + 1) *
         ((int)(cfg.size[1] / cfg.resolution) + 1);
}

void RayCaster::setNoise(ray_noise::RayNoise2 noise) {
  delete _noise;
  auto n = ray_noise::RayNoise2(noise);
  n.dist = dist;
  n.pos = pos;
  n.pos_w = pos_w;
  n.h_ray_num = h_ray_num;
  n.v_ray_num = v_ray_num;
  is_compute_hit = true;
  _noise = new ray_noise::RayNoise2(n);
  has_noise = true;
}

#ifdef USE_OPENCV
void RayCaster::setNoise(ray_noise::RayNoise3 noise) {
  delete _noise;
  auto n = ray_noise::RayNoise3(noise);
  n.dist = dist;
  n.h_ray_num = h_ray_num;
  n.v_ray_num = v_ray_num;
  n.get_normal_data = [this](double *data, bool is_inf_max, bool is_inv,
                             double scale) {
    get_normal_data(data, is_inf_max, is_inv, scale);
  };
  _noise = new ray_noise::RayNoise3(n);
}
#endif

void RayCaster::compute_ray_vec() {
  if (is_offert) {
    yaw = atan2(mat[3], mat[0]);
  }
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j) * 3;
      if (is_offert) {
        if (type == RayCasterType::base) {
          mju_mulMatVec3(ray_vec + idx, mat, _ray_vec + idx);
          mju_mulMatVec3(ray_vec_offset + idx, mat, _ray_vec_offset + idx);
        }
        if (type == RayCasterType::yaw) {
          rotate_vector_with_yaw(ray_vec_offset + idx, yaw,
                                 _ray_vec_offset + idx);
        }
      } else {
        mju_mulMatVec3(ray_vec + idx, mat, _ray_vec + idx);
      }
    }
  }
}

void RayCaster::create_rays() {

  if (type != RayCasterType::world) {
    mjtNum start_x = -size[0] / 2;
    mjtNum start_y = size[1] / 2;
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j) * 3;
        ray_vec[idx + 0] = _ray_vec[idx + 0] = 0.0;
        ray_vec[idx + 1] = _ray_vec[idx + 1] = 0.0;
        ray_vec[idx + 2] = _ray_vec[idx + 2] = -deep_max;
        ray_vec_offset[idx + 0] = _ray_vec_offset[idx + 0] = start_x;
        ray_vec_offset[idx + 1] = _ray_vec_offset[idx + 1] = start_y;
        ray_vec_offset[idx + 2] = _ray_vec_offset[idx + 2] = 0.0;
        start_x += resolution;
      }
      start_x = -size[0] / 2;
      start_y -= resolution;
    }
  } else {
    // x和y互换
    mjtNum start_x = size[0] / 2;
    mjtNum start_y = size[1] / 2;
    int tmp = h_ray_num;
    h_ray_num = v_ray_num;
    v_ray_num = tmp;
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j) * 3;
        ray_vec[idx + 0] = _ray_vec[idx + 0] = 0.0;
        ray_vec[idx + 1] = _ray_vec[idx + 1] = 0.0;
        ray_vec[idx + 2] = _ray_vec[idx + 2] = -deep_max;
        ray_vec_offset[idx + 0] = _ray_vec_offset[idx + 0] = start_x;
        ray_vec_offset[idx + 1] = _ray_vec_offset[idx + 1] = start_y;
        ray_vec_offset[idx + 2] = _ray_vec_offset[idx + 2] = 0.0;
        start_y -= resolution;
      }
      start_x -= resolution;
      start_y = size[1] / 2;
    }
  }
}

void RayCaster::compute_distance() {
  compute_ray_vec();
  if (is_offert) {
    int geomid[1];
    for (int i = 0; i < nray; i++) {
      mjtNum pnt[3] = {pos[0], pos[1], pos[2]};
      pnt[0] += ray_vec_offset[i * 3];
      pnt[1] += ray_vec_offset[i * 3 + 1];
      pnt[2] += ray_vec_offset[i * 3 + 2];
      dist_ratio[i] = mj_ray(m, d, pnt, ray_vec + i * 3, geomgroup, 1,
                             no_detect_body_id, geomid);
      if (geomid[0] == -1) {
        dist_ratio[i] = 1;
      } else if (dist_ratio[i] > 1) {
        dist_ratio[i] = 1;
      } else if (dist_ratio[i] < deep_min_ratio) {
        dist_ratio[i] = deep_min_ratio;
      }
      dist[i] = deep_max * dist_ratio[i];
      _noise->produce_noise(dist[i]);
      if (dist[i] > deep_max) {
        dist[i] = deep_max;
      } else if (dist[i] < deep_min) {
        dist[i] = deep_min;
      }
    }
  } else {
    mj_multiRay(m, d, pos, ray_vec, geomgroup, 1, no_detect_body_id, geomids,
                dist_ratio, nray, deep_max);
    for (int i = 0; i < nray; i++) {
      if (geomids[i] == -1) {
        dist_ratio[i] = 1;
      } else if (dist_ratio[i] > 1) {
        dist_ratio[i] = 1;
      } else if (dist_ratio[i] < deep_min_ratio) {
        dist_ratio[i] = deep_min_ratio;
      }
      dist[i] = deep_max * dist_ratio[i];
      _noise->produce_noise(dist[i]);
      if (dist[i] > deep_max) {
        dist[i] = deep_max;
      } else if (dist[i] < deep_min) {
        dist[i] = deep_min;
      }
    }
  }
  if (is_compute_hit)
    compute_hit();
  if (is_compute_hit_b)
    compute_hit_b();
  _noise->produce_united_noise();
}

void RayCaster::get_inv_image_data(unsigned char *image_data, bool is_noise,
                                   bool is_inf_max) {
  get_normal_data(image_data, is_noise, is_inf_max, true, 255);
}

void RayCaster::get_image_data(unsigned char *image_data, bool is_noise,
                               bool is_inf_max, bool is_inv) {
  get_normal_data(image_data, is_noise, is_inf_max, false, 255);
}

std::vector<double> RayCaster::get_normal_data_vec(bool is_noise,
                                                   bool is_inf_max, bool is_inv,
                                                   double scale) {
  std::vector<double> data(nray);
  get_normal_data(data, is_noise, is_inf_max, is_inv, scale);
  return data;
}

std::vector<double> RayCaster::get_data_vec(bool is_inf_max) {
  if (is_inf_max)
    return std::vector<double>(dist, dist + nray);
  else {
    std::vector<double> vec(nray);
    for (int i = 0; i < nray; i++) {
      if (geomids[i] < 0)
        vec[i] = 0.0;
      else
        vec[i] = dist[i];
    }
    return vec;
  }
}

std::vector<std::vector<double>> RayCaster::get_data_pos_w() {
  std::vector<std::vector<double>> pos(nray, std::vector<double>(3, 0));
  _get_data_pos_dim2(pos, pos_w);
  return pos;
}

std::vector<std::vector<double>> RayCaster::get_data_pos_b() {
  std::vector<std::vector<double>> pos(nray, std::vector<double>(3, 0));
  _get_data_pos_dim2(pos, pos_b);
  return pos;
}

void RayCaster::draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                          float *rgba) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, mjGEOM_SPHERE, NULL, NULL, NULL, rgba);
  mjv_connector(geom, mjGEOM_LINE, width, from, to);
}

void RayCaster::draw_geom(mjvScene *scn, int type, mjtNum *size, mjtNum *pos,
                          mjtNum *mat, float rgba[4]) {
  scn->ngeom += 1;
  mjvGeom *geom = scn->geoms + scn->ngeom - 1;
  mjv_initGeom(geom, type, size, pos, mat, rgba);
}

void RayCaster::draw_ary(int idx, int width, float *color, mjvScene *scn,
                         bool is_scale) {
  mjtNum start[3] = {pos[0], pos[1], pos[2]};
  mjtNum end[3] = {pos[0], pos[1], pos[2]};
  if (is_offert) {
    start[0] = end[0] += ray_vec_offset[idx * 3];
    start[1] = end[1] += ray_vec_offset[idx * 3 + 1];
    start[2] = end[2] += ray_vec_offset[idx * 3 + 2];
  }
  if (is_scale)
    mju_addToScl3(end, ray_vec + (idx * 3), dist_ratio[idx]);
  else
    mju_addTo3(end, ray_vec + (idx * 3));
  draw_line(scn, start, end, width, color);
}

void RayCaster::draw_deep_ray(mjvScene *scn, int ratio, int width, bool edge,
                              float *color) {
  float color_[4] = {1.0, 0.0, 0.0, 1.0};
  if (color != nullptr) {
    color_[0] = color[0];
    color_[1] = color[1];
    color_[2] = color[2];
    color_[3] = color[3];
  }
  if (edge) {
    for (int i = 0; i < v_ray_num; i += ratio) {
      int idx = _get_idx(i, 0);
      draw_ary(idx, width, color_, scn, false);
      idx = _get_idx(i, h_ray_num - 1);
      draw_ary(idx, width, color_, scn, false);
    }
    for (int j = 0; j < h_ray_num; j += ratio) {
      int idx = _get_idx(0, j);
      draw_ary(idx, width, color_, scn, false);
      idx = _get_idx(v_ray_num - 1, j);
      draw_ary(idx, width, color_, scn, false);
    }
  } else {
    for (int i = 0; i < v_ray_num; i += ratio) {
      for (int j = 0; j < h_ray_num; j += ratio) {
        int idx = _get_idx(i, j);
        draw_ary(idx, width, color_, scn, false);
      }
    }
  }
}

void RayCaster::draw_deep_ray(mjvScene *scn, int idx, int width, float *color) {
  float color_[4] = {1.0, 0.0, 0.0, 1.0};
  if (color != nullptr) {
    color_[0] = color[0];
    color_[1] = color[1];
    color_[2] = color[2];
    color_[3] = color[3];
  }
  mjtNum start[3] = {pos[0], pos[1], pos[2]};
  mjtNum end[3] = {pos[0], pos[1], pos[2]};
  if (is_offert) {
    start[0] = end[0] += ray_vec_offset[idx * 3];
    start[1] = end[1] += ray_vec_offset[idx * 3 + 1];
    start[2] = end[2] += ray_vec_offset[idx * 3 + 2];
  }
  mju_addTo3(end, ray_vec + idx * 3);
  draw_line(scn, start, end, width, color_);
}

void RayCaster::draw_deep(mjvScene *scn, int ratio, int width, float *color) {
  float color_[4] = {1.0, 0.0, 0.0, 1.0};
  if (color != nullptr) {
    color_[0] = color[0];
    color_[1] = color[1];
    color_[2] = color[2];
    color_[3] = color[3];
  }
  for (int i = 0; i < v_ray_num; i += ratio) {
    for (int j = 0; j < h_ray_num; j += ratio) {
      int idx = _get_idx(i, j);
      draw_ary(idx, width, color_, scn, true);
    }
  }
}

void RayCaster::draw_hip_point(mjvScene *scn, int ratio, mjtNum size,
                               float *color) {
  float color_[4] = {1.0, 0.0, 0.0, 1.0};
  if (color != nullptr) {
    color_[0] = color[0];
    color_[1] = color[1];
    color_[2] = color[2];
    color_[3] = color[3];
  }
  mjtNum size_[3] = {size, size, size};
  mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int i = 0; i < v_ray_num; i += ratio) {
    for (int j = 0; j < h_ray_num; j += ratio) {
      int idx = _get_idx(i, j);
      if (pos_w[idx * 3] == NAN)
        continue;
      draw_geom(scn, mjGEOM_SPHERE, size_, pos_w + (idx * 3), mat, color_);
    }
  }
}

void RayCaster::rotate_vector_with_yaw(mjtNum result[3], mjtNum yaw,
                                       const mjtNum vec[3]) {
  mjtNum c = cos(yaw);
  mjtNum s = sin(yaw);
  result[0] = c * vec[0] - s * vec[1];
  result[1] = s * vec[0] + c * vec[1];
  result[2] = vec[2];
}

void RayCaster::compute_hit() {
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j);
      if (geomids[idx] == -1 || dist[idx] == 0 || dist_ratio[idx] > LIMIT_MAX) {
        pos_w[idx * 3] = pos_w[idx * 3 + 1] = pos_w[idx * 3 + 2] = NAN;
        continue;
      }
      pos_w[idx * 3] = pos[0];
      pos_w[idx * 3 + 1] = pos[1];
      pos_w[idx * 3 + 2] = pos[2];
      if (is_offert) {
        pos_w[idx * 3] += ray_vec_offset[idx * 3];
        pos_w[idx * 3 + 1] += ray_vec_offset[idx * 3 + 1];
        pos_w[idx * 3 + 2] += ray_vec_offset[idx * 3 + 2];
      }
      mju_addToScl3(pos_w + (idx * 3), ray_vec + (idx * 3), dist_ratio[idx]);
    }
  }
}

void RayCaster::compute_hit_b() {
  for (int i = 0; i < v_ray_num; i++) {
    for (int j = 0; j < h_ray_num; j++) {
      int idx = _get_idx(i, j);
      if (geomids[idx] == -1 || dist[idx] == 0 || dist_ratio[idx] > LIMIT_MAX) {
        pos_b[idx * 3] = pos_b[idx * 3 + 1] = pos_b[idx * 3 + 2] = NAN;
        continue;
      }
      pos_b[idx * 3] = 0.0;
      pos_b[idx * 3 + 1] = 0.0;
      pos_b[idx * 3 + 2] = 0.0;
      if (is_offert) {
        pos_b[idx * 3] += _ray_vec_offset[idx * 3];
        pos_b[idx * 3 + 1] += _ray_vec_offset[idx * 3 + 1];
        pos_b[idx * 3 + 2] += _ray_vec_offset[idx * 3 + 2];
      }
      mju_addToScl3(pos_b + (idx * 3), _ray_vec + (idx * 3), dist_ratio[idx]);
    }
  }
}