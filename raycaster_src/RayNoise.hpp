#pragma once
#include "Noise.hpp"
#include "mujoco/mjtnum.h"
#include "mujoco/mujoco.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ray_noise {
using namespace std_noise;

// 每个像素按照一定概率置0
class RayNoise1 : public ray_noise::DistributionNoise<
                      std::uniform_real_distribution<mjtNum>> {
public:
  RayNoise1(mjtNum low = 0.0, mjtNum high = 1.0, mjtNum zero_probability = 0.0,
            unsigned int seed = 0)
      : DistributionNoise(low, high), _zero_probability(zero_probability) {
    if (seed != 0) {
      set_seed(seed);
    }
    dist_zero = std::uniform_real_distribution<mjtNum>(0, 1);
  }

  mjtNum get_low() const { return dist.a(); }
  mjtNum get_high() const { return dist.b(); }
  mjtNum get_zero_probability() const { return _zero_probability; }
  void set_params(mjtNum low, mjtNum high, mjtNum zero_probability) {
    dist = std::uniform_real_distribution<mjtNum>(low, high);
    _zero_probability = zero_probability;
  }

  void produce_noise(mjtNum &data) override {
    data += static_cast<mjtNum>(dist(gen));
    mjtNum zero_p = dist_zero(gen);
    if (zero_p < _zero_probability)
      data = 0;
  }

protected:
  mjtNum _zero_probability = 0.0;
  std::uniform_real_distribution<mjtNum> dist_zero;
};

// 前三个为RayNoise1参数，从min_angle到max_angle之间(最高180)为0概率逐渐升高，大于max_angle保持最高概率
class RayNoise2 : public RayNoise1 {
public:
  RayNoise2(mjtNum low, mjtNum high, mjtNum zero_probability, mjtNum min_angle,
            mjtNum max_angle, mjtNum low_probability = 0.1,
            mjtNum high_probability = 0.5, unsigned int seed = 0)
      : RayNoise1(low, high, zero_probability), min_angle(min_angle),
        max_angle(max_angle), low_probability(low_probability),
        high_probability(high_probability) {
    _angle_dif = max_angle - min_angle;
  };
  mjtNum min_angle;
  mjtNum max_angle;
  mjtNum low_probability;
  mjtNum high_probability;
  mjtNum *dist;
  mjtNum *pos;
  mjtNum *pos_w;
  int h_ray_num;
  int v_ray_num;

  void produce_united_noise() {
    std::vector<int> zero_id;
    // 预留内存以提高性能，虽然只是大概估计
    zero_id.reserve(h_ray_num * v_ray_num / 10);

    for (int i = 1; i < v_ray_num - 1; i++) {
      for (int j = 1; j < h_ray_num - 1; j++) {
        int idx = _get_idx(i, j);
        mjtNum center_val = dist[idx];

        // 如果中心点本身无效（比如已经是0或者无限远），可能不需要计算
        // 这里保留原逻辑，不轻易跳过

        int max_idx = -1;
        mjtNum max_diff = -std::numeric_limits<mjtNum>::max();

        auto check_neighbor = [&](int ni, int nj) {
          int n_idx = _get_idx(ni, nj);
          mjtNum diff = dist[n_idx] - center_val;
          if (diff > max_diff) {
            max_diff = diff;
            max_idx = n_idx;
          }
        };

        check_neighbor(i - 1, j - 1);
        check_neighbor(i - 1, j);
        check_neighbor(i - 1, j + 1);
        check_neighbor(i, j - 1);
        check_neighbor(i, j + 1);
        check_neighbor(i + 1, j - 1);
        check_neighbor(i + 1, j);
        check_neighbor(i + 1, j + 1);

        if (max_idx == -1)
          continue;

        int data_pos = idx * 3;
        int data_pos2 = max_idx * 3;

        mjtNum angle = getDegAngle(pos, pos_w + data_pos, pos_w + data_pos2);

        if (angle > max_angle) {
          mjtNum zero_p = dist_zero(gen);
          if (zero_p < high_probability)
            zero_id.push_back(idx);
        } else if (angle > min_angle) {
          mjtNum p = ((angle - min_angle) / _angle_dif) *
                         (high_probability - low_probability) +
                     low_probability;
          mjtNum zero_p = dist_zero(gen);
          if (zero_p < p)
            zero_id.push_back(idx);
        }
      }
    }
    int len = zero_id.size();
    for (int k = 0; k < len; k++) {
      dist[zero_id[k]] = 0;
    }
  }

private:
  mjtNum _angle_dif;
  int _get_idx(int v, int h) { return v * h_ray_num + h; }

  // 手动实现向量计算，移除 Eigen 依赖
  // p1: 相机位置, p2: 当前点位置, p3: 邻居点位置
  // 计算向量 v1 = p1 - p2 (当前点指向相机)
  // 计算向量 v2 = p3 - p2 (当前点指向邻居点)
  mjtNum getDegAngle(const mjtNum *p1, const mjtNum *p2, const mjtNum *p3) {
    // 1. 向量减法
    mjtNum v1[3] = {p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]};
    mjtNum v2[3] = {p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]};

    // 2. 点积 (Dot Product): a·b
    mjtNum dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];

    // 3. 叉积 (Cross Product): a x b
    // cx = ay*bz - az*by
    // cy = az*bx - ax*bz
    // cz = ax*by - ay*bx
    mjtNum cross_x = v1[1] * v2[2] - v1[2] * v2[1];
    mjtNum cross_y = v1[2] * v2[0] - v1[0] * v2[2];
    mjtNum cross_z = v1[0] * v2[1] - v1[1] * v2[0];

    // 4. 叉积的模长 (Norm of Cross Product)
    mjtNum cross_norm =
        std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);

    // 5. 计算角度 (atan2 更稳定)
    // |a x b| = |a||b|sin(theta)
    // a · b   = |a||b|cos(theta)
    // tan(theta) = |a x b| / (a · b)
    mjtNum radian_angle = std::atan2(cross_norm, dot);

    return radian_angle * 180.0 / M_PI;
  }
};

class RayNoise3 : public std_noise::Noise {
public:
  RayNoise3(double max_incidence_angle_deg) {
    if (max_incidence_angle_deg < 0)
      max_incidence_angle_deg = 0;
    if (max_incidence_angle_deg > 90)
      max_incidence_angle_deg = 90;
    double rad = max_incidence_angle_deg * M_PI / 180.0;
    this->min_reflection_intensity = std::cos(rad);
  };
  mjtNum *ray_vec;
  mjtNum *ray_normal;
  bool *is_lost;
  mjtNum min_reflection_intensity;
  int nray = 0;
  void produce_united_noise() {
    for (int i = 0; i < nray; ++i) {
      int idx = i * 3;
      mjtNum dot = ray_vec[idx] * ray_normal[idx] +
                   ray_vec[idx + 1] * ray_normal[idx + 1] +
                   ray_vec[idx + 2] * ray_normal[idx + 2];
      mjtNum intensity = (dot < 0) ? -dot : dot;
      if (intensity < min_reflection_intensity) {
        is_lost[i] = true;
      } else {
        is_lost[i] = false;
      }
    }
  };
};

class RayNoise4 : public std_noise::Noise {
  // only stereo camera TODO
public:
  RayNoise4() {

  };
  mjtNum *ray_vec;
  mjtNum *ray_normal;
  mjtNum *left_ray_normal;
  mjtNum *right_ray_normal;
  bool *is_lost;
  int nray = 0;
  void produce_united_noise() {};
};
} // namespace ray_noise
