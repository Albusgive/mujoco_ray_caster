#pragma once
#include "Noise.hpp"
#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <vector>

#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
#endif

typedef Eigen::Vector3d Point;

namespace ray_noise {
using namespace std_noise;

// 每个像素按照一定概率置0
class RayNoise1 : public ray_noise::DistributionNoise<
                      std::uniform_real_distribution<double>> {
public:
  RayNoise1(double low = 0.0, double high = 1.0, double zero_probability = 0.0,
            unsigned int seed = 0)
      : DistributionNoise(low, high), _zero_probability(zero_probability) {
    if (seed != 0) {
      set_seed(seed);
    }
    dist_zero = std::uniform_real_distribution<double>(0, 1);
  }

  double get_low() const { return dist.a(); }
  double get_high() const { return dist.b(); }
  double get_zero_probability() const { return _zero_probability; }
  void set_params(double low, double high, double zero_probability) {
    dist = std::uniform_real_distribution<double>(low, high);
    _zero_probability = zero_probability;
  }
  void produce_noise(float &data) override {
    data += static_cast<float>(dist(gen));
    double zero_p = dist_zero(gen);
    if (zero_p < _zero_probability)
      data = 0;
  }

  void produce_noise(double &data) override {
    data += static_cast<double>(dist(gen));
    double zero_p = dist_zero(gen);
    if (zero_p < _zero_probability)
      data = 0;
  }

protected:
  double _zero_probability = 0.0;
  std::uniform_real_distribution<double> dist_zero;
};

// 前三个为RayNoise1参数，从min_angle到max_angle之间(最高180)为0概率逐渐升高，大于max_angle保持最高概率
class RayNoise2 : public RayNoise1 {
public:
  RayNoise2(double low, double high, double zero_probability, double min_angle,
            double max_angle, double low_probability = 0.1,
            double high_probability = 0.5, unsigned int seed = 0)
      : RayNoise1(low, high, zero_probability), min_angle(min_angle),
        max_angle(max_angle), low_probability(low_probability),
        high_probability(high_probability) {
    _angle_dif = max_angle - min_angle;
  };
  double min_angle;
  double max_angle;
  double low_probability;
  double high_probability;
  double *dist;
  double *pos;
  double *pos_w;
  int h_ray_num;
  int v_ray_num;
  void produce_united_noise() {
    std::vector<int> zero_id;
    for (int i = 1; i < v_ray_num - 1; i++) {
      for (int j = 1; j < h_ray_num - 1; j++) {
        int idx = _get_idx(i, j);
        double center_val = dist[idx];
        int max_idx = -1;
        double max_diff = -std::numeric_limits<double>::max();
        auto check_neighbor = [&](int ni, int nj) {
          int n_idx = _get_idx(ni, nj);
          double diff = dist[n_idx] - center_val;
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
        int data_pos = idx * 3;
        int data_pos2 = max_idx * 3;
        double angle = getDegAngle(
            Point(pos[0], pos[1], pos[2]),
            Point(pos_w[data_pos], pos_w[data_pos + 1], pos_w[data_pos + 2]),
            Point(pos_w[data_pos2], pos_w[data_pos2 + 1],
                  pos_w[data_pos2 + 2]));
        if (angle > max_angle) {
          double zero_p = dist_zero(gen);
          if (zero_p < high_probability)
            zero_id.push_back(idx);
        } else if (angle > min_angle) {
          double p = ((angle - min_angle) / _angle_dif) *
                         (high_probability - low_probability) +
                     low_probability;
          double zero_p = dist_zero(gen);
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
  double _angle_dif;
  int _get_idx(int v, int h) { return v * h_ray_num + h; }
  double getDegAngle(Point p1, Point p2, Point p3) {
    Eigen::Vector3d v1 = p1 - p2;
    Eigen::Vector3d v2 = p3 - p2;
    double dot = v1.dot(v2);
    double cross_norm = v1.cross(v2).norm();
    double radian_angle = atan2(cross_norm, dot);
    return radian_angle * 180 / M_PI;
  }
};

#ifdef USE_OPENCV
// 前三个为RayNoise1参数，从min_angle到max_angle之间(最高180)为0概率逐渐升高，大于max_angle保持最高概率
// gradient range为 -16-16
class RayNoise3 : public RayNoise1 {
public:
  RayNoise3(double low, double high, double zero_probability,
            double min_gradient, double max_gradient,
            double low_probability = 0.1, double high_probability = 0.5,
            unsigned int seed = 0)
      : RayNoise1(low, high, zero_probability), min_gradient(min_gradient),
        max_gradient(max_gradient), low_probability(low_probability),
        high_probability(high_probability) {
    _gradient_dif = max_gradient - min_gradient;
  };
  double min_gradient;
  double max_gradient;
  double low_probability;
  double high_probability;
  double *normal = nullptr;
  double *dist;
  double *nor;
  int h_ray_num;
  int v_ray_num;
  std::function<void(double *data, bool is_inf_max, bool is_inv, double scale)>
      get_normal_data;
  void produce_united_noise() {
    if (normal == nullptr)
      normal = new double[h_ray_num * v_ray_num];
    get_normal_data(normal, true, false, 1.0);
    cv::Mat src(v_ray_num, h_ray_num, CV_64FC1, normal);
    cv::Mat scharr_x, scharr_y;
    cv::Mat scharr_grad;
    cv::Scharr(src, scharr_x, CV_64FC1, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::Scharr(src, scharr_y, CV_64FC1, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    cv::magnitude(scharr_x, scharr_y, scharr_grad);
    int totalElements = scharr_grad.rows * scharr_grad.cols;
    auto _dist_zero = [&](double &data, int idx) {
      if (data > max_gradient) {
        double zero_p = dist_zero(gen);
        if (zero_p < high_probability)
          dist[idx] = 0;
      } else if (data > min_gradient) {
        double p = ((data - min_gradient) / _gradient_dif) *
                       (high_probability - low_probability) +
                   low_probability;
        double zero_p = dist_zero(gen);
        if (zero_p < p)
          dist[idx] = 0;
      }
    };

    if (scharr_grad.isContinuous()) {
      double *data = scharr_grad.ptr<double>(0);
      for (int i = 0; i < totalElements; i++) {
        _dist_zero(data[i], i);
      }
    } else {
      for (int y = 0; y < scharr_grad.rows; y++) {
        double *row_ptr = scharr_grad.ptr<double>(y);
        for (int x = 0; x < scharr_grad.cols; x++) {
          _dist_zero(row_ptr[x], _get_idx(y, x));
        }
      }
    }
  }

private:
  double _gradient_dif;
  int _get_idx(int v, int h) { return v * h_ray_num + h; }
};
#endif
} // namespace ray_noise
