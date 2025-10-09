#pragma once
#include "Noise.hpp"
#include "RayNoise.hpp"
#include "mujoco/mjthread.h"
#include "mujoco/mjtnum.h"
#include <array>
#include <cmath>
#include <iostream>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <type_traits>
#include <vector>

#define LIMIT_MAX 0.999999999

enum RayCasterType { base, yaw, world, none };

class RayCasterCfg {
public:
  const mjModel *m;
  mjData *d;
  std::string cam_name;
  mjtNum resolution;
  std::array<mjtNum, 2> size;
  std::array<mjtNum, 2> dis_range;
  RayCasterType type = RayCasterType::none;
  bool is_detect_parentbody = false;
};

class RayCaster {
public:
  RayCaster();
  RayCaster(RayCasterCfg &cfg);
  RayCaster(const mjModel *m, mjData *d, std::string cam_name,
            mjtNum resolution, const std::array<mjtNum, 2> &size,
            const std::array<mjtNum, 2> &dis_range,
            RayCasterType type = RayCasterType::none,
            bool is_detect_parentbody = false);
  ~RayCaster();
  /** @brief 初始化
   * @param m mjModel
   * @param d mjData
   * @param cam_name 相机名称
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   * @param is_detect_parentbody 是否检测自身
   */
  void _init(const mjModel *m, mjData *d, std::string cam_name, int h_ray_num,
             int v_ray_num, const std::array<mjtNum, 2> &dis_range,
             bool is_detect_parentbody);

  /** @brief 设置线程数量
   * @param n 线程数量
   */
  void set_num_thread(int n);

  /** @brief 计算距离 数值存放在dist中*/
  void compute_distance();

  /** @brief 绘制测量射线 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param width 射线宽度
   * @param edge 仅绘制边缘
   * @param color 颜色
   */
  void draw_deep_ray(mjvScene *scn, int ratio, int width = 5, bool edge = false,
                     float *color = nullptr);
  /** @brief 绘制特定射线 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param idx 射线索引 get_idx获取
   * @param width 射线宽度
   * @param color 颜色
   */
  void draw_deep_ray(mjvScene *scn, int idx, int width = 5,
                     float *color = nullptr);

  /** @brief 绘制距离线段 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param width 射线宽度
   * @param color 颜色
   */
  void draw_deep(mjvScene *scn, int ratio, int width = 5,
                 float *color = nullptr);
  /** @brief 绘制射线命中点 在mjv_updateScene和mjr_render中间
   * @param scn mjvScene
   * @param ratio 绘制比例
   * @param size 点大小
   * @param color 颜色
   */
  void draw_hip_point(mjvScene *scn, int ratio, mjtNum size = 0.1,
                      float *color = nullptr);

  /** @brief 获取dist中索引
  * @param h 水平索引
  * @param v 垂直索引
  无效索引返回-1
  */
  int get_idx(int h, int v);

  template <typename NoiseType> void setNoise(NoiseType noise) {
    delete _noise;
    _noise = new NoiseType(noise);
    if constexpr (std::is_same_v<NoiseType, ray_noise::Noise>)
      return;
    has_noise = true;
  }
  bool has_noise = false;

  static int get_nray(RayCasterCfg &cfg);
  void setNoise(ray_noise::RayNoise2 noise);
#ifdef USE_OPENCV
  void setNoise(ray_noise::RayNoise3 noise);
#endif
  ray_noise::Noise *_noise;

  mjtNum *dist;               // 距离 h_ray_num * v_ray_num
  int nray;                   // 射线数量
  int no_detect_body_id = -1; // 是否检测 id 不检测就是-1

  const mjModel *m;
  mjData *d;
  int cam_id;  // 相机id
  mjtNum *pos; // 相机位置
  mjtNum *mat; // 相机的旋转矩阵
  mjtNum yaw = 0.0;
  int h_ray_num = 50; // 水平
  int v_ray_num = 50; // 垂直
  mjtNum deep_max = 1e6;
  mjtNum deep_min = 0;
  mjtNum deep_min_ratio;
  mjtNum deep_min_ratio_dif;
  mjtNum deep_min_dif;
  mjtNum *_ray_vec; // h_ray_num * v_ray_num * 3 相对于相机坐标系的偏转
  mjtNum *_ray_vec_offset; // h_ray_num * v_ray_num * 3 相对于相机坐标系的位移
  mjtNum *ray_vec; // h_ray_num * v_ray_num * 3 世界坐标系下的偏转
  mjtNum *ray_vec_offset; // h_ray_num * v_ray_num * 3 世界坐标系下的位移
  int *geomids;           // 命中的geomid
  mjtNum *dist_ratio;
  mjtByte geomgroup[8] = {true,  true,  false,
                          false, false, false}; // 检测哪些类型的geom
  bool is_offert = true;
  RayCasterType type = RayCasterType::none;
  int num_thread = 0;

  int _get_idx(int h, int v);
  // 将ray从相机坐标系转换到世界坐标系
  void compute_ray_vec();

  // 初始化时创建射线相对于相机坐标系偏转向量 _ray_vec，非单位向量
  virtual void create_rays();

  void get_inv_image_data(unsigned char *image_data, bool is_noise = false,
                          bool is_inf_max = true);

  void get_image_data(unsigned char *image_data, bool is_noise = false,
                      bool is_inf_max = true, bool is_inv = false);

  std::vector<double> get_normal_data_vec(bool is_noise, bool is_inf_max,
                                          bool is_inv, double scale = 1.0);

  // 直接测量距离信息
  std::vector<double> get_data_vec(bool is_inf_max = true);

  // 世界坐标系命中位置 没命中的返回(NAN,NAN,NAN)
  std::vector<std::vector<double>> get_data_pos_w();
  // 自身坐标系命中位置 没命中的返回(NAN,NAN,NAN)
  std::vector<std::vector<double>> get_data_pos_b();

  void draw_line(mjvScene *scn, mjtNum *from, mjtNum *to, mjtNum width,
                 float *rgba);
  void draw_geom(mjvScene *scn, int type, mjtNum *size, mjtNum *pos,
                 mjtNum *mat, float rgba[4]);

  void rotate_vector_with_yaw(mjtNum result[3], mjtNum yaw,
                              const mjtNum vec[3]);

  // 如果不绘制落点可以关掉提高性能，ray_noise::RayNoise2会自动开启is_compute_hit
  bool is_compute_hit = true;
  bool is_compute_hit_b = true;
  mjtNum *pos_w; // 命中位置
  mjtNum *pos_b; // 命中位置
  void compute_hit();
  void compute_hit_b();

private:
  mjThreadPool *pool = nullptr;
  struct RayTaskData {
    RayCaster *instance; // 指向你的类实例
    int start;
    int end;
  };
  std::vector<RayTaskData> ray_task_datas;
  static void *ray_task_func(void *user_data) {
    RayTaskData *data = static_cast<RayTaskData *>(user_data);
    // 调用成员函数
    data->instance->compute_ray(data->start, data->end);
    return nullptr;
  }

  void compute_ray(int start, int end);

  void draw_ary(int idx, int width, float *color, mjvScene *scn, bool is_scale);

  mjtNum resolution;
  mjtNum size[2];
  /** @brief 初始化
   * @param m mjModel
   * @param d mjData
   * @param cam_name 相机名称
   * @param resolution 分辨率
   * @param size
   * 相机朝向方向画面的y,x宽度,world则是世界坐标系下按照图像坐标系排列（左上->右下）
   * (M) like isaac sim
   * @param dis_range 距离范围 [最小，最大] (M)
   * @param is_detect_parentbody 是否检测自身
   */
  void init(const mjModel *m, mjData *d, std::string cam_name,
            mjtNum resolution, const std::array<mjtNum, 2> &size,
            const std::array<mjtNum, 2> &dis_range, RayCasterType type,
            bool is_detect_parentbody);

  /*-----------模板-----------*/
  template <typename T>
  void _get_normal_data(T &data, bool is_inf_max, bool is_inv, double scale) {
    if (is_inv) {
      if (is_inf_max) {
        for (int idx = 0; idx < nray; idx++) {
          if (dist[idx] == 0)
            data[idx] = 0;
          else
            data[idx] = (1 - (dist[idx] - deep_min) / deep_min_dif) * scale;
        }
      } else {
        for (int idx = 0; idx < v_ray_num; idx++) {
          if (geomids[idx] < 0)
            data[idx] = 0;
          else if (dist[idx] == 0)
            data[idx] = 0;
          else
            data[idx] = (1 - (dist[idx] - deep_min) / deep_min_dif) * scale;
        }
      }
    } else {
      if (is_inf_max) {
        for (int idx = 0; idx < nray; idx++) {
          if (dist[idx] == 0)
            data[idx] = 0;
          else
            data[idx] = ((dist[idx] - deep_min) / deep_min_dif) * scale;
        }
      } else {
        for (int idx = 0; idx < v_ray_num; idx++) {
          if (geomids[idx] < 0)
            data[idx] = 0;
          else if (dist[idx] == 0)
            data[idx] = 0;
          else
            data[idx] = ((dist[idx] - deep_min) / deep_min_dif) * scale;
        }
      }
    }
  }

  template <typename T>
  void _get_normal_data_from_ratio(T &data, bool is_inf_max, bool is_inv,
                                   double scale) {
    if (is_inv) {
      if (is_inf_max) {
        for (int idx = 0; idx < nray; idx++) {
          data[idx] =
              (1 - (dist_ratio[idx] - deep_min_ratio) / deep_min_ratio_dif) *
              scale;
        }
      } else {
        for (int idx = 0; idx < v_ray_num; idx++) {
          if (geomids[idx] < 0)
            data[idx] = 0;
          else
            data[idx] =
                (1 - (dist_ratio[idx] - deep_min_ratio) / deep_min_ratio_dif) *
                scale;
        }
      }
    } else {
      if (is_inf_max) {
        for (int idx = 0; idx < nray; idx++) {
          data[idx] =
              ((dist_ratio[idx] - deep_min_ratio) / deep_min_ratio_dif) * scale;
        }
      } else {
        for (int idx = 0; idx < v_ray_num; idx++) {
          if (geomids[idx] < 0)
            data[idx] = 0;
          else
            data[idx] =
                ((dist_ratio[idx] - deep_min_ratio) / deep_min_ratio_dif) *
                scale;
        }
      }
    }
  }

  template <typename T> void _get_data_pos_dim1(T &data, const mjtNum *pos) {
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j);
        data[idx * 3] = pos[idx * 3];
        data[idx * 3 + 1] = pos[idx * 3 + 1];
        data[idx * 3 + 2] = pos[idx * 3 + 2];
      }
    }
  }

  template <typename T> void _get_data_pos_dim2(T &data, const mjtNum *pos) {
    for (int i = 0; i < v_ray_num; i++) {
      for (int j = 0; j < h_ray_num; j++) {
        int idx = _get_idx(i, j);
        data[idx][0] = pos[idx * 3];
        data[idx][1] = pos[idx * 3 + 1];
        data[idx][2] = pos[idx * 3 + 2];
      }
    }
  }

public:
  template <typename T>
  void get_normal_data(T &data, bool is_noise, bool is_inf_max, bool is_inv,
                       double scale) {
    if (!is_noise) {
      _get_normal_data_from_ratio(data, is_inf_max, is_inv, scale);
    } else {
      _get_normal_data(data, is_inf_max, is_inv, scale);
    }
  }

  template <typename T>
  void get_data(T &data, bool is_inf_max = true, bool is_noise = false) {
    if (is_noise == has_noise) {
      if (is_inf_max) {
        if (sizeof(T) == sizeof(mjtNum))
          memcpy(data, dist, nray * sizeof(double));
        else {
          for (int i = 0; i < nray; i++) {
            data[i] = dist[i];
          }
        }
      } else {
        for (int i = 0; i < nray; i++) {
          if (geomids[i] < 0)
            data[i] = 0.0;
          else
            data[i] = dist[i];
        }
      }
    } else {
      if (is_inf_max) {
        for (int i = 0; i < nray; i++) {
          data[i] = dist_ratio[i] * deep_max;
        }
      } else {
        for (int i = 0; i < nray; i++) {
          if (geomids[i] < 0)
            data[i] = 0.0;
          else
            data[i] = dist_ratio[i] * deep_max;
        }
      }
    }
  }

  template <typename T> void get_data_pos_w(T &data) {
    _get_data_pos_dim1(data, pos_w);
  }
  template <typename T> void get_data_pos_b(T &data) {
    _get_data_pos_dim1(data, pos_b);
  }
};
