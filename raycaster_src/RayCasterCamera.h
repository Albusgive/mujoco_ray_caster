#pragma once
#include "RayCaster.h"
#include <cmath>
#include <mujoco/mujoco.h>
#include <string>

class RayCasterCameraCfg {
public:
  const mjModel *m = nullptr;
  mjData *d = nullptr;
  std::string cam_name = "";
  mjtNum focal_length = 24.0;          // 焦距 (cm)
  mjtNum horizontal_aperture = 20.955; // 水平孔径 (cm)
  mjtNum vertical_aperture = 0;
  int h_ray_num = 160;
  int v_ray_num = 90;
  std::array<mjtNum, 2> dis_range = {0.0, 100.0};
  bool is_detect_parentbody = false;
  mjtNum baseline = 0.0; // 基线距离 (cm)
  mjtNum loss_angle=0.0;
};

class RayCasterCamera : public RayCaster {
public:
  RayCasterCamera();
  RayCasterCamera(const RayCasterCameraCfg &cfg);
  ~RayCasterCamera();
  void init(const RayCasterCameraCfg &cfg);

private:
  mjtNum focal_length = 24.0;          // 焦距 (cm)
  mjtNum horizontal_aperture = 20.955; // 水平孔径 (cm)
  mjtNum vertical_aperture = 0;
  mjtNum aspect_ratio = 16.0 / 9.0;     // 宽高比
  mjtNum h_pixel_size = 0.0;            // 像素水平尺寸 (cm)
  mjtNum v_pixel_size = 0.0;            // 像素垂直尺寸 (cm)
  mjtNum baseline = 0.0;                // 基线距离 (cm)
  mjtNum left_pos_w[3], right_pos_w[3]; // Stereo相机位置

#if mjVERSION_HEADER >= 341
  mjtNum *left_ray_normal = nullptr;
  mjtNum *right_ray_normal = nullptr;
  void compute_loss_ray(int idx) override {}
#endif

  // 计算射线向量
  void compute_ray_vec_virtual_plane();
  void create_rays() override { compute_ray_vec_virtual_plane(); };

  struct StereoTaskData {
    RayCasterCamera *instance; // 指向你的类实例
    int start;
    int end;
  };
  std::vector<StereoTaskData> stereo_task_datas;
  static void *stereo_task_func(void *user_data) {
    StereoTaskData *data = static_cast<StereoTaskData *>(user_data);
    data->instance->compute_stereo_ray(data->start, data->end);
    return nullptr;
  }

public:
  void set_num_thread(int n) override;
  void compute_stereo_ray(int start, int end);
  void compute_distance() override;
};
