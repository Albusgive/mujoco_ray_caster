#pragma once
#include "RayCaster.h"
#include <cmath>
#include <mujoco/mujoco.h>
#include <string>

class RayCasterCameraCfg {
public:
  const mjModel *m;
  mjData *d;
  std::string cam_name;
  mjtNum focal_length = 24.0;          // 焦距 (cm)
  mjtNum horizontal_aperture = 20.955; // 水平孔径 (cm)
  mjtNum vertical_aperture = 0;
  int h_ray_num = 160;
  int v_ray_num = 90;
  std::array<mjtNum, 2> dis_range;
  bool is_detect_parentbody = false;
};

class RayCasterCamera : public RayCaster {
public:
  RayCasterCamera();
  RayCasterCamera(RayCasterCameraCfg &cfg);
  /** @brief 初始化相机 - 使用焦距和孔径
   * @param m mjModel
   * @param d mjData
   * @param cam_id 相机id
   * @param focal_length 焦距 (cm)
   * @param horizontal_aperture 水平孔径 (cm)
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   */
  RayCasterCamera(const mjModel *m, mjData *d, std::string cam_name,
                  mjtNum focal_length, mjtNum horizontal_aperture,
                  int h_ray_num, int v_ray_num,
                  const std::array<mjtNum, 2> &dis_range,
                  mjtNum vertical_aperture = 0,
                  bool is_detect_parentbody = false);
  ~RayCasterCamera();
  void init(const mjModel *m, mjData *d, std::string cam_name, mjtNum focal_length,
            mjtNum horizontal_aperture, int h_ray_num, int v_ray_num,
            const std::array<mjtNum, 2> &dis_range, mjtNum vertical_aperture,
            bool is_detect_parentbody);

private:
  mjtNum focal_length = 24.0;          // 焦距 (cm)
  mjtNum horizontal_aperture = 20.955; // 水平孔径 (cm)
  mjtNum vertical_aperture = 0;
  mjtNum aspect_ratio = 16.0 / 9.0; // 宽高比
  mjtNum h_pixel_size = 0.0;        // 像素水平尺寸 (cm)
  mjtNum v_pixel_size = 0.0;        // 像素垂直尺寸 (cm)

  // 计算射线向量
  void compute_ray_vec_virtual_plane();
  void create_rays() override { compute_ray_vec_virtual_plane(); };
};
