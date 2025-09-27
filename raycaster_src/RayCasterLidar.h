#pragma once
#include "RayCaster.h"
#include <cmath>
#include <mujoco/mujoco.h>
#include <string>

class RayCasterLidarCfg {
public:
  const mjModel *m;
  mjData *d;
  std::string cam_name;
  mjtNum fov_h;
  mjtNum fov_v;
  int h_ray_num = 160;
  int v_ray_num = 90;
  std::array<mjtNum, 2> dis_range;
  bool is_detect_self = false;
};

class RayCasterLidar : public RayCaster {
public:
  RayCasterLidar();
  RayCasterLidar(RayCasterLidarCfg &cfg);
  /** @brief 初始化相机 - 使用焦距和孔径
   * @param m mjModel
   * @param d mjData
   * @param cam_id 相机id
   * @param fovh 焦距 (cm)
   * @param fov_v 水平孔径 (cm)
   * @param h_ray_num 水平射线数量
   * @param v_ray_num 垂直射线数量
   * @param dis_range 距离范围 [最小，最大] (M)
   */
  RayCasterLidar(const mjModel *m, mjData *d, std::string cam_name, mjtNum fov_h,
                 mjtNum fov_v, int h_ray_num, int v_ray_num,
                 const std::array<mjtNum, 2> &dis_range,
                 bool is_detect_self = false);
  ~RayCasterLidar();
  void init(const mjModel *m, mjData *d, std::string cam_name, mjtNum fov_h,
            mjtNum fov_v, int h_ray_num, int v_ray_num,
            const std::array<mjtNum, 2> &dis_range,
            bool is_detect_self = false);

private:
  mjtNum fov_h = 50.0;
  mjtNum fov_v = 50.0;
  mjtNum h_res = 0.0; // 水平分辨率
  mjtNum v_res = 0.0; // 垂直分辨率

  void create_rays() override;
};
