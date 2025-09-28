#include "mujoco_thread.h"
#include "tf2/transform_datatypes.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mujoco/mjtnum.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class MJ_ENV : public mujoco_thread, public rclcpp::Node {
public:
  MJ_ENV(std::string model_file, std::string node_name, double max_FPS = 60)
      : rclcpp::Node(node_name) {
    load_model(model_file);
    set_window_size(1200, 900);
    set_window_title("MUJOCO");
    set_max_FPS(max_FPS);
    sub_step = 10;

    int sensor_id = mj_name2id(m, mjOBJ_SENSOR, "raycasterlidar");
    if (sensor_id == -1) {
      std::cout << "no found sensor" << std::endl;
      return;
    }
    data_pos = m->sensor_adr[sensor_id];

    sensor_id = mj_name2id(m, mjOBJ_SENSOR, "camera_quat");
    if (sensor_id == -1) {
      std::cout << "no found sensor" << std::endl;
      return;
    }
    camera_quat_sensor_d_point = m->sensor_adr[sensor_id];

    sensor_id = mj_name2id(m, mjOBJ_SENSOR, "camera_pos");
    if (sensor_id == -1) {
      std::cout << "no found sensor" << std::endl;
      return;
    }
    camera_pos_sensor_d_point = m->sensor_adr[sensor_id];

    auto [h_ray_num, v_ray_num, data_pairs] =
        get_ray_caster_info(m, d, "raycasterlidar");
    std::cout << "h_ray_num: " << h_ray_num << ", v_ray_num: " << v_ray_num
              << std::endl;
    std::cout << "data_ps: ";
    for (const auto &pair : data_pairs) {
      std::cout << "(" << pair.first << ", " << pair.second << "), ";
    }
    std::cout << std::endl;
    pos_w_data_point = data_pairs[0].first;
    pos_w_data_size = data_pairs[0].second;
    pos_b_data_point = data_pairs[1].first;
    pos_b_data_size = data_pairs[1].second;

    pos_w_pointcloud_publisher =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mujoco/pos_w_pointcloud", 1);
    pos_b_pointcloud_publisher =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mujoco/pos_b_pointcloud", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  };
  void step() override {
    pointcloud_pub(d->sensordata + data_pos + pos_w_data_point, pos_w_data_size,
                   pos_w_pointcloud_publisher);
    // pointcloud_pub(d->sensordata + data_pos + pos_b_data_point,
    // pos_b_data_size,
    //                pos_b_pointcloud_publisher, camera_frame_id);
    pub_frame();
  };
  int data_pos = 0;
  int pos_w_data_point = 0;
  int pos_w_data_size = 0;
  int pos_b_data_point = 0;
  int pos_b_data_size = 0;
  int camera_quat_sensor_d_point = 0;
  int camera_pos_sensor_d_point = 0;
  std::string camera_frame_id = "camera";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pos_w_pointcloud_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pos_b_pointcloud_publisher;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void pointcloud_pub(
      mjtNum *data, int data_size,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
      std::string frame_id = "map") {
    int len = data_size / 3;
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id; // 根据你的坐标系修改
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.height = 1;
    msg.width = len;     // 点的数量
    msg.point_step = 12; // 每个点的大小（3个float * 4字节）
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;
    msg.data.resize(msg.row_step);
    float *data_ptr = reinterpret_cast<float *>(msg.data.data());
    for (int i = 0; i < len; i++) {
      data_ptr[0] = static_cast<float>(data[i * 3 + 0]); // x
      data_ptr[1] = static_cast<float>(data[i * 3 + 1]); // y
      data_ptr[2] = static_cast<float>(data[i * 3 + 2]); // z
      data_ptr += 3;
    }
    publisher->publish(msg);
  }
  void pub_frame() {
    rclcpp::Time now;
    geometry_msgs::msg::TransformStamped t;
    mjtNum *pos = d->sensordata + camera_pos_sensor_d_point;
    mjtNum *quat = d->sensordata + camera_quat_sensor_d_point;
    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = camera_frame_id;

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = pos[0];
    t.transform.translation.y = pos[1];
    t.transform.translation.z = pos[2];

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    t.transform.rotation.w = quat[0];
    t.transform.rotation.x = quat[1];
    t.transform.rotation.y = quat[2];
    t.transform.rotation.z = quat[3];

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  std::tuple<int, int, std::vector<std::pair<int, int>>>
  get_ray_caster_info(const mjModel *model, mjData *d,
                      const std::string &sensor_name) {
    std::vector<std::pair<int, int>> data_ps;
    int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id == -1) {
      std::cout << "no found sensor" << std::endl;
      return std::make_tuple(0, 0, data_ps);
    }
    int sensor_plugin_id = m->sensor_plugin[sensor_id];
    int state_idx = m->plugin_stateadr[sensor_plugin_id];

    for (int i = state_idx + 2;
         i < state_idx + m->plugin_statenum[sensor_plugin_id]; i += 2) {
      data_ps.emplace_back(d->plugin_state[i], d->plugin_state[i + 1]);
    }
    int h_ray_num = d->plugin_state[state_idx + 0];
    int v_ray_num = d->plugin_state[state_idx + 1];
    return std::make_tuple(h_ray_num, v_ray_num, data_ps);
  }
};

int main(int argc, const char **argv) {
  mj_loadAllPluginLibraries(
      "../../../../lib", +[](const char *filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MJ_ENV>("../../../../model/ray_caster2.xml",
                                       "mujoco_ray_caster", 60);
  node->connect_windows_sim();
  node->render();
  node->sim2thread();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
