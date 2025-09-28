cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T)

set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)

find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
)
add_executable(sensor_data sensor_data.cpp mujoco_thread.cpp)
target_link_libraries(sensor_data mujoco::mujoco glut GL GLU glfw ${OpenCV_LIBS} 
${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} 
${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${geometry_msgs_LIBRARIES})

