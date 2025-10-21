cmake_minimum_required(VERSION 3.20)
project(MUJOCO_T)

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)

include_directories(
        ${rclcpp_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
)

add_executable(sensor_data sensor_data.cpp mujoco_thread.cpp)
target_link_libraries(sensor_data mujoco::mujoco glut GL GLU glfw 
${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} 
${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${geometry_msgs_LIBRARIES})

