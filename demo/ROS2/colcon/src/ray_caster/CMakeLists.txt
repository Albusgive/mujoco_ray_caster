cmake_minimum_required(VERSION 3.8)
project(ray_caster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
rclcpp
std_msgs
sensor_msgs
tf2
tf2_ros
geometry_msgs
)

# Find MuJoCo if available
set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)


file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(sensor_data ${SRC})
ament_target_dependencies(sensor_data
  ${dependencies}
)
target_include_directories(sensor_data PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/ray_caster)
target_link_libraries(sensor_data mujoco::mujoco glut GL GLU glfw)
get_target_property(MUJOCO_LIB_PATH mujoco::mujoco LOCATION)
get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB_PATH}" DIRECTORY)

set_target_properties(sensor_data PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
)

install(TARGETS sensor_data
  DESTINATION lib/${PROJECT_NAME}
)

set(plugin_path "${CMAKE_SOURCE_DIR}/../../../../../")
add_definitions(-DMJ_PLUGIN_PATH="${plugin_path}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
