set(MUJOCO_SENSOR_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

set(MUJOCO_SENSOR_SRCS
    register.cc
    ray_plugin.h
    ray_plugin.cc
    ray_caster_plugin.h
    ray_caster_plugin.cc
    ray_caster_camera_plugin.h
    ray_caster_camera_plugin.cc
    ray_caster_lidar_plugin.h
    ray_caster_lidar_plugin.cc
)
file(GLOB RAY_CASTER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/raycaster_src/*)

add_library(sensor_raycaster SHARED)
target_sources(sensor_raycaster PRIVATE ${MUJOCO_SENSOR_SRCS} ${RAY_CASTER_SRC})
target_include_directories(sensor_raycaster PRIVATE ${MUJOCO_SENSOR_INCLUDE})
target_link_libraries(sensor_raycaster PRIVATE mujoco)
target_compile_options(
  sensor_raycaster
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  sensor_raycaster
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)


add_custom_command(TARGET sensor_raycaster POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:sensor_raycaster>"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/"
    COMMENT "Copying sensor_raycaster library to source directory..."
)