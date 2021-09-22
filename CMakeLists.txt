cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(wall_follower)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lpthread")

add_subdirectory(omnibot_msgs)

find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

# 1D control executable.
add_executable(1D_control src/1D_control.cpp
  src/common/drive.cpp
  src/common/lidar.cpp
  src/common/utils.cpp
)
target_link_libraries(1D_control
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librplidar_sdk.a
  lcm
  omnibot_msgs-cpp
)
target_include_directories(1D_control PRIVATE
  include
)

# 2D control nearest executable.
add_executable(2D_control_nearest src/2D_control_nearest.cpp
  src/common/drive.cpp
  src/common/lidar.cpp
  src/common/utils.cpp
)
target_link_libraries(2D_control_nearest
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/librplidar_sdk.a
  lcm
  omnibot_msgs-cpp
)
target_include_directories(2D_control_nearest PRIVATE
  include
)

if (CMAKE_BUILD_TYPE MATCHES Test)
endif()
