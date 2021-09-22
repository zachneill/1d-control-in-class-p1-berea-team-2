cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(omnibot_msgs)

find_package(lcm REQUIRED)

include(${LCM_USE_FILE})
include(GenerateExportHeader)

lcm_wrap_types(
  C_EXPORT omnibot_msgs
  C_SOURCES c_sources
  C_HEADERS c_headers
  CPP_HEADERS cpp_headers
  lcmtypes/omni_motor_command_t.lcm
)

lcm_add_library(omnibot_msgs C ${c_sources} ${c_headers})
generate_export_header(omnibot_msgs)
target_include_directories(omnibot_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(omnibot_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

lcm_add_library(omnibot_msgs-cpp CPP ${cpp_headers})
target_include_directories(omnibot_msgs-cpp INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})
