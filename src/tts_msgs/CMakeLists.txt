cmake_minimum_required(VERSION 3.5)
project(tts_msgs)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Disable type description generation (builtin_interfaces not available in stub build)
set(ROSIDL_SKIP_TYPE_DESCRIPTION_GENERATION ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/TTS.action
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
