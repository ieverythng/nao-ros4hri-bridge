cmake_minimum_required(VERSION 3.8)
project(nao_replay_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(naoqi_libqi REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(nao_posture_bridge_node src/nao_posture_bridge_node.cpp)
target_compile_options(nao_posture_bridge_node PRIVATE -std=gnu++17 -Wno-unused-parameter -Wno-vla)
target_include_directories(nao_posture_bridge_node PRIVATE ${naoqi_libqi_INCLUDE_DIRS})
ament_target_dependencies(nao_posture_bridge_node rclcpp std_msgs)
target_link_libraries(
  nao_posture_bridge_node
  ${naoqi_libqi_LIBRARIES}
  boost_thread
  boost_system
  boost_chrono
  pthread
)

install(TARGETS nao_posture_bridge_node DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    nao_replay_motion/replay_motion_skill_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME replay_motion_skill_server_node
)
install(
  PROGRAMS
    nao_replay_motion/head_motion_skill_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME head_motion_skill_server_node
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_nao_replay_motion_unit
    test/test_nao_replay_motion_unit.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
