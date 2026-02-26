cmake_minimum_required(VERSION 3.8)
project(nao_posture_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(naoqi_libqi REQUIRED)

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
    nao_posture_bridge/posture_skill_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME posture_skill_server_node
)

install(
  PROGRAMS
    nao_posture_bridge/say_skill_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME say_skill_server_node
)

install(
  PROGRAMS
    nao_posture_bridge/speech_skill_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speech_skill_server_node
)

ament_package()
