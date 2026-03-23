cmake_minimum_required(VERSION 3.8)
project(robomvp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generowanie typów wiadomości, serwisów i akcji
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetection.msg"
  "msg/MarkerPose.msg"
  "msg/Offset.msg"
  "msg/State.msg"
  "action/ManipulationTask.action"
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  robomvp/camera_interface.py
  robomvp/marker_detection.py
  robomvp/marker_pose_estimator.py
  robomvp/main_node.py
  robomvp/robomvp_diagnostics.py
  robomvp/robomvp_odometry.py
  robomvp/robomvp_tf_publisher.py
  robomvp/robomvp_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
