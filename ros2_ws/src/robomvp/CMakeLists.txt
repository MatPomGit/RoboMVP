cmake_minimum_required(VERSION 3.8)
project(robomvp)

# Kompilator z ostrzeżeniami
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Wymagane pakiety ament
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generowanie niestandardowych typów wiadomości
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetection.msg"
  "msg/MarkerPose.msg"
  "msg/Offset.msg"
  "msg/State.msg"
)

# Instalacja pakietu Python
ament_python_install_package(${PROJECT_NAME})

# Instalacja skryptów wykonywalnych węzłów
install(PROGRAMS
  robomvp/camera_interface.py
  robomvp/marker_detection.py
  robomvp/marker_pose_estimator.py
  robomvp/main_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalacja plików uruchomieniowych
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testy ament
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
