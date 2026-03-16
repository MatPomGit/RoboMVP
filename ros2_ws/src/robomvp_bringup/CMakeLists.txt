cmake_minimum_required(VERSION 3.8)
project(robomvp_bringup)

# Wymagane pakiety ament
find_package(ament_cmake REQUIRED)

# Instalacja plików uruchomieniowych
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testy ament
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
