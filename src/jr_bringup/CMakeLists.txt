cmake_minimum_required(VERSION 3.8)
project(jr_bringup)

find_package(ament_cmake REQUIRED)

# launch-only package: install our launch (and config when we add it)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
