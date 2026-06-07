cmake_minimum_required(VERSION 3.8)
project(ur3_mtc_pick_place)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ur3_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  srdf
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
