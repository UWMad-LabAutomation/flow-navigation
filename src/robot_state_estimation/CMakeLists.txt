# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.
cmake_minimum_required(VERSION 3.16)
project(robot_state_estimation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
