# Copyright (C) 2024 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(event_camera_example)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(dv-processing REQUIRED)

################################################################################
# example
################################################################################
set(lib_name example)
add_library(${lib_name} SHARED
  src/example.cpp
)
target_link_libraries(${lib_name}
  dv::processing
)
ament_target_dependencies(${lib_name}
  rclcpp
  event_camera_msgs
  event_camera_codecs
  dv_ros_msgs
)
install(
  TARGETS ${lib_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# example node
################################################################################
set(exec_name example_node)
add_executable(${exec_name}
  src/example_node.cpp
)
target_link_libraries(${exec_name} example)
ament_target_dependencies(${exec_name}
  rclcpp
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
)

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


