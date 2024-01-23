cmake_minimum_required(VERSION 3.5)
project(magnet_controller)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(ament_cmake_DIR /opt/ros/foxy/share/ament_cmake/cmake)
#set(ament_cmake_core_DIR /opt/ros/foxy/share/ament_cmake_core/cmake)
#set(ament_cmake_libraries_DIR /opt/ros/foxy/share/ament_cmake_libraries/cmake)
#set(JetsonGPIO_DIR /home/itr/colcon_ws/src/JetsonGPIO/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(JetsonGPIO REQUIRED)
find_package(Qt5 COMPONENTS Widgets Gui REQUIRED)

add_executable(controller src/magnet_controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs JetsonGPIO)

add_executable(gui
src/magnet_control_gui.cpp)
ament_target_dependencies(gui rclcpp std_msgs Qt5)

target_link_libraries(gui Qt5::Widgets Qt5::Gui)

target_link_directories(gui
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  controller
  gui
  DESTINATION lib/${PROJECT_NAME})


ament_package()