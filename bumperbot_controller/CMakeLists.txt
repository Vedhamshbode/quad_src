cmake_minimum_required(VERSION 3.8)
project(bumperbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) #add for python
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)     
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libserial
pkg_check_modules(SERIAL REQUIRED libserial)
add_library(${PROJECT_NAME} SHARED
  src/bumperbot_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SERIAL_LIBRARIES})

# Ensure compiler flags are applied
target_compile_options(${PROJECT_NAME} PRIVATE ${SERIAL_CFLAGS_OTHER})
target_link_options(${PROJECT_NAME} PRIVATE ${SERIAL_LDFLAGS_OTHER})
ament_target_dependencies(${PROJECT_NAME}
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface bumperbot_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_python_install_package(${PROJECT_NAME})  #for declaring python scripts

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
# when you add new folder write here to install!!!
add_executable(controller_cpp src/simple_controller.cpp)
ament_target_dependencies(controller_cpp rclcpp geometry_msgs std_msgs ${Eigen_LIBRARIES} sensor_msgs nav_msgs tf2 tf2_ros)

install(
  DIRECTORY include/  
  DESTINATION include
)

install(
  TARGETS
  controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch   
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  ${PROJECT_NAME}/noisy_controller.py
  ${PROJECT_NAME}/waypoint_navigation.py

  DESTINATION lib/${PROJECT_NAME}

)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
