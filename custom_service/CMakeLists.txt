cmake_minimum_required(VERSION 3.8)
project(custom_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PassingSw.srv"
  "srv/IdSw.srv"
  "srv/IkSw.srv"
  DEPENDENCIES std_msgs
)

# Installation steps
install(
  DIRECTORY
    srv
  DESTINATION
    share/${PROJECT_NAME}/
)


ament_package()
