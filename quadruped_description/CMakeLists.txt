cmake_minimum_required(VERSION 3.8)
project(quadruped_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_service REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    scripts
    launch
    config
    meshes
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  PROGRAMS 
  scripts/test.py
  scripts/static_tf.py
  DESTINATION lib/${PROJECT_NAME}

)
add_executable(traj_bezier scripts/traj_bezier.cpp) 
ament_target_dependencies(traj_bezier rclcpp std_msgs custom_service trajectory_msgs builtin_interfaces geometry_msgs visualization_msgs)


install(TARGETS
traj_bezier
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
