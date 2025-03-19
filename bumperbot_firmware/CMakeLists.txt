cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libserial
pkg_check_modules(SERIAL REQUIRED libserial)

ament_python_install_package(${PROJECT_NAME})

add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)

# Properly link libserial
target_include_directories(simple_serial_transmitter PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(simple_serial_transmitter ${SERIAL_LIBRARIES})

# Ensure compiler flags are applied
target_compile_options(simple_serial_transmitter PRIVATE ${SERIAL_CFLAGS_OTHER})
target_link_options(simple_serial_transmitter PRIVATE ${SERIAL_LDFLAGS_OTHER})

add_executable(simple_serial_receiver src/simple_serial_receiver.cpp)
ament_target_dependencies(simple_serial_receiver rclcpp std_msgs)

# Properly link libserial
target_include_directories(simple_serial_receiver PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(simple_serial_receiver ${SERIAL_LIBRARIES})

# Ensure compiler flags are applied
target_compile_options(simple_serial_receiver PRIVATE ${SERIAL_CFLAGS_OTHER})
target_link_options(simple_serial_receiver PRIVATE ${SERIAL_LDFLAGS_OTHER})


install(TARGETS
  simple_serial_transmitter
  simple_serial_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}

)


install(PROGRAMS
${PROJECT_NAME}/simple_serial_transmitter.py
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
