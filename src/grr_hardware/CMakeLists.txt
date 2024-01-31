cmake_minimum_required(VERSION 3.8)
project(grr_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  /robotics/bcm2835-1.73/
  ${colcon_INCLUDE_DIRS}
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(robot src/robot.cpp)

ament_target_dependencies(robot rclcpp std_msgs sensor_msgs)

target_link_libraries(robot -lbcm2835)

add_library(
  ${PROJECT_NAME} SHARED
  #rc/I2C.cpp
  src/backcolorsensor.cpp
)

install(TARGETS
  robot
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
