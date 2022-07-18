cmake_minimum_required(VERSION 3.5)
project(zendar_ros_driver)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. Install OpenCV from the command line using the command(s) --\
      sudo apt-get install libopencv-dev")
endif(NOT OpenCV_FOUND)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${IMAGE_TRANSPORT_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cc
  src/ego_vehicle.cc
  src/range_markers.cc
  src/zendar_driver_node.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  image_transport
)

target_link_libraries(${PROJECT_NAME}
  ${zendar_api_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${PCL_LIBRARIES}
  unwind
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
