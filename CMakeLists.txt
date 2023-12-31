cmake_minimum_required(VERSION 3.8)
project(pcdmap2pgm)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(backward_ros REQUIRED) 
ament_auto_find_build_dependencies()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_add_executable(convert_pcd2pgm src/pcd2pgm.cpp)

ament_target_dependencies(convert_pcd2pgm
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        pcl_conversions
        pcl_msgs
        )

target_link_libraries(convert_pcd2pgm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_backward(convert_pcd2pgm)
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

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  src
)
