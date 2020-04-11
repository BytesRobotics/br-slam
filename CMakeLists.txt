cmake_minimum_required(VERSION 3.5)
project(br-slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
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
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(br-slam src/br-slam.cpp bytes-orb-slam/src/System.cpp bytes-orb-slam/src/ORBExtractor.cpp bytes-orb-slam/src/Frame.cpp bytes-orb-slam/src/Tracking.cpp)

target_include_directories(br-slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/br-slam>
  $<INSTALL_INTERFACE:include/br-slam>)

target_include_directories(br-slam PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bytes-orb-slam/include>
        $<INSTALL_INTERFACE:include/bytes-orb-slam>)

ament_target_dependencies(
        br-slam
        rclcpp
        message_filters
        sensor_msgs
        nav_msgs
        cv_bridge
        OpenCV
        Eigen3
        image_geometry
        tf2
        tf2_ros
)

install(TARGETS br-slam
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
