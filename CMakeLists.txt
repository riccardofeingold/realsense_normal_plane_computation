cmake_minimum_required(VERSION 3.8)
project(magnecko_realsense_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  realsense2
  PCL
  pcl_conversions
  Eigen3
  OpenCV
  cv_bridge
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0. For older
# cv_bridge versions, we have to use the header 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

add_executable(magnecko_realsense_node src/magnecko_realsense_node.cpp)

target_compile_features(magnecko_realsense_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(magnecko_realsense_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  magnecko_realsense_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(
  magnecko_realsense_node PUBLIC 
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  /usr/lib/x86_64-linux-gnu/libtiff.so.5
)

install(TARGETS magnecko_realsense_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

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
