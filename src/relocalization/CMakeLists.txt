cmake_minimum_required(VERSION 3.5)
project(relocalization)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wextra -Wpedantic)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

set(USE_LIVOX 1)
# define use_livox
if(USE_LIVOX)
  add_definitions(-DUSE_LIVOX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3)
find_package(small_gicp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(teaserpp REQUIRED)  # 查找 TEASER++
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(icp_node src/icp_node.cpp)
ament_target_dependencies(icp_node rclcpp std_msgs  geometry_msgs  tf2 tf2_ros tf2_geometry_msgs pcl_ros )
if(USE_LIVOX)
  ament_target_dependencies(icp_node livox_ros_driver2)
endif()

add_executable(transform_publisher src/transform_publisher.cpp)
ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros)


add_executable(sac_ia_gicp
  src/sac_ia_gicp.cpp
)
ament_target_dependencies(sac_ia_gicp  rclcpp  std_msgs  PCL  pcl_ros  Eigen3)


# add small_gicp 
add_executable(small_gicp_node src/small_gicp_node.cpp)

ament_target_dependencies(small_gicp_node
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  pcl_ros 
  Eigen3
)

# small_gicp 是一个通过 find_package 引入的现代 CMake 库，需要用 target_link_libraries 链接
target_link_libraries(small_gicp_node 
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
)

if(USE_LIVOX)
  ament_target_dependencies(small_gicp_node livox_ros_driver2)
endif()

add_executable(teaser_node src/teaser_node.cpp)
# 包含目录
ament_target_dependencies(teaser_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  Eigen3
)
target_link_libraries(teaser_node
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
  ${PCL_LIBRARIES}
)
if(USE_LIVOX)
  ament_target_dependencies(teaser_node livox_ros_driver2)
endif()

# === TEASER + GICP 结合节点 ===
add_executable(teaser_gicp_node src/teaser_gicp_node.cpp)
ament_target_dependencies(teaser_gicp_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  Eigen3
)
target_link_libraries(teaser_gicp_node
  teaserpp::teaser_registration
  teaserpp::teaser_features
  teaserpp::teaser_io
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES}
)
if(USE_LIVOX)
  ament_target_dependencies(teaser_gicp_node livox_ros_driver2)
endif()



# Install launch files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS icp_node transform_publisher sac_ia_gicp small_gicp_node # <--- 添加这个
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS teaser_node teaser_gicp_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
