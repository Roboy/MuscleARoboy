cmake_minimum_required(VERSION 2.8.3)
project(musclearoboy)

find_package(catkin REQUIRED COMPONENTS  
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  image_geometry
  camera_info_manager
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( virtual_projection src/virtual_projection.cpp )
target_link_libraries( virtual_projection ${catkin_LIBRARIES} )

add_executable( cloud_publisher src/cloud_publisher.cpp )
target_link_libraries( cloud_publisher ${catkin_LIBRARIES} )

add_executable( image_publisher src/image_publisher.cpp )
target_link_libraries( image_publisher ${catkin_LIBRARIES} )

add_executable( smartphone_hmd src/smartphone_hmd.cpp )
target_link_libraries( smartphone_hmd ${catkin_LIBRARIES} )

add_executable( feature_detection src/feature_detection.cpp )
target_link_libraries( feature_detection ${catkin_LIBRARIES} )