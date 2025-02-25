cmake_minimum_required(VERSION 3.0.2)
project(LiDAR)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
   sensor_msgs
   cv_bridge
   image_transport
  #  Total_msgs
)

find_package( OpenCV REQUIRED )

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  coeff_arr.msg
  coeff.msg
  fusInfo.msg
  ground_msg_arr.msg
  ground_msg.msg
  lane_arr.msg
  lane.msg
  object_msg_arr.msg
  object_msg.msg
  objInfo.msg
  objsInfo.msg
  RANSAC_points_arr.msg
  RANSAC_points.msg
  RANSAC.msg
  raw4cam_arr.msg
  raw4cam.msg
  tunneloutpoint_arr.msg
  tunneloutpoint.msg
  totalInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  # Total_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation roscpp std_msgs message_runtime
  DEPENDS Eigen
  # Total_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(filter_div src/Lidar_code/Lidar_filter.cpp)
target_link_libraries(filter_div ${catkin_LIBRARIES})
#add_dependencies(filter_div ${catkin_EXPORTED_TARGETS})

add_library(etc_div src/Lidar_code/Lidar_etc.cpp)
target_link_libraries(etc_div ${catkin_LIBRARIES})
#add_dependencies(etc_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(msg_div src/Lidar_code/Lidar_msg.cpp)
target_link_libraries(msg_div ${catkin_LIBRARIES})
#add_dependencies(msg_div ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(Lidar_code src/Lidar_code/Lidar_code.cpp)
target_link_libraries(Lidar_code ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pcl_visualization filter_div etc_div msg_div)
#add_dependencies(Lidar_code ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_library(decalre_div src/extra_code/Lidar_declare.h)
#target_link_libraries(decalre_div ${catkin_LIBRARIES})

#add_library(param_div src/extra_code/set_param.h)
#target_link_libraries(param_div ${catkin_LIBRARIES})

#
add_executable(line_visualizer src/node_code/line_visualizer.cpp)
target_link_libraries(line_visualizer Lidar_code ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#
add_executable(Lidar_UDP src/node_code/Lidar_UDP_communication.cpp)
target_link_libraries(Lidar_UDP Lidar_code ${catkin_LIBRARIES} pthread)

add_executable(Lidar_GE src/node_code/Lidar_GroundExtraction.cpp)
target_link_libraries(Lidar_GE Lidar_code)

add_executable(Lidar_Lane src/node_code/Lidar_Lane.cpp)
target_link_libraries(Lidar_Lane Lidar_code)

add_executable(Lidar_ROI src/node_code/Lidar_input_ROI.cpp)
target_link_libraries(Lidar_ROI Lidar_code)

add_executable(Lidar_ransac src/node_code/Lidar_ransac.cpp)
target_link_libraries(Lidar_ransac Lidar_code)

add_executable(Lidar_Clustering src/node_code/Lidar_Clustering.cpp)
target_link_libraries(Lidar_Clustering Lidar_code)

# add_executable(Lidar_Downsampling src/node_code/Lidar_Downsampling.cpp)
# target_link_libraries(Lidar_Downsampling Lidar_code)

add_executable(signal_test src/node_code/signal_test.cpp)
target_link_libraries(signal_test Lidar_code)

# install(TARGETS Lidar_visual_2D
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


