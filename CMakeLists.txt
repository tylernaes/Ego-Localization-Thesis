cmake_minimum_required(VERSION 2.8.3)
project(naes)

find_package(catkin REQUIRED COMPONENTS
  rplidar_ros
  roscpp
  rosserial_python
  std_msgs
  sensor_msgs
  message_generation
  rosbag
)

add_message_files(FILES
  current_position.msg
  wheel_speeds.msg
  stop.msg
  ego_points.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  naes
)

catkin_package(
     INCLUDE_DIRS include
#    LIBRARIES
     CATKIN_DEPENDS rospy roscpp std_msgs message_runtime rplidar_ros sensor_msgs rosserial_python rosbag
#    DEPENDS system_lib
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_executable(zeroing_node src/zeroing.cpp)
target_link_libraries(zeroing_node ${catkin_LIBRARIES})

add_executable(localization_node src/localization.cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES})

add_executable(tracking_node src/tracking.cpp)
target_link_libraries(tracking_node ${catkin_LIBRARIES})

add_executable(drive_track_node src/drive_track.cpp)
target_link_libraries(drive_track_node ${catkin_LIBRARIES})

add_executable(adjustment_node src/adjustment.cpp)
target_link_libraries(adjustment_node ${catkin_LIBRARIES})

add_dependencies(zeroing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drive_track_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(adjustment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS zeroing_node localization_node tracking_node drive_track_node adjustment_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include 
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
ego_points.launch
drive_track.launch
adjustment.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

