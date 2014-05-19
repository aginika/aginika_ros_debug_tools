cmake_minimum_required(VERSION 2.8.3)
project(aginika_ros_debug_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rviz
  geometry_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  tf2
  topic_tools
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs pcl_ros sensor_msgs std_msgs tf tf2 topic_tools
  DEPENDS pcl rviz
  LIBRARIES aginika_ros_debug_tools
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

qt4_wrap_cpp(MOC_FILES
  src/debug_button.h
)

## Declare a cpp library
add_library(aginika_ros_debug_tools
  SHARED
  src/aginika_ros_debug_tools.cpp
  src/debug_button.cpp
  ${MOC_FILES}
  )

target_link_libraries(aginika_ros_debug_tools ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(aginika_ros_debug_tools ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})



add_executable(topic_pulse_drive src/topic_pulse_drive.cpp)
target_link_libraries(topic_pulse_drive ${catkin_LIBRARIES})
add_dependencies(topic_pulse_drive ${PROJECT_NAME}_gencpp)

## Mark cpp header files for installation
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(TARGETS aginika_ros_debug_tools
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS)



