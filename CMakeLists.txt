cmake_minimum_required(VERSION 2.8.3)

project(qbot_4wheeled_robot)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# RQT setup:
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_cpp
  geometry_msgs
)
find_package(Qt5Widgets REQUIRED)

set(qbot_4wheeled_robot_SRCS
  src/gui_plugin.cpp
)
set(qbot_4wheeled_robot_HDRS
  src/gui_plugin.h
)
set(qbot_4wheeled_robot_UIS
  resources/gui_plugin.ui
)

set(qbot_4wheeled_robot_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  )

catkin_package(
  INCLUDE_DIRS ${qbot_4wheeled_robot_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp roscpp geometry_msgs
)

catkin_python_setup()

qt5_wrap_cpp(qbot_4wheeled_robot_MOCS ${qbot_4wheeled_robot_HDRS})
qt5_wrap_ui(qbot_4wheeled_robot_UIS_H ${qbot_4wheeled_robot_UIS})

include_directories(${qbot_4wheeled_robot_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${qbot_4wheeled_robot_SRCS} ${qbot_4wheeled_robot_MOCS} ${qbot_4wheeled_robot_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
