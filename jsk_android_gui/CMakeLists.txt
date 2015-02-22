cmake_minimum_required(VERSION 2.8.3)
project(jsk_android_gui)

find_package(catkin REQUIRED COMPONENTS
  jsk_gui_msgs
)

catkin_package(
  CATKIN_DEPENDS jsk_gui_msgs
)

install(FILES
  jsk_android_gui.app
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
