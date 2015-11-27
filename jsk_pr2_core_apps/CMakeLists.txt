cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_core_apps)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS
  scripts/qrcode.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  local_app_manager
  task_bridge_apps
  apps
  calib_image
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  robot_place_info.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
