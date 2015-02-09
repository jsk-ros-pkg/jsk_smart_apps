cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_core_apps)

find_package(catkin REQUIRED COMPONENTS
  detect_cans_in_fridge_201202
  jsk_android_gui_api9
  jsk_pr2_startup
  roseus
  roseus_tutorials
  sound_play
  topic_tools
)

catkin_package(
  CATKIN_DEPENDS detect_cans_in_fridge_201202 jsk_android_gui_api9 jsk_pr2_startup roseus roseus_tutorials sound_play topic_tools
)

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
