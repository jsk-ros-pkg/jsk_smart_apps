cmake_minimum_required(VERSION 2.8.3)
project(local_app_manager)

find_package(catkin REQUIRED COMPONENTS
  pr2_app_manager
  jsk_2013_04_pr2_610
  jsk_pr2_core_apps
)

catkin_package(
  CATKIN_DEPENDS pr2_app_manager jsk_2013_04_pr2_610 jsk_pr2_core_apps
)

install(PROGRAMS
  scripts/qrcode.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  app_manager.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
  

install(DIRECTORY config mock_apps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
