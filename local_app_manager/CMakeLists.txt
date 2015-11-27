cmake_minimum_required(VERSION 2.8.3)
project(local_app_manager)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

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
