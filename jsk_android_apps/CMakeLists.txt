cmake_minimum_required(VERSION 2.8.3)
project(jsk_android_apps)

find_package(catkin REQUIRED rosjava_build_tools)

if(EXISTS $ENV{ANDROID_HOME})
  catkin_android_setup(assembleRelease uploadArchives)
endif()

catkin_package()

##############################################################################
# Installation
##############################################################################

if(EXISTS $ENV{ANDROID_HOME})
  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/jsk_android_apps/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/jsk_android_apps)
endif()