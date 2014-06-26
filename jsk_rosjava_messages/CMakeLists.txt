##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(jsk_rosjava_messages)

if($ENV{ROS_DISTRO} STREQUAL "groovy")
  message("jsk_rosjava_message is not supported on rosjava")
  return()
endif($ENV{ROS_DISTRO} STREQUAL "groovy")

find_package(rosjava_build_tools QUIET)
if(NOT rosjava_build_tools_FOUND)
  message("rosjava_build_tools is not found")
  return()
endif()
##############################################################################
# Catkin
##############################################################################

set(ENV{ROS_MAVEN_DEPLOYMENT_REPOSITORY} ${CATKIN_DEVEL_PREFIX}/share/maven)

find_package(catkin REQUIRED rosjava_build_tools)
# Set the gradle targets you want catkin's make to run by default
catkin_rosjava_setup(publishMavenJavaPublicationToMavenRepository)
catkin_package()

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
# install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/${PROJECT_NAME}/ 
#         DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/${PROJECT_NAME})
