cmake_minimum_required(VERSION 2.8.3)
project(jsk_smart_apps)

find_package(catkin REQUIRED rosjava_build_tools)
# Set the gradle targets you want catkin's make to run by default
catkin_android_setup(assembleRelease uploadArchives)
catkin_package()


