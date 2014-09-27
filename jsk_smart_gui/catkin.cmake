cmake_minimum_required(VERSION 2.8.3)
project(jsk_smart_gui)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  roseus
  jsk_gui_msgs
  jsk_pcl_ros
  jsk_maps
  image_view2
  tf
  euslisp
  sound_play
  topic_tools
  dynamic_reconfigure
  image_transport
  image_geometry
  message_generation
  dynamic_tf_publisher
  posedetection_msgs)

add_service_files(FILES point2screenpoint.srv)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS)

set(COMPILE_FLAGS "-O2 -Wno-write-strings -Wno-comment")
add_definitions(-DLINUX -DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()


include_directories(/usr/include /usr/X11R6/include ${euslisp_SOURCE_DIR}/jskeus/eus/lisp/c include ${euslisp_PREFIX}/jskeus/eus/lisp/c ${catkin_INCLUDE_DIRS})
add_library(eusimage_geometry SHARED src/eusimage_geometry.cpp)
target_link_libraries(eusimage_geometry ${catkin_LIBRARIES})
add_dependencies(eusimage_geometry ${PROJECT_NAME}_gencpp)
set_target_properties(eusimage_geometry PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(eusimage_geometry PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(pointtopixel src/3dtopixel.cpp)
target_link_libraries(pointtopixel ${catkin_LIBRARIES})
add_dependencies(pointtopixel ${PROJECT_NAME}_gencpp)


install(DIRECTORY src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

install(TARGETS eusimage_geometry pointtopixel
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
