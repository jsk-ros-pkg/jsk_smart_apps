cmake_minimum_required(VERSION 2.8.3)
project(jsk_smart_gui)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_geometry
  message_generation
  roseus)

add_service_files(FILES point2screenpoint.srv)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS roseus)

add_definitions(-O2 -Wno-write-strings -Wno-comment -falign-functions=16 -DLINUX -DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()


find_package(euslisp REQUIRED)
if(NOT euslisp_INCLUDE_DIRS)
  if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
    set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
  else()
    set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
  endif()
  message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
  set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
endif()
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")
include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
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
