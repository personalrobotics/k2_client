cmake_minimum_required(VERSION 2.8.3)
project(k2_client)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

find_package(catkin REQUIRED
            COMPONENTS roscpp rospy std_msgs cv_bridge image_transport camera_info_manager message_generation)
find_package(Jsoncpp REQUIRED)

add_message_files(
  FILES
  Activities.msg
  Appearance.msg
  Audio.msg
  Body.msg
  BodyArray.msg
  Expressions.msg
  JointOrientationAndType.msg
  JointPositionAndState.msg
  Lean.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport camera_info_manager message_runtime
  DEPENDS Jsoncpp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIR}
)

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)
file(GLOB_RECURSE SOURCES src/*.cpp)

include_directories(${includeFolders})

foreach(ITEM ${SOURCES})
    get_filename_component(filename ${ITEM} NAME_WE)
    add_executable(${filename} ${HEADERS} ${ITEM})
    target_link_libraries(${filename} ${catkin_LIBRARIES})
    target_link_libraries(${filename} ${JSONCPP_LIBRARY})
    add_dependencies(${filename} ${PROJECT_NAME}_gencpp)
endforeach()
