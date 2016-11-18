cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories ("~/ws/gazebo_dart/src/gazebo/gazebo/physics/bullet")
include_directories ("~/ws/gazebo_dart/src/bullet3/src")
include_directories ("~/ws/gazebo_dart/src/bullet3/src/BulletSoftBody")


add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(visual_plugin SHARED visual_plugin.cc)
target_link_libraries(visual_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(factory SHARED factory.cc)
target_link_libraries(factory  ${GAZEBO_LIBRARIES})

