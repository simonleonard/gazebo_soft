cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/hudsonthomas/ws/gazebo_dart/devel/lib/x86_64-linux-gnu/cmake /home/hudsonthomas/ws/gazebo_dart/devel/lib/cmake/bullet )
find_package(gazebo REQUIRED )

include_directories(${GAZEBO_INCLUDE_DIRS} /usr/local/include/bullet)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(custom_main custom_main.cc)
target_link_libraries(custom_main ${GAZEBO_LIBRARIES} pthread)

add_executable(custom_main_vector custom_main_vector.cc)
target_link_libraries(custom_main_vector ${GAZEBO_LIBRARIES} pthread)
