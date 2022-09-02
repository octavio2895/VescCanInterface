cmake_minimum_required(VERSION 2.8.3)

project (vesc_can_interface)
add_compile_options(-std=c++11)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")
# set (GLOB source_files "${source_dir}/*.cpp")

# include_directories(
#   include
#   ${Boost_INCLUDE_DIRS}
# #   ${catkin_INCLUDE_DIRS}
# )

find_package( Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

add_library(ProjectLibs STATIC src/vesc_can_interface.cpp src/crc.cpp)
add_library(vesc_can_interface STATIC src/vesc_can_interface.cpp src/crc.cpp)

#add_library(crc STATIC src/crc.c)

set(SOURCES src/vesc_can_test.cpp
  src/crc.c
  src/vesc_can_interface.cpp
  )

# node executable
add_executable(vesc_can_test src/vesc_can_test.cpp)

# add_dependencies(vesc_can_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_can_test PUBLIC ProjectLibs Threads::Threads)
target_link_libraries(vesc_can_interface PUBLIC vesc_can_interface Threads::Threads)
#target_link_libraries(vesc_can_test vesc_can_interface crc)

