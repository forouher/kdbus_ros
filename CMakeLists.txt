cmake_minimum_required(VERSION 2.8.3)
project(kdbus_tests)

#add_definitions(-std=c1)
#SET(CMAKE_CXX_FLAGS "-std=std99")
SET(CMAKE_C_FLAGS "-std=gnu99")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  roscpp std_msgs tf2_msgs)

## System dependencies are found with CMake's conventions
#SET(CMAKE_INCLUDE_PATH "/home/forouher/src/boost_1_55_0")
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs tf2_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(talko src/talker.cpp)
add_executable(alloc_test src/alloc_test.cpp)
add_executable(alloc_test2 src/alloc_test2.cpp)
add_executable(alloc_test3 src/alloc_test3.cpp)
add_executable(test_writer src/kdbus-enum.c src/kdbus-util.c src/test_writer.cpp)
add_executable(test_reader src/kdbus-enum.c src/kdbus-util.c src/test_reader.cpp)

target_link_libraries(alloc_test3 rt)
target_link_libraries(talko ${catkin_LIBRARIES})
target_link_libraries(test_reader ${catkin_LIBRARIES})
target_link_libraries(test_writer ${catkin_LIBRARIES})

