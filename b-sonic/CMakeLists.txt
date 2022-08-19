cmake_minimum_required(VERSION 3.0.2)
project(b-sonic)

# set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin 
  COMPONENTS
    roscpp
    common_msgs
    pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    common_msgs
    pcl_ros

  DEPENDS
    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (${PROJECT_NAME}_node 
  src/bsonic.cpp
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)
