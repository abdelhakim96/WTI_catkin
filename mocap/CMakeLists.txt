cmake_minimum_required(VERSION 2.8.3)
project(mocap)
set(PROJECT_NAME mocap)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
# list here the project dependencies, 3rd party packages such as rosbag or std_msgs
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    rospy
    roslib
    std_msgs
    gazebo_msgs
    tf
)

# find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
)

set(PROJECT_LIBS
    ${catkin_LIBRARIES}
    Eigen3::Eigen
)

###########
## Build ##
###########

add_definitions(${EIGEN_DEFINITIONS})
include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

############
# BINARIES #
############

# Node executables
add_executable(mocap_bridge_gazebo src/mocap_bridge_gazebo.cpp)

# Node linking
target_link_libraries(mocap_bridge_gazebo ${PROJECT_LIBS})

