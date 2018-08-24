cmake_minimum_required(VERSION 2.8.3)
project(astar)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  #rospy
  #std_msgs
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astar_planner.cpp   
   )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/astar_planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/constants.h
   )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES astar
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
