cmake_minimum_required(VERSION 2.8.3)
project(map_merger)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(TEST_NAME ${PROJECT_NAME}_UnitTest)

include_directories(/usr/local/lib/rtabmap-0.19)
find_package(catkin REQUIRED COMPONENTS
  #relative_nav
  rtabmap_ros
  #rosflight_utils
  roscpp
  rospy
  tf
  cmake_modules
  #pcl_ros
  #pcl_conversions
  #eigen_conversions
)
#find_package(cnpy REQUIRED)


catkin_package(INCLUDE_DIRS      include
               DEPENDS           roscpp
                                 rospy
                                 tf
                                 rtabmap_ros)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  #/usr/local/include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN_INCLUDE_DIRS}
)

#link_directories(/usr/local/lib)


## Declare a cpp library
# add_library(obstacle_avoidance
#   src/${PROJECT_NAME}/obstacle_avoidance.cpp
# )

#file(GLOB WAYPOINT_LIB RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/waypoint_lib/*)

#add_library(WaypointLib STATIC
            #data/test_waypoints.npz
            #include/waypoint_lib/waypoint_manager.h
            #include/waypoint_lib/waypoint_manager.cpp)
#target_link_libraries(WaypointLib cnpy)

## Declare a cpp executable
add_executable( ${PROJECT_NAME}
                src/map_merger_node.cpp
                src/map_merger.cpp
                include/map_merger.h
)

#add_dependencies(relative_waypoint_manager rosflight_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable( ${TEST_NAME}
                #src/waypoint_manager_test.cpp
#)

#target_link_libraries(${TEST_NAME}
                      #${catkin_LIBRARIES}
                      #WaypointLib
                      #cnpy
                      #gtest
                      #gtest_main)



