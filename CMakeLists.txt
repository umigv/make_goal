cmake_minimum_required(VERSION 2.8.3)
project(make_goal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS move_base_msgs nav_msgs roscpp std_msgs
                                        tf2 tf2_geometry_msgs tf2_ros
                                        umigv_utilities geographic_msgs sensor_msgs geodesy)

find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS move_base_msgs nav_msgs roscpp std_msgs tf2
                              tf2_geometry_msgs tf2_ros umigv_utilities
               DEPENDS Boost)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(SOURCES src/goal_director.cpp src/make_goal.cpp
            src/message_utils.cpp)

add_executable(make_goal ${SOURCES})
target_link_libraries(make_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(make_goal ${catkin_EXPORTED_TARGETS})

install(TARGETS make_goal
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
