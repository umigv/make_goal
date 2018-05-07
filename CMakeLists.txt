cmake_minimum_required(VERSION 2.8.3)
project(make_goal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS move_base_msgs nav_msgs roscpp
                                        std_msgs umigv_utilities)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS move_base_msgs nav_msgs roscpp
                              std_msgs umigv_utilities)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(make_goal src/make_goal.cpp)
target_link_libraries(make_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(make_goal ${catkin_EXPORTED_TARGETS})

install(TARGETS make_goal
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
