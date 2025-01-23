cmake_minimum_required(VERSION 3.5)
project(euclidean_clustering)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
)

## Find PCL library
find_package(PCL REQUIRED)

## Include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Declare catkin package
catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
        DEPENDS PCL
)

## Add executable
add_executable(euclidean_clustering_node src/euclidean_clustering_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(euclidean_clustering_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

## Add dependencies to ensure correct build order
add_dependencies(euclidean_clustering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)