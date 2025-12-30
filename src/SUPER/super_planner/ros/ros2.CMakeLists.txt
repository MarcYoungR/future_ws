cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_CXX_STANDARD 17)
option(RELEASE "Enable Release mode" ON)
#if (RELEASE)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall")
#else ()
#    set(CMAKE_BUILD_TYPE Debug)
#    set(CMAKE_CXX_FLAGS "-O -Wall -g")
#endif ()

add_compile_options(-Werror=return-type)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DUSE_ROS2)


string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

# for CLION to search the package, user should change to his/her own relative location
set(CMAKE_PREFIX_PATH ../../../install)
include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
        -ldw
)

set(ros_libs
        rclcpp
        std_msgs
        mars_quadrotor_msgs
        sensor_msgs
        tf2_ros
        nav_msgs
        geometry_msgs
        visualization_msgs
        rog_map
        tf2_ros
        pcl_conversions

)

include_directories(
        ${includes}
)


#=================FOR libsuper ============================#
#    file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super
        ${srcs}
)
target_include_directories(super PUBLIC
        ${includes})
ament_target_dependencies(super PUBLIC
        ${ros_libs}
)

#=================FOR Benchmark Node ============================#
add_executable(fsm_node
        Apps/fsm_node_ros2.cpp
)
target_link_libraries(fsm_node PUBLIC
        super
        ${third_party_libs}
)
ament_target_dependencies(fsm_node PUBLIC
        ${ros_libs}
)

#=================INSTALL all targets ============================#

install(
        TARGETS
        fsm_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(${includes})

install(DIRECTORY include/
        DESTINATION include/
)

# install the launch file
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)


ament_package()
