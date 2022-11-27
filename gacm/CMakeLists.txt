cmake_minimum_required(VERSION 2.8)
project(gacm)

# set(CMAKE_BUILD_TYPE debug)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -Wall   -O3 -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -Wall -O3")

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        cv_bridge
        image_transport

        #pcl_ros
        #pcl_conversions

        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        cloud_msgs

        camera_model
        netvlad_tf_test
)


find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()

# find_package(Sophus REQUIRED)
# find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS cloud_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}

)

link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
)

add_executable(featureTrackingNode src/featureTrackingNode.cpp src/parameters.cpp)
add_dependencies(featureTrackingNode ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureTrackingNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(plOdometryNode src/plOdometryNode.cpp src/parameters.cpp)
add_dependencies(plOdometryNode ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(plOdometryNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(submapManagerNode src/submapManagerNode.cpp src/parameters.cpp)
add_dependencies(submapManagerNode ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(submapManagerNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})

