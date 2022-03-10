cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_2dmap)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
        )

catkin_package(
        CATKIN_DEPENDS
        std_msgs

        DEPENDS PCL
        )

###########
## Build ##
###########

include_directories(
  include
  ${CV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_to_2dmap src/pointcloud_to_2dmap.cpp)
target_link_libraries(pointcloud_to_2dmap
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(pmf src/pmf.cpp)
target_link_libraries(pmf
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        )

install(TARGETS
        pointcloud_to_2dmap
        pmf

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )