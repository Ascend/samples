cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(CUDA)
find_package(Eigen3 QUIET)

#AW_CHECK_CUDA()

if (USE_CUDA)
    add_definitions(-DCUDA_FOUND)
    list(APPEND ndt_gpu)
endif ()

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
	# velodyne_pointcloud
	velodyne_pcl
	ole_pointcloud
        ndt_cpu
        ndt_omp
        message_generation
        )

###################################
## catkin specific configuration ##
###################################

add_service_files(
        FILES
        GlobalLocalizer.srv
)

generate_messages(
        #
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        tf
	#velodyne_pointcloud
	velodyne_pcl
	ole_pointcloud
        ndt_cpu
        ndt_omp
        message_runtime
        DEPENDS PCL
)

###########
## Build ##
###########



include_directories(include ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})

add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
add_dependencies(ndt_mapping ${catkin_EXPORTED_TARGETS})

add_executable(global_localizer nodes/global_localizer/global_localizer.cpp)
target_link_libraries(global_localizer ${catkin_LIBRARIES})
add_dependencies(global_localizer ${catkin_EXPORTED_TARGETS})


if (USE_CUDA)
    target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(ndt_mapping PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(global_localizer PRIVATE ${CUDA_INCLUDE_DIRS})
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_matching PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
    set_target_properties(ndt_mapping PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
    set_target_properties(global_localizer PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))


add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(queue_counter ${catkin_EXPORTED_TARGETS})


install(TARGETS
        ndt_matching
        ndt_mapping
        global_localizer
        queue_counter

        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
