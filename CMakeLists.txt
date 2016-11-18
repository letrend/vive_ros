cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#==============================================================================
# Dependencies
#==============================================================================

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        std_srvs
        geometry_msgs
        tf_conversions
        )

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
message(STATUS ${EIGEN3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_xfeatures2d REQUIRED)

# OpenVR
find_package( OpenVR )
include_directories( ${OpenVR_INCLUDE_DIRS} )

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#==============================================================================
# Application
#==============================================================================

# VIVE node:
add_executable(vive_node
        src/vive_node.cpp
        src/vr_interface.cpp
        )
target_link_libraries(vive_node ${catkin_LIBRARIES} ${OpenVR_LIBRARIES})

# VIVE reconstructor:
add_executable(reconstructor
        src/reconstructor.cpp
        src/vr_interface.cpp
        )
target_link_libraries(reconstructor ${catkin_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS})
