cmake_minimum_required( VERSION 2.8 )
project( DirectSparseVO )

# compiler
set( CMAKE_CXX_COMPILER "g++" )

# debug
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wno-reorder" )

# release
#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3")

### FIND_PACKAGE(OpenMP)
### IF(OPENMP_FOUND)
###  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
###  MESSAGE(STATUS "Compiling with OpenMP support")
### ENDIF(OPENMP_FOUND)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
include_directories( ${PROJECT_SOURCE_DIR}/include )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED HINTS "/opt/ros/kinetic/include/opencv-3.1.0-dev/" )
#include_directories(/opt/ros/kinetic/include/opencv-3.1.0-dev/)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( "/home/rochelle/Workspace/ygz-slam/thirdparty/Sophus/" )
# glog 
find_package( Glog REQUIRED )
include_directories( ${GLOG_INCLUDE_DIRS} )
# boost
find_package( Boost COMPONENTS filesystem REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
# fast 
find_package( fast REQUIRED )
include_directories( ${fast_INCLUDE_DIRS} )
# ceres 
####  include_directories( "/home/rochelle/Workspace/3rdparty/ceres-solver/include/" )
####  include_directories( "/home/rochelle/Workspace/3rdparty/ceres-solver/build/config/" )
####  set(CERES_LIBRARIES "/home/rochelle/Workspace/3rdparty/ceres-solver/build/lib/libceres.a")
# pangolin 
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )
# cholmod
#### set(CHOLMOD_LIBRARIES "/usr/lib/x86_64-linux-gnu/libcholmod.so;/usr/lib/x86_64-linux-gnu/libamd.so;/usr/lib/x86_64-linux-gnu/libcolamd.so;/usr/lib/x86_64-linux-gnu/libcamd.so;/usr/lib/x86_64-linux-gnu/libccolamd.so;/usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so")
# g2o
#### include_directories( "/home/rochelle/Workspace/3rdparty/g2o/" )
#### include_directories( "/home/rochelle/Workspace/3rdparty/g2o/build/" )
#### file(GLOB G2O_LIBS "/home/rochelle/Workspace/3rdparty/g2o/lib/*.so")

# TODO: add gtsam and other libs you will use
# dbow3 
include_directories( ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/src )

# pcl
#include_directories( "/home/rochelle/Workspace/3rdparty/pcl/" )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${GLOG_LIBRARY}
    ${Boost_LIBRARIES}
    ${fast_LIBRARIES}
####    ${CSPARSE_LIBRARY}
####    ${CERES_LIBRARIES}
####    ${CHOLMOD_LIBRARIES}
    ${Pangolin_LIBRARIES}
###    ${G2O_LIBS}
    /home/rochelle/Workspace/ygz-slam/thirdparty/DBoW3/build/src/libDBoW3.so
    /home/rochelle/Workspace/ygz-slam/thirdparty/Sophus/build/libSophus.so
)

############### executable ######################
add_subdirectory( src )
add_subdirectory( test )
