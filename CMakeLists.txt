cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)

# find_package(Eigen3)
find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED )
find_package(FMT REQUIRED)
LIST( APPEND CMAKE_MODULE_PATH /home/sky/programs/g2o/cmake_modules )
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
find_package(Qt5 COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)
include_directories("/usr/include/suitesparse/")


add_executable(cameraCalib src/cameraCalib.cpp)
target_link_libraries(cameraCalib Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pcdTransfer src/pcdTransfer.cpp src/common.h)
target_link_libraries(pcdTransfer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cornerPhoto src/corner_photo.cpp src/common.h)
target_link_libraries(cornerPhoto Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(getExt1 src/cam_lid_external1.cpp src/common.h)
target_link_libraries(getExt1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(getExt4g2o src/cam_lid_g2o.cpp src/common.h)
target_link_libraries(getExt4g2o ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${Sophus_LIBRARIES}
                              fmt::fmt g2o_core g2o_types_slam3d g2o_solver_csparse 
                              g2o_stuff g2o_csparse_extension)

add_executable(getExt2 src/cam_lid_external2.cpp src/common.h)
target_link_libraries(getExt2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})


add_executable(projectCloud src/projectCloud.cpp src/common.h)
target_link_libraries(projectCloud Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(colorLidar src/color_lidar_display.cpp src/common.h)
target_link_libraries(colorLidar Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})



