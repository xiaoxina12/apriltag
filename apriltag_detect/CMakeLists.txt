cmake_minimum_required(VERSION 3.0.2)
project(apriltag_detect)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages



find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    image_transport
    cv_bridge
    message_filters
    sensor_msgs
    std_msgs
    tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
  include
  ${EIGEN3_INCLUDE_DIRS}
  DEPENDS system_lib
  apriltag
)



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)


link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(apriltag_detect  src/apriltag_detect_node.cpp  src/common_functions.cpp)
add_dependencies(apriltag_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apriltag_detect  apriltag::apriltag  ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS}  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so.0.7.0 )

