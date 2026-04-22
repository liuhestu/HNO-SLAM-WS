cmake_minimum_required(VERSION 3.0.2)
project(hno_slam)

# Enable C++17 for structured bindings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  message_filters
  tf
  ov_core
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hno_slam_core
  CATKIN_DEPENDS roscpp ov_core
)

include_directories(
  include/hno_slam
  thirdparty
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(hno_slam_core
  src/State.cpp
  src/Initializer.cpp
  src/Frontend.cpp
  src/Observer.cpp
  src/System.cpp
)
target_link_libraries(hno_slam_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hno_slam_node src/main_node.cpp)
target_link_libraries(hno_slam_node hno_slam_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS hno_slam_core hno_slam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
