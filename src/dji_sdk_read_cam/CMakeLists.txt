cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_read_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages
catkin_package()

#for aruco
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  #opencv2
  roscpp
  sensor_msgs
  std_msgs
  dji_sdk
)

find_package(aruco REQUIRED)#for aruco

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    MESSAGE("openMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include_directories(
  include
  include/AprilTags
  include/DJI
  include/MST
  include/ARToolKitPlus
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}    #for aruco
  )

link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)

add_executable(dji_sdk_read_cam src/nv_cam.cpp src/apriltagdetector.cpp  src/CircleDetection.cpp)
target_link_libraries(
  dji_sdk_read_cam
  dcam
  apriltags
  ARToolKitPlus
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${aruco_LIBS})          #for aruco
add_dependencies(dji_sdk_read_cam dji_sdk_generate_messages_cpp)
