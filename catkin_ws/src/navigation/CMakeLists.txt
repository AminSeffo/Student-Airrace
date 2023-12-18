cmake_minimum_required(VERSION 2.8.3)
project(navigation)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  std_msgs
  aruco_msgs
  geometry_msgs
  tf
  eigen_catkin
)

catkin_package(CATKIN_DEPENDS 
  roscpp
  aruco_msgs
  std_msgs  
  
)


include_directories(${catkin_INCLUDE_DIRS}
  include
)



add_executable(calibration src/calibration.cpp)
add_dependencies(calibration ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibration ${catkin_LIBRARIES})

add_executable(navigation src/main.cpp src/navigation.cpp)
add_dependencies(navigation ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation ${catkin_LIBRARIES})



