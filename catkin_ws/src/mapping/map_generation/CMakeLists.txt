cmake_minimum_required(VERSION 2.8.3)
project(map_generation)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  aruco_msgs
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES
   CreateMap.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   aruco_msgs
   std_msgs
   geometry_msgs
 )

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  roscpp
  aruco_msgs
  std_msgs  
  tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(map_client src/map_client_node.cpp)
add_dependencies(map_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_client ${catkin_LIBRARIES})

add_executable(map_service src/map_service_node.cpp)
add_dependencies(map_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_service ${catkin_LIBRARIES})




