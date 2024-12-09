cmake_minimum_required(VERSION 3.0.2)
project(occupancygrid_creator)


## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

# Define the current package
catkin_package(
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 nav_msgs
                 geometry_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# Directories to include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${PROJECT_NAME}_gencfg
)

set(LIBRARY_HEADERS ${LIBRARY_HEADERS}
  include/${PROJECT_NAME}/occupancygrid_creator.h
)

set(LIBRARY_SOURCES ${LIBRARY_SOURCES}
  src/occupancygrid_creator.cpp
)


# Add the sources and headers to the library
add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS} )
# Link the library to other libraries needed
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
# Specify include directories to use when compiling given target
target_include_directories(${PROJECT_NAME} PUBLIC)

# Sometimes need to tell cmake the language we are using
#set_target_properties(lmpcc_solver_lib PROPERTIES LINKER_LANGUAGE CXX)

# Installation rules for the created library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# Executable node
add_executable(occupancygrid_creator_node src/occupancygrid_creator_node.cpp)
add_dependencies(occupancygrid_creator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occupancygrid_creator_node ${catkin_LIBRARIES} ${PROJECT_NAME})

