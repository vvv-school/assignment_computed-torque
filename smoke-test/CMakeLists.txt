# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Francesco Romano <francesco.romano@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

# set the project name
project(TestAssignmentComputedTorque)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# add the required cmake packages
find_package(RobotTestingFramework 1.6.51 COMPONENTS DLL)
find_package(YARP 3.1.101 REQUIRED COMPONENTS OS dev sig math robottestingframework)
find_package(Eigen3 REQUIRED)

# add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(SYSTEM
                    ${RobotTestingFramework_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE test.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} ${RobotTestingFramework_LIBRARIES} ${YARP_LIBRARIES})
