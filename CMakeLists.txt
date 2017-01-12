cmake_minimum_required(VERSION 2.8.12)

project(assignment_computed_torque)

find_package(YARP REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
# find_package(iDynTree REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(SYSTEM ${yarpWholeBodyInterface_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp Module.h Module.cpp)
target_link_libraries(${PROJECT_NAME} ${yarpWholeBodyInterface_LIBRARIES} ${YARP_LIBRARIES})
