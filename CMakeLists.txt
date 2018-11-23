cmake_minimum_required(VERSION 2.8.0)
project(conoid)

set(CMAKE_CXX_FLAGS "-Wall -O2")

set(SOURCES
    robot.cc
)
set(HEADERS
    robot.h
    utility.h
)

# setup eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# setup choreonoid
find_package(choreonoid)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_library(conoid SHARED ${SOURCES})
target_link_libraries(conoid ${CHOREONOID_LIBRARIES})

install(TARGETS conoid LIBRARY DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/conoid)