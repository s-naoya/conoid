cmake_minimum_required(VERSION 2.8.12)
project(conoid)

set(CMAKE_CXX_FLAGS "-Wall -O2")
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

# setup conoid
set(SOURCES
    src/robot.cc
)
set(HEADERS
    include/robot.h
    include/utility.h
)

# setup eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# setup choreonoid
find_package(choreonoid)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

# create shared library
add_library(conoid SHARED ${SOURCES})
target_link_libraries(conoid PUBLIC
    ${CHOREONOID_LIBRARIES}
)

# install shared library and header
install(TARGETS conoid
    LIBRARY     DESTINATION lib
    INCLUDES    DESTINATION include
)
install(FILES ${HEADERS}
    DESTINATION include/conoid
)

# generate and install conoid-config.cmake
set(con_dir ${CMAKE_INSTALL_PREFIX})
set(include_dirs ${con_dir}/include ${EIGEN3_INCLUDE_DIR} ${CHOREONOID_INCLUDE_DIRS})
set(library_dirs ${con_dir}/lib/conoid ${CHOREONOID_LIBRARY_DIRS})
set(libs conoid ${CHOREONOID_LIBRARIES})
configure_file(cmake/conoid-config.cmake.in cmake/conoid-config.cmake @ONLY)
set(CMAKE_CONFIG_DIR share/cmake/conoid)
install(
  FILES cmake/conoid-config.cmake
  DESTINATION   ${CMAKE_CONFIG_DIR}
)