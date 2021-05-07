cmake_minimum_required(VERSION 2.8.3)
project(norlab_dense_mapper)

set(CMAKE_CXX_STANDARD 11)

set(EXTERNAL_INCLUDE_DIRS "")
set(EXTERNAL_LIBS "")

find_package(libpointmatcher CONFIG)
set(EXTERNAL_INCLUDE_DIRS ${EXTERNAL_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libpointmatcher_LIBRARIES})

# norlab_dense_mapper target
include_directories(norlab_dense_mapper ${libpointmatcher_INCLUDE_DIRS})
add_library(norlab_dense_mapper SHARED norlab_dense_mapper/DenseMapper.cpp norlab_dense_mapper/DenseMap.cpp norlab_dense_mapper/Trajectory.cpp norlab_dense_mapper/RAMCellManager.cpp norlab_dense_mapper/HardDriveCellManager.cpp)
target_link_libraries(norlab_dense_mapper ${libpointmatcher_LIBRARIES})

# install target
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(DEF_INSTALL_CMAKE_DIR lib/cmake/norlab_dense_mapper CACHE PATH "Installation directory for CMake files")

install(TARGETS norlab_dense_mapper DESTINATION ${INSTALL_LIB_DIR})

install(FILES norlab_dense_mapper/DenseMapper.h norlab_dense_mapper/DenseMap.h norlab_dense_mapper/Trajectory.h norlab_dense_mapper/CellManager.h
        DESTINATION ${INSTALL_INCLUDE_DIR}/norlab_dense_mapper
        )

configure_file(norlab_dense_mapperConfig.cmake.in
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_dense_mapperConfig.cmake" @ONLY
        )
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/norlab_dense_mapperConfig.cmake"
        DESTINATION ${DEF_INSTALL_CMAKE_DIR}
        COMPONENT dev
        )

# uninstall target
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()
