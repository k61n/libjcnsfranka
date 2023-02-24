cmake_minimum_required(VERSION 3.16)

project(jcnsfranka
    VERSION 0.2.4
    LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(orl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

add_library(jcnsfranka SHARED
        src/jcnsfranka.h
        src/robot.h
        src/robot.cpp
        src/robot_c.cpp
        src/comtest.h
        src/comtest.cpp
        src/gripper.h
        src/gripper.cpp
)

target_compile_definitions(jcnsfranka PUBLIC JCNSFRANKA_LIBRARY)
target_link_libraries(jcnsfranka orl::orl)

# Installation
include(GNUInstallDirs)
install(TARGETS jcnsfranka
        EXPORT jcnsfrankaTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES src/jcnsfranka.h src/robot.h src/comtest.h src/gripper.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/jcnsfranka)
install(DIRECTORY pyjcnsfranka/pyjcnsfranka
        DESTINATION "${Python3_SITELIB}")

# cmake targets
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/jcnsfranka)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        jcnsfrankaConfigVersion.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY SameMajorVersion)
configure_package_config_file(cmake/jcnsfrankaConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/jcnsfrankaConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/jcnsfrankaConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/jcnsfrankaConfigVersion.cmake"
        DESTINATION ${INSTALL_CONFIGDIR})
install(EXPORT jcnsfrankaTargets
        FILE jcnsfrankaTargets.cmake
        NAMESPACE jcnsfranka::
        DESTINATION ${INSTALL_CONFIGDIR})

## Packaging
set(CPACK_PACKAGE_VENDOR "FZJ GmbH")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION ${libjcnsfranka_VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FZJ GmbH")
include(CPack)
