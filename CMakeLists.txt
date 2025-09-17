cmake_minimum_required(VERSION 3.16)

project(jcnsfranka
    VERSION 0.4.5
    LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(orl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED CONFIG)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(Qt6 COMPONENTS Widgets QUIET)
if (${Qt6_FOUND})
    set(QtV 6)
else ()
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    set(QtV 5)
endif ()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

# libjcnsfranka
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_library(jcnsfranka SHARED ${SOURCES})
set_target_properties(jcnsfranka PROPERTIES
    VERSION ${jcnsfranka_VERSION}
    SOVERSION ${jcnsfranka_VERSION_MAJOR}
    OUTPUT_NAME "jcnsfranka")
target_compile_definitions(jcnsfranka PUBLIC JCNSFRANKA_LIBRARY)
target_link_libraries(jcnsfranka orl::orl)

# jcnsfranka-demo
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/demo)
if (${QtV} EQUAL 6)
    qt6_add_resources(DEMO_RES ${DEMO_RES})
else ()
    qt5_add_resources(DEMO_RES ${DEMO_RES})
endif ()
add_executable(jcnsfranka-demo ${DEMO_SOURCES} ${DEMO_RES})
target_link_libraries(jcnsfranka-demo Qt${QtV}::Widgets orl::orl jcnsfranka)

# Installation
include(GNUInstallDirs)
install(TARGETS jcnsfranka jcnsfranka-demo
    EXPORT jcnsfrankaTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/jcnsfranka)
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
# python library
install(DIRECTORY python/pyjcnsfranka
    DESTINATION "${Python3_SITELIB}")
install(FILES python/bin/jcnsfranka-comtest
    DESTINATION ${CMAKE_INSTALL_BINDIR})

## Packaging
set(CPACK_PACKAGE_VENDOR "FZJ GmbH")
set(CPACK_PACKAGE_VERSION ${libjcnsfranka_VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FZJ GmbH")
include(CPack)
