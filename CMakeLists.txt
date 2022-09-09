cmake_minimum_required(VERSION 3.21)

project(jcnsfranka
    VERSION 0.1.1
    LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(orl REQUIRED)

add_library(jcnsfranka SHARED
  jcnsfranka.cpp
  jcnsfranka.h
)

target_compile_definitions(jcnsfranka PUBLIC JCNSFRANKA_LIBRARY)
target_link_libraries(jcnsfranka orl::orl)

# Installation
include(GNUInstallDirs)
install(TARGETS jcnsfranka DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES jcnsfranka.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

## Packaging
set(CPACK_PACKAGE_VENDOR "FZJ GmbH")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION ${libjcnsfranka_VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FZJ GmbH")
include(CPack)
