cmake_minimum_required(VERSION 3.21)

project(jcnsfranka
    VERSION 0.1.1
    LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(orl REQUIRED)

add_library(jcnsfranka SHARED
  jcnsfranka_global.h
  jcnsfranka.cpp
  jcnsfranka.h
)

target_compile_definitions(jcnsfranka PRIVATE JCNSFRANKA_LIBRARY)
target_link_libraries(jcnsfranka Eigen3::Eigen Franka::Franka orl::orl)

install(TARGETS jcnsfranka DESTINATION "${PROJECT_SOURCE_DIR}/demo/jcnsfranka/lib")
install(FILES
    jcnsfranka.h
    jcnsfranka_global.h
    DESTINATION "${PROJECT_SOURCE_DIR}/demo/jcnsfranka/include")
