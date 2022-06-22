cmake_minimum_required(VERSION 3.14)

project(jcnsfranka LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(jcnsfranka SHARED
  jcnsfranka_global.h
  jcnsfranka.cpp
  jcnsfranka.h
)

target_compile_definitions(jcnsfranka PRIVATE JCNSFRANKA_LIBRARY)
target_link_libraries(jcnsfranka Franka::Franka Eigen3::Eigen)
