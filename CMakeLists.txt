cmake_minimum_required(VERSION 3.21)

project(JcnsFranka LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(orl REQUIRED)

add_library(JcnsFranka SHARED
  jcnsfranka_global.h
  jcnsfranka.cpp
  jcnsfranka.h
  examples_common.h
  examples_common.cpp
)

target_compile_definitions(JcnsFranka PRIVATE JCNSFRANKA_LIBRARY)
target_link_libraries(JcnsFranka Eigen3::Eigen Franka::Franka orl::orl)

install(TARGETS JcnsFranka DESTINATION "${PROJECT_SOURCE_DIR}/demo/jcnsfranka/lib")
install(FILES
    jcnsfranka.h
    jcnsfranka_global.h
    examples_common.h
    DESTINATION "${PROJECT_SOURCE_DIR}/demo/jcnsfranka/include")
