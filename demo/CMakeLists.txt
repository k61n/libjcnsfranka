cmake_minimum_required(VERSION 3.21)

project(franka_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT DEFINED ${MY_VARIABLE})
    set(JCNSFRANKA_DIR ${PROJECT_SOURCE_DIR}/../install)
endif()
INCLUDE_DIRECTORIES(${JCNSFRANKA_DIR}/include)
LINK_DIRECTORIES(${JCNSFRANKA_DIR}/lib)

file(GLOB SOURCES
)

add_executable(franka_demo main.cpp ${SOURCES})
target_link_libraries(franka_demo Franka::Franka Eigen3::Eigen JcnsFranka)
