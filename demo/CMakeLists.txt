cmake_minimum_required(VERSION 3.16)

project(franka_demo
    VERSION 0.2.1
    LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(jcnsfranka REQUIRED)
find_package(orl REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        res.qrc
)

add_executable(franka_demo ${PROJECT_SOURCES})
target_link_libraries(franka_demo Qt6::Widgets orl::orl jcnsfranka::jcnsfranka)
