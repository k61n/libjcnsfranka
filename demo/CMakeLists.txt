cmake_minimum_required(VERSION 3.21)

project(franka_demo VERSION 0.2 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT DEFINED CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH /home/jcns/Qt/6.3.1/gcc_64)
endif()
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

if(NOT DEFINED JCNSFRANKA_DIR)
    set(JCNSFRANKA_DIR ${PROJECT_SOURCE_DIR}/jcnsfranka)
endif()
INCLUDE_DIRECTORIES(${JCNSFRANKA_DIR}/include)
LINK_DIRECTORIES(${JCNSFRANKA_DIR}/lib)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        res.qrc
)

add_executable(franka_demo ${PROJECT_SOURCES})
target_link_libraries(franka_demo Qt${QT_VERSION_MAJOR}::Widgets Franka::Franka Eigen3::Eigen JcnsFranka)

install(TARGETS franka_demo DESTINATION "${PROJECT_SOURCE_DIR}/install")
