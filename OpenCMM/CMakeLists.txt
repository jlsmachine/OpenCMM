cmake_minimum_required(VERSION 3.24)
project(OpenCMM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 1. Dependencies via vcpkg
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCASCADE CONFIG REQUIRED)
find_package(Qt6 COMPONENTS Core Gui Widgets SerialPort REQUIRED)

# 2. Source & Header Files (Including Milestone 2 Viewport)
set(PROJECT_SOURCES
    src/main.cpp
    src/hal/PicoSerialController.cpp
    src/math/MetrologyMath.cpp
    src/ui/MainWindow.cpp
    src/ui/OcctViewport.cpp
    include/core/Units.h
    include/hal/ICmmController.h
    include/hal/PicoSerialController.h
    include/math/MetrologyMath.h
    include/ui/MainWindow.h
    include/ui/OcctViewport.h
)

add_executable(OpenCMM ${PROJECT_SOURCES})

target_include_directories(OpenCMM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(OpenCMM PRIVATE
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
    ${OpenCASCADE_LIBRARIES}
)

if(WIN32)
    target_compile_definitions(OpenCMM PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()