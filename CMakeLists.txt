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

# 2. Consolidated Source & Header Files
set(PROJECT_SOURCES
    # Core & Main Entry
    src/main.cpp
    src/core/AppSettings.cpp
    include/core/AppSettings.h
    include/core/Units.h

    # Hardware Abstraction Layer (HAL)
    src/hal/PicoSerialController.cpp
    include/hal/ICmmController.h
    include/hal/PicoSerialController.h

    # Metrology Math & CAD Twin
    src/math/MetrologyMath.cpp
    include/math/MetrologyMath.h
    src/cad/CmmDigitalTwin.cpp

    # DMIS 5.3 & Characterization Parsers
    src/dmis/DMIS53Parser.cpp
    include/dmis/DMIS53Parser.h
    #src/dmis/DMIS53ParserTest.cpp
    src/dmis/ChFileParser.cpp
    include/dmis/ChFileParser.h

    # User Interface & Viewports
    src/ui/MainWindow.cpp
    include/ui/MainWindow.h
    src/ui/OcctViewport.cpp
    include/ui/OcctViewport.h
    src/ui/SettingsDialog.cpp
    include/ui/SettingsDialog.h
    src/dmis/QifExporter.cpp       # <--- Add this line
    include/dmis/QifExporter.h
)

add_executable(OpenCMM ${PROJECT_SOURCES} "include/DMIS/QifExporter.h")

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

# Automatically copy Qt platforms plugin to output directory on Windows
if(WIN32)
    add_custom_command(TARGET OpenCMM POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:OpenCMM>/platforms
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/Qt6/plugins/platforms/qwindows.dll"
            "$<TARGET_FILE_DIR:OpenCMM>/platforms/qwindows.dll"
        COMMENT "Deploying Qt windows platform plugin..."
        VERBATIM
    )
endif()

# Copy Machines directory to build output
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/Machines" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# Enable testing framework support
enable_testing()

# Define the DMIS parser test executable (incorporating your test runner source)
add_executable(dmis_parser_test
    src/dmis/DMIS53Parser.cpp
    include/dmis/DMIS53Parser.h
    src/dmis/DMIS53ParserTest.cpp
 "include/DMIS/QifExporter.h")

target_include_directories(dmis_parser_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dmis_parser_test PRIVATE
    Qt6::Core
)

# Register the test executable with CTest using the --test-dmis flag
add_test(NAME DMIS53ParserValidationTest COMMAND dmis_parser_test --test-dmis)
