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

    # Pure MCP Host/Plugin Interface Header (ITAR-safe, no implementation)
    include/mcp/IMcpPlugin.h

    # Hardware Abstraction Layer (HAL)
    src/hal/PicoSerialController.cpp
    include/hal/ICmmController.h
    include/hal/PicoSerialController.h

    # Metrology Math & CAD Twin
    src/math/MetrologyMath.cpp
    include/math/MetrologyMath.h
    src/cad/CmmDigitalTwin.cpp
    include/cad/CmmDigitalTwin.h

    # DMIS 5.3 & Characterization Parsers
    src/dmis/DMIS53Parser.cpp
    include/dmis/DMIS53Parser.h
    src/dmis/ChFileParser.cpp
    include/dmis/ChFileParser.h
    src/dmis/QifExporter.cpp
    include/dmis/QifExporter.h

    # User Interface & Viewports
    src/ui/MainWindow.cpp
    include/ui/MainWindow.h
    src/ui/OcctViewport.cpp
    include/ui/OcctViewport.h
    src/ui/SettingsDialog.cpp
    include/ui/SettingsDialog.h
    src/ui/CalibrationWidget.cpp
    include/ui/CalibrationWidget.h
    src/ui/RibbonCustomizeDialog.cpp
    include/ui/RibbonCustomizeDialog.h
)

add_executable(OpenCMM ${PROJECT_SOURCES})

target_include_directories(OpenCMM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCASCADE_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
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

# Define the DMIS parser test executable
add_executable(dmis_parser_test
    src/dmis/DMIS53Parser.cpp
    include/dmis/DMIS53Parser.h
    src/dmis/DMIS53ParserTest.cpp
)

target_include_directories(dmis_parser_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dmis_parser_test PRIVATE
    Qt6::Core
    Eigen3::Eigen
)

add_test(NAME DMIS53ParserValidationTest COMMAND dmis_parser_test --test-dmis)

# --- Optional Modular MCP Server Plugin (Disable or omit for ITAR compliance) ---
option(OPENCMM_BUILD_MCP_PLUGIN "Build optional modular MCP Server plugin DLL" ON)

if(OPENCMM_BUILD_MCP_PLUGIN)
    add_library(opencmm_mcp MODULE
        include/mcp/IMcpPlugin.h
        src/mcp/OpenCmmMcpPlugin.h
        src/mcp/OpenCmmMcpPlugin.cpp
    )

    target_include_directories(opencmm_mcp PRIVATE include)
    target_link_libraries(opencmm_mcp PRIVATE Qt6::Core ws2_32)

    set_target_properties(opencmm_mcp PROPERTIES
        AUTOMOC ON
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
    )

    # Standalone zero-dependency stdio-to-TCP bridge for MCP clients
    add_executable(opencmm_mcp_bridge
        src/mcp/mcp_stdio_bridge.cpp
    )
    add_Dependencies(OpenCMM opencmm_mcp opencmm_mcp_bridge)
    target_compile_definitions(opencmm_mcp_bridge PRIVATE OPENCMM_BRIDGE_STANDALONE)
    target_link_libraries(opencmm_mcp_bridge PRIVATE ws2_32)
    set_target_properties(opencmm_mcp_bridge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
    )

    install(TARGETS opencmm_mcp opencmm_mcp_bridge
        RUNTIME DESTINATION plugins COMPONENT mcp_module
        LIBRARY DESTINATION plugins COMPONENT mcp_module
    )
endif()

# --- CPack Component Configuration for Installer (NSIS / WiX / IFW) ---
include(CPackComponent)
cpack_add_component(core_studio
    DISPLAY_NAME "OpenCMM Metrology Studio (Core)"
    DESCRIPTION "Core CAD, DRO, DMIS 5.3, and QIF 2.1 inspection studio (ITAR-Safe)."
    REQUIRED
)
cpack_add_component(mcp_module
    DISPLAY_NAME "AI Model Context Protocol (MCP) Server"
    DESCRIPTION "Optional local MCP plugin for AI agent integration. Leave unchecked for strict ITAR / air-gapped environments."
    DISABLED
)