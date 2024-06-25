cmake_minimum_required(VERSION 3.22)
project(CDT)

### Options
option(BUILD_EXAMPLE "Example" OFF)
option(BUILD_TESTS "Tests" OFF)

### CMake config ###
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS YES)

add_library(project_warnings INTERFACE)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
include(cmake/CompilerFlags.cmake)
include(cmake/ExternalTools.cmake)

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}  
        PRIVATE
        src/Triangulation.cpp
        src/Grid.cpp
        )

target_include_directories(${PROJECT_NAME}
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_EXAMPLE)
    add_executable(CDTDemo src/CDTDemo/main.cpp)
    target_link_libraries(CDTDemo PRIVATE CDT sfml-graphics sfml-window sfml-system ImGui-SFML::ImGui-SFML)
endif()

if(BUILD_TESTS)
    enable_testing()
    include(GoogleTest)

    add_executable(test_all src/Tests/main.cpp)
    target_link_libraries(test_all PRIVATE gtest CDT)

    gtest_discover_tests(test_all)
endif()


