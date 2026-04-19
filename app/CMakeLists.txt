cmake_minimum_required(VERSION 3.26)
project(OurPaintDCMConsole LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OURPAINTDCM_CONSOLE_USE_LOCAL_SOURCE
        "Build and link local OurPaintDCM source when target is not available"
        ON)

if(NOT TARGET OurPaintDCM)
    if(OURPAINTDCM_CONSOLE_USE_LOCAL_SOURCE)
        add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_BINARY_DIR}/ourpaintdcm-lib")
    else()
        find_package(OurPaintDCM REQUIRED CONFIG)
    endif()
endif()

add_executable(dcm_console main.cpp)
target_link_libraries(dcm_console PRIVATE OurPaintDCM)
