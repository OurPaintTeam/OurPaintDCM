cmake_minimum_required(VERSION 3.26)
project(OurPaintDCM VERSION 0.01 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)
# Основная библиотека OurPaintDCM
add_library(OurPaintDCM STATIC ${SRC_FILES})

target_include_directories(OurPaintDCM PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/figures>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/functions>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/system>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/requirements>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/utils>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/figures>
        $<INSTALL_INTERFACE:include/functions>
        $<INSTALL_INTERFACE:include/system>
        $<INSTALL_INTERFACE:include/requirements>
        $<INSTALL_INTERFACE:include/utils>
)

target_link_libraries(OurPaintDCM PUBLIC Math)
add_subdirectory(math)

if(PROJECT_IS_TOP_LEVEL)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    add_subdirectory(tests)
endif()
