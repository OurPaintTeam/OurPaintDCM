cmake_minimum_required(VERSION 3.26)
project(OurPaintDCM VERSION 0.01 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
# Основная библиотека OurPaintDCM
add_library(OurPaintDCM STATIC ${SRC_FILES})

target_include_directories(OurPaintDCM PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/figures>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/functions>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/system>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/requirements>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/headers/utils>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/optimizers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/optimizers/MatrixOptimizers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/optimizers/EigenOptimizers>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/decomposition>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/graph>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/Tasks>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/Tasks/Matrix>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/math/headers/Tasks/Eigen>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/figures>
        $<INSTALL_INTERFACE:include/functions>
        $<INSTALL_INTERFACE:include/system>
        $<INSTALL_INTERFACE:include/requirements>
        $<INSTALL_INTERFACE:include/utils>
        $<INSTALL_INTERFACE:include/math>
        $<INSTALL_INTERFACE:include/math/optimizers>
        $<INSTALL_INTERFACE:include/math/optimizers/MatrixOptimizers>
        $<INSTALL_INTERFACE:include/math/optimizers/EigenOptimizers>
        $<INSTALL_INTERFACE:include/math/decomposition>
        $<INSTALL_INTERFACE:include/math/graph>
        $<INSTALL_INTERFACE:include/math/Tasks>
        $<INSTALL_INTERFACE:include/math/Tasks/Matrix>
        $<INSTALL_INTERFACE:include/math/Tasks/Eigen>
)

target_link_libraries(OurPaintDCM PUBLIC Math)
target_link_libraries(OurPaintDCM PUBLIC Eigen3::Eigen)
add_subdirectory(math)
add_subdirectory(app)

enable_testing()
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
add_subdirectory(tests)
