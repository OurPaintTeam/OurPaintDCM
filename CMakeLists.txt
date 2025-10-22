cmake_minimum_required(VERSION 3.26)
project(OurPaintDCM VERSION 0.01 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${PROJECT_SOURCE_DIR}/headers
        ${PROJECT_SOURCE_DIR}/headers/figures
        ${PROJECT_SOURCE_DIR}/headers/functions
        ${PROJECT_SOURCE_DIR}/headers/requirements
        ${PROJECT_SOURCE_DIR}/headers/utils)
include_directories(math/headers
        math/headers/optimizers
        math/headers/optimizers/MatrixOptimizers
        math/headers/optimizers/EigenOptimizers
        math/headers/optimizers
        math/headers/decomposition
        math/headers/graph
        math/headers/Tasks/Matrix
        math/headers/Tasks/Eigen
        math/headers/Tasks)
file(GLOB_RECURSE SRC_FILES src/*.cpp)
# Основная библиотека OurPaintDCM
add_library(OurPaintDCM STATIC ${SRC_FILES})
target_link_libraries(OurPaintDCM PUBLIC Math)
target_link_libraries(OurPaintDCM PUBLIC Eigen3::Eigen)
add_subdirectory(math)
target_include_directories(OurPaintDCM PUBLIC ${PROJECT_SOURCE_DIR}/include)

enable_testing()
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
add_subdirectory(tests)
