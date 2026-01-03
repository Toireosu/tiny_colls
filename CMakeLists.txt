cmake_minimum_required(VERSION 4.0)
project(tiny_colls)

include(FetchContent)
include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_C_CLANG_TIDY
# clang-tidy;
# -warnings-as-errors=*
# )

add_library(tiny_colls)
set_property(TARGET tiny_colls PROPERTY CXX_STANDARD 20)
target_sources(tiny_colls
    PRIVATE
        src/tiny_colls.cc
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS
            include
        FILES
            include/tiny_colls.h
            include/tiny_colls/collider.h
            include/tiny_colls/collision.h
)

target_include_directories(tiny_colls
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tiny_colls/details>
)

add_subdirectory(examples)
add_subdirectory(tests)