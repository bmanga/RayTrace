cmake_minimum_required(VERSION 3.12)
project(RayTrace)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -flto")


include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(RayTrace
        src/main.cpp
        )
target_include_directories(RayTrace PRIVATE include/)
conan_target_link_libraries(RayTrace)
