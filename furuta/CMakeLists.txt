cmake_minimum_required(VERSION 3.16)
project(furuta LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(furuta src/main.cpp)
target_include_directories(furuta PRIVATE include)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(furuta PRIVATE Eigen3::Eigen)