cmake_minimum_required(VERSION 3.16)
project(furuta LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(furuta src/main.cpp)
target_include_directories(furuta PRIVATE include)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(furuta PRIVATE Eigen3::Eigen)

include(CTest)
if(BUILD_TESTING)
    add_executable(lqr_design tools/lqr_design.cpp)
    target_include_directories(lqr_design PRIVATE include)
    target_link_libraries(lqr_design PRIVATE Eigen3::Eigen)
    add_test(NAME lqr_gains COMMAND lqr_design)

    add_executable(lqr_test tests/lqr_test.cpp)
    target_include_directories(lqr_test PRIVATE include)
    target_link_libraries(lqr_test PRIVATE Eigen3::Eigen)
    add_test(NAME lqr_control COMMAND lqr_test)
endif()
