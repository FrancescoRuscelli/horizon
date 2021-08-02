project(horizon)
cmake_minimum_required(VERSION 3.0)

find_package(Eigen3 REQUIRED)
find_package(casadi 3.5.5 REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

# ilqr library
add_library(ilqr STATIC
    src/ilqr.cpp
    src/wrapped_function.cpp
)

target_link_libraries(ilqr Eigen3::Eigen casadi)


# python binding
set(PYBIND11_PYTHON_VERSION 3)
find_package(pybind11 REQUIRED)

pybind11_add_module(pyilqr pyilqr.cpp)
target_link_libraries(pyilqr PUBLIC ilqr)


# test executable
add_executable(ilqr_test src/ilqr_test.cpp)
target_link_libraries(ilqr_test ilqr)
