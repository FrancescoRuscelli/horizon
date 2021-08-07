project(horizon)
cmake_minimum_required(VERSION 3.0)

# options
option(HORIZON_PROFILING OFF "enable profiling features")

if(${HORIZON_PROFILING})
    add_definitions(-DHORIZON_PROFILING)
endif()

find_package(Eigen3 REQUIRED)
find_package(casadi 3.5.5 REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(CMAKE_CXX_STANDARD 17)



# ilqr library
add_library(ilqr STATIC
    src/ilqr.cpp
    src/wrapped_function.cpp
    src/profiling.cpp
)

# sqp library
add_library(sqp STATIC
    src/wrapped_function.cpp
    src/profiling.cpp
)


target_link_libraries(ilqr Eigen3::Eigen casadi)
target_link_libraries(sqp Eigen3::Eigen casadi)

# python binding
set(PYBIND11_PYTHON_VERSION 3)
find_package(pybind11 REQUIRED)

pybind11_add_module(pyilqr pyilqr.cpp)
target_link_libraries(pyilqr PUBLIC ilqr)

install(TARGETS pyilqr
    DESTINATION horizon/solvers)


# test executable
add_executable(ilqr_test src/ilqr_test.cpp)
target_link_libraries(ilqr_test ilqr)

add_executable(sqp_test src/sqp_test.cpp)
target_link_libraries(sqp_test sqp)
