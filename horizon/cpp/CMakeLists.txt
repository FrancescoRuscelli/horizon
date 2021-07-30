project(horizon)
cmake_minimum_required(VERSION 3.0)

find_package(Eigen3 REQUIRED)
find_package(casadi 3.5.5 REQUIRED)

add_executable(ilqr
    src/ilqr.cpp
)

target_link_libraries(ilqr Eigen3::Eigen casadi)


