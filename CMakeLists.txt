cmake_minimum_required(VERSION 3.10)
project(ompl_cpp_benchmark)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find OMPL, Assimp, and Eigen3
find_package(ompl REQUIRED)
find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(run_cpp_benchmark main.cpp)

target_include_directories(run_cpp_benchmark PRIVATE 
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(run_cpp_benchmark 
    ${OMPL_LIBRARIES} 
    assimp 
    Eigen3::Eigen
)
