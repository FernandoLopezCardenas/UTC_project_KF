cmake_minimum_required(VERSION 3.10)

project(KFC_codigo)

add_subdirectory(${CMAKE_SOURCE_DIR}/src)

find_package(Eigen3 3.3 REQUIRED)

add_compile_options(-Wall -Wextra)