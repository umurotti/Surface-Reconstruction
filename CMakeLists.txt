cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(FARO_LABS_INTERN_TASK)
find_package(PCL 1.3 REQUIRED COMPONENTS common io surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(surface_reconstruct surface_reconstruct.cpp)
target_link_libraries(surface_reconstruct ${PCL_LIBRARIES})