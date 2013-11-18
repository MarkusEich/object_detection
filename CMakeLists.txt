# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
include(${CMAKE_SOURCE_DIR}/cmake/modules/Rock.cmake)
rock_init(object_detection 0.1)
rock_find_cmake(Boost)
rock_standard_layout()
