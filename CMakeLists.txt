cmake_minimum_required(VERSION 3.5.2)
project(brandes)
find_package ( Threads REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -O2")

set(SOURCE_FILES src/brandes.cpp src/brandes.h)
add_executable(brandes ${SOURCE_FILES})