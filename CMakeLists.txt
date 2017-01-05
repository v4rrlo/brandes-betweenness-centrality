cmake_minimum_required(VERSION 3.6)
project(PW_PROJEKT_2)
find_package ( Threads REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -O2")

set(SOURCE_FILES brandes.cpp brandes.h)
add_executable(PW_PROJEKT_2 ${SOURCE_FILES})