cmake_minimum_required(VERSION 3.10)
project(Robot_Play_Chess)

set(CMAKE_CXX_STANDARD 14)

add_executable(Robot_Play_Chess
        src/ChessEngine/ChessEngine.cpp)
