cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

project(0104.MaximumDepthOfBinaryTree CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("../CommonDefs")

set(SOURCES solution.cpp)

set(HEADERS)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} PRIVATE gtest gtest_main)

add_test(NAME 0104.MaximumDepthOfBinaryTree COMMAND ${PROJECT_NAME})
