cmake_minimum_required(VERSION 2.8)
project(python_test)
#set(CMAKE_BUILD_TYPE Debug)

find_package(Boost COMPONENTS thread serialization python REQUIRED)
include_directories(${Boost_INCLUDE_DIR} 
/usr/include/python2.7)

add_executable(python_cpp main.cpp)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

target_link_libraries(python_cpp
    ${Boost_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libboost_python-py27.so
    /usr/lib/x86_64-linux-gnu/libstdc++.so.6
    /usr/lib/x86_64-linux-gnu/libpython2.7.so
    )    
