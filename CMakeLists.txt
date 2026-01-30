cmake_minimum_required(VERSION 3.10)
project(rt_fci_udp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

add_executable(rt_fci_udp src/rt_fci_udp.cpp)

target_link_libraries(rt_fci_udp Franka::Franka Threads::Threads)

target_compile_options(rt_fci_udp PRIVATE -O2)
