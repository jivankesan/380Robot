cmake_minimum_required(VERSION 3.16)
project(robot_raw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0 -fsanitize=address,undefined")

add_executable(robot
    src/main.cpp
    src/serial.cpp
    src/control.cpp
    src/fsm.cpp
)

target_include_directories(robot PRIVATE ${CMAKE_SOURCE_DIR})

# pthreads
find_package(Threads REQUIRED)
target_link_libraries(robot PRIVATE Threads::Threads)

install(TARGETS robot DESTINATION bin)
