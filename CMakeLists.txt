project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
# debug宏
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

AUX_SOURCE_DIRECTORY(./src src)
ADD_EXECUTABLE(main ${src})