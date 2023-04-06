cmake_minimum_required (VERSION 3.8)
project(CodeCraftSDK)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_definitions(-DDEBUG)
add_compile_definitions(DISABLE_PTHREAD_HOOK)
add_compile_options(-O3)
if (!WIN32)
    link_libraries(pthread m)
endif (!WIN32)

AUX_SOURCE_DIRECTORY(./src src)
list(REMOVE_ITEM src "./load_pthread.cpp")
INCLUDE_DIRECTORIES(./src)
ADD_EXECUTABLE(main ${src})