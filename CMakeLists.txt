cmake_minimum_required(VERSION 3.19.0)
project(InfoCom)

if(DEFINED INIT AND(INIT STREQUAL "Y"))
    message("Export compile commands.")
    set(INIT "N" CACHE STRING "" FORCE)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
else()
    message("Not export compile commands.")
    set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/include/)
link_directories(${CMAKE_SOURCE_DIR}/dll/)
set(VVS_LIBRARIES "-lsegment -lIO -lregistration -loctree")

if(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O0")
    message("Debug mode:${CMAKE_C_FLAGS_DEBUG}")
    set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/debug)
    file(MAKE_DIRECTORY debug)
    link_directories(${CMAKE_SOURCE_DIR}/debug/)
elseif(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Release"))
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3")
    set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/release)
    file(MAKE_DIRECTORY release)
    link_directories(${CMAKE_SOURCE_DIR}/release/)
    message("Release mode:${CMAKE_C_FLAGS_RELEASE}")
else()
    message("Default:Release")
    message("Relase mode:${CMAKE_C_FLAGS_RELEASE}")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3")
    set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/release)
    file(MAKE_DIRECTORY release)
    link_directories(${CMAKE_SOURCE_DIR}/release/)
endif()

message(${CMAKE_EXPORT_COMPILE_COMMANDS})

add_subdirectory(${CMAKE_SOURCE_DIR}/src)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

if(EXISTS "${CMAKE_SOURCE_DIR}/test/CMakeLists.txt")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)
    add_subdirectory(${CMAKE_SOURCE_DIR}/test)
    message("test available.")
endif()
