cmake_minimum_required(VERSION 3.21.0)

project(RTWeekend LANGUAGES CXX)

set( CMAKE_CXX_STANDARD 23 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Sources

set ( 
    EXTERNAL 
        src/external/stb_image.h
)

set (
    SOURCE_ONE_WEEKEND
        src/InOneWeekend/main.cc
        src/InOneWeekend/camera.h
        src/InOneWeekend/color.h
        src/InOneWeekend/hittable.h
        src/InOneWeekend/hittable_list.h
        src/InOneWeekend/interval.h
        src/InOneWeekend/material.h
        src/InOneWeekend/ray.h
        src/InOneWeekend/rtweekend.h
        src/InOneWeekend/sphere.h
        src/InOneWeekend/vec3.h
)

include_directories(src)

message (STATUS "Compiler ID: " ${CMAKE_CXX_COMPILER_ID})
message (STATUS "Release flags: " ${CMAKE_CXX_FLAGS_RELEASE})
message (STATUS "Debug flags: " ${CMAKE_CXX_FLAGS_DEBUG})

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wnon-virtual-dtor) # Class has virtual functions, but its destructor is not virtual
    add_compile_options(-Wreorder) # Data member will be initialized after [other] data member
    add_compile_options(-Wmaybe-uninitialized) # Variable improperly initialized
    add_compile_options(-Wunused-variable) # Variable is defined but unused
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wnon-virtual-dtor) # Class has virtual functions, but its destructor is not virtual
    add_compile_options(-Wreorder) # Data member will be initialized after [other] data member
    add_compile_options(-Wsometimes-uninitialized) # Variable improperly initialized
    add_compile_options(-Wunused-variable) # Variable is defined but unused
endif()

add_executable(inOneWeekend ${EXTERNAL} ${SOURCE_ONE_WEEKEND})
find_package(TBB REQUIRED)
target_link_libraries(inOneWeekend PRIVATE TBB::tbb)