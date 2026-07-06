cmake_minimum_required(VERSION 3.16)

project(Parallel LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# scarica e compila raylib automaticamente
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.0
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(BoidSimulation
    main.cpp
    src/grid.cpp
)

target_link_libraries(BoidSimulation PRIVATE raylib)

target_include_directories(BoidSimulation PRIVATE
    ${CMAKE_SOURCE_DIR}/includes
)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(BoidSimulation PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Enable OpenMP only when the toolchain provides it.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(BoidSimulation PRIVATE OpenMP::OpenMP_CXX)
endif()
