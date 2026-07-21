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

find_package(OpenMP)

add_library(simulation_core
    src/grid.cpp
    src/simulation.cpp
)

target_include_directories(simulation_core PUBLIC
    ${CMAKE_SOURCE_DIR}/includes
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(simulation_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(BoidSimulation
    main.cpp
)

target_link_libraries(BoidSimulation PRIVATE simulation_core raylib)

add_executable(benchmark
    benchmark.cpp
)

target_link_libraries(benchmark PRIVATE simulation_core)
set_target_properties(benchmark PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(simulation_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(BoidSimulation PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(benchmark PRIVATE -Wall -Wextra -Wpedantic)
endif()
