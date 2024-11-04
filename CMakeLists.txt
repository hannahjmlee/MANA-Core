cmake_minimum_required(VERSION 3.10)
project(ManaProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Release)  # Debug, Release, RelWithDebInfo, MinSizeRel

# Specify the directories for headers
include_directories(${CMAKE_SOURCE_DIR}/src)

# Gather all source files
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# Set the constraint analysis executable
add_executable(constraint_analysis.exe ${CMAKE_SOURCE_DIR}/tests/constraint_analysis.cpp ${SRC_FILES})

# Set include directories for constraint_analysis and others
target_include_directories(constraint_analysis.exe PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Set the output directory for the executable
set_target_properties(constraint_analysis.exe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
