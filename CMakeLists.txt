cmake_minimum_required(VERSION 3.10)
project(ManaProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Enable debug symbols
set(CMAKE_BUILD_TYPE Debug)  # Or Release, RelWithDebInfo, MinSizeRel


# Specify the directories for headers
include_directories(${CMAKE_SOURCE_DIR}/src)

# Gather all source files
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)


# Set the main executable
add_executable(main.exe ${CMAKE_SOURCE_DIR}/main.cpp ${SRC_FILES})

# Set include directories for main and others
target_include_directories(main.exe PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Link OpenMP libraries to the main target
target_link_libraries(main.exe PRIVATE OpenMP::OpenMP_CXX)

# Set the output directory for the executable
set_target_properties(main.exe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Do the same for the CBS Analysis executable
add_executable(cbs_analysis.exe ${CMAKE_SOURCE_DIR}/tests/cbs_analysis.cpp ${SRC_FILES})

target_include_directories(cbs_analysis.exe PRIVATE ${CMAKE_SOURCE_DIR}/src)

#Link OpenMP libraries to the target
target_link_libraries(cbs_analysis.exe PRIVATE OpenMP::OpenMP_CXX)

set_target_properties(cbs_analysis.exe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
