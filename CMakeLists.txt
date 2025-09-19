cmake_minimum_required(VERSION 3.16)  
project(dascot LANGUAGES CXX)

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------
# Dependencies: nlohmann/json, boost graph
# -------------------------------
include(FetchContent)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(json)

# -------------------------------
# Source files
# -------------------------------
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(dascot ${SRC_FILES})

target_include_directories(dascot PRIVATE include)

target_link_libraries(dascot PRIVATE nlohmann_json::nlohmann_json)

# -------------------------------
# Tests
# -------------------------------
enable_testing()
# add_subdirectory(tests)
