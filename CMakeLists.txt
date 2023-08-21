cmake_minimum_required(VERSION 3.16)
project(NarrowPassage)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER g++)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(SFML 2.5.1 COMPONENTS system window graphics network audio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Add source files
file(GLOB_RECURSE SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} Threads::Threads yaml-cpp sfml-graphics sfml-window sfml-system)
