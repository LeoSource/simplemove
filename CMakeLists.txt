cmake_minimum_required(VERSION 3.0.0)
project(simplemove VERSION 0.1.0)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)

add_subdirectory(examples)
# set(EXE_NAME "simplemove_demo")
# link_directories(${LIBRARY_OUTPUT_PATH})
# add_executable(${EXE_NAME} examples/main.cpp)
# target_link_libraries(${EXE_NAME} simplemove)