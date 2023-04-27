set(LIB_NAME "simplemove")
add_library(${LIB_NAME} SHARED robot_trajectory.cpp 
    time_optimal_trajectory_generation.cpp)