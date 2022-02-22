cmake_minimum_required(VERSION 3.5)
#add_subdirectory(Src)
project(quad_sim_blender LANGUAGES C)

add_executable(quad_sim_blender Src/main.c Src/quadrotor.c Src/attitude_control2.c Src/attitude_control.c Src/blender_socket.c Src/kalman_backend.c Src/kalman_filter.c Src/mathlib.c Src/matrix_lib.c Src/pid.c Src/position_control.c Src/state_estimation.c Src/timer.c)

target_link_libraries(quad_sim_blender PUBLIC m)

target_include_directories(quad_sim_blender PUBLIC Inc/)
