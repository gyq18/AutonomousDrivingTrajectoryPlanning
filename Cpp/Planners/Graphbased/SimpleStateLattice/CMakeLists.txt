# created at 2022/4/20
# author: Lishengyong

cmake_minimum_required(VERSION 3.1)

project(StateLatticePlannerCpp)
# because constexpr used, we need c++ 17
set (CMAKE_CXX_STANDARD 17)


# OpenCV required. 
# If your OpenCV is not installed in default position, then
# add your OpenCV path to "PATHS" to help cmake find it.
# The path is the directory of "OpenCVConfig.cmake"
find_package(OpenCV REQUIRED PATHS opencv_dist/lib/cmake/opencv4)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(vis STATIC 
    vis_api.cpp
    vec2d.cpp

)

target_link_libraries(vis -dynamic-libstdc++)

# add all cpp files ...
add_executable(main 
    main.cpp
    iofromha.cpp
    mathstruct.cpp
    TransformP2T.cpp
    vec2d.cpp
    Visualize.cpp
    state_lattice.cpp
    state_lattice_planner.cpp
)

target_link_libraries(vis
    ${OpenCV_LIBS}

    # c++ 
    )


target_link_libraries(main
    ${OpenCV_LIBS})

