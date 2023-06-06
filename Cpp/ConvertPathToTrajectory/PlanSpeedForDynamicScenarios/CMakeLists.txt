# created at 2022/4/20
# author: Lishengyong

cmake_minimum_required(VERSION 3.1)

project(Dynamic)
# because constexpr used, we need c++ 17
set (CMAKE_CXX_STANDARD 17)

# OpenCV required. 
# If your OpenCV is not installed in default position, then
# add your OpenCV path to "PATHS" to help cmake find it.
# The path is the directory of "OpenCVConfig.cmake"
find_package(OpenCV REQUIRED PATHS "/Users/lbaker/Projects/LatticeSearch/cpp code library/opencv_dist/lib/cmake/opencv4")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("statelatticecpp/")
include_directories("eigen-3.4.0")
include_directories("/opt/ibm/ILOG/CPLEX_Studio221/cplex/include")
include_directories("/opt/ibm/ILOG/CPLEX_Studio221/concert/include")

# set(statelatticecpp_files "statelatticecpp/.cpp")

add_library(plan STATIC PlanSpeedForDynamicScenarios.cpp)
link_directories("/opt/ibm/ILOG/CPLEX_Studio221/cplex/lib/x86-64_linux/static_pic")
link_directories("/opt/ibm/ILOG/CPLEX_Studio221/concert/lib/x86-64_linux/static_pic")
target_link_libraries(plan  pthread ilocplex concert cplex dl ${OpenCV_LIBS} )

# add all cpp files ...
add_executable(main 
    main.cpp
    GenerateDynamicObstacles_unstructured.cpp
    "statelatticecpp/Visualize.cpp"
    "statelatticecpp/vec2d.cpp"
    "statelatticecpp/state_lattice.cpp"
    "statelatticecpp/state_lattice_planner.cpp"
    "statelatticecpp/mathstruct.cpp"
    # ${statelatticecpp_files}
)

target_link_libraries(main 
    plan)

