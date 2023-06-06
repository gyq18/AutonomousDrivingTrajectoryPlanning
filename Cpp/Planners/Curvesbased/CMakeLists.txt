cmake_minimum_required(VERSION 3.0.0)
project(PathPlanning VERSION 0.1.0)


FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(".\\include\\")
include_directories(".\\3rdParty\\Mosek\\9.3\\tools\\platform\\win64x86\\h\\")
link_directories(".\\3rdParty\\Mosek\\9.3\\tools\\platform\\win64x86\\bin")
set(SRC_LISTS                   
    ./src/bezier.cpp
    ./src/mathstruct.cpp
    ./src/PlanPRMPath.cpp
    ./src/TransformP2T.cpp
    ./src/vec2d.cpp
    ./src/Visualize.cpp
    ./main.cpp
)
add_executable(PathPlanning ${SRC_LISTS})

TARGET_LINK_LIBRARIES (PathPlanning Eigen3::Eigen ${OpenCV_LIBS} fusion64_9_3.lib mosek64_9_3.lib)  


