
# Introduction
This is an library which implements planning speed for dynamic scenario.
For the path searching, the lib use state lattice method.


# Requirement
C++ compiler with support of c++17.

OpenCV > 4
nlohmann/json https://github.com/nlohmann/json
Eigen3 https://gitlab.com/libeigen/eigen/

# Test
We use CMake to build the project. Edit **main.cpp** to configure the test program.

1. make a build directory.
```bash
mkdir build
cd build 
```

2. build the project.
```bash 
cmake ..
make 
```

if you want to make a release build, use CMAKE_BUILD_TYPE flag.
```bash 
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
```

3. run the program. 
```bash
./main
```

! do make sure the **insert_points.txt** and **look_up_table_153244.txt** have the correct relative path to the excutable program.
