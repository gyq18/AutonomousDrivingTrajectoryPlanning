<!--
 * @Author: Shengyong Li
 * @Date: 2022-04-23 11:34:30
 * @LastEditTime: 2022-04-23 20:10:03
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of State Lattice Planning
-->


# Introduction
This is an implementation of State Lattice Planning method.


# Requirement
C++ compiler with support of c++17.

OpenCV > 4

nlohmann/json https://github.com/nlohmann/json

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