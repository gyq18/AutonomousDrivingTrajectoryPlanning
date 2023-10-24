# LBADTP: Libraries and Benchmarks for Autonomous Driving Trajectory Planning


![](./results/result.png)

This repository contains the implementation of the above paper. It is accepted to IEEE TIV.
- Paper Link: 
- Authors: [Yuqing Guo](https://github.com/gyq18), [Zelin Guo](https://github.com/guozelin-ai), [Yazhou Wang](https://github.com/albertwang1), [Danya Yao](https://www.au.tsinghua.edu.cn/info/1076/1608.htm), [Bai Li](http://grjl.hnu.edu.cn/p/19232984984B4AF50942E7C9F74E071F), [Li Li](https://www.au.tsinghua.edu.cn/info/1096/1530.htm)
    - Contact Yuqing via email: gyq18@mails.tsinghua.org.cn)

## Introduction

LBADTP provides various autonomous driving trajectory planning test scenarios and planners with the following features:
- Test scenarios of varying complexity covering the full range of structured and unstructured autonomous driving applications.
- A complete implementation of state-of-art trajectory planners in Matlab, Python, and C++. 


## Benchmarks

### Scenarios
The test scenarios for autonomous driving trajectory planners are organized in the following structure:
```
scenarios/
    ├ structured scenarios/
        ├ Case1.csv
        ├ Case2.csv
        └ ...
    └ unstructured scenarios/
        ├ Case1.csv
        ├ Case2.csv
        └ ...
```
Each scenario contains the following information:
- The information for planning ranges and obstacles
- The vehicle geometrics and kinematics settings
- The initial and terminal vehicle state constraints

### Evaluation

- Runtime
- Score w.r.t. kinematic/dynamic feasibility, collision-free safety, trajectory smoothness, time-energy, etc.
   - For more information about the score criteria, please refer to the score criteria in  [TPAP Competition](https://www.ieee-itsc2022.org/program/tpap-competition)

## Libraries
The code implementation of state-of-art trajectory planners in Matlab/Python/C++ is organized in the following structure.
Here we take the Matlab language as an example.
```
Matlab/
    ├ Main.m
    ├ Planners/
        ├ Graphbased/
            ├ PlanAStarPath.m
            ├ PlanHybridAStarPath.m
            ├ PlanHybridAStarPath.m
            ├ PlanSimpleStateLatticePath.m
            ├ PlanControlLatticeTrajectory.m
            └ ...
        ├ Samplingbased/
            ├ PlanInformedRRTPath.m
            ├ PlanKinodynamicRRTTrajectory.m
            ├ PlanCLRRTTrajectory.m
            └ ...
        ├ Optimizationbased/
            ├ PlanOBCATrajectory.m
            ├ PlanSCPTrajectory.m
	    ├ PlanL1SCPTrajectory.m
	    ├ PlanL2SCPTrajectory.m
		├convex corridors for collision constraints/
		    ├FindCFS.m
		    ├FindBox.m
		    ├FindBubble.m
        └ ...
    ├ CheckCollision/
        ├ CheckByCircle.m
        ├ CheckByAABB.m
        ├ CheckByOBB.m
        └ CheckByArea.m
    ├ ConvertPathtToTrajectory/
        ├ PlanSpeedForStaicScenarios.m
        ├ PlanSpeedForDynamicScenarios.m
    └ ...
```
## Dependencies
- [CPLEX](https://www.ibm.com/analytics/cplex-optimizer)


## How to use

1. Clone this repo.

> git clone https://github.com/gyq18/AutonomousDrivingTrajectoryPlanning

2. Install the required libraries.

3. Execute main.m/main.py/main.cpp script in each directory. You can replace the planner in the main.m/main.py/main.cpp script.

4. Add star to this repo if you like it :smiley:. 


## Citation

If you find our code or paper useful, please consider citing
```
@article{guo2023XXX,
  title={XXXXX},
  author={XXXX},
  journal={XXXXX},
  year={XXXXX},
  publisher={XXXXX}
}
```

## Contributors
- [Yuqing Guo](https://github.com/gyq18), [Zelin Guo](https://github.com/guozelin-ai), [Yazhou Wang](https://github.com/albertwang1), [Danya Yao](https://www.au.tsinghua.edu.cn/info/1076/1608.htm), and [Li Li](https://www.au.tsinghua.edu.cn/info/1096/1530.htm) are with the Department of Automation, Tsinghua University, Beijing 100084, China.
- [Bai Li](https://github.com/libai1943) and [Yakun Ouyang](https://github.com/yakunouyang) with the College of Mechanical and Vehicle Engineering, Hunan
University, Changsha 410082, China.

## Acknowledgement

Special thanks to [Matthias Althoff](https://www.in.tum.de/i06/people/prof-dr-ing-matthias-althoff/) and [Kristoffer Bergman](http://users.isy.liu.se/rt/kribe48/) for their help.

## Reference code

- [MoveIt!](https://planners-benchmarking.readthedocs.io/en/latest/user_guide/2_motion_planners.html)
- [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
- [Stochastic Trajectory Optimization for Motion Planning (STOMP) ](http://wiki.ros.org/stomp_motion_planner)
- [Covariant Hamiltonian Optimization for Motion Planning (CHOMP)](https://www.ri.cmu.edu/pub_files/2009/5/icra09-chomp.pdf)
- [Search-Based Planning Library (SBPL)](http://wiki.ros.org/sbpl)
- [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
- [TUMcps/CORA](https://github.com/TUMcps/CORA)
- [changliuliu/CFS](https://github.com/changliuliu/CFS)
- [XiaojingGeorgeZhang/H-OBCA](https://github.com/XiaojingGeorgeZhang/H-OBCA)
- [libai1943/CartesianPlanner](https://github.com/libai1943/CartesianPlanner)
- [libai2020/On_Road_Single_Vehicle_Trajectory_Decision](https://github.com/libai2020/On_Road_Single_Vehicle_Trajectory_Decision)
- [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
- [Mesywang/Corridor-based and BezierCurve-based Trajectory](https://github.com/Mesywang/Motion-Planning-Algorithms/tree/master/HardConstraintTrajectoryOptimization)

