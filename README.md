<div align="center">

# :soccer: INHA GoalKeeper
**Advanced Autonomous Agent for Humanoid Soccer**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-3490dc.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![BehaviorTree](https://img.shields.io/badge/BehaviorTree-V4-2ca02c.svg?style=for-the-badge)](https://www.behaviortree.dev/)
[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg?style=for-the-badge)](LICENSE)

*Dynamic Decision Making • Tactical Positioning • Human-like Agility*

---
</div>

Hi 👋 We are Inha-United !
Inha-United is a team competing in the RoboCup Soccer Humanoid League.

This repository focuses on algorithms and methodologies for goalkeeper behavior decision-making in autonomous humanoid soccer, based on the **BehaviorTree.CPP** framework.

Starting from the demo provided by Booster Robotics, we have modularized the system, extended its functionality, and continuously improved its performance through our own research and development efforts.

## Decision-Making Framework
The goalkeeper’s decision-making framework is composed of three high-level states: **Hold**, **Clearing**, and **Find**.

* **Hold**:
In this state, the robot predicts the ball trajectory and continuously computes and moves to the optimal position that minimizes the opponent’s shooting angle.

* **Clearing**:
When the ball enters a critical area, the goalkeeper performs a clearing action, kicking the ball away toward the direction opposite the goal.

* **Find**:
If the ball position is lost, the robot combines head rotation and body rotation to obtain an omnidirectional field of view and re-locate the ball efficiently.

The detailed decision-making process is illustrated in the figure below.
<img src="images/goalkeeper_bt.png" width="700" height="" />

## Installation & Run
```bash
mkdir  ~/INHA_GoalKeeper
git clone https://github.com/Inha-united-soccer/INHA_GoalKeeper.git

# Build 
./scripts/build.sh

# Start
./scripts/start.sh
