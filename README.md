# Assignment_RT2
First Assignment of Research Track 2:

This project implements a robot navigation system in ROS2 using **Actions, TF2 and modular nodes**.  
The robot can be commanded to reach a target pose *(x, y, theta)*, providing real-time feedback during execution.

---
## Overview
The system allows:
- sending navigation goals to the robot  
- executing them asynchronously  
- receiving real-time feedback about position and errors  

The architecture follows the **2 components + 1 standalone node** paradigm:

- **action_server**: responsible for robot motion. It executes navigation goals by publishing velocity commands on `/cmd_vel`. It uses **tf2** to compute the relative transformation between the robot and the goal, acting both as a TF broadcaster (from odometry) and a TF listener.

- **action_client**: acts as an intermediary between the user interface and the server. It receives target poses via the `/ui_goal` topic, sends them to the action server, and handles feedback and result callbacks. It also publishes goal status updates.

- **user_interface**: It is a node that allows the user to input target poses and cancel goals via keyboard. It communicates with the action client through topics.

---
##  Project Structure

The package `assignment1` is organized as follows:
```txt
assignment1/
├── action/
│   └── NavigateToPose.action
├── include/
│   ├── action_client.hpp
│   └── action_server.hpp
├── launch/
│   └── launch.py
├── msg/
│   └── UiGoal.msg
├── src/
│   ├── action_client.cpp
│   ├── action_server.cpp
│   └── user_interface.cpp
├── CMakeLists.txt
└── package.xml
```

## Custom Interfaces  

### Action — `NavigateToPose.action`  

```txt
# Goal
float64 x
float64 y
float64 theta
---
# Result
bool success
string message
---
# Feedback
float64 current_x
float64 current_y
float64 current_theta
float64 distance_error
float64 angle_error
```

### Message — UiGoal.msg

Used for communication between user interface and action client:
```txt
float64 x
float64 y
float64 theta
```
---

### The header files — `include/assignment1/`  

`action_client.hpp` and `action_server.hpp` declare the two classes `ActionClient` and `ActionServer`, including their methods and members. This separation allows `user_interface.cpp` to interact with the nodes through their interfaces without requiring the full implementation.

---

## Source Files — `src/`

### 1) action_client.cpp — ActionClient

This node handles communication between the user interface and the action server.

**It:**
- subscribes to `/ui_goal` to receive target poses  
- subscribes to `/ui_cancel` to receive cancel requests  
- publishes goal status on `/goal_status`  
- creates a static TF transform (`odom → goal_frame`) representing the target pose  
- sends the goal asynchronously to the action server  

**Callbacks:**
- `goal_response_callback` — logs whether the goal is accepted or rejected  
- `feedback_callback` — prints real-time navigation feedback  
- `result_callback` — handles final result and publishes goal status (`SUCCEEDED`, `CANCELLED`, `ABORTED`)  

---

### 2) action_server.cpp — ActionServer

This node implements the navigation logic of the robot.

**It:**
- subscribes to `/odom` and broadcasts the robot pose as a TF transform  
- publishes velocity commands on `/cmd_vel`  
- uses a TF2 listener and buffer to compute the transform between robot and goal  

**Execution logic:**
- continuously computes the transform `base_footprint → goal_frame`  
- calculates distance and angle error  
- moves the robot proportionally to the error  
- aligns the robot orientation at the end  
- stops when the goal is reached within a tolerance  

**Callbacks:**
- `handle_goal` — accepts incoming goals  
- `handle_cancel` — handles cancellation requests  
- `execute` — runs in a separate thread and performs navigation  

---

### 3) user_interface.cpp — User Interface

This node provides a command-line interface for user interaction.

**It:**
- allows the user to input target `(x, y, theta)`  
- publishes goals on `/ui_goal`  
- publishes cancel requests on `/ui_cancel`  
- subscribes to `/goal_status` to monitor execution  

**Features:**
- supports real-time cancellation using `c + ENTER`  
- uses multithreading and `select()` for non-blocking input  
- uses pipes to safely interrupt input threads  
- allows sequential goal execution

---
### **TF2 FRAME STRUCTURE**

The navigation relies on three main frames:

| Frame            | Publisher     | Description             |
|------------------|--------------|--------------------------|
| `odom`           | Fixed        | Global reference frame   |
| `base_footprint` | ActionServer | Robot position           |
| `goal_frame`     | ActionClient | Target pose              |

The server continuously computes the transform:

```txt
base_footprint → goal_frame
```
When the distance error is below the threshold, the goal is considered reached. 

**HOW TO RUN**

### 1. First build the workspace

```bash
colcon build
source install/setup.bash
```

The system is started by a single launch file that brings up three elements:
```bash
ros2 launch assignment1 launch.py
```

### 1) Simulation Environment
→ Includes the `bme_gazebo_sensors` package launch, runs the robot simulation and publishes odometry on `/odom`

### 2) Components (Action Server + Client)
→ handle navigation logic and communication using ROS 2 actions and TF2

### 3) User Interface
→ allows the user to input target poses (x, y, theta) and cancel goals interactively

---

It is suggested on Rviz window to add the TF frames and switch from fixed frame to odom. In this way it is possible to see the robot's motion from the point of view ofr the world odom. 

---
### Author
Andrea Laura --- andrelaura1.al@gmail.com
