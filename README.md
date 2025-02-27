# RT1-Assignment2-ROS

## Project Description

This project includes two main ROS nodes to manage a mobile robot's goal control and provide a service that returns the last set goal coordinates. The system allows the user to send a goal to the robot, monitor its status, and dynamically manage goal execution.

For additional documentation: https://mattiatinfena.github.io/RT1-Assignment2-ROS/

### Main Features:
1. **Action Client Node**:  
   - Allows the user to set target coordinates and send them to an action server.  
   - Manages feedback on the robot's progress towards the goal.  
   - Publishes the robot's current position on the `/robot_status` topic.  

2. **Service Node**:  
   - Provides a custom ROS service that returns the last set goal coordinates through the `/target_service` service.  

---

## Features

### **Action Client Node (`actionClientNode`)**

- **Goal Management**:  
  Allows users to input the destination coordinates (`x`, `y`) for the robot's goal.  
  Sends the goal to the action server (`/reaching_goal`) and monitors the goal status (active, succeeded, or aborted).  
  Users can:
  - Cancel a goal.  
  - Request real-time feedback on the goal progress.  
  - Exit the program at any time.

- **Feedback Handling**:  
  Periodically receives feedback from the action server regarding the robot's progress towards the goal.

- **Robot Status Publisher**:  
  Subscribes to the `/odom` topic to receive odometry data and publishes the robot's position and velocity on the `/robot_status` topic using a custom message (`RobotInfo`).

---

### **Service Node (`serviceNode`)**

- **Goal Query Service**:  
  Implements a custom service (`/target_service`) that returns the last set goal coordinates (`x`, `y`).  
  Uses ROS parameters (`/des_pos_x` and `/des_pos_y`) to retrieve the last goal sent by the user.

---

## Prerequisites

1. **ROS (Noetic)** installed.  
2. **Python 2.7** for the action client node and **Python 3.x** for the service node.  
3. Clone the repo https://github.com/CarmineD8/assignment_2_2024 for the server

---

## Installation

Clone the repository into your ROS workspace:

```bash
git clone https://github.com/MattiaTinfena/RT1-Assignment2-ROS.git
```

Compile the package:

```bash
catkin_make
```
---

## Execution

Start the simulation:

```bash
roslaunch assignment2_ros1 assignment2.launch
```
## Usage

- The user can choose the target point
- While the robot is moving the user can as for feedback (position or velocity) or quit the simulation

