# Unitree Quadruped Robot Control System

This ROS catkin workspace contains a comprehensive control system for Unitree quadruped robots, including the Go1 and A1 models. The workspace provides simulation capabilities, real robot control, navigation, and motion planning functionality.

## Overview

The project is organized around three main ROS packages that work together to provide a complete robotics control framework:

- **unitree_guide**: Core control algorithms and robot state management
- **unitree_legged_msgs**: ROS message definitions for robot communication
- **unitree_ros**: Robot descriptions, gazebo simulation, and hardware interfaces

## Package Structure

### 1. unitree_guide
**Path**: `src/unitree_guide/`

This is the main control package containing the core algorithms for quadruped robot control. It accompanies the book "四足机器人控制算法--建模、控制与实践" (Quadruped Robot Control Algorithms - Modeling, Control and Practice) published by Unitree Robotics.

**Key Components:**
- **unitree_guide/**: Main control framework with balance control, gait planning, and robot state management
- **unitree_actuator_sdk/**: Motor control SDK for direct actuator communication
- **unitree_move_base/**: Navigation and path planning integration with ROS move_base

**Main Features:**
- Balance control algorithms (`src/control/BalanceCtrl.cpp`)
- Robot kinematic models (`src/common/unitreeRobot.cpp`, `src/common/unitreeLeg.cpp`)
- Control framework (`src/control/ControlFrame.cpp`)
- Low-pass filtering (`src/common/LowPassFilter.cpp`)
- Gazebo simulation integration (`launch/gazeboSim.launch`)

**Configuration Options** (CMakeLists.txt):
- `ROBOT_TYPE`: Go1 or A1 robot support
- `PLATFORM`: amd64 or arm64 architecture
- `SIMULATION`: Enable/disable Gazebo simulation
- `REAL_ROBOT`: Enable/disable real robot hardware interface
- `MOVE_BASE`: Enable/disable navigation capabilities

### 2. unitree_legged_msgs
**Path**: `src/unitree_legged_msgs/`

ROS message package defining communication protocols for Unitree quadruped robots.

**Message Types:**
- `HighCmd.msg` / `HighState.msg`: High-level robot commands and state (gait, velocity, body pose)
- `LowCmd.msg` / `LowState.msg`: Low-level motor commands and feedback
- `MotorCmd.msg` / `MotorState.msg`: Individual motor control and status
- `IMU.msg`: Inertial measurement unit data
- `BmsCmd.msg` / `BmsState.msg`: Battery management system interface
- `LED.msg`: LED control for robot status indication
- `Cartesian.msg`: Cartesian coordinate definitions

**Key Features:**
- Comprehensive robot state representation
- Motor-level control granularity
- Battery and system health monitoring
- Wireless remote control integration

### 3. unitree_ros
**Path**: `src/unitree_ros/`

Complete ROS integration package providing robot descriptions, simulation environments, and hardware interfaces.

**Sub-packages:**
- **Robot Descriptions**: URDF models for multiple Unitree robots
  - `a1_description`: A1 robot model and visualization
  - `aliengo_description`: Aliengo robot model
  - `aliengoZ1_description`: AliengoZ1 variant
  - `b1_description`: B1 robot model
  - `b2_description`: B2 robot model
  - `b2w_description`: B2W variant

- **Control Systems**:
  - `unitree_controller`: Robot joint controllers and hardware interfaces
  - `unitree_legged_control`: Advanced locomotion control algorithms

- **Simulation**:
  - `unitree_gazebo`: Gazebo simulation worlds and robot models
  - `unitree_ros_to_real`: Bridge between simulation and real robot hardware

## Build Configuration

The system supports multiple build configurations through CMake variables:

### Robot Types
- **Go1**: Latest Unitree robot with enhanced capabilities
- **A1**: Earlier model with proven stability

### Platforms
- **amd64**: Standard x86-64 development platforms
- **arm64**: Embedded ARM platforms (robot onboard computers)

### Operation Modes
- **Simulation Mode**: Uses Gazebo physics simulation
- **Real Robot Mode**: Connects to actual hardware via Unitree SDK
- **Move Base Mode**: Enables ROS navigation stack integration

## Dependencies

**ROS Packages:**
- `controller_manager`: Joint controller management
- `joint_state_controller`: Joint state publishing
- `robot_state_publisher`: Robot TF tree publishing
- `gazebo_ros`: Gazebo simulation integration
- `move_base` (optional): Navigation stack

**External Libraries:**
- **Unitree Legged SDK**: Hardware communication library
  - SDK 3.2 for A1 robots
  - SDK 3.8.0 for Go1 robots
- **LCM (Lightweight Communications and Marshalling)**: Inter-process communication
- **Boost**: C++ utilities and threading

## Launch Files

### Primary Launch Files
- `unitree_guide/launch/gazeboSim.launch`: Complete Gazebo simulation with robot spawning and controllers
- `unitree_move_base/launch/move_base.launch`: Navigation stack integration
- `unitree_move_base/launch/rvizMoveBase.launch`: RViz visualization for navigation

### Robot-Specific Launch Files
- `robots/*/launch/*_rviz.launch`: Robot visualization in RViz
- `robots/*/launch/*_gazebo.launch`: Robot-specific Gazebo simulation

## Key Executables

- **junior_ctrl**: Main control executable (`unitree_guide/src/main.cpp`)
  - Implements core control loops
  - Manages robot state and motion planning
  - Handles communication with hardware/simulation

## Message Flow Architecture

```
High-Level Commands (HighCmd) → Control Framework → Low-Level Commands (LowCmd) → Motors
                                       ↓
Real Robot Hardware ← Unitree SDK ← Motor States (LowState) ← Joint Controllers
       ↓
Robot State (HighState) → ROS Topics → Navigation/Planning Systems
```

## Usage Examples

### Simulation Setup
```bash
# Launch Gazebo simulation with Go1 robot
roslaunch unitree_guide gazeboSim.launch rname:=go1 wname:=earth

# Start the main control system
rosrun unitree_guide junior_ctrl
```

### Real Robot Control
1. Set `REAL_ROBOT=ON` and `SIMULATION=OFF` in CMakeLists.txt
2. Build the workspace: `catkin_make`
3. Connect to robot's onboard computer
4. Launch control system: `rosrun unitree_guide junior_ctrl`

### Navigation Integration
```bash
# Launch robot with navigation capabilities
roslaunch unitree_move_base move_base.launch

# Visualize in RViz
roslaunch unitree_move_base rvizMoveBase.launch
```

## Development Notes

### SDK Integration
The system integrates different Unitree SDK versions based on robot type:
- **A1 robots**: Uses SDK 3.2 with platform-specific libraries
- **Go1 robots**: Uses SDK 3.8.0 with static linking

### Threading and Communication
- Uses LCM for low-latency robot communication
- Implements pthread-based concurrent control loops
- ROS integration for high-level planning and visualization

### Debugging Support
- Optional Python integration for debugging and data analysis
- Compile-time debug flags for development builds
- Comprehensive logging and state monitoring

## Safety Considerations

- The system includes safety checks for conflicting build configurations
- Robot type validation prevents incompatible SDK linking
- Hardware/simulation mode exclusivity prevents unsafe operation
- Emergency stop capabilities through high-level command interface

This workspace provides a complete development and deployment environment for Unitree quadruped robots, supporting both research and practical applications in robotics locomotion and navigation.