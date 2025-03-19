# ROS2/PX4 Drone Simulation and Autonomous Navigation

## 📌 Overview

This project integrates **ROS 2** with **PX4** to simulate and control a drone in **Gazebo**. It provides two primary functionalities:

1. **Parametric Mission Node**: Enables autonomous navigation missions with configurable parameters (destination, altitude, velocity, etc.).
2. **Keyboard Control Node**: Allows manual control of the drone via keyboard commands.

Additionally, the project includes real-time telemetry monitoring via ROS 2 topics.

---

## 🚀 1. Setting Up the ROS2/PX4 Simulation

### **Prerequisites**

Ensure the following are installed:

- [ROS 2 (Humble)](https://docs.px4.io/main/en/ros2/user_guide.html#install-ros-2)
- [PX4 Autopilot](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4)
this project was built on WSL on ubuntu 22.04 for more instructions follow this [docs](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html#opening-a-wsl-shell)

### **Running the Simulation**

To launch the PX4 simulation with Gazebo:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This starts the PX4 simulation with Gazebo.

in another terminal to check if MicroXRCEAgent is installed, run: 
```bash
MicroXRCEAgent udp4 -p 8888
```
#### **Setup issues**
px4_msg building errors were solved by installing earlier version of stuptools
```bash
pip install --user setuptools==56.0.0
```
If something is stopping you from getting out of pre flight mode, attempt to setup [QGROUNDCONTROL](https://docs.px4.io/main/en/advanced_config/ethernet_setup.html#qgroundcontrol-setup-example):

---

## ✈️ 2. Parametric Mission Node

This ROS 2 node allows configuring autonomous missions using **ROS parameters**.

### **Functionality**

- **Takeoff** to a specified altitude.
- Navigate to a **user-defined destination**.
- **Land** after reaching the destination.
- Configurable parameters via **launch file**

### **Parameters**

These parameters are defined in `parametric_mission.launch.py`:

- `altitude`: Target flight altitude
- `velocity`: Horizontal flight speed
- `destination`: [X, Y] target coordinates
- `position_tolerance`: Distance threshold for reaching the destination
- `max_acceleration`: Maximum acceleration allowed
- `max_yaw_angle`: Maximum yaw rotation in degrees
- `climb_rate`: Rate of ascent
- `descent_rate`: Rate of descent
- `pid_p`, `pid_i`, `pid_d`: PID controller parameters

### **Running the Node**

You can launch it via a launch file:

```bash
ros2 launch px4_ros_com parametric_mission.launch.py
```

Or via CLI by setting parameters:

```bash
ros2 run px4_ros_com parametric_mission.py --ros-args -p altitude:=-10.0 -p velocity:=3.0 -p destination:="[20.0, 15.0]"
```

---

## ⌨️ 3. Keyboard Control Node

This node enables manual control of the drone via keyboard commands.

### **Controls**

| Key     | Action            |
| ------- | ----------------- |
| `W`     | Move forward      |
| `S`     | Move backward     |
| `A`     | Move left         |
| `D`     | Move right        |
| `Q`     | Increase altitude |
| `E`     | Decrease altitude |
| `T`     | Takeoff           |
| `G`     | Land              |
| `Space` | Emergency stop    |

### **Running the Node**

```bash
ros2 run px4_ros_com keyboard_drone_control.py
```
---

## 📡 4. Real-Time Telemetry Monitoring

To monitor the drone's position (X, Y, Z) in real-time:

```bash
ros2 topic echo /fmu/out/vehicle_local_position
```
This continuously displays position updates in the terminal.
Telemetry can also be checkd with:
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
and
```bash
ros2 launch px4_ros_com vehicle_gps_position_listener
```
---

## 📜 5. Summary of ROS 2 Topics Used

| Topic                             | Purpose                                    |
| --------------------------------- | ------------------------------------------ |
| `/fmu/in/trajectory_setpoint`     | Sends position setpoints to the drone      |
| `/fmu/in/vehicle_command`         | Sends takeoff, land, and offboard commands |
| `/fmu/out/vehicle_local_position` | Outputs the drone's real-time position     |

---
