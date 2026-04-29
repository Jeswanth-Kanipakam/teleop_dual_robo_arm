# Dual-Arm Vision-Based Teleoperation System

This project implements a ROS 2-based teleoperation platform that enables a single human operator to control a pair of dual 6DOF Piper arms and two-finger grippers in a visualization environment. By leveraging real-time computer vision and high-speed motion planning, the system mirrors the operator's physical movements with millisecond-level latency.

## 🚀 Key Features
* **Dual Arm Control**: Independent control of two 6DOF arms using MediaPipe hand classification (Left vs. Right).
* **Swift & Smooth Motion**: Integrated with **MoveIt Servo** for low-latency Cartesian-to-Joint conversion, reaching target values within milliseconds.
* **Gesture-Based Gripping**: Maps physical hand aperture to dual-finger gripper actions via Action Clients.
* **Safe Operation**: Built-in self-collision avoidance for dual arms and smoothing filters to handle noisy camera data.
* **2D Constraint Compliance**: Optimized for standard webcams by constraining movement to a specific $Y-Z$ plane with fixed depth ($X = 0.4$).

## 🛠️ Prerequisites & Installation

### 1. System Requirements
* **OS**: Ubuntu (Optimized for ROS 2 Jazzy)
* **ROS 2**: Desktop Installation
* **Hardware**: Standard Webcam

### 2. Install ROS 2 Dependencies
Ensure the following ROS 2 packages are installed for motion planning and hardware abstraction:
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-moveit-servo \
                 ros-${ROS_DISTRO}-v4l2-camera \
                 ros-${ROS_DISTRO}-joint-state-broadcaster \
                 ros-${ROS_DISTRO}-joint-trajectory-controller \
                 ros-${ROS_DISTRO}-gripper-controllers
```

### 3. Install Python Libraries
These libraries power the vision-tracking engine:
```bash
# Note: On Ubuntu 24.04+, use --break-system-packages if not in a venv
pip install opencv-python mediapipe numpy
```

## 📂 Setup and Build
```bash
# Create workspace
mkdir -p ~/teleop_ws/src
cd ~/teleop_ws/src

# Clone this repository and ensure teleop_description is present
# git clone https://github.com/Jeswanth-Kanipakam/teleop_dual_robo_arm

# Build the workspace
cd ~/teleop_ws
colcon build --symlink-install
source install/setup.bash
```

## 🎮 How to Run
To launch the full integrated system (RViz Visualization + Controllers + Vision Node):
```bash
ros2 launch teleop_vision integrated_teleop.launch.py
```
* **RViz** will open showing the dual Piper arms.
* The **Webcam** feed will start. Bring your hands into view to begin control.

## 📊 Message Publishing
As per the assignment requirements, the system publishes live joint data:
* **Current Joint States**: Published to `/joint_states` by the broadcaster.
* **Teleop Data**: Target joint data is published to `/teleop_joint_angles`.

## 📚 Open-Source Acknowledgments & Implementation

This project utilizes the following open-source resources:

1.  **MediaPipe (Google)**:
    * **Implementation**: Used in `HandDetectorModule.py` to identify 21 hand landmarks and classify handedness.
    * **Analysis**: I extracted the `label` (Left/Right) to route commands and used the Euclidean distance between the thumb and index finger to calculate gripper aperture.
2.  **MoveIt 2 & MoveIt Servo**:
    * **Implementation**: Configured as the real-time bridge via `servo_config.yaml`.
    * **Analysis**: To meet the "millisecond" requirement, I bypassed standard MoveGroup planning in favor of MoveIt Servo, which converts Cartesian poses into smooth joint trajectories instantly.
3.  **ROS 2 Control**:
    * **Implementation**: Defined in `ros2_controllers.yaml` and `dual_piper.ros2_control.xacro`.
    * **Analysis**: Used `mock_components/GenericSystem` for safe visualization without physical hardware.

## 👤 Author
**Jeswanth Kanipakam** 
M.Sc. Autonomy Technologies  
