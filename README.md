# 🧭 EKF-SLAM Based Mobile Robot Navigation (ROS 2)

This repository contains an implementation of **Extended Kalman Filter (EKF) based Simultaneous Localization and Mapping (SLAM)** for a differential-drive mobile robot simulated in **Gazebo** using **ROS 2**.

The robot:

* Localizes itself using noisy odometry and landmark observations
* Estimates landmark positions simultaneously (landmark-based SLAM)
* Navigates autonomously through predefined waypoints using the EKF pose
* Visualizes pose, trajectory, and landmarks in RViz
* Logs data for post-processing and analysis

---

## 📌 Features

* EKF-SLAM with joint robot + landmark state
* Range–bearing landmark sensor model
* Odometry-based prediction and landmark-based correction
* TF-based global consistency (`map → odom`)
* Autonomous waypoint navigation
* RViz visualization of:

  * EKF pose
  * Robot trajectory
  * Estimated landmarks
* CSV logging and result plots

---

## 🛠️ Tech Stack

* ROS 2 (Humble)
* Gazebo
* Python
* NumPy
* Matplotlib
* Pandas
* RViz2

---

## 📂 Repository Structure

````text
EKF-SLAM/
├── src/
│   ├── duckie_bot_description/      # URDF, Gazebo world, launch files
│   ├── duckie_bot_slam/             # ROS 2 Python package
│   │   └── duckie_bot_slam/
│   │       └── ekf_slam/
│   │           ├── core_logic.py            # EKF-SLAM core logic
│   │           ├── ekf_slam_node.py         # EKF-SLAM ROS node
│   │           ├── landmark_sensor_node.py  # Simulated landmark sensor
│   │           ├── waypoint_controller.py   # Waypoint navigation
│   │           └── scripts/
│   │               └── generate_results.py  # Result plotting & analysis
│   └── landmark_msgs/               # Custom landmark message package
│       └── LandmarkObs.msg
└── README.md
````

---

## ⚙️ Dependencies

Install required system and Python dependencies:

```bash
sudo apt update
sudo apt install python3-pandas python3-matplotlib
```

Ensure **ROS 2 Humble** is installed and sourced.

---

## 🚀 Build Instructions

```bash
cd ~/duckie_bot_ws
colcon build
source install/setup.bash
```

---

## ▶️ Running the Project (Step-by-Step)

### **Terminal 1 — Launch Gazebo and Robot**

```bash
ros2 launch duckie_bot_description gazebo.launch.py
```

---

### **Terminal 2 — Publish Static Transform (`map → odom`)**

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

---

### **Terminal 3 — Start RViz**

```bash
rviz2
```

Set **Fixed Frame** to `map`.

---

### **Terminal 4 — Start Landmark Sensor**

```bash
ros2 run duckie_bot_slam landmark_sensor_node
```

---

### **Terminal 5 — Start EKF-SLAM Node**

```bash
ros2 run duckie_bot_slam ekf_slam_node --ros-args -p use_sim_time:=true
```

This node:

* Predicts pose using odometry
* Corrects pose using landmark observations
* Publishes `/ekf_pose`, `/ekf_path`, and landmark markers

When stopped, landmark estimation errors are printed:

```text
--- LANDMARK ESTIMATION ERRORS ---
Landmark 0: 5.7843 meters
Landmark 1: 19.8532 meters
Landmark 2: 7.0807 meters
Landmark 3: 15.6109 meters
Landmark 4: 10.6590 meters
```

---

### **Terminal 6 — Run Waypoint Controller**

```bash
ros2 run duckie_bot_slam waypoint_controller_node
```

The robot navigates through predefined goals:

```text
G1 = (2, 2)
G2 = (8, 4)
G3 = (5, 7)
```

Example output:

```text
Reached goal 1: (2.0, 2.0)
Reached goal 2: (8.0, 4.0)
Reached goal 3: (5.0, 7.0)
All goals reached.
```

---

## 📈 Result Analysis

After the simulation:

```bash
python3 src/ekf_slam/scripts/generate_results.py
```

This script:

* Plots EKF vs odometry trajectory
* Visualizes control signals
* Helps analyze SLAM performance

---

## 🧠 EKF-SLAM Overview (Conceptual)

* Odometry is used to **predict** robot motion
* Landmark observations are used to **correct** drift
* Robot pose and landmark positions are estimated jointly
* Uncertainty determines how much correction is applied

The mathematical formulation is documented separately in the project report.

---

## ⚠️ Limitations

* Assumes Gaussian noise
* EKF linearization can introduce errors for large uncertainty
* Landmark-based SLAM requires reliable data association
* Map accuracy depends on landmark placement

---

## 🔮 Future Improvements

* Visual landmark detection using camera
* Occupancy grid mapping
* Graph-based SLAM (GTSAM)
* Autonomous exploration instead of fixed waypoints

---

## 📜 License

Educational and academic use only.
