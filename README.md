# 🚁 Uvify IFO-S PX4 Simulation with Realsense & RViz Visualization

This package provides a streamlined way to run a **PX4 Software-in-the-Loop (SITL)** simulation of the **Uvify IFO-S** drone, complete with:

* 📸 **Intel Realsense D435i** depth camera integration
* 🧠 Full **RViz visualization** with the drone model and sensor outputs
* 🌐 Support for simulating **multiple drones**

Whether you're testing autonomy, visual perception, or just exploring PX4, this setup gets you flying in simulation quickly.

---

## 🛠 Installation Guide

### 1. PX4 Autopilot + MAVROS

Start by installing PX4 and MAVROS, following the [official PX4 MAVROS guide](https://docs.px4.io/main/en/ros/mavros_installation.html):

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```

Install MAVROS:

```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 2. Gazebo Classic with PX4

Build the simulator using **Gazebo Classic**:

```bash
cd PX4-Autopilot/
make px4_sitl gazebo-classic
```

To verify depth camera integration:

```bash
make px4_sitl gazebo-classic_iris_depth_camera
```


## 🧩 Project Setup

Clone this repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/invidia0/uvify_sitl.git
cd ..
catkin build
```

Don't forget to source your ROS environment:

```bash
source devel/setup.bash
```


## 🚀 Running the Simulation

Launch the full simulation (PX4 + Gazebo + RViz + Realsense) with:

```bash
roslaunch uvify_sitl sim_bringup.launch
```

You should see:

* A Gazebo world with your IFO-S drone
* RViz showing the robot model
* Live **PointCloud2** data from the simulated Realsense D435i


## 🙌 Contributions

Contributions and suggestions are welcome! Feel free to open issues or submit pull requests.