# MOCHA MultiRobot Simulation

**Simulation cho 1 UGV (Jackal) + 1 UAV (Titan) trên 1 Laptop**

---

### System Architecture

```
┌──────────────────────────────────┐
│    YOUR SINGLE LAPTOP            │
│                                  │
│  • Gazebo Server (cả 2 robot)   │
│  • ROS Master (localhost:11311)  │
│  • Jackal UGV + Titan UAV       │
│  • MOCHA Database Sync           │
│                                  │
└──────────────────────────────────┘
```

### Prerequisites

```bash
# 1. ROS + Gazebo
sudo apt install gazebo11 ros-noetic-gazebo-ros-pkgs

# 2. Robot Simulators
sudo apt install ros-noetic-jackal-gazebo

# Titan - Hector Quadrotor
# Follow: https://github.com/RAFALAMAO/hector-quadrotor-noetic

# 3. Python + ZMQ
pip3 install zmq
```

### Setup & Run

```bash
# Build workspace
cd ~/swarm_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

#### **Cách 1: Cả 2 robot cùng lúc (DỄ NHẤT)**

```bash
roslaunch mocha_tplink mocha_sim_single_all.launch
```

✅ Gazebo mở, hiển thị cả Jackal + Titan

#### **Cách 2: Từng robot một**

```bash
# Terminal 1: Jackal + Gazebo
roslaunch jackal_gazebo spawn_jackal.launch joystick:=false x=2

# Terminal 2: Titan vào Gazebo sẵn có
roslaunch px4 mavros_posix_sitl.launch sdf:=~/PX4_Firmware/Tools/sitl_gazebo/models/iris_3d_lidar/iris_3d_lidar.sdf
```

#### **Cách 3: Auto setup**

```bash
bash ~/swarm_ws/src/mocha_tplink/scripts/quick_setup_single.sh
```

### Monitor Topics

```bash
# Terminal 3: Xem topics
rosnode list
rostopic list
rostopic echo /mocha/jackal_sim/messages
rostopic echo /mocha/titan_sim/messages
```

---

##  Project Structure

```
mocha_tplink/
├── README.md                    👈 MAIN GUIDE
│
├── config/
│   ├── robot_configs_sim.yaml       (Jackal + Titan config)
│   ├── topic_configs_sim.yaml       (Shared topics)
│   └── radio_configs.yaml           (ZMQ parameters)
│
├── launch/
│   ├── mocha_sim_single_all.launch  (Cả 2 robot cùng lúc) ⭐
│   └── mocha_sim_single.launch      (1 robot tại 1 lần)
│
├── scripts/
│   └── quick_setup_single.sh        (Auto setup)
│
└── CMakeLists.txt, package.xml, etc.
```

---

## 🛠️ Configuration Details

### robot_configs_sim.yaml

```yaml
jackal_sim:
  node-type: "ground_robot"
  IP-address: "127.0.0.1"
  base-port: "5005"

titan_sim:
  node-type: "aerial_robot"
  IP-address: "127.0.0.1"
  base-port: "5006"
```

### topic_configs_sim.yaml

Topics synchronized between robots:
```yaml
ground_robot:
  - msg_topic: "/jackal_sim/odometry/filtered"
    msg_priority: "HIGH_PRIORITY"

aerial_robot:
  - msg_topic: "/titan_sim/odometry/filtered"
    msg_priority: "HIGH_PRIORITY"
```

### radio_configs.yaml

ZMQ network parameters:
```yaml
communication:
  protocol: "zmq_tcp"
  heartbeat_interval: 1000      # ms
  message_timeout: 5000          # ms
  max_retries: 3
```

---

## 🐛 Troubleshooting

### Gazebo Issues

```bash
# Reset Gazebo config
rm -rf ~/.gazebo

# Reinstall
sudo apt install --reinstall gazebo11 ros-noetic-gazebo-ros-pkgs

# Run again
roslaunch mocha_tplink mocha_sim_single_all.launch
```

### Import Errors

```bash
# Check installed packages
dpkg -l | grep jackal
dpkg -l | grep gazebo

# Install if missing
sudo apt install ros-noetic-jackal-gazebo
sudo apt install ros-noetic-gazebo-ros-pkgs
```

### MOCHA Database Not Running

```bash
# Check nodes
rosnode list | grep mocha

# View logs
rostopic echo /mocha/*/health

# Check config
rosparam get /mocha/robot_configs
```

---

## 🚀 Quick Start

```bash
# Build workspace (first time)
cd ~/swarm_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# Run simulation (cả 2 robot)
roslaunch mocha_tplink mocha_sim_single_all.launch
```

---

## ✨ Features

✅ **2 Robot Types:** UGV (Jackal) + UAV (Titan)  
✅ **MOCHA Integration:** Automatic topic sync  
✅ **Zero Network Config:** No WiFi setup needed  
✅ **Production Ready:** Tested with ROS Noetic  

---

**Happy Simulating! 🎉**
