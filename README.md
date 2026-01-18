# LIMO COBOT: Autonomous Pick-and-Place Robot

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ROS Noetic">
  <img src="https://img.shields.io/badge/Ubuntu-20.04-orange" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/Python-3.8+-green" alt="Python 3.8+">
  <img src="https://img.shields.io/badge/Status-Working-brightgreen" alt="Status: Working">
</p>

## 🎯 Project Overview

This project implements a **fully autonomous pick-and-place mission** using the AgileX LIMO mobile robot equipped with a MyCobot 280 robotic arm. The system demonstrates advanced robotics concepts including:

- **Computer Vision** - Color-based object detection with HSV filtering
- **Sensor Fusion** - Combined RGB-D camera and 2D LIDAR
- **Motion Planning** - MoveIt-based arm trajectory planning
- **Event-Driven Architecture** - Mission controller with state machine coordination
- **Safety Systems** - Emergency stop, collision avoidance, and retry logic

### Mission Sequence

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                          AUTONOMOUS PICK-AND-PLACE MISSION                      │
├────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│   [1] SEARCH        [2] APPROACH       [3] PICK           [4] TRANSPORT        │
│   ┌─────────┐       ┌─────────┐       ┌─────────┐        ┌─────────┐          │
│   │ 🔍 Find │  ───► │ 🎯 Move │  ───► │ 🤖 Grab │  ───►  │ 🔄 Turn │          │
│   │   Cube  │       │ to Table│       │  Cube   │        │ Around  │          │
│   └─────────┘       └─────────┘       └─────────┘        └─────────┘          │
│                                                                │               │
│   [8] COMPLETE      [7] PLACE         [6] APPROACH       [5] SEARCH           │
│   ┌─────────┐       ┌─────────┐       ┌─────────┐        ┌─────────┐          │
│   │ ✅ Done │  ◄─── │ 📦 Drop │  ◄─── │ 🎯 Move │  ◄───  │ 🔍 Find │          │
│   │         │       │ in Box  │       │ to Box  │        │  Basket │          │
│   └─────────┘       └─────────┘       └─────────┘        └─────────┘          │
│                                                                                 │
└────────────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
Final_Project_copy/
├── src/
│   ├── limo_vision/                    # 🧠 Core Mission Scripts
│   │   └── scripts/
│   │       ├── mission_controller.py           # Event-driven mission coordinator
│   │       ├── blue_cube_approach_pick_enhanced.py  # Cube detection & approach
│   │       ├── red_basket_approach.py          # Basket detection & approach
│   │       ├── final_arm_pick.py               # MoveIt arm control
│   │       └── mission_status.py               # Shared status constants
│   │
│   ├── limo_project/                   # 🚀 Launch & Configuration
│   │   ├── launch/
│   │   │   ├── complete_pick_place_mission.launch  # ⭐ MAIN LAUNCH FILE
│   │   │   ├── gazebo.launch                   # Gazebo simulation
│   │   │   ├── demo_gazebo.launch              # Demo with RViz
│   │   │   ├── limo_gmapping.launch            # SLAM mapping
│   │   │   └── limo_navigation_diff.launch     # Navigation stack
│   │   ├── worlds/
│   │   │   └── clearpath_playpen_Pick.world    # Mission environment
│   │   ├── param/                              # Navigation parameters
│   │   └── maps/                               # Saved maps
│   │
│   ├── limo_cobot_moveit/              # 🦾 MoveIt Configuration
│   │   ├── config/                     # Motion planning configs
│   │   └── launch/                     # MoveIt launch files
│   │
│   ├── limo_description/               # 🤖 Robot URDF Models
│   │   ├── urdf/                       # LIMO robot description
│   │   └── meshes/                     # 3D mesh files
│   │
│   ├── mycobot_description/            # 🦾 Arm URDF Models
│   │   └── urdf/                       # MyCobot arm description
│   │
│   └── limo_gazebo_sim/                # 🌍 Simulation Support
│       ├── worlds/                     # Additional world files
│       └── launch/                     # Alternative launch files
│
├── build/                              # Catkin build output
├── devel/                              # Development space
└── README.md                           # This file
```

---

## 🚀 Quick Start

### Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Ubuntu | 20.04 LTS | Required for ROS Noetic |
| ROS | Noetic | Full desktop install recommended |
| Gazebo | 11.x | Comes with ROS Noetic |
| MoveIt | 1.1.x | `sudo apt install ros-noetic-moveit` |
| Python | 3.8+ | With OpenCV and NumPy |

### Installation

```bash
# 1. Clone/copy project to your workspace
cd ~
cp -r /path/to/Final_Project_copy ~/Final_Project_copy

# 2. Install ROS dependencies
cd ~/Final_Project_copy
rosdep install --from-paths src --ignore-src -r -y

# 3. Install Python dependencies
pip3 install opencv-python numpy filterpy

# 4. Build the workspace
catkin_make

# 5. Source the workspace (add to ~/.bashrc for persistence)
source devel/setup.bash
echo "source ~/Final_Project_copy/devel/setup.bash" >> ~/.bashrc
```

### Running the Mission

```bash
# Launch the complete autonomous mission
roslaunch limo_project complete_pick_place_mission.launch sim:=true

# The robot will automatically:
# 1. Search for the blue cube
# 2. Approach and pick it up
# 3. Search for the red basket
# 4. Approach and place the cube inside
```

### Emergency Stop

```bash
# In case of emergency, stop all robot motion:
rostopic pub /emergency_stop std_msgs/Bool "data: true" -1

# To resume (restart the mission):
rostopic pub /emergency_stop std_msgs/Bool "data: false" -1
```

---

## 🏗️ System Architecture

### Node Communication Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                           MISSION CONTROLLER                                  │
│                        (mission_controller.py)                                │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │  INIT → FIND_CUBE → PICK_CUBE → FIND_BASKET → PLACE_CUBE → COMPLETE    │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────┬───────────────────────────────────────────────┘
                               │
           ┌───────────────────┼───────────────────┐
           │                   │                   │
           ▼                   ▼                   ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  BLUE CUBE NODE  │  │    ARM NODE      │  │  BASKET NODE     │
│                  │  │                  │  │                  │
│ /blue_cube_      │  │ /arm_pick/       │  │ /basket_approach/│
│   approach/      │  │   trigger        │  │   trigger        │
│   trigger ◄──────│  │   trigger ◄──────│  │   trigger ◄──────│
│                  │  │                  │  │                  │
│ /blue_cube_      │  │ /arm_pick/       │  │ /basket_approach/│
│   approach/      │  │   status ────────│  │   status ────────│
│   status ────────│  │                  │  │                  │
└────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
         │                     │                     │
         └──────────┬──────────┴──────────┬──────────┘
                    │                     │
                    ▼                     ▼
            ┌───────────────┐     ┌───────────────┐
            │   /cmd_vel    │     │  /emergency   │
            │  (Robot Base) │     │    _stop      │
            └───────────────┘     └───────────────┘
```

### Safety Features

| Feature | Implementation | Purpose |
|---------|---------------|---------|
| **Emergency Stop** | `/emergency_stop` topic | Immediately halt all motion |
| **cmd_vel Arbitration** | `_safe_publish_cmd()` | Prevent multiple nodes publishing |
| **Collision Avoidance** | MoveIt planning scene | Arm avoids table collision |
| **Retry Logic** | Mission controller | Retry failed pick/place operations |
| **Timeout Handling** | Phase timeouts | Prevent infinite loops |

---

## 📋 Node Details

### 1. Mission Controller (`mission_controller.py`)

**Purpose:** Orchestrates the entire mission using event-driven coordination.

**Key Features:**
- State machine with 6 phases: INIT → FIND_CUBE → PICK_CUBE → FIND_BASKET → PLACE_CUBE → COMPLETE
- Configurable timeouts and retry limits
- Emergency stop handling with graceful shutdown
- Retry repositioning (backs up before retrying failed operations)

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `~find_cube_timeout` | 180.0s | Max time to find cube |
| `~pick_cube_timeout` | 60.0s | Max time for pick operation |
| `~max_pick_retries` | 3 | Retry attempts for picking |
| `~max_place_retries` | 3 | Retry attempts for placing |

### 2. Blue Cube Approach (`blue_cube_approach_pick_enhanced.py`)

**Purpose:** Detects and approaches the blue cube using vision and LIDAR.

**State Machine:**
```
IDLE → SEARCH → APPROACH → APPROACH_CUBE → BLIND_APPROACH → DONE
```

**Key Features:**
- HSV color filtering for blue detection
- Kalman filtering for smooth position tracking
- Multi-phase approach (table → cube → blind approach)
- Coverage mapping to avoid revisiting searched areas
- Camera blind spot handling (< 0.2m uses dead reckoning)

**Parameters:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `CUBE_STOP_LIDAR` | 0.06m | Stop distance from cube |
| `SWITCH_TO_CUBE_LIDAR` | 0.55m | Switch to precise approach |
| `BLUE_LOW/HIGH` | HSV range | Blue color detection |

### 3. Red Basket Approach (`red_basket_approach.py`)

**Purpose:** Detects and approaches the red basket for cube placement.

**State Machine:**
```
IDLE → BACKUP → SEARCH → APPROACH → FINAL_APPROACH → DONE
```

**Key Features:**
- Dual HSV range for red (wraps around hue 0/180)
- Shape validation (aspect ratio, minimum area)
- Edge rejection to avoid partial detections
- Alignment control before final approach

**Parameters:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `STOP_DIST` | 0.12m | Stop distance from basket |
| `MIN_SAFE_DIST` | 0.10m | Emergency stop distance |
| `CONFIRM_DETECTIONS` | 3 | Required consecutive detections |

### 4. Arm Controller (`final_arm_pick.py`)

**Purpose:** Controls the MyCobot arm for pick and place operations.

**Key Features:**
- MoveIt integration with RRTstar planner
- Cartesian path planning for precise movements
- Gazebo model state integration for cube position
- Collision-aware planning with table obstacle
- Threaded execution to prevent blocking

**Operations:**
| Operation | Description |
|-----------|-------------|
| `_pick_cube()` | Move to cube, descend, grip, lift |
| `_place_in_basket()` | Extend over basket, release, retract |
| `_move_to_home()` | Return to safe home position |

---

## 🔧 Configuration

### World Environment

The simulation world contains:

| Object | Position (x, y, z) | Description |
|--------|-------------------|-------------|
| Magenta Table | (4.0, 0, 0.10) | Cube pickup location |
| Blue Cube | (3.96, 0, 0.225) | Target object |
| Red Basket | (3.6, 3.0, 0.10) | Drop-off location |

### Tuning Color Detection

Edit HSV ranges in the respective scripts:

```python
# Blue cube (blue_cube_approach_pick_enhanced.py)
self.BLUE_LOW = np.array([85, 80, 40])
self.BLUE_HIGH = np.array([145, 255, 255])

# Red basket (red_basket_approach.py)
self.BASKET_LOW = np.array([0, 100, 50])
self.BASKET_HIGH = np.array([10, 255, 255])
self.BASKET_LOW2 = np.array([170, 100, 50])   # Red wraps around
self.BASKET_HIGH2 = np.array([180, 255, 255])
```

---

## 🔍 Debugging & Monitoring

### Useful Commands

```bash
# View all active nodes
rosnode list

# Monitor mission status
rostopic echo /mission/status

# View cube detection debug image
rosrun image_view image_view image:=/blue_cube/annotated

# View basket detection debug image
rosrun image_view image_view image:=/basket_debug/image

# Check arm status
rostopic echo /arm_pick/status

# Monitor velocity commands
rostopic echo /cmd_vel

# View TF tree
rosrun tf view_frames && evince frames.pdf
```

### ROS Topics Reference

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `Twist` | Navigation nodes | Robot velocity |
| `/mission/status` | `String` | Mission controller | Current phase |
| `/blue_cube_approach/trigger` | `Bool` | Mission controller | Start cube search |
| `/blue_cube_approach/status` | `String` | Blue cube node | Approach status |
| `/arm_pick/trigger` | `Bool` | Mission controller | Trigger pick |
| `/arm_pick/status` | `String` | Arm node | Pick/place status |
| `/basket_approach/trigger` | `Bool` | Mission controller | Start basket search |
| `/basket_approach/status` | `String` | Basket node | Approach status |
| `/emergency_stop` | `Bool` | Any | Emergency stop |

---

## 🔬 Technical Implementation Details

### Sensor Fusion Strategy

1. **Camera** (Intel RealSense D435i style)
   - RGB for color-based detection
   - Depth for distance estimation (0.2m - 10m range)
   - 30 FPS update rate

2. **LIDAR** (YDLIDAR X4)
   - 2D laser scan for precise distance
   - Handles camera blind spot (<0.2m)
   - Obstacle detection for safety

3. **Fusion Logic**
   - Far range (>0.55m): Camera guides approach
   - Near range (<0.55m): LIDAR takes over
   - Blind spot (<0.2m): Dead reckoning

### Motion Planning

- **Planner:** RRTstar (optimal path planning)
- **Planning time:** 5.0s max per attempt
- **Collision checking:** Enabled with table obstacle
- **Cartesian paths:** Used for precise vertical movements

---

## 📄 License

This project is developed for educational purposes as part of the **AER657 Cognitive Robotics** course at Cairo University.

---

## 👨‍💻 Authors

- **Karim** - Project development and implementation

---

## 📚 References

1. ROS Navigation Stack: http://wiki.ros.org/navigation
2. MoveIt Documentation: https://moveit.ros.org/documentation/
3. OpenCV Color Detection: https://docs.opencv.org/
4. Kalman Filtering: https://filterpy.readthedocs.io/
