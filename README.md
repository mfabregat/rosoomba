# R(OS)oomba - ROS 2 Roomba/Create Robot Driver & Simulation

A comprehensive ROS 2 driver and simulation package for iRobot Create 1, Create 2, and Roomba series robots. This project provides both real robot control capabilities and full Gazebo simulation support with teleoperation functionality.

## 🤖 Overview

R(OS)oomba is a complete ROS 2 implementation that wraps the C++ library [libcreate](https://github.com/AutonomyLab/libcreate), which uses iRobot's [Open Interface Specification](https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf). The project includes robot description files, Gazebo simulation, comprehensive control interfaces, and teleoperation capabilities.

## ✨ Features

| Feature | Real Robot | Simulation |
|---------|------------|------------|
| Odometry | ✅ Available | ✅ Available |
| Drive (v,ω) | ✅ Available | ✅ Available |
| Joystick Teleoperation | ✅ Available | ✅ Available |
| Bumpers | ✅ Available | ⚠️ Partial |
| Cliff sensors | ✅ Available | ⚠️ Partial |
| Battery info | ✅ Available | ❌ N/A |
| LEDs | ✅ Available | ❌ N/A |
| Sound | ✅ Available | ❌ N/A |
| Joint States | ✅ Available | ✅ Available |
| ROS 2 Control | ❌ N/A | ✅ Available |
| Gazebo Integration | ❌ N/A | ✅ Available |
| RViz Visualization | ✅ Available | ✅ Available |

## 🚀 Supported Robots

### iRobot Create Series
- **Create 1** (Roomba 500 series base)
- **Create 2** (Roomba 600/700 series base)

### iRobot Roomba Series
- **Roomba 400 series**
- **Roomba 500 series**
- **Roomba 600 series**
- **Roomba 700 series**
- **Roomba 800 series** (confirmed by community)

## 🐳 Quick Start with Dev Container

This project includes a fully configured development container for easy setup:

### Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/rosoomba.git
   cd rosoomba
   ```

2. Open in VS Code:
   ```bash
   code .
   ```

3. When prompted, click "Reopen in Container" or press `Ctrl+Shift+P` and select "Dev Containers: Reopen in Container"

4. Wait for the container to build and start (first time may take a few minutes)

5. Build the workspace:
   ```bash
   # Use the integrated build task
   Ctrl+Shift+P → "Tasks: Run Task" → "build"
   
   # Or manually in terminal
   ./vscode/tasks/build.sh
   ```

## 📦 Package Structure

The workspace contains several specialized packages:

### Core Packages (credits to [AutonomyLab](https://github.com/AutonomyLab))
- **`create_driver`** - Main ROS 2 driver for real iRobot Create/Roomba robots
- **`rosoomba_msgs`** - Custom message definitions for Create-specific sensors and commands
- **`libcreate`** - C++ library for low-level robot communication

### Simulation Packages  
- **`rosoomba_bringup`** - Launch files and configurations for both real and simulated robots
- **`rosoomba_description`** - URDF robot model, meshes, and visualization components
- **`rosoomba_gazebo`** - Gazebo simulation worlds and robot spawn configurations

## 🎮 Usage

### Simulation Mode

Launch the complete simulation environment with Gazebo, RViz, and joystick control:

```bash
# Source the workspace
source install/setup.bash

# Launch simulation with all components
ros2 launch rosoomba_bringup rosoomba_sim.launch.py

# Launch without RViz (for headless operation)
ros2 launch rosoomba_bringup rosoomba_sim.launch.py rviz:=false
```

#### Simulation Components
- **Gazebo** - Physics simulation environment
- **RViz** - 3D visualization and robot state monitoring  
- **ros2_control** - Differential drive controller
- **Joy teleoperation** - Joystick control interface
- **ROS-Gazebo Bridge** - Communication between ROS 2 and Gazebo

### Real Robot Mode

For controlling actual iRobot Create/Roomba robots:

```bash
# For Create 2 (Roomba 600/700 series)
ros2 launch create_bringup create_2.launch

# For Create 1 (Roomba 500 series)  
ros2 launch create_bringup create_1.launch

# For Roomba 400 series
ros2 launch create_bringup roomba_400.launch
```

#### Launch Arguments
- **`config`** - Path to configuration YAML file (default: `create_bringup/config/default.yaml`)
- **`desc`** - Enable robot description (default: `true`)

Example with custom configuration:
```bash
ros2 launch create_bringup create_2.launch config:=/path/to/config.yaml desc:=false
```

### Joystick Teleoperation

Connect a compatible joystick/gamepad and use it to control the robot:

```bash
# Launch only teleoperation (for use with existing robot)
ros2 launch rosoomba_bringup joy_teleop.launch.py
```

#### Default Controls
- **Left Stick** - Linear velocity (forward/backward)
- **Right Stick** - Angular velocity (rotation)
- **Button mapping** - Configurable via `config/joy_config.yaml`

## 🔧 Configuration

### Controller Configuration
Edit `src/rosoomba_bringup/config/rosoomba_controllers.yaml` to modify:
- PID gains for differential drive controller
- Wheel separation and radius parameters
- Joint names and controller settings

### Gazebo Bridge Configuration  
Edit `src/rosoomba_bringup/config/gz_bridge.yaml` to configure:
- Topic mappings between ROS 2 and Gazebo
- Message type conversions
- Communication interfaces

### Joystick Configuration
Edit `src/rosoomba_bringup/config/joy_config.yaml` to customize:
- Button mappings
- Axis configurations  
- Velocity scaling factors

## 🛠️ Manual Installation (Alternative)

If not using the dev container:

### Prerequisites
- **ROS 2** (Jazzy, Humble, or Iron)
- **Gazebo** (Garden or later)
- **Ubuntu packages**: `python3-rosdep`, `python3-colcon-common-extensions`

### Installation Steps

1. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install python3-rosdep python3-colcon-common-extensions
   ```

2. **Create workspace:**
   ```bash
   mkdir -p ~/rosoomba_ws/src
   cd ~/rosoomba_ws/src
   ```

3. **Clone repositories:**
   ```bash
   git clone <this-repo-url> .
   vcs import < ros2.repos
   ```

4. **Install ROS dependencies:**
   ```bash
   cd ~/rosoomba_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Build workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🔌 ROS 2 Topics

### Published Topics (Real Robot)
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot pose and velocity |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint positions |
| `/battery/voltage` | `std_msgs/Float32` | Battery voltage |
| `/battery/current` | `std_msgs/Int16` | Battery current |
| `/battery/charge` | `std_msgs/Float32` | Battery charge level |
| `/battery/capacity` | `std_msgs/Float32` | Battery capacity |
| `/bumper` | `rosoomba_msgs/Bumper` | Bumper sensor states |
| `/cliff` | `rosoomba_msgs/Cliff` | Cliff sensor readings |
| `/mode` | `rosoomba_msgs/Mode` | Robot operation mode |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/dock` | `std_msgs/Empty` | Dock command |
| `/undock` | `std_msgs/Empty` | Undock command |
| `/define_song` | `rosoomba_msgs/DefineSong` | Define custom songs |
| `/play_song` | `rosoomba_msgs/PlaySong` | Play defined songs |

### Simulation Additional Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/Twist` | Controller velocity input |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | Controller joint states |

## 🧪 Development

### Building
```bash
# Quick build
./vscode/tasks/build.sh

# Clean build
./vscode/tasks/hard_build.sh

# Manual colcon build
colcon build --symlink-install
```

### Testing
```bash
# Run tests
colcon test

# Check test results  
colcon test-result --verbose
```

### Linting
```bash
# Run all linters
ament_lint_auto

# Specific linters
ament_cpplint src/
ament_flake8 src/
```

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork** the repository
2. **Create** a feature branch: `git checkout -b feature/amazing-feature`
3. **Commit** your changes: `git commit -m 'Add amazing feature'`
4. **Push** to the branch: `git push origin feature/amazing-feature`
5. **Open** a Pull Request

### Code Style
- Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use `ament_lint_auto` for automated checking
- Document all public APIs

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

### Original Authors
- **[Jacob Perron](http://jacobperron.ca)** - Original ROS 2 port and main development
- **[Autonomy Lab](https://autonomy.cs.sfu.ca), Simon Fraser University**

### Contributors
- **[Michael Browne](http://brownem.engineer/)** - Roomba 700/800 series testing
- **[Clyde McQueen](https://github.com/clydemcqueen)** - Sound support implementation
- **[Ben Wolsieffer](https://github.com/lopsided98)** - JointState publisher, Create 1 description
- **[Pedro Grojsgold](https://github.com/pgold)** - Initial ROS 2 port
- **[Josh Gadeken](https://github.com/process1183)** - OI Mode reporting workaround

### Libraries & Dependencies
- **[libcreate](https://github.com/AutonomyLab/libcreate)** - Core C++ library for iRobot communication
- **[ROS 2](https://docs.ros.org)** - Robot Operating System framework
- **[Gazebo](https://gazebosim.org/)** - Physics simulation engine
- **[ros2_control](https://control.ros.org/)** - Real-time control framework

## 🔗 Links

- **Documentation**: [ROS Wiki](http://wiki.ros.org/create_robot)
- **Support**: [ROS Answers (tag: create_robot)](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:create_robot/page:1/)
- **Issues**: [GitHub Issues](https://github.com/your-username/rosoomba/issues)
- **Original Project**: [AutonomyLab/create_robot](https://github.com/AutonomyLab/create_robot)

---

**Happy Robooting! 🤖**
