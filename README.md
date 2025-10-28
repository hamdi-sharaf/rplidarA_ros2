# SLAMTEC LIDAR ROS2 Package

ROS2 node for SLAMTEC LIDAR

> **📝 Note**: This version includes angle limiting features. By default, the RPLidar scans the **full 360-degree range** (0-360°). See [CONFIGURATION.md](CONFIGURATION.md) for detailed configuration options and [ANGLE_LIMIT_README.md](ANGLE_LIMIT_README.md) for angle limiting features.

Visit following Website for more details about SLAMTEC LIDAR:

SLAMTEC LIDAR roswiki: <http://wiki.ros.org/rplidar>

SLAMTEC LIDAR HomePage: <http://www.slamtec.com/en/Lidar>

SLAMTEC LIDAR SDK: <https://github.com/Slamtec/rplidar_sdk>

SLAMTEC LIDAR Tutorial: <https://github.com/robopeak/rplidar_ros/wiki>

## Supported SLAMTEC LIDAR

| Lidar Model |
| ---------------------- |
|RPLIDAR A1              |
|RPLIDAR A2              |
|RPLIDAR A3              |
|RPLIDAR S1              |
|RPLIDAR S2              |
|RPLIDAR S2E             |
|RPLIDAR S3              |
|RPLIDAR T1              |
|RPLIDAR C1              |



## Installation Instructions

### Prerequisites

- ROS2 installed (Humble, Rolling, Galactic, or Foxy)
- Colcon build tools
- Git

### Step 1: Create ROS2 Workspace

[ROS2 Tutorials Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)

Create a workspace directory (e.g., `rplidarA_ros2`):

```bash
mkdir -p ~/rplidarA_ros2/src
cd ~/rplidarA_ros2/src
```

### Step 2: Clone Repository

Clone this repository into your workspace:

```bash
cd ~/rplidarA_ros2/src
git clone https://github.com/hamdi-sharaf/rplidarA_ros2.git .
```

Or if cloning the original SLAMTEC repository:

```bash
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
```

### Step 3: Install Dependencies

Make sure all ROS2 dependencies are installed:

```bash
cd ~/rplidarA_ros2
sudo apt update
sudo apt install ros-<rosdistro>-tf2-ros ros-<rosdistro>-rviz2
```

Replace `<rosdistro>` with your ROS2 distribution (humble, rolling, galactic, or foxy).

If colcon is not installed:

```bash
sudo apt install python3-colcon-common-extensions
```

### Step 4: Build the Package

From the root of your workspace, build the package:

```bash
cd ~/rplidarA_ros2
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install
```

**Note:** The `--symlink-install` flag allows you to edit Python launch files without rebuilding.

### Step 5: Source the Workspace

After building, source the workspace:

```bash
source ~/rplidarA_ros2/install/setup.bash
```

**Tip:** Add this to your `.bashrc` for automatic sourcing in new terminals:

```bash
echo "source ~/rplidarA_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Verify Installation

Check that the device is detected:

```bash
ls -l /dev/ttyUSB*
# Should show: crw-rw---- 1 root dialout ... /dev/ttyUSB0
```

Check if your user is in the dialout group:

```bash
groups $USER
# If 'dialout' is not listed, add yourself:
sudo usermod -aG dialout $USER
# Then log out and log back in
```

## Run rplidar_ros

### Basic Usage (For example A2 lidar)

Run the RPLidar node without RViz visualization:

```bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py 

```

### Run with Angle Limiting

Launch with limited scan angle range (e.g., 120° to 240°):

```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=120.0 angle_max_limit:=240.0
```

### Run with TF Transform (Integration with Robot)

Launch with TF transform from `base_link` to `laser_frame`:

```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py 

```

## RVIZ Visualization

### Basic Usage

```bash
ros2 launch rviz2 rviz2
```
Select fix frame as laser_frame


## RPLIDAR frame

RPLIDAR frame must be broadcasted according to picture shown in rplidar-frame.png

## Additional Features

### Angle Limiting
This package includes angle limiting features that allow you to restrict the scan range. By default, RPLidar scans the full 360-degree range (0-360°).

See [ANGLE_LIMIT_README.md](rplidar_ros/ANGLE_LIMIT_README.md) for detailed information.

### TF Transform Integration
The `rplidar_limited_angle_launch.py` includes built-in TF transform publisher to connect the lidar to your robot's `base_link` frame.

**Verify TF tree:**
```bash
ros2 run tf2_tools view_frames
# Opens a PDF showing the TF tree

ros2 run tf2_ros tf2_echo base_link laser_frame
# Shows the transform between frames
```

### Configuration Options
For detailed configuration options, see [CONFIGURATION.md](rplidar_ros/CONFIGURATION.md)

## Topics Published

- `/scan` (sensor_msgs/LaserScan) - Lidar scan data

## Topics Subscribed

None

## Parameters

- `serial_port` (string, default: "/dev/ttyUSB0") - Serial port name
- `serial_baudrate` (int, default: 115200) - Serial baudrate
- `frame_id` (string, default: "laser_frame") - Frame ID for scan data
- `inverted` (bool, default: false) - Invert scan direction
- `angle_compensate` (bool, default: true) - Enable angle compensation
- `scan_mode` (string, default: "") - Scan mode (Standard, Express, Boost, Sensitivity, Stability)
- `angle_min_limit` (float, default: 0.0) - Minimum angle limit in degrees
- `angle_max_limit` (float, default: 360.0) - Maximum angle limit in degrees

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

See [LICENSE](rplidar_ros/LICENSE) file for details.
