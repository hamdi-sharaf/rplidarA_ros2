# RPLidar Angle Limiting Feature

## Overview
This modification adds the ability to limit the RPLidar's scan data to a specific angular range. This is useful when you only need scan data from a specific sector (e.g., front-facing only).

## New Parameters

Two new ROS2 parameters have been added:

- **`angle_min_limit`** (float, default: 0.0)
  - Minimum angle in degrees to include in scan data
  - Valid range: 0.0 to 360.0
  - Default provides full 360° coverage

- **`angle_max_limit`** (float, default: 360.0)
  - Maximum angle in degrees to include in scan data
  - Valid range: 0.0 to 360.0
  - Default provides full 360° coverage

## Usage

### Default Behavior (Full 360° Scan)

By default, the RPLidar now scans the **full 360-degree range**. Simply launch with any standard launch file:

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Method 1: Using the Custom Launch File for Limited Angles

The `rplidar_limited_angle_launch.py` provides a convenient way to launch with specific angle limits (defaults to 120° to 240° in the launch file):

```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py
```

Customize the angle range:

```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=20.0 angle_max_limit:=160.0
```

### Method 2: Setting Parameters in Existing Launch Files

Add the parameters to any existing launch file:

```python
parameters=[{
    'serial_port': '/dev/ttyUSB0',
    'angle_min_limit': 120.0,
    'angle_max_limit': 240.0,
    # ... other parameters
}]
```

### Method 3: Command Line with Existing Launch Files

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args -p angle_min_limit:=120.0 -p angle_max_limit:=240.0
```

### Method 4: Setting Parameters at Runtime

You can also change the parameters while the node is running:

```bash
ros2 param set /rplidar_node angle_min_limit 120.0
ros2 param set /rplidar_node angle_max_limit 240.0
```

Note: Parameter changes at runtime require restarting the scan to take effect.

## How It Works

1. The node filters incoming scan data based on the angle of each measurement point
2. Only points within the specified angle range [angle_min_limit, angle_max_limit] are included
3. The filtering works with both angle compensation enabled and disabled
4. The published LaserScan message will have angle_min and angle_max set to match the filtered range

## Example Use Cases

### Front-Facing Only (120° to 240°)
```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=120.0 angle_max_limit:=240.0
```

### Side-Facing (80° to 1200°)
```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=80.0 angle_max_limit:=1200.0
```

### Half Circle Front (0° to 180°)
```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=0.0 angle_max_limit:=180.0
```

## Visualization

To visualize the limited scan data with RViz:

```bash
# Terminal 1: Launch the limited angle lidar
ros2 launch rplidar_ros rplidar_limited_angle_launch.py

# Terminal 2: Launch RViz
rviz2 -d $(ros2 pkg prefix rplidar_ros)/share/rplidar_ros/rviz/rplidar_ros.rviz
```

## Testing

To verify the angle limits are working:

```bash
# Check current parameters
ros2 param get /rplidar_node angle_min_limit
ros2 param get /rplidar_node angle_max_limit

# Monitor scan data
ros2 topic echo /scan
```

The `angle_min` and `angle_max` fields in the LaserScan message should reflect your configured limits.

## Notes

- **Default configuration (0-360°)** provides full circular scan data without any filtering
- To limit the scan range, explicitly set the `angle_min_limit` and `angle_max_limit` parameters
- The lidar will still physically scan 360°, but only data within the specified range will be published
- Setting angle_min_limit > angle_max_limit may result in unexpected behavior
- Very narrow angle ranges may result in sparse scan data
- Performance impact is minimal as filtering is done efficiently during data processing
