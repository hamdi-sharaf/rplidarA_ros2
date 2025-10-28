# RPLidar ROS2 Configuration Guide

## Default Behavior

**As of the latest update, the RPLidar node defaults to full 360-degree scanning.**

### Default Parameters
- `angle_min_limit`: **0.0°** (was previously 10.0°)
- `angle_max_limit`: **360.0°** (was previously 170.0°)

This means when you launch the RPLidar with any standard launch file, you will get **complete 360° scan data** without any filtering.

## Quick Start

### Full 360° Scanning (Default)
```bash
# Launch with any standard launch file - no angle configuration needed
ros2 launch rplidar_ros rplidar_a1_launch.py
ros2 launch rplidar_ros rplidar_a2m7_launch.py
ros2 launch rplidar_ros rplidar_a3_launch.py
# ... etc
```

### Limited Angle Scanning
If you want to restrict the scan to a specific angle range, you have several options:

#### Option 1: Use the dedicated limited angle launch file
```bash
# Launches with 120-240° (front-facing only)
ros2 launch rplidar_ros rplidar_limited_angle_launch.py

# Custom angle range
ros2 launch rplidar_ros rplidar_limited_angle_launch.py angle_min_limit:=45.0 angle_max_limit:=315.0
```

#### Option 2: Override parameters with any launch file
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=90.0 \
  -p angle_max_limit:=270.0
```

#### Option 3: Set parameters at runtime
```bash
ros2 param set /rplidar_node angle_min_limit 120.0
ros2 param set /rplidar_node angle_max_limit 240.0
```

## Parameter Reference

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `angle_min_limit` | float | 0.0 | 0.0-360.0 | Minimum angle in degrees to include in scan |
| `angle_max_limit` | float | 360.0 | 0.0-360.0 | Maximum angle in degrees to include in scan |

## Common Use Cases

### 1. Full Circle (Default)
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
# Uses: angle_min=0.0, angle_max=360.0
```

### 2. Front Hemisphere (0° to 180°)
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=0.0 -p angle_max_limit:=180.0
```

### 3. Front Facing (120° to 240°)
```bash
ros2 launch rplidar_ros rplidar_limited_angle_launch.py
# Or:
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=120.0 -p angle_max_limit:=240.0
```

### 4. Side Scanning (80° to 100°)
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=80.0 -p angle_max_limit:=100.0
```

## Angle Convention

- **0°**: Forward direction (front of the robot)
- **90°**: Left side
- **180°**: Backward direction (rear of the robot)
- **270°**: Right side
- Angles increase **counter-clockwise**

```
        90° (Left)
           |
           |
180° ------+------ 0° (Front)
 (Back)    |
           |
       270° (Right)
```

## Verification

To check the current configuration:

```bash
# Check parameter values
ros2 param get /rplidar_node angle_min_limit
ros2 param get /rplidar_node angle_max_limit

# Monitor scan data
ros2 topic echo /scan
```

The `angle_min` and `angle_max` fields in the LaserScan message will reflect your configured limits.

## Important Notes

1. **Physical Scanning**: The lidar physically scans 360° regardless of the angle limits. The filtering happens in software.

2. **Default Change**: If you were using older versions where the default was 10-170°, you'll now get full 360° data by default. This is generally what most users want.

3. **Performance**: The full 360° scan has minimal performance impact. Filtering only affects the published data size.

4. **Launch Files**: 
   - All standard launch files (`rplidar_a1_launch.py`, etc.) now default to 0-360°
   - `rplidar_limited_angle_launch.py` specifically defaults to 120-240° for users who want limited angles

## Migration from Previous Version

If you were relying on the old 10-170° default behavior:

```bash
# Old behavior (implicit with old version):
ros2 launch rplidar_ros rplidar_a1_launch.py
# Was limited to 10-170°

# New behavior (explicit parameters needed for same result):
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=10.0 -p angle_max_limit:=170.0
```

## Additional Resources

- Full angle limiting documentation: [ANGLE_LIMIT_README.md](ANGLE_LIMIT_README.md)
- Change summary: [CHANGES_SUMMARY.md](CHANGES_SUMMARY.md)
- Main README: [README.md](README.md)
