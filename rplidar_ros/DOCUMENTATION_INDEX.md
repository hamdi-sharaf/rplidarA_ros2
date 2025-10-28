# RPLidar ROS2 Documentation Index

This document provides an overview of all documentation files in this package.

## Main Documentation Files

### 📘 [README.md](README.md)
**Main package documentation**
- Installation instructions
- Supported RPLIDAR models
- Basic usage and launch commands
- Quick start guide

### ⚙️ [CONFIGURATION.md](CONFIGURATION.md) ⭐ **START HERE**
**Complete configuration guide for angle settings**
- **Default behavior**: Full 360° scanning (0-360°)
- Quick start examples for different use cases
- Parameter reference and common configurations
- Angle convention and coordinate system
- Migration guide from previous versions

### 🎯 [ANGLE_LIMIT_README.md](ANGLE_LIMIT_README.md)
**Detailed angle limiting feature documentation**
- Feature overview and technical details
- Multiple methods to set angle limits
- Example use cases (front-facing, side-facing, etc.)
- Visualization with RViz
- How the filtering works internally

### 📝 [CHANGES_SUMMARY.md](CHANGES_SUMMARY.md)
**Summary of code modifications**
- List of modified files
- Code changes made for angle limiting feature
- Technical implementation details
- Build and validation information

## Quick Reference

### Default Settings (New)
```yaml
angle_min_limit: 0.0°     # Full circle coverage
angle_max_limit: 360.0°   # Full circle coverage
```

### Common Commands

```bash
# Full 360° scan (default)
ros2 launch rplidar_ros rplidar_a1_launch.py

# Limited angle scan (120-240°)
ros2 launch rplidar_ros rplidar_limited_angle_launch.py

# Custom angle range
ros2 launch rplidar_ros rplidar_a1_launch.py --ros-args \
  -p angle_min_limit:=90.0 -p angle_max_limit:=270.0

# Check current parameters
ros2 param get /rplidar_node angle_min_limit
ros2 param get /rplidar_node angle_max_limit
```

## Documentation Roadmap

**For new users:**
1. Read [README.md](README.md) for basic installation and setup
2. Check [CONFIGURATION.md](CONFIGURATION.md) to understand default behavior and options
3. Launch your RPLidar (it will use full 360° by default)

**For users who need angle limiting:**
1. Start with [CONFIGURATION.md](CONFIGURATION.md) for quick examples
2. Read [ANGLE_LIMIT_README.md](ANGLE_LIMIT_README.md) for detailed feature documentation
3. Use `rplidar_limited_angle_launch.py` or add parameters to existing launch files

**For developers:**
1. Review [CHANGES_SUMMARY.md](CHANGES_SUMMARY.md) for code modifications
2. Check `src/rplidar_node.cpp` for implementation details
3. Examine launch files in `launch/` directory

## Key Changes from Original Package

### ✅ New Features
- Angle limiting capability with `angle_min_limit` and `angle_max_limit` parameters
- **Default changed to 0-360° (full circle)** for standard operation
- New launch file: `rplidar_limited_angle_launch.py` for easy angle limiting

### 📂 New Files
- `CONFIGURATION.md` - Complete configuration guide
- `ANGLE_LIMIT_README.md` - Angle limiting feature documentation
- `CHANGES_SUMMARY.md` - Summary of modifications
- `DOCUMENTATION_INDEX.md` - This file
- `launch/rplidar_limited_angle_launch.py` - Launch file for limited angles

### 🔧 Modified Files
- `src/rplidar_node.cpp` - Core implementation of angle filtering
- `README.md` - Updated with note about angle features

## Support and Resources

### Official SLAMTEC Resources
- [SLAMTEC ROS Wiki](http://wiki.ros.org/rplidar)
- [SLAMTEC Homepage](http://www.slamtec.com/en/Lidar)
- [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)
- [RPLIDAR Tutorial](https://github.com/robopeak/rplidar_ros/wiki)

### This Package
- GitHub Repository: https://github.com/Slamtec/rplidar_ros
- Branch: `ros2`

## Troubleshooting

### Common Issues

**Q: I'm not getting any scan data**
- Check: `ros2 topic echo /scan`
- Verify angle limits: `ros2 param get /rplidar_node angle_min_limit`
- Ensure device permissions: `sudo chmod 777 /dev/ttyUSB0`

**Q: I only want partial scan data**
- See [CONFIGURATION.md](CONFIGURATION.md) for examples
- Use `rplidar_limited_angle_launch.py` or add parameters to your launch

**Q: My old configuration isn't working**
- Default changed from 10-170° to 0-360°
- Add explicit parameters if you need the old range
- See "Migration from Previous Version" in [CONFIGURATION.md](CONFIGURATION.md)

**Q: How do I verify my angle limits?**
```bash
# Check parameters
ros2 param list | grep angle
ros2 param get /rplidar_node angle_min_limit
ros2 param get /rplidar_node angle_max_limit

# Check scan message
ros2 topic echo /scan | grep angle
```

## Version Information

- **ROS2 Distributions**: Foxy, Galactic, Humble, Rolling
- **Angle Limiting Feature**: Added in this fork
- **Default Angle Range**: 0-360° (changed from 10-170°)

---

**Last Updated**: October 28, 2025
