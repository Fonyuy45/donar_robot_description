# Donar Robot Description

A ROS2 Humble differential drive robot with simplified ball casters for stable Gazebo Ignition simulation.

## Features

- ✅ Differential drive with 2 powered wheels
- ✅ Simplified ball caster design (no hopping!)
- ✅ Dual format support (SDF for Gazebo, URDF for RViz/Nav2)
- ✅ Full ROS2 Humble integration
- ✅ Ready for sensor integration

## Quick Start

### Build
```bash
cd ~/donar_robot_ws
colcon build --packages-select donar_robot_description
source install/setup.bash
```

### Launch Gazebo
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/donar_robot_ws/install/donar_robot_description/share
ros2 launch donar_robot_description gazebo_sdf.launch.py
```

### Launch RViz
```bash
ros2 run rviz2 rviz2 -d src/donar_robot_description/rviz/urdf_config.rviz
```

### Control Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/donar_robot/cmd_vel
```

## Specifications

- **Mass:** 35 kg (base) + 4 kg (wheels) + 0.8 kg (casters) ≈ 40 kg total
- **Max Linear Velocity:** 2.8 m/s
- **Max Angular Velocity:** 3.0 rad/s
- **Wheel Radius:** 0.14 m
- **Wheel Separation:** 0.52 m

## Package Structure
donar_robot_description/
├── config/          # Controller configurations
├── launch/          # Launch files (Gazebo, RViz)
├── meshes/          # STL mesh files
├── models/          # SDF robot models
├── rviz/            # RViz configurations
├── urdf/            # URDF/xacro files
└── worlds/          # Gazebo world files

## Development Status

- [x] Base robot model
- [x] Simplified caster design
- [x] Gazebo Ignition integration
- [x] RViz visualization
- [x] Differential drive control
- [ ] Lidar sensor
- [ ] Camera sensor
- [ ] IMU sensor
- [ ] Nav2 integration
- [ ] SLAM capability

## License

MIT

## Author

Dieudonne YUFONYUY
