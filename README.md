# Local LiDAR Planner Workspace

This workspace contains a stripped-down ROS 2 package `local_lidar_planner` that performs purely local, LiDAR-based obstacle avoidance. The package was extracted from the main autonomy stack so it can be versioned separately and iterated on without coupling to global-mapping logic.

## Layout

- `src/local_lidar_planner`: ROS 2 package with source, launch files, and the precomputed path library copied from the original stack.

You can build it with:

```bash
cd /root/local_lidar_planner_ws
colcon build --packages-select local_lidar_planner
source install/setup.bash
ros2 launch local_lidar_planner local_lidar_planner.launch
```

Adjust dependencies or launch arguments to match your hardware (e.g., remap `/registered_scan`, inject your custom waypoint source, etc.).
