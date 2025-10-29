# Grasping Workspace

This is my workspace for dexterous grasping.

## Run

```bash
ros2 launch grasping_control real_prepare.launch.py
ros2 run grasping_perception object_pointcloud_publisher
ros2 run grasping_control main_grasping.py
```