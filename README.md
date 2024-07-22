# ROSCon 2024 Workshop: Manipulate With MoveIt Like a Pro

TODO

# Build

Build the image with (TODO):

```bash
docker build -t roscon-2024-moveit2-workshop .
```

Start with:
```bash
./docker/start.sh
```

Start a shell session with:
```bash
./docker/shell
```

# Run UR
If we want to use a UR, we should probably need to wrap this to make it easier to type.
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=0.0.0.0 use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
