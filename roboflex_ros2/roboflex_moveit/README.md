# roboflex_moveit

MoveIt 2 configuration package for RoboFlex.

## Purpose

- Owns MoveIt semantic/planning configuration (SRDF, kinematics, joint limits).
- Owns MoveIt launches (`move_group`, MoveIt RViz, setup assistant, demo).
- Owns MoveIt controller mapping (`moveit_controllers.yaml`).

## Canonical Launches

```bash
ros2 launch roboflex_moveit move_group.launch.py
ros2 launch roboflex_moveit moveit_rviz.launch.py
```

Real hardware (no simulation clock):

```bash
ros2 launch roboflex_moveit move_group.launch.py use_sim_time:=false
ros2 launch roboflex_moveit moveit_rviz.launch.py use_sim_time:=false
```

Combined hardware stack (control + MoveIt + RViz) is provided by bringup:

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py
```

## Key Config Files

- `config/roboflex_description.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/moveit_controllers.yaml`
