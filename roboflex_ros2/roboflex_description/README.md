# roboflex_description

Description/assets package for RoboFlex.

## Purpose

- Owns URDF/Xacro robot description.
- Owns robot meshes and simulation model/world assets.
- No bringup logic lives here (moved to `roboflex_control` and `roboflex_bringup`).

## Contents

- `urdf/`: robot URDF/Xacro files
- `meshes/`: mesh files used by URDF
- `models/`: Gazebo models used by simulation world
- `worlds/`: Gazebo world files
- `media/`: simulation media assets

## Related Packages

- Bringup: [`../roboflex_bringup`](../roboflex_bringup)
- Control: [`../roboflex_control`](../roboflex_control)
- MoveIt: [`../roboflex_moveit`](../roboflex_moveit)
