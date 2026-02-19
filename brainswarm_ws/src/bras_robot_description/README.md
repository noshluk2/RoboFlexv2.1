# bras_robot_description

Simulation assets package used by RoboFlex Gazebo bringup.

## Use This README When

- You are editing simulation world files, models, or environment assets.
- You need to know where Gazebo world content is loaded from.

## Contents

- `worlds/simulation_world_new.sdf`: default simulation world
- `models/`: SDF models used in the world
- `urdf/`: additional robot description artifacts
- `config/`: simulation-side configuration

This package is loaded automatically by:

- [`../roboflex_description/launch/simulation.launch.py`](../roboflex_description/launch/simulation.launch.py)

For full simulation instructions, see:

- [`../roboflex_description/SIMULATION.md`](../roboflex_description/SIMULATION.md)

For project-level documentation routing, see:

- [`../../../README.md`](../../../README.md)
