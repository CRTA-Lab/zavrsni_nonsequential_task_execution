# Non-sequential task description

This ROS 2 package contains the robot-description files used during the
preparation of the UR3e and UR5e models for the thesis:

**Non-sequential Task Execution with Two Industrial Robots**

## Included models

- `urdf/ur3e_visible_mesh.urdf`
- `urdf/ur5e_visible_mesh.urdf`

## Purpose

The URDF files were used to import and prepare visible UR3e and UR5e robot
models in CoppeliaSim.

## URDF import problem

The original Universal Robots descriptions referenced meshes through paths
such as:

`package://ur_description/...`

CoppeliaSim did not resolve these package paths during direct URDF import.

The generated URDF files were therefore adapted to use file paths pointing to
the installed ROS 2 Humble `ur_description` package, for example:

`file:///opt/ros/humble/share/ur_description/...`

## Third-party material

The Universal Robots geometry, meshes and original robot-description resources
are third-party material and remain subject to their original licenses.

Damjan Drobac prepared and adapted the files for integration into this thesis
project.

## Complete preparation and import instructions

See [URDF model preparation](../../../docs/urdf_models.md).
