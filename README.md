# URDF Collision Checker

## Project Overview

This project provides a command-line tool to check for collisions between two robotic links or objects defined by URDF (Unified Robot Description Format) files. It loads the two specified URDF models into a PyBullet physics simulation, sets their positions and orientations as provided by the user, and then performs a collision check. The tool reports whether a collision is detected and also provides the closest distance between the two objects.

## Features

*   Loads two URDF models.
*   Allows user to specify the 3D position (X, Y, Z) and orientation (quaternion X, Y, Z, W) for each model via command-line arguments.
*   Detects collisions between the specified models.
*   Calculates and reports the minimum distance between the two models.
*   Supports non-convex shapes for collision detection, provided they are defined as mesh geometries within the URDF's `<collision>` tag.

## Requirements

*   Python 3.x
*   Libraries:
    *   `pybullet`
    *   `numpy`

    These can be installed by using the `requirements.txt` file.

## Setup

1.  Clone the repository (if applicable).
2.  Install the required Python libraries:
    ```bash
    pip install -r requirements.txt
    ```

## Usage

The script is run from the command line with paths to the two URDF files and optional arguments for their poses.

### Syntax

```bash
python collision_checker.py <urdf_file_1> <urdf_file_2> [--pos1 X Y Z] [--orn1 QX QY QZ QW] [--pos2 X Y Z] [--orn2 QX QY QZ QW]
```

### Arguments

*   `urdf_file_1`: Path to the first URDF file.
*   `urdf_file_2`: Path to the second URDF file.
*   `--pos1 X Y Z`: (Optional) Position (X, Y, Z coordinates) for the first object. Defaults to `0 0 0`.
*   `--orn1 QX QY QZ QW`: (Optional) Orientation (quaternion X, Y, Z, W) for the first object. Defaults to `0 0 0 1` (no rotation).
*   `--pos2 X Y Z`: (Optional) Position for the second object. Defaults to `0 0 0`.
*   `--orn2 QX QY QZ QW`: (Optional) Orientation (quaternion) for the second object. Defaults to `0 0 0 1`.

### Examples

1.  Check collision between a sphere and a box, placing them slightly separated along the Z-axis:
    ```bash
    python collision_checker.py urdf_files/sphere.urdf urdf_files/box.urdf --pos1 0 0 0.6 --pos2 0 0 -0.6
    ```
    (Assuming sphere radius is 0.5 and box half-height is 0.5, this should result in no collision with a distance of 0.2)

2.  Check collision between the L-shaped object and a sphere, placing the sphere within the L-shape's concave region and rotating the L-shape 90 degrees around Z-axis:
    ```bash
    python collision_checker.py urdf_files/l_shape.urdf urdf_files/sphere.urdf --pos1 0 0 0 --orn1 0 0 0.7071 0.7071 --pos2 0.5 0.5 0.5
    ```

## Running Tests

Unit tests are provided to verify the functionality of the collision checker, including pose specification and non-convex shape collisions. To run the tests:

```bash
python -m unittest discover tests
```
Alternatively, you can run the test file directly:
```bash
python tests/test_collision_checker.py
```

## Notes on URDFs and Collision Geometry

*   The collision geometry for an object is defined within its URDF file using the `<collision>` tag. This geometry is what PyBullet uses for collision detection calculations.
*   Simple primitive shapes (box, sphere, cylinder, capsule) can be directly defined within the `<geometry>` tag inside `<collision>`.
*   For non-convex shapes or more complex models, you should use a mesh file. This is specified using the `<mesh filename="path/to/your/mesh.obj"/>` element within `<collision><geometry>`. Supported mesh formats include OBJ, STL, and Collada (DAE). The provided `l_shape.urdf` and `l_shape.obj` files demonstrate this approach for a non-convex L-shape. Ensure that the mesh normals are defined correctly for accurate collision detection.

---
This README provides a comprehensive guide for users and developers of the URDF Collision Checker.
