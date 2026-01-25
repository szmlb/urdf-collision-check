# Instructions for Agents

## Project Overview
This project is a URDF collision checker using PyBullet. It checks for collisions between two objects moving along trajectories.

## Running Tests
To run the full test suite:
```bash
python -m unittest discover tests
```
Ensure you have installed dependencies from `requirements.txt`.

## Project Structure
- `collision_checker.py`: Main script and logic.
- `urdf_files/`: Contains URDF models and meshes.
  - **Note on Non-Convex Meshes**: PyBullet's `loadURDF` creates convex hulls for meshes by default. For concave objects (like `l_shape.urdf`), use a decomposed mesh (generated via VHACD) referenced in the `<collision>` section of the URDF. See `l_shape.urdf` and `l_shape_vhacd.obj` for an example.
- `tests/`: Unit tests.
- `trajectories/`: Example trajectory CSV files.

## Coding Standards
- Follow PEP 8.
- Ensure all new features have accompanying tests.
- When modifying URDFs or meshes, verify collision shapes using the visualization mode (`--visualize`) or specific tests.

## Known Issues / Tips
- If you add a new concave URDF, you MUST generate a convex decomposition (e.g., using `p.vhacd` or Blender) and update the URDF to point to it for collision. Do not rely on `loadURDF` to handle concave meshes automatically for dynamic objects.
