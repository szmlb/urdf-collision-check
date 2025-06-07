import pybullet as p

def load_urdf(urdf_path):
  """Loads a URDF model into PyBullet.

  Args:
    urdf_path: Path to the URDF file.

  Returns:
    The body unique ID of the loaded model.
  """
  # Ensure PyBullet is connected.
  # The connection should be managed by the main script or a dedicated class.
  if not p.isConnected():
    raise RuntimeError("PyBullet not connected. Call p.connect() before loading URDFs.")

  model_id = p.loadURDF(urdf_path)
  return model_id

def check_collision(body_id_1, body_id_2):
  """Checks for collision between two bodies.

  Args:
    body_id_1: Unique ID of the first body.
    body_id_2: Unique ID of the second body.

  Returns:
    A tuple (is_collision, closest_distance).
    is_collision (bool): True if a collision is detected, False otherwise.
    closest_distance (float): The minimum distance between the two bodies.
                              float('inf') if they are further apart than the detection threshold.
  """
  if not p.isConnected():
    raise RuntimeError("PyBullet not connected. Call p.connect() before checking collision.")

  contact_points = p.getClosestPoints(bodyA=body_id_1, bodyB=body_id_2, distance=10.0) # Max distance to check

  if not contact_points:
    return False, float('inf') # No points found, bodies are far apart

  # Find the minimum distance among all contact points
  # The 8th element in each contact point tuple is the contact distance.
  closest_distance = min(point[8] for point in contact_points)

  is_collision = closest_distance <= 0

  return is_collision, closest_distance

if __name__ == '__main__':
  import argparse

  parser = argparse.ArgumentParser(description="Check for collision between two URDF models.")
  parser.add_argument("urdf_file_1", help="Path to the first URDF file.")
  parser.add_argument("urdf_file_2", help="Path to the second URDF file.")

  args = parser.parse_args()

  physics_client_id = p.connect(p.DIRECT) # Connect once

  try:
    model_id_1 = load_urdf(args.urdf_file_1)
    print(f"Loaded model 1 ('{args.urdf_file_1}') with ID: {model_id_1}")

    model_id_2 = load_urdf(args.urdf_file_2)
    print(f"Loaded model 2 ('{args.urdf_file_2}') with ID: {model_id_2}")

    # For meaningful collision checking, objects should be positioned.
    # By default, loadURDF loads objects at the origin.
    # We can add options to set positions or use fixed positions for now.
    # Example: Move the second object slightly to avoid immediate collision at origin if they are identical.
    # p.resetBasePositionAndOrientation(model_id_2, [1, 0, 0], [0,0,0,1])


    is_collision, closest_distance = check_collision(model_id_1, model_id_2)

    print("\n--- Collision Check Result ---")
    if is_collision:
      print("Collision DETECTED!")
    else:
      print("No collision detected.")
    print(f"Closest distance: {closest_distance:.4f} units")

  except Exception as e:
    print(f"An error occurred: {e}")
  finally:
    if p.isConnected(physics_client_id):
      p.disconnect(physics_client_id)
