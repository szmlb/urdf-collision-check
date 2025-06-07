import pybullet as p

def load_urdf(urdf_path, physics_client_id): # Added physics_client_id
  """Loads a URDF model into PyBullet using a specific physics client.

  Args:
    urdf_path: Path to the URDF file.
    physics_client_id: The ID of the PyBullet physics client to use.

  Returns:
    The body unique ID of the loaded model.
  """
  if not p.isConnected(physics_client_id):
      raise RuntimeError(f"Physics client {physics_client_id} not connected for load_urdf.")
  # PyBullet's loadURDF uses the "current" client implicitly.
  # For calls that take physicsClientId, it should be used.
  # Here, we assume the context (test or main script) has set the correct client.
  model_id = p.loadURDF(urdf_path)
  return model_id

def check_collision(body_id_1, body_id_2, physics_client_id): # physics_client_id already added
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
  # All PyBullet calls that interact with the simulation should specify the client.
  contact_points = p.getClosestPoints(bodyA=body_id_1, bodyB=body_id_2, distance=10.0, physicsClientId=physics_client_id)

  if not contact_points:
    return False, float('inf') # No points found, bodies are far apart

  # Find the minimum distance among all contact points
  # The 8th element in each contact point tuple is the contact distance.
  closest_distance = min(point[8] for point in contact_points)

  is_collision = closest_distance <= 0

  return is_collision, closest_distance

def perform_collision_check_logic(urdf_path1, pos1, orn1, urdf_path2, pos2, orn2, physics_client_id):
  """
  Loads two URDF models, sets their poses, and checks for collision.
  This is the core logic, designed to be callable from tests or other scripts.
  Assumes PyBullet client (physics_client_id) is connected and managed by the caller.
  """
  if not p.isConnected(physics_client_id):
      raise RuntimeError(f"Physics client {physics_client_id} not connected for perform_collision_check_logic.")

  model_id_1 = load_urdf(urdf_path1, physics_client_id)
  p.resetBasePositionAndOrientation(model_id_1, pos1, orn1, physicsClientId=physics_client_id)

  model_id_2 = load_urdf(urdf_path2, physics_client_id)
  p.resetBasePositionAndOrientation(model_id_2, pos2, orn2, physicsClientId=physics_client_id)

  is_collision, closest_distance = check_collision(model_id_1, model_id_2, physics_client_id)

  # Return model IDs as well, they might be useful for debugging or advanced tests
  return is_collision, closest_distance, model_id_1, model_id_2

if __name__ == '__main__':
  import argparse

  parser = argparse.ArgumentParser(description="Check for collision between two URDF models.")
  parser.add_argument("urdf_file_1", help="Path to the first URDF file.")
  parser.add_argument("--pos1", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                      help="Position (X Y Z) for the first URDF model. Default: 0 0 0")
  parser.add_argument("--orn1", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0],
                      help="Orientation quaternion (X Y Z W) for the first URDF model. Default: 0 0 0 1")

  parser.add_argument("urdf_file_2", help="Path to the second URDF file.")
  parser.add_argument("--pos2", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                      help="Position (X Y Z) for the second URDF model. Default: 0 0 0")
  parser.add_argument("--orn2", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0],
                      help="Orientation quaternion (X Y Z W) for the second URDF model. Default: 0 0 0 1")
  parser.add_argument('--visualize', action='store_true', help='Enable GUI visualization of the URDFs and collision check.')

  args = parser.parse_args()

  # Connect to PyBullet: p.GUI if visualize flag is set, otherwise p.DIRECT
  connection_mode = p.GUI if args.visualize else p.DIRECT
  physics_client_id = p.connect(connection_mode) # Main script connects once

  if args.visualize:
      # You might want to configure the camera or add other GUI elements here
      # For example, set a camera distance and orientation
      p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0.5,0.5,0.5], physicsClientId=physics_client_id)
      print("GUI visualization enabled. Close the PyBullet window to exit.")

  try:
    # Call the refactored logic function
    is_collision, closest_distance, model_id_1, model_id_2 = perform_collision_check_logic(
        args.urdf_file_1, args.pos1, args.orn1,
        args.urdf_file_2, args.pos2, args.orn2,
        physics_client_id
    )

    # Output results
    print(f"Loaded model 1 ('{args.urdf_file_1}') with ID: {model_id_1} at pos: {args.pos1}, orn: {args.orn1}")
    print(f"Loaded model 2 ('{args.urdf_file_2}') with ID: {model_id_2} at pos: {args.pos2}, orn: {args.orn2}")
    print("\n--- Collision Check Result ---")
    if is_collision:
      print("Collision DETECTED!")
    else:
      print("No collision detected.")
    print(f"Closest distance: {closest_distance:.4f} units")

    if args.visualize:
      print("\nGUI mode active. Close the PyBullet window to exit or press Ctrl+C in terminal.")
      try:
        while p.isConnected(physics_client_id):
          # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) # Optional
          p.stepSimulation(physicsClientId=physics_client_id)
          # time.sleep(1./240.) # Keep this commented unless specific sim frequency is needed
      except p.error:
        # This exception can occur if the window is closed by the user
        print("PyBullet window closed by user.")
      # The main finally block will handle disconnection.

  except Exception as e:
    print(f"An error occurred: {e}")
  finally:
    if p.isConnected(physics_client_id):
      p.disconnect(physics_client_id)
