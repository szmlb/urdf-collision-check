import pybullet as p
import csv # For reading trajectory CSV files

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


# --- Main Trajectory Analysis Function ---

def run_trajectory_collision_analysis(args, physics_client_id):
    """
    Loads URDFs and trajectories, then checks for collisions at each synchronized timestamp.
    """
    print("Starting trajectory collision analysis...")

    # Load URDFs
    print(f"Loading URDF 1: {args.urdf_file_1}")
    body1_id = load_urdf(args.urdf_file_1, physics_client_id)
    print(f"Loading URDF 2: {args.urdf_file_2}")
    body2_id = load_urdf(args.urdf_file_2, physics_client_id)
    print(f"URDFs loaded. Body1 ID: {body1_id}, Body2 ID: {body2_id}")

    # Load trajectories
    print(f"Loading trajectory 1 from: {args.traj1}")
    trajectory1_data = load_trajectory(args.traj1)
    print(f"Loading trajectory 2 from: {args.traj2}")
    trajectory2_data = load_trajectory(args.traj2)

    if not trajectory1_data or not trajectory2_data:
        print("Error: One or both trajectories are empty or failed to load.")
        return [] # Return empty results if trajectories are missing

    # Generate master timestamp list
    times1 = {pt['time'] for pt in trajectory1_data}
    times2 = {pt['time'] for pt in trajectory2_data}
    master_timestamps = sorted(list(times1.union(times2)))

    print(f"\nLoaded {len(trajectory1_data)} points for trajectory 1.")
    print(f"Loaded {len(trajectory2_data)} points for trajectory 2.")
    if not master_timestamps:
        print("Error: No common or unique timestamps found to process.")
        return []
    print(f"Generated {len(master_timestamps)} unique timestamps for processing: from {master_timestamps[0]:.2f}s to {master_timestamps[-1]:.2f}s.")

    results_data = []

    playback_start_time = 0
    first_timestamp = 0
    if args.visualize and master_timestamps:
        playback_start_time = time.time()
        first_timestamp = master_timestamps[0]

        # Initial pose setup for visualization
        initial_pos1_viz, initial_orn1_viz = get_pose_at_time(trajectory1_data, first_timestamp)
        initial_pos2_viz, initial_orn2_viz = get_pose_at_time(trajectory2_data, first_timestamp)
        if initial_pos1_viz and initial_orn1_viz:
             p.resetBasePositionAndOrientation(body1_id, initial_pos1_viz, initial_orn1_viz, physicsClientId=physics_client_id)
        if initial_pos2_viz and initial_orn2_viz:
             p.resetBasePositionAndOrientation(body2_id, initial_pos2_viz, initial_orn2_viz, physicsClientId=physics_client_id)
        print("Visualizing initial poses. Playback will attempt to follow trajectory time.")
        # Camera is set once initially by main, can be updated in loop.
        p.stepSimulation(physicsClientId=physics_client_id)


    for t_master in master_timestamps:
        pos1, orn1 = get_pose_at_time(trajectory1_data, t_master)
        pos2, orn2 = get_pose_at_time(trajectory2_data, t_master)

        if pos1 is None or orn1 is None or pos2 is None or orn2 is None:
            print(f"Warning: Could not retrieve pose for one or both objects at time {t_master:.2f}. Skipping this timestamp.")
            # Optionally, append a row with error/NaN or skip, decided to skip.
            continue

        p.resetBasePositionAndOrientation(body1_id, pos1, orn1, physicsClientId=physics_client_id)
        p.resetBasePositionAndOrientation(body2_id, pos2, orn2, physicsClientId=physics_client_id)

        is_coll, dist = check_collision(body1_id, body2_id, physics_client_id)
        results_data.append({'time': t_master, 'is_collision': 1 if is_coll else 0, 'closest_distance': dist})

        if args.visualize:
            print(f"Time: {t_master:.2f}s, Collision: {'Yes' if is_coll else 'No'}, Distance: {dist:.4f}")

            # Camera tracking object 1 (body1_id)
            if body1_id is not None: # Ensure body1_id is valid
                current_pos1, _ = p.getBasePositionAndOrientation(body1_id, physicsClientId=physics_client_id)
                p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-35,
                                             cameraTargetPosition=current_pos1, physicsClientId=physics_client_id)

            p.stepSimulation(physicsClientId=physics_client_id)

            expected_elapsed_real_time = t_master - first_timestamp
            actual_elapsed_real_time = time.time() - playback_start_time
            sleep_duration = expected_elapsed_real_time - actual_elapsed_real_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            # If execution is slower than real-time, it will just run as fast as possible.

    print(f"\nTrajectory processing complete. {len(results_data)} points processed.")

    # The main script's finally block handles disconnection.
    # The post-trajectory visualization loop is handled in the main script block.
    return results_data

# --- CSV Writing Function ---

def write_results_to_csv(results_data, output_filepath):
    """
    Writes the collision check results to a CSV file.
    results_data: List of dictionaries, each with 'time', 'is_collision', 'closest_distance'.
    """
    if not results_data:
        print("No results data to write to CSV.")
        # Optionally, create an empty CSV with headers if that's desired
        # For now, just return if no data.
        return

    fieldnames = ['time', 'is_collision', 'closest_distance']
    try:
        with open(output_filepath, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results_data)
        print(f"Results successfully written to {output_filepath}")
    except IOError:
        # More specific exceptions could be caught, e.g., PermissionError
        print(f"Error: Could not write results to {output_filepath}. Check path and permissions.")
    except Exception as e:
        print(f"An unexpected error occurred while writing CSV: {e}")


# --- Trajectory Processing Functions ---

def load_trajectory(filepath):
    """
    Loads trajectory data from a CSV file.
    Expected CSV format: time,x,y,z,qx,qy,qz,qw
    """
    trajectory_points = []
    try:
        with open(filepath, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader) # Skip header row

            # Basic validation of header (optional, but good practice)
            expected_header = ['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
            if header != expected_header:
                print(f"Warning: CSV header for {filepath} is '{header}', expected '{expected_header}'.")
                # Depending on strictness, could raise an error or try to proceed.

            for row_num, row in enumerate(reader, 2): # Start row_num from 2 (since header is 1st)
                if len(row) != 8:
                    print(f"Error: File {filepath}, row {row_num}: Expected 8 columns, found {len(row)}. Skipping row.")
                    continue
                try:
                    point = {
                        'time': float(row[0]),
                        'pos': [float(row[1]), float(row[2]), float(row[3])],
                        'orn': [float(row[4]), float(row[5]), float(row[6]), float(row[7])]
                    }
                    trajectory_points.append(point)
                except ValueError as e:
                    print(f"Error: File {filepath}, row {row_num}: Could not convert value to float - {e}. Skipping row.")
                    continue

        # Sort by time to ensure correct order for interpolation, though usually pre-sorted.
        trajectory_points.sort(key=lambda p: p['time'])
        return trajectory_points

    except FileNotFoundError:
        print(f"Error: Trajectory file not found: {filepath}")
        raise
    except Exception as e:
        print(f"Error loading trajectory from {filepath}: {e}")
        raise

def get_pose_at_time(trajectory_data, current_time):
    """
    Finds the pose from trajectory_data at or just before current_time.
    Uses last-known-pose interpolation (zero-order hold).
    Returns (pos, orn) or (None, None) if trajectory_data is empty.
    """
    if not trajectory_data:
        return None, None

    # If current_time is before the first point in the trajectory, use the first point's pose.
    if current_time < trajectory_data[0]['time']:
        return trajectory_data[0]['pos'], trajectory_data[0]['orn']

    # Iterate backwards through the trajectory_data
    # to find the most recent point at or before current_time.
    for point in reversed(trajectory_data):
        if point['time'] <= current_time:
            return point['pos'], point['orn']

    # This part should ideally not be reached if current_time >= trajectory_data[0]['time']
    # and trajectory_data is sorted and non-empty.
    # However, as a fallback or for an unexpected case, return the first pose.
    # (Or handle as an error, e.g., raise ValueError("Time out of bounds and logic error in get_pose_at_time"))
    return trajectory_data[0]['pos'], trajectory_data[0]['orn']

if __name__ == '__main__':
  import argparse # argparse is already imported in the original file
  import time # time import for visualize loop, if sleep is used.

  parser = argparse.ArgumentParser(description="Check for collision between two URDF models over trajectories.")
  parser.add_argument("urdf_file_1", help="Path to the first URDF file.")
  parser.add_argument("urdf_file_2", help="Path to the second URDF file.")

  # New trajectory and output arguments
  parser.add_argument('--traj1', type=str, required=True,
                      help='Path to the CSV trajectory file for the first object (format: time,x,y,z,qx,qy,qz,qw).')
  parser.add_argument('--traj2', type=str, required=True,
                      help='Path to the CSV trajectory file for the second object (format: time,x,y,z,qx,qy,qz,qw).')
  parser.add_argument('--output', type=str, required=True,
                      help='Path for the output CSV results file (format: time,is_collision,closest_distance).')

  # Keep visualize flag
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
    # Run the main trajectory collision analysis
    results_data = run_trajectory_collision_analysis(args, physics_client_id)

    # Write results to CSV
    if results_data: # Only write if there are results
        write_results_to_csv(results_data, args.output)
    else:
        print("\nNo results were generated to write to output file.")

    # If in visualize mode, the run_trajectory_collision_analysis function handles stepping.
    # After it finishes, if visualize is still true, we might want to keep the window open
    # to inspect the final state, or just let it proceed to disconnect.
    # The current run_trajectory_collision_analysis loop steps through the whole trajectory.
    # If visualize is true, it prints status and steps.
    # The additional loop below is for keeping the window open *after* trajectory processing.
    if args.visualize:
      print("\nTrajectory visualization complete. GUI will remain active.")
      print("Close the PyBullet window to exit or press Ctrl+C in terminal.")
      try:
        # Keep window open until user closes it
        while p.isConnected(physics_client_id):
          p.stepSimulation(physicsClientId=physics_client_id)
          # A small sleep helps keep the GUI responsive without burning CPU
          time.sleep(1./240.)
      except p.error:
        print("PyBullet window closed by user.")

  except Exception as e:
    print(f"An error occurred: {e}")
  finally:
    if p.isConnected(physics_client_id):
      p.disconnect(physics_client_id)
