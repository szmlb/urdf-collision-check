import unittest
import pybullet as p
import os
import sys
import csv # For reading CSV files in tests
import argparse # For creating mock args Namespace for run_trajectory_collision_analysis

# Add project root to sys.path to allow importing collision_checker
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

from collision_checker import (
    perform_collision_check_logic,
    load_trajectory,
    get_pose_at_time,
    run_trajectory_collision_analysis,
    write_results_to_csv
)

class TestCollisionChecker(unittest.TestCase):
    # Base class for tests that need a PyBullet connection
    physics_client_id = -1

    @classmethod
    def setUpClass(cls):
        cls.physics_client_id = p.connect(p.DIRECT)
        if cls.physics_client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet in setUpClass.")

    @classmethod
    def tearDownClass(cls):
        if p.isConnected(cls.physics_client_id):
            p.disconnect(cls.physics_client_id)

    def rel_path_urdf(self, urdf_filename):
        return os.path.join(project_root, "urdf_files", urdf_filename)

    def rel_path_test_data(self, data_filename):
        return os.path.join(os.path.dirname(__file__), "test_data", data_filename)

class TestPoseSpecification(TestCollisionChecker):

    def test_object_placement_no_collision(self):
        sphere_urdf = self.rel_path_urdf("sphere.urdf")
        is_collision, distance, _, _ = perform_collision_check_logic(
            sphere_urdf, [0,0,0], [0,0,0,1],
            sphere_urdf, [10,0,0], [0,0,0,1],
            self.physics_client_id
        )
        self.assertFalse(is_collision)
        self.assertGreater(distance, 1.0)

    def test_object_placement_collision(self):
        sphere_urdf = self.rel_path_urdf("sphere.urdf")
        is_collision, distance, _, _ = perform_collision_check_logic(
            sphere_urdf, [0,0,0], [0,0,0,1],
            sphere_urdf, [0.1,0,0], [0,0,0,1],
            self.physics_client_id
        )
        self.assertTrue(is_collision)
        self.assertLessEqual(distance, 0.0)

class TestNonConvexCollisions(TestCollisionChecker):

    def test_lshape_sphere_collision_concave_area(self):
        l_shape_urdf = self.rel_path_urdf("l_shape.urdf")
        sphere_urdf = self.rel_path_urdf("sphere.urdf")
        is_collision, _, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],
            sphere_urdf, [0.5,0.5,0.5], [0,0,0,1],
            self.physics_client_id
        )
        self.assertTrue(is_collision)

    def test_lshape_sphere_no_collision_near_concave(self):
        l_shape_urdf = self.rel_path_urdf("l_shape.urdf")
        sphere_urdf = self.rel_path_urdf("sphere.urdf")
        is_collision, distance, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],
            sphere_urdf, [0.5,2.5,0.5], [0,0,0,1],
            self.physics_client_id
        )
        self.assertFalse(is_collision)
        self.assertGreater(distance, 0.0)

    def test_lshape_box_collision(self):
        l_shape_urdf = self.rel_path_urdf("l_shape.urdf")
        box_urdf = self.rel_path_urdf("box.urdf")
        is_collision, _, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],
            box_urdf, [1.5,0.5,0.5], [0,0,0,1],
            self.physics_client_id
        )
        self.assertTrue(is_collision)

class TestTrajectoryFunctions(unittest.TestCase):

    def setUp(self):
        self.test_data_dir = os.path.join(os.path.dirname(__file__), "test_data")
        self.sphere_traj_path = os.path.join(self.test_data_dir, "sphere_trajectory.csv")

    def test_load_trajectory_valid(self):
        trajectory = load_trajectory(self.sphere_traj_path)
        self.assertIsInstance(trajectory, list)
        self.assertGreater(len(trajectory), 0)
        first_point = trajectory[0]
        self.assertIn('time', first_point)
        self.assertIn('pos', first_point)
        self.assertIn('orn', first_point)
        self.assertEqual(len(first_point['pos']), 3)
        self.assertEqual(len(first_point['orn']), 4)
        self.assertEqual(first_point['time'], 0.0)
        self.assertEqual(first_point['pos'], [0.0,0.0,0.5])

    def test_load_trajectory_file_not_found(self):
        invalid_path = os.path.join(self.test_data_dir, "non_existent_trajectory.csv")
        with self.assertRaises(FileNotFoundError):
            load_trajectory(invalid_path)

    def test_get_pose_at_time(self):
        sample_traj = [
            {'time': 0.0, 'pos': [0,0,0], 'orn': [0,0,0,1]},
            {'time': 0.1, 'pos': [1,0,0], 'orn': [0,0,0,1]},
            {'time': 0.2, 'pos': [2,0,0], 'orn': [0,0,0,1]},
        ]
        pos, orn = get_pose_at_time(sample_traj, 0.1)
        self.assertEqual(pos, [1,0,0])
        pos, orn = get_pose_at_time(sample_traj, 0.15)
        self.assertEqual(pos, [1,0,0])
        pos, orn = get_pose_at_time(sample_traj, -0.1)
        self.assertEqual(pos, [0,0,0])
        pos, orn = get_pose_at_time(sample_traj, 0.3)
        self.assertEqual(pos, [2,0,0])
        pos, orn = get_pose_at_time([], 0.1)
        self.assertIsNone(pos)
        self.assertIsNone(orn)
        single_point_traj = [{'time': 0.0, 'pos': [0,0,0], 'orn': [0,0,0,1]}]
        pos, orn = get_pose_at_time(single_point_traj, 0.0)
        self.assertEqual(pos, [0,0,0])
        pos, orn = get_pose_at_time(single_point_traj, 0.1)
        self.assertEqual(pos, [0,0,0])

class TestTrajectoryCollision(TestCollisionChecker):

    def compare_csv_files(self, file1_path, file2_path, tolerance=1e-5):
        with open(file1_path, 'r') as f1, open(file2_path, 'r') as f2:
            reader1 = csv.DictReader(f1)
            reader2 = csv.DictReader(f2)
            rows1 = list(reader1)
            rows2 = list(reader2)
            self.assertEqual(len(rows1), len(rows2), f"CSV files have different number of rows: {len(rows1)} vs {len(rows2)}")
            for i, (row1, row2) in enumerate(zip(rows1, rows2)):
                self.assertEqual(float(row1['time']), float(row2['time']), f"Time mismatch in row {i+1}") # Compare times as float
                self.assertEqual(int(row1['is_collision']), int(row2['is_collision']), f"Collision state mismatch in row {i+1} at time {row1['time']}")
                dist1 = float(row1['closest_distance'])
                dist2 = float(row2['closest_distance'])
                self.assertAlmostEqual(dist1, dist2, delta=tolerance, msg=f"Distance mismatch in row {i+1} at time {row1['time']}: {dist1} vs {dist2}")
        return True

    def test_trajectory_collision_output(self):
        urdf1_path = self.rel_path_urdf("sphere.urdf")
        urdf2_path = self.rel_path_urdf("box.urdf")
        traj1_path = self.rel_path_test_data("sphere_trajectory.csv")
        traj2_path = self.rel_path_test_data("box_trajectory.csv")

        temp_output_filename = "temp_sphere_box_results.csv"
        # Ensure temp file is in test_data for cleanup convenience if needed, or use tempfile module
        temp_output_path = self.rel_path_test_data(temp_output_filename)
        expected_output_path = self.rel_path_test_data("expected_sphere_box_results.csv")

        mock_args = argparse.Namespace(
            urdf_file_1=urdf1_path, urdf_file_2=urdf2_path,
            traj1=traj1_path, traj2=traj2_path,
            output=temp_output_path, visualize=False
        )

        results = run_trajectory_collision_analysis(mock_args, self.physics_client_id)
        self.assertTrue(results, "run_trajectory_collision_analysis returned no results.")
        write_results_to_csv(results, temp_output_path)

        self.assertTrue(os.path.exists(temp_output_path), f"Temporary output file {temp_output_path} was not created.")
        self.compare_csv_files(expected_output_path, temp_output_path)

        if os.path.exists(temp_output_path):
            os.remove(temp_output_path)

if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)
