import unittest
import pybullet as p
import os
import sys

# Add project root to sys.path to allow importing collision_checker
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

from collision_checker import perform_collision_check_logic

class TestCollisionChecker(unittest.TestCase):
    physics_client_id = -1

    @classmethod
    def setUpClass(cls):
        # Connect to PyBullet once for all tests in this class
        cls.physics_client_id = p.connect(p.DIRECT)
        if cls.physics_client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet in setUpClass.")

    @classmethod
    def tearDownClass(cls):
        # Disconnect from PyBullet once after all tests in this class
        if p.isConnected(cls.physics_client_id):
            p.disconnect(cls.physics_client_id)

    def rel_path(self, path_suffix):
        # Helper to get path relative to project root for URDF files
        return os.path.join(project_root, "urdf_files", path_suffix)

class TestPoseSpecification(TestCollisionChecker):

    def test_object_placement_no_collision(self):
        sphere_urdf = self.rel_path("sphere.urdf")
        # Place spheres far apart
        is_collision, distance, _, _ = perform_collision_check_logic(
            sphere_urdf, [0, 0, 0], [0,0,0,1],  # Sphere 1 at origin
            sphere_urdf, [10, 0, 0], [0,0,0,1], # Sphere 2 at (10,0,0)
            self.physics_client_id
        )
        self.assertFalse(is_collision, "Collision detected when objects are far apart.")
        self.assertGreater(distance, 1.0, "Distance should be large when objects are far apart.") # Sphere radius 0.5, so center-to-center 10, surface distance 9

    def test_object_placement_collision(self):
        sphere_urdf = self.rel_path("sphere.urdf")
        # Place spheres overlapping (Sphere 1 at origin, Sphere 2 also at origin)
        is_collision, distance, _, _ = perform_collision_check_logic(
            sphere_urdf, [0, 0, 0], [0,0,0,1],
            sphere_urdf, [0.1, 0, 0], [0,0,0,1], # Slightly offset for clear overlap
            self.physics_client_id
        )
        self.assertTrue(is_collision, "No collision detected when objects are overlapping.")
        self.assertLessEqual(distance, 0.0, "Distance should be <= 0 for colliding objects.")

class TestNonConvexCollisions(TestCollisionChecker):

    def test_lshape_sphere_collision_concave_area(self):
        l_shape_urdf = self.rel_path("l_shape.urdf")
        sphere_urdf = self.rel_path("sphere.urdf")

        # L-shape vertical arm: X[0,1], Y[0,2], Z[0,1]
        # L-shape horizontal arm: X[1,2], Y[0,1], Z[0,1]
        # Place sphere (radius 0.5) inside the "elbow" of L-shape
        # e.g., sphere center at (0.5, 0.5, 0.5) which is inside vertical arm
        is_collision, distance, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],           # L-shape at origin
            sphere_urdf, [0.5, 0.5, 0.5], [0,0,0,1],    # Sphere in concave region
            self.physics_client_id
        )
        self.assertTrue(is_collision, "No collision detected for L-shape and sphere in concave area.")

    def test_lshape_sphere_no_collision_near_concave(self):
        l_shape_urdf = self.rel_path("l_shape.urdf")
        sphere_urdf = self.rel_path("sphere.urdf")
        # Place sphere near L-shape's elbow but not touching
        # L-shape Y for vertical part is [0,2]. Horizontal part Y is [0,1]
        # Place sphere at X=0.5 (center of vertical part's width), Y=2.0 (at edge of vertical part), Z=0.5
        # Sphere radius 0.5. L-shape vertical arm Y extent is 2.0.
        # If sphere center is at Y=2.5, it should not collide.
        is_collision, distance, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],        # L-shape at origin
            sphere_urdf, [0.5, 2.5, 0.5], [0,0,0,1], # Sphere just outside Y-extent of vertical arm
            self.physics_client_id
        )
        self.assertFalse(is_collision, "Collision detected for L-shape and sphere near concave area (should be no collision).")
        self.assertGreater(distance, 0.0, "Distance should be > 0 for non-colliding objects.")


    def test_lshape_box_collision(self):
        l_shape_urdf = self.rel_path("l_shape.urdf")
        box_urdf = self.rel_path("box.urdf") # Box is 1x1x1

        # Place box to collide with the horizontal arm of L-shape
        # Horizontal arm: X[1,2], Y[0,1], Z[0,1]
        # Place box center at (1.5, 0.5, 0.5) - this is center of horizontal arm
        is_collision, distance, _, _ = perform_collision_check_logic(
            l_shape_urdf, [0,0,0], [0,0,0,1],      # L-shape at origin
            box_urdf, [1.5, 0.5, 0.5], [0,0,0,1],  # Box overlapping with horizontal arm
            self.physics_client_id
        )
        self.assertTrue(is_collision, "No collision detected for L-shape and box colliding.")

if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)
