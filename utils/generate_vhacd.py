import pybullet as p
import argparse
import os
import sys

def generate_vhacd(input_file, output_file, log_file="vhacd_log.txt", resolution=100000):
    """
    Generates a convex decomposition of a mesh using PyBullet's vhacd.
    """
    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found.")
        return False

    print(f"Starting VHACD decomposition for {input_file}...")
    print(f"Output will be saved to {output_file}")
    print(f"Log will be saved to {log_file}")

    # Check if we are connected, if not, connect to DIRECT
    if not p.isConnected():
        p.connect(p.DIRECT)

    try:
        # p.vhacd returns None, it prints to stdout/log
        p.vhacd(input_file, output_file, log_file, resolution=resolution)
        if os.path.exists(output_file):
            print(f"Successfully created {output_file}")
            return True
        else:
            print(f"Error: Output file {output_file} was not created.")
            return False
    except Exception as e:
        print(f"An error occurred during VHACD: {e}")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate convex decomposition (VHACD) for a mesh file.")
    parser.add_argument("input_mesh", help="Path to the input mesh file (e.g., .obj).")
    parser.add_argument("output_mesh", help="Path to the output mesh file (e.g., .obj).")
    parser.add_argument("--resolution", type=int, default=100000, help="Voxel resolution (default: 100000).")
    parser.add_argument("--log", default="vhacd_log.txt", help="Path to log file.")

    args = parser.parse_args()

    success = generate_vhacd(args.input_mesh, args.output_mesh, args.log, args.resolution)
    if not success:
        sys.exit(1)
