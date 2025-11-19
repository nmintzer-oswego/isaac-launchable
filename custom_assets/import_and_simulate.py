#!/usr/bin/env python3

"""
Official Isaac Sim Python API example for importing and simulating custom assets.
This script demonstrates the standard workflow for loading URDF/USD assets.

Usage:
    ./isaaclab.sh -p /workspace/custom_assets/import_and_simulate.py \
        --kit_args="--no-window --enable omni.kit.livestream.webrtc"
"""

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.asset.importer.urdf import _urdf
import omni.kit.commands
import carb
import os

def import_urdf_to_usd(urdf_path: str, usd_output_path: str = None):
    """
    Import a URDF file and convert it to USD format using the official API.

    Args:
        urdf_path: Path to the URDF file
        usd_output_path: Optional output path for USD file (defaults to same dir as URDF)

    Returns:
        Tuple of (success, usd_path)
    """
    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF file not found: {urdf_path}")
        return False, None

    # Default USD output path
    if usd_output_path is None:
        usd_output_path = urdf_path.replace('.urdf', '.usd')

    try:
        # Configure import settings
        import_config = _urdf.ImportConfig()
        import_config.fix_base = True  # Set to False for mobile robots
        import_config.make_instanceable = True  # Important for performance
        import_config.merge_fixed_joints = True
        import_config.convex_decomp = False
        import_config.self_collision = False
        import_config.density = 0.0  # Use default material densities

        # Parse the URDF file
        result, robot_model = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=import_config
        )

        if not result:
            carb.log_error("Failed to parse URDF file")
            return False, None

        # Import robot to stage
        result, prim_path = omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_robot=robot_model,
            import_config=import_config,
            stage_path="/World/ImportedRobot",
            dest_path=usd_output_path
        )

        if result:
            carb.log_info(f"Successfully imported URDF to: {usd_output_path}")
            return True, usd_output_path
        else:
            carb.log_error("Failed to import URDF to USD")
            return False, None

    except Exception as e:
        carb.log_error(f"Error during URDF import: {str(e)}")
        return False, None

def main():
    """Main simulation function"""

    # Initialize Isaac Sim world
    world = World(stage_units_in_meters=1.0)

    # Example paths - update these to your actual asset paths
    robot_usd_path = "/workspace/custom_assets/robots/my_robot.usd"
    robot_urdf_path = "/workspace/custom_assets/robots/my_robot.urdf"
    environment_usd_path = "/workspace/custom_assets/environments/warehouse.usd"

    # Option 1: Load pre-converted USD files directly
    if os.path.exists(robot_usd_path):
        carb.log_info(f"Loading robot from USD: {robot_usd_path}")
        add_reference_to_stage(
            usd_path=robot_usd_path,
            prim_path="/World/Robot"
        )
    # Option 2: Import URDF on the fly if USD doesn't exist
    elif os.path.exists(robot_urdf_path):
        carb.log_info(f"URDF found, converting to USD: {robot_urdf_path}")
        success, usd_path = import_urdf_to_usd(robot_urdf_path, robot_usd_path)
        if success:
            add_reference_to_stage(
                usd_path=usd_path,
                prim_path="/World/Robot"
            )
    else:
        carb.log_warn("No robot asset found. Place your robot files in:")
        carb.log_warn("  - USD: /workspace/custom_assets/robots/my_robot.usd")
        carb.log_warn("  - URDF: /workspace/custom_assets/robots/my_robot.urdf")

    # Load environment if available
    if os.path.exists(environment_usd_path):
        carb.log_info(f"Loading environment from: {environment_usd_path}")
        add_reference_to_stage(
            usd_path=environment_usd_path,
            prim_path="/World/Environment"
        )
    else:
        carb.log_info("No environment found, using default ground plane")
        world.scene.add_default_ground_plane()

    # Reset the world to initialize physics
    world.reset()

    # Simulation loop
    carb.log_info("Simulation started. View at http://localhost/viewer")
    carb.log_info("Press Ctrl+C to stop")

    try:
        while True:
            world.step(render=True)
    except KeyboardInterrupt:
        carb.log_info("Simulation stopped by user")

if __name__ == "__main__":
    main()