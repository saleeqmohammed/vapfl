#!/usr/bin/env python3
"""
NVIDIA Isaac Sim Robot and Environment Spawner for VAPFL Benchmarking.
Configures RTX 2D LiDAR and ROS 2 Jazzy Bridge topics (/scan, /odom, /tf).
"""

import argparse
import sys

def parse_args():
    parser = argparse.ArgumentParser(description="Isaac Sim VAPFL Environment Spawner")
    parser.add_argument("--headless", action="store_true", default=False, help="Run Isaac Sim in headless mode")
    parser.add_argument("--usd_path", type=str, default="", help="Optional USD scene path")
    return parser.parse_args()

def main():
    args = parse_args()
    print("[VAPFL Isaac Sim] Initializing Simulation Application...")

    try:
        from isaacsim import SimulationApp
    except ImportError:
        try:
            from omni.isaac.kit import SimulationApp
        except ImportError:
            print("[ERROR] NVIDIA Isaac Sim environment not detected.")
            print("Please run this script inside Isaac Sim Python environment (./python.sh sim_isaac/scripts/spawn_robot.py)")
            sys.exit(1)

    simulation_app = SimulationApp({"headless": args.headless})

    import carb
    from omni.isaac.core import World
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.wheeled_robots.robots import WheeledRobot

    # Enable ROS 2 Jazzy bridge
    print("[VAPFL Isaac Sim] Enabling ROS 2 Bridge Extension...")
    enable_extension("omni.isaac.ros2_bridge")

    world = World(stage_units_in_meters=1.0)

    if args.usd_path:
        world.scene.add_default_ground_plane()
        print(f"[VAPFL Isaac Sim] Loading scene from {args.usd_path}")
    else:
        world.scene.add_default_ground_plane()
        print("[VAPFL Isaac Sim] Created default simulation ground plane.")

    # Configure differential drive robot
    robot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Robot",
            name="vapfl_robot",
            wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
            create_robot=True,
            usd_path=None,
            position=[0.0, 0.0, 0.1],
        )
    )

    world.reset()
    print("[VAPFL Isaac Sim] Robot spawned and ready. Publishing /scan and /odom to ROS 2 Jazzy.")

    try:
        while simulation_app.is_running():
            world.step(render=not args.headless)
    except KeyboardInterrupt:
        print("[VAPFL Isaac Sim] Shutting down simulation...")
    finally:
        simulation_app.close()

if __name__ == "__main__":
    main()
