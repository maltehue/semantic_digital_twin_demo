from semantic_digital_twin.adapters.mjcf import MJCFParser
from multiverse_simulator import MultiverseViewer
from semantic_digital_twin.adapters.multi_sim import MujocoSim
import os
import time

if __name__ == "__main__":
    scene_path = os.path.join(
        os.path.dirname(__file__), "..", "assets", "fera", "scene.xml"
    )
    image_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    multiverse_params = {
        "host": "tcp://127.0.0.1",
        "server_port": "7000",
        "client_port": "7501",
        "world_name": "world",
        "simulation_name": "mujoco_sim",
        "send": {
            "body": ["position", "quaternion"],
            "joint": [
                "joint_angular_position",
                "joint_linear_position",
                "joint_angular_velocity",
                "joint_linear_velocity",
                "joint_force",
                "joint_torque",
            ],
        },
        "receive": {
            "shoulder_pan_position": ["cmd_joint_angular_position"],
            "shoulder_lift_position": ["cmd_joint_angular_position"],
            "elbow_position": ["cmd_joint_angular_position"],
            "wrist_1_position": ["cmd_joint_angular_position"],
            "wrist_2_position": ["cmd_joint_angular_position"],
            "wrist_3_position": ["cmd_joint_angular_position"],
        },
    }
    world = MJCFParser(scene_path).parse()
    viewer = MultiverseViewer()
    
    headless = (
        os.environ.get("CI", "false").lower() == "true"
    )  # headless in CI environments
    multi_sim = MujocoSim(
        world=world,
        viewer=viewer,
        headless=headless,
        step_size=0.01,
        integrator="IMPLICITFAST",
        multiverse_params=multiverse_params,
    )
    multi_sim.start_simulation()

    print("Wait 1s...")
    time.sleep(1)
    viewer.write_objects = {}
    print("Everything is ready")