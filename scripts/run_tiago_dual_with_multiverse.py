from semantic_digital_twin.adapters.mjcf import MJCFParser
from multiverse_simulator import MultiverseViewer
from semantic_digital_twin.adapters.multi_sim import MujocoSim
import os
import time

if __name__ == "__main__":
    scene_path = os.path.join(
        os.path.dirname(__file__), "..", "assets", "apartment_with_tiago_dual.xml"
    )
    image_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    world = MJCFParser(scene_path).parse()
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
            "base_link": ["odometric_velocity"],
            "arm_left_1_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_2_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_3_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_4_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_5_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_6_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_left_7_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_1_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_2_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_3_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_4_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_5_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_6_joint_velocity": ["cmd_joint_angular_velocity"],
            "arm_right_7_joint_velocity": ["cmd_joint_angular_velocity"],
            "torso_lift_joint_position": ["cmd_joint_linear_position"],
            "head_1_joint_position": ["cmd_joint_angular_position"],
            "head_2_joint_position": ["cmd_joint_angular_position"],
            "gripper_left_left_finger_joint_position": ["cmd_joint_linear_position"],
            "gripper_left_right_finger_joint_position": ["cmd_joint_linear_position"],
            "gripper_right_left_finger_joint_position": ["cmd_joint_linear_position"],
            "gripper_right_right_finger_joint_position": ["cmd_joint_linear_position"],
        },
    }
    viewer = MultiverseViewer()
    viewer.write_objects = {
        "base_link": {
            "position": [1.5, 2.5, 0.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
        },
        # "arm_left_1_joint": {"joint_angular_position": [0.2]},
        # "arm_left_2_joint": {"joint_angular_position": [-1.34]},
        # "arm_left_3_joint": {"joint_angular_position": [-0.2]},
        # "arm_left_4_joint": {"joint_angular_position": [1.94]},
        # "arm_left_5_joint": {"joint_angular_position": [-1.57]},
        # "arm_left_6_joint": {"joint_angular_position": [1.37]},
        # "arm_left_7_joint": {"joint_angular_position": [0.0]},
        # "arm_right_1_joint": {"joint_angular_position": [0.2]},
        # "arm_right_2_joint": {"joint_angular_position": [-1.34]},
        # "arm_right_3_joint": {"joint_angular_position": [-0.2]},
        # "arm_right_4_joint": {"joint_angular_position": [1.94]},
        # "arm_right_5_joint": {"joint_angular_position": [-1.57]},
        # "arm_right_6_joint": {"joint_angular_position": [1.37]},
        # "arm_right_7_joint": {"joint_angular_position": [0.0]},
        "torso_lift_joint": {"joint_linear_position": [0.3]},
        "torso_lift_joint_position": {"cmd_joint_linear_position": [0.3]},
    }
    headless = (
        os.environ.get("CI", "false").lower() == "true"
    )  # headless in CI environments
    multi_sim = MujocoSim(
        world=world,
        viewer=viewer,
        headless=headless,
        step_size=0.01,
        integrator="IMPLICITFAST",
        cone="PYRAMIDAL",
        multiverse_params=multiverse_params,
    )
    multi_sim.start_simulation()

    print("Wait 1s...")
    time.sleep(1)
    viewer.write_objects = {}
    print("Everything is ready")

    try:
        for i in range(600):
            # capture_rgb = multi_sim.simulator.capture_rgb(camera_name="head_camera")
            # rgb = capture_rgb.result
            # # Save as png
            # cv2.imwrite(os.path.join(image_dir, f"rgb_{i}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop simulation!")
    finally:
        multi_sim.stop_simulation()
