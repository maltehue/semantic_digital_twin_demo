from semantic_digital_twin.adapters.mjcf import MJCFParser
from multiverse_simulator import MultiverseViewer
from semantic_digital_twin.adapters.multi_sim import MujocoSim
import os
import time
import cv2

if __name__ == "__main__":
    scene_path = os.path.join(os.path.dirname(__file__), "..", "assets", "apartment_with_tiago.xml")
    image_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    world = MJCFParser(scene_path).parse()
    viewer = MultiverseViewer()
    viewer.write_objects = {
        "base_link": {
            "position": [1.5, 2.5, 0.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
        },
        "arm_1_joint": {"joint_angular_position": [0.2]},
        "arm_2_joint": {"joint_angular_position": [-1.34]},
        "arm_3_joint": {"joint_angular_position": [-0.2]},
        "arm_4_joint": {"joint_angular_position": [1.94]},
        "arm_5_joint": {"joint_angular_position": [-1.57]},
        "arm_6_joint": {"joint_angular_position": [1.37]},
        "arm_7_joint": {"joint_angular_position": [0.0]},
        "torso_lift_joint": {"joint_angular_position": [0.3]},
        "arm_position_1_position": {"cmd_joint_angular_position": [0.2]},
        "arm_position_2_position": {"cmd_joint_angular_position": [-1.34]},
        "arm_position_3_position": {"cmd_joint_angular_position": [-0.2]},
        "arm_position_4_position": {"cmd_joint_angular_position": [1.94]},
        "arm_position_5_position": {"cmd_joint_angular_position": [-1.57]},
        "arm_position_6_position": {"cmd_joint_angular_position": [1.37]},
        "arm_position_7_position": {"cmd_joint_angular_position": [0.0]},
        "torso_lift_joint_position": {"cmd_joint_angular_position": [0.3]},
    }
    headless = os.environ.get("CI", "false").lower() == "true" # headless in CI environments
    multi_sim = MujocoSim(world=world, viewer=viewer, headless=headless, step_size=1E-3, integrator="IMPLICITFAST")
    multi_sim.start_simulation()

    print("Wait 1s...")
    time.sleep(1)
    viewer.write_objects = {}
    print("Everything is ready")
    # multi_sim.pause_simulation()

    try:
        for i in range(60):
            capture_rgb = multi_sim.simulator.capture_rgb(camera_name="head_camera")
            rgb = capture_rgb.result
            # Save as png
            cv2.imwrite(os.path.join(image_dir, f"rgb_{i}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop simulation!")
    finally:
        multi_sim.stop_simulation()
