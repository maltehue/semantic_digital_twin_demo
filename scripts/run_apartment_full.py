from typing import Dict, Any, List, Tuple

from semantic_digital_twin.adapters.mjcf import MJCFParser
from multiverse_simulator import (
    MultiverseViewer,
    MultiverseSimulatorState,
    MultiverseCallbackResult,
)
from semantic_digital_twin.adapters.multi_sim import MujocoSim
import os
import time
import pandas
from threading import Thread


def get_data_from_csv(
    csv_path: str, data_structure: Dict[str, Dict[str, Any]]
) -> Tuple[List[float], Dict[str, Dict[str, List[List[float]]]]]:
    df = pandas.read_csv(csv_path)
    time_stamp = df["time"].tolist()
    data: Dict[str, Dict[str, List[List[float]]]] = {}

    for obj_name, values in data_structure.items():
        data[obj_name] = {}
        for attribute_name in values.keys():
            keys = [
                f"{obj_name}:{attribute_name}_{i}"
                for i in range(len(values[attribute_name]))
            ]
            if all(key in df.columns for key in keys):
                data[obj_name][attribute_name] = df[keys].values.tolist()

    return time_stamp, data


def get_contact_data(mujoco_sim: MujocoSim, N_tries=100):
    while mujoco_sim.simulator.state == MultiverseSimulatorState.RUNNING:
        left_hand_contact_bodies = mujoco_sim.simulator.get_contact_bodies(
            body_name="lh_palm", including_children=True
        )
        right_hand_contact_bodies = mujoco_sim.simulator.get_contact_bodies(
            body_name="rh_palm", including_children=True
        )
        assert (
            left_hand_contact_bodies.type
            == MultiverseCallbackResult.ResultType.SUCCESS_WITHOUT_EXECUTION
        ), "Failed to get left hand contact bodies"
        assert (
            right_hand_contact_bodies.type
            == MultiverseCallbackResult.ResultType.SUCCESS_WITHOUT_EXECUTION
        ), "Failed to get right hand contact bodies"
        contact_bodies = left_hand_contact_bodies.result.union(
            right_hand_contact_bodies.result
        )
        if len(left_hand_contact_bodies.result) > 0:
            print(f"Left hand has contact with:", left_hand_contact_bodies.result)
        if len(right_hand_contact_bodies.result) > 0:
            print(f"Right hand has contact with:", right_hand_contact_bodies.result)
        for contact_body in contact_bodies:
            for _ in range(N_tries):
                contact_with = mujoco_sim.simulator.get_contact_bodies(
                    body_name=contact_body, including_children=True
                )
                assert (
                    contact_with.type
                    == MultiverseCallbackResult.ResultType.SUCCESS_WITHOUT_EXECUTION
                ), f"Failed to get contact bodies for {contact_body}"
                if len(contact_with.result) > 0:
                    print(f"  {contact_body} is in contact with:", contact_with.result)
                    break
            else:
                print(f"  {contact_body} contact bodies could not be determined.")
        time.sleep(0.1)


def get_data(mujoco_sim: MujocoSim):
    while mujoco_sim.simulator.state == MultiverseSimulatorState.RUNNING:
        cabinet10_drawer1_joint_pos = mujoco_sim.simulator.get_joint_value(
            "cabinet10_drawer1_joint"
        ).result
        milk_box_pos = mujoco_sim.simulator.get_body_position("milk_box").result
        bowl_pos = mujoco_sim.simulator.get_body_position("bowl").result
        print(
            f"cabinet10_drawer1_joint_pos: {cabinet10_drawer1_joint_pos}, milk_box_pos: {milk_box_pos}, bowl_pos: {bowl_pos}"
        )
        time.sleep(0.1)


if __name__ == "__main__":
    scene_path = os.path.join(
        os.path.dirname(__file__), "..", "assets", "apartment_full.xml"
    )
    csv_path = os.path.join(os.path.dirname(__file__), "..", "multiverse", "data.csv")
    write_objects = {
        "LeftHand_WristRoot": {
            "position": [1.5, 2.6, 1],
            "quaternion": [0, 1, 0, 0],
        },
        "LeftHand_ThumbTip": {
            "position": [1.5785, 2.59142, 1.0835],
            "quaternion": [-0.653282, 0.653282, 0.270598, 0.270598],
        },
        "LeftHand_IndexTip": {
            "position": [1.665, 2.6, 1.033],
            "quaternion": [0, 1, 0, 0],
        },
        "LeftHand_MiddleTip": {
            "position": [1.669, 2.6, 1.011],
            "quaternion": [0, 1, 0, 0],
        },
        "LeftHand_RingTip": {
            "position": [1.665, 2.6, 0.989],
            "quaternion": [0, 1, 0, 0],
        },
        "LeftHand_PinkyTip": {
            "position": [1.6565, 2.6, 0.967],
            "quaternion": [0, 1, 0, 0],
        },
        "RightHand_WristRoot": {
            "position": [1.5, 2.4, 1],
            "quaternion": [0, 0, 0, 1],
        },
        "RightHand_ThumbTip": {
            "position": [1.5785, 2.40858, 1.0835],
            "quaternion": [0.270598, -0.270598, 0.653282, 0.653282],
        },
        "RightHand_IndexTip": {
            "position": [1.665, 2.4, 1.033],
            "quaternion": [0, 0, 0, 1],
        },
        "RightHand_MiddleTip": {
            "position": [1.669, 2.4, 1.011],
            "quaternion": [0, 0, 0, 1],
        },
        "RightHand_RingTip": {
            "position": [1.665, 2.4, 0.989],
            "quaternion": [0, 0, 0, 1],
        },
        "RightHand_PinkyTip": {
            "position": [1.6565, 2.4, 0.967],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
        },
    }
    image_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    world = MJCFParser(scene_path).parse()
    viewer = MultiverseViewer()
    viewer.write_objects = write_objects
    time_stamp, data = get_data_from_csv(csv_path, write_objects)
    headless = (
        os.environ.get("CI", "false").lower() == "true"
    )  # headless in CI environments
    multi_sim = MujocoSim(
        world=world,
        viewer=viewer,
        headless=headless,
        step_size=5e-3,
        integrator="RK4",
        cone="ELLIPTIC",
        impratio="10",
        multiccd=True,
        energy=False,
    )
    multi_sim.start_simulation()

    print("Wait 1s...")
    time.sleep(1)
    print("Everything is ready")

    contact_thread = Thread(target=get_contact_data, args=(multi_sim,), daemon=True)
    contact_thread.start()

    data_thread = Thread(target=get_data, args=(multi_sim,), daemon=True)
    data_thread.start()

    start_idx = 0
    stop_idx = len(time_stamp) - 1
    current_idx = start_idx
    time_offset = time_stamp[current_idx]
    start_time = time.time()
    current_time = time.time() + time_offset - start_time
    try:
        while current_idx < stop_idx:
            last_time = current_time
            current_time = time.time() + time_offset - start_time
            while (
                current_idx < len(time_stamp) and time_stamp[current_idx] < current_time
            ):
                current_idx += 1
            if current_idx >= len(time_stamp) - 1:
                break
            for obj_name, values in data.items():
                for value_name, value_list in values.items():
                    write_objects[obj_name][value_name][:] = value_list[current_idx]
            viewer.write_objects = write_objects
            time.sleep(time_stamp[current_idx + 1] - time_stamp[current_idx])
        time.sleep(1000)

    except KeyboardInterrupt:
        print("Stop simulation!")
    finally:
        multi_sim.stop_simulation()
        contact_thread.join()
        data_thread.join()
