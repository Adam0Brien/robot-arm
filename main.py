#!/usr/bin/env python3
"""
Robot arm entrypoint. Uses the chainable Arm API from arm.py.
Run on the Raspberry Pi: python3 main.py
"""
from arm import Arm

if __name__ == "__main__":
    arm = Arm()

    # Full ROM: sweep each joint 0° → 180° → 0°
    STEP = 5
    PAUSE = 0.06

    arm.home().wait(0.5)

    # joints = [
    #     ("base", arm.move_base),
    #     ("shoulder", arm.move_shoulder),
    #     ("elbow", arm.move_elbow),
    #     ("wrist", arm.move_wrist),
    #     ("gripper", arm.move_gripper),
    # ]
    # for name, move_fn in joints:
    #     for angle in range(0, 181, STEP):
    #         move_fn(angle).wait(PAUSE)
    #     for angle in range(180, -1, -STEP):
    #         move_fn(angle).wait(PAUSE)
    #     arm.wait(0.2)

    

    arm.home().wait(0.3)

    # Example 2: Wave (shoulder up/down, gripper open/close) then point
    # (
    #     arm.home()
    #     .wait(0.5)
    #     .move_shoulder(120).open_gripper().wait(0.3)
    #     .move_shoulder(60).close_gripper().wait(0.3)
    #     .move_shoulder(120).open_gripper().wait(0.3)
    #     .move_shoulder(60).close_gripper().wait(0.3)
    #     .home().wait(0.3)
    #     .move_shoulder(45).move_elbow(135).move_wrist(180)  # point forward
    #     .wait(1.5)
    #     .home()
    # )

    # Or step by step:
    # arm.move_base(90)
    # arm.move_shoulder(45)
    # arm.move_elbow(135)
    # arm.move_wrist(90)
    # arm.move_gripper(80)
    # arm.wait(2)
    # arm.move_to(shoulder=30, gripper=60)  # only change shoulder and gripper
    # arm.open_gripper().wait(1).close_gripper()
    # arm.home()
