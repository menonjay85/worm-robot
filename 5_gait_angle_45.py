import asyncio
import math
from pxr import UsdPhysics

async def drive_joints():
    stage = omni.usd.get_context().get_stage()

    # Get DriveAPI for each joint
    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_4/joint_5"), "angular")

    # Defining the movement sequence for the joints based on MATLAB code
    alpha = math.radians(45)  # Convert to radians for consistency
    time_intervals = [2, 2, 2, 2, 2]  # Each step has a duration of 2 seconds

    current_index = 0

    while True:
        # Calculate joint angles based on constraints
        if current_index == 0:
            # Step One (Mechanism Start)
            theta_1 = math.radians(0)
            theta_2 = math.radians(0)
            theta_3 = math.radians(0)
            theta_4 = math.radians(0)
            theta_5 = math.radians(0)
            print("Step 0: Start")
        elif current_index == 1:
            # Step One (Mechanism M1)
            theta_1 = math.radians(0)
            theta_2 = math.radians(-45)
            theta_3 = math.radians(-45)
            theta_4 = math.radians(45)
            theta_5 = math.radians(0)
            print("Step 1: M1")
        elif current_index == 2:
            # Step One (Mechanism M2)
            theta_1 = math.radians(0)
            theta_2 = math.radians(45)
            theta_3 = math.radians(-45)
            theta_4 = math.radians(-45)
            theta_5 = math.radians(45)
            print("Step 2: M2")
        elif current_index == 3:
            # Step One (Mechanism M2')
            theta_1 = math.radians(0)
            theta_2 = math.radians(0)
            theta_3 = math.radians(45)
            theta_4 = math.radians(-45)
            theta_5 = math.radians(-45)
            print("Step 3: M2'")
        else:
            # Step Two (Mechanism M1')
            theta_1 = math.radians(0)
            theta_2 = math.radians(0)
            theta_3 = math.radians(0)
            theta_4 = math.radians(0)
            theta_5 = math.radians(0)
            print("Step 4: M1'")

        # Set the target positions for each joint to simulate inchworm motion
        joint_1.GetTargetPositionAttr().Set(math.degrees(theta_1))
        joint_2.GetTargetPositionAttr().Set(math.degrees(theta_2))
        joint_3.GetTargetPositionAttr().Set(math.degrees(theta_3))
        joint_4.GetTargetPositionAttr().Set(math.degrees(theta_4))
        joint_5.GetTargetPositionAttr().Set(math.degrees(theta_5))

        # Print the current index before waiting
        print(f"Current index: {current_index}")

        # Wait for a specified time interval before switching
        await asyncio.sleep(time_intervals[current_index])

        # Toggle to the next position in the sequence
        current_index = (current_index + 1) % 5

# Start the asyncio task to drive the joints
asyncio.ensure_future(drive_joints())