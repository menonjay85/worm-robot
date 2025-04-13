import asyncio
from pxr import UsdPhysics

async def drive_joints():
    stage = omni.usd.get_context().get_stage()

    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_4/joint_5"), "angular")

    positions = [0, -75, -45]
    current_index = 0

    while True:
        # Set the target position for both joints
        joint_2.GetTargetPositionAttr().Set(positions[current_index])
        joint_3.GetTargetPositionAttr().Set(positions[current_index])

        # Toggle between 0 and -65
        current_index = 1 - current_index

        # Wait for 1 second before switching
        await asyncio.sleep(2)

# Start the asyncio task to drive the joints
asyncio.ensure_future(drive_joints())
