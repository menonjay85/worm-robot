import asyncio
import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")

async def init():
    stage = omni.usd.get_context().get_stage()
    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")
    await asyncio.sleep(2)
    print("Initialized...")

async def set_head_mass():
    head = "/worm_5dof/base_link"
    headPrim = stage.GetPrimAtPath(head)
    hm = UsdPhysics.MassAPI.Apply(headPrim)
    print(hm.GetMassAttr().Set(1000))
    await asyncio.sleep(2)
    print("Anchored head...")

async def task_1():
    print("Task 1 begin")
    # Set joint positions
    joint_1.GetTargetPositionAttr().Set(-45)
    joint_2.GetTargetPositionAttr().Set(45)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(45)
    joint_5.GetTargetPositionAttr().Set(-45)
    await asyncio.sleep(2)
    print("Task 1 complete.")

async def reset_head_mass():
    head = "/worm_5dof/base_link"
    headPrim = stage.GetPrimAtPath(head)
    hm = UsdPhysics.MassAPI.Apply(headPrim)
    print(hm.GetMassAttr().Set(0.03948))
    await asyncio.sleep(2)
    print("Unanchored head...")

async def set_tail_mass():
    tail = "/worm_5dof/link_5"
    tailPrim = stage.GetPrimAtPath(tail)
    tm = UsdPhysics.MassAPI.Apply(tailPrim)
    print(tm.GetMassAttr().Set(50))
    await asyncio.sleep(2)
    print("Anchored tail...")

async def reset_tail_mass():
    tail = "/worm_5dof/link_5"
    tailPrim = stage.GetPrimAtPath(tail)
    tm = UsdPhysics.MassAPI.Apply(tailPrim)
    print(tm.GetMassAttr().Set(0.03948))
    await asyncio.sleep(2)
    print("Unanchored tail...")


async def task_2():
    print("Task 2 begin")
    joint_1.GetTargetPositionAttr().Set(0)
    joint_2.GetTargetPositionAttr().Set(0)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(0)
    joint_5.GetTargetPositionAttr().Set(0)
    await asyncio.sleep(2)
    print("Task 2 complete.")

# Main sequence: Ensure sequential execution
async def main_sequence():
    await init()
    await reset_tail_mass()
    await set_head_mass()
    await task_1()
    await reset_head_mass()
    await set_tail_mass()
    await task_2()

# Start the sequence
run_coroutine(main_sequence())
