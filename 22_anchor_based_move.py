import asyncio
import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

# Accessing the joints
joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")

# Task 1: Manipulate the base link
async def my_task1():
    print("Task 1 begin")
    
    robot_base2 = "/worm_5dof/link_5"
    cubePrim2 = stage.GetPrimAtPath(robot_base2)
    massAPI2 = UsdPhysics.MassAPI.Apply(cubePrim2)
    print(massAPI2.GetMassAttr().Set(0.03948))
    
    robot_base = "/worm_5dof/base_link"
    cubePrim = stage.GetPrimAtPath(robot_base)
    massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
    print(massAPI.GetMassAttr().Set(50))
    
    # Set joint positions
    joint_1.GetTargetPositionAttr().Set(-45)
    joint_2.GetTargetPositionAttr().Set(45)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(45)
    joint_5.GetTargetPositionAttr().Set(-45)
    
    # Wait to observe animation
    await asyncio.sleep(2)
    print("Task 1 complete")

# Task 2: Manipulate link_5
async def my_task2():
    print("Task 2 begin")
    
    # Revert the mass of base_link
    robot_base = "/worm_5dof/base_link"
    cubePrim = stage.GetPrimAtPath(robot_base)
    massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
    print(f"Reverting mass of base_link: {massAPI.GetMassAttr().Set(0.03948)}")
    
    robot_base2 = "/worm_5dof/link_5"
    cubePrim2 = stage.GetPrimAtPath(robot_base2)
    massAPI2 = UsdPhysics.MassAPI.Apply(cubePrim2)
    print(massAPI2.GetMassAttr().Set(5))
    
    # Reset joint positions
    joint_1.GetTargetPositionAttr().Set(0)
    joint_2.GetTargetPositionAttr().Set(0)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(0)
    joint_5.GetTargetPositionAttr().Set(0)
    
    # Wait to observe animation
    await asyncio.sleep(2)
    print("Task 2 complete")

# Main sequence: Ensure sequential execution
async def main_sequence():
    await my_task1()  # Run task 1
    await my_task2()  # Run task 2

# Start the sequence
run_coroutine(main_sequence())
