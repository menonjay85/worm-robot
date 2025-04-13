import asyncio
import math
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
theta_2 = math.radians(30)
joint_2.GetTargetPositionAttr().Set(math.degrees(theta_2))

joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
theta_1 = math.radians(0)
joint_1.GetTargetPositionAttr().Set(math.degrees(theta_1))