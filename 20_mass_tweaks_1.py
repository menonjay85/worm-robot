import numpy as np
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.cloner import GridCloner
from omni.isaac.core.articulations import ArticulationView
from pxr import UsdGeom


prims = ArticulationView(prim_paths_expr="/worm_5dof", name="worm_view")
prims.initialize()
print(prims.num_dof)    

masses = np.tile(np.array([0.03948, 0.03948, 0.03948, 0.03948, 0.03948, 1.0]), (1,1))
prims.set_body_masses(masses)

# ------------------
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView
from pxr import UsdPhysics
prim_path = "/worm_5dof"
prim = get_prim_at_path(prim_path)

if prim is None:
    raise ValueError(f"Prim at path {prim_path} does not exist.")
else:
    print(f"Found prim at path: {prim_path}")

# Initialize the robot articulation
robot_prim_path = "/worm_5dof"
robot = Articulation(prim_path=robot_prim_path, name="5DOF_Robot")

# Initialize the articulation
robot.initialize()
stage = omni.usd.get_context().get_stage()
robot.set_solver_position_iteration_count(32)
robot.set_enabled_self_collisions(True)
mass = 12.0
robot_base = "/worm_5dof/base_link"
cubePrim = stage.GetPrimAtPath(robot_base)
massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
massAPI.CreateMassAttr(mass)
mass = 12.0
massAPI.GetMassAttr().Set(mass)

robot_base = "/worm_5dof/link_1"
cubePrim = stage.GetPrimAtPath(robot_base)
massAPI = UsdPhysics.MassAPI.Apply(cubePrim)

print(massAPI.GetMassAttr().Set(100000))

print(massAPI.GetPrincipalAxesAttr().Get())
massAPI.CreatePrincipalAxesAttr() == [1,2,3,4]
# Refer:!!!
# https://docs.omniverse.nvidia.com/isaacsim/latest/reference_python_api.html
# https://docs.omniverse.nvidia.com/kit/docs/pxr-usd-api/latest/pxr/UsdPhysics.html#pxr.UsdPhysics.MassAPI.GetPrincipalAxesAttr
# https://docs.omniverse.nvidia.com/kit/docs/pxr-usd-api/latest/pxr/Usd.html#pxr.Usd.Attribute.ClearAtTime
# C:\Users\menon\AppData\Local\ov\pkg\isaac-sim-4.1.0\kit\exts\usdrt.scenegraph\include\usdrt\scenegraph\usd\usdPhysics

