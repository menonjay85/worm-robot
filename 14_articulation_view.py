import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.cloner import GridCloner
from omni.isaac.core.articulations import ArticulationView
from pxr import UsdGeom

prims = ArticulationView(prim_paths_expr="/worm_5dof", name="worm_view")

print(prims)
print(prims.initialized)
prims.initialize()
print(prims.num_dof)

print(prims.get_dof_types())


print(prims.get_jacobian_shape())
print(prims.get_jacobians())