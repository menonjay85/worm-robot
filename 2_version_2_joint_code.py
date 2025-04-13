import os
import asyncio
import math
import weakref
import omni
import omni.kit.commands
import omni.ui as ui
from omni.importer.urdf.scripts.ui import (
    btn_builder,
    get_style,
    make_menu_item_description,
    setup_ui_headers,
)
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, PhysicsSchemaTools, Sdf, UsdLux, UsdPhysics
import carb.tokens
import omni
from pxr import PhysxSchema, UsdGeom, UsdPhysics


def set_drive_parameters(drive, target_type, target_value, stiffness=None, damping=None, max_force=None):
    """Enable velocity drive for a given joint"""

    if target_type == "position":
        if not drive.GetTargetPositionAttr():
            drive.CreateTargetPositionAttr(target_value)
        else:
            drive.GetTargetPositionAttr().Set(target_value)
    elif target_type == "velocity":
        if not drive.GetTargetVelocityAttr():
            drive.CreateTargetVelocityAttr(target_value)
        else:
            drive.GetTargetVelocityAttr().Set(target_value)

    if stiffness is not None:
        if not drive.GetStiffnessAttr():
            drive.CreateStiffnessAttr(stiffness)
        else:
            drive.GetStiffnessAttr().Set(stiffness)

    if damping is not None:
        if not drive.GetDampingAttr():
            drive.CreateDampingAttr(damping)
        else:
            drive.GetDampingAttr().Set(damping)

    if max_force is not None:
        if not drive.GetMaxForceAttr():
            drive.CreateMaxForceAttr(max_force)
        else:
            drive.GetMaxForceAttr().Set(max_force)


stage = omni.usd.get_context().get_stage()

PhysxSchema.PhysxArticulationAPI.Get(stage, "/new_assem_wo_mating").CreateSolverPositionIterationCountAttr(64)
PhysxSchema.PhysxArticulationAPI.Get(stage, "/new_assem_wo_mating").CreateSolverVelocityIterationCountAttr(64)

joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/base_link/joint_1"), "angular")
joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_1/joint_2"), "angular")
joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_2/joint_3"), "angular")
joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_3/joint_4"), "angular")
joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_4/joint_5"), "angular")

set_drive_parameters(joint_1, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
set_drive_parameters(joint_2, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
set_drive_parameters(joint_3, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
set_drive_parameters(joint_4, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
set_drive_parameters(joint_5, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))

set_drive_parameters(joint_1, "position", 0)
set_drive_parameters(joint_2, "position", -65)
set_drive_parameters(joint_3, "position", -65)
set_drive_parameters(joint_4, "position", 0)
set_drive_parameters(joint_5, "position", 0)