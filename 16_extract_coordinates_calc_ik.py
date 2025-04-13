import omni.usd
from pxr import Sdf
import math
import asyncio
import math
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

# Define a helper function to ensure two objects are not equal
def assertNotEqual(first, second, msg=None):
    """Fail if the two objects are equal."""
    if first == second:
        raise AssertionError(msg or f"{first} == {second}")

# Define a helper function to ensure two objects are equal
def assertEqual(first, second, msg=None):
    """Fail if the two objects are not equal."""
    if first != second:
        raise AssertionError(msg or f"{first} != {second}")

# Get the USD stage
stage = omni.usd.get_context().get_stage()

# Specify the path to the primitive
prim_path = "/worm_5dof"
prim = stage.GetPrimAtPath(prim_path)

# Assert that the primitive exists
assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath, f"Prim at path {prim_path} does not exist.")
print(f"Primitive at path {prim_path} exists with path: {prim.GetPath()}")

# Check if specific joints exist
root_joint_path = "/worm_5dof/base_link/joint_1"
root_joint = stage.GetPrimAtPath(root_joint_path)
assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath, f"Root joint at {root_joint_path} does not exist.")

wrist_joint_path = "/worm_5dof/link_1/joint_2"
wrist_joint = stage.GetPrimAtPath(wrist_joint_path)
assertNotEqual(wrist_joint.GetPath(), Sdf.Path.emptyPath, f"Wrist joint at {wrist_joint_path} does not exist.")

# Validate the type of the wrist joint
assertEqual(wrist_joint.GetTypeName(), "PhysicsRevoluteJoint", f"Wrist joint at {wrist_joint_path} is not of type 'PhysicsRevoluteJoint'.")

print("All validations passed successfully!")


head_prim = stage.GetPrimAtPath("/worm_5dof/base_link")
j1_prim = stage.GetPrimAtPath("/worm_5dof/link_1")
j2_prim = stage.GetPrimAtPath("/worm_5dof/link_2")
j3_prim = stage.GetPrimAtPath("/worm_5dof/link_3")
j4_prim = stage.GetPrimAtPath("/worm_5dof/link_4")
j5_prim = stage.GetPrimAtPath("/worm_5dof/link_5")

head_pos = head_prim.GetAttribute("xformOp:translate").Get()
j1_pos = j1_prim.GetAttribute("xformOp:translate").Get()
j2_pos = j2_prim.GetAttribute("xformOp:translate").Get()
j3_pos = j3_prim.GetAttribute("xformOp:translate").Get()
j4_pos = j4_prim.GetAttribute("xformOp:translate").Get()
j5_pos = j5_prim.GetAttribute("xformOp:translate").Get()

link_length = head_pos - j1_pos
tail_pos = j5_pos + link_length

print(link_length)
print(head_pos - j1_pos)
print(j1_pos -j2_pos)
print(j2_pos -j3_pos)
print(j3_pos -j4_pos)
print(j4_pos -j5_pos)
print(tail_pos -j5_pos)

# Since the measurements are in cms
print(round(float(link_length[0]*100), 3))
print(round(float((head_pos[0] - j1_pos[0])*100), 3))
print(round(float((j1_pos[0] -j2_pos[0])*100), 3))
print(round(float((j2_pos[0] -j3_pos[0])*100), 3))
print(round(float((j3_pos[0] -j4_pos[0])*100), 3))
print(round(float((j4_pos[0] -j5_pos[0])*100), 3))
print(round(float((tail_pos[0] -j5_pos[0])*100), 3))

total_rob_length = round(float(6*link_length[0]*100), 3)
print(total_rob_length)

fwd_mmt_desired = 3 #cms

link_length = round(float(link_length[0]*100), 3)
# print("...")
# print(link_length)

# Define the variables
x = fwd_mmt_desired
L = link_length

# Calculate theta in radians
theta_radians = math.acos(1 - (x / (4 * L)))

# Convert theta to degrees
theta_degrees = math.degrees(theta_radians)
theta_degrees = round(float(theta_degrees), 3)

print(f"Theta: {theta_degrees}")

joint = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
theta_degrees = -math.radians(theta_degrees)
joint.GetTargetPositionAttr().Set(math.degrees(theta_degrees))