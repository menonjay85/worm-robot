import omni.usd
from pxr import Sdf

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

basePrim = stage.GetPrimAtPath("/worm_5dof/base_link")
base_pos = basePrim.GetAttribute("xformOp:translate").Get()
linkPrim = stage.GetPrimAtPath("/worm_5dof/link_1")
link_pos = linkPrim.GetAttribute("xformOp:translate").Get()
print(base_pos)
print(link_pos)
print(base_pos - link_pos)

basePrim = stage.GetPrimAtPath("/worm_5dof/link_1")
base_pos = basePrim.GetAttribute("xformOp:translate").Get()
linkPrim = stage.GetPrimAtPath("/worm_5dof/link_2")
link_pos = linkPrim.GetAttribute("xformOp:translate").Get()
print(base_pos)
print(link_pos)
print(base_pos - link_pos)

basePrim = stage.GetPrimAtPath("/worm_5dof/link_2")
base_pos = basePrim.GetAttribute("xformOp:translate").Get()
linkPrim = stage.GetPrimAtPath("/worm_5dof/link_3")
link_pos = linkPrim.GetAttribute("xformOp:translate").Get()
print(base_pos)
print(link_pos)
print(base_pos - link_pos)
vec = base_pos - link_pos
print(vec[0]*100)

basePrim = stage.GetPrimAtPath("/worm_5dof/link_1")
base_pos = basePrim.GetAttribute("xformOp:translate").Get()
c1 = base_pos + vec
print(c1)

# Everything is in cms