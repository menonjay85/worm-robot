import asyncio
import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import Sdf
import omni.usd
from pxr import Sdf
import math
import csv
import os
import asyncio
import math
from pxr import UsdPhysics

csv_file = 'joint_positions_log.csv'
csv_path = os.path.abspath(csv_file)  # Get absolute path of the file
fieldnames = [
    'timestamp',
    'joint_1_position',
    'joint_2_position',
    'joint_3_position',
    'joint_4_position',
    'joint_5_position'
]

# Initialize the CSV file with headers
with open(csv_path, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()

print(f" CSV file is being saved at: {csv_path}")


# 📊 Continuous Joint Position Logging
async def log_joint_positions(joint_1, joint_2, joint_3, joint_4, joint_5):
    """Continuously log joint angular positions to CSV with maximum precision."""
    while True:
        with open(csv_path, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writerow({
                'timestamp': asyncio.get_event_loop().time(),
                'joint_1_position': joint_1.GetPrim().GetAttribute("state:angular:physics:position").Get(),
                'joint_2_position': joint_2.GetPrim().GetAttribute("state:angular:physics:position").Get(),
                'joint_3_position': joint_3.GetPrim().GetAttribute("state:angular:physics:position").Get(),
                'joint_4_position': joint_4.GetPrim().GetAttribute("state:angular:physics:position").Get(),
                'joint_5_position': joint_5.GetPrim().GetAttribute("state:angular:physics:position").Get(),
            })
        await asyncio.sleep(0.0000001)  # Run the loop as fast as possible without delays


async def my_task():
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
	head_pos_old = head_prim.GetAttribute("xformOp:translate").Get()
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
	
	fwd_mmt_desired = 7 #cms
	
	link_length = round(float(link_length[0]*100), 3)
	# print("...")
	# print(link_length)
	
	# Define the variables
	x = fwd_mmt_desired
	L = link_length
	
	# Calculate theta in radians
	theta_radians = math.acos(1 - (x / (2* L)))
	
	# Convert theta to degrees
	theta_degrees = math.degrees(theta_radians)
	theta_degrees = round(float(theta_degrees), 3)
	
	print(f"Theta: {theta_degrees}")
	
	joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
	joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
	joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
	joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
	joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")
	asyncio.create_task(log_joint_positions(joint_1, joint_2, joint_3, joint_4, joint_5))
	theta_degrees = -math.radians(theta_degrees)
	joint_2.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_3.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_4.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_5.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	await asyncio.sleep(2)
	tail = "/worm_5dof/link_5"
	tailPrim = stage.GetPrimAtPath(tail)
	tm = UsdPhysics.MassAPI.Apply(tailPrim)
	print(tm.GetMassAttr().Set(50))
	await asyncio.sleep(2)
	print("Anchored tail...")
	joint_1.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_2.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_3.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_4.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_5.GetTargetPositionAttr().Set(math.degrees(0))
	await asyncio.sleep(2)
	joint_1.GetTargetPositionAttr().Set(math.degrees(0))
	joint_2.GetTargetPositionAttr().Set(math.degrees(0))
	joint_3.GetTargetPositionAttr().Set(math.degrees(0))
	joint_4.GetTargetPositionAttr().Set(math.degrees(0))
	joint_5.GetTargetPositionAttr().Set(math.degrees(0))
	await asyncio.sleep(2)
	print(tm.GetMassAttr().Set(0.03948))
	await asyncio.sleep(2)
	print("Reset Anchored tail...")
	x1 = round(float(head_pos_old[0]*100), 3)
	head_pos_new = head_prim.GetAttribute("xformOp:translate").Get()
	x2 = round(float(head_pos_new[0]*100), 3)
	print(x1)
	print(x2)
	#print(x1-x2)
	#print(x2-x1)
	print(x1-x2)
	j5_pos_new = j5_prim.GetAttribute("xformOp:translate").Get()
	print((j5_pos_new[0] - j5_pos[0]))


run_coroutine(my_task())
