import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import Sdf
import omni.usd
from pxr import Sdf
import math
import asyncio
from pxr import UsdPhysics
from math import sin, cos
from pylx16a.lx16a import *
import time

LX16A.initialize("COM3", 0.1)


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
	
	# Linear easing function (for reference)
	def ease_linear(t, b, c, d):
	    return c * t / d + b
	
	def ease_in_out_sine(t, b, c, d):
	    """
	    Easing function: ease-in-out sine.
	    
	    t: current time
	    b: starting value
	    c: change in value (target - start)
	    d: duration
	    """
	    return -c/2 * (math.cos(math.pi * t / d) - 1) + b
	
	# Cubic ease-in-out function
	def ease_in_out_cubic(t, b, c, d):
	    t /= d/2
	    if t < 1:
	        return c/2 * t*t*t + b
	    t -= 2
	    return c/2 * (t*t*t + 2) + b
	
	def ease_to(servo, target, duration, steps=50, easing_func=ease_in_out_sine):
	    """
	    Smoothly move the servo from its current angle to the target angle.
	    
	    Parameters:
	    servo: the servo object to move
	    target: the target physical angle (native range: 0-240°)
	    duration: total time for the movement (in seconds)
	    steps: number of intermediate steps
	    easing_func: the easing function to use
	    """
	    start = servo.get_physical_angle()
	    change = target - start
	    dt = duration / steps
	    for step in range(1, steps + 1):
	        t = step * dt
	        current_angle = easing_func(t, start, change, duration)
	        servo.move(current_angle)
	        time.sleep(dt)
	    # Ensure the target is exactly reached at the end.
	    servo.move(target)
	
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

	# Scan for connected servos
	connected_servos = []
	
	for servo_id in range(0, 254):
	    try:
	        servo = LX16A(servo_id)
	        servo.get_physical_angle()
	        connected_servos.append(servo_id)
	    except:
	        pass
	print("Connected Servo IDs:", connected_servos)
	
	# Print initial physical angles
	print("Initial angles:")
	print(LX16A(1).get_physical_angle())
	print(LX16A(2).get_physical_angle())
	print(LX16A(3).get_physical_angle())
	print(LX16A(4).get_physical_angle())
	print(LX16A(5).get_physical_angle())
	
	# Define the measured home positions for each servo (native angles)
	servo1_home = 119.52 
	servo2_home = 99.6 
	servo3_home = 131.52 
	servo4_home = 114.24 
	servo5_home = 87.84 
	
	# Move all servos to their home positions
	LX16A(1).move(servo1_home)
	LX16A(2).move(servo2_home)
	LX16A(3).move(servo3_home)
	LX16A(4).move(servo4_home)
	LX16A(5).move(servo5_home)
	
	time.sleep(1)  # Allow time for servos to reach home

	
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
	
	fwd_mmt_desired = 2 #cms
	
	link_length = round(float(link_length[0]*100), 3)
	# print("...")
	# print(link_length)
	
	# Define the variables
	x = fwd_mmt_desired
	L = link_length
	
	# Calculate theta in radians
	theta_radians = math.acos(1 - (x / (2* L)))
	
	servo1 = LX16A(1)
	servo2 = LX16A(2)
	servo3 = LX16A(3)
	servo4 = LX16A(4)
	servo5 = LX16A(5)
	
	
	theta_degrees = math.degrees(theta_radians)
	theta_degrees = round(float(theta_degrees), 3)

	# In the centered system, home (0°) is defined by the measured home positions.
	# To move Servo 2 to a target relative angle, set target_angle_centered accordingly.
	# For example, for -45° relative:
	target_angle1 = -theta_degrees
	target_angle2 = theta_degrees
	target_angle3 = theta_degrees
	target_angle4 = -theta_degrees
	target_angle5 = 0
	
	physical_target_servo1 = servo1_home + target_angle1
	physical_target_servo2 = servo2_home + target_angle2
	physical_target_servo3 = servo3_home + target_angle3
	physical_target_servo4 = servo4_home + target_angle4
	physical_target_servo5 = servo5_home + target_angle5
	
	print(f"Theta: {theta_degrees}")
	
	joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
	joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
	joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
	joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
	joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")
	time.sleep(2)
	
	print("Easing Servo 1 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle1, physical_target_servo1))
	print("Easing Servo 2 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle2, physical_target_servo2))
	print("Easing Servo 3 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle3, physical_target_servo3))
	print("Easing Servo 4 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle4, physical_target_servo4))
	print("Easing Servo 5 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle5, physical_target_servo5))
	ease_to(servo1, physical_target_servo1, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo2, physical_target_servo2, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo3, physical_target_servo3, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo4, physical_target_servo4, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo5, physical_target_servo5, duration=2, steps=50, easing_func=ease_in_out_cubic)
	time.sleep(6)  # Wait a moment for movement to complete
	
	# Verify final angle by converting physical angle to the centered system
	final_angle1 = servo1.get_physical_angle()
	final_angle2 = servo2.get_physical_angle()
	final_angle3 = servo3.get_physical_angle()
	final_angle4 = servo4.get_physical_angle()
	final_angle5 = servo5.get_physical_angle()
	
	centered_final1 = final_angle1 - servo1_home
	centered_final2 = final_angle2 - servo2_home
	centered_final3 = final_angle3 - servo3_home
	centered_final4 = final_angle4 - servo4_home
	centered_final5 = final_angle5 - servo5_home
	
	print("Servo 1 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final1, final_angle1))
	print("Servo 2 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final2, final_angle2))
	print("Servo 3 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final3, final_angle3))
	print("Servo 4 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final4, final_angle4))
	print("Servo 5 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final5, final_angle5))
	time.sleep(2)
	
	target_angle11 = 0
	target_angle22 = -theta_degrees 
	target_angle33 = theta_degrees 
	target_angle44 = theta_degrees 
	target_angle55 = -theta_degrees
	
	physical_target_servo11 = servo1_home + target_angle11
	physical_target_servo22 = servo2_home + target_angle22
	physical_target_servo33 = servo3_home + target_angle33
	physical_target_servo44 = servo4_home + target_angle44
	physical_target_servo55 = servo5_home + target_angle55
	
	
	joint_2.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_3.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_4.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_5.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	await asyncio.sleep(2)
	
	
	print("Easing Servo 1 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle1, physical_target_servo1))
	print("Easing Servo 2 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle2, physical_target_servo2))
	print("Easing Servo 3 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle3, physical_target_servo3))
	print("Easing Servo 4 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle4, physical_target_servo4))
	print("Easing Servo 5 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle5, physical_target_servo5))
	ease_to(servo1, physical_target_servo11, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo2, physical_target_servo22, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo3, physical_target_servo33, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo4, physical_target_servo44, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo5, physical_target_servo55, duration=2, steps=50, easing_func=ease_in_out_cubic)
	time.sleep(6)  # Wait a moment for movement to complete
	
	# Verify final angle by converting physical angle to the centered system
	final_angle11 = servo1.get_physical_angle()
	final_angle22 = servo2.get_physical_angle()
	final_angle33 = servo3.get_physical_angle()
	final_angle44 = servo4.get_physical_angle()
	final_angle55 = servo5.get_physical_angle()
	
	centered_final11 = final_angle11 - servo1_home
	centered_final22 = final_angle22 - servo2_home
	centered_final33 = final_angle33 - servo3_home
	centered_final44 = final_angle44 - servo4_home
	centered_final55 = final_angle55 - servo5_home
	
	print("Servo 1 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final11, final_angle11))
	print("Servo 2 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final22, final_angle22))
	print("Servo 3 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final33, final_angle33))
	print("Servo 4 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final44, final_angle44))
	print("Servo 5 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final55, final_angle55))
	time.sleep(2)
	
	
	tail = "/worm_5dof/link_5"
	tailPrim = stage.GetPrimAtPath(tail)
	tm = UsdPhysics.MassAPI.Apply(tailPrim)
	print(tm.GetMassAttr().Set(50))
	await asyncio.sleep(2)
	print("Anchored tail...")
	
	theta_degrees = -math.radians(theta_degrees)
	target_angle1 = 0
	target_angle2 = theta_degrees 
	target_angle3 = -theta_degrees 
	target_angle4 = -theta_degrees 
	target_angle5 = theta_degrees
	
	physical_target_servo1 = servo1_home + target_angle1
	physical_target_servo2 = servo2_home + target_angle2
	physical_target_servo3 = servo3_home + target_angle3
	physical_target_servo4 = servo4_home + target_angle4
	physical_target_servo5 = servo5_home + target_angle5
	
	
	joint_1.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_2.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_3.GetTargetPositionAttr().Set(math.degrees(-theta_degrees))
	joint_4.GetTargetPositionAttr().Set(math.degrees(theta_degrees))
	joint_5.GetTargetPositionAttr().Set(math.degrees(0))
	await asyncio.sleep(2)
	
	print("Easing Servo 1 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle1, physical_target_servo1))
	print("Easing Servo 2 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle2, physical_target_servo2))
	print("Easing Servo 3 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle3, physical_target_servo3))
	print("Easing Servo 4 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle4, physical_target_servo4))
	print("Easing Servo 5 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle5, physical_target_servo5))
	ease_to(servo1, physical_target_servo1, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo2, physical_target_servo2, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo3, physical_target_servo3, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo4, physical_target_servo4, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo5, physical_target_servo5, duration=2, steps=50, easing_func=ease_in_out_cubic)
	time.sleep(3)  # Wait a moment for movement to complete
	
	# Verify final angle by converting physical angle to the centered system
	final_angle1 = servo1.get_physical_angle()
	final_angle2 = servo2.get_physical_angle()
	final_angle3 = servo3.get_physical_angle()
	final_angle4 = servo4.get_physical_angle()
	final_angle5 = servo5.get_physical_angle()
	
	centered_final1 = final_angle1 - servo1_home
	centered_final2 = final_angle2 - servo2_home
	centered_final3 = final_angle3 - servo3_home
	centered_final4 = final_angle4 - servo4_home
	centered_final5 = final_angle5 - servo5_home
	
	print("Servo 1 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final1, final_angle1))
	print("Servo 2 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final2, final_angle2))
	print("Servo 3 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final3, final_angle3))
	print("Servo 4 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final4, final_angle4))
	print("Servo 5 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final5, final_angle5))
	time.sleep(2)
	
	
	target_angle1 = 0
	target_angle2 = 0 
	target_angle3 = 0 
	target_angle4 = 0 
	target_angle5 = 0
	
	physical_target_servo1 = servo1_home + target_angle1
	physical_target_servo2 = servo2_home + target_angle2
	physical_target_servo3 = servo3_home + target_angle3
	physical_target_servo4 = servo4_home + target_angle4
	physical_target_servo5 = servo5_home + target_angle5
	
	
	joint_1.GetTargetPositionAttr().Set(math.degrees(0))
	joint_2.GetTargetPositionAttr().Set(math.degrees(0))
	joint_3.GetTargetPositionAttr().Set(math.degrees(0))
	joint_4.GetTargetPositionAttr().Set(math.degrees(0))
	joint_5.GetTargetPositionAttr().Set(math.degrees(0))
	await asyncio.sleep(2)
	print(tm.GetMassAttr().Set(0.03948))
	await asyncio.sleep(2)
	print("Reset Anchored tail...")
	
	print("Easing Servo 1 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle1, physical_target_servo1))
	print("Easing Servo 2 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle2, physical_target_servo2))
	print("Easing Servo 3 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle3, physical_target_servo3))
	print("Easing Servo 4 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle4, physical_target_servo4))
	print("Easing Servo 5 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle5, physical_target_servo5))
	ease_to(servo1, physical_target_servo1, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo2, physical_target_servo2, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo3, physical_target_servo3, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo4, physical_target_servo4, duration=2, steps=50, easing_func=ease_in_out_cubic)
	ease_to(servo5, physical_target_servo5, duration=2, steps=50, easing_func=ease_in_out_cubic)
	time.sleep(3)  # Wait a moment for movement to complete
	
	# Verify final angle by converting physical angle to the centered system
	final_angle1 = servo1.get_physical_angle()
	final_angle2 = servo2.get_physical_angle()
	final_angle3 = servo3.get_physical_angle()
	final_angle4 = servo4.get_physical_angle()
	final_angle5 = servo5.get_physical_angle()
	
	centered_final1 = final_angle1 - servo1_home
	centered_final2 = final_angle2 - servo2_home
	centered_final3 = final_angle3 - servo3_home
	centered_final4 = final_angle4 - servo4_home
	centered_final5 = final_angle5 - servo5_home
	
	print("Servo 1 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final1, final_angle1))
	print("Servo 2 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final2, final_angle2))
	print("Servo 3 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final3, final_angle3))
	print("Servo 4 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final4, final_angle4))
	print("Servo 5 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final5, final_angle5))
	time.sleep(2)
	
	
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
