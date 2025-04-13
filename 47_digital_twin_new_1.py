import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import Sdf, UsdPhysics
import omni.usd
import math
import asyncio
import time
from pylx16a.lx16a import *

##############################################################################
#                           HELPER FUNCTIONS
##############################################################################
def assertNotEqual(first, second, msg=None):
    if first == second:
        raise AssertionError(msg or f"{first} == {second}")

def assertEqual(first, second, msg=None):
    if first != second:
        raise AssertionError(msg or f"{first} != {second}")

##############################################################################
#                           EASING FUNCTIONS
##############################################################################
def ease_linear(t, b, c, d):
    return c * t / d + b

def ease_in_out_sine(t, b, c, d):
    return -c/2 * (math.cos(math.pi * t / d) - 1) + b

def ease_in_out_cubic(t, b, c, d):
    t /= d/2
    if t < 1:
        return c/2 * t*t*t + b
    t -= 2
    return c/2 * (t*t*t + 2) + b

def ease_to(servo, target, duration, steps=50, easing_func=ease_in_out_sine):
    start = servo.get_physical_angle()
    change = target - start
    dt = duration / steps
    for step in range(1, steps + 1):
        t = step * dt
        current_angle = easing_func(t, start, change, duration)
        servo.move(current_angle)
        time.sleep(dt)
    servo.move(target)

def ease_to_all(servos, targets, duration, steps=50, easing_func=ease_in_out_cubic):
    # Get current positions for all servos
    starts = [servo.get_physical_angle() for servo in servos]
    changes = [target - start for target, start in zip(targets, starts)]
    dt = duration / steps
    for step in range(1, steps + 1):
        t = step * dt
        for i, servo in enumerate(servos):
            intermediate = easing_func(t, starts[i], changes[i], duration)
            servo.move(intermediate)
        time.sleep(dt)
    for i, servo in enumerate(servos):
        servo.move(targets[i])

def print_final_angles(servo_list, home_dict, label):
    print(f"\n--- Final Angles After {label} ---")
    for s_id in servo_list:
        s = LX16A(s_id)
        phys = s.get_physical_angle()
        centered = phys - home_dict[s_id]
        print(f"Servo {s_id} => Centered: {centered:.2f}°, Physical: {phys:.2f}°")
    print("-----\n")

##############################################################################
#                          INITIALIZATION
##############################################################################
LX16A.initialize("COM3", 0.1)

try:
    servo1 = LX16A(1)
    servo1.set_angle_limits(0, 240)
    servo2 = LX16A(2)
    servo2.set_angle_limits(0, 240)
    servo3 = LX16A(3)
    servo3.set_angle_limits(0, 240)
    servo4 = LX16A(4)
    servo4.set_angle_limits(0, 240)
    servo5 = LX16A(5)
    servo5.set_angle_limits(0, 240)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

##############################################################################
#                          MAIN TASK (ASYNC)
##############################################################################
async def my_task():
    global servo1, servo2, servo3, servo4, servo5  # Ensure these are in scope
    stage = omni.usd.get_context().get_stage()
    
    ############################################################################
    #                         Validate USD Prims
    ############################################################################
    prim_path = "/worm_5dof"
    prim = stage.GetPrimAtPath(prim_path)
    assertNotEqual(prim.GetPath(), Sdf.Path.emptyPath, f"Prim at path {prim_path} does not exist.")
    print(f"Primitive at path {prim_path} exists with path: {prim.GetPath()}")

    root_joint_path = "/worm_5dof/base_link/joint_1"
    root_joint = stage.GetPrimAtPath(root_joint_path)
    assertNotEqual(root_joint.GetPath(), Sdf.Path.emptyPath, f"Root joint at {root_joint_path} does not exist.")

    wrist_joint_path = "/worm_5dof/link_1/joint_2"
    wrist_joint = stage.GetPrimAtPath(wrist_joint_path)
    assertNotEqual(wrist_joint.GetPath(), Sdf.Path.emptyPath, f"Wrist joint at {wrist_joint_path} does not exist.")
    assertEqual(wrist_joint.GetTypeName(), "PhysicsRevoluteJoint",
                f"Wrist joint at {wrist_joint_path} is not of type 'PhysicsRevoluteJoint'.")
    
    print("All validations passed successfully!")

    ############################################################################
    #                       Find Connected Servos
    ############################################################################
    connected_servos = []
    for servo_id in range(1, 7):
        try:
            s = LX16A(servo_id)
            s.get_physical_angle()
            connected_servos.append(servo_id)
        except:
            pass
    print("Connected Servo IDs:", connected_servos)

    ############################################################################
    #                    Define Home Angles and Move to Home
    ############################################################################
    home_angles = {
        1: 119.52,
        2: 99.6,
        3: 131.52,
        4: 114.24,
        5: 87.84
    }
    for s_id, h_angle in home_angles.items():
        if s_id in connected_servos:
            LX16A(s_id).move(h_angle)
    time.sleep(1.0)
    print("Initial angles:")
    for s_id in sorted(home_angles.keys()):
        if s_id in connected_servos:
            print(f"Servo {s_id} => {LX16A(s_id).get_physical_angle():.2f}°")

    ############################################################################
    #                       Compute Link Length from USD
    ############################################################################
    head_prim = stage.GetPrimAtPath("/worm_5dof/base_link")
    j1_prim = stage.GetPrimAtPath("/worm_5dof/link_1")
    j2_prim = stage.GetPrimAtPath("/worm_5dof/link_2")
    j3_prim = stage.GetPrimAtPath("/worm_5dof/link_3")
    j4_prim = stage.GetPrimAtPath("/worm_5dof/link_4")
    j5_prim = stage.GetPrimAtPath("/worm_5dof/link_5")

    head_pos = head_prim.GetAttribute("xformOp:translate").Get()
    head_pos_old = head_pos
    j1_pos = j1_prim.GetAttribute("xformOp:translate").Get()
    j2_pos = j2_prim.GetAttribute("xformOp:translate").Get()
    j3_pos = j3_prim.GetAttribute("xformOp:translate").Get()
    j4_pos = j4_prim.GetAttribute("xformOp:translate").Get()
    j5_pos = j5_prim.GetAttribute("xformOp:translate").Get()

    link_length_vec = head_pos - j1_pos
    link_length_m = link_length_vec[0]
    link_length_cm = link_length_m * 100.0

    tail_pos = j5_pos + link_length_vec

    print("Link length vector (m):", link_length_vec)
    print("Per-segment length (cm):", round(link_length_cm, 3))

    total_rob_length = round(6 * link_length_cm, 3)
    print("Estimated total robot length (cm):", total_rob_length)

    ############################################################################
    #                        Calculate Desired Theta
    ############################################################################
    fwd_mmt_desired = 2.0  # in cm
    x = fwd_mmt_desired
    L = 7  # Set for computation

    theta_radians = math.acos(1 - (x / (2 * L)))
    theta_degrees = math.degrees(theta_radians)
    theta_degrees = round(float(theta_degrees), 3)
    print(f"Theta for forward displacement = {theta_degrees:.3f}°")

    ############################################################################
    #                      Collect USD Physics Drive APIs
    ############################################################################
    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")

    time.sleep(2)

    # First forward bend: compute target physical angles.
    cmd_1 = home_angles[1] - theta_degrees
    cmd_2 = home_angles[2] + theta_degrees
    cmd_3 = home_angles[3] + theta_degrees
    cmd_4 = home_angles[4] - theta_degrees
    cmd_5 = home_angles[5] + 0

    print("Moving to first forward bend (0.5 seconds each)...")
    ease_to_all([servo2, servo3, servo1, servo4, servo5],
                [cmd_2, cmd_3, cmd_1, cmd_4, cmd_5],
                duration=0.5, steps=10, easing_func=ease_in_out_cubic)
    time.sleep(6)
    
    phys_1 = servo1.get_physical_angle()
    phys_2 = servo2.get_physical_angle()
    phys_3 = servo3.get_physical_angle()
    phys_4 = servo4.get_physical_angle()
    phys_5 = servo5.get_physical_angle()

    theta_1 = home_angles[1] - phys_1
    theta_2 = phys_2 - home_angles[2]
    theta_3 = phys_3 - home_angles[3]
    theta_4 = home_angles[4] - phys_4
    theta_5 = home_angles[5] - phys_5

    joint_5.GetTargetPositionAttr().Set(-theta_1)
    joint_4.GetTargetPositionAttr().Set(theta_2)
    joint_3.GetTargetPositionAttr().Set(theta_3)
    joint_2.GetTargetPositionAttr().Set(-theta_4)
    joint_1.GetTargetPositionAttr().Set(theta_5)
    await asyncio.sleep(4)

    # Digital twin gait 2: calculate new target angles.
    cmd_11 = home_angles[1] + 0
    cmd_22 = home_angles[2] - theta_degrees
    cmd_33 = home_angles[3] + theta_degrees
    cmd_44 = home_angles[4] + theta_degrees
    cmd_55 = home_angles[5] - theta_degrees

    print("Moving to second forward bend (0.5 seconds each)...")
    ease_to_all([servo2, servo4, servo1, servo5, servo3],
                [cmd_22, cmd_44, cmd_11, cmd_55, cmd_33],
                duration=0.5, steps=10, easing_func=ease_in_out_cubic)
    time.sleep(6)

    phys_11 = servo1.get_physical_angle()
    phys_22 = servo2.get_physical_angle()
    phys_33 = servo3.get_physical_angle()
    phys_44 = servo4.get_physical_angle()
    phys_55 = servo5.get_physical_angle()

    theta_11 = home_angles[1] - phys_11
    theta_22 = home_angles[2] - phys_22
    theta_33 = phys_33 - home_angles[3]
    theta_44 = phys_44 - home_angles[4] 
    theta_55 = home_angles[5] - phys_55

    joint_5.GetTargetPositionAttr().Set(theta_11)
    joint_4.GetTargetPositionAttr().Set(-theta_22)
    joint_3.GetTargetPositionAttr().Set(theta_33)
    joint_2.GetTargetPositionAttr().Set(theta_44)
    joint_1.GetTargetPositionAttr().Set(-theta_55)
    await asyncio.sleep(4)

run_coroutine(my_task())
