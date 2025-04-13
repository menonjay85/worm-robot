import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import Sdf, UsdPhysics
import omni.usd
import math
import asyncio
import time

# Extra import for plotting
import matplotlib.pyplot as plt

# LX-16A servo library
from pylx16a.lx16a import *

##############################################################################
#                           INITIALIZATION
##############################################################################

# Initialize LX-16A servo communication on COM3, with 0.1s timeout
LX16A.initialize("COM4", 0.1)

##############################################################################
#                        HELPER FUNCTIONS
##############################################################################

def assertNotEqual(first, second, msg=None):
    """
    Fail if the two objects are equal.
    """
    if first == second:
        raise AssertionError(msg or f"{first} == {second}")

def assertEqual(first, second, msg=None):
    """
    Fail if the two objects are not equal.
    """
    if first != second:
        raise AssertionError(msg or f"{first} != {second}")


##############################################################################
#                           EASING FUNCTIONS
##############################################################################
def ease_linear(t, b, c, d):
    """
    Linear easing from b -> b+c.
    """
    return c * t / d + b

def ease_in_out_sine(t, b, c, d):
    """
    Easing function: ease-in-out sine.
    """
    return -c/2 * (math.cos(math.pi * t / d) - 1) + b

def ease_in_out_cubic(t, b, c, d):
    """
    Easing function: ease-in-out cubic.
    """
    t /= d/2
    if t < 1:
        return c/2 * t*t*t + b
    t -= 2
    return c/2 * (t*t*t + 2) + b

def ease_to(servo, target, duration, steps=50, easing_func=ease_in_out_sine):
    """
    Smoothly move the servo from its current angle to the target angle.
    
    Parameters:
        servo (LX16A): The servo object to move.
        target (float): The target physical angle (0-240° range).
        duration (float): Total time for the movement (in seconds).
        steps (int): Number of intermediate steps.
        easing_func (callable): The easing function to use.
    """
    start = servo.get_physical_angle()
    change = target - start
    dt = duration / steps

    for step in range(1, steps + 1):
        t = step * dt
        current_angle = easing_func(t, start, change, duration)
        servo.move(current_angle)
        # We'll do a blocking sleep here; you could also use asyncio.sleep if desired
        time.sleep(dt)
    # Ensure the target is reached at the end
    servo.move(target)

def print_final_angles(servo_list, home_dict, label):
    """
    Print final servo angles in both physical and 'centered' (relative-to-home) systems.
    
    Parameters:
        servo_list (list[int]): List of servo IDs to check.
        home_dict (dict[int,float]): Mapping from servo ID -> home angle.
        label (str): A label or phase name for reference in prints.
    """
    print(f"\n--- Final Angles After {label} ---")
    for s_id in servo_list:
        s = LX16A(s_id)
        phys = s.get_physical_angle()
        centered = phys - home_dict[s_id]
        print(f"Servo {s_id} => Centered: {centered:.2f}°, Physical: {phys:.2f}°")
    print("-----\n")

##############################################################################
#                          MAIN TASK (ASYNC)
##############################################################################
async def my_task():
    # Get the USD stage
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
    for servo_id in range(1, 7):  # Typically you only need to check up to the max servo count
        try:
            s = LX16A(servo_id)
            s.get_physical_angle()  # Will raise an exception if not found
            connected_servos.append(servo_id)
        except:
            pass

    print("Connected Servo IDs:", connected_servos)

    # If you already know which servos are used (IDs 1..5), you can skip scanning 
    # or at least confirm that each of those is in `connected_servos`.

    ############################################################################
    #                    Define Home Angles and Move to Home
    ############################################################################
    # Measured home positions for each servo
    home_angles = {
        1: 119.52,
        2: 99.6,
        3: 131.52,
        4: 114.24,
        5: 87.84
    }

    # Move each servo to home
    for s_id, h_angle in home_angles.items():
        if s_id in connected_servos:
            LX16A(s_id).move(h_angle)
    time.sleep(1.0)  # Let them settle

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
    link_length_m = link_length_vec[0]  # presumably the X component is the length
    link_length_cm = link_length_m * 100.0

    tail_pos = j5_pos + link_length_vec

    # Debug prints
    print("Link length vector (m):", link_length_vec)
    print("Per-segment length (cm):", round(link_length_cm, 3))

    total_rob_length = round(6 * link_length_cm, 3)
    print("Estimated total robot length (cm):", total_rob_length)

    ############################################################################
    #                        Calculate Desired Theta
    ############################################################################
    fwd_mmt_desired = 2.0  # in cm
    x = fwd_mmt_desired
    L = link_length_cm

    # theta = arccos(1 - x/(2L)), in radians
    theta_radians = math.acos(1 - (x / (2 * L)))
    theta_degrees = math.degrees(theta_radians)
    theta_degrees = round(float(theta_degrees), 3)

    ############################################################################
    #                      Collect USD Physics Drive APIs
    ############################################################################
    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")

    time.sleep(2)

    ############################################################################
    #                  PHASE 1: First Forward Bend
    ############################################################################
    print(f"Theta for forward displacement = {theta_degrees:.3f}°")

    # Relative angles (centered around home = 0°)
    target_angle1 = -theta_degrees
    target_angle2 = theta_degrees
    target_angle3 = theta_degrees
    target_angle4 = -theta_degrees
    target_angle5 = 0

    # Physical angles
    phys_1 = home_angles[1] + target_angle1
    phys_2 = home_angles[2] + target_angle2
    phys_3 = home_angles[3] + target_angle3
    phys_4 = home_angles[4] + target_angle4
    phys_5 = home_angles[5] + target_angle5

    print("Moving to first forward bend (2 seconds each)...")
    ease_to(LX16A(5), phys_5, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(4), phys_4, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(3), phys_3, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(2), phys_2, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(1), phys_1, duration=2, steps=50, easing_func=ease_in_out_cubic)

    time.sleep(6)  # let motion finish
    print_final_angles([1, 2, 3, 4, 5], home_angles, "PHASE 1")

    ############################################################################
    #                  PHASE 2: Second Bend / Variation
    ############################################################################
    # New angles
    target_angle11 = 0
    target_angle22 = -theta_degrees
    target_angle33 = theta_degrees
    target_angle44 = theta_degrees
    target_angle55 = -theta_degrees

    phys_11 = home_angles[1] + target_angle11
    phys_22 = home_angles[2] + target_angle22
    phys_33 = home_angles[3] + target_angle33
    phys_44 = home_angles[4] + target_angle44
    phys_55 = home_angles[5] + target_angle55

    print("Moving to second bend sequence...")
    # Also setting joint attributes in USD
    joint_5.GetTargetPositionAttr().Set(-theta_degrees)  # degrees
    joint_4.GetTargetPositionAttr().Set(theta_degrees)
    joint_3.GetTargetPositionAttr().Set(theta_degrees)
    joint_2.GetTargetPositionAttr().Set(-theta_degrees)
    await asyncio.sleep(4)
    ease_to(LX16A(5), phys_55, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(4), phys_44, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(3), phys_33, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(2), phys_22, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(1), phys_11, duration=2, steps=50, easing_func=ease_in_out_cubic)
    print_final_angles([1, 2, 3, 4, 5], home_angles, "PHASE 2")
    time.sleep(4)

    ############################################################################
    #         PHASE 3: Anchor Tail (Increase Mass) & Another Bend
    ############################################################################
    tail = "/worm_5dof/link_5"
    tailPrim = stage.GetPrimAtPath(tail)
    tm = UsdPhysics.MassAPI.Apply(tailPrim)
    # Increase mass to anchor tail
    tm.GetMassAttr().Set(5.0)
    await asyncio.sleep(2)
    print("Anchored tail...")

    # Another set of angles
    # Notice here we invert 'theta_degrees' in radians,
    # then set new target angles
    theta_rad = -math.radians(theta_degrees)
    target_angle1 = 0
    target_angle2 = math.degrees(theta_rad)
    target_angle3 = -math.degrees(theta_rad)
    target_angle4 = -math.degrees(theta_rad)
    target_angle5 = math.degrees(theta_rad)

    # Physical angles
    phys_1 = home_angles[1] + target_angle1
    phys_2 = home_angles[2] + target_angle2
    phys_3 = home_angles[3] + target_angle3
    phys_4 = home_angles[4] + target_angle4
    phys_5 = home_angles[5] + target_angle5

    # Set joint targets in USD
    joint_1.GetTargetPositionAttr().Set(math.degrees(theta_rad))
    joint_2.GetTargetPositionAttr().Set(-math.degrees(theta_rad))
    joint_3.GetTargetPositionAttr().Set(-math.degrees(theta_rad))
    joint_4.GetTargetPositionAttr().Set(math.degrees(theta_rad))
    joint_5.GetTargetPositionAttr().Set(0)

    await asyncio.sleep(2)
    print("Moving to anchored bend...")

    ############################################################################
    #     PHASE 4: Reset Tail Mass & Move Everything Back to Home
    ############################################################################
    print("Reset tail mass to normal...")
    tm.GetMassAttr().Set(0.03948)
    await asyncio.sleep(2)
    print("Reset Anchored tail...")

    # Move all joints (simulation and real) back to 0 offset
    target_angle1 = 0
    target_angle2 = 0
    target_angle3 = 0
    target_angle4 = 0
    target_angle5 = 0

    phys_1 = home_angles[1] + target_angle1
    phys_2 = home_angles[2] + target_angle2
    phys_3 = home_angles[3] + target_angle3
    phys_4 = home_angles[4] + target_angle4
    phys_5 = home_angles[5] + target_angle5

    joint_1.GetTargetPositionAttr().Set(0)
    joint_2.GetTargetPositionAttr().Set(0)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(0)
    joint_5.GetTargetPositionAttr().Set(0)
    await asyncio.sleep(2)

    print("Returning to home angles...")
    ease_to(LX16A(5), phys_5, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(4), phys_4, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(3), phys_3, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(2), phys_2, duration=2, steps=50, easing_func=ease_in_out_cubic)
    ease_to(LX16A(1), phys_1, duration=2, steps=50, easing_func=ease_in_out_cubic)
    time.sleep(3)

    print_final_angles([1, 2, 3, 4, 5], home_angles, "PHASE 4")

    ############################################################################
    #                       CHECK FINAL HEAD POSITION
    ############################################################################
    head_pos_new = head_prim.GetAttribute("xformOp:translate").Get()
    x1 = round(float(head_pos_old[0] * 100), 3)
    x2 = round(float(head_pos_new[0] * 100), 3)

    print(f"Old head X (cm): {x1}")
    print(f"New head X (cm): {x2}")
    print(f"Net movement (cm) = {x2 - x1:.3f}")

    j5_pos_new = j5_prim.GetAttribute("xformOp:translate").Get()
    print("Tail displacement (m) =", (j5_pos_new[0] - j5_pos[0]))

    ############################################################################
    #    ADDITIONAL DATA LOGGING AND PLOTTING (ANGLE, TEMP, VOLTAGE)
    ############################################################################
    # We'll collect ~2 seconds of data from all 5 servos, sampling every 0.1s.
    print("\nCollecting servo telemetry (angle, temperature, voltage) for 2 seconds...")

    time_data = []
    angle_data = {sid: [] for sid in [1, 2, 3, 4, 5]}
    temp_data = {sid: [] for sid in [1, 2, 3, 4, 5]}
    vin_data  = {sid: [] for sid in [1, 2, 3, 4, 5]}

    start_time = time.time()
    duration = 2.0  # seconds
    sample_interval = 0.1

    while True:
        now = time.time() - start_time
        if now > duration:
            break
        time_data.append(now)

        for sid in [1, 2, 3, 4, 5]:
            if sid in connected_servos:
                servo = LX16A(sid)
                angle_data[sid].append(servo.get_physical_angle())
                temp_data[sid].append(servo.get_temp())
                vin_data[sid].append(servo.get_vin())  # in millivolts
            else:
                # If not connected, append None or a sentinel
                angle_data[sid].append(None)
                temp_data[sid].append(None)
                vin_data[sid].append(None)

        time.sleep(sample_interval)

    print("Data collection complete. Now plotting...")

    # Generate basic plots for each servo
    # (One figure per servo)
    for sid in [1, 2, 3, 4, 5]:
        if sid not in connected_servos:
            continue  # skip unconnected servos
        plt.figure()
        plt.title(f"Servo {sid} Telemetry")
        plt.plot(time_data, angle_data[sid], label="Angle (deg)")
        plt.plot(time_data, temp_data[sid], label="Temp (°C)")
        # Convert from millivolts to volts for plotting
        plt.plot(time_data, [v/1000.0 if v else None for v in vin_data[sid]], label="Voltage (V)")
        plt.xlabel("Time (s)")
        plt.legend()
        plt.grid(True)
        plt.show()

##############################################################################
# Run the async task
##############################################################################
run_coroutine(my_task())
