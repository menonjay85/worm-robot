import omni.kit.app
import omni.client
from omni.kit.async_engine import run_coroutine
from pxr import Sdf, UsdPhysics
import omni.usd
import math
import asyncio
import time
from pylx16a.lx16a import *

# Additional imports for Koopman parts
import numpy as np
import tensorflow as tf
from scipy.integrate import odeint

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
#              DESIRED TRAJECTORY & VELOCITY FUNCTIONS
##############################################################################
def cubic_trajectory(p0, pf, t, T):
    """
    Computes the desired position using a cubic polynomial trajectory.
    Boundary conditions: p(0)=p0, p(T)=pf, with zero initial and final velocities.
    """
    tau = t / T  # normalized time
    return p0 + (pf - p0) * (3*tau**2 - 2*tau**3)

def cubic_trajectory_velocity(p0, pf, t, T):
    """
    Computes the derivative (velocity) of the cubic trajectory.
    """
    tau = t / T
    return (pf - p0) * (6*tau/T - 6*tau**2/T)

##############################################################################
#                  KOOPMAN OPERATOR FUNCTIONS (Option 1)
##############################################################################
def calc_Koopman(theta1, theta2, theta3, theta4, theta5):
    """
    Constructs a state vector of length 6 (5 servo angles plus a padded value)
    and uses a sliding window (X[:-1] and X[1:]) to train a neural network that
    predicts the next state and computes a Koopman operator.
    """
    # Create a state vector of length 6.
    X = np.zeros(6)
    X[0] = theta1
    X[1] = theta2
    X[2] = theta3
    X[3] = theta4
    X[4] = theta5
    X[5] = 0  # Padding (could also be the mean of the angles)
    
    # Construct training data using a sliding window.
    X_train = X[:-1].reshape(-1, 1)  # Shape (5, 1)
    Y_train = X[1:].reshape(-1, 1)   # Shape (5, 1)
    
    print("State vector X:")
    print(X)
    print("X_train:")
    print(X_train)
    print("Y_train:")
    print(Y_train)
    
    # Build and train a simple neural network.
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(64, activation='relu', input_dim=1),
        tf.keras.layers.Dense(32, activation='sigmoid'),
        tf.keras.layers.Dense(1)
    ])
    
    model.compile(optimizer='adam', loss='mean_squared_logarithmic_error')
    history = model.fit(X_train, Y_train, epochs=100, verbose=0)
    
    # Get predicted next-state values.
    X_pred = model.predict(X_train)
    print("X_pred:")
    print(X_pred)
    
    # Compute the Koopman operator via a least-squares solution.
    K = np.dot(X_pred, np.linalg.pinv(X_train))
    print("Koopman operator:")
    print(K)
    print("Koopman operator shape:", K.shape)
    print("Eigenvalues:", np.linalg.eigvals(K))
    
    return K

def KModel(y, t, K):
    """
    Applies the Koopman operator as a scalar multiplier.
    y is expected to be a vector of 5 elements.
    """
    # For a 1x1 Koopman operator (scalar), multiply each element.
    ydot = np.array(y) * K[0, 0]
    return ydot

##############################################################################
#               KOOPMAN-PID CONTROL FUNCTION
##############################################################################
def koopman_pid_control_all(servos, targets, duration, dt=0.05, Kp=0.7, Ki=0.01, Kd=0.009):
    """
    Uses a PID loop combined with a Koopman correction to drive each servo to its target
    following a cubic trajectory.
    
    Parameters:
      servos (list): List of LX16A servo objects.
      targets (list): Desired final angles for each servo.
      duration (float): Total time for the motion.
      dt (float): Control loop time step.
      Kp, Ki, Kd: PID gains.
    """
    # Record initial positions for each servo.
    p0s = [servo.get_physical_angle() for servo in servos]
    # Initialize PID state variables.
    prev_errors = [0.0 for _ in servos]
    integrals = [0.0 for _ in servos]
    steps = int(duration / dt)
    
    for step in range(steps + 1):
        t = step * dt
        
        # Calculate desired positions from cubic trajectories.
        desired_state = []
        for i, servo in enumerate(servos):
            p0 = p0s[i]
            pf = targets[i]
            p_desired = cubic_trajectory(p0, pf, t, duration)
            desired_state.append(p_desired)
        
        # PID loop: compute corrections for each servo.
        pid_corrections = []
        for i, servo in enumerate(servos):
            p_actual = servo.get_physical_angle()
            error = desired_state[i] - p_actual
            integrals[i] += error * dt
            derivative = (error - prev_errors[i]) / dt if dt > 0 else 0.0
            pid_output = Kp * error + Ki * integrals[i] + Kd * derivative
            pid_corrections.append(pid_output)
            prev_errors[i] = error
        
        # Combine desired positions with PID corrections.
        corrected_state = [desired_state[i] + pid_corrections[i] for i in range(len(servos))]
        
        # --- Koopman Correction ---
        # Use the (uncorrected) desired state as input for computing the Koopman operator.
        K = calc_Koopman(desired_state[0], desired_state[1],
                         desired_state[2], desired_state[3],
                         desired_state[4])
        Y0 = np.array(desired_state)
        T_new = np.linspace(0, dt, num=1000)
        Y = odeint(KModel, Y0, T_new, args=(K,))
        predicted_state = Y[-1]
        # ---------------------------------
        
        # Command each servo with the Koopman-PID corrected angle.
        for i, servo in enumerate(servos):
            new_angle = predicted_state[i]
            if 0 < new_angle < 240:
                servo.move(new_angle)
        time.sleep(dt)
    
    # Finally, ensure each servo reaches the target.
    for i, servo in enumerate(servos):
        servo.move(targets[i])

##############################################################################
#                              INITIALIZATION
##############################################################################
LX16A.initialize("COM4", 0.1)

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
    global servo1, servo2, servo3, servo4, servo5  # Ensure servos are in scope
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
    
    ############################################################################
    #                Move to Home Before Starting Gait 1
    ############################################################################
    print("Returning to home positions before starting gait 1...")
    koopman_pid_control_all([servo1, servo2, servo3, servo4, servo5],
                             [home_angles[1], home_angles[2], home_angles[3], home_angles[4], home_angles[5]],
                             duration=1, dt=0.05, Kp=0.7, Ki=0.01, Kd=0.009)
    time.sleep(1)
    
    ############################################################################
    #                     First Forward Bend (Gait 1)
    ############################################################################
    cmd_1 = home_angles[1] - theta_degrees
    cmd_2 = home_angles[2] + theta_degrees
    cmd_3 = home_angles[3] + theta_degrees
    cmd_4 = home_angles[4] - theta_degrees
    cmd_5 = home_angles[5]  # unchanged
    print("Moving to first forward bend (using Koopman-PID control)...")
    koopman_pid_control_all([servo2, servo3, servo1, servo4, servo5],
                             [cmd_2, cmd_3, cmd_1, cmd_4, cmd_5],
                             duration=1, dt=0.05, Kp=0.7, Ki=0.01, Kd=0.009)
    time.sleep(1)
    
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
    
    ############################################################################
    #                     Second Forward Bend (Gait 2)
    ############################################################################
    cmd_11 = home_angles[1]
    cmd_22 = home_angles[2] - theta_degrees
    cmd_33 = home_angles[3] + theta_degrees
    cmd_44 = home_angles[4] + theta_degrees
    cmd_55 = home_angles[5] - theta_degrees
    print("Moving to second forward bend (using Koopman-PID control)...")
    koopman_pid_control_all([servo2, servo4, servo1, servo5, servo3],
                             [cmd_22, cmd_44, cmd_11, cmd_55, cmd_33],
                             duration=1, dt=0.05, Kp=0.7, Ki=0.01, Kd=0.009)
    time.sleep(1)
    
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
    
    ############################################################################
    #                Move Back to Home After Finishing Gait 2
    ############################################################################
    print("Returning to home positions after finishing gait 2...")
    koopman_pid_control_all([servo1, servo2, servo3, servo4, servo5],
                             [home_angles[1], home_angles[2], home_angles[3], home_angles[4], home_angles[5]],
                             duration=1, dt=0.05, Kp=0.7, Ki=0.01, Kd=0.009)
    time.sleep(1)
    joint_5.GetTargetPositionAttr().Set(0)
    joint_4.GetTargetPositionAttr().Set(0)
    joint_3.GetTargetPositionAttr().Set(0)
    joint_2.GetTargetPositionAttr().Set(0)
    joint_1.GetTargetPositionAttr().Set(0)
    await asyncio.sleep(4)

run_coroutine(my_task())
