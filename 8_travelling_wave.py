import asyncio
import math
from pxr import UsdPhysics

async def drive_joints():
    stage = omni.usd.get_context().get_stage()

    # Get the joint references for each of the 5 segments
    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_4/joint_5"), "angular")

    joints = [joint_1, joint_2, joint_3, joint_4, joint_5]

    # Parameters
    n_segments = 5              # Number of worm segments
    amplitude = 1             # Amplitude of the traveling wave (meters)
    wavelength = 4              # Wavelength of the wave (meters)
    frequency = 1               # Frequency of wave propagation (Hz)
    dt = 0.05                   # Time step (seconds)
    speed = 0.2                # Forward speed of the worm (meters per time step)
    total_time = 100             # Total simulation time (seconds)

    # Initialize segment positions along x-axis
    x = [i * (wavelength / n_segments) for i in range(n_segments)]
    t = 0

    while t < total_time:
        # Update position and orientation of each segment
        for i in range(n_segments):
            phase = 2 * math.pi * (frequency * t - x[i] / wavelength)  # Wave phase
            servo_angle = math.degrees(math.sin(phase) * amplitude)     # Calculate servo angle based on wave

            # Set the target position for the joint
            joints[i].GetTargetPositionAttr().Set(servo_angle)

        # Wait for the next time step
        await asyncio.sleep(dt)
        t += dt

# Start the asyncio task to drive the joints
asyncio.ensure_future(drive_joints())
