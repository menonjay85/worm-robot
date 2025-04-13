import asyncio
import math
from pxr import UsdPhysics

# Custom Geometric Algebra class for 2D Clifford Algebra operations
class GeometricAlgebra2D:
    def __init__(self, scalar=0, e1=0, e2=0):
        self.scalar = scalar  # Scalar part
        self.e1 = e1          # e1 basis component (x-direction)
        self.e2 = e2          # e2 basis component (y-direction)

    def __add__(self, other):
        return GeometricAlgebra2D(self.scalar + other.scalar, self.e1 + other.e1, self.e2 + other.e2)

    def __mul__(self, other):
        # Geometric product (Clifford Algebra) for two 2D multivectors
        scalar = self.scalar * other.scalar + self.e1 * other.e1 + self.e2 * other.e2
        e1 = self.scalar * other.e1 + self.e1 * other.scalar
        e2 = self.scalar * other.e2 + self.e2 * other.scalar
        return GeometricAlgebra2D(scalar, e1, e2)

    def rotate(self, angle):
        # Rotate using a rotor defined by angle
        rotor = GeometricAlgebra2D(math.cos(angle / 2), 0, math.sin(angle / 2))
        return rotor * self * rotor

    def angle(self):
        # Calculate angle based on e1 and e2 components
        return math.degrees(math.atan2(self.e2, self.e1))

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
    amplitude = 0.1             # Amplitude of the traveling wave (meters)
    wavelength = 2              # Wavelength of the wave (meters)
    frequency = 1               # Frequency of wave propagation (Hz)
    dt = 0.05                   # Time step (seconds)
    total_time = 100             # Total simulation time (seconds)

    # Initialize segment positions along x-axis
    x = [i * (wavelength / n_segments) for i in range(n_segments)]
    t = 0

    while t < total_time:
        # Update position and orientation of each segment 
        for i in range(n_segments):
            phase = 2 * math.pi * (frequency * t + x[i] / wavelength)  # Wave phase
            rotation = GeometricAlgebra2D(1, 0, 0).rotate(amplitude * math.sin(phase))  # Rotation using Clifford Algebra

            # Calculate servo angle from rotation
            servo_angle = rotation.angle()

            # Set the target position for the joint
            joints[i].GetTargetPositionAttr().Set(servo_angle)

        # Wait for the next time step
        await asyncio.sleep(dt)
        t += dt

# Start the asyncio task to drive the joints
asyncio.ensure_future(drive_joints())
