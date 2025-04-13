from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf

# Acquire the dynamic control interface
dc = _dynamic_control.acquire_dynamic_control_interface()

# Define the path to the articulation (update this with your articulation's root path)
articulation_path = "/new_assem_wo_mating/base_link"

# Get the articulation
art = dc.get_articulation(articulation_path)
if art == _dynamic_control.INVALID_HANDLE:
    raise ValueError(f"Articulation at path {articulation_path} not found.")

# Get the number of joints
num_joints = dc.get_articulation_joint_count(art)

# Initialize a dictionary to store joint coordinates
joint_coordinates = {}

# Get the body states of all articulation bodies
body_states = dc.get_articulation_body_states(art, _dynamic_control.STATE_POS)

# Iterate through each joint to retrieve its position
for joint_index in range(num_joints):
    # Get the joint handle
    joint_handle = dc.get_articulation_joint(art, joint_index)
    if joint_handle == _dynamic_control.INVALID_HANDLE:
        continue

    # Get the name of the joint
    joint_name = dc.get_joint_name(joint_handle)

    # Retrieve the position of the joint's associated body in world coordinates
    try:
        joint_position = body_states["pose"]["p"][joint_index]  # Ensure we access the correct index
    except IndexError:
        print(f"Joint index {joint_index} out of bounds for body_states.")
        continue

    # Convert the position to Gf.Vec3d and round to 5 decimal places
    rounded_position = Gf.Vec3d(
        round(float(joint_position[0]), 5),
        round(float(joint_position[1]), 5),
        round(float(joint_position[2]), 5),
    )

    # Store the rounded position in the dictionary
    joint_coordinates[joint_name] = rounded_position

# Print the joint coordinates
for joint, coords in joint_coordinates.items():
    print(f"Joint: {joint}, Coordinates: {coords}")
