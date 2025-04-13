import json
import asyncio
from pxr import UsdPhysics

async def drive_joints_from_json(json_string):
    # Parse the JSON string into a dictionary.
    data = json.loads(json_string)
    
    # Get the code snippet from the JSON object.
    code_str = data.get("code", "")
    
    # Execute the code snippet safely to extract variable 'a'.
    local_vars = {}
    try:
        exec(code_str, {}, local_vars)
    except Exception as e:
        print("Error executing JSON code snippet:", e)
        return
    
    a = local_vars.get("a")
    if a is None:
        print("Error: 'a' was not defined in the JSON code snippet.")
        return

    # Extract joint angles from 'a' directly (assuming they are in degrees).
    try:
        joint_1_value = float(a[0])
        joint_2_value = float(a[1])
        joint_3_value = float(a[2])
        joint_4_value = float(a[3])
        # Check if the fifth element is a list before indexing.
        if isinstance(a[4], list):
            joint_5_value = float(a[4][1])
        else:
            joint_5_value = float(a[4])
    except Exception as e:
        print("Error processing joint angles:", e)
        return

    # Retrieve the USD stage from the Omniverse context.
    stage = omni.usd.get_context().get_stage()

    # Define prim paths for the joints.
    joint_paths = [
        "/worm_5dof/base_link/joint_1",
        "/worm_5dof/link_1/joint_2",
        "/worm_5dof/link_2/joint_3",
        "/worm_5dof/link_3/joint_4",
        "/worm_5dof/link_4/joint_5"
    ]
    joint_values = [joint_1_value, joint_2_value, joint_3_value, joint_4_value, joint_5_value]
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

    # Process each joint: verify the prim exists and then update its target position.
    for path, value, name in zip(joint_paths, joint_values, joint_names):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            print(f"Error: Invalid prim at path {path} for {name}.")
            continue
        try:
            drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
            drive_api.GetTargetPositionAttr().Set(value)
            print(f"Updated {name} to {value} degrees.")
        except Exception as e:
            print(f"Error updating {name} at {path}: {e}")

    print("Joint angles updated from JSON.")

# Example JSON input with the "code" field.
json_input = '''{
    "task_id": 1,
    "script_title": "array",
    "task_description": "Accessing and manipulating nested elements within an array.",
    "code": "a = [0, 25, 35, -45, 25]"
}'''

asyncio.ensure_future(drive_joints_from_json(json_input))
