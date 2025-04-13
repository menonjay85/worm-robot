import csv
import random
import os

# Create a CSV file with random joint values
def create_csv(filename, num_rows):
    joint_names = ["joint_2", "joint_3"]
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["time"] + joint_names)  # Header row

        for i in range(num_rows):
            time = i * 2  # Assuming 2 seconds between each row
            joint_values = [random.uniform(-60, 0) for _ in joint_names]
            writer.writerow([time] + joint_values)

    # Output the directory where the CSV file has been created
    print(f"CSV file created at: {os.path.abspath(filename)}")

# Create a CSV file with 10 rows of random joint values
# create_csv('joint_values.csv', 10)

import asyncio
import csv
from pxr import UsdPhysics

async def drive_joints_from_csv(filename):
    stage = omni.usd.get_context().get_stage()

    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/new_assem_wo_mating/link_4/joint_5"), "angular")

    # Read joint values from CSV file
    with open(filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            # Set target positions for the driven joints from the CSV row
            joint_2_value = float(row["joint_2"])
            joint_3_value = float(row["joint_3"])

            joint_2.GetTargetPositionAttr().Set(joint_2_value)
            joint_3.GetTargetPositionAttr().Set(joint_3_value)

            # Set the rest of the joints to zero
            joint_1.GetTargetPositionAttr().Set(0.0)
            joint_4.GetTargetPositionAttr().Set(0.0)
            joint_5.GetTargetPositionAttr().Set(0.0)

            # Output the values being fed into the joints
            print(f"Driving joints with values: joint_2 = {joint_2_value}, joint_3 = {joint_3_value}")

            # Wait for the specified time interval before moving to the next row
            await asyncio.sleep(2)

# Start the asyncio task to drive the joints from the CSV file
asyncio.ensure_future(drive_joints_from_csv('joint_values.csv'))
