import csv
import random
import os

# Create a CSV file with random joint values
def create_csv(filename, num_rows):
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
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


    joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/base_link/joint_1"), "angular")
    joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_1/joint_2"), "angular")
    joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_2/joint_3"), "angular")
    joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_3/joint_4"), "angular")
    joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/worm_5dof/link_4/joint_5"), "angular")

    # Read joint values from CSV file
    with open(filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            # Set target positions for the driven joints from the CSV row
            joint_1_value = float(row["joint_1"])
            joint_2_value = float(row["joint_2"])
            joint_3_value = float(row["joint_3"])
            joint_4_value = float(row["joint_4"])
            joint_5_value = float(row["joint_5"])

            joint_1.GetTargetPositionAttr().Set(joint_1_value)
            joint_2.GetTargetPositionAttr().Set(joint_2_value)
            joint_3.GetTargetPositionAttr().Set(joint_3_value)
            joint_4.GetTargetPositionAttr().Set(joint_4_value)
            joint_5.GetTargetPositionAttr().Set(joint_5_value)

            # Output the values being fed into the joints
            print(f"Driving joints with values: joint_1 = {joint_1_value}, joint_2 = {joint_2_value}, joint_3 = {joint_3_value},  joint_4 = {joint_4_value},joint_5 = {joint_5_value}")

            # Wait for the specified time interval before moving to the next row
            await asyncio.sleep(2)

# Start the asyncio task to drive the joints from the CSV file
asyncio.ensure_future(drive_joints_from_csv('joint_values.csv'))

