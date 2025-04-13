import asyncio
import omni
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdLux
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.range_sensor import _range_sensor
import numpy as np

# ------------------------------------------------------------------
# Async function: Spawn ultrasonic sensor with multiple emitters/firing groups.
# ------------------------------------------------------------------
async def spawn_ultrasonic_function(task):
    done, pending = await asyncio.wait({task})
    if task in done:
        stage = omni.usd.get_context().get_stage()

        # Set up stage: Z-up and 1.0 meter per unit.
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        # Create a physics scene.
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

        # Define emitter poses and adjacency lists.
        origin = Gf.Vec3d(4.8, 6.4, 0.0)
        emitter_poses = [
            ((0, 0, 75.0),   Gf.Vec3d(3.844, 0.9384, 0.525)),
            ((0, 0, 30.0),   Gf.Vec3d(4.046, 0.7735, 0.56)),
            ((0, 0, 11.8),   Gf.Vec3d(4.172, 0.3256, 0.591)),
            ((0, 0, -11.8),  Gf.Vec3d(4.172, -0.3256, 0.591)),
            ((0, 0, -30.0),  Gf.Vec3d(4.046, -0.7735, 0.561)),
            ((0, 0, -75.0),  Gf.Vec3d(3.844, -0.9384, 0.525)),
            ((0, 0, 99.2),   Gf.Vec3d(-1.454, 0.9352, 0.5367)),
            ((0, 0, 150.0),  Gf.Vec3d(-1.789, 0.788, 0.558)),
            ((0, 0, 175.5),  Gf.Vec3d(-1.887, 0.36, 0.6249)),
            ((0, 0, -175.5), Gf.Vec3d(-1.887, -0.36, 0.6249)),
            ((0, 0, -150.0), Gf.Vec3d(-1.789, -0.788, 0.558)),
            ((0, 0, -99.2),  Gf.Vec3d(-1.454, -0.9352, 0.5367)),
        ]
        adjacency = [
            [0, 1],
            [0, 1, 2],
            [1, 2, 3],
            [2, 3, 4],
            [3, 4, 5],
            [4, 5],
            [6, 7],
            [6, 7, 8],
            [7, 8, 9],
            [8, 9, 10],
            [9, 10, 11],
            [10, 11],
        ]

        # Create ultrasonic emitter prims.
        emitters = []
        for i in range(len(emitter_poses)):
            pose = emitter_poses[i]
            adj_list = adjacency[i]
            result, emitter_prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicEmitter",
                path="/World/UltrasonicEmitter",
                per_ray_intensity=0.4,
                yaw_offset=0.0,
                adjacency_list=adj_list,
            )
            # Set translation and rotation.
            emitter_prim.GetPrim().GetAttribute("xformOp:translate").Set(origin + pose[1])
            emitter_prim.GetPrim().GetAttribute("xformOp:rotateXYZ").Set(pose[0])
            emitters.append(emitter_prim)
        emitter_paths = [e.GetPath() for e in emitters]

        # Create two firing groups.
        result, group_1 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup_0",
            emitter_modes=[(0, 1), (3, 0), (4, 1), (7, 0), (8, 1), (11, 0)],
            receiver_modes=[
                (0, 1), (1, 1), (2, 0), (3, 0),
                (3, 1), (4, 0), (4, 1), (5, 1),
                (6, 0), (7, 0), (7, 1), (8, 0),
                (8, 1), (9, 1), (10, 0), (11, 0),
            ],
        )
        result, group_2 = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicFiringGroup",
            path="/World/UltrasonicFiringGroup_1",
            emitter_modes=[(1, 1), (2, 0), (5, 1), (6, 0), (9, 1), (10, 0)],
            receiver_modes=[
                (0, 1), (1, 0), (1, 1), (2, 0),
                (2, 1), (3, 0), (4, 1), (5, 1),
                (6, 0), (7, 0), (8, 1), (9, 0),
                (9, 1), (10, 0), (10, 1), (11, 0),
            ],
        )

        ultrasonicPath = "/World/UltrasonicArray"
        result, ultrasonic = omni.kit.commands.execute(
            "RangeSensorCreateUltrasonicArray",
            path=ultrasonicPath,
            min_range=0.4,
            max_range=4.5,
            draw_points=False,
            draw_lines=True,
            horizontal_fov=90.0,
            vertical_fov=15.0,
            horizontal_resolution=0.3,
            vertical_resolution=0.5,
            num_bins=224,
            emitter_prims=emitter_paths,
            firing_group_prims=[group_1.GetPath(), group_2.GetPath()],
        )

        # Adjust camera view.
        set_camera_view(
            eye=[20.0, 10.0, 5.0],
            target=[5.0, 5.0, 0.0],
            camera_prim_path="/OmniverseKit_Persp"
        )
        print("Ultrasonic sensor created at", ultrasonicPath)
        return ultrasonic, ultrasonicPath

# ------------------------------------------------------------------
# Function: Spawn obstacles (cube and cylinder with distant light).
# ------------------------------------------------------------------
def spawn_obstacles():
    stage = omni.usd.get_context().get_stage()
    cube_path = "/World/Cube"
    cylinder_path = "/World/Cylinder"
    offset = Gf.Vec3f(-0.4636036, 7.2820291, 0.618376)
    offset_cylinder = Gf.Vec3f(3.8492474, 2.0546415, 0.6868243)
    size = 1.00
    cylinder_height = 2.00
    radius = 0.10

    if get_prim_at_path("/World/DistantLight"):
        delete_prim("/World/DistantLight")
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
    distantLight.CreateIntensityAttr(500)
    distantLight.AddRotateXYZOp().Set((-36, 36, 0))

    if stage.GetPrimAtPath(cube_path):
        return

    cylinderGeom = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cubeGeom = UsdGeom.Cube.Define(stage, cube_path)
    cubePrim = stage.GetPrimAtPath(cube_path)
    cylinderPrim = stage.GetPrimAtPath(cylinder_path)

    cubeGeom.CreateSizeAttr(size)
    cylinderGeom.CreateHeightAttr(cylinder_height)
    cylinderGeom.CreateRadiusAttr(radius)
    cubeGeom.AddTranslateOp().Set(offset)
    cylinderGeom.AddTranslateOp().Set(offset_cylinder)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    UsdPhysics.CollisionAPI.Apply(cylinderPrim)
    print("Obstacles created at", cube_path, "and", cylinder_path)

# ------------------------------------------------------------------
# Function: Read sensor data and envelope array.
# ------------------------------------------------------------------
def get_info_function(ultrasonic, ultrasonicPath, sensor_interface):
    try:
        if ultrasonic is None:
            print("Ultrasonic sensor not found.")
            return
        if not omni.usd.get_context().get_stage().GetPrimAtPath(ultrasonicPath).IsValid():
            print("Ultrasonic sensor prim is not valid.")
            return
        maxDepth = ultrasonic.GetMaxRangeAttr().Get()
        # Get sensor data from the C++ interface.
        depth = sensor_interface.get_depth_data(ultrasonicPath, 5)
        zenith = sensor_interface.get_zenith_data(ultrasonicPath)
        azimuth = sensor_interface.get_azimuth_data(ultrasonicPath)
        
        # Format table string.
        tableString = ""
        numCols = len(zenith)
        rowString = ""
        for i in range(numCols):
            rowString += "{" + str(i + 2) + ":.5f}   "
        rowString = "{0:16}  {1:10}" + rowString + "\n"
        
        tableString += rowString.format("Azimuth \\ Zenith", " | ", *zenith)
        tableString += "-" * len(tableString) + "\n"
        for row, cols in enumerate(depth):
            entry = [ray * maxDepth / 65535.0 for ray in cols]
            tableString += rowString.format("{0:.5f}".format(azimuth[row]), " | ", *entry)
        
        print("Sensor Data:")
        print(tableString)
    except Exception as e:
        print("Error in get_info_function:", e)

def draw_envelope_frame(sensor_interface, ultrasonicPath):
    try:
        envelope_arr = sensor_interface.get_envelope_array(ultrasonicPath)
        print("Envelope Array:")
        if envelope_arr is not None:
            for i in range(envelope_arr.shape[0]):
                print(f"Bin {i}: {envelope_arr[i].tolist()}")
        else:
            print("No envelope data available.")
    except Exception as e:
        print("Error in draw_envelope_frame:", e)

# ------------------------------------------------------------------
# Main: Create stage, sensor, obstacles, and start a loop to read sensor data.
# ------------------------------------------------------------------
async def main():
    # Create a new stage and spawn the ultrasonic sensor.
    task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
    ultrasonic, ultrasonicPath = await spawn_ultrasonic_function(task)
    
    # Wait a short time for sensor initialization.
    await asyncio.sleep(2.0)
    
    # Spawn obstacles.
    spawn_obstacles()
    
    # Acquire the ultrasonic sensor interface.
    sensor_interface = _range_sensor.acquire_ultrasonic_sensor_interface()
    
    # Periodically read sensor info and envelope data.
    while True:
        get_info_function(ultrasonic, ultrasonicPath, sensor_interface)
        draw_envelope_frame(sensor_interface, ultrasonicPath)
        await asyncio.sleep(1.0)  # Adjust polling rate as needed.

# Launch the main function.
asyncio.ensure_future(main())
