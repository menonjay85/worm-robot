import asyncio
import time
import weakref

import numpy as np
import omni
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.ui as ui
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import btn_builder, get_style, setup_ui_headers, str_builder
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

_sensor = _range_sensor.acquire_generic_sensor_interface()
_timeline = omni.timeline.get_timeline_interface()
stage = omni.usd.get_context().get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

_pattern_set = False
_generic = False
_plot = False
_sampling_rate = 2.4e5  # number of samples per second
_plot_duration = 4  # seconds to collect sample before plotting
_record_start = time.perf_counter()
_streaming = True

_genericPath = "/World/GenericSensor"
_generic = RangeSensorSchema.Generic.Define(stage, Sdf.Path(_genericPath))

_generic.CreateStreamingAttr().Set(_streaming)

_generic.CreateMinRangeAttr().Set(0.4)
_generic.CreateMaxRangeAttr().Set(100.0)

_generic.CreateSamplingRateAttr().Set(_sampling_rate)
_generic.CreateDrawPointsAttr().Set(True)
_generic.CreateDrawLinesAttr().Set(True)

set_camera_view(eye=[-5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

print("sending more data")

def _test_streaming_data():
    batch_size = int(1e6)  # size of each batch of data being processed
    half_batch = int(batch_size / 2)
    frequency = 10
    N_pts = int(batch_size / frequency / 2)
    # azimuth angle zigzag between the limits (frequency) times every batch
    azimuth = np.tile(np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency)
    # zenith angle goes up and down once every batch
    zenith = np.append(np.linspace(-np.pi / 4, np.pi / 4, half_batch), np.linspace(np.pi / 4, -np.pi / 4, half_batch))
    # custom pattern must be sent as an arrya of [azimuth, zenith] angles.
    sensor_pattern = np.stack((azimuth, zenith))
    origin_offsets = 0.05 * np.random.random((batch_size, 3))
    return sensor_pattern, origin_offsets

def _test_repeating_data(self):
    batch_size = int(1e6)  # size of each batch of data being processed
    half_batch = int(batch_size / 2)
    frequency = 10
    N_pts = int(batch_size / frequency / 2)
    azimuth = np.tile(np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency)
    zenith = np.append(-0.5 * np.ones(half_batch), 0.5 * np.ones(half_batch))
    sensor_pattern = np.stack((azimuth, zenith))
    origin_offsets = 0.05 * np.random.random((batch_size, 3))
    return sensor_pattern, origin_offsets

if _streaming:
	sensor_pattern, origin_offsets = _test_streaming_data()
else:
	sensor_pattern, origin_offsets = _test_repeating_data()

_sensor.set_next_batch_rays(_genericPath, sensor_pattern)
_sensor.set_next_batch_offsets(_genericPath, origin_offsets)

if _plot:
    if (time.perf_counter() - _record_start) < _plot_duration:
        _point_cloud_data = np.append(
            _point_cloud_data, _sensor.get_point_cloud_data(_genericPath), axis=0
        )
    else:
        _plot = False
        _plot_pattern(_point_cloud_data)

CubePath = "/World/Wall"
offset = Gf.Vec3f(2.00, 0.0, 0.0)
size = 1


# To create a cube, we first define our geometry at our chosen path.  Then, becuase
# we will need the primitive later, we query the prim from the stage. If the prim already exists, skip creation
cubeGeom = UsdGeom.Cube.Define(stage, CubePath)
cubePrim = stage.GetPrimAtPath(CubePath)

# Remember!  Attributes do not exist until they are created.  Here we set the value to the non defualt at
# creation.  Note that moving the cube to a different location involves adding a translation operation to
# our primitive.
cubeGeom.CreateSizeAttr(size)
cubeGeom.AddTranslateOp().Set(offset)
UsdGeom.XformCommonAPI(cubePrim).SetScale((1.00, 5.00, 4.00))

# In order for our cube to interact with the LIDAR, it needs to be able to colide with our physX line traces.
# to do this, we give our cube the collision API, and set it's material and collision group.
UsdPhysics.CollisionAPI.Apply(cubePrim)