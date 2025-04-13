import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

# -----------------------------------------------------
# 1. Define the new refined 9-step waypoints for servos
# -----------------------------------------------------
# Each sublist represents: [Servo1, Servo2, Servo3, Servo4, Servo5]
# waypoints = [
#     [103.4,  95.5, 207.1, 113.3, 62.6],
#     [121.0,  99.6,  76.6, 220.8, 38.9],
#     [121.2,  99.6,  52.6, 195.8, 38.9],
#     [121.0,  99.6,  97.2, 134.9, 38.9],
#     [121.0,  99.6, 120.0, 125.0, 91.0],
#     [121.0,  89.5, 125.0, 141.6, 91.0],
#     [121.0,  90.0, 125.0, 121.9, 91.0],
#     [120.7,  89.3, 125.0, 159.6, 91.0],
#     [120.7,  96.2, 129.1, 120.0, 91.0],
#     [94.3, 112.6, 212.9, 55.4, 87.6],
#     [108.7, 112.6, 127.4, 119.3, 87.6]
# ]

waypoints = [
    [103.4,  95.5, 207.1, 113.3, 62.6],
    [121.0,  99.6,  76.6, 220.8, 38.9],
    [121.2,  99.6,  52.6, 195.8, 38.9],
    [121.0,  99.6,  97.2, 134.9, 38.9],
    [121.0,  99.6, 120.0, 125.0, 91.0],
    [121.0,  89.5, 125.0, 141.6, 91.0],
    [121.0,  90.0, 125.0, 121.9, 91.0],
    [120.7,  89.3, 125.0, 159.6, 91.0],
    [120.7,  96.2, 129.1, 120.0, 91.0],
    # [94.3, 112.6, 212.9, 55.4, 87.6],
    # [108.7, 112.6, 127.4, 119.3, 87.6],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [86,133,164,83,82],
    [121,62,173,155,59],
    [76,133,164,83,82],
    [121,62,173,155,59],
    [76,133,164,83,82],
    [121,62,173,155,59],
    [76,133,164,83,82],
    [121,62,173,155,59],
    [76,133,164,83,82],
    [121,62,173,155,59],
    [76,133,164,83,82],
    [121,62,173,155,59],
    [119,100,130,114,87]
]


# Assume each waypoint is 1 second apart:
num_steps = len(waypoints)
t_waypoints = np.arange(num_steps)  # [0, 1, 2, ... 8]

# Convert to a NumPy array: shape is (9 x 5)
servo_angles = np.array(waypoints)

# -----------------------------------------------------
# 2. Fit a cubic spline to each servo’s data
# -----------------------------------------------------
splines = []
for servo in range(5):
    # For each servo, build a natural cubic spline
    cs = CubicSpline(t_waypoints, servo_angles[:, servo], bc_type='natural')
    splines.append(cs)

# Define a helper function that returns all servo angles at time t
def get_servo_angles(t):
    """
    For a given time 't', returns a list of interpolated servo angles
    [servo1_angle, servo2_angle, ..., servo5_angle].
    """
    return [spline(t) for spline in splines]

# Optionally, plot the raw waypoints and the spline trajectories
t_dense = np.linspace(0, num_steps - 1, 200)
plt.figure(figsize=(10, 6))
for servo in range(5):
    y_dense = splines[servo](t_dense)
    plt.plot(t_dense, y_dense, '-', label=f'Servo {servo+1} Trajectory')
    plt.plot(t_waypoints, servo_angles[:, servo], 'o', markersize=6)
plt.xlabel('Time (s)')
plt.ylabel('Servo Angle (°)')
plt.title('Cubic Spline Trajectory for Each Servo')
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------------------------------
# 3. Initialize the servos (if not already done)
# -----------------------------------------------------
# IMPORTANT: Adjust the port ("COM4") if needed.
LX16A.initialize("COM4", 0.1)
servo_ids = [1, 2, 3, 4, 5]
servos = []
for sid in servo_ids:
    try:
        servo = LX16A(sid)
        servos.append(servo)
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} not responding. Exiting...")
        quit()

# Enable torque on all servos so they hold the commanded positions.
for s in servos:
    try:
        s.enable_torque()  # or use s.set_torque_enable(True) if required
    except Exception as err:
        print(f"Error enabling torque for servo {s._id}: {err}")

print("Torque enabled for all servos.")

# -----------------------------------------------------
# 4. Continuous control loop: Move servos along the computed trajectory
# -----------------------------------------------------
# Define simulation parameters
t_total = t_waypoints[-1]  # Last waypoint time (8 seconds in this case)
dt = 0.05                  # Control loop period (50 ms)
t_current = 0.0            # Start time

print("Starting trajectory following...")
while t_current <= t_total:
    # Get interpolated angles at the current time
    angles = get_servo_angles(t_current)
    
    # Issue move commands for each servo.
    # Using a short move duration (dt in milliseconds) for incremental updates.
    for servo, angle in zip(servos, angles):
        try:
            # The move duration is set as dt (in ms), here we convert dt to ms
            servo.move(angle, int(dt * 1000))
        except ServoTimeoutError:
            print(f"Servo {servo._id} failed to move at time {t_current:.2f}s.")
    
    # Wait for dt seconds and increment the current time
    time.sleep(dt)
    t_current += dt

print("Trajectory completed.")
