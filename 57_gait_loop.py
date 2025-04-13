from pylx16a.lx16a import LX16A, ServoTimeoutError
import time

LX16A.initialize("COM4", 0.1)

try:
    servo1 = LX16A(1); servo1.set_angle_limits(0, 240)
    servo2 = LX16A(2); servo2.set_angle_limits(0, 240)
    servo3 = LX16A(3); servo3.set_angle_limits(0, 240)
    servo4 = LX16A(4); servo4.set_angle_limits(0, 240)
    servo5 = LX16A(5); servo5.set_angle_limits(0, 240)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting…")
    quit()

servos = [servo1, servo2, servo3, servo4, servo5]

home = [119, 100, 130, 114, 87]
pos1 = [86, 133, 164, 83, 82]
pos2 = [121,  62, 173,155, 59]

# -------- easing helper --------
def move_servos_eased(order, positions, duration_ms=500):
    """
    order      – list of servo indices (0‑based) in the order you want them to start
    positions  – list/tuple of target angles (deg) for all 5 servos
    duration_ms– how long each servo should take to reach its target
    """
    for i in order:
        try:
            servos[i].move(positions[i], duration_ms)  # built‑in timed move
        except ServoTimeoutError:
            print(f"Servo {i+1} failed to move.")
    # give the slowest move time to finish (+ a small buffer)
    time.sleep(duration_ms/1000.0 + 0.05)

# keep your original four movement “chunks”
def move_home_start(): move_servos_eased([0,1,2,3,4], home)
def move_pos1():       move_servos_eased([1,2,0,3,4], pos1)
def move_pos2():       move_servos_eased([1,3,0,4,2], pos2)
def move_home_end():   move_servos_eased([1,3,0,4,2], home)

def move_fwd():
    move_home_start()
    move_pos1()
    move_pos2()
    move_home_end()

# run 4 gait cycles
for _ in range(14):
    move_fwd()

# read back angles safely
for idx, s in enumerate(servos, start=1):
    try:
        print(f"Servo {idx} angle: {s.get_physical_angle():.1f}°")
    except ServoTimeoutError:
        print(f"Servo {idx} not responding to angle read.")
