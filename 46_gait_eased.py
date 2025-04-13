from pylx16a.lx16a import LX16A, ServoTimeoutError
import time
import math

# Initialize communication
LX16A.initialize("COM3", 0.1)

try:
    servo1 = LX16A(1)
    servo1.set_angle_limits(0, 240)
    servo2 = LX16A(2)
    servo2.set_angle_limits(0, 240)
    servo3 = LX16A(3)
    servo3.set_angle_limits(0, 240)
    servo4 = LX16A(4)
    servo4.set_angle_limits(0, 240)
    servo5 = LX16A(5)
    servo5.set_angle_limits(0, 240)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

# Define target positions
home = [119, 100, 130, 114, 87]
pos1 = [86, 133, 164, 83, 82]
pos2 = [121, 62, 173, 155, 59]

# -------------------------
# EASING FUNCTION DEFINITION
# -------------------------
def ease_in_out_cubic(t, b, c, d):
    t = t / (d / 2)
    if t < 1:
        return c / 2 * t**3 + b
    t -= 2
    return c / 2 * (t**3 + 2) + b

# -------------------------
# Safe reading for servo angle with retries
# -------------------------
def safe_get_physical_angle(servo, retries=3, delay=0.1):
    for i in range(retries):
        try:
            return servo.get_physical_angle()
        except ServoTimeoutError as e:
            # Optionally print a warning:
            print(f"Warning: Servo {servo._id} did not respond, retrying ({i+1}/{retries})...")
            time.sleep(delay)
    raise ServoTimeoutError(f"Servo {servo._id} not responding after {retries} attempts.")

# -------------------------
# EASE TO ALL: Update multiple servos together
# -------------------------
def ease_to_all(servos, targets, duration, steps=50, easing_func=ease_in_out_cubic):
    """
    Smoothly move multiple servos concurrently by computing intermediate
    positions in a single loop.
    """
    # Read current positions safely for all servos
    starts = [safe_get_physical_angle(servo) for servo in servos]
    changes = [target - start for target, start in zip(targets, starts)]
    dt = duration / steps

    for step in range(1, steps + 1):
        t = step * dt
        for i, servo in enumerate(servos):
            intermediate = easing_func(t, starts[i], changes[i], duration)
            servo.move(intermediate)
        time.sleep(dt)

    # Ensure final positions are set
    for i, servo in enumerate(servos):
        servo.move(targets[i])

# -------------------------
# MOVEMENT FUNCTIONS USING ease_to_all
# -------------------------
def move_home_start(arr):
    ease_to_all([servo1, servo2, servo3, servo4, servo5], arr, duration=2, steps=50)

def move_pos1(arr):
    ease_to_all([servo2, servo3, servo1, servo4, servo5],
                [arr[1], arr[2], arr[0], arr[3], arr[4]],
                duration=2, steps=50)

def move_pos2(arr):
    ease_to_all([servo2, servo4, servo1, servo5, servo3],
                [arr[1], arr[3], arr[0], arr[4], arr[2]],
                duration=2, steps=50)

def move_home_end(arr):
    ease_to_all([servo2, servo4, servo1, servo5, servo3],
                [arr[1], arr[3], arr[0], arr[4], arr[2]],
                duration=2, steps=50)

def move_fwd(home, pos1, pos2):
    move_home_start(home)
    time.sleep(0.5)
    move_pos1(pos1)
    time.sleep(0.5)
    move_pos2(pos2)
    time.sleep(0.5)
    move_home_end(home)
    time.sleep(0.5)

# -------------------------
# RUN THE MOVEMENT SEQUENCE
# -------------------------
for _ in range(3):
    move_fwd(home, pos1, pos2)

print("Final angles:")
print("Servo 1:", servo1.get_physical_angle())
print("Servo 2:", servo2.get_physical_angle())
print("Servo 3:", servo3.get_physical_angle())
print("Servo 4:", servo4.get_physical_angle())
print("Servo 5:", servo5.get_physical_angle())
