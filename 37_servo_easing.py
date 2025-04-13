from pylx16a.lx16a import *
import time, math

# Linear easing function (for reference)
def ease_linear(t, b, c, d):
    return c * t / d + b

def ease_in_out_sine(t, b, c, d):
    """
    Easing function: ease-in-out sine.
    
    t: current time
    b: starting value
    c: change in value (target - start)
    d: duration
    """
    return -c/2 * (math.cos(math.pi * t / d) - 1) + b

# Cubic ease-in-out function
def ease_in_out_cubic(t, b, c, d):
    t /= d/2
    if t < 1:
        return c/2 * t*t*t + b
    t -= 2
    return c/2 * (t*t*t + 2) + b

def ease_to(servo, target, duration, steps=50, easing_func=ease_in_out_sine):
    """
    Smoothly move the servo from its current angle to the target angle.
    
    Parameters:
      servo: the servo object to move
      target: the target physical angle (native range: 0-240°)
      duration: total time for the movement (in seconds)
      steps: number of intermediate steps
      easing_func: the easing function to use
    """
    start = servo.get_physical_angle()
    change = target - start
    dt = duration / steps
    for step in range(1, steps + 1):
        t = step * dt
        current_angle = easing_func(t, start, change, duration)
        servo.move(current_angle)
        time.sleep(dt)
    # Ensure the target is exactly reached at the end.
    servo.move(target)

# Initialize the bus
LX16A.initialize("COM4", 0.1)

# Scan for connected servos
connected_servos = []
for servo_id in range(0, 254):
    try:
        servo = LX16A(servo_id)
        servo.get_physical_angle()
        connected_servos.append(servo_id)
    except:
        pass
print("Connected Servo IDs:", connected_servos)

# Print initial physical angles
print("Initial angles:")
print(LX16A(1).get_physical_angle())
print(LX16A(2).get_physical_angle())
print(LX16A(3).get_physical_angle())
print(LX16A(4).get_physical_angle())
print(LX16A(5).get_physical_angle())

# Define the measured home positions for each servo (native angles)
servo1_home = 119.52 
servo2_home = 99.6 
servo3_home = 131.52 
servo4_home = 114.24 
servo5_home = 87.84 

# Move all servos to their home positions
LX16A(1).move(servo1_home)
LX16A(2).move(servo2_home)
LX16A(3).move(servo3_home)
LX16A(4).move(servo4_home)
LX16A(5).move(servo5_home)

time.sleep(1)  # Allow time for servos to reach home

# # In the centered system, home (0°) is defined by the measured home positions.
# # To move Servo 2 to a target relative angle, set target_angle_centered accordingly.
# # For example, for -45° relative:
# target_angle_centered = 0   # desired relative angle in centered system
# physical_target_servo2 = servo2_home + target_angle_centered

# # Create Servo 2 object and ease it to the target smoothly.
# servo2 = LX16A(2)
# print("Easing Servo 2 to {}° relative to home (Physical target: {:.2f}°)".format(target_angle_centered, physical_target_servo2))
# # You can choose your preferred easing function here; we'll use ease_in_out_cubic in this example.
# ease_to(servo2, physical_target_servo2, duration=2, steps=50, easing_func=ease_in_out_cubic)

# time.sleep(1)  # Wait a moment for movement to complete

# # Verify final angle by converting physical angle to the centered system
# final_angle = servo2.get_physical_angle()
# centered_final = final_angle - servo2_home
# print("Servo 2 now reads {:.2f}° in the centered system (Physical: {:.2f}°)".format(centered_final, final_angle))
