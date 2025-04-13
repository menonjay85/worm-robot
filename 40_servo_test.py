from pylx16a.lx16a import *
import time, math

# Initialize the bus
LX16A.initialize("COM3", 0.1)

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
