from pylx16a.lx16a import LX16A, ServoTimeoutError
import time

# Initialize the serial communication with a short delay between bytes.
LX16A.initialize("COM4", 0.1)

# Create a list of servo instances for servo IDs 1 through 5.
servo_ids = [1, 2, 3, 4, 5]
servos = []

for sid in servo_ids:
    try:
        servo = LX16A(sid)
        servos.append(servo)
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} not responding. Exiting...")
        quit()

# Disable the torque on all servos to allow manual movement.
for s in servos:
    try:
        s.disable_torque()
    except ServoTimeoutError:
        print(f"Failed to disable torque on servo {s._id}.")
    except Exception as err:
        print(f"Error on servo {s._id} while disabling torque: {err}")

print("Torque disabled for all servos. You can now move the robot freely.")
print("Starting continuous angle readback...")

# Continuously read and print the physical angles from each servo.
while True:
    for s in servos:
        try:
            angle = s.get_physical_angle()
            print(f"Servo {s._id} angle: {angle:.1f}°", end="\t")
        except ServoTimeoutError:
            print(f"Servo {s._id} not responding.", end="\t")
    print()  # Newline after each full set of servo readings.
    time.sleep(0.1)
