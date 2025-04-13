from pylx16a.lx16a import LX16A, ServoTimeoutError
import time

LX16A.initialize("COM4", 0.1)

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


home = [119,100,130,114,87]
pos1 = [86,133,164,83,82]
pos2 = [121,62,173,155,59]

def move_home_start(arr):
    servo1.move(arr[0])
    servo2.move(arr[1])
    servo3.move(arr[2])
    servo4.move(arr[3])
    servo5.move(arr[4])

def move_pos1(arr):
    servo2.move(arr[1])
    servo3.move(arr[2])
    servo1.move(arr[0])
    servo4.move(arr[3])
    servo5.move(arr[4])

def move_pos2(arr):
    servo2.move(arr[1])
    servo4.move(arr[3])
    servo1.move(arr[0])
    servo5.move(arr[4])
    servo3.move(arr[2])

def move_home_end(arr):
    servo2.move(arr[1])
    servo4.move(arr[3])
    servo1.move(arr[0])
    servo5.move(arr[4])
    servo3.move(arr[2])
    
def move_fwd(home,pos1,pos2):
    move_home_start(home)
    time.sleep(0.5)
    move_pos1(pos1)
    time.sleep(0.5)
    move_pos2(pos2)
    time.sleep(0.5)
    move_home_end(home)
    time.sleep(0.5)

for _ in range(4):
    move_fwd(home,pos1,pos2)

theta5_act = servo5.get_physical_angle()
print(theta5_act)
theta1_act = servo1.get_physical_angle()
print(theta1_act)
#servo1.move(theta1_act - 20)
theta2_act = servo2.get_physical_angle()
print(theta2_act)
theta3_act = servo3.get_physical_angle()
print(theta3_act)
theta4_act = servo4.get_physical_angle()
print(theta4_act)

'''
move_robot(pos2)
time.sleep(2)
move_robot(home)
time.sleep(2)
'''
'''

theta1_cmd = servo1.get_commanded_angle()
theta2_cmd = servo2.get_commanded_angle()
theta3_cmd = servo3.get_commanded_angle()
theta4_cmd = servo4.get_commanded_angle()
theta5_cmd = servo5.get_commanded_angle()
'''


#, theta2_act, theta3_act, theta4_act, theta5_act)