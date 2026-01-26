from BallBalance import *
from ST3215_Actuator import Actuator, MotorMode
import numpy as np
import time

BB = BallBalance()

ID = [1, 2, 3]
indices = [0, 1, 2]
act = Actuator("/dev/ttyACM0", ID)
act.set_zero_pos([2040, 2048, 2048])
act.set_direction(np.full(3, -1))
act.set_mode([MotorMode.POS])


act.torque_enable()

act.target_pos_deg([0, 0, 0])
print("Zeroing...")
time.sleep(1)
print("Position start with: ", act.get_pos(indices))
motor_angle = np.rad2deg(BB.IK(0, 0, 80))

for alpha in np.arange(-0.05, .05, 0.01):
        motor_angle = np.rad2deg(BB.IK(alpha, 0.05-alpha, 80))
        print(motor_angle)
        # act.target_pos_deg([0, 0, 0])
        act.target_pos_deg(motor_angle)
        time.sleep(1/60)

act.target_pos_deg([0, 0, 0])
time.sleep(0.2)
act.torque_disable()
print("Position end with: ", act.get_pos(indices))
