# import ST3215_Actuator as STACT
from ST3215_Actuator import Actuator, MotorMode
import time
import numpy as np

# act = STACT.Actuator("COM7", [1,])
ID = [12,]
indices = [0,]
act = Actuator("/dev/ttyACM0", ID)

# act.set_direction(np.full(1, 1))


act.set_mode([MotorMode.POS])

act.torque_enable()
print("Torque enabled")
act.target_pos_step([0,])

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())

act.target_pos_step([2048, ])

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())



act.torque_disable()

act.target_pos_step([3072, ])
act.torque_disable()

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())