# import ST3215_Actuator as STACT
from ST3215_Actuator import Actuator, MotorMode
import time
import numpy as np

# act = STACT.Actuator("COM7", [1,])
ID = [1, 2, 3]
indices = [0, 1, 2]
act = Actuator("/dev/ttyACM0", ID)
act.set_zero_pos([2040, 2048, 2048])
act.set_direction(np.full(3, -1))
act.set_mode([MotorMode.POS])

act.torque_enable()
time.sleep(1)
# act.target_pos_step([-100,])
act.target_pos_deg([0, 0, 0])

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())


time.sleep(1)
# act.target_pos_step([-100,])
act.target_pos_deg([5, 5, 5])

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())

time.sleep(1)
# act.target_pos_step([100,])
act.target_pos_deg([-5, -5, -5])

time.sleep(3)
print("Position angle: ", act.get_pos(indices))
print("Position step: ", act.get_raw_step())
time.sleep(1)

act.target_pos_deg([0, 0, 0])

act.torque_disable()

