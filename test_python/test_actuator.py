# import ST3215_Actuator as STACT
from ST3215_Actuator import Actuator, MotorMode
import time
import numpy as np

# act = STACT.Actuator("COM7", [1,])
ID = [4, 5]
indices = [0, 1]
act = Actuator("/dev/ttyACM0", ID)

act.torque_disable()


print(act.get_raw_step())

