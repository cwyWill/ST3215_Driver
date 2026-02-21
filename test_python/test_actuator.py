# import ST3215_Actuator as STACT
from ST3215_Actuator import Actuator, MotorMode
import time
import numpy as np

# act = STACT.Actuator("COM7", [1,])
ID = [11, 12, 13, 14, 15, 16,]
indices = [0,]
act = Actuator("/dev/ttyACM0", ID)

act.torque_disable()


print(act.get_raw_step())

