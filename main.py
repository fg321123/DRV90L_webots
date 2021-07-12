from controller import Supervisor
import numpy as np
from robot_controller import DRV90_Robot
from tensorboardX import SummaryWriter
# np.set_printoptions(suppress=True)

savePath="runs/PPO_test"
writer = SummaryWriter(savePath)

# supervisor = Supervisor()
# timeStep = int(4 * supervisor.getBasicTimeStep())
armChain_down = DRV90_Robot("DRV90ASS_3axis.urdf")

err_code = armChain_down.position_move(0.25859184, -0.7, -0.0000015)
print(err_code)
# while supervisor.step(timeStep) != -1:
#     err_code = armChain_down.position_move(0.25859184, -0.7,-0.0000015)
#     print(err_code)
