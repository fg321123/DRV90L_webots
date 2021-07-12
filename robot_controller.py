from ikpy.chain import Chain
from controller import Supervisor
import numpy as np
import time
# from controller import Supervisor
#
# supervisor = Supervisor()
# timeStep = int(4 * supervisor.getBasicTimeStep())

class DRV90_Robot:
    def __init__(self, robot_urdf_name, end_down = True):
        self.motors = []
        self.armChain_down = Chain.from_urdf_file(robot_urdf_name)
        self.end_down = end_down
        self.supervisor = Supervisor()
        self.timestep = int(4 * self.supervisor.getBasicTimeStep())

        for jointName in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            self.motor = self.supervisor.getDevice(jointName)
            self.motors.append(self.motor)
            self.position_sensor = self.motor.getPositionSensor()
            self.position_sensor.enable(self.timestep)

    def find_endposition(self):
        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors]
        # initial_position = [0] + [m.getTargetPosition() for m in self.motors]
        if self.end_down:
            position = self.armChain_down.forward_kinematics(initial_position[:5])
            return position[0, 3], position[1, 3], position[2, 3]

    def position_move(self, x_target, y_target, z_target):
        iter_move = 0
        if self.end_down :
            ikResults = self.armChain_down.inverse_kinematics([x_target, y_target, z_target])
            ikResults = np.append(ikResults, - np.pi / 2 + (ikResults[2] + ikResults[3]))
            ikResults = np.append(ikResults, 0)
            for i in range(len(self.motors)):
                self.motors[i].setPosition(ikResults[i + 1])
            while iter_move < 100 and self.supervisor.step(self.timestep) != -1:
                x_f, y_f, z_f = self.find_endposition()
                diff = round(abs(x_f - x_target) + abs(y_f - y_target) + abs(z_f - z_target), 6)
                iter_move += 1
                if abs(diff) < 10e-6:
                    print(iter_move)
                    return 0
            return 1
        else:
            pass
