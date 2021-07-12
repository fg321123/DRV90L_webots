from controller import Supervisor
import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import keyboard
import numpy as np
import threading
import time

np.set_printoptions(suppress=True)

def Initial_confi(initialize=False):
    Arm_position_controll(initialize=initialize, confi_now=False)


def CheckCollision():
    A = ['Collision' for WallName in List_Wall if WallName.getNumberOfContactPoints() > 0]
    if A != []:  # collision happend
        return (A[0])
    else:  # no collision
        return (' \n')


def Choose_mode(enable=True):
    print('==============================================\n')
    print('Choose Mode: (1)Arm Position Controll (2)Arm Joint Controll\n')
    print('===============================================\n')
    while enable and supervisor.step(timeStep) != -1:
        if keyboard.is_pressed('1'):
            print('You choose: (1)Arm Position Controll')
            enable = False
            return '1'
        elif keyboard.is_pressed('2'):
            print('You choose: (2)Arm Joint Controll')
            enable = False
            return '2'


def Choose_joint(enable=True):
    print('==============================================\n')
    print('Choose Mode: (1)Joint 1 (2)Joint 2 (3)Joint 3 (4)Joint 4 (5)Joint 5 (6)Joint 6\n')
    print('===============================================\n')
    while enable and supervisor.step(timeStep) != -1:
        if keyboard.is_pressed('1'):
            print('You choose: (1)Joint 1')
            enable = False
            return '1'
        elif keyboard.is_pressed('2'):
            print('You choose: (2)Joint 2')
            enable = False
            return '2'
        elif keyboard.is_pressed('3'):
            print('You choose: (3)Joint 3')
            enable = False
            return '3'
        elif keyboard.is_pressed('4'):
            print('You choose: (4)Joint 4')
            enable = False
            return '4'
        elif keyboard.is_pressed('5'):
            print('You choose: (5)Joint 5')
            enable = False
            return '5'
        elif keyboard.is_pressed('6'):
            print('You choose: (6)Joint 6')
            enable = False
            return '6'


def Arm_position_move(x_amm, y_amm, z_amm):
    # print('amm',x_amm, y_amm, z_amm)
    ikResults = armChain.inverse_kinematics([x_amm, y_amm, z_amm])
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])


def Arm_joint_move(angle, axis='all'):
    if axis == 'all':
        joint_move = [0.0] + angle
        joint_move = np.array(joint_move) * np.pi / 180.0
        print(joint_move[1:])
        for i in range(len(motors)):
            motors[i].setPosition(joint_move[i + 1])
    elif axis >= 1 and axis <= 6:
        joint_move = [0] + [m.getTargetPosition() for m in motors]
        # joint_move = [0] + [m.getPositionSensor().getValue() for m in motors]
        joint_move[axis] = angle * np.pi / 180
        print(joint_move[1:])
        for i in range(len(motors)):
            motors[i].setPosition(joint_move[i + 1])


def Gripper_controll():
    while supervisor.step(timeStep) != -1:
        time.sleep(0.1)
        force = touchsensor.getValues()
        print('%.2f, %.2f, %.2f' % (force[0], force[1], force[2]))


def Arm_position_controll(x_ac=0.25859184, y_ac=-0.53559201, z_ac=-0.0000015, step=0.02, initialize=False,
                          confi_now=True, esc=False):
    global x, y, z
    if initialize:
        Arm_position_move(x_ac, y_ac, z_ac)

        def joint_up():
            global y
            y = (y - step) if not esc else y

        def joint_down():
            global y
            y = (y + step) if not esc else y

        def joint_left():
            global z
            z = (z - step) if not esc else z

        def joint_right():
            global z
            z = (z + step) if not esc else z

        def joint_front():
            global x
            x = (x + step) if not esc else x

        def joint_back():
            global x
            x = (x - step) if not esc else x

        keyboard.add_hotkey('a', joint_left)
        keyboard.add_hotkey('w', joint_up)
        keyboard.add_hotkey('s', joint_front)
        keyboard.add_hotkey('ctrl+a', joint_right)
        keyboard.add_hotkey('ctrl+w', joint_down)
        keyboard.add_hotkey('ctrl+s', joint_back)
    else:
        if confi_now:
            x, y, z = Find_endposition()
            Arm_position_move(x, y, z)
            print('press esc to exit\n')
            while not esc and supervisor.step(timeStep) != -1:
                time.sleep(0.01)
                Arm_position_move(x, y, z)
                if keyboard.is_pressed('esc'):
                    print('exit arm controll\n')
                    esc = True
        else:
            while not esc and supervisor.step(timeStep) != -1:
                time.sleep(0.01)
                Arm_position_move(x_ac, y_ac, z_ac)
                x, y, z = x_ac, y_ac, z_ac
                esc = True
            # print('exit arrivei')


def Find_endposition():
    armPosition = arm.getPosition()
    # initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    initial_position = [0] + [m.getTargetPosition() for m in motors]
    position = armChain.forward_kinematics(initial_position)
    x_fe = position[0, 3]
    y_fe = position[1, 3]
    z_fe = position[2, 3]
    return position[0, 3], position[1, 3], position[2, 3]


supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())
wood1 = supervisor.getFromDef('cover')
arm = supervisor.getSelf()
touchsensor = supervisor.getDevice("touch sensor")
touchsensor.enable(timeStep)

print('input')
aa = input()
print(aa)
armChain = Chain.from_urdf_file('DRV90ASS_fix_axis.urdf')
motors = []
joints = []
for motorName in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
    motor = supervisor.getDevice(motorName)
    motor.setVelocity(4.5)
    if motorName == ('Joint5' or 'Joint4'):
        motor.setVelocity(9.5)
    motors.append(motor)
    position_sensor = motor.getPositionSensor()
    position_sensor.enable(timeStep)
    joints.append(motor.getTargetPosition())

RigWall = supervisor.getFromDef('RigWall_0_870_1160')  # Right Wall
LefWall = supervisor.getFromDef('LefWall_0_870_1160')  # Left  Wall
BacWall = supervisor.getFromDef('BacWall_1600_0_1160')  # Back  Wall
FroWall = supervisor.getFromDef('FroWall_1600_0_1160')  # Front Wall
TopWall = supervisor.getFromDef('TopWall_1600_870_0')  # Top  Wall
List_Wall = [RigWall, LefWall, BacWall, FroWall, TopWall]

tool_space = 0.1  # 0.3

thread_gripper_controll = threading.Thread(target=Gripper_controll)
thread_gripper_controll.start()

keyboard.add_hotkey('i', Initial_confi)
Initial_confi(initialize=True)
x, y, z = Find_endposition()

while supervisor.step(timeStep) != -1:

    option = Choose_mode()
    if option == '1':
        Arm_position_controll()
    elif option == '2':

        # Arm_joint_move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Arm_joint_move(0,5)
        for i in range(6):
            Arm_joint_move(0, i)

'''
while supervisor.step(timeStep) != -1:
    time.sleep(0.01)

    # print(touchsensor.getValue())

    targetPosition = wood1.getPosition()   
    armPosition = arm.getPosition()

    x =    targetPosition[0] - armPosition[0]
    y = - (targetPosition[1] - armPosition[1]) - tool_space
    z = - (targetPosition[2] - armPosition[2])

    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] #actual axis angle
    # print(np.round(initial_position,3))
    # ikResults = armChain.inverse_kinematics([x, y, z], max_iter = 4, initial_position=initial_position)
    # ikResults = armChain.inverse_kinematics([x, y, z], max_iter = 20)
    # ikResults = armChain.inverse_kinematics([x, y, z], initial_position=initial_position)
    ikResults = armChain.inverse_kinematics([x, y, z])
    position = armChain.forward_kinematics(ikResults)

    ikResults = np.array([0.0, 0.0, -30.0, 60.0, 0.0, 0.0, 0.0]) *3.14/180
    position = armChain.forward_kinematics(ikResults)
    print('fk,',position[0:3,3], 'xyz,[',x,y,z,']')
    # print('ik,',np.round(ikResults/np.pi*180,3))
    # print('diff,',position[0,3]-x, position[1,3]-y, position[2,3]-z)
    # print('diff angle,',np.round((initial_position - ikResults)/np.pi*180,3))

    print(CheckCollision())

    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i +1])
'''