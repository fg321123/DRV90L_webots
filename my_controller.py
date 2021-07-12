from controller import Supervisor
import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import keyboard
import numpy as np
import threading
import time
np.set_printoptions(suppress=True)

def Initial_confi(initialize = False):
    Arm_joystick_controll(initialize = initialize, confi_now = False)

def CheckCollision():
    A = ['Collision' for WallName in List_Wall if WallName.getNumberOfContactPoints() > 0]
    if A != []:# collision happend
        return(A[0])
    else:     # no collision
        return(' \n')

def choose_thread(ans):
    global pressed
    try:
        pressed = float(input())
    except:
        pressed = 1001
        print('enter wrong!')
    while not (pressed in ans):
        try:
            pressed = float(input())
        except:
            pressed = 1001
            print('enter wrong!')

def Choose_mode():
    global pressed
    pressed = 1001
    print('\n'*80)
    print('===========================================================')
    print('Choose Mode: (1)Arm Joystick Controll (2)Arm Position Controll (3)Arm Joint Controll')
    print('===========================================================')
    print('enter mode #number')
    choose_mode_thread = threading.Thread(target=choose_thread, args = ([1,2,3],))
    choose_mode_thread.start()
    while supervisor.step(timeStep) != -1:
        if pressed == 1:
            print('You choose: (1)Arm Joystick Controll\n')
            enable = False
            return '1'
        if pressed == 2:
            print('You choose: (2)Arm Position Controll\n')
            enable = False
            return '2'
        if pressed == 3:
            print('You choose: (3)Arm Joint Controll\n')
            enable = False
            return '3'

        # keyboard.hook(keyboard_hook)
        # if str(pressed) == 'KeyboardEvent(1 up)':
        #     print('You choose: (1)Arm Position Controll\n')
        #     enable = False
        #     pressed = 'KeyboardEvent(-1 up)'
        #     return '1'
        # if str(pressed) == 'KeyboardEvent(2 up)':
        #     print('You choose: (2)Arm Joint Controll\n')
        #     enable = False
        #     pressed = 'KeyboardEvent(-1 up)'
        #     return '2'
        # pressed = -1

        # if keyboard.is_pressed('1'):
        #     print('You choose: (1)Arm Position Controll\n')
        #     enable = False
        #     return '1'
        # if keyboard.is_pressed('2'):
        #     print('You choose: (2)Arm Joint Controll\n')
        #     enable = False
        #     return '2'
            
def Choose_joint(esc = False):
    global pressed
    pressed = 1001
    print('\n'*80)
    print('=============================================================================================')
    print('Choose Joint: (0)All Joints (1)Joint 1 (2)Joint 2 (3)Joint 3 (4)Joint 4 (5)Joint 5 (6)Joint 6')
    print('=============================================================================================')
    print('==press "tab" to exit==')
    print('enter joint #number')
    choose_mode_thread = threading.Thread(target=choose_thread, args = ([0, 1,2,3,4,5,6],))
    choose_mode_thread.start()
    while (not esc) and supervisor.step(timeStep) != -1:
        if pressed <= 6 and pressed >= 0:
            Choose_angle(pressed)
            break
        if keyboard.is_pressed('tab'):
            print('exit choose joint')
            esc = True

        # keyboard.hook(keyboard_hook)
        # pressed_joint = int(re.split(r'(KeyboardEvent\()|(\s)|(\))', str(pressed))[4])
        # pressed_release = re.split(r'(KeyboardEvent\()|(\s)|(\))', str(pressed))[8]
        # if pressed_joint <= 7 and pressed_joint >= 0 and pressed_release == 'up':
        #     print(pressed_joint)
        #     Choose_angle(pressed_joint)
        #     break

def Choose_angle(choosed_joint, esc = False):
        global pressed
        pressed = 1001
        print('\n' * 80)
        choosed_joint = int(choosed_joint)
        if choosed_joint != 0:
            min_angle = motors[choosed_joint-1].getMinPosition() * 180 / np.pi
            max_angle = motors[choosed_joint-1].getMaxPosition() * 180 / np.pi
            print("Joint %d upper limit: %3.1f lower limit :%4.1f" % (choosed_joint, min_angle, max_angle))
            print('Please enter the rotation angle of axis',choosed_joint,'in degrees')
            choose_mode_thread = threading.Thread(target=choose_thread, args=(np.linspace(int(min_angle),int(max_angle),int((max_angle-min_angle)*10+1)),))
            choose_mode_thread.start()
            while (not esc) and supervisor.step(timeStep) != -1:
                if min_angle <= pressed <= max_angle:
                    Arm_joint_move(pressed, choosed_joint)
                    break
        else:
            for motorCount in range(6):
                min_angle = motors[motorCount].getMinPosition() * 180 / np.pi
                max_angle = motors[motorCount].getMaxPosition() * 180 / np.pi
                print("Joint %d upper limit: %3.1f lower limit :%4.1f" % (motorCount,  min_angle, max_angle))
            print('Please enter the rotation angle of six-axis in degrees')
            print('example: 20, -20 ,50, 150, -120, 180')
            all_axis = input()
            print(all_axis)
            all_axis_list = list(map(float, all_axis.split(',')))
            print(all_axis_list)
            # print('Input angle', all_axis_list)
            Arm_joint_move(all_axis_list)

def Arm_position_move(x_amm, y_amm, z_amm, end_down = True):
    # print('amm',x_amm, y_amm, z_amm)
    iter_move = 0
    if end_down:
        ikResults = armChain_down.inverse_kinematics([x_amm, y_amm, z_amm])
        ikResults = np.append(ikResults, - np.pi/2 + (ikResults[2]+ikResults[3]))
        ikResults = np.append(ikResults, 0)
        for i in range(len(motors)):
            motors[i].setPosition(ikResults[i +1])
        while iter_move < 100:
            x_f, y_f, z_f = Find_endposition(end_down = end_down)
            diff = round((x_f - x_amm) +  (y_f - y_amm) + (z_f - z_amm),6)
            time.sleep(0.01)
            iter_move += 1
            if abs(diff) < 10e-6 :
                return 0
        return 1

    else:
        ikResults = armChain.inverse_kinematics([x_amm, y_amm, z_amm])
        for i in range(len(motors)):
            motors[i].setPosition(ikResults[i +1])

def Arm_joint_move(angle, axis = 'all'):
    if axis == 'all':
        joint_move = [0.0] + angle
        joint_move = np.array(joint_move) * np.pi / 180.0
        for i in range(len(motors)):
            motors[i].setPosition(joint_move[i + 1])
    elif axis >= 1 and axis <= 6:        
        joint_move = [0] + [m.getTargetPosition() for m in motors] 
        # joint_move = [0] + [m.getPositionSensor().getValue() for m in motors] 
        joint_move[axis] = angle * np.pi /180
        for i in range(len(motors)):
            motors[i].setPosition(joint_move[i +1])

def Gripper_controll():
    def Grasp():
        for i in range(3):
            gripper_motors[i].setPosition(0.85)       
    def Release():
        for i in range(3):
            gripper_motors[i].setPosition(gripper_motors[i].getMinPosition())
    print('gripper controll start\n')
    keyboard.add_hotkey('g',Grasp)
    keyboard.add_hotkey('r',Release)
    keyboard.wait()    

def Arm_joystick_controll(x_ac = 0.25859184, y_ac = -0.7, z_ac = -0.0000015, step = 0.02, initialize = False, confi_now = True, esc = False):
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
            print('\n' * 80)
            print('==press "tab" to exit==\n')
            print('Control instruction:')
            print('w:up      ctrl+w:down')
            print('s:forward ctrl+s:backward')
            print('a:lef     ctrl+a:right')
            print('i:initial configuraion')
            print('\nGripper instruction:')
            print('g:grasp   r:release')
            while not esc and supervisor.step(timeStep) != -1:
                time.sleep(0.01)
                err_arrive = Arm_position_move(x, y, z)
                if err_arrive != 0:
                    print('not arrive position')
                if keyboard.is_pressed('tab'):
                    print('exit arm controll\n')
                    esc = True
        else:
            while not esc and supervisor.step(timeStep) != -1:
                time.sleep(0.01)
                Arm_position_move(x_ac, y_ac, z_ac)
                x, y, z = x_ac, y_ac, z_ac
                esc = True
            # print('exit arrive')
    
def Find_endposition(end_down = True):
    # initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    initial_position = [0] + [m.getTargetPosition() for m in motors]
    if end_down:
        position = armChain_down.forward_kinematics(initial_position[:5])
        x_fe =  position[0, 3]
        y_fe =  position[1, 3]
        z_fe =  position[2, 3]
        return x_fe, y_fe, z_fe
    else:
        position = armChain.forward_kinematics(initial_position)
        x_fe =  position[0, 3]
        y_fe =  position[1, 3]
        z_fe =  position[2, 3]
        return x_fe, y_fe, z_fe
        


supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())
wood1 = supervisor.getFromDef('cover')
arm = supervisor.getSelf()
# touchsensor = supervisor.getDevice("touch sensor")
# touchsensor.enable(timeStep)

armChain_down = Chain.from_urdf_file('DRV90ASS_3axis.urdf')
armChain = Chain.from_urdf_file('DRV90ASS_fix_axis.urdf')
motors = []   
joints = []
for motorName in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
    motor = supervisor.getDevice(motorName)
    # motor.setVelocity(4.5)
    # if motorName == ('Joint5' or 'Joint4'):
    #     motor.setVelocity(9.5)
    motors.append(motor)
    position_sensor = motor.getPositionSensor()
    position_sensor.enable(timeStep)
    joints.append(motor.getTargetPosition())
    
gripper_motors = []
for motorName in ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]:
    motor = supervisor.getDevice(motorName)
    gripper_motors.append(motor)
    position_sensor = motor.getPositionSensor()
    position_sensor.enable(timeStep)
    # joints.append(motor.getTargetPosition())

RigWall = supervisor.getFromDef('RigWall_0_870_1160') # Right Wall
LefWall = supervisor.getFromDef('LefWall_0_870_1160') # Left  Wall
BacWall = supervisor.getFromDef('BacWall_1600_0_1160')# Back  Wall
FroWall = supervisor.getFromDef('FroWall_1600_0_1160')# Front Wall
TopWall = supervisor.getFromDef('TopWall_1600_870_0') #  Top  Wall
List_Wall = [RigWall, LefWall, BacWall, FroWall, TopWall]

tool_space = 0.1 #0.3

thread_gripper_controll = threading.Thread(target = Gripper_controll)
thread_gripper_controll.start()

keyboard.add_hotkey('i', Initial_confi)
Initial_confi(initialize = True)
x, y, z = Find_endposition()

# Arm_position_move(0.25859184, -0.7, -1.5e-06)

while supervisor.step(timeStep) != -1:

    option = Choose_mode()
    if option == '1':
        Arm_joystick_controll()
    elif option == '2':
        Arm_position_move()
    elif option == '3':
        Choose_joint()
        # Arm_joint_move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Arm_joint_move(0,5)
        # for i in range(6):
        #     Arm_joint_move(0,i)


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