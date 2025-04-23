import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import math
import datetime

import socket
from _thread import *

HOST = 'NETWORK HOST'
PORT = 8080
port = 'USB PORT'

client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

client_socket.connect((HOST, PORT))
print("server connected")

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address: https://emanual.robotis.com/docs/kr/dxl/ax/ax-12a/
ADDR_TORQUE_ENABLE      = 24              
ADDR_GOAL_POSITION      = 30
ADDR_MOVING_SPEED           = 32
ADDR_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
# IDchange
DXLs_ID                      = 6  
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
# BAUDRATE                    = 57600
DEVICENAME                  = port    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
##################??##############
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


disk_r=0.6
tool_r=0.115
s_calibration=0
s_initial=512+s_calibration

def getPosition(motor_ID):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_ID, ADDR_PRESENT_POSITION)
    return dxl_present_position

def SetGoalPosition(motor_ID, goalPosition):
    ##### motor goal position #####
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, motor_ID, ADDR_GOAL_POSITION, goalPosition)
    ######### motor speed #########
    packetHandler.write2ByteTxRx(portHandler, motor_ID, 32, 250)


'''
2nd 알고리즘

'''
steer_offset=0
def angle2goal(ang, motor_ID, sORf):
    global init_steer, init_yaw, steer_offset
    # IDchange
    if motor_ID==6:
        angle=ang-init_steer
    print('angle: ', angle, end='  |  motor: ')
    angle /= 57.29 # degree -> radian
    string_len = angle * (tool_r)
    motor_angle = string_len / disk_r
    motor_angle *= 57.29  # radian -> degree

    scale=1.5
    goal_position = s_initial + (motor_angle/0.293)*scale
    # IDchange
    if motor_ID==6:
        goal_position=goal_position
    print(goal_position)
    # IDchange
    if motor_ID==6: #steer
        if goal_position < s_initial-120:
            print("under limitaion of steering motor\n%f\n"%goal_position)
            return int(s_initial-110)
        if goal_position > s_initial+120:
            print("above limitaion of steering motor\n%f\n"%goal_position)
            return int(s_initial+110)
    if goal_position < 0:
        print("under minimal limitaion(0) of motor")
        return 0
    if goal_position > 1023:
        print("above maximal limitaion(1023) of motor")
        return 1023
    return int(goal_position)

import numpy as np

def mov_avg_filter(x,x_data):
    n=5
    for i in range(n-1):
        x_data[i]=x_data[i+1]
    x_data[n-1]=x
    x_avg = np.mean(x_data)
    return x_avg,x_data

def init():
    # Open port
    try: 
        portHandler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque: Steering
    dxl_comm_result_s, dxl_error_s = packetHandler.write1ByteTxRx(portHandler, DXLs_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result_s != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result_s))
        print("Press any key to terminate... = client_socket.recv(1024)")
        getch()
        quit()
    elif dxl_error_s != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error_s))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("Steering Dynamixel has been successfully connected")
    # Set Dynamixel to Default
    SetGoalPosition(DXLs_ID,s_initial)



# init
init() # motor initialize

client_socket.settimeout(2.0)

k=0


try:
    while True :
        try:
            data = client_socket.recv(1024)

            convdata=repr(data.decode('utf-8'))
            
            mydata2=convdata.replace(":",",")
            mydata2=mydata2.split(',')
            if len(mydata2) != 14:
                continue
            else: 
                mydata=mydata2

            Phantom_X=0
            Phantom_Y=0
            Phantom_Z=0
            Yaw=0
            Roll=0
            button,clutch=0,0
            

            if (len(mydata)<14):
                pass
            
            elif len(mydata)==14 and mydata[0]=="'999":
                if k==0:
                    print('init successed')

                    if int(mydata[12]) == 1:
                        sORf=1
                        interface='Stylus'
                        divider_s=1
                    elif int(mydata[12]) == 2:
                        sORf=2
                        interface='Finger'
                        divider_s=1

                    init_steer=0
                    steer_list=[init_steer,init_steer,init_steer,init_steer,init_steer]
                    now = datetime.datetime.now()
                    file_name = str("DY_%s_%s_%s.txt" % (interface, str(now)[:-16], str(now)[-15:-7].replace(':','-')))
                    f=open('FILE_PATH%s'%file_name,'w+')

                    k+=1

                if interface=='Stylus':
                    steer,steer_list=mov_avg_filter(float(mydata[9])/divider_s,steer_list)
                elif interface=='Finger':
                    steer,steer_list=mov_avg_filter(float(mydata[1])/divider_s,steer_list)
                goal_angle=angle2goal(steer,DXLs_ID, sORf)
                SetGoalPosition(DXLs_ID, goal_angle)

                ######## steering motor position ######
                now = datetime.datetime.now()
                f.write(str(now)+'| %f %f %f | %f %f %f | %d | %d | %d'
                        %(Phantom_X,Phantom_Y,Phantom_Z,steer,Yaw,Roll,goal_angle,button,clutch))
                f.write('\n')


                SetGoalPosition(6, angle2goal(steer, getPosition(6),6, sORf))
                print(angle2goal(steer, getPosition(1),1, sORf))

        except socket.timeout:
            print("stop")
            k=0
            init()
except KeyboardInterrupt:
    SetGoalPosition(DXLs_ID, s_initial)
