########### Rolling 절댓값 ###########
import os
import psutil
import sys
from _thread import *
from ctypes import *
sys.path.append('./.local/lib/python3.6/site-packages')
sys.path.append('UR_RTDE PATH')
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import json
import time
from math import *
import numpy as np
from threading import *
import datetime

import select
import tty
import struct

import select

###################################################
import socket
HOST = 'NETWORK HOST'
PORT = 8080
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
exit_flag=False
data=[0,0,0,0,0,0,0,0,0,0,0,0,0]
lock=Lock()

# Parameters
vel = 1
acc = 1
rtde_frequency = 100.0
dt = 1.0/rtde_frequency

flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
robot_ip = "ROBOT IP"

lookahead_time = 0.15
gain = 300

# ur_rtde realtime priorities
rt_receive_priority = 90
rt_control_priority = 85

rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(robot_ip, rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP, ur_cap_port, rt_control_priority)

# Set application real-time priority
os_used = sys.platform
process = psutil.Process(os.getpid())
if os_used == "win32":  # Windows (either 32-bit or 64-bit)
    process.nice(psutil.REALTIME_PRIORITY_CLASS)
elif os_used == "linux":  # linux
    rt_app_priority = 80
    param = os.sched_param(rt_app_priority)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
    except OSError:
        print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
    else:
        print("Process real-time priority set to: %u" % rt_app_priority)

time_counter = 0.0

from scipy import ndimage
def q_filter(q_list, new_q,size):
    q_list=list(q_list)
    q_list.append(new_q)
    new_q_list=q_list
    filtered_q_list=ndimage.median_filter(np.array(new_q_list), size=size)
    filtered_q_list=np.array(filtered_q_list)
    return new_q_list[1:], filtered_q_list[-1]

a_val = [0, -0.4250, -0.3922, 0, 0, 0, 0]
len_6_rigid=(330+20)*0.001
len_extractor=20
len_flex=(50+5+len_extractor)*0.001
d_val = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996, len_6_rigid+len_flex]
alpha_val = [pi/2, 0, 0, pi/2, -pi/2, 0, 0]
R_URtoPh=np.array([[-1,0,0],[0,-1,0],[0,0,1]])

######## Gain #########
Kt=np.diag([7,7,7])
KRCM=np.diag([100,100,100])
######################################################
nt3=np.zeros((3,3))
Gain1=np.concatenate((Kt,nt3),axis=1)
Gain2=np.concatenate((nt3,KRCM),axis=1)
Gain=np.concatenate((Gain1,Gain2),axis=0)

def getFK(th, tool):
    
    p06 = np.array([-0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]) + 0.0996*sin(th[0])*cos(th[4]) + 0.1333*sin(th[0]) + 0.3922*sin(th[1])*sin(th[2])*cos(th[0]) - 0.3922*cos(th[0])*cos(th[1])*cos(th[2]) - 0.425*cos(th[0])*cos(th[1]),
                    -0.0996*((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[0])*sin(th[1])*sin(th[2]) - 0.3922*sin(th[0])*cos(th[1])*cos(th[2]) - 0.425*sin(th[0])*cos(th[1]) - 0.0996*cos(th[0])*cos(th[4]) - 0.1333*cos(th[0]),
                    -0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*sin(th[3]) - 0.3922*sin(th[1])*cos(th[2]) - 0.425*sin(th[1]) - 0.3922*sin(th[2])*cos(th[1]) + 0.1625])
    p07 = np.array([tool*(-((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + sin(th[0])*cos(th[4])) - 0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]) + 0.0996*sin(th[0])*cos(th[4]) + 0.1333*sin(th[0]) + 0.3922*sin(th[1])*sin(th[2])*cos(th[0]) - 0.3922*cos(th[0])*cos(th[1])*cos(th[2]) - 0.425*cos(th[0])*cos(th[1]),
                    tool*(-((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) - cos(th[0])*cos(th[4])) - 0.0996*((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[0])*sin(th[1])*sin(th[2]) - 0.3922*sin(th[0])*cos(th[1])*cos(th[2]) - 0.425*sin(th[0])*cos(th[1]) - 0.0996*cos(th[0])*cos(th[4]) - 0.1333*cos(th[0]),
                    -tool*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*sin(th[3]) - 0.3922*sin(th[1])*cos(th[2]) - 0.425*sin(th[1]) - 0.3922*sin(th[2])*cos(th[1]) + 0.1625])
    return p06,p07

def getBendPose(theta_val, steer):
    steer_rad=steer*pi/180
    T_0_6=np.array([[(((-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*sin(theta_val[3]))*cos(theta_val[4]) + sin(theta_val[0])*sin(theta_val[4]))*cos(theta_val[5]) + (-(-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*cos(theta_val[3]))*sin(theta_val[5]),
        -(((-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*sin(theta_val[3]))*cos(theta_val[4]) + sin(theta_val[0])*sin(theta_val[4]))*sin(theta_val[5]) + (-(-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*cos(theta_val[3]))*cos(theta_val[5]),
        -((-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*sin(theta_val[3]))*sin(theta_val[4]) + sin(theta_val[0])*cos(theta_val[4]),
        d_val[5]*(-((-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*sin(theta_val[3]))*sin(theta_val[4]) + sin(theta_val[0])*cos(theta_val[4])) + 0.0997*(-sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) + cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) - 0.0997*(-sin(theta_val[1])*cos(theta_val[0])*cos(theta_val[2]) - sin(theta_val[2])*cos(theta_val[0])*cos(theta_val[1]))*cos(theta_val[3]) + 0.1333*sin(theta_val[0]) + 0.3922*sin(theta_val[1])*sin(theta_val[2])*cos(theta_val[0]) - 0.3922*cos(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]) - 0.425*cos(theta_val[0])*cos(theta_val[1])],
       [(((-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*cos(theta_val[4]) - sin(theta_val[4])*cos(theta_val[0]))*cos(theta_val[5]) + (-(-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*sin(theta_val[5]),
        -(((-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*cos(theta_val[4]) - sin(theta_val[4])*cos(theta_val[0]))*sin(theta_val[5]) + (-(-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*cos(theta_val[5]),
        -((-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*sin(theta_val[4]) - cos(theta_val[0])*cos(theta_val[4]),
        d_val[5]*(-((-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + (-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*sin(theta_val[4]) - cos(theta_val[0])*cos(theta_val[4])) + 0.0997*(-sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) + sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) - 0.0997*(-sin(theta_val[0])*sin(theta_val[1])*cos(theta_val[2]) - sin(theta_val[0])*sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]) + 0.3922*sin(theta_val[0])*sin(theta_val[1])*sin(theta_val[2]) - 0.3922*sin(theta_val[0])*cos(theta_val[1])*cos(theta_val[2]) - 0.425*sin(theta_val[0])*cos(theta_val[1]) - 0.1333*cos(theta_val[0])],
       [((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*cos(theta_val[4])*cos(theta_val[5]) + ((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) - (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*sin(theta_val[5]),
        -((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*sin(theta_val[5])*cos(theta_val[4]) + ((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) - (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]))*cos(theta_val[5]),
        -((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*sin(theta_val[4]),
        -d_val[5]*((-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*sin(theta_val[3]) + (sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*cos(theta_val[3]))*sin(theta_val[4]) - 0.0997*(-sin(theta_val[1])*sin(theta_val[2]) + cos(theta_val[1])*cos(theta_val[2]))*cos(theta_val[3]) + 0.0997*(sin(theta_val[1])*cos(theta_val[2]) + sin(theta_val[2])*cos(theta_val[1]))*sin(theta_val[3]) - 0.3922*sin(theta_val[1])*cos(theta_val[2]) - 0.425*sin(theta_val[1]) - 0.3922*sin(theta_val[2])*cos(theta_val[1]) + 0.1625],
       [0, 0, 0, 1]])
    T_6_rigid=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,len_6_rigid],[0,0,0,1]])
    T_rigid_flex = np.array(([cos(steer_rad), 0, sin(steer_rad), 0.050*(1-cos(steer_rad))/steer_rad],
                            [0, 1, 0, 0],
                            [-sin(steer_rad), 0, cos(steer_rad), 0.050*sin(steer_rad)/steer_rad],
                            [0,0,0,1]))
    T= T_0_6@T_6_rigid@T_rigid_flex
    # get p0_tip from calculated Ts
    p0_tip=T[:3,-1]
    return p0_tip

def getJacobian(th):
    tool=d_val[-1]
    jaMat06 = np.array([[-0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (sin(th[0])*sin(th[1])*cos(th[2]) + sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(sin(th[0])*sin(th[1])*cos(th[2]) + sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]) - 0.3922*sin(th[0])*sin(th[1])*sin(th[2]) + 0.3922*sin(th[0])*cos(th[1])*cos(th[2]) + 0.425*sin(th[0])*cos(th[1]) + 0.0996*cos(th[0])*cos(th[4]) + 0.1333*cos(th[0]),
                         -0.0996*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[1])*cos(th[0])*cos(th[2]) + 0.425*sin(th[1])*cos(th[0]) + 0.3922*sin(th[2])*cos(th[0])*cos(th[1]),
                         -0.0996*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[1])*cos(th[0])*cos(th[2]) + 0.3922*sin(th[2])*cos(th[0])*cos(th[1]),
                         -0.0996*(-(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]),
                         -0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*cos(th[4]) - 0.0996*sin(th[0])*sin(th[4]),
                         0, 0.0],
                         [-0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]) + 0.0996*sin(th[0])*cos(th[4]) + 0.1333*sin(th[0]) + 0.3922*sin(th[1])*sin(th[2])*cos(th[0]) - 0.3922*cos(th[0])*cos(th[1])*cos(th[2]) - 0.425*cos(th[0])*cos(th[1]),
                           -0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[0])*sin(th[1])*cos(th[2]) + 0.425*sin(th[0])*sin(th[1]) + 0.3922*sin(th[0])*sin(th[2])*cos(th[1]),
                          -0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[0])*sin(th[1])*cos(th[2]) + 0.3922*sin(th[0])*sin(th[2])*cos(th[1]),
                          -0.0996*(-(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) + 0.0997*(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]),
                          -0.0996*((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*cos(th[4]) + 0.0996*sin(th[4])*cos(th[0]),
                          0, 0.0],
                          [0,
                           -0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[1])*sin(th[2]) - 0.3922*cos(th[1])*cos(th[2]) - 0.425*cos(th[1]),
                           -0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[1])*sin(th[2]) - 0.3922*cos(th[1])*cos(th[2]),
                           -0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) - (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + 0.0997*(sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]),
                           -0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*cos(th[4]),
                           0, 0.0]])
    
    jaMat07 = np.array([[tool*(-((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (sin(th[0])*sin(th[1])*cos(th[2]) + sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + cos(th[0])*cos(th[4])) - 0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (sin(th[0])*sin(th[1])*cos(th[2]) + sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(sin(th[0])*sin(th[1])*cos(th[2]) + sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]) - 0.3922*sin(th[0])*sin(th[1])*sin(th[2]) + 0.3922*sin(th[0])*cos(th[1])*cos(th[2]) + 0.425*sin(th[0])*cos(th[1]) + 0.0996*cos(th[0])*cos(th[4]) + 0.1333*cos(th[0]),
                         -tool*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[1])*cos(th[0])*cos(th[2]) + 0.425*sin(th[1])*cos(th[0]) + 0.3922*sin(th[2])*cos(th[0])*cos(th[1]),
                         -tool*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*((sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[1])*sin(th[2])*cos(th[0]) - cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[1])*cos(th[0])*cos(th[2]) + 0.3922*sin(th[2])*cos(th[0])*cos(th[1]),
                         -tool*(-(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*(-(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]),
                         tool*(-((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*cos(th[4]) - sin(th[0])*sin(th[4])) - 0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*cos(th[4]) - 0.0996*sin(th[0])*sin(th[4]),
                            0, 0],
                            [tool*(-((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + sin(th[0])*cos(th[4])) - 0.0996*((-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2])*cos(th[0]) + cos(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[0])*cos(th[2]) - sin(th[2])*cos(th[0])*cos(th[1]))*cos(th[3]) + 0.0996*sin(th[0])*cos(th[4]) + 0.1333*sin(th[0]) + 0.3922*sin(th[1])*sin(th[2])*cos(th[0]) - 0.3922*cos(th[0])*cos(th[1])*cos(th[2]) - 0.425*cos(th[0])*cos(th[1]),
                            -tool*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[0])*sin(th[1])*cos(th[2]) + 0.425*sin(th[0])*sin(th[1]) + 0.3922*sin(th[0])*sin(th[2])*cos(th[1]),
                            -tool*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*((sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0997*(sin(th[0])*sin(th[1])*sin(th[2]) - sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]) + 0.3922*sin(th[0])*sin(th[1])*cos(th[2]) + 0.3922*sin(th[0])*sin(th[2])*cos(th[1]),
                            -tool*(-(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) - 0.0996*(-(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*sin(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*cos(th[3]))*sin(th[4]) + 0.0997*(-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + 0.0997*(-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]),
                            tool*(-((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*cos(th[4]) + sin(th[4])*cos(th[0])) - 0.0996*((-sin(th[0])*sin(th[1])*sin(th[2]) + sin(th[0])*cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[0])*sin(th[1])*cos(th[2]) - sin(th[0])*sin(th[2])*cos(th[1]))*sin(th[3]))*cos(th[4]) + 0.0996*sin(th[4])*cos(th[0]),
                            0, 0],
                            [0,
                            -tool*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) - 0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[1])*sin(th[2]) - 0.3922*cos(th[1])*cos(th[2]) - 0.425*cos(th[1]),
                            -tool*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) - 0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) + (-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) - 0.0997*(-sin(th[1])*cos(th[2]) - sin(th[2])*cos(th[1]))*cos(th[3]) + 0.3922*sin(th[1])*sin(th[2]) - 0.3922*cos(th[1])*cos(th[2]),
                            -tool*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) - (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) - 0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*cos(th[3]) - (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*sin(th[3]))*sin(th[4]) + 0.0997*(-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + 0.0997*(sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]),
                            -tool*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*cos(th[4]) - 0.0996*((-sin(th[1])*sin(th[2]) + cos(th[1])*cos(th[2]))*sin(th[3]) + (sin(th[1])*cos(th[2]) + sin(th[2])*cos(th[1]))*cos(th[3]))*cos(th[4]),
                            0, 0]])
    return jaMat06,jaMat07

def qdot(my_pos):
    global lam
    global pRCM
    global p6_real, p7_real, theta_real
    
    theta_real=rtde_r.getActualQ()
    theta_real.append(0.0)
    p6_real,p7_real=getFK(theta_real,d_val[-1])

    lamx=(pRCM[0]-p6_real[0])**2
    lamy=(pRCM[1]-p6_real[1])**2
    lamz=(pRCM[2]-p6_real[2])**2
    lam=sqrt(lamx+lamy+lamz)/d_val[-1]
    pRCM=p6_real+(lam*(p7_real-p6_real))

    J6,J7 = getJacobian(theta_real)

    p7p6_x=p7_real[0]-p6_real[0]
    p7p6_y=p7_real[1]-p6_real[1]
    p7p6_z=p7_real[2]-p6_real[2]
    p7p6=[[p7p6_x],[p7p6_y],[p7p6_z]]

    J_RCM=J6+lam*(J7-J6)
    J_RCM=np.concatenate((J_RCM,p7p6),axis=1)

    J_pseudo=np.array(np.concatenate((J7,[[0],[0],[0]]),axis=1))
    J_pseudo=np.concatenate((J_pseudo,J_RCM),axis=0)
    J_pseudo=np.array(J_pseudo,dtype=float)
    J_pseudo_inv=np.linalg.pinv(J_pseudo)    

    p6_init,p7_init=getFK(theta_real,d_val[-1])
    p_des=[p7_init[0]+my_pos[0],p7_init[1]+my_pos[1],p7_init[2]+my_pos[2]]
    t=p7_real
    err=np.concatenate((p_des-t,ptrocar-pRCM),axis=0)
    sol=J_pseudo_inv@Gain@err
    qdot=sol[:-1]
    qdes=theta_real+qdot*dt

    return qdes[:-1]

time_counter = 0.0


i=0
  
print ('>> Connect Server')

pose_init=rtde_r.getActualTCPPose()
target=pose_init.copy()
target[1]=target[1]-0.1
rtde_c.moveL(target, 0.02, 0.05)

while True:
    moved=rtde_r.getActualTCPPose()
    print(abs(moved[1]-target[1]))
    if abs(moved[1]-target[1])<=0.005:
        break
print('UR moved: ',np.array(pose_init[:3])-np.array(moved[:3]))

time_counter = 0.0
theta_init=rtde_r.getActualQ()
theta_init.append(0.0)
p6_init,p7_init=getFK(theta_init,d_val[-1])
p7_real=[0,0,0]
len_trocar=len_6_rigid+0.007
ptrocar=p6_init+((p7_init-p6_init)*((len_trocar)/(d_val[-1])))
print('p6_init: ',p6_init, 'p7_init: ',p7_init, 'ptrocar: ',ptrocar)
pRCM=ptrocar
pRCM_0=ptrocar

myx=0
myy=0
myz=0

i=0
  
initial_q=rtde_r.getActualQ()
initial_tcp_pose=rtde_r.getActualTCPPose().copy()
initial=theta_init[:-1]

bend_pos=p7_init



client_socket.settimeout(2.0)

while True:
    i=0
    num=0
    q_list=[[],[],[],[],[],[]]
 
    while i==0:
        try:
            received_data = client_socket.recv(1024)
            convdata=repr(received_data.decode('utf-8'))
            data2=convdata.split(',')
            if len(data2) != 14:
                continue
            else: data=data2
            sORf=int(data[12])
            Phantom_X=float(data[6])
            Phantom_Y=float(data[7])
            Phantom_Z=float(data[8])
            if sORf==1:
                Steer= float(data[9])
            elif sORf==2:
                Steer=float(data[1])
            Yaw= float(data[10])
            Roll=float(data[11]) 
            button=int(data[4][:-2])
            clutch=int(data[3])

            mypx=Phantom_X
            mypy=Phantom_Y
            mypz=Phantom_Z
            mypoz=Roll
            if data[0]!="'999" and data[13]!="666'":
                sORf=0
                clutch=0
                button=0
                dynamixel_process=0
            else:
                sORf=int(data[12])
                if int(data[4][:-2])>200:
                    button=1 
                else: button=0
                if int(data[3])>200:
                    clutch=3
                else: clutch=0
                dynamixel_process=1

            if sORf == 1:
                # 20:1
                interface='Stylus'
                scale_x=0.001*0.05
                scale_y=0.001*0.05
                scale_z=0.001*0.05
                scale_roll=1.0
                now = datetime.datetime.now()
                file_name = str("UR_%s_%s_%s.txt" % (interface, str(now)[:-16], str(now)[-15:-7].replace(':','-')))
                print(file_name)
                f=open('/FILE_PATH/%s'%file_name,'w+')
                i=1
            elif sORf ==2:
                interface='Finger'
                # 20:1
                scale_x=0.001*0.05
                scale_y=0.001*0.05
                scale_z=0.001*0.05
                scale_roll=1.0
                now = datetime.datetime.now()
                file_name = str("UR_%s_%s_%s.txt" % (interface, str(now)[:-16], str(now)[-15:-7].replace(':','-')))
                print(file_name)
                f=open('/FILE_PATH/%s'%file_name,'w+')
                i=1
        except socket.timeout:
            print("stop")
        
    while True:
        try:
            received_data = client_socket.recv(1024)
            convdata=repr(received_data.decode('utf-8'))
            data2=convdata.split(',')
            if len(data2) != 14:
                continue
            else: data=data2
            sORf=int(data[12])
            Phantom_X=float(data[6])
            Phantom_Y=float(data[7])
            Phantom_Z=float(data[8])
            if sORf==1:
                Steer= float(data[9])
            elif sORf==2:
                Steer=float(data[1])
            Yaw= float(data[10])
            Roll=float(data[11]) 
            button=int(data[4][:-2])
            clutch=int(data[3])
            if int(data[4][:-2])>200:
                button=1 
            else: button=0
            if int(data[3])>200:
                clutch=3
            else: clutch=0
            dynamixel_process=1

            fi_x,fi_y,fi_z=Phantom_Y,-Phantom_X,Phantom_Z
            phantom_ori=[Roll,Yaw,Steer]
            myoz=phantom_ori[0]*pi/180
            
            myx=scale_x*fi_x
            myy=scale_y*fi_y
            myz=scale_z*fi_z

            mydx=myx-mypx
            mydy=myy-mypy
            mydz=myz-mypz

            dx,dy,dz = mydx,mydy,mydz

            if clutch==0: #움직임 반영
                q_new=qdot([dx,dy,dz])
                p6_new,p7_new=getFK(q_new,d_val[-1])

                q_realval=rtde_r.getActualQ()

                bend_pos=np.array(getBendPose(q_realval,Steer))
                len_l_axis=0.131
                len_s_axis=0.076
                soft=0.7
                p7_init_for_limit=np.array(p7_init.copy())-np.array([0,0.02,0])
                
                if num<=5:
                    for i in range(5):
                        q_list[i].append(q_new[i])
                    num+=1
                else:
                    q_list[0],q_new[0]=q_filter(q_list[0], q_new[0],5)
                    q_list[1],q_new[1]=q_filter(q_list[1], q_new[1],5)
                    q_list[2],q_new[2]=q_filter(q_list[2], q_new[2],5)
                    q_list[3],q_new[3]=q_filter(q_list[3], q_new[3],5)
                    q_list[4],q_new[4]=q_filter(q_list[4], q_new[4],5)
                q_new[5]=myoz+theta_init[5]
                mypx=myx
                mypy=myy
                mypz=myz
                
            elif clutch==3: #움직임 반영 x
                print('not moving...')
                actual_tcp_pose=rtde_r.getActualTCPPose()
                mypx=myx
                mypy=myy
                mypz=myz
                mydx=0
                mydy=0
                mydz=0
                q_realval=rtde_r.getActualQ()
                
                q_new=q_realval

                q_new[5]=myoz+theta_init[5]

            t_start = rtde_c.initPeriod()
            rtde_c.servoJ(q_new, vel, acc, dt, lookahead_time, gain)
            rtde_c.waitPeriod(t_start)
            time_counter += dt
            p6_save,p7_save=getFK(rtde_r.getActualQ(),len_6_rigid)
            now = datetime.datetime.now()
            print("Phantom X:", Phantom_X,"| Phantom Y:",Phantom_Y,"| Phantom Z:",Phantom_Z,"| Steering:",Steer,"| Yawing:",Yaw,"| Rolling:",Roll,"| sORf:",sORf,"| Button:",button,"| Clutch",clutch,"| dynamixel_process:",dynamixel_process)
            f.write(str(now)+'| %f %f %f | %f %f %f | %f %f %f | %f %f %f | %f %f %f | %d | %d'
                    %(Phantom_X,Phantom_Y,Phantom_Z, Steer,Yaw,Roll, p6_save[0],p6_save[1],p6_save[2], p7_save[0],p7_save[1],p7_save[2], bend_pos[0],bend_pos[1],bend_pos[2], button,clutch))
            f.write('\n')
        
        except socket.timeout:
            print("stop")
            break
    rtde_c.servoStop()
    direction='start'
    while True:
        current_tcp_pose=np.array(rtde_r.getActualTCPPose().copy())
        target=np.array(current_tcp_pose.copy())
        diff=-(np.array(current_tcp_pose[:3])-np.array(initial_tcp_pose[:3]))
        print('Current Position: ', current_tcp_pose[:3])
        print('Initial Position: ', initial_tcp_pose[:3])
        print('X:', diff[0], ' | Y:',diff[1],' | Z:',diff[2])
        direction=input('Base frame 기준\na: +X(왼) | d: -X(오) | w: -Y(앞) | s: +Y(뒤) | r: +Z(위) | f: -Z(아래) | End: 끝\n')
        if direction=='w':
            target[1]=target[1]-0.0035
        elif direction=='s':
            target[1]=target[1]+0.0035
        elif direction=='a':
            target[0]=target[0]+0.0035
        elif direction=='d':
            target[0]=target[0]-0.0035
        elif direction=='r':
            target[2]=target[2]+0.0035
        elif direction=='f':
            target[2]=target[2]-0.0035
        elif direction=='End' or direction=='end':
            t_start = rtde_c.initPeriod()
            rtde_c.servoJ(initial_q, vel, acc, dt, lookahead_time, gain)
            rtde_c.waitPeriod(t_start)
            time_counter += dt
            break
        else:
            print('Base frame 기준\na: +X(왼) | d: -X(오) | w: -Y(앞) | s: +Y(뒤) | r: +Z(위) | f: -Z(아래) | End: 끝\n')
            continue
        t_start = rtde_c.initPeriod()   
        rtde_c.moveL(target, 0.02, 0.05)
        rtde_c.waitPeriod(t_start)
        time_counter += dt
