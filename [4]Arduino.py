import os
import psutil
import sys
from _thread import *
from ctypes import *
sys.path.append('./.local/lib/python3.6/site-packages')
sys.path.append('UR_RTDE PATH')
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import serial
import json
import time
from math import *
import numpy as np
from threading import *
import datetime

import select
import tty
import struct
import datetime
import select

###################################################
import socket
HOST = 'NETWORK HOST'
PORT = 8080
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
exit_flag=False
data=[0,0,0,0,0,0,0,0,0,0]
lock=Lock()

def receive_motion_data():
    global exit_flag
    ser = serial.Serial('USB PORT', 19200)
    while not exit_flag:
        if ser.readable():
            val=ser.readline()
            while val[0]!=57 or val[0:3]!=b'999':
                val=ser.readline()
            try:
                chunk = client_socket.send(val)
                print(datetime.datetime.now())
                print(val)
                if not chunk:
                    raise ConnectionError("Connection closed unexpectedly")
                time.sleep(0.005)
            except Exception as e:
                exit_flag = True
                print(f"Exception: {e}")
                break
    else:
        client_socket.close()

print ('>> Connect Server')

while True:
    receive_motion_data()
