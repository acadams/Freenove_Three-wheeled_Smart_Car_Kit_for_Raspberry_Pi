from TCPClient import TCPClient
from Command import COMMAND as cmd

from Freenove_Math import constrain
import time
import threading
import math
import socket
from CloseThreading import *
import smbus
import time

def numMap(value,fromLow,fromHigh,toLow,toHigh):
	return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

class CarController:
    tcp = TCPClient()
    cameraHorizontalPosition = 90
    cameraVerticalPosition = 90
    SERVO_MIN_ANGLE = 0
    SERVO_MAX_ANGLE = 180
    sonicIndex = 0
    isTcpIdle = True
    CMD_SERVO1 		= 	0
    CMD_SERVO2 		= 	1
    CMD_SERVO3 		= 	2
    CMD_SERVO4 		= 	3
    CMD_PWM1		=	4
    CMD_PWM2		=	5
    CMD_DIR1		=	6
    CMD_DIR2		=	7
    CMD_BUZZER		=	8
    CMD_IO1			=	9
    CMD_IO2			=	10
    CMD_IO3			=	11
    CMD_SONIC		=	12
    SERVO_MAX_PULSE_WIDTH = 2500
    SERVO_MIN_PULSE_WIDTH = 500
    Is_IO1_State_True = False
    Is_IO2_State_True = False
    Is_IO3_State_True = False
    Is_Buzzer_State_True = False

    # Quick one-liner for getting the local IP Address
    default_Server_IP = str(socket.gethostbyname(socket.gethostname()))

    ## Init function
    def __init__(self):
        ## allows for thread locking
        self.mutex = threading.Lock()
        self.address = 0x18
        self.bus = smbus.SMBus(1)
        self.bus.open(1)
        try:
            self.tcp.connectToServer(address= (self.default_Server_IP, 12345))
        except Exception:
            print("Unable to connect to TCP Server. Make sure to run:")
            print("python Main.py &")
            print("In the Server directory")
            return
    def writeReg(self,cmd,value):
        try:
            self.bus.write_i2c_block_data(self.address,cmd,[value>>8,value&0xff])
            time.sleep(0.001)
        except Exception,e:
            print Exception,"I2C Error :",e

    ### Sonar Motion Methods ###
    def moveSonarHorizontally(self, delta):
        ## DEF CODE

    def resetCameraPosition(self):
        self.cameraHorizontalPosition = 90
        self.cameraVerticalPosition = 90
        self.tcp.sendData(cmd.CMD_CAMERA_LEFT + str(90))
        self.tcp.sendData(cmd.CMD_CAMERA_UP + str(90))

    ### Car Motor Methods ###
    def moveCarForward(self, seconds):
        self.writeReg(self.CMD_DIR1,0)
        self.writeReg(self.CMD_DIR2,0)
        for i in range(0,1000,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)
        time.sleep(seconds)

    def moveCarBackwards(self):
        self.writeReg(self.CMD_DIR1,1)
        self.writeReg(self.CMD_DIR2,1)
        for i in range(0,1000,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)
        time.sleep(seconds)

    def stopCar(self):
        self.tcp.sendData(cmd.CMD_STOP)

    ### Car Turning Methods ###
    def turnLeft(self, value):
        ## DEF CODE

    def turnRight(self, value):
        ## DEF CODE