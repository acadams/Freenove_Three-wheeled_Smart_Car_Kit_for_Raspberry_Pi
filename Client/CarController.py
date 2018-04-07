from TCPClient import TCPClient
from Command import COMMAND as cmd

from Freenove_Math import constrain
import time
import threading
import math
import socket
from CloseThreading import *

class CarController:
    tcp = TCPClient()
    cameraHorizontalPosition = 90
    cameraVerticalPosition = 90
    SERVO_MIN_ANGLE = 0
    SERVO_MAX_ANGLE = 180
    sonicIndex = 0
    isTcpIdle = True

    # Quick one-liner for getting the local IP Address
    default_Server_IP = str(socket.gethostbyname(socket.gethostname()))

    ## Init function
    def __init__(self):
        ## allows for thread locking
        self.mutex = threading.Lock()
        try:
            self.tcp.connectToServer(address= (self.default_Server_IP, 12345))
        except Exception:
            print("Unable to connect to TCP Server. Make sure to run:")
            print("python Main.py &")
            print("In the Server directory")
            return

    ### Camera Motion Methods ###
    def moveCameraVertically(self, delta):
        self.cameraVerticalPosition = self.cameraVerticalPosition + delta
        self.cameraVerticalPosition = constrain(self.cameraVerticalPosition, self.SERVO_MIN_ANGLE, self.SERVO_MAX_ANGLE)
        self.tcp.sendData(cmd.CMD_CAMERA_UP + str(self.cameraVerticalPosition))

    def moveCameraHorizontally(self, delta):
        self.cameraHorizontalPosition = self.cameraHorizontalPosition + delta
        self.cameraHorizontalPosition = constrain(self.cameraHorizontalPosition, self.SERVO_MIN_ANGLE, self.SERVO_MAX_ANGLE)
        self.tcp.sendData(cmd.CMD_CAMERA_LEFT + str(self.cameraHorizontalPosition))        
    
    def stopCameraMovement(self):
        self.tcp.sendData(cmd.CMD_CAMERA_STOP)

    def resetCameraPosition(self):
        self.cameraHorizontalPosition = 90
        self.cameraVerticalPosition = 90
        self.tcp.sendData(cmd.CMD_CAMERA_LEFT + str(90))
        self.tcp.sendData(cmd.CMD_CAMERA_UP + str(90))

    ### Car Motor Methods ###
    def moveCarForward(self, speed):
        self.setMoveSpeed(cmd.CMD_FORWARD, speed)

    def on_btn_Backward_pressed(self, speed):
        self.setMoveSpeed(cmd.CMD_BACKWARD, speed)

    def stopCar(self):
        self.tcp.sendData(cmd.CMD_STOP)

    ### Car Turning Methods ###
    def turnLeft(self, value):
        self.tcp.sendData(cmd.CMD_TURN_LEFT + str(value))

    def on_btn_TurnLeft_released(self):
        self.tcp.sendData(cmd.CMD_TURN_CENTER + str(90))

    def turnRight(self, value):
        self.tcp.sendData(cmd.CMD_TURN_RIGHT + str(value))

    # Utility method for gradually sending speed to the car shield
    def setMoveSpeed(self, CMD, spd):
        self.tcp.sendData(CMD + str(spd/3))
        time.sleep(0.07)
        self.tcp.sendData(CMD + str(spd/3*2))
        time.sleep(0.07)
        self.tcp.sendData(CMD + str(spd))
