from TCPClient import TCPClient

import time
import threading
import socket
from CloseThreading import *
import smbus

def numMap(value,fromLow,fromHigh,toLow,toHigh):
	return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

class CarController:
    tcp = TCPClient()
    sonicHorizontalPosition = 90

    ## Ultrasonic Data
    sonicIndex = 0
    sonicBuff = [0] * 20

    CMD_SERVO1 = 0 # Steering
    CMD_SERVO2 = 1 # Horizontal Rotation
    CMD_SERVO3 = 2
    CMD_SERVO4 = 3

    ## The two motors
    CMD_PWM1 = 4
    CMD_PWM2 = 5

    ## Direction will either be 0 (forward) or 1 (backwards)
    CMD_DIR1 = 6
    CMD_DIR2 = 7

    CMD_BUZZER = 8
    CMD_IO1 = 9
    CMD_IO2 = 0
    CMD_IO3 = 1
    CMD_SONIC = 2
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
    def writeReg(self, cmd, value):
        try:
            self.bus.write_i2c_block_data(self.address, cmd, [value>>8, value&0xff])
            time.sleep(0.001)
        except Exception,e:
            print Exception,"I2C Error :",e

	def readReg(self,cmd):
		[a,b] = self.bus.read_i2c_block_data(self.address, cmd, 2)
		return a<<8 | b

    def sendTCPData(self, data):
        self.tcpClientSock.send(data)

    ### Sonar Motion Methods ###
    def getSonicData(self):
        SonicEchoTime = self.readReg(self.CMD_SONIC)
		distance = SonicEchoTime * 17.0 / 1000.0
		return distance

    def turnOnSonar(self):
        self.recvSonicThread = Recv_Sonic_Thread(self)
        self.recvSonicThread.start()
        self.scanSonicThread = Scan_Sonic_Thread(self)
        self.scanSonicThread.start()

    def turnOffSonar(self):
        if self.recvSonicThread.is_alive():
            self.recvSonicThread.isRun = False
        if self.scanSonicThread.is_alive():
            self.scanSonicThread.isRun = False

    ### Car Motor Methods ###
    def moveCarForward(self):
        self.writeReg(self.CMD_DIR1,0)
        self.writeReg(self.CMD_DIR2,0)
        for i in range(0,1000,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)

    def moveCarBackwards(self):
        self.writeReg(self.CMD_DIR1,1)
        self.writeReg(self.CMD_DIR2,1)
        for i in range(0,1000,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)

    def stopCar(self):
        self.writeReg(mdev.CMD_PWM1,0)
        self.writeReg(mdev.CMD_PWM2,0)

    ### Car Turning Methods ###
    def turnLeft(self, value):
        min_Angle = 45
        inteval_Angle = 10
        for angle in range(90, min_Angle + 1, inteval_Angle):
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO1, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
        for angle in range(min_Angle, 90, inteval_Angle):
            if self.wgt_main.mutex.acquire():
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO1, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
        print('finished turning left')

    def turnRight(self, value):
        max_Angle = 135
        inteval_Angle = 10
        for angle in range(90, max_Angle + 1, inteval_Angle):
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO1, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
        for angle in range(max_Angle, 90, inteval_Angle):
            if self.wgt_main.mutex.acquire():
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO1, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
        print('finished turning right')

## Setup for UltraSonic Threads ##
class Recv_Sonic_Thread(threading.Thread):
    def __init__(self, widget):
        super(Recv_Sonic_Thread, self).__init__()
        self.wgt_main = widget
        self.isRun = True
    
    def run(self):
        while self.isRun:
            sonic = self.wgt_main.tcp.recvData()
            try:
                iSonic = float(sonic)
            except Exception, e:
                print "Sonic Data error: ", e
                iSonic = 0
            self.wgt_main.sonicBuff[self.wgt_main.sonicIndex] = iSonic

class Scan_Sonic_Thread(threading.Thread):
    def __init__(self, widget):
        super(Scan_Sonic_Thread, self).__init__()
        self.wgt_main = widget
        self.isRun = True

    def run(self):
        while self.isRun:
            self.scan_Sonic()
            
    def scan_Sonic(self):
        self.min_Angle = 45
        self.max_Angle = 135
        self.inteval_Angle = 10
        self.scan_speed = 0.05
        #print "scan Sonic...."
        for angle in range(self.min_Angle, self.max_Angle + 1, self.inteval_Angle):
            self.wgt_main.sonicIndex = angle / self.inteval_Angle
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                # self.wgt_main.tcp.sendData(cmd.CMD_SONIC_LEFT+str(self.wgt_main.sonicHorizontalPosition))
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO2, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                # self.wgt_main.tcp.sendData(cmd.CMD_ULTRASONIC)
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
            time.sleep(self.scan_speed)
        print self.wgt_main.sonicBuff

        for angle in range(self.max_Angle, self.min_Angle - 1, -1 * self.inteval_Angle):
            self.wgt_main.sonicIndex = angle / self.inteval_Angle
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                # self.wgt_main.tcp.sendData(cmd.CMD_SONIC_LEFT+str(self.wgt_main.sonicHorizontalPosition))
                value = int(self.wgt_main.sonicHorizontalPosition)
                mdev.writeReg(self.wgt_main.CMD_SERVO2, numMap(value,0,180,SERVO_MIN_PULSE_WIDTH,SERVO_MAX_PULSE_WIDTH))
                # self.wgt_main.tcp.sendData(cmd.CMD_ULTRASONIC)
                sonic = self.wgt_main.getSonic()
                self.wgt_main.sendData(str(sonic))
                self.wgt_main.mutex.release()
            time.sleep(self.scan_speed)
        print self.wgt_main.sonicBuff