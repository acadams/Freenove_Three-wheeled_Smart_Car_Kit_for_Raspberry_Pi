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

    ## Ultrasonic Data
    sonicIndex = 0
    sonicBuff = [0] * 20

    CMD_SERVO1 = 0 # Steering
    CMD_SERVO2 = 1 # Horizontal Rotation of Sonar
    CMD_SERVO3 = 2 # Unused
    CMD_SERVO4 = 3 # Unused

    ## The two motors
    CMD_PWM1 = 4
    CMD_PWM2 = 5

    ## Direction will either be 0 (forward) or 1 (backwards)
    CMD_DIR1 = 6
    CMD_DIR2 = 7

    # Buzzer
    CMD_BUZZER = 8

    # Ultrasonic
    CMD_SONIC = 2

    # PWM Constraints
    SERVO_MAX_PULSE_WIDTH = 2500
    SERVO_MIN_PULSE_WIDTH = 500

    CAR_SPEED = 350

    # Quick one-liner for getting the local IP Address
    default_Server_IP = str(socket.gethostbyname(socket.gethostname()))

    ## Init function
    def __init__(self, sonarAngle = 90, turnAngle = 90):
        ## allows for thread locking
        self.mutex = threading.Lock()
        self.address = 0x18
        self.bus = smbus.SMBus(1)
        self.bus.open(1)
        self.setStraightSonarAngle(sonarAngle)
        self.setStraightTurnAngle(turnAngle)

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
        except Exception as e:
            print ("I2C Error :" + str(e))

    def readReg(self,cmd):
        [a,b] = self.bus.read_i2c_block_data(self.address, cmd, 2)
        return a<<8 | b

    def turnOnSonar(self):
        self.scanSonicThread = Scan_Sonic_Thread(self)
        self.scanSonicThread.start()
        self.recvSonicThread = Recv_Sonic_Thread(self)
        self.recvSonicThread.start()

    def turnOffSonar(self):
        if self.recvSonicThread.is_alive():
            self.recvSonicThread.isRun = False
        if self.scanSonicThread.is_alive():
            self.scanSonicThread.isRun = False

    ### Car Motor Methods ###
    def moveCarForward(self):
        self.writeReg(self.CMD_DIR1,0)
        self.writeReg(self.CMD_DIR2,0)
        # always start movement at min speed
        for i in range(0,350,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)

    def slowCarDown(self):
        if (self.CAR_SPEED == 350):
            # do nothing, at min speed
            return
        for speedDelta in range(self.CAR_SPEED, 350, -10):
            self.writeReg(self.CMD_PWM1,speedDelta)
            self.writeReg(self.CMD_PWM2,speedDelta)
            time.sleep(0.005)
        self.CAR_SPEED = 350

    def speedCarUp(self):
        if (self.CAR_SPEED == 500):
            # do nothing, at max speed
            return
        for speedDelta in range(self.CAR_SPEED, 500, 10):
            self.writeReg(self.CMD_PWM1,speedDelta)
            self.writeReg(self.CMD_PWM2,speedDelta)
            time.sleep(0.005)
        self.CAR_SPEED = 500

    ## Helper method for calibrating what 'straight' is for the
    ## car's wheel axis
    def setStraightTurnAngle(self, angle):
        self.straightTurnAngle = angle
        self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))

    ## Helper method for calibrating what 'straight' is for the
    ## car's sonar axis
    def setStraightSonarAngle(self, angle):
        self.straightSonarAngle = angle
        self.writeReg(self.CMD_SERVO2, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))

    def moveCarBackwards(self):
        self.writeReg(self.CMD_DIR1,1)
        self.writeReg(self.CMD_DIR2,1)
        # not using the car speed, will always reverse at min speed for safety
        for i in range(0,350,10):
            self.writeReg(self.CMD_PWM1,i)
            self.writeReg(self.CMD_PWM2,i)
            time.sleep(0.005)

    def stopCar(self):
        for speedDelta in range(self.CAR_SPEED, 0, 10):
            self.writeReg(self.CMD_PWM1,speedDelta)
            self.writeReg(self.CMD_PWM2,speedDelta)
            time.sleep(0.005)

    ### Car Turning Methods ###
    def turnRight(self):
        min_Angle = self.straightTurnAngle - 45
        inteval_Angle = 10
        for angle in range(self.straightTurnAngle, min_Angle + 1, -inteval_Angle):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(.2)
        for angle in range(min_Angle, self.straightTurnAngle, inteval_Angle):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(.2)
        print('finished turning right')

    def turnLeft(self):
        max_Angle = self.straightTurnAngle + 45
        inteval_Angle = 10
        for angle in range(self.straightTurnAngle, max_Angle + 1, inteval_Angle):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(.2)
        for angle in range(max_Angle, self.straightTurnAngle + 1, -inteval_Angle):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(.2)
        print('finished turning left')

    def turnOnBuzzer(self, seconds):
        self.writeReg(self.CMD_BUZZER, 2000)
        time.sleep(seconds)
        self.writeReg(self.CMD_BUZZER, 0)

    def turnAtAngle(self, turnAngle, turnTime):
        if (turnAngle >= 90):
            intervalScalar = 1
        else:
            intervalScalar = -1
        intervalAngle = float(abs(self.straightTurnAngle - turnAngle)) / 10
        turnSpeed = turnTime / 20
        for angle in range(self.straightTurnAngle, turnAngle + 1, (intervalAngle * intervalScalar)):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(turnSpeed)
        for angle in range(turnAngle, self.straightTurnAngle + 1, -(intervalAngle * intervalScalar)):
            if self.mutex.acquire():
                self.writeReg(self.CMD_SERVO1, numMap(angle,0,180,self.SERVO_MIN_PULSE_WIDTH,self.SERVO_MAX_PULSE_WIDTH))
                self.mutex.release()
                time.sleep(turnSpeed)

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
            except Exception as e:
                print ("Sonic Data error: ", e)
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
        self.min_Angle = 9
        self.max_Angle = 181
        self.inteval_Angle = 9
        self.scan_speed = 0.5 / (40)

        for angle in range(self.min_Angle, self.max_Angle, self.inteval_Angle):
            self.wgt_main.sonicIndex = min(angle / self.inteval_Angle, 19)
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                self.wgt_main.writeReg(self.wgt_main.CMD_SERVO2, numMap(angle,0,180,self.wgt_main.SERVO_MIN_PULSE_WIDTH,self.wgt_main.SERVO_MAX_PULSE_WIDTH))
                self.wgt_main.tcp.sendData(">Ultrasonic")
                self.wgt_main.mutex.release()
            time.sleep(self.scan_speed)
        for angle in range(self.max_Angle, self.min_Angle + 1, -1 * self.inteval_Angle):
            self.wgt_main.sonicIndex = min(angle / self.inteval_Angle, 19)
            self.wgt_main.sonicHorizontalPosition = angle
            if self.wgt_main.mutex.acquire():
                self.wgt_main.writeReg(self.wgt_main.CMD_SERVO2, numMap(angle,0,180,self.wgt_main.SERVO_MIN_PULSE_WIDTH,self.wgt_main.SERVO_MAX_PULSE_WIDTH))
                self.wgt_main.tcp.sendData(">Ultrasonic")
                self.wgt_main.mutex.release()
            time.sleep(self.scan_speed)
