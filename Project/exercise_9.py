from CarController import *
from utility_methods import *
from time import sleep
from copy import deepcopy
import threading
import RPi.GPIO as GPIO
import sys
import os

## Default variables
STOP_LIMIT = 50
SLOW_DOWN_LIMIT = 110
TURN_LIMIT = 90

'''
This is the MoveCarThread Class. We did not go into threading in the previous
session. The primary thing you need to know is that each of the functions from
the exercises need to go into this class for things to work.

This class Executes Exercise 9 in the final project. We've set our stop limit
(above) at 50.
'''
class MoveCarThread(threading.Thread):
    def __init__(self, checkTime):
        super(MoveCarThread, self).__init__()
        self.controller = CarController(90, 120)
        self.isRun = True
        self.CHECK_TIME = checkTime
        GPIO.setmode(GPIO.BOARD)

    def setLightPins(self, redPin, greenPin, bluePin):
          GPIO.cleanup()
          GPIO.setmode(GPIO.BOARD)
          GPIO.setup(redPin, GPIO.OUT)
          GPIO.setup(greenPin, GPIO.OUT)
          GPIO.setup(bluePin, GPIO.OUT)
          self.RED_PIN = redPin
          self.GREEN_PIN = greenPin
          self.BLUE_PIN = bluePin

    def toggleLightPins(self, turnOnRed, turnOnGreen, turnOnBlue):
        '''
        Anthony why are you negating everything??? B/c the led light
        starts turned ON, so everything we want to do should be
        reversed
        '''
        GPIO.output(self.RED_PIN, not turnOnRed)
        GPIO.output(self.GREEN_PIN, not turnOnGreen)
        GPIO.output(self.BLUE_PIN, not turnOnBlue)

    '''
    Helper function properly shutting down the car
    Turns off motors, and sonic, and sets the light state to red (stopped)
    '''
    def shutDownCar(self):
        self.controller.stopCar()
        self.controller.turnOffSonar()
        sleep(self.CHECK_TIME)
        self.controller.setStraightSonarAngle(90)
        self.isRun = False

    def run(self):
        self.controller.turnOnSonar()
        self.controller.moveCarForward()
        shouldStopBool = False
        try:
            while self.isRun:
                print("Your code should go here")
                sleep(5)
                self.shutDownCar()

        ## try/except blocks are something we didn't cover.
        ## This is to make sure that if the code errors out, we stop the car
        except Exception as inst:
            exec_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exec_type, fname, exc_tb.tb_lineno)
            print type(inst)
            print (inst.args)
            print(inst)
            self.shutDownCar()

        except KeyboardInterrupt as inst:
            exec_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exec_type, fname, exc_tb.tb_lineno)
            print type(inst)
            print (inst.args)
            print(inst)
            self.shutDownCar()           


'''
Last block of code: This is how we can use the class above
to setup and start our car.
'''
moveCar = MoveCarThread(0.5)
moveCar.setLightPins(37,40,38)
