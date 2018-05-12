from CarController import *
from time import sleep
from copy import deepcopy
import threading
import RPi.GPIO as GPIO

## Default variables
STOP_LIMIT = 40
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

    def splitData(self, sonicData):
        splitDataList = []
        splitDataList.append(sonicData[0:4])
        splitDataList.append(sonicData[4:8])
        splitDataList.append(sonicData[8:12])
        splitDataList.append(sonicData[12:16])
        splitDataList.append(sonicData[16:20])
        return splitDataList

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

    def getTopData(self, splitData):
        return splitData[2]

    def getAverageDistance(self, distanceList):
        return float(sum(distanceList)) / len(distanceList)

    def shouldStopCar(self, topData):
        averageDistance = self.getAverageDistance(topData)
        print('Average data value: ', averageDistance)
        return averageDistance < STOP_LIMIT

    '''
    Helper method for filtering out large/nonexistant data that we
    don't wan't to bother reading. Don't worry about how this function
    works unless you are interested in list comprehension
    '''
    def filterData(self, distanceData):
        return [data for data in distanceData if (data < 100 and data != 0)]

    '''
    Helper function properly shutting down the car
    Turns off motors, and sonic, and sets the light state to red (stopped)
    '''
    def shutDownCar(self):
        self.controller.stopCar()
        self.controller.turnOffSonar()
        sleep(self.CHECK_TIME)
        self.toggleLightPins(True, False, False)
        self.controller.setStraightSonarAngle(90)
        self.isRun = False

    def run(self):
        self.controller.turnOnSonar()
        self.controller.moveCarForward()
        self.toggleLightPins(False, True, False)
        shouldStop = False
        try:
            while self.isRun:
                '''
                The while loop below is where we start our decision making for the
                car. Read this while as: While I should not stop...keep moving.
                It then
                1. gets the data
                2. splits it
                3. Get the top data
                4. filters out unwanted information
                5. checks if the average value is below the limit.

                As long as it is over the limit, the car will continue to
                move forward.
                '''
                while (not shouldStop):
                    # Gives the ultrasonic time to get the data
                    sleep(self.CHECK_TIME)
                    sonicData = deepcopy(self.controller.sonicBuff)

                    # Step 2
                    newData = self.splitData(sonicData)
                    # Step 3
                    allTopData = self.getTopData(newData)
                    # Step 4
                    filteredTopData = self.filterData(allTopData)
                    # Step 5
                    if (len(filteredTopData) > 0):
                        shouldStop = self.shouldStopCar(filteredTopData)
                        print('should I stop? ', shouldStop)
                        

                ## These don't get executed until we should  stop
                print('Car should be stopped, shut things down.')
                self.shutDownCar()


        ## try/except blocks are something we didn't cover.
        ## This is to make sure that if the code errors out, we stop the car
        except Exception as inst:
            print type(inst)
            print (inst.args)
            print(inst)
            self.shutDownCar()

        except KeyboardInterrupt as inst:
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
moveCar.start()
