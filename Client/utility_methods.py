import RPi.GPIO as GPIO

##SLOW_DOWN_LIMIT = 0
##TURN_LIMIT = 0
##STOP_LIMIT = 0
##
##RED_PIN = 36
##GREEN_PIN = 38
##BLUE_PIN = 37

### Exercise 1 ###

## Use this list to test your Exercise 1 methods
testBuffer = [17, 35, 39, 39, 35, 4, 19, 36, 17, 9, 22, 42, 32, 47, 41, 29, 30, 44, 48, 14]

def splitData(sonicData):

  # Our final answer
  splitData = []
  numberOfSection = 5
  sizeOfSection = len(sonicData) / numberOfSection
  for sectionNumber in range(numberOfSection):
    startingIndex = sectionNumber * sizeOfSection
    endingIndex = sectionNumber * sizeOfSection + sizeOfSection
    distanceList = sonicData[startingIndex:endingIndex]
    splitData.append(distanceList)

  return splitData

### Exercise 2 ###

## Use this list to test your Exercise 2 methods
testSplitList = [[16, 42, 8, 22], [32, 5, 38, 23], [7, 27, 32, 22], [47, 1, 3, 47], [33, 38, 40, 22]]

def getFarLeftData(splitData):
  return splitData[0]

def getTopLeftData(splitData):
  return splitData[1]

def getTopData(splitData):
  return splitData[2]

def getTopRightData(splitData):
  return splitData[3]

def getFarRightData(splitData):
  return splitData[4]

'''
Helper method for filtering out large/nonexistant data that we
don't wan't to bother reading. Don't worry about how this function
works unless you are interested in list comprehension
'''
def filterData(distanceData):
  return [data for data in distanceData if (data < 100 and data != 0)]


### Exercise 3 ###
## Use this list to test your Exercise 3 methods
testDistanceList = [11, 14, 5, 46, 46]

def getAverageDistance(distanceList):
  if len(distanceList) == 0:
    return 0
  return sum(distanceList) / float(len(distanceList))

### Exercise 4 ###
def setSlowDownLimit(distanceLimit):
  SLOW_DOWN_LIMIT = distanceLimit

def shouldSlowDown(splitData, SLOW_DOWN_LIMIT):
  for distanceData in splitData:
    distanceAverage = getAverageDistance(distanceData)
    if (distanceAverage < SLOW_DOWN_LIMIT):
      return True
  return False

### Exercise 5 ###
def setTurnLimit(distanceLimit):
  TURN_LIMIT = distanceLimit

def shouldTurnLeft(splitData, TURN_LIMIT):
  farRightData = getFarRightData(splitData)
  topRightData = getTopRightData(splitData)
  if (len(filterData(farRightData)) == 0 and len(filterData(topRightData)) == 0):
    return False
  if (getAverageDistance(filterData(farRightData)) < TURN_LIMIT):
    if (getAverageDistance(filterData(topRightData)) < TURN_LIMIT):
      return True
  return False

def canTurnLeft(splitData, TURN_LIMIT):
  topData = getTopData(splitData)
  topLeftData = getTopLeftData(splitData)
  if(getAverageDistance(topData) >= TURN_LIMIT):
    if(getAverageDistance(topLeftData) >= TURN_LIMIT):
      return True
  return False

def shouldTurnRight(splitData, TURN_LIMIT):
  farLeftData = getFarLeftData(splitData)
  topLeftData = getTopLeftData(splitData)
  if (len(filterData(farLeftData)) == 0 and len(filterData(topLeftData)) == 0):
    return False
  if (getAverageDistance(filterData(farLeftData)) < TURN_LIMIT):
    if (getAverageDistance(filterData(topLeftData)) < TURN_LIMIT):
      return True
  return False

def canTurnRight(splitData, TURN_LIMIT):
  topData = getTopData(splitData)
  topRightData = getTopRightData(splitData)
  if(getAverageDistance(topData) >= TURN_LIMIT):
    if(getAverageDistance(topRightData) >= TURN_LIMIT):
      return True
  return False

### Exercise 6 ###
def setStopLimit(distanceLimit):
  STOP_LIMIT = distanceLimit

def shouldStop(splitData, STOP_LIMIT):
  topData = filterData(getTopData(splitData))
  if (len(filterData(topData)) == 0):
    return False
  return getAverageDistance(topData) < STOP_LIMIT

##### Exercise 7 ###
##def setLightPins(redPin, greenPin, bluePin):
##  GPIO.cleanup()
##  GPIO.setmode(GPIO.BOARD)
##  GPIO.setup(redPin, GPIO.OUT)
##  GPIO.setup(greenPin, GPIO.OUT)
##  GPIO.setup(bluePin, GPIO.OUT)
##
##def toggleLightPins(turnOnRed, turnOnGreen, turnOnBlue):
##  GPIO.output(RED_PIN, turnOnRed)
##  GPIO.output(GREEN_PIN, turnOnGreen)
##  GPIO.output(BLUE_PIN, turnOnBlue)
