from CarController import *
from time import sleep
from copy import deepcopy
import threading
import RPi.GPIO as GPIO

controller = CarController(90, 120)
controller.stopCar()
