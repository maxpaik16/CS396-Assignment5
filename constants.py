
import numpy as np

NUM_STEPS = 2000

GRAVITY_X = 0
GRAVITY_Y = 0
GRAVITY_Z = -9.8

amplitudeFront = np.pi / 3
frequencyFront = 20
phaseOffsetFront = 0

amplitudeBack = np.pi / 4
frequencyBack = 20
phaseOffsetBack = np.pi

MAX_FORCE = 50

SLEEP_TIME = 1/240

numberOfGenerations = 20

populationSize = 10

numSensorNeurons = 4
numMotorNeurons = 3

motorJointRange = 1.