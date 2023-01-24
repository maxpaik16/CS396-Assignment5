
import numpy as np
import pyrosim.pyrosim as pyrosim
import os

import time
import random
import constants as c


class SOLUTION:

    def __init__(self, ID):
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights *= 2
        self.weights -= 1
        self.myID = ID

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Robot()
        self.Create_Brain()
        # print('python simulate.py {} {} &'.format(directOrGUI, self.myID))
        os.system('python simulate.py {} {} 2&>1 &'.format(directOrGUI, self.myID))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists('fitness{}.txt'.format(self.myID)):
            time.sleep(.01)
        with open('fitness{}.txt'.format(self.myID), 'r') as f:
            self.fitness = float(f.read())
        # print(self.fitness)
        os.system('rm {}'.format('fitness{}.txt'.format(self.myID)))

    def Evaluate(self, directOrGUI):
        self.Create_World()
        self.Create_Robot()
        self.Create_Brain()
        os.system('python simulate.py {} {} &'.format(directOrGUI, self.myID))

    def Create_World(self):
        pyrosim.Start_SDF('world.sdf')

        pyrosim.Send_Cube(
            name="Box",
            pos=[0, 0., .5],
            size=[20, 1.5, 1]
        )

        pyrosim.End()

        while not os.path.exists('world.sdf'):
            time.sleep(.01)

    def Create_Robot(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(
            name="Link1",
            pos=[0, 0, 1.5],
            size=[1, .5, .5]
        )

        pyrosim.Send_Joint(
            name="Link1_Link2",
            parent="Link1",
            child="Link2",
            type="revolute",
            position=[.5, 0, 1.5],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="Link2",
            pos=[.5, 0, 0],
            size=[1, .5, .5]
        )

        pyrosim.Send_Joint(
            name="Link2_Link3",
            parent="Link2",
            child="Link3",
            type="revolute",
            position=[1, 0, 0],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="Link3",
            pos=[.5, 0, 0],
            size=[1, .5, .5]
        )

        pyrosim.Send_Joint(
            name="Link3_Link4",
            parent="Link3",
            child="Link4",
            type="revolute",
            position=[1, 0, 0],
            jointAxis="0 0 1"
        )

        pyrosim.Send_Cube(
            name="Link4",
            pos=[.5, 0, 0],
            size=[1, .5, .5]
        )

        pyrosim.End()

        while not os.path.exists('body.urdf'):
            time.sleep(.01)

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork('brain{}.nndf'.format(self.myID))

        pyrosim.Send_Sensor_Neuron(name=0, linkName='Link1')
        pyrosim.Send_Sensor_Neuron(name=1, linkName='Link2')
        pyrosim.Send_Sensor_Neuron(name=2, linkName='Link3')
        pyrosim.Send_Sensor_Neuron(name=3, linkName='Link4')

        pyrosim.Send_Motor_Neuron(name=4, jointName='Link1_Link2')
        pyrosim.Send_Motor_Neuron(name=5, jointName='Link2_Link3')
        pyrosim.Send_Motor_Neuron(name=6, jointName='Link3_Link4')

        for i in range(c.numSensorNeurons):
            for j in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + c.numSensorNeurons, weight=self.weights[i, j])

        pyrosim.End()

        while not os.path.exists('brain{}.nndf'.format(self.myID)):
            time.sleep(.01)

    def Mutate(self):
        row = random.randint(0, c.numSensorNeurons - 1)
        column = random.randint(0, c.numMotorNeurons - 1)

        self.weights[row, column] = 2 * random.random() - 1

    def Set_ID(self, val):
        self.myID = val

