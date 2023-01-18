
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
            pos=[-2, 2, .5],
            size=[1, 1, 1]
        )

        pyrosim.End()

        while not os.path.exists('world.sdf'):
            time.sleep(.01)

    def Create_Robot(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(
            name="LinkTorso",
            pos=[0, 0, 1.],
            size=[1, 1, 1]
        )

        pyrosim.Send_Joint(
            name="LinkTorso_LinkBackLeg",
            parent="LinkTorso",
            child="LinkBackLeg",
            type="revolute",
            position=[0, -.5, 1],
            jointAxis="1 0 0"
        )

        pyrosim.Send_Cube(
            name="LinkBackLeg",
            pos=[0, -.5, 0],
            size=[.2, 1, .2]
        )

        pyrosim.Send_Joint(
            name="LinkBackLeg_LinkLowerBackLeg",
            parent="LinkBackLeg",
            child="LinkLowerBackLeg",
            type="revolute",
            position=[0, -1., 0],
            jointAxis="1 0 0"
        )

        pyrosim.Send_Cube(
            name="LinkLowerBackLeg",
            pos=[0, 0, -.5],
            size=[.2, .2, 1]
        )

        pyrosim.Send_Joint(
            name="LinkTorso_LinkFrontLeg",
            parent="LinkTorso",
            child="LinkFrontLeg",
            type="revolute",
            position=[0, .5, 1],
            jointAxis="1 0 0"
        )

        pyrosim.Send_Cube(
            name="LinkFrontLeg",
            pos=[0, .5, 0],
            size=[.2, 1, .2]
        )

        pyrosim.Send_Joint(
            name="LinkFrontLeg_LinkLowerFrontLeg",
            parent="LinkFrontLeg",
            child="LinkLowerFrontLeg",
            type="revolute",
            position=[0, 1., 0],
            jointAxis="1 0 0"
        )

        pyrosim.Send_Cube(
            name="LinkLowerFrontLeg",
            pos=[0, 0, -.5],
            size=[.2, .2, 1]
        )

        pyrosim.Send_Joint(
            name="LinkTorso_LinkLeftLeg",
            parent="LinkTorso",
            child="LinkLeftLeg",
            type="revolute",
            position=[-.5, 0, 1],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="LinkLeftLeg",
            pos=[-.5, 0, 0],
            size=[1, .2, .2]
        )

        pyrosim.Send_Joint(
            name="LinkLeftLeg_LinkLowerLeftLeg",
            parent="LinkLeftLeg",
            child="LinkLowerLeftLeg",
            type="revolute",
            position=[-1, 0., 0],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="LinkLowerLeftLeg",
            pos=[0, 0, -.5],
            size=[.2, .2, 1]
        )

        pyrosim.Send_Joint(
            name="LinkTorso_LinkRightLeg",
            parent="LinkTorso",
            child="LinkRightLeg",
            type="revolute",
            position=[.5, 0, 1],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="LinkRightLeg",
            pos=[.5, 0, 0],
            size=[1, .2, .2]
        )

        pyrosim.Send_Joint(
            name="LinkRightLeg_LinkLowerRightLeg",
            parent="LinkRightLeg",
            child="LinkLowerRightLeg",
            type="revolute",
            position=[1, 0., 0],
            jointAxis="0 1 0"
        )

        pyrosim.Send_Cube(
            name="LinkLowerRightLeg",
            pos=[0, 0, -.5],
            size=[.2, .2, 1]
        )

        pyrosim.End()

        while not os.path.exists('body.urdf'):
            time.sleep(.01)

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork('brain{}.nndf'.format(self.myID))

        pyrosim.Send_Sensor_Neuron(name=0, linkName='LinkTorso')
        pyrosim.Send_Sensor_Neuron(name=1, linkName='LinkBackLeg')
        pyrosim.Send_Sensor_Neuron(name=2, linkName='LinkFrontLeg')
        pyrosim.Send_Sensor_Neuron(name=3, linkName='LinkLeftLeg')
        pyrosim.Send_Sensor_Neuron(name=4, linkName='LinkRightLeg')
        pyrosim.Send_Sensor_Neuron(name=5, linkName='LinkLowerBackLeg')
        pyrosim.Send_Sensor_Neuron(name=6, linkName='LinkLowerFrontLeg')
        pyrosim.Send_Sensor_Neuron(name=7, linkName='LinkLowerLeftLeg')
        pyrosim.Send_Sensor_Neuron(name=8, linkName='LinkLowerRightLeg')

        pyrosim.Send_Motor_Neuron(name=9, jointName='LinkTorso_LinkBackLeg')
        pyrosim.Send_Motor_Neuron(name=10, jointName='LinkTorso_LinkFrontLeg')
        pyrosim.Send_Motor_Neuron(name=11, jointName='LinkTorso_LinkLeftLeg')
        pyrosim.Send_Motor_Neuron(name=12, jointName='LinkTorso_LinkRightLeg')
        pyrosim.Send_Motor_Neuron(name=13, jointName='LinkBackLeg_LinkLowerBackLeg')
        pyrosim.Send_Motor_Neuron(name=14, jointName='LinkFrontLeg_LinkLowerFrontLeg')
        pyrosim.Send_Motor_Neuron(name=15, jointName='LinkLeftLeg_LinkLowerLeftLeg')
        pyrosim.Send_Motor_Neuron(name=16, jointName='LinkRightLeg_LinkLowerRightLeg')

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

