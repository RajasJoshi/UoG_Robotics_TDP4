import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SupervisorBase import SupervisorBase

while True:
    supervisor = SupervisorBase()
    timeStep = int(supervisor.getBasicTimeStep())
    while supervisor.step(supervisor.timeStep) != -1:
        supervisor.sendSupervisorData()
