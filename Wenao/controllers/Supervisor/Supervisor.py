import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SupervisorBase import SupervisorBase
from Utils.Consts import TIME_STEP

supervisor = SupervisorBase()

while supervisor.step(TIME_STEP) != -1:
    supervisor.sendSupervisorData()
    supervisor.isitGoal()
    supervisor.updateScoreboard()
