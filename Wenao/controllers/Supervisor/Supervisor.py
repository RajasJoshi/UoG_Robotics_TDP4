import os, sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from Base.SupervisorBase import SupervisorBase
from Utils.Consts import TIME_STEP

supervisor = SupervisorBase()

while supervisor.step(TIME_STEP) != -1:
    supervisor.sendSupervisorData()
    supervisor.isitGoal()
    supervisor.updateScoreboard()
