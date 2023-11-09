from Base.SupervisorBase import SupervisorBase
from Scoreboard import Scoreboard
from Utils.Consts import TIME_STEP

while True:
    supervisor = SupervisorBase()
    scoreboard = Scoreboard()

    while supervisor.step(TIME_STEP) != -1:
        scoreboard.updateScoreboard(supervisor)
        supervisor.sendSupervisorData()
