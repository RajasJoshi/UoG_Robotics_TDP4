import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from Base.SupervisorBase import SupervisorBase
from Utils.Consts import TIME_STEP
from Utils import Functions

class Supervisor(SupervisorBase):
    def __init__(self):
        super().__init__()

    def run(self):
        while super().step(TIME_STEP) != -1:
            super().sendSupervisorData()
            super().isitGoal()
            super().updateScoreboard()
            self.logging()

    def logging(self):
        data = super().get_data()
        Functions.write_data('log.csv',data)

if __name__ == '__main__':
    supervisor = Supervisor()
    supervisor.run()