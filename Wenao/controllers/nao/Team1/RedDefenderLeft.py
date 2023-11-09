"""
Red Team Left Defender robot behaviours.
"""
from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import TIME_STEP, Motions


class DefenderLeft(SoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.isNewBallDataAvailable():
                # Do not remove this!
                # ----------------------
                self.getSupervisorData()
                # ----------------------

                # Use the ballData (location) to do something.
                ballCoordinate = self.getBallData()
                selfCoordinate = self.getSelfCoordinate()
                decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

                self.startMotion()
            else:
                # It seems there is a problem.
                # print("NO BALL DATA!!!")
                pass

    # Override decideMotion
    def decideMotion(self, ballCoordinate, selfCoordinate):
        # Check the goal scored to balance itself.
        if self.checkGoal() == 1:
            return self.motions.handWave
        elif self.checkGoal() == -1:
            return self.motions.standInit
