import math
from enum import Enum

from controller import Motion

TIME_STEP = 40

PI = math.pi


class Motions:
    def __init__(self):
        self.handWave = MotionBase("handWave", "../../motions/HandWave.motion")
        self.forwards = MotionBase("forwards", "../../motions/Forwards.motion")
        self.backwards = MotionBase("backwards", "../../motions/Backwards.motion")
        self.shoot = MotionBase("shoot", "../../motions/Shoot.motion")
        self.sideStepLeft = MotionBase(
            "sideStepLeft", "../../motions/SideStepLeft.motion"
        )
        self.sideStepRight = MotionBase(
            "sideStepRight", "../../motions/SideStepRight.motion"
        )
        self.standUpFromFront = MotionBase(
            "standUpFromFront", "../../motions/StandUpFromFront.motion"
        )
        self.standUpFromBack = MotionBase(
            "standUpFromBack", "../../motions/StandUpFromBack.motion"
        )
        self.turnLeft40 = MotionBase("turnLeft40", "../../motions/TurnLeft40.motion")
        self.turnLeft60 = MotionBase("turnLeft60", "../../motions/TurnLeft60.motion")
        self.turnLeft180 = MotionBase("turnLeft180", "../../motions/TurnLeft180.motion")
        self.turnRight40 = MotionBase("turnRight40", "../../motions/TurnRight40.motion")
        self.turnRight60 = MotionBase("turnRight60", "../../motions/TurnRight60.motion")
        self.standInit = MotionBase("standInit", "../../motions/StandInit.motion")


class MotionBase(Motion):
    def __init__(self, name, path):
        super().__init__(path)
        self.name = name
