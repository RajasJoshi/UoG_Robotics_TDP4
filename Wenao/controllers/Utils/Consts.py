import math
from enum import Enum

from controller import Motion

TIME_STEP = 40

PI = math.pi


class Motions:
    def __init__(self):
        self.handWave = MotionBase("handWave", "../../motions/HandWave.motion")
        self.forwards = MotionBase("forwards", "../../motions/Forwards.motion")
        self.forwardsSprint = MotionBase(
            "forwardsSprint", "../../motions/ForwardsSprint.motion"
        )
        self.forwards50 = MotionBase("forwards50", "../../motions/Forwards50.motion")
        self.backwards = MotionBase("backwards", "../../motions/Backwards.motion")
        self.shoot = MotionBase("shoot", "../../motions/Shoot.motion")
        self.rightShoot = MotionBase("rightShoot", "../../motions/RightShoot.motion")
        self.longShoot = MotionBase("longShoot", "../../motions/LongPass.motion")
        self.leftSidePass = MotionBase(
            "leftSidePass", "../../motions/SidePass_Left.motion"
        )
        self.rightSidePass = MotionBase(
            "rightSidePass", "../../motions/SidePass_Right.motion"
        )
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
        self.turnLeft10 = MotionBase("turnLeft10", "../../motions/TurnLeft10.motion")
        self.turnLeft20 = MotionBase("turnLeft20", "../../motions/TurnLeft20.motion")
        self.turnLeft30 = MotionBase("turnLeft30", "../../motions/TurnLeft30.motion")
        self.turnLeft40 = MotionBase("turnLeft40", "../../motions/TurnLeft40.motion")
        self.turnLeft60 = MotionBase("turnLeft60", "../../motions/TurnLeft60.motion")
        self.turnLeft180 = MotionBase("turnLeft180", "../../motions/TurnLeft180.motion")
        self.turnRight10 = MotionBase("turnRight10", "../../motions/TurnRight10.motion")
        self.turnRight10_V2 = MotionBase(
            "turnRight10", "../../motions/TurnRight10_V2.motion"
        )
        self.turnRight40 = MotionBase("turnRight40", "../../motions/TurnRight40.motion")
        self.turnRight60 = MotionBase("turnRight60", "../../motions/TurnRight60.motion")
        self.standInit = MotionBase("standInit", "../../motions/StandInit.motion")


class MotionBase(Motion):
    def __init__(self, name, path):
        super().__init__(path)
        self.name = name
