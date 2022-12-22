from controller import Motion

class Current_motion_manager:
    def __init__(self):
        self.currentMotion = None
    
    def get(self):
        return self.currentMotion
    
    def isOver(self):
        return self.currentMotion.isOver()
    
    def set(self, motion):
        if self.currentMotion:
            self.currentMotion.stop()
            self._reset_isOver_flag(self.currentMotion)
        self.currentMotion = motion
        motion.play()

    def _reset_isOver_flag(self, motion):
        motion.play()
        motion.stop()

class Motion_library:
    def __init__(self):
        self.motions = {
            'Backwards': Motion('../motions/Backwards.motion'),
            'ForwardLoop': Motion('../motions/ForwardLoop.motion'),
            'Forwards50': Motion('../motions/Forwards50.motion'),
            'Forwards': Motion('../motions/Forwards.motion'),
            'GetUpBack': Motion('../motions/GetUpBack.motion'),
            'GetUpFront': Motion('../motions/GetUpFront.motion'),
            'HandWave': Motion('../motions/HandWave.motion'),
            'Shoot': Motion('../motions/Shoot.motion'),
            'SideStepLeft': Motion('../motions/SideStepLeft.motion'),
            'SideStepRight': Motion('../motions/SideStepRight.motion'),
            'Stand': Motion('../motions/Stand.motion'),
            'StandUpFromFront': Motion('../motions/StandUpFromFront.motion'),
            'TaiChi': Motion('../motions/TaiChi.motion'),
            'TurnLeft180': Motion('../motions/TurnLeft180.motion'),
            'TurnLeft40': Motion('../motions/TurnLeft40.motion'),
            'TurnLeft60': Motion('../motions/TurnLeft60.motion'),
            'TurnRight40': Motion('../motions/TurnRight40.motion'),
            'TurnRight60': Motion('../motions/TurnRight60.motion'),
            'WipeForehead': Motion('../motions/WipeForehead.motion')
        }
        self.motions['ForwardLoop'].setLoop(True)
    
    def add(self, name, motion):
        self.motions[name] = motion
    
    def get(self, name):
        return self.motions[name]