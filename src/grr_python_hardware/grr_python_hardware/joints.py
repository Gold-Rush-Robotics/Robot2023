from grr_roboclaw.roboclaw import Roboclaw
from trajectory_msgs.msg import JointTrajectoryPoint
from dataclasses import dataclass
from numpy import pi
from adafruit_pca9685 import PCA9685

@dataclass
class SubState:
    name: str
    position: float
    velocity: float
    effort: float

class Joint:
    def __init__(self, name: str, attachment: int) -> None:
        self.name = name
        self.attachment = attachment
    def get_state(self) -> SubState:
        return SubState("", 0, 0, 0)
    def follow_state(self, state: JointTrajectoryPoint, index: int) -> bool:
        return False
    def __str__(self) -> str:
        return f"Joint of type {__name__}"
    
class Motor(Joint):
    def __init__(self, name: str, attachment: int, rc_addy: int, m1: bool, ticks_per_rev: int, roboclaw: Roboclaw) -> None:
        super().__init__(name, attachment)
        self.rc_addy = rc_addy
        self.m1 = m1
        self.roboclaw = roboclaw
        self.ticks_per_rev = ticks_per_rev
    
    def get_state(self) -> SubState:
        if self.ticks_per_rev != 0:
            position = self.roboclaw.ReadEncM1(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadEncM2(self.rc_addy)[1]
            position = (position / self.ticks_per_rev) * 2 * pi
            velocity = self.roboclaw.ReadSpeedM1(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadSpeedM2(self.rc_addy)[1]
        else:
            position = 0
            velocity = 0
        effort = self.roboclaw.ReadCurrents(self.rc_addy)[1 if self.m1 else 2]
        effort /= self.roboclaw.ReadM1MaxCurrent(self.rc_addy)[1] if self.m1 else self.roboclaw.ReadM2MaxCurrent(self.rc_addy)[1] 
        return SubState(self.name, position, velocity, effort*100)

    def __str__(self) -> str:
        return super().__str__() + f" named: {self.name} @ {self.rc_addy} {'m1' if self.m1 else 'm2'} on attachment {self.attachment} w/ {self.ticks_per_rev} t/r"
    
    def follow_state(self, state: JointTrajectoryPoint, index: int) -> bool:
        if self.ticks_per_rev != 0:
            buffer = ""
            self.roboclaw.SpeedAccelDistanceM1(self.rc_addy, state.accelerations[index], state.velocities[index], state.positions[index], buffer)
        else:
                            
            self.roboclaw.SpeedM1(self.rc_addy, state.effort[index]) if self.m1 else self.roboclaw.SpeedM2(self.rc_addy, state.effort[index])
            
        
        return True
    
class Servo(Joint):
    def __init__(self, name: str, attachment: int, servo_port: int, maximum_val: int, minimum_val: int, pca:PCA9685) -> None:
        super().__init__(name, attachment)
        self.servo_port = servo_port
        self.maximum_val = maximum_val
        self.minimum_val = minimum_val
        self.pca = pca
        
    def get_state(self) -> SubState:
        position = self.pca.channels[self.servo_port].duty_cycle
        return SubState(self.name, position, 0, 0)
    
    def follow_state(self, state: JointTrajectoryPoint, index: int) -> bool:
        effort = state.effort[index]
        scaled = (float(effort) / float(100))*0xFFFF
        self.pca.channels[self.servo_port].duty_cycle = scaled
        return True