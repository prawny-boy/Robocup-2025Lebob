from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop

DRIVEBASE_WHEEL_DIAMETER = 0
DRIVEBASE_AXLE_TRACK = 0
ROBOT_SPEED = 500
ROBOT_ACCELERATION = 750
ROBOT_TURN_RATE = 750
ROBOT_TURN_ACCELERATION = 3000

class Robot:
    def __init__(self):
        self.hub = PrimeHub(Axis.Z, Axis.X)
        self.left_drive = Motor(Port.A)
        self.right_drive = Motor(Port.B)

        self.colour_sensor_left = ColorSensor(Port.C)
        self.colour_sensor_right = ColorSensor(Port.D)

        self.drivebase = DriveBase(self.left_drive, self.right_drive, DRIVEBASE_WHEEL_DIAMETER, DRIVEBASE_AXLE_TRACK)
        self.drivebase.use_gyro(False)
        self.drivebase.settings(
            straight_speed=ROBOT_SPEED, 
            straight_acceleration=ROBOT_ACCELERATION, 
            turn_rate=ROBOT_TURN_RATE, 
            turn_acceleration=ROBOT_TURN_ACCELERATION
        )
    
    def turn_in_degrees(self, degrees):
        self.drivebase.turn(degrees)
    
    def move_forward(self, distance):
        """Moves forward in mm."""
        self.drivebase.straight(distance)

def main():
    robot.drivebase.straight(100)

robot = Robot()

if __name__ == "main":
    main()