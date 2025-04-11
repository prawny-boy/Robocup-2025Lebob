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
        """Turns in degrees."""
        self.drivebase.turn(degrees)
    
    def move_forward(self, distance):
        """Moves forward in mm."""
        self.drivebase.straight(distance)

    def set_speed(self, speed):
        """Sets the speed of the robot in drivebase settings."""
        if speed == 0:
            speed = ROBOT_SPEED
        self.drivebase.settings(straight_speed=speed)

def main():
    robot = Robot()

    while True:
        # Sense colours below
        colour_left = robot.colour_sensor_left.color()
        colour_right = robot.colour_sensor_right.color()

        # Turn if needed
        if colour_left == Color.BLACK and colour_right == Color.WHITE: # Left
            robot.turn_in_degrees(-1)
        elif colour_left == Color.WHITE and colour_right == Color.BLACK: # Right
            robot.turn_in_degrees(1)
        else: # Forward
            robot.move_forward(1)

if __name__ == "main":
    main()