from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop

DRIVEBASE_WHEEL_DIAMETER = 56
DRIVEBASE_AXLE_TRACK = 112
ROBOT_SPEED = 500
ROBOT_ACCELERATION = 750
ROBOT_TURN_RATE = 750
ROBOT_TURN_ACCELERATION = 3000

ports = {
    "left_drive": Port.E,
    "right_drive": Port.C,
    "colour_sensor_left": Port.D,
    "colour_sensor_right": Port.A,
    "ultrasonic_sensor": Port.B,
    "arm_motor": Port.F,
}

class Robot:
    def __init__(self):
        self.hub = PrimeHub(Axis.Z, Axis.X)
        self.left_drive = Motor(ports["left_drive"])
        self.right_drive = Motor(ports["right_drive"])

        self.colour_sensor_left = ColorSensor(ports["colour_sensor_left"])
        self.colour_sensor_right = ColorSensor(ports["colour_sensor_right"])
        self.ultrasonic_sensor = UltrasonicSensor(ports["ultrasonic_sensor"])
        self.arm_motor = Motor(ports["arm_motor"])

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
        reflection_left = robot.colour_sensor_left.reflection()
        reflection_right = robot.colour_sensor_right.reflection()
        print(reflection_left, reflection_right)

        # Turn if needed
        if False:
            if colour_left == Color.BLACK and colour_right == Color.WHITE: # Left
                robot.turn_in_degrees(-1)
            elif colour_left == Color.WHITE and colour_right == Color.BLACK: # Right
                robot.turn_in_degrees(1)
            else: # Forward
                robot.move_forward(1)

main()