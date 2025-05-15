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
    "color_sensor_left": Port.D,
    "color_sensor_right": Port.A,
    "ultrasonic_sensor": Port.B,
    "arm_motor": Port.F,
}

class Robot:
    def __init__(self):
        self.hub = PrimeHub(Axis.Z, Axis.X)
        self.left_drive = Motor(ports["left_drive"])
        self.right_drive = Motor(ports["right_drive"])

        self.color_sensor_left = ColorSensor(ports["color_sensor_left"])
        self.color_sensor_right = ColorSensor(ports["color_sensor_right"])
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

    def information_to_color(self, information):
        if information["color"] == Color.WHITE and information["hsv"].v > 80 and information["reflection"] == 100:
            return Color.GRAY
        elif information["color"] == Color.WHITE:
            return Color.WHITE
        elif information["color"] == Color.GREEN and information["hsv"].h > 135 and information["hsv"].h < 165:
            return Color.GREEN
        elif information["color"] == Color.BLUE and information["hsv"].s > 80 and information["hsv"].v > 80:
            return Color.BLUE
        elif information["color"] == Color.RED and information["hsv"].s > 85:
            return Color.RED
        elif information["color"] == Color.RED: # information["hsv"].s <= 85
            return Color.ORANGE
        elif information["color"] in [Color.BLUE, Color.BLACK, Color.GREEN] and information["hsv"].s < 20 and information["reflection"] < 30:
            return Color.BLACK
        else: 
            return Color.NONE
    
    def get_colors(self):
        self.left_color_sensor_information = {
            "reflection": self.color_sensor_left.reflection(),
            "color": self.color_sensor_left.color(),
            "hsv": self.color_sensor_left.hsv(),
        }
        self.right_color_sensor_information = {
            "reflection": self.color_sensor_right.reflection(),
            "color": self.color_sensor_right.color(),
            "hsv": self.color_sensor_right.hsv(),
        }

        self.left_color = self.information_to_color(self.left_color_sensor_information)
        self.right_color = self.information_to_color(self.right_color_sensor_information)

    def move(self):
        if self.left_color == Color.BLACK and self.right_color == Color.WHITE: # Left
            self.turn_in_degrees(-1)
        elif self.left_color == Color.WHITE and self.right_color == Color.BLACK: # Right
            self.turn_in_degrees(1)
        else: # Forward
            self.move_forward(1)

    def debug(self):            
        print(self.left_color_sensor_information, self.left_color, self.right_color)
    
    def run(self):
        while True:
            self.get_colors()
            # self.move()
            self.debug()

def main():
    robot = Robot()
    robot.run()

main()