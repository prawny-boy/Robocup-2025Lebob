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
ROBOT_MOVE_SPEED = 350
ROBOT_TURNING_DEGREES = 6

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
        self.left_drive = Motor(ports["left_drive"], positive_direction=Direction.COUNTERCLOCKWISE)
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

        self.current_direction = "straight"
    
    def turn_in_degrees(self, degrees=ROBOT_TURNING_DEGREES):
        """Turns in degrees."""
        self.drivebase.curve(25, degrees, Stop.COAST, False)
    
    def move_forward(self, distance, speed=None):
        """Moves forward in mm."""
        if speed is not None:
            self.set_speed(speed)
        self.drivebase.straight(distance)
    
    def start_motors(self, left_speed, right_speed):
        if not self.left_drive.speed() == left_speed:
            self.left_drive.run(left_speed)
        if not self.right_drive.speed() == right_speed:
            self.right_drive.run(right_speed)
    
    def stop_motors(self):
        self.left_drive.stop()
        self.right_drive.stop()

    def set_speed(self, speed):
        """Sets the speed of the robot in drivebase settings."""
        if speed == 0:
            speed = ROBOT_SPEED
        self.drivebase.settings(straight_speed=speed)

    def information_to_color(self, information):
        if information["reflection"] == 100:
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

    def turn_green(self, direction):
        if direction == "left":
            degrees = -75
        else:
            degrees = 75

        self.drivebase.curve(75, degrees, Stop.COAST, True)

    def update(self):
        # forward
        if self.left_color == Color.WHITE and self.right_color == Color.WHITE:
            self.current_direction = "straight"

        # black line following
        elif self.left_color == Color.BLACK and self.current_direction != "left":
            self.current_direction = "new left" # new will mean that it will stop then turn
        elif self.right_color == Color.BLACK and self.current_direction != "right":
            self.current_direction = "new right"

        # green
        elif self.left_color == Color.GREEN:
            self.current_direction = "green left"
        elif self.right_color == Color.GREEN:
            self.current_direction = "green right"

    def move(self):
        if self.current_direction == "straight":
            self.start_motors(ROBOT_MOVE_SPEED, ROBOT_MOVE_SPEED)

        elif self.current_direction == "new left": # this will stop, then set it to left
            self.stop_motors()
            self.turn_in_degrees(-ROBOT_TURNING_DEGREES)
            self.current_direction = "left"
        elif self.current_direction == "left": # left, no stopping
            self.turn_in_degrees(-ROBOT_TURNING_DEGREES)

        elif self.current_direction == "new right": # this will stop, then set it to right
            self.stop_motors()
            self.turn_in_degrees(ROBOT_TURNING_DEGREES)
            self.current_direction = "right"
        elif self.current_direction == "right": # right, no stopping
            self.turn_in_degrees(ROBOT_TURNING_DEGREES)
        
        # green
        elif self.current_direction == "green left":
            self.stop_motors()
            self.turn_green("left")
        elif self.current_direction == "green right":
            self.stop_motors()
            self.turn_green("right")
        
    def debug(self):            
        print(self.current_direction)
    
    def run(self):
        while True:
            self.get_colors()
            self.update()
            self.move()
            self.debug()

def main():
    robot = Robot()
    robot.run()

main()