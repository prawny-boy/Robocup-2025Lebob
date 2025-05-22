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

        self.left_ticks_on_green = 0
        self.right_ticks_on_green = 0
        self.green_turn_cooldown = 0
    
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

    def update(self):
        self.green_turn_cooldown += 1
        if self.left_color == Color.GREEN:
            self.left_ticks_on_green += 1
        else:
            self.left_ticks_on_green = 0
        
        if self.right_color == Color.GREEN:
            self.right_ticks_on_green += 1
        else:
            self.right_ticks_on_green = 0

    def move(self):
        if self.left_color == Color.WHITE and self.right_color == Color.WHITE:
            self.move_forward(5)
        elif self.left_color == Color.BLACK and self.right_color == Color.WHITE:
            self.turn_in_degrees(-2)
        elif self.left_color == Color.WHITE and self.right_color == Color.BLACK:
            self.turn_in_degrees(2)
        
        if self.green_turn_cooldown >= 20:
            if self.left_ticks_on_green >= 5:
                self.move_forward(80)
                self.turn_in_degrees(-90)
                self.move_forward(50)
                green_turn_cooldown = 0
            elif self.right_ticks_on_green >= 5:
                self.move_forward(80)
                self.turn_in_degrees(90)
                self.move_forward(50)
                green_turn_cooldown = 0
            else:
                self.move_forward(2)
    #     elif self.left_color == Color.LEFT and self.right_color == Color.RIGHT:
    #         self.action()
    def debug(self):            
        print(self.left_ticks_on_green, self.right_ticks_on_green, self.green_turn_cooldown)
    
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