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

# Robot Behavior
ROBOT_FORWARD_SPEED = 350
ROBOT_TURNING_INCREMENT = 6

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

        self.robot_state = "obstacle"
    
    def turn_in_degrees(self, degrees=ROBOT_TURNING_INCREMENT):
        """Turns in degrees."""
        self.drivebase.curve(25, degrees, Stop.COAST, False)
    
    def short_turn_in_degrees(self, degrees):
        self.drivebase.turn(degrees)
    
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
        # if information["reflection"] == 100:
        #     return Color.GRAY
        if information["color"] == Color.WHITE:
            return Color.WHITE
        elif information["color"] == Color.GREEN and information["hsv"].h > 135 and information["hsv"].h < 165:
            return Color.GREEN
        elif information["color"] == Color.BLUE and information["hsv"].s > 80 and information["hsv"].v > 80:
            return Color.BLUE
        elif information["color"] == Color.RED and information["hsv"].s > 85:
            return Color.RED
        elif information["color"] == Color.RED: # information["hsv"].s <= 85
            return Color.ORANGE
        elif information["hsv"].s < 20 and information["reflection"] < 30 and information["reflection"] > 3:
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
        self.get_colors()
        self.ultrasonic = self.ultrasonic_sensor.distance()

        # check for obstacle
        if self.ultrasonic < 80:
            self.robot_state = "obstacle"
            return

        # forward
        if self.left_color == Color.WHITE and self.right_color == Color.WHITE:
            self.robot_state = "straight"

        # black line following
        elif self.left_color == Color.BLACK and self.robot_state != "left":
            self.robot_state = "new left" # new will mean that it will stop then turn
        elif self.right_color == Color.BLACK and self.robot_state != "right":
            self.robot_state = "new right"

        # green
        elif self.left_color == Color.GREEN:
            self.robot_state = "green left"
        elif self.right_color == Color.GREEN:
            self.robot_state = "green right"

        # nothing
        else:
            self.robot_state = "stop"

    def move(self):
        # obstackle
        if self.robot_state == "obstacle":
            self.stop_motors()
            self.short_turn_in_degrees(90)
            self.drivebase.curve(180, -150, Stop.BRAKE, True)
            self.start_motors(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED)
            self.get_colors()
            while self.right_color == Color.WHITE:
                self.get_colors()
            self.turn_in_degrees(45)
            self.robot_state = "straight"

        # straight
        if self.robot_state == "straight":
            self.start_motors(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED)

        elif self.robot_state == "new left": # this will stop, then set it to left
            self.stop_motors()
            self.turn_in_degrees(-ROBOT_TURNING_INCREMENT)
            self.robot_state = "left"
        elif self.robot_state == "left": # left, no stopping
            self.turn_in_degrees(-ROBOT_TURNING_INCREMENT)

        elif self.robot_state == "new right": # this will stop, then set it to right
            self.stop_motors()
            self.turn_in_degrees(ROBOT_TURNING_INCREMENT)
            self.robot_state = "right"
        elif self.robot_state == "right": # right, no stopping
            self.turn_in_degrees(ROBOT_TURNING_INCREMENT)
        
        # green
        elif self.robot_state == "green left":
            self.stop_motors()
            self.turn_green("left")
        elif self.robot_state == "green right":
            self.stop_motors()
            self.turn_green("right")

        # stop
        elif self.robot_state == "stop":
            self.stop_motors()
        
    def debug(self):
        print(f"LC: {self.left_color} RC: {self.right_color} U: {self.ultrasonic} S: {self.robot_state}{" "*30}")
    
    def run(self):
        while True:
            self.update()
            self.move()
            self.debug()

def main():
    robot = Robot()
    robot.run()

main()