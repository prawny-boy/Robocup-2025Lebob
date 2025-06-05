from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop

# --- CONSTANTS ---
CONSTANTS = {
    "DRIVEBASE_WHEEL_DIAMETER": 56,
    "DRIVEBASE_AXLE_TRACK": 112,
    "ROBOT_SPEED": 500,
    "ROBOT_ACCELERATION": 750,
    "ROBOT_TURN_RATE": 750,
    "ROBOT_TURN_ACCELERATION": 3000,
    "ROBOT_MOVE_SPEED": 350,
    "ROBOT_TURNING_DEGREES": 6,
    "ULTRASONIC_THRESHOLD": 80,
    "TURN_GREEN_DEGREES": 75,
    "CURVE_RADIUS_GREEN": 75,
    "CURVE_RADIUS_OBSTACLE": 200,
    "OBSTACLE_TURN_DEGREES": 140,
    "OBSTACLE_INITIAL_TURN_DEGREES": 90,
    "OBSTACLE_FINAL_TURN_DEGREES": 45,
    "CURVE_RADIUS_LINE_FOLLOW": 4,
}

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

        self.drivebase = DriveBase(
            self.left_drive,
            self.right_drive,
            CONSTANTS["DRIVEBASE_WHEEL_DIAMETER"],
            CONSTANTS["DRIVEBASE_AXLE_TRACK"]
        )
        self.drivebase.use_gyro(False)
        self.drivebase.settings(
            straight_speed=CONSTANTS["ROBOT_SPEED"],
            straight_acceleration=CONSTANTS["ROBOT_ACCELERATION"],
            turn_rate=CONSTANTS["ROBOT_TURN_RATE"],
            turn_acceleration=CONSTANTS["ROBOT_TURN_ACCELERATION"]
        )

        self.robot_state = "obstacle" # Initial state
        self.iteration_count = 0 # Initialize the iteration counter

    def turn_in_degrees(self, degrees):
        """Turn in degrees. Using a curve for line following."""
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_LINE_FOLLOW"], degrees, Stop.COAST, False)

    def sharp_turn_in_degrees(self, degrees):
        """Perform a sharp turn in degrees."""
        self.drivebase.turn(degrees)

    def move_forward(self, distance, speed=None):
        """Move forward in mm."""
        if speed is not None:
            self.set_speed(speed)
        self.drivebase.straight(distance)

    def start_motors(self, left_speed, right_speed):
        """Start the motors, if not already started, each with the specified speed."""
        if not self.left_drive.speed() == left_speed:
            self.left_drive.run(left_speed)
        if not self.right_drive.speed() == right_speed:
            self.right_drive.run(right_speed)

    def stop_motors(self):
        """Stop the motors."""
        self.left_drive.stop()
        self.right_drive.stop()

    def set_speed(self, speed):
        """Set the speed of the robot in drivebase settings."""
        if speed == 0:
            speed = CONSTANTS["ROBOT_SPEED"]
        self.drivebase.settings(straight_speed=speed)

    def information_to_color(self, information):
        """Convert color sensor information to a color."""
        if information["color"] == Color.WHITE:
            return Color.WHITE
        elif information["color"] == Color.GREEN and 135 < information["hsv"].h < 165:
            return Color.GREEN
        elif information["color"] == Color.BLUE and information["hsv"].s > 80 and information["hsv"].v > 80:
            return Color.BLUE
        elif information["color"] == Color.RED and information["hsv"].s > 85:
            return Color.RED
        elif information["color"] == Color.RED: # information["hsv"].s <= 85
            return Color.ORANGE
        elif information["hsv"].s < 20 and 3 < information["reflection"] < 30:
            return Color.BLACK
        else:
            return Color.NONE

    def get_colors(self):
        """Get the reflection, color, and HSV values of the left and right color sensors to be used for color detection."""
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
        """When there is a green on the left or the right, react to it by doing a larger turn left or right."""
        if direction == "left":
            degrees = -CONSTANTS["TURN_GREEN_DEGREES"] 
        else:
            degrees = CONSTANTS["TURN_GREEN_DEGREES"]
        
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_GREEN"], degrees, Stop.COAST, True)

    def avoid_obstacle(self):
        self.stop_motors()
        self.sharp_turn_in_degrees(CONSTANTS["OBSTACLE_INITIAL_TURN_DEGREES"])
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_OBSTACLE"], -CONSTANTS["OBSTACLE_TURN_DEGREES"], Stop.BRAKE, True)
        self.start_motors(CONSTANTS["ROBOT_MOVE_SPEED"], CONSTANTS["ROBOT_MOVE_SPEED"])

        self.get_colors() # Re-read colors after turning
        while self.right_color == Color.WHITE: # Keep turning until right sensor sees something other than white
            self.get_colors()
        self.turn_in_degrees(CONSTANTS["OBSTACLE_FINAL_TURN_DEGREES"])
        self.robot_state = "straight" # Reset state after handling obstacle
    
    def update(self):
        """Update the state of the robot."""
        self.get_colors()
        self.ultrasonic = self.ultrasonic_sensor.distance()

        # Check for obstacle
        if self.ultrasonic < CONSTANTS["ULTRASONIC_THRESHOLD"]:
            self.robot_state = "obstacle"
            return # Exit update early if obstacle detected

        # Forward (white on both, or black on both)
        if (self.left_color == Color.WHITE and self.right_color == Color.WHITE) or \
           (self.left_color == Color.BLACK and self.right_color == Color.BLACK):
            self.robot_state = "straight"

        # Black line following
        elif self.left_color == Color.BLACK:
            self.robot_state = "left" # Directly transition to "left"
        elif self.right_color == Color.BLACK:
            self.robot_state = "right" # Directly transition to "right"

        # Green
        elif self.left_color == Color.GREEN:
            self.robot_state = "green left"
        elif self.right_color == Color.GREEN:
            self.robot_state = "green right"

        # Nothing
        else:
            self.robot_state = "stop"

    def move(self):
        """Move the robot based on its current state."""
        # Obstacle
        if self.robot_state == "obstacle":
            self.avoid_obstacle()

        # Straight
        elif self.robot_state == "straight":
            self.start_motors(CONSTANTS["ROBOT_MOVE_SPEED"], CONSTANTS["ROBOT_MOVE_SPEED"])

        # Black line following - now immediately turns
        elif self.robot_state == "left":
            self.turn_in_degrees(-CONSTANTS["ROBOT_TURNING_DEGREES"]) # Turn left
        elif self.robot_state == "right":
            self.turn_in_degrees(CONSTANTS["ROBOT_TURNING_DEGREES"]) # Turn right

        # Green
        elif self.robot_state == "green left":
            self.stop_motors()
            self.turn_green("left")
        elif self.robot_state == "green right":
            self.stop_motors()
            self.turn_green("right")

        # Stop
        elif self.robot_state == "stop":
            self.stop_motors()

    def debug(self):
        """Print debug text."""
        print(self.left_color_sensor_information, self.left_color)

    def run(self):
        while True:
            self.iteration_count += 1 # Increment the counter at the start of each loop
            self.update()
            self.move()
            # self.debug()

def main():
    robot = Robot()
    robot.run()

main() # Note: don't put in if __name__ == "__main__" because __name__ is something different for robot