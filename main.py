from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop

ALLOW_YELLOW = False    

# --- CONSTANTS ---
CONSTANTS = {
    "DRIVEBASE_WHEEL_DIAMETER": 56,
    "DRIVEBASE_AXLE_TRACK": 112,
    "ARM_MOVE_SPEED": 500,
    "DEFAULT_SPEED": 130,
    "DEFAULT_ACCELERATION": 600,
    "DEFAULT_TURN_RATE": 600,
    "DEFAULT_TURN_ACCELERATION": 2000,
    "OBSTACLE_MOVE_SPEED": 300,
    "MOVE_SPEED": 130,
    "ULTRASONIC_THRESHOLD": 70,
    "TURN_GREEN_DEGREES": 70,
    "TURN_YELLOW_DEGREES": 20,
    "CURVE_RADIUS_GREEN": 80,
    "CURVE_RADIUS_OBSTACLE": 130,
    "OBSTACLE_TURN_DEGREES": 140,
    "OBSTACLE_INITIAL_TURN_DEGREES": 90,
    "OBSTACLE_FINAL_TURN_DEGREES": 45,
    "CURVE_RADIUS_LINE_FOLLOW": 4,
    "MAX_TURN_RATE": 500,
    "BLACK_COUNTER_THRESHOLD": 1000,
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
        self.previous_state = "line"

        self.drivebase = DriveBase(
            self.left_drive,
            self.right_drive,
            CONSTANTS["DRIVEBASE_WHEEL_DIAMETER"],
            CONSTANTS["DRIVEBASE_AXLE_TRACK"]
        )
        self.drivebase.use_gyro(False)
        self.settings_default()

        self.robot_state = "obstacle" # Initial state
        self.iteration_count = 0 # Initialize the iteration counter
        self.black_counter = 0
        self.on_inverted = False

        self.shortcut_information = {
            "is following shortcut": False, # If the robot is following a shortcut
            "first turned": None, # Which direction the robot turned first due to a shortcut
            "left seen black since": False, # If the left sensor has seen black since the last shortcut
            "right seen black since": False # If the right sensor has seen black since the last shortcut
        }
        self.default_shortcut_information = self.shortcut_information.copy()

    def settings_default(self):
        self.drivebase.settings(
            straight_speed=CONSTANTS["DEFAULT_SPEED"],
            straight_acceleration=CONSTANTS["DEFAULT_ACCELERATION"],
            turn_rate=CONSTANTS["DEFAULT_TURN_RATE"],
            turn_acceleration=CONSTANTS["DEFAULT_TURN_ACCELERATION"]
        )
    
    def battery_display(self):
        battery_voltage = self.hub.battery.voltage()

        print("Battery:", battery_voltage)
    
    def turn_in_degrees(self, degrees, wait=False):
        """Turn in degrees. Using a curve for line following."""
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_LINE_FOLLOW"], degrees, Stop.COAST, wait)

    def sharp_turn_in_degrees(self, degrees):
        """Perform a sharp turn in degrees."""
        self.drivebase.turn(degrees)

    def move_forward(self, distance, speed=None, wait=False):
        """Move forward in mm."""
        if speed is not None:
            self.set_speed(speed)
        self.drivebase.straight(distance, wait)

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

    def rotate_arm(self, degrees, stop_method=Stop.BRAKE):
        """Rotate the arm's motor based on degrees."""
        self.arm_motor.run_angle(CONSTANTS["ARM_MOVE_SPEED"], degrees, stop_method, False)

    def set_speed(self, speed):
        """Set the speed of the robot in drivebase settings."""
        if speed == 0:
            speed = CONSTANTS["DEFAULT_SPEED"]
        self.drivebase.settings(straight_speed=speed)

    def information_to_color(self, information):
        """Convert color sensor information to a color."""
        if information["reflection"] > 97: # Probably needs to be better
            return Color.GRAY
        elif information["color"] == Color.WHITE:
            return Color.WHITE
        elif information["hsv"].h < 67 and information["hsv"].h > 45 and ALLOW_YELLOW:
            return Color.YELLOW
        elif information["color"] == Color.GREEN and 135 < information["hsv"].h < 165:
            return Color.GREEN
        elif information["color"] == Color.BLUE and information["hsv"].s > 80 and information["hsv"].v > 80:
            return Color.BLUE
        elif information["color"] == Color.RED and information["hsv"].s > 85:
            return Color.RED
        elif information["color"] == Color.RED: # information["hsv"].s <= 85
            return Color.ORANGE
        elif information["hsv"].s < 25 and information["reflection"] < 30:
            return Color.BLACK
        else:
            return Color.NONE
            # Could try return Color.BLACK

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

    def follow_line(self):
        if not self.on_inverted:
            reflection_difference = self.left_color_sensor_information["reflection"] - self.right_color_sensor_information["reflection"]
        else:
            reflection_difference = self.right_color_sensor_information["reflection"] - self.left_color_sensor_information["reflection"]
        
        turn_rate = max(min(3.3 * reflection_difference, CONSTANTS["MAX_TURN_RATE"]), -CONSTANTS["MAX_TURN_RATE"])
        self.drivebase.drive(CONSTANTS["MOVE_SPEED"], turn_rate)

        while not self.drivebase.done(): # To check if both are black
            self.get_colors()
            if self.left_color == Color.BLACK and self.right_color == Color.BLACK:
                self.stop_motors()
                self.black_counter += 1
    
    def follow_color(self, color_to_follow=Color.YELLOW):
        if self.left_color == color_to_follow:
            self.turn_in_degrees(-CONSTANTS["TURN_YELLOW_DEGREES"])
        elif self.right_color == color_to_follow:
            self.turn_in_degrees(CONSTANTS["TURN_YELLOW_DEGREES"])
        else:
            self.move_forward(10, wait=False)

    def turn_and_detect_ultrasonic(self, degrees):
        "Turn the number of degrees, and meanwhile, find the lowest ultrasonic and return the lowest ultrasonic."
        lowest_ultrasonic = 2000
        lowest_ultrasonic_angle = 0

        self.drivebase.reset()
        self.sharp_turn_in_degrees(degrees)

        while not self.drivebase.done():
            new_ultrasonic = self.ultrasonic_sensor.distance()

            if new_ultrasonic < lowest_ultrasonic:
                lowest_ultrasonic = new_ultrasonic
                lowest_ultrasonic_angle = self.drivebase.angle()
        
        return lowest_ultrasonic, lowest_ultrasonic_angle
    
    def gray_ending(self):
        self.drivebase.settings(
            straight_speed=80,
            straight_acceleration=450,
            turn_rate=400,
            turn_acceleration=1600
        )

        self.move_forward(290)
        self.move_forward(-20)

        self.stop_motors()

        lowest_ultrasonic, lowest_ultrasonic_angle = self.turn_and_detect_ultrasonic(360)

        self.drivebase.reset()
        self.sharp_turn_in_degrees(lowest_ultrasonic_angle)
        self.move_forward(lowest_ultrasonic - 20)
        self.rotate_arm(180)

        self.sharp_turn_in_degrees(180)
        self.move_forward(lowest_ultrasonic - 20)

        self.sharp_turn_in_degrees(-lowest_ultrasonic_angle)
        
        self.move_forward(270)

        # Back at the gray here

        self.sharp_turn_in_degrees(45)
        self.rotate_arm(-180)
        self.sharp_turn_in_degrees(-45)

        self.settings_default()

        self.move_forward(10)

    def avoid_obstacle(self):
        self.stop_motors()
        self.rotate_arm(-90)
        self.sharp_turn_in_degrees(CONSTANTS["OBSTACLE_INITIAL_TURN_DEGREES"])
        self.set_speed(CONSTANTS["OBSTACLE_MOVE_SPEED"])
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_OBSTACLE"], -CONSTANTS["OBSTACLE_TURN_DEGREES"], Stop.BRAKE, True)
        self.start_motors(CONSTANTS["OBSTACLE_MOVE_SPEED"], CONSTANTS["OBSTACLE_MOVE_SPEED"])

        self.get_colors() # Re-read colors after turning
        while self.right_color == Color.WHITE: # Keep turning until right sensor sees something other than white
            self.get_colors()
        self.turn_in_degrees(CONSTANTS["OBSTACLE_FINAL_TURN_DEGREES"])
        self.robot_state = "straight" # Reset state after handling obstacle
        self.rotate_arm(90)
        self.set_speed(CONSTANTS["DEFAULT_SPEED"])
    
    def update(self):
        """Update the state of the robot."""
        self.get_colors()
        self.ultrasonic = self.ultrasonic_sensor.distance()

        self.previous_state = self.robot_state

        if (self.left_color == Color.GRAY and self.right_color == Color.GRAY) or (self.left_color == Color.GREEN and self.right_color == Color.GREEN):
            self.robot_state = "gray"
            return
        
        if self.left_color == Color.BLACK and self.right_color == Color.BLACK:
            self.black_counter += 1
        else:
            self.black_counter = 0
        
        if self.black_counter > CONSTANTS["BLACK_COUNTER_THRESHOLD"]:
            self.on_inverted = True
        if self.left_color == Color.WHITE and self.right_color == Color.WHITE:
            self.on_inverted = False
        
        # Check for obstacle
        if self.ultrasonic < CONSTANTS["ULTRASONIC_THRESHOLD"]:
            self.robot_state = "obstacle"
            return # Exit update early if obstacle detected

        # Shortcut / Yellow
        elif ALLOW_YELLOW and (self.left_color == Color.YELLOW or self.right_color == Color.YELLOW):
            self.robot_state = "yellow line"
            if not self.shortcut_information["is following shortcut"]:
                if self.left_color == Color.YELLOW:
                    self.turn_in_degrees(-90)

                    if self.shortcut_information["first turned"] == None:
                        self.shortcut_information["first turned"] = "left"

                if self.right_color == Color.YELLOW:
                    self.turn_in_degrees(90)

                    if self.shortcut_information["first turned"] == None:
                        self.shortcut_information["first turned"] = "right"
            
            self.shortcut_information["is following shortcut"] = True
        
        # Line
        elif not self.shortcut_information["is following shortcut"] and \
            self.left_color in [Color.WHITE, Color.BLACK, Color.GRAY] and \
            self.right_color in [Color.WHITE, Color.BLACK, Color.GRAY] and \
            not (self.left_color == self.right_color and self.left_color == Color.GRAY): # Both not gray
            self.robot_state = "line"

        # Turn / Green
        elif self.left_color == Color.GREEN:
            self.robot_state = "green left"
        elif self.right_color == Color.GREEN:
            self.robot_state = "green right"

        # Nothing
        else:
            self.robot_state = "line"
            # self.robot_state = "stop"

        # Stopping shortcut
        if self.left_color == Color.BLACK:
            self.shortcut_information["left seen black since"] = True
        if self.right_color == Color.BLACK:
            self.shortcut_information["right seen black since"] = True

        if self.shortcut_information["left seen black since"] and self.shortcut_information["right seen black since"]:
            if self.shortcut_information["first turned"] == "left":
                self.turn_in_degrees(90) # Turn right if it turned left for the shortcut
            else:
                self.turn_in_degrees(-90) # Turn left if it turned right for the shortcut

            self.robot_state = "line" # Go back to normal line following
            self.shortcut_information = self.default_shortcut_information.copy()

    def move(self):
        """Move the robot based on its current state."""
        # Gray
        if self.robot_state == "gray":
            self.gray_ending()
        
        # Obstacle
        elif self.robot_state == "obstacle":
            self.avoid_obstacle()

        # ShortcutYellow
        elif self.shortcut_information["is following shortcut"]:
            self.follow_color()
        
        # Line
        elif self.robot_state == "line":
            self.follow_line()

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
        # print(self.robot_state)
        # print(self.left_color_sensor_information, self.left_color)
        print(self.right_color_sensor_information, self.right_color)
        # print(self.left_color, self.right_color)
        # print(self.iteration_count)
        # print(self.ultrasonic)

    def run(self):
        self.battery_display()
        self.rotate_arm(180, Stop.COAST_SMART) # Reset
        while True:
            self.iteration_count += 1 # Increment the counter at the start of each loop
            self.update()
            self.move()
            self.debug()

def main():
    robot = Robot()
    robot.run()

main() # Note: don't put in if __name__ == "__main__" because __name__ is something different for robot