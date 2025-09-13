from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop
from pybricks.tools import StopWatch, wait

ALLOW_YELLOW = True

# --- CONSTANTS ---
CONSTANTS = {
    "DRIVEBASE_WHEEL_DIAMETER": 56,
    "DRIVEBASE_AXLE_TRACK": 112,
    "ARM_MOVE_SPEED": 200,
    "DEFAULT_SPEED": 80,
    "DEFAULT_ACCELERATION": 600,
    "DEFAULT_TURN_RATE": 60,
    "DEFAULT_TURN_ACCELERATION": 800,
    "OBSTACLE_MOVE_SPEED": 300,
    "MOVE_SPEED": 80,
    # Obstacle detection/avoidance (merged from old_code)
    "ULTRASONIC_THRESHOLD": 100,
    "OBSTACLE_DETECT_CONFIRM": 4,
    "BLACK_WHEEL_SPEED": 30,
    "TURN_GREEN_DEGREES": 50,
    "BACK_AFTER_GREEN_TURN_DISTANCE": 14,
    "TURN_YELLOW_DEGREES": 20,
    "CURVE_RADIUS_GREEN": 78,
    "CURVE_RADIUS_OBSTACLE": 180,
    "OBSTACLE_TURN_DEGREES": 175,
    "OBSTACLE_INITIAL_TURN_DEGREES": 90,
    "OBSTACLE_FINAL_TURN_DEGREES": 70,
    "OBSTACLE_ARM_RETURN_DELAY": 3000,
    # Obstacle line-jump tuning
    "OBSTACLE_LINE_CONFIRM": 2,
    "OBSTACLE_FORWARD_STEP_MM": 10,
    "OBSTACLE_FORWARD_MAX_MM": 180,
    "OBSTACLE_RIGHT_ALIGN_DEG": 90,
    "OBSTACLE_ENTRY_FORWARD_MM": 20,
    "CURVE_RADIUS_LINE_FOLLOW": 4,
    "MAX_TURN_RATE": 100,
    "BLACK_COUNTER_THRESHOLD": 1000,
    # Line follow + reacquire tuning used by obstacle logic
    "LINE_KP": 3.0,
    "LINE_KD": 18.0,
    "LINE_MIN_SPEED": 80,
    "LINE_MAX_TURN_RATE": 320,
    "CORNER_ERR_THRESHOLD": 13,
    "LINE_BLACK_REF_THRESHOLD": 39,
    "LINE_REACQUIRE_TURN_STEP_DEG": 10,
    "LINE_REACQUIRE_TURN_MAX_DEG": 120,
    "LINE_REACQUIRE_CONFIRM": 2,
    "TURNING_WITH_WEIGHT_CORRECITON_MULTIPLIER": 1,
    # Can edge-refinement sweep tuning
    "CAN_REFINE_SWEEP_HALF_DEG": 80,
    "CAN_EDGE_MARGIN_MM": 120,
    "CAN_EDGE_CONFIRM": 3,
    "CAN_SWEEP_TURN_RATE": 100,
    "CAN_SWEEP_TURN_ACCEL": 300,
    "PROPORTIONAL_GAIN": 5,
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
        self.clock = StopWatch()
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
            CONSTANTS["DRIVEBASE_AXLE_TRACK"],
        )
        self.drivebase.use_gyro(False)
        self.settings_default()

        self.robot_state = "obstacle" # Initial state
        self.iteration_count = 0 # Initialize the iteration counter
        self.black_counter = 0
        self.on_inverted = False
        self.move_arm_back_after_obstacle_time = None
        self._obstacle_below_count = 0
        self.has_sensed_green = False

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

    def sharp_turn_in_degrees(self, degrees, wait=True):
        """Perform a sharp turn in degrees."""
        self.drivebase.turn(degrees, wait=wait)

    def move_forward(self, distance, speed=None, wait=True):
        """Move forward in mm."""
        if speed is not None:
            self.set_speed(speed)
        self.drivebase.straight(distance, wait=wait)

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

    def rotate_arm(self, degrees, stop_method=Stop.BRAKE, wait=False):
        """Rotate the arm's motor based on degrees."""
        self.arm_motor.run_angle(CONSTANTS["ARM_MOVE_SPEED"], degrees, stop_method, wait=wait)

    def set_speed(self, speed):
        """Set the speed of the robot in drivebase settings."""
        if speed == 0:
            speed = CONSTANTS["DEFAULT_SPEED"]
        self.drivebase.settings(straight_speed=speed)

    def information_to_color(self, information):
        """Convert color sensor information to a color."""
        if information["reflection"] > 99: # Probably needs to be better
            return Color.GRAY
        elif information["color"] == Color.WHITE:
            return Color.WHITE
        elif information["hsv"].h < 67 and information["hsv"].h > 45 and ALLOW_YELLOW:
            return Color.YELLOW
        elif information["color"] == Color.GREEN and 135 < information["hsv"].h < 165 and information["hsv"].s > 30:
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

    def both_black_slow(self):
        self.move_forward(10) # Slow down a lot
    
    def turn_green(self, direction):
        """When there is a green on the left or the right, react to it by doing a larger turn left or right."""
        
        # Check if it actually green and not an error
        self.move_forward(10)
        self.get_colors()
        new_left_color = self.information_to_color(self.left_color_sensor_information)
        new_right_color = self.information_to_color(self.right_color_sensor_information)

        if not ((direction == "left" and new_left_color == Color.GREEN) or (direction == "right" and new_right_color == Color.GREEN)):
            return
        elif (new_left_color == new_right_color == Color.GREEN):
            self.drivebase.stop()
            self.green_spill_ending()
            return

        if direction == "left":
            degrees = -CONSTANTS["TURN_GREEN_DEGREES"] 
        else:
            degrees = CONSTANTS["TURN_GREEN_DEGREES"]
        
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_GREEN"], degrees, Stop.COAST, True)

        stop = False
        while not self.drivebase.done():
            self.get_colors()
            if self.left_color == Color.GREEN and self.right_color == Color.GREEN:
                self.drivebase.stop()
                self.green_spill_ending()
                stop = True
                break
        
        if not stop:
            self.move_forward(-CONSTANTS["BACK_AFTER_GREEN_TURN_DISTANCE"])

    def follow_line(self):
        if not self.on_inverted:
            reflection_difference = (self.left_color_sensor_information["reflection"] + 8) - self.right_color_sensor_information["reflection"]
        else:
            reflection_difference = self.right_color_sensor_information["reflection"] - self.left_color_sensor_information["reflection"]
        
        turn_rate = max(min(CONSTANTS["PROPORTIONAL_GAIN"] * reflection_difference, CONSTANTS["MAX_TURN_RATE"]), -CONSTANTS["MAX_TURN_RATE"])
        self.drivebase.drive(CONSTANTS["MOVE_SPEED"], turn_rate)

        while not self.drivebase.done(): # To check if both are black
            self.get_colors()
            if self.left_color == Color.BLACK and self.right_color == Color.BLACK:
                self.both_black_slow()
                self.black_counter += 1
    
    def follow_color(self, color_to_follow=Color.YELLOW):
        if self.left_color == color_to_follow:
            self.turn_in_degrees(-CONSTANTS["TURN_YELLOW_DEGREES"])
        elif self.right_color == color_to_follow:
            self.turn_in_degrees(CONSTANTS["TURN_YELLOW_DEGREES"])
        else:
            self.move_forward(10, wait=False)

    def turn_and_detect_ultrasonic(self, degrees=360):
        "Turn the number of degrees, and meanwhile, find the lowest ultrasonic and return the lowest ultrasonic."
        lowest_ultrasonic = 2000
        lowest_ultrasonic_angle = 0

        self.drivebase.reset()
        self.sharp_turn_in_degrees(degrees, wait=False)

        while not self.drivebase.done():
            new_ultrasonic = self.ultrasonic_sensor.distance()

            if new_ultrasonic < lowest_ultrasonic:
                lowest_ultrasonic = new_ultrasonic
                lowest_ultrasonic_angle = self.drivebase.angle()

            print(new_ultrasonic, self.drivebase.angle())
        return lowest_ultrasonic, lowest_ultrasonic_angle

    def read_ultra_mm_obstacle(self):
        """Median-filtered ultrasonic for obstacle detection (no near/far rejection)."""
        samples = []
        for _ in range(3):
            d = self.ultrasonic_sensor.distance()
            if d is None:
                d = 9999
            samples.append(d)
            wait(2)
        samples.sort()
        return samples[1]
    
    def green_spill_ending(self):
        self.move_forward(20)
        if self.has_sensed_green: # Only run the green spill ending once
            return

        self.has_sensed_green = True
        self.move_forward(10)
        self.get_colors()
        if self.left_color != Color.GREEN or self.right_color != Color.GREEN:
            self.move_forward(-10)
            return

        self.drivebase.settings( # Slow down
            straight_speed=80,
            straight_acceleration=450,
            turn_rate=140,
            turn_acceleration=1600
        )

        self.rotate_arm(-86, stop_method=Stop.HOLD) # Arm up

        self.move_forward(280) # Go to middle - 10 (because already moved forward 10)
        self.move_forward(-20) # Go back just in case hit the can
        # self.hub.imu.reset_heading(0)

        self.stop_motors() # Stop

        lowest_ultrasonic = 2000
        while lowest_ultrasonic == 2000:
            lowest_ultrasonic, lowest_ultrasonic_angle = self.turn_and_detect_ultrasonic() # Turn 360 degrees, find the lowest ultrasonic and angle

        # if lowest_ultrasonic_angle > 180:
        #     lowest_ultrasonic_angle -= 360

        # self.drivebase.reset()

        # Refine target by sweeping to find left/right edges and aim at midpoint
        sweep_half = int(CONSTANTS.get("CAN_REFINE_SWEEP_HALF_DEG", 80))
        edge_margin = int(CONSTANTS.get("CAN_EDGE_MARGIN_MM", 120))
        edge_threshold = lowest_ultrasonic + edge_margin
        # Save/adjust turn settings for smoother sweep
        try:
            ss, sa, prev_tr, prev_ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
            prev_ta = None
        try:
            self.drivebase.settings(
                turn_rate=int(CONSTANTS.get("CAN_SWEEP_TURN_RATE", 100)),
                turn_acceleration=int(CONSTANTS.get("CAN_SWEEP_TURN_ACCEL", 300)),
            )
        except Exception:
            pass

        # Move to left of the detected sector and sweep right across
        self.sharp_turn_in_degrees(lowest_ultrasonic_angle - sweep_half, wait=True)
        self.drivebase.reset()
        self.sharp_turn_in_degrees(2 * sweep_half, wait=False)

        saw_inside = False
        first_enter = None
        last_seen = None
        lost = 0
        confirm = int(CONSTANTS.get("CAN_EDGE_CONFIRM", 3))
        while not self.drivebase.done():
            d = self.ultrasonic_sensor.distance()
            if d is None:
                d = 9999
            a = self.drivebase.angle()
            if d <= edge_threshold:
                if not saw_inside:
                    saw_inside = True
                    first_enter = a
                last_seen = a
                lost = 0
            else:
                if saw_inside:
                    lost += 1
                    if lost >= confirm and last_seen is not None:
                        break
        self.drivebase.stop()

        # Restore previous turn settings
        try:
            if prev_tr is not None and prev_ta is not None:
                self.drivebase.settings(turn_rate=prev_tr, turn_acceleration=prev_ta)
            elif prev_tr is not None:
                self.drivebase.settings(turn_rate=prev_tr)
        except Exception:
            pass

        if first_enter is not None and last_seen is not None:
            target_angle_rel = (first_enter + last_seen) / 2.0
            # Rotate from current sweep end to the midpoint
            delta = target_angle_rel - self.drivebase.angle()
            self.sharp_turn_in_degrees(delta)
        else:
            # Fall back to the raw first-hit angle
            self.sharp_turn_in_degrees(lowest_ultrasonic_angle)

        # Approach using fresh distance at the refined heading
        fresh_d = self.ultrasonic_sensor.distance()
        if fresh_d is None:
            fresh_d = lowest_ultrasonic
        approach = max(0, min(260, int(fresh_d) - 20))
        self.move_forward(approach)
        self.rotate_arm(-95, stop_method=Stop.COAST, wait=True) # Arm down, capture the can

        self.sharp_turn_in_degrees(180)
        self.move_forward(min(lowest_ultrasonic - 20, 260))

        # self.move_forward(max(-(lowest_ultrasonic - 20), -260)) # Go back to middle

        return_to_exit_angle = -lowest_ultrasonic_angle
        # return_to_exit_angle = -self.hub.imu.heading() + 180
        # if return_to_exit_angle > 180:
        #     return_to_exit_angle -= 360

        self.sharp_turn_in_degrees(return_to_exit_angle * CONSTANTS["TURNING_WITH_WEIGHT_CORRECITON_MULTIPLIER"]) # Turn back, and face exit
        self.move_forward(270) # Go to exit

        away_can_angle = 50
        away_can_distance = 58
        self.sharp_turn_in_degrees(away_can_angle)
        self.move_forward(away_can_distance) # Go forward a bit so the can is not on the path
        self.rotate_arm(180) # Arm up, release the can
        self.move_forward(-away_can_distance) # Go back
        self.sharp_turn_in_degrees(-away_can_angle) # Turn back

        self.settings_default() # Reset speed and settings

        self.move_forward(30) # Hopefully sense the black line again

    def reacquire_line_oscillate(self, step_deg=10, max_deg=120, confirm=2):
        def confirm_line(n):
            count = 0
            for _ in range(max(1, n)):
                self.get_colors()
                if self.left_color == Color.BLACK or self.right_color == Color.BLACK:
                    count += 1
                else:
                    count = 0
                if count >= n:
                    return True
                wait(5)
            return False

        current_offset = 0
        if confirm_line(confirm):
            return True

        amp = step_deg
        while amp <= max_deg:
            delta = -amp - current_offset
            if delta != 0:
                self.sharp_turn_in_degrees(delta)
                current_offset += delta
            if confirm_line(confirm):
                return True

            delta = amp - current_offset
            if delta != 0:
                self.sharp_turn_in_degrees(delta)
                current_offset += delta
            if confirm_line(confirm):
                return True

            amp += step_deg

        return False

    def align_to_line_in_place(self, timeout_ms=1500, err_tol=3):
        start = self.clock.time()
        prev_err = 0
        kp = CONSTANTS.get("LINE_KP", 3.2)
        kd = 0.0
        max_turn = CONSTANTS.get("LINE_MAX_TURN_RATE", 320)
        while self.clock.time() - start < timeout_ms:
            self.get_colors()
            left_ref = self.left_color_sensor_information["reflection"]
            right_ref = self.right_color_sensor_information["reflection"]
            error = right_ref - left_ref if self.on_inverted else left_ref - right_ref
            d_err = error - prev_err
            prev_err = error
            if abs(error) <= err_tol:
                break
            turn_rate = kp * error + kd * d_err
            if turn_rate > max_turn:
                turn_rate = max_turn
            elif turn_rate < -max_turn:
                turn_rate = -max_turn
            self.drivebase.drive(0, turn_rate)
            wait(10)
        self.drivebase.stop()

    def avoid_obstacle(self):
        # Modified: arc around obstacle, but proactively rejoin the line by
        # driving until black is detected, then turning RIGHT onto the line.
        self.stop_motors()
        self.rotate_arm(-90)

        # Initial sidestep turn
        self.sharp_turn_in_degrees(CONSTANTS["OBSTACLE_INITIAL_TURN_DEGREES"])

        # Start arc without blocking so we can watch sensors
        self.drivebase.curve(
            CONSTANTS["CURVE_RADIUS_OBSTACLE"],
            -CONSTANTS["OBSTACLE_TURN_DEGREES"],
            Stop.COAST,
            False,
        )

        # Monitor for encountering the line (black) before the arc completes
        confirm_need = int(CONSTANTS.get("OBSTACLE_LINE_CONFIRM", 2))
        confirm = 0
        line_hit = False
        while not self.drivebase.done():
            self.get_colors()
            if self.left_color == Color.BLACK or self.right_color == Color.BLACK:
                confirm += 1
                if confirm >= confirm_need:
                    line_hit = True
                    break
            else:
                confirm = 0
            wait(10)

        self.drivebase.stop()

        if line_hit:
            # On first sight of black: drive a little forward, then turn RIGHT and resume line following
            entry_fwd = int(CONSTANTS.get("OBSTACLE_ENTRY_FORWARD_MM", 20))
            if entry_fwd > 0:
                self.move_forward(entry_fwd)
            self.sharp_turn_in_degrees(abs(int(CONSTANTS.get("OBSTACLE_RIGHT_ALIGN_DEG", 90))), wait=True)
            # No sweeping/align; immediately resume line following in that direction
            self.robot_state = "line"
            self.move_arm_back_after_obstacle_time = self.clock.time() + CONSTANTS["OBSTACLE_ARM_RETURN_DELAY"]
            return

        # Fallback: if we never detected black during the arc, use oscillating reacquire
        found = self.reacquire_line_oscillate(
            step_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_STEP_DEG", 10),
            max_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_MAX_DEG", 120),
            confirm=CONSTANTS.get("LINE_REACQUIRE_CONFIRM", 2),
        )
        if not found:
            self.turn_in_degrees(CONSTANTS["OBSTACLE_FINAL_TURN_DEGREES"])
            found = self.reacquire_line_oscillate(
                step_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_STEP_DEG", 10),
                max_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_MAX_DEG", 120),
                confirm=CONSTANTS.get("LINE_REACQUIRE_CONFIRM", 2),
            )
        if found:
            self.align_to_line_in_place(timeout_ms=1500, err_tol=3)

        self.robot_state = "line"
        self.move_arm_back_after_obstacle_time = self.clock.time() + CONSTANTS["OBSTACLE_ARM_RETURN_DELAY"]
    
    def update(self):
        """Update the state of the robot."""
        self.get_colors()
        self.ultrasonic = self.read_ultra_mm_obstacle()

        self.previous_state = self.robot_state

        # Arm return timer after obstacle handling
        if self.move_arm_back_after_obstacle_time is not None and self.clock.time() >= self.move_arm_back_after_obstacle_time:
            self.move_arm_back_after_obstacle_time = None
            self.rotate_arm(90, stop_method=Stop.COAST)

        if (self.left_color == Color.GRAY and self.right_color == Color.GRAY) or (self.left_color == Color.GREEN and self.right_color == Color.GREEN):
            self.robot_state = "gray"
            return
        
        if self.left_color == Color.BLACK and self.right_color == Color.BLACK:
            self.black_counter += 1
            self.both_black_slow()
        else:
            self.black_counter = 0
        
        if self.black_counter > CONSTANTS["BLACK_COUNTER_THRESHOLD"]:
            self.on_inverted = True
        if self.left_color == Color.WHITE and self.right_color == Color.WHITE:
            self.on_inverted = False
        
        # Check for obstacle using median-filtered reads and confirmation
        if self.ultrasonic is not None and self.ultrasonic < CONSTANTS["ULTRASONIC_THRESHOLD"]:
            self._obstacle_below_count += 1
        else:
            self._obstacle_below_count = 0
        if self._obstacle_below_count >= CONSTANTS.get("OBSTACLE_DETECT_CONFIRM", 2):
            self._obstacle_below_count = 0
            self.robot_state = "obstacle"
            return

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
            self.green_spill_ending()
        
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
        # print(self.right_color_sensor_information, self.right_color)
        # print(self.left_color, self.right_color)
        # print(self.iteration_count)
        # print(self.ultrasonic)
        pass

    def run(self):
        self.battery_display()
        self.rotate_arm(180, Stop.COAST_SMART) # Reset
        while True:
            self.iteration_count += 1 # Increment the counter at the start of each loop
            self.update()
            self.move()
            # self.debug()

def main():
    robot = Robot()
    robot.run()

main() # Note: don't put in if __name__ == "__main__" because __name__ is something different for robot
