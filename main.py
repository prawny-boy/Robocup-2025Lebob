from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop
from pybricks.tools import StopWatch, wait

# Enable/disable optional yellow-line shortcut behavior
ALLOW_YELLOW = True

# --- CONSTANTS ---
CONSTANTS = {
    "DRIVEBASE_WHEEL_DIAMETER": 56,
    "DRIVEBASE_AXLE_TRACK": 112,
    "ARM_MOVE_SPEED": 200,
    "DEFAULT_SPEED": 80,
    "DEFAULT_ACCELERATION": 600,
    "DEFAULT_TURN_RATE": 100,
    "DEFAULT_TURN_ACCELERATION": 1000,
    "OBSTACLE_MOVE_SPEED": 300,
    "MOVE_SPEED": 80,
    "ULTRASONIC_THRESHOLD": 50,
    "BLACK_WHEEL_SPEED": 30,
    # Optional: direct-drive trim to correct mechanical bias (1.00 = no change)
    "LEFT_MOTOR_TRIM": 1.00,
    "RIGHT_MOTOR_TRIM": 1.00,
    "TURN_GREEN_DEGREES": 50,
    "BACK_AFTER_GREEN_TURN_DISTANCE": 60,
    "TURN_YELLOW_DEGREES": 20,
    "CURVE_RADIUS_GREEN": 78,
    "CURVE_RADIUS_OBSTACLE": 160,
    "OBSTACLE_TURN_DEGREES": 175,
    "OBSTACLE_INITIAL_TURN_DEGREES": 90,
    "OBSTACLE_FINAL_TURN_DEGREES": 70,
    "OBSTACLE_ARM_RETURN_DELAY": 3000,
    "CURVE_RADIUS_LINE_FOLLOW": 4,
    "MAX_TURN_RATE": 100,
    "BLACK_COUNTER_THRESHOLD": 1000,
    "TURNING_WITH_WEIGHT_CORRECITON_MULTIPLIER": 1,
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
            CONSTANTS["DRIVEBASE_AXLE_TRACK"]
        )
        # Enable gyro-based correction for straight driving and turns
        self.gyro_ok = False
        if CONSTANTS.get("GYRO_ENABLED", True):
            try:
                wait(int(CONSTANTS.get("IMU_STABILIZE_MS", 800)))
                try:
                    self.hub.imu.reset_heading(0)
                except Exception:
                    pass
                self.drivebase.use_gyro(True)
                self.gyro_ok = True
            except Exception:
                try:
                    self.drivebase.use_gyro(False)
                except Exception:
                    pass
                self.gyro_ok = False
        else:
            try:
                self.drivebase.use_gyro(False)
            except Exception:
                pass
        self.settings_default()

        self.robot_state = "obstacle"  # Initial state
        self.iteration_count = 0
        self.black_counter = 0
        self.on_inverted = False
        self.move_arm_back_after_obstacle_time = None
        self.debug_enabled = False
        self.prev_error = 0
        self.corner_hold = 0
        self.line_pivoting = False
        self._obstacle_below_count = 0
        self.d_err_filtered = 0.0
        self.corner_until_ms = 0
        self.prev_left_is_black = False
        self.prev_right_is_black = False
        self.black_slow_until_ms = 0
        self.yellow_last_seen_side = 0   # -1=left, +1=right, 0=unknown
        self.yellow_black_confirm = 0
        self.y_l_score = 0.0
        self.y_r_score = 0.0

        self.shortcut_information = {
            "is following shortcut": False,
            "first turned": None,
            "left seen black since": False,
            "right seen black since": False
        }
        self.default_shortcut_information = self.shortcut_information.copy()

        self.can_recording = False
        self.can_path = []
        self.can_sweep_slow_mode = False
        self.spill_entry_heading = None
        self._green_left_count = 0
        self._green_right_count = 0
        self._spill_cooldown_until = 0
        self._spill_retry_backoff_ms = CONSTANTS.get("SPILL_RETRY_COOLDOWN_MS", 1200)

    # --- helpers ---
    def drive_with_bias(self, speed_mm_s, turn_deg_s):
        """All driving goes through this so global bias always applies."""
        bias = float(CONSTANTS.get("BASE_TURN_BIAS", 0.0))
        self.drivebase.drive(int(speed_mm_s), float(turn_deg_s) + bias)

    def read_ultra_mm(self):
        """Raw ultrasonic distance in mm; returns 9999 if no valid target."""
        d = self.ultrasonic_sensor.distance()
        return 9999 if d is None else d

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

    def read_ultra_mm_can(self):
        """Ultrasonic read with median filter and bounds for can logic."""
        samples = []
        for _ in range(CONSTANTS.get("CAN_US_MEDIAN_SAMPLES", 3)):
            d = self.ultrasonic_sensor.distance()
            if d is None:
                d = 9999
            samples.append(d)
            wait(5)
        samples.sort()
        m = samples[len(samples)//2]
        if m < CONSTANTS.get("CAN_MIN_VALID_MM", 60):
            return 9999
        return 9999 if m > CONSTANTS.get("CAN_IGNORE_ABOVE_MM", 1800) else m

    def settings_default(self):
        self.drivebase.settings(
            straight_speed=CONSTANTS["DEFAULT_SPEED"],
            straight_acceleration=CONSTANTS["DEFAULT_ACCELERATION"],
            turn_rate=CONSTANTS["DEFAULT_TURN_RATE"],
            turn_acceleration=CONSTANTS["DEFAULT_TURN_ACCELERATION"]
        )

    def log(self, *msg):
        if self.debug_enabled:
            print(*msg)

    # --- Battery helpers ---
    def battery_percent(self):
        try:
            mv = self.hub.battery.voltage()
        except Exception:
            mv = None
        if mv is None:
            return None, None
        v_min = CONSTANTS.get("BATTERY_V_MIN_MV", 6200)
        v_max = CONSTANTS.get("BATTERY_V_MAX_MV", 8400)
        mv = max(0, mv)
        pct = int(max(0, min(100, round((mv - v_min) * 100 / max(1, (v_max - v_min))))))
        return pct, mv / 1000.0

    def can_log(self, *msg):
        if CONSTANTS.get("CAN_DEBUG_PRINT", False):
            print(*msg)

    def battery_display(self):
        pct, volts = self.battery_percent()
        if pct is None:
            print("Battery: unknown")
            return
        warn = " (LOW)" if pct < CONSTANTS.get("BATTERY_WARN_PERCENT", 75) else ""
        print("Battery:", f"{pct}%", f"({volts:.2f} V){warn}")

    def turn_in_degrees(self, degrees, wait=False):
        """Turn by a small angle using a gentle curve."""
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_LINE_FOLLOW"], degrees, Stop.COAST, wait)

    def sharp_turn_in_degrees(self, degrees, wait=True):
        """Perform an in-place sharp turn by the given angle in degrees."""
        self.drivebase.turn(degrees, wait=wait)

    def move_forward(self, distance, speed=None, wait=True):
        """Move straight by the given distance in mm."""
        if speed is not None:
            self.set_speed(speed)
        trim = float(CONSTANTS.get("STRAIGHT_TURN_TRIM", 0.0))
        # If no trim requested and gyro is active, use built-in straight
        if abs(trim) < 1e-3 and getattr(self, "gyro_ok", False):
            self.drivebase.straight(distance, wait=wait)
            return
        # Otherwise, emulate a straight with constant turn trim to compensate bias
        try:
            self.drivebase.reset()
        except Exception:
            pass
        target = float(distance)
        direction = 1 if target >= 0 else -1
        try:
            ss, _, _, _ = self.drivebase.settings()
        except Exception:
            ss = int(CONSTANTS.get("DEFAULT_SPEED", 150))
        mm_s = int(ss if speed is None else speed) * direction
        self.drive_with_bias(mm_s, trim)
        if wait:
            while True:
                try:
                    d = self.drivebase.distance()
                except Exception:
                    d = 0.0
                if (direction > 0 and d >= target) or (direction < 0 and d <= target):
                    break
                try:
                    wait(10)
                except Exception:
                    pass
            self.drivebase.stop()

    # --- Sounds ---
    def announce_state(self, state):
        if not CONSTANTS.get("SOUNDS_ENABLED", True):
            return
        sp = self.hub.speaker
        try:
            if state == "line":
                sp.beep(700, 60)
            elif state == "obstacle":
                sp.beep(350, 90); wait(40); sp.beep(350, 90)
            elif state == "gray":
                sp.beep(900, 80); wait(40); sp.beep(1100, 80)
            elif state == "yellow line":
                sp.beep(1200, 60); wait(30); sp.beep(1200, 60); wait(30); sp.beep(1200, 60)
            elif state == "green left":
                sp.beep(1000, 70)
            elif state == "green right":
                sp.beep(600, 70)
            elif state == "stop":
                sp.beep(250, 200)
            else:
                sp.beep(500, 50)
        except Exception:
            pass

    # --- Recording helpers for can routine ---
    def can_start_recording(self):
        self.can_path = []
        self.can_recording = True

    def can_cancel_recording(self):
        self.can_path = []
        self.can_recording = False

    def normalize_angle(self, angle):
        return ((angle + 180) % 360) - 180

    def face_heading(self, target_heading_deg):
        try:
            current = self.hub.imu.heading()
        except Exception:
            current = 0
        delta = self.normalize_angle(target_heading_deg - current)
        if self.can_recording:
            self.can_turn(delta)
        else:
            self.sharp_turn_in_degrees(delta)

    def can_move_forward(self, distance, speed=None, wait=True):
        if self.can_recording:
            self.can_path.append(("move", distance))
        self.move_forward(distance, speed=speed, wait=wait)

    def can_turn(self, degrees, wait=True):
        if self.can_recording:
            self.can_path.append(("turn", degrees))
        self.sharp_turn_in_degrees(degrees, wait=wait)

    def can_backtrack(self):
        for action, value in reversed(self.can_path):
            if action == "move":
                self.move_forward(-value)
            elif action == "turn":
                self.sharp_turn_in_degrees(-value)
        self.can_cancel_recording()

    def start_motors(self, left_speed, right_speed):
        lt = float(CONSTANTS.get("LEFT_MOTOR_TRIM", 1.0))
        rt = float(CONSTANTS.get("RIGHT_MOTOR_TRIM", 1.0))
        ls = int(left_speed * lt)
        rs = int(right_speed * rt)
        if self.left_drive.speed() != ls:
            self.left_drive.run(ls)
        if self.right_drive.speed() != rs:
            self.right_drive.run(rs)

    def stop_motors(self):
        self.left_drive.stop()
        self.right_drive.stop()

    def rotate_arm(self, degrees, stop_method=Stop.BRAKE, wait=False):
        self.arm_motor.run_angle(CONSTANTS["ARM_MOVE_SPEED"], degrees, stop_method, wait=wait)

    def set_speed(self, speed):
        if speed == 0:
            speed = CONSTANTS["DEFAULT_SPEED"]
        self.drivebase.settings(straight_speed=speed)

    def information_to_color(self, information):
        gray_ref_min = int(CONSTANTS.get("GRAY_REFLECTION_MIN", 99))
        if information["reflection"] > gray_ref_min:
            return Color.GRAY
        elif information["color"] == Color.WHITE:
            return Color.WHITE
        elif (
            ALLOW_YELLOW
            and CONSTANTS.get("YELLOW_HUE_MIN") is not None
            and CONSTANTS.get("YELLOW_HUE_MAX") is not None
            and CONSTANTS.get("YELLOW_HUE_MIN") < information["hsv"].h < CONSTANTS.get("YELLOW_HUE_MAX")
        ):
            return Color.YELLOW
        elif (
            information["color"] == Color.GREEN
            and CONSTANTS.get("GREEN_HUE_MIN", 135) < information["hsv"].h < CONSTANTS.get("GREEN_HUE_MAX", 165)
            and information["hsv"].s > CONSTANTS.get("GREEN_S_MIN", 30)
        ):
            return Color.GREEN
        elif information["color"] == Color.BLUE and information["hsv"].s > 80 and information["hsv"].v > 80:
            return Color.BLUE
        elif information["color"] == Color.RED and information["hsv"].s > 85:
            return Color.RED
        elif information["color"] == Color.RED:
            return Color.ORANGE
        elif information["hsv"].s < 25 and information["reflection"] < int(CONSTANTS.get("LINE_BLACK_REF_THRESHOLD", 30)):
            return Color.BLACK
        else:
            return Color.NONE

    def yellow_score(self, information):
        hsv = information.get("hsv")
        if hsv is None:
            return 0.0
        try:
            h = float(hsv.h)
            s = float(hsv.s)
            v = float(hsv.v)
        except Exception:
            return 0.0
        h_center = float(CONSTANTS.get("YELLOW_HUE_CENTER", 55.0))
        if "YELLOW_HUE_WIDTH" in CONSTANTS:
            h_width = float(CONSTANTS.get("YELLOW_HUE_WIDTH", 22.0))
        else:
            y_min = float(CONSTANTS.get("YELLOW_HUE_MIN", 45.0))
            y_max = float(CONSTANTS.get("YELLOW_HUE_MAX", 67.0))
            h_width = max(1.0, (y_max - y_min) / 2.0)
            if "YELLOW_HUE_CENTER" not in CONSTANTS:
                h_center = (y_min + y_max) / 2.0
        h_score = max(0.0, 1.0 - abs(h - h_center) / max(1.0, h_width))
        s_score = max(0.0, min(1.0, s / 100.0))
        v_score = max(0.0, min(1.0, (v - 25.0) / 75.0))
        return h_score * (0.6 + 0.3 * s_score + 0.1 * v_score)

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

    def both_black_slow(self):
        self.move_forward(10) # Slow down a lot
    
    def turn_green(self, direction):
        self.move_forward(8)

        self.get_colors()
        if (
            (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
            or (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
        ):
            self.drivebase.stop()
            self.green_spill_ending()
            return

        degrees = -CONSTANTS["TURN_GREEN_DEGREES"] if direction == "left" else CONSTANTS["TURN_GREEN_DEGREES"]
        self.drivebase.curve(CONSTANTS["CURVE_RADIUS_GREEN"], degrees, Stop.COAST, False)

        entered = False
        confirm_needed = CONSTANTS.get("SPILL_ENTRY_CONFIRM", 2)
        confirm = 0
        while not self.drivebase.done():
            self.get_colors()
            both_green = (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
            both_gray = (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
            if both_green or (CONSTANTS.get("TREAT_GRAY_AS_SPILL", False) and both_gray):
                confirm += 1
                if confirm >= confirm_needed:
                    self.drivebase.stop()
                    self.green_spill_ending()
                    entered = True
                    break
            else:
                confirm = 0

        if not entered:
            self.move_forward(-CONSTANTS["BACK_AFTER_GREEN_TURN_DISTANCE"])
            self.robot_state = "line"
            now = self.clock.time()
            self._spill_cooldown_until = now + self._spill_retry_backoff_ms
            self._spill_retry_backoff_ms = min(5000, int(self._spill_retry_backoff_ms * 2))

    def follow_line(self):
        left_ref = self.left_color_sensor_information["reflection"]
        right_ref = self.right_color_sensor_information["reflection"]

        error = right_ref - left_ref if self.on_inverted else left_ref - right_ref

        kp = CONSTANTS["LINE_KP"]
        kd = CONSTANTS["LINE_KD"]
        d_err = error - self.prev_error
        self.prev_error = error
        turn_rate = kp * error + kd * d_err

        max_turn = CONSTANTS.get("LINE_MAX_TURN_RATE", 320)
        if turn_rate > max_turn:
            turn_rate = max_turn
        elif turn_rate < -max_turn:
            turn_rate = -max_turn

        err_scale = min(1.0, abs(error) / max(1, CONSTANTS["CORNER_ERR_THRESHOLD"]))
        base_speed = CONSTANTS["MOVE_SPEED"]
        speed = max(CONSTANTS["LINE_MIN_SPEED"], int(base_speed * (1 - 0.4 * err_scale)))

        self.drive_with_bias(speed, turn_rate)

    def follow_yellow(self):
        y_left = (self.left_color == Color.YELLOW)
        y_right = (self.right_color == Color.YELLOW)
        if y_left and not y_right:
            self.yellow_last_seen_side = -1
        elif y_right and not y_left:
            self.yellow_last_seen_side = +1

        raw_y_l = self.yellow_score(self.left_color_sensor_information)
        raw_y_r = self.yellow_score(self.right_color_sensor_information)
        error = (raw_y_r - raw_y_l) * float(CONSTANTS.get("YELLOW_ERR_SCALE", 50))

        now = self.clock.time()
        if not hasattr(self, "y_pid_last_ms"):
            self.y_pid_last_ms = 0
            self.y_prev_error = 0.0
            self.y_error_integral = 0.0
            self.y_d_err_filtered = 0.0
        dt = 0.01 if self.y_pid_last_ms == 0 else max(0.001, (now - self.y_pid_last_ms) / 1000.0)
        self.y_pid_last_ms = now

        kp = float(CONSTANTS.get("YELLOW_KP", CONSTANTS.get("LINE_KP", 3.2)))
        ki = float(CONSTANTS.get("YELLOW_KI", CONSTANTS.get("LINE_KI", 0.02)))
        kd = float(CONSTANTS.get("YELLOW_KD", CONSTANTS.get("LINE_KD", 18.0)))

        self.y_error_integral += error * dt
        i_max = float(CONSTANTS.get("LINE_I_MAX", 80.0))
        if self.y_error_integral > i_max:
            self.y_error_integral = i_max
        elif self.y_error_integral < -i_max:
            self.y_error_integral = -i_max

        raw_d = (error - self.y_prev_error) / dt
        self.y_prev_error = error
        alpha = float(CONSTANTS.get("YELLOW_DERIV_ALPHA", 0.2))
        self.y_d_err_filtered = alpha * raw_d + (1.0 - alpha) * self.y_d_err_filtered

        turn = kp * error + ki * self.y_error_integral + kd * self.y_d_err_filtered

        max_turn = int(CONSTANTS.get("YELLOW_MAX_TURN_RATE", CONSTANTS.get("LINE_MAX_TURN_RATE", 320)))
        if turn > max_turn:
            turn = max_turn
        elif turn < -max_turn:
            turn = -max_turn

        base = int(CONSTANTS.get("YELLOW_BASE_SPEED", CONSTANTS.get("MOVE_SPEED", 170)))
        min_s = int(CONSTANTS.get("YELLOW_MIN_SPEED", CONSTANTS.get("LINE_MIN_SPEED", 80)))
        if raw_y_l < 0.1 and raw_y_r < 0.1:
            if getattr(self, "yellow_last_seen_side", 0) != 0 and abs(turn) < max_turn * 0.4:
                turn = self.yellow_last_seen_side * max(60, int(max_turn * 0.25))
            speed = max(min_s, int(base * 0.5))
        else:
            reflection_difference = self.right_color_sensor_information["reflection"] - self.left_color_sensor_information["reflection"]
        
        turn_rate = max(min(4.2 * reflection_difference, CONSTANTS["MAX_TURN_RATE"]), -CONSTANTS["MAX_TURN_RATE"])
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
            first, second, final = +deg, -deg, +deg
        self.sharp_turn_in_degrees(first)
        self.move_forward(step)
        self.sharp_turn_in_degrees(second)
        self.move_forward(step)
        self.sharp_turn_in_degrees(final)
        try:
            self.align_to_line_in_place(timeout_ms=1200, err_tol=4)
        except Exception:
            pass
        self.shortcut_information["is following shortcut"] = False
        self.robot_state = "line"

    def turn_and_detect_ultrasonic(self, degrees=360):
        lowest_ultrasonic = 9999
        lowest_ultrasonic_angle = 0
        self.drivebase.reset()
        try:
            ss, sa, prev_tr, ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
        sweep_rate = (
            CONSTANTS["CAN_SWEEP_TURN_RATE_SLOW"]
            if getattr(self, "can_sweep_slow_mode", False)
            else CONSTANTS["CAN_SWEEP_TURN_RATE"]
        )
        self.drivebase.settings(turn_rate=sweep_rate)
        self.sharp_turn_in_degrees(degrees, wait=False)

        while not self.drivebase.done():
            new_ultrasonic = self.ultrasonic_sensor.distance()

            if new_ultrasonic < lowest_ultrasonic:
                lowest_ultrasonic = new_ultrasonic
                lowest_ultrasonic_angle = self.drivebase.angle()

            print(new_ultrasonic, self.drivebase.angle())
        return lowest_ultrasonic, lowest_ultrasonic_angle
    
    def green_spill_ending(self):
        self._spill_retry_backoff_ms = CONSTANTS.get("SPILL_RETRY_COOLDOWN_MS", 1200)
        self.can_start_recording()
        try:
            self.spill_entry_heading = self.hub.imu.heading()
        except Exception:
            self.spill_entry_heading = None
        self.can_move_forward(10)
        self.get_colors()
        in_spill = (
            (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
            or (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
        )
        if not in_spill:
            self.can_backtrack()
            return

        self.drivebase.settings(
            straight_speed=80,
            straight_acceleration=450,
            turn_rate=140,
            turn_acceleration=1600
        )

        self.rotate_arm(-86, stop_method=Stop.HOLD)  # Arm up

        self.can_move_forward(280)
        self.can_move_forward(-20)
        self.stop_motors()

        mid_angle, min_dist = self.find_can_edges_midpoint(
            sweep_deg=160, threshold=CONSTANTS["CAN_SCAN_MAX_MM"]
        )
        if mid_angle is None:
            self.settings_default()
            return

        self.approach_can_at_angle(mid_angle, stop_offset_mm=20)

        self.stop_motors() # Stop

        lowest_ultrasonic = 2000
        while lowest_ultrasonic == 2000:
            lowest_ultrasonic, lowest_ultrasonic_angle = self.turn_and_detect_ultrasonic() # Turn 360 degrees, find the lowest ultrasonic and angle

        # if lowest_ultrasonic_angle > 180:
        #     lowest_ultrasonic_angle -= 360

        # self.drivebase.reset()

        self.sharp_turn_in_degrees(lowest_ultrasonic_angle) # Turn to the lowest ultrasonic
        self.move_forward(min(lowest_ultrasonic - 20, 260)) # Go to the lowest ultrasonic
        self.rotate_arm(-95, stop_method=Stop.COAST, wait=True) # Arm down, capture the can

        self.sharp_turn_in_degrees(180)
        self.move_forward(min(lowest_ultrasonic - 20, 260))

        # self.move_forward(max(-(lowest_ultrasonic - 20), -260)) # Go back to middle

        return_to_exit_angle = -lowest_ultrasonic_angle
        # return_to_exit_angle = -self.hub.imu.heading() + 180
        if return_to_exit_angle > 180:
            return_to_exit_angle -= 360

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

        self.move_forward(10) # Hopefully sense the black line again

    def avoid_obstacle(self):
        self.stop_motors()
        self.rotate_arm(-90)

        self.sharp_turn_in_degrees(CONSTANTS["OBSTACLE_INITIAL_TURN_DEGREES"])
        self.drivebase.curve(
            CONSTANTS["CURVE_RADIUS_OBSTACLE"],
            -CONSTANTS["OBSTACLE_TURN_DEGREES"],
            Stop.BRAKE,
            True,
        )

        self.drivebase.stop()
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
        self.get_colors()
        self.ultrasonic = self.read_ultra_mm_obstacle()

        if not hasattr(self, "_battery_warned"):
            self._battery_warned = False
            self._last_batt_status = 0
        pct, volts = self.battery_percent()
        if pct is not None:
            now = self.clock.time()
            if now - self._last_batt_status > CONSTANTS.get("BATTERY_STATUS_INTERVAL_MS", 30000):
                print("Battery:", f"{pct}%", f"({volts:.2f} V)")
                self._last_batt_status = now
            if (pct < CONSTANTS.get("BATTERY_WARN_PERCENT", 75)) and (not self._battery_warned):
                print("WARNING: Battery low:", f"{pct}%", f"({volts:.2f} V)")
                self._battery_warned = True

        if self.move_arm_back_after_obstacle_time is not None and self.clock.time() >= self.move_arm_back_after_obstacle_time:
            self.move_arm_back_after_obstacle_time = None
            self.rotate_arm(90, stop_method=Stop.COAST)

        both_green = (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
        both_gray = (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
        if both_green or (CONSTANTS.get("TREAT_GRAY_AS_SPILL", False) and both_gray):
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

        if self.ultrasonic is not None and self.ultrasonic < CONSTANTS["ULTRASONIC_THRESHOLD"]:
            self._obstacle_below_count += 1
        else:
            self._obstacle_below_count = 0
        if self._obstacle_below_count >= CONSTANTS.get("OBSTACLE_DETECT_CONFIRM", 2):
            self._obstacle_below_count = 0
            self.robot_state = "obstacle"
            return

        elif ALLOW_YELLOW and ((self.left_color == Color.YELLOW) ^ (self.right_color == Color.YELLOW)):
            if self.left_color == Color.YELLOW:
                self.robot_state = "yellow-right"
            else:
                self.robot_state = "yellow-left"

        elif (
            not self.shortcut_information["is following shortcut"]
            and self.left_color in [Color.WHITE, Color.BLACK, Color.GRAY]
            and self.right_color in [Color.WHITE, Color.BLACK, Color.GRAY]
            and (
                self.left_color != self.right_color
                or self.left_color != Color.GRAY
            )
        ):
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
        if self.robot_state != self.previous_state:
            self.announce_state(self.robot_state)

        if self.robot_state == "gray":
            self.green_spill_ending()
        elif self.robot_state == "obstacle":
            self.avoid_obstacle()
        elif self.robot_state == "yellow-left":
            self.execute_yellow_shortcut("left")
        elif self.robot_state == "yellow-right":
            self.execute_yellow_shortcut("right")
        elif self.robot_state == "line":
            self.follow_line()
        elif self.robot_state == "green left":
            self.stop_motors()
            self.turn_green("left")
        elif self.robot_state == "green right":
            self.stop_motors()
            self.turn_green("right")
        elif self.robot_state == "stop":
            self.stop_motors()

        self.previous_state = self.robot_state

    def debug(self):
        pass

    def run(self):
        self.battery_display()
        self.rotate_arm(180, Stop.COAST)
        try:
            self.hub.imu.reset_heading(0)
        except Exception:
            pass
        while True:
            self.iteration_count += 1
            self.update()
            self.move()
            # self.debug()

def main():
    robot = Robot()
    robot.run()

main() # Note: don't put in if __name__ == "__main__" because __name__ is something different for robot