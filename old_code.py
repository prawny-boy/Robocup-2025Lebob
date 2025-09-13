from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop
from pybricks.tools import StopWatch, wait

# --- CONSTANTS ---
CONSTANTS = {
    "DRIVEBASE_WHEEL_DIAMETER": 56,
    "DRIVEBASE_AXLE_TRACK": 112,
    "ARM_MOVE_SPEED": 500,
    "DEFAULT_SPEED": 170,
    "DEFAULT_ACCELERATION": 600,
    "DEFAULT_TURN_RATE": 150,
    "DEFAULT_TURN_ACCELERATION": 1600,
    "MOVE_SPEED": 120,
    "ULTRASONIC_THRESHOLD": 140,        # obstacle trigger (mm)
    "OBSTACLE_DETECT_CONFIRM": 4,       # consecutive reads to confirm obstacle
    "BLACK_WHEEL_SPEED": 30,
    # Optional: direct-drive trim to correct mechanical bias (1.00 = no change)
    "LEFT_MOTOR_TRIM": 1.00,
    "RIGHT_MOTOR_TRIM": 1.00,
    "TURN_GREEN_DEGREES": 50,
    "BACK_AFTER_GREEN_TURN_DISTANCE": 40,
    "CURVE_RADIUS_GREEN": 78,
    "CURVE_RADIUS_OBSTACLE": 160,
    "OBSTACLE_TURN_DEGREES": 175,
    "OBSTACLE_INITIAL_TURN_DEGREES": 90,
    "OBSTACLE_FINAL_TURN_DEGREES": 70,
    "OBSTACLE_ARM_RETURN_DELAY": 3000,
    "CURVE_RADIUS_LINE_FOLLOW": 4,
    "BLACK_COUNTER_THRESHOLD": 100,
    # Timeout (ms) to avoid getting stuck while leaving white during obstacle bypass
    "OBSTACLE_WHITE_TIMEOUT_MS": 2500,
    # Line follow tuning (PD + safety)
    "LINE_KP": 3.0,
    "LINE_KD": 18.0,
    "LINE_MIN_SPEED": 80,
    "LINE_MAX_TURN_RATE": 320,
    "CORNER_ERR_THRESHOLD": 13,     # reflectance delta that indicates a sharp corner
    # Can scanning/approach tuning
    "CAN_DEBUG_PRINT": True,
    "CAN_SWEEP_TURN_RATE": 100,
    "CAN_SWEEP_TURN_RATE_SLOW": 60,
    "CAN_SCAN_MAX_MM": 350,
    "CAN_FIRST_HIT_MAX_MM": 350,
    "CAN_EDGE_MARGIN_MM": 120,
    "CAN_IGNORE_ABOVE_MM": 500,
    "CAN_MIN_VALID_MM": 60,
    "CAN_US_MEDIAN_SAMPLES": 5,
    # Micro-oscillation scan tuning
    "CAN_OSC_AMP_DEG": 12,
    "CAN_OSC_PASSES": 3,
    "CAN_OSC_TURN_RATE": 60,
    "CAN_OSC_CENTER_STEP_DEG": 30,
    # Line reacquire after spill exit
    "LINE_REACQUIRE_STEP_MM": 15,
    "LINE_REACQUIRE_CONFIRM": 2,
    "LINE_REACQUIRE_MAX_MM": 400,
    "LINE_REACQUIRE_BACK_MM": 50,
    # Robust line detection threshold (reflection) for reacquire
    "LINE_BLACK_REF_THRESHOLD": 39,
    "LINE_REACQUIRE_TURN_STEP_DEG": 10,
    "LINE_REACQUIRE_TURN_MAX_DEG": 120,
    "CAN_SEGMENT_MIN_POINTS": 4,
    # Push off spill + exit spill tuning
    "CAN_PUSH_STEP_MM": 20,
    "CAN_PUSH_CONFIRM": 4,
    "CAN_PUSH_MAX_MM": 500,
    "CAN_RELEASE_BACK_MM": 80,
    "SPILL_EXIT_STEP_MM": 20,
    "SPILL_EXIT_CONFIRM": 3,
    "SPILL_EXIT_MAX_MM": 400,
    # Battery status
    "BATTERY_V_MIN_MV": 6600,
    "BATTERY_V_MAX_MV": 8400,
    "BATTERY_WARN_PERCENT": 85,
    "BATTERY_STATUS_INTERVAL_MS": 30000,
    # Behavior toggles
    "SOUNDS_ENABLED": True,
    "ENABLE_INVERTED_MODE": False,
    # Green spill detection stability
    "GREEN_DETECT_CONFIRM": 5,
    "SPILL_ENTRY_CONFIRM": 5,
    "SPILL_RETRY_COOLDOWN_MS": 1200,
    # Obstacle reacquire guard
    "OBSTACLE_REACQUIRE_MAX_MM": 220,
    # Gyro/IMU behavior
    "GYRO_ENABLED": True,
    "IMU_STABILIZE_MS": 1200,
    # Straight-drive trim for manual straight helper. Keep small.
    "STRAIGHT_TURN_TRIM": 0,
    # Base global bias added to every drive() turn rate. Negative steers right.
    "BASE_TURN_BIAS": -10,
    # Optional color calibration overrides (gray removed)
    
    # Green
    "GREEN_HUE_MIN": 100,
    "GREEN_HUE_MAX": 190,
    "GREEN_S_MIN": 25,
    
    # Gridlock navigation constants
    "GRIDLOCK_ALIGN_ATTEMPTS": 8,
    "GRIDLOCK_ALIGN_TURN_DEG": 10,
    "GRIDLOCK_LINE_HIT_CONFIRM": 2,
    "GRIDLOCK_HEADING_TOL": 8,
    "GRIDLOCK_STOP_MS": 200,
    "GRIDLOCK_ROCK_STEP_MM": 15,
    "GRIDLOCK_DRIVE_SPEED": 120,
    "GRIDLOCK_FOLLOW_TIMEOUT_MS": 12000,
    "GRIDLOCK_TURN_ANGLE_90": 90,
    "GRIDLOCK_CENTER_FINE_STEP_MM": 6,
    "GRIDLOCK_CENTER_MAX_STEPS": 6,
    "GRIDLOCK_ALIGN_BLACK_CONFIRM": 3,
    "GRIDLOCK_INTERSECTION_CONFIRM": 3,
}
# aahaan
# vivek
# pedram
# pedram 2
# aahaan 2
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
        

    # ========== GRIDLOCK HELPERS ==========
    def gridlock_snap_heading(self, force=False):
        """Snap IMU heading to nearest cardinal (0, 90, -90, 180) when within tolerance.
        If force=True, always snap to the nearest cardinal without checking tolerance."""
        try:
            h = self.hub.imu.heading()
        except Exception:
            return
        cardinals = [0, 90, -90, 180, -180]
        # Distance on a circle
        def ang_diff(a, b):
            return abs(((a - b + 180) % 360) - 180)
        cardinal = min(cardinals, key=lambda c: ang_diff(h, c))
        err = ang_diff(h, cardinal)
        if err < float(CONSTANTS.get("GRIDLOCK_HEADING_TOL", 8)) or force:
            try:
                self.hub.imu.reset_heading(cardinal)
            except Exception:
                pass

    def gridlock_snap_heading_turn(self, tolerance_deg=None):
        """Physically turn to nearest cardinal if within tolerance, then snap IMU."""
        if tolerance_deg is None:
            tolerance_deg = CONSTANTS.get("GRIDLOCK_HEADING_TOL", 8)
        try:
            current = self.hub.imu.heading()
        except Exception:
            return
        cardinals = [0, 90, -90, 180, -180]
        def ang_diff(a, b):
            return abs(((a - b + 180) % 360) - 180)
        target = min(cardinals, key=lambda c: ang_diff(current, c))
        delta = self.normalize_angle(target - current)
        if abs(delta) <= float(tolerance_deg) * 2:
            self.sharp_turn_in_degrees(delta, wait=True)
            self.gridlock_snap_heading(force=True)

    def gridlock_check_black_both(self, confirm_cycles=2):
        """Return True if both color sensors read black reflectance for confirm_cycles in a row."""
        thr = int(CONSTANTS.get("LINE_BLACK_REF_THRESHOLD", 39))
        for _ in range(max(1, int(confirm_cycles))):
            try:
                l = self.color_sensor_left.reflection() < thr
                r = self.color_sensor_right.reflection() < thr
            except Exception:
                return False
            if not (l and r):
                return False
            wait(10)
        return True

    def gridlock_align_on_intersection(self):
        """Gently pivot and rock to align on an intersection until both sensors see black reliably."""
        self.drivebase.stop()
        wait(CONSTANTS.get("GRIDLOCK_STOP_MS", 200))
        pivot = int(CONSTANTS.get("GRIDLOCK_ALIGN_TURN_DEG", 8))
        rock_step = int(CONSTANTS.get("GRIDLOCK_ROCK_STEP_MM", 15))
        confirm = int(CONSTANTS.get("GRIDLOCK_ALIGN_BLACK_CONFIRM", 3))
        attempts = int(CONSTANTS.get("GRIDLOCK_ALIGN_ATTEMPTS", 6))
        for _ in range(attempts):
            if self.gridlock_check_black_both(confirm):
                break
            # Small left-right-left pivot
            self.sharp_turn_in_degrees(-pivot, wait=True)
            if self.gridlock_check_black_both(confirm):
                break
            self.sharp_turn_in_degrees(2 * pivot, wait=True)
            if self.gridlock_check_black_both(confirm):
                break
            self.sharp_turn_in_degrees(-pivot, wait=True)
            if self.gridlock_check_black_both(confirm):
                break
            # Gentle forward-back rock
            self.move_forward(rock_step)
            if self.gridlock_check_black_both(confirm):
                break
            self.move_forward(-2 * rock_step)
            if self.gridlock_check_black_both(confirm):
                break
            self.move_forward(rock_step)
        self.drivebase.stop()
        wait(CONSTANTS.get("GRIDLOCK_STOP_MS", 200))
        self.gridlock_snap_heading_turn()

    def gridlock_nudge_center(self):
        """Small forward/back nudges to center on the intersection."""
        step = int(CONSTANTS.get("GRIDLOCK_CENTER_FINE_STEP_MM", 6))
        max_steps = int(CONSTANTS.get("GRIDLOCK_CENTER_MAX_STEPS", 6))
        confirm = int(CONSTANTS.get("GRIDLOCK_INTERSECTION_CONFIRM", 3))
        for _ in range(max_steps):
            if self.gridlock_check_black_both(confirm):
                return True
            self.move_forward(step)
            if self.gridlock_check_black_both(confirm):
                return True
            self.move_forward(-2 * step)
            if self.gridlock_check_black_both(confirm):
                return True
            self.move_forward(step)
        return False

    def gridlock_turn_90(self, direction="right"):
        """Turn 90° left/right, then re-align and re-snap to the grid."""
        d = int(CONSTANTS.get("GRIDLOCK_TURN_ANGLE_90", 90))
        angle = d if direction == "right" else -d
        self.sharp_turn_in_degrees(angle, wait=True)
        wait(CONSTANTS.get("GRIDLOCK_STOP_MS", 200))
        self.gridlock_align_on_intersection()
        self.gridlock_snap_heading_turn()

    def gridlock_drive_to_next_intersection(self):
        """Follow line to next intersection (both sensors black), then align and correct heading."""
        # Set controlled approach speed
        try:
            ss, sa, tr, ta = self.drivebase.settings()
        except Exception:
            ss = CONSTANTS.get("DEFAULT_SPEED", 170)
        approach_speed = int(CONSTANTS.get("GRIDLOCK_DRIVE_SPEED", ss))
        self.drivebase.settings(straight_speed=approach_speed)
        timeout_ms = int(CONSTANTS.get("GRIDLOCK_FOLLOW_TIMEOUT_MS", 12000))
        start = self.clock.time()
        found = False
        while self.clock.time() - start < timeout_ms:
            self.get_colors()
            self.follow_line()
            if self.gridlock_check_black_both(CONSTANTS.get("GRIDLOCK_INTERSECTION_CONFIRM", 3)):
                found = True
                break
            wait(8)
        self.drivebase.stop()
        wait(CONSTANTS.get("GRIDLOCK_STOP_MS", 200))
        if found:
            self.gridlock_align_on_intersection()
            self.gridlock_nudge_center()
            self.gridlock_snap_heading_turn()
        # Restore default drive settings
        self.settings_default()
        return found

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
        # Gray handling removed
        if information["color"] == Color.WHITE:
            return Color.WHITE
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
        speed_mm_s = int(CONSTANTS.get("BLACK_DRIVE_SPEED", CONSTANTS.get("BLACK_WHEEL_SPEED", 30)))
        trim = float(CONSTANTS.get("STRAIGHT_TURN_TRIM", 0.0))
        self.drive_with_bias(speed_mm_s, trim)

    def turn_green(self, direction):
        self.move_forward(8)

        self.get_colors()
        if (
            self.left_color == Color.GREEN and self.right_color == Color.GREEN
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
            if both_green:
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
            new_ultra = self.read_ultra_mm_can()
            self.can_log("CAN Fallback 360 d=", new_ultra, "a=", self.drivebase.angle())
            if new_ultra < lowest_ultrasonic:
                lowest_ultrasonic = new_ultra
                lowest_ultrasonic_angle = self.drivebase.angle()
        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)
        return lowest_ultrasonic, lowest_ultrasonic_angle

    def sweep_find_can_midpoint(self, sweep_deg=160, threshold=None, confirm=3):
        if threshold is None:
            threshold = CONSTANTS["CAN_SCAN_MAX_MM"]

        self.sharp_turn_in_degrees(-sweep_deg // 2, wait=True)
        self.drivebase.reset()

        self.sharp_turn_in_degrees(sweep_deg, wait=False)
        points = []
        while not self.drivebase.done():
            d = self.read_ultra_mm()
            a = self.drivebase.angle()
            points.append((a, d))

        segments = []
        cur = []
        for a, d in points:
            if d < threshold:
                cur.append((a, d))
            else:
                if len(cur) >= CONSTANTS["CAN_SEGMENT_MIN_POINTS"]:
                    segments.append(cur)
                cur = []
        if len(cur) >= CONSTANTS["CAN_SEGMENT_MIN_POINTS"]:
            segments.append(cur)

        if not segments:
            return None, None

        def seg_min(seg):
            return min(seg, key=lambda t: t[1])

        best_seg = min(segments, key=lambda s: seg_min(s)[1])
        min_a, min_d = seg_min(best_seg)

        window = [(a, d) for a, d in best_seg if d <= min_d + 40]
        if window:
            wsum = 0.0
            asum = 0.0
            for a, d in window:
                w = 1.0 / max(1.0, float(d))
                wsum += w
                asum += w * a
            target_angle = asum / max(1e-6, wsum)
        else:
            target_angle = min_a

        return target_angle, min_d

    def find_can_edges_midpoint(self, sweep_deg=160, threshold=None, confirm=3):
        if threshold is None:
            threshold = CONSTANTS["CAN_SCAN_MAX_MM"]

        attempts = 0
        first_hit_angle = None
        first_hit_distance = 9999
        min_d = 9999
        while True:
            self.drivebase.reset()
            start_ang = self.drivebase.angle()
            try:
                ss, sa, prev_tr, ta = self.drivebase.settings()
            except Exception:
                prev_tr = None
            sweep_rate = (
                CONSTANTS["CAN_SWEEP_TURN_RATE_SLOW"]
                if self.can_sweep_slow_mode
                else CONSTANTS["CAN_SWEEP_TURN_RATE"]
            )
            self.drivebase.settings(turn_rate=sweep_rate)
            first_hit_angle = None
            while self.drivebase.angle() != 0:
                break
            self.can_log("CAN 360 sweep: rate=", sweep_rate)
            self.sharp_turn_in_degrees(360, wait=False)
            while not self.drivebase.done():
                d = self.read_ultra_mm_can()
                a = self.drivebase.angle()
                self.can_log("CAN 360 d=", d, "a=", a)
                if d < min_d:
                    min_d = d
                if d <= CONSTANTS["CAN_FIRST_HIT_MAX_MM"] and first_hit_angle is None:
                    first_hit_angle = a
                    first_hit_distance = d
                    break
            self.drivebase.stop()
            if prev_tr is not None:
                self.drivebase.settings(turn_rate=prev_tr)
            if self.can_recording:
                self.can_path.append(("turn", self.drivebase.angle() - start_ang))

            if first_hit_angle is not None:
                self.can_log("CAN 360: first hit at angle=", first_hit_angle, "min_d=", min_d)
                break

            if not self.can_sweep_slow_mode:
                attempts += 1
                if attempts < 2:
                    self.can_log("CAN 360: no detection, fast attempt", attempts)
                    continue
                self.can_sweep_slow_mode = True
                self.can_log("CAN 360: switching to slow sweep rate")
                continue
            else:
                self.can_log("CAN 360: no detection even in slow mode; giving up")
                return None, None

        self.drivebase.reset()
        start_ang = self.drivebase.angle()
        self.sharp_turn_in_degrees(-(sweep_deg // 2), wait=False)
        left_edge = None
        last_seen = None
        lost = 0
        edge_threshold = first_hit_distance + CONSTANTS["CAN_EDGE_MARGIN_MM"]
        while not self.drivebase.done():
            d = self.read_ultra_mm_can()
            a = self.drivebase.angle()
            self.can_log("CAN LEFT d=", d, "a=", a)
            if d <= edge_threshold:
                last_seen = a
                lost = 0
                if d < min_d:
                    min_d = d
            else:
                lost += 1
                if lost >= confirm and last_seen is not None:
                    left_edge = last_seen
                    break
        self.drivebase.stop()
        if self.can_recording:
            self.can_path.append(("turn", self.drivebase.angle() - start_ang))
        if left_edge is None and last_seen is not None:
            left_edge = last_seen

        start_ang = self.drivebase.angle()
        self.sharp_turn_in_degrees(sweep_deg, wait=False)
        right_edge = None
        last_seen = None
        lost = 0
        while not self.drivebase.done():
            d = self.read_ultra_mm_can()
            a = self.drivebase.angle()
            self.can_log("CAN RIGHT d=", d, "a=", a)
            if d <= edge_threshold:
                last_seen = a
                lost = 0
                if d < min_d:
                    min_d = d
            else:
                lost += 1
                if lost >= confirm and last_seen is not None:
                    right_edge = last_seen
                    break
        self.drivebase.stop()
        if self.can_recording:
            self.can_path.append(("turn", self.drivebase.angle() - start_ang))
        if right_edge is None and last_seen is not None:
            right_edge = last_seen

        if left_edge is None or right_edge is None:
            return None, None

        mid = (left_edge + right_edge) / 2.0
        return mid, min_d

    def find_can_angle_micro_oscillation(self, amp_deg=None, passes=None):
        if amp_deg is None:
            amp_deg = CONSTANTS["CAN_OSC_AMP_DEG"]
        if passes is None:
            passes = CONSTANTS["CAN_OSC_PASSES"]

        try:
            ss, sa, prev_tr, ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
        self.drivebase.settings(turn_rate=CONSTANTS["CAN_OSC_TURN_RATE"])

        self.drivebase.reset()
        best_d = 9999
        best_a = 0

        def sample_during_turn(target_abs_angle):
            nonlocal best_d, best_a
            delta = target_abs_angle - self.drivebase.angle()
            self.sharp_turn_in_degrees(delta, wait=False)
            while not self.drivebase.done():
                d = self.read_ultra_mm_can()
                a = self.drivebase.angle()
                self.can_log("CAN OSC d=", d, "a=", a)
                if d < best_d:
                    best_d = d
                    best_a = a

        for _ in range(passes):
            sample_during_turn(-amp_deg)
            sample_during_turn(+amp_deg)

        sample_during_turn(0)

        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)

        if best_d <= CONSTANTS["CAN_FIRST_HIT_MAX_MM"]:
            return best_a, best_d
        return None, None

    def find_can_angle_micro_oscillation_full360(self, amp_deg=None, center_step_deg=None, passes=None):
        if amp_deg is None:
            amp_deg = CONSTANTS["CAN_OSC_AMP_DEG"]
        if center_step_deg is None:
            center_step_deg = CONSTANTS["CAN_OSC_CENTER_STEP_DEG"]
        if passes is None:
            passes = CONSTANTS["CAN_OSC_PASSES"]

        try:
            ss, sa, prev_tr, ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
        self.drivebase.settings(turn_rate=CONSTANTS["CAN_OSC_TURN_RATE"])

        self.drivebase.reset()
        best_d = 9999
        best_a = None

        def sample_during_turn(target_abs_angle):
            nonlocal best_d, best_a
            delta = target_abs_angle - self.drivebase.angle()
            self.sharp_turn_in_degrees(delta, wait=False)
            while not self.drivebase.done():
                d = self.read_ultra_mm_can()
                a = self.drivebase.angle()
                self.can_log("CAN OSC360 d=", d, "a=", a)
                if d < best_d:
                    best_d = d
                    best_a = a

        centers = list(range(0, 360, max(1, int(center_step_deg))))
        for _ in range(passes):
            for c in centers:
                sample_during_turn(c - amp_deg)
                sample_during_turn(c + amp_deg)

        sample_during_turn(0)

        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)

        if best_a is None:
            return None, None
        if best_d <= CONSTANTS["CAN_FIRST_HIT_MAX_MM"]:
            return best_a, best_d
        return None, None

    def approach_can_at_angle(self, target_angle_deg, stop_offset_mm=20, max_step_mm=30):
        total_forward = 0
        current = self.drivebase.angle()
        self.can_turn(target_angle_deg - current)
        while True:
            d = self.read_ultra_mm_can()
            self.can_log("CAN APPROACH d=", d)
            if d >= 9000:
                break
            if d <= stop_offset_mm + 5:
                break

            step = max(10, min(max_step_mm, d - stop_offset_mm))
            self.can_log("CAN APPROACH step=", step)
            self.can_move_forward(step)
            total_forward += step
        return total_forward

    def push_can_off_spill(self, step_mm=20, confirm=3, max_push_mm=500):
        pushed = 0
        off_count = 0
        while pushed < max_push_mm:
            self.get_colors()
            on_green = (self.left_color == Color.GREEN) or (self.right_color == Color.GREEN)
            du = self.read_ultra_mm_can()
            self.can_log("CAN PUSH on_green=", on_green, "d=", du)
            if not on_green:
                off_count += 1
                if off_count >= confirm:
                    break
            else:
                off_count = 0
            self.can_move_forward(step_mm)
            pushed += step_mm
        return pushed

    def push_can_to_back_of_spill_until_boundary(self, step_mm=20, bound_confirm=3, max_push_mm=700):
        """Push the can straight until both sensors read WHITE, then stop.
        Advances in small recorded steps for accurate backtrack."""
        if self.spill_entry_heading is not None:
            self.face_heading(self.spill_entry_heading)

        pushed = 0
        hit_confirm = 0
        while pushed < max_push_mm:
            self.get_colors()
            # Release condition: both sensors see white (spill boundary)
            boundary = (self.left_color == Color.WHITE and self.right_color == Color.WHITE)
            if boundary:
                hit_confirm += 1
                if hit_confirm >= bound_confirm:
                    break
            else:
                hit_confirm = 0

            # Advance in small straight steps and record them for precise backtrack
            self.can_move_forward(step_mm)
            pushed += step_mm
        return pushed

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
            self.left_color == Color.GREEN and self.right_color == Color.GREEN
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

        # Use main2.py style 360-degree scan to detect the can direction
        lowest_ultrasonic, lowest_ultrasonic_angle = self.turn_and_detect_ultrasonic(360)
        if lowest_ultrasonic is None or lowest_ultrasonic >= 9000:
            # No valid detection
            self.settings_default()
            return

        # Normalize angle similar to main2.py and approach using existing helpers
        if lowest_ultrasonic_angle > 180:
            lowest_ultrasonic_angle -= 360
        # Reset angle reference before approaching
        self.drivebase.reset()
        self.approach_can_at_angle(lowest_ultrasonic_angle, stop_offset_mm=20)

        self.rotate_arm(-95, stop_method=Stop.COAST, wait=True)

        push_forward = self.push_can_to_back_of_spill_until_boundary(
            step_mm=CONSTANTS["CAN_PUSH_STEP_MM"],
            bound_confirm=CONSTANTS["CAN_PUSH_CONFIRM"],
            max_push_mm=CONSTANTS["CAN_PUSH_MAX_MM"],
        )

        self.rotate_arm(180)
        self.can_move_forward(-CONSTANTS["CAN_RELEASE_BACK_MM"])

        self.can_backtrack()

        if self.spill_entry_heading is not None:
            exit_heading = self.normalize_angle(self.spill_entry_heading + 180)
            self.face_heading(exit_heading)
        else:
            self.sharp_turn_in_degrees(180)
        self.exit_green_spill_forward(
            step_mm=CONSTANTS["SPILL_EXIT_STEP_MM"],
            confirm=CONSTANTS["SPILL_EXIT_CONFIRM"],
            max_mm=CONSTANTS["SPILL_EXIT_MAX_MM"],
        )

        self.move_forward(-CONSTANTS["LINE_REACQUIRE_BACK_MM"])
        self.get_colors()
        if self.left_color == Color.GREEN and self.right_color == Color.GREEN:
            self.exit_green_spill_forward(
                step_mm=CONSTANTS["SPILL_EXIT_STEP_MM"],
                confirm=CONSTANTS["SPILL_EXIT_CONFIRM"],
                max_mm=CONSTANTS["SPILL_EXIT_STEP_MM"] * 3,
            )

        self.reacquire_line_oscillate(
            step_deg=CONSTANTS["LINE_REACQUIRE_TURN_STEP_DEG"],
            max_deg=CONSTANTS["LINE_REACQUIRE_TURN_MAX_DEG"],
            confirm=CONSTANTS["LINE_REACQUIRE_CONFIRM"],
        )

        self.settings_default()
        self.robot_state = "line"

    def exit_green_spill_forward(self, step_mm=20, confirm=3, max_mm=400):
        moved = 0
        off_count = 0
        while moved < max_mm:
            self.get_colors()
            both_green = self.left_color == Color.GREEN and self.right_color == Color.GREEN
            du = self.read_ultra_mm_can()
            self.can_log("SPILL EXIT both_green=", both_green, "d=", du)
            if not both_green:
                off_count += 1
                if off_count >= confirm:
                    break
            else:
                off_count = 0
            self.move_forward(step_mm)
            moved += step_mm
        return moved

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
            self.drive_with_bias(0, turn_rate)
            wait(10)
        self.drivebase.stop()

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
        if both_green:
            self.robot_state = "spill"
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

        

        if (
            not self.shortcut_information["is following shortcut"]
            and self.left_color in [Color.WHITE, Color.BLACK]
            and self.right_color in [Color.WHITE, Color.BLACK]
        ):
            self.robot_state = "line"

        elif self.left_color == Color.GREEN or self.right_color == Color.GREEN:
            if self.left_color == Color.GREEN:
                self._green_left_count += 1
            else:
                self._green_left_count = 0
            if self.right_color == Color.GREEN:
                self._green_right_count += 1
            else:
                self._green_right_count = 0

            if self.clock.time() >= self._spill_cooldown_until:
                if self._green_left_count >= CONSTANTS.get("GREEN_DETECT_CONFIRM", 2):
                    self.robot_state = "green left"
                elif self._green_right_count >= CONSTANTS.get("GREEN_DETECT_CONFIRM", 2):
                    self.robot_state = "green right"
                else:
                    self.robot_state = "line"
            else:
                self.robot_state = "line"
        else:
            self.robot_state = "line"

    def move(self):
        if self.robot_state != self.previous_state:
            self.announce_state(self.robot_state)

        if self.robot_state == "spill":
            self.green_spill_ending()
        elif self.robot_state == "obstacle":
            self.avoid_obstacle()
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
            self.debug()

def main():
    robot = Robot()
    try:
        robot.run()
    finally:
        try:
            robot.stop_motors()
        except Exception:
            pass

if __name__ == "__main__":
    main()
# vivek
