from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Button, Stop
from pybricks.tools import StopWatch, wait

# Enable/disable optional yellow-line shortcut behavior
ALLOW_YELLOW = False

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
    "ULTRASONIC_THRESHOLD": 140,        # obstacle trigger (mm) - earlier avoidance
    "OBSTACLE_DETECT_CONFIRM": 4,       # consecutive reads below threshold to confirm (reduce false triggers)
    "BLACK_WHEEL_SPEED": 30,
    "TURN_GREEN_DEGREES": 50,
    "BACK_AFTER_GREEN_TURN_DISTANCE": 6,
    "TURN_YELLOW_DEGREES": 20,
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
    "CAN_DEBUG_PRINT": False,           # disable debug printing in can routines
    "CAN_SWEEP_TURN_RATE": 120,        # deg/s used for the initial 360° can sweep
    "CAN_SWEEP_TURN_RATE_SLOW": 60,    # deg/s used after two failed 360 sweeps
    "CAN_SCAN_MAX_MM": 350,             # default scan threshold (<= this is "seen")
    "CAN_FIRST_HIT_MAX_MM": 550,        # permissive threshold for first detection during 360
    "CAN_EDGE_MARGIN_MM": 120,          # margin above first-hit distance to keep the object in sweeps
    "CAN_IGNORE_ABOVE_MM": 1800,        # treat readings above this as no target (environmental)
    "CAN_MIN_VALID_MM": 60,             # ignore too-close returns (sensor near-field)
    "CAN_US_MEDIAN_SAMPLES": 5,         # median filter samples for ultrasonic in can logic
    # Micro-oscillation scan tuning
    "CAN_OSC_AMP_DEG": 12,              # oscillation amplitude (+/- degrees)
    "CAN_OSC_PASSES": 3,                # number of back-and-forth passes
    "CAN_OSC_TURN_RATE": 60,            # deg/s during oscillation
    "CAN_OSC_CENTER_STEP_DEG": 30,      # center advance step to cover full 360
    # Line reacquire after spill exit
    "LINE_REACQUIRE_STEP_MM": 15,
    "LINE_REACQUIRE_CONFIRM": 2,
    "LINE_REACQUIRE_MAX_MM": 400,
    "LINE_REACQUIRE_BACK_MM": 50,
    # Robust line detection threshold (reflection) for reacquire
    "LINE_BLACK_REF_THRESHOLD": 35,
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
    "BATTERY_V_MIN_MV": 6600,          # tune to your hub pack (empty)
    "BATTERY_V_MAX_MV": 8400,          # tune to your hub pack (full)
    "BATTERY_WARN_PERCENT": 85,         # warn when below this percent
    "BATTERY_STATUS_INTERVAL_MS": 30000,
    # Behavior toggles
    "SOUNDS_ENABLED": True,             # control state-change beeps
    "TREAT_GRAY_AS_SPILL": False,       # if True, treat both-sensor GRAY as spill entry
    "ENABLE_INVERTED_MODE": False,      # auto-switch follow_line error sign on inverted tracks
    # Green spill detection stability
    "GREEN_DETECT_CONFIRM": 3,          # cycles before entering green-left/right state
    "SPILL_ENTRY_CONFIRM": 3,           # consecutive reads to confirm in-spill during turn
    "SPILL_RETRY_COOLDOWN_MS": 1200,    # initial cooldown after a failed turn_green
    # Obstacle reacquire guard
    "OBSTACLE_REACQUIRE_MAX_MM": 220,    # hard cap forward travel while searching for line
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
        try:
            self.drivebase.use_gyro(True)
        except Exception:
            # Fallback silently if gyro not available
            pass
        self.settings_default()

        self.robot_state = "obstacle"  # Initial state
        self.iteration_count = 0
        self.black_counter = 0
        self.on_inverted = False
        # Schedule (ms since start) for arm to return after obstacle; None when not scheduled
        self.move_arm_back_after_obstacle_time = None
        self.debug_enabled = False  # toggle prints on hub
        self.prev_error = 0
        self.corner_hold = 0
        self.line_pivoting = False
        self._obstacle_below_count = 0

        self.shortcut_information = {
            "is following shortcut": False,
            "first turned": None,
            "left seen black since": False,
            "right seen black since": False
        }
        self.default_shortcut_information = self.shortcut_information.copy()

        # Can routine path recording (to return exactly to entry point)
        self.can_recording = False
        self.can_path = []  # list of (action, value), action in {"move", "turn"}
        # Can sweep behavior: after two failed 360 sweeps, switch to slow sweeps permanently
        self.can_sweep_slow_mode = False
        self.spill_entry_heading = None
        # Green detection helpers
        self._green_left_count = 0
        self._green_right_count = 0
        self._spill_cooldown_until = 0
        self._spill_retry_backoff_ms = CONSTANTS.get("SPILL_RETRY_COOLDOWN_MS", 1200)

    # --- helpers ---
    def read_ultra_mm(self):
        """Raw ultrasonic distance in mm; returns 9999 if no valid target."""
        d = self.ultrasonic_sensor.distance()
        if d is None:
            return 9999
        return d

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
        if m > CONSTANTS.get("CAN_IGNORE_ABOVE_MM", 1800):
            return 9999
        return m

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
        print("Battery:", f"{pct}%", f"({volts:.2f} V)" + warn)
    
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
        self.drivebase.straight(distance, wait=wait)

    # --- Sounds ---
    def announce_state(self, state):
        """Play a short, distinct beep pattern for each state start."""
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
            # If speaker not available or busy, ignore
            pass

    # --- Recording helpers for can routine ---
    def can_start_recording(self):
        self.can_path = []
        self.can_recording = True

    def can_cancel_recording(self):
        self.can_path = []
        self.can_recording = False

    def normalize_angle(self, angle):
        # normalize to [-180, 180)
        a = ((angle + 180) % 360) - 180
        return a

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
        """Reverse the recorded path exactly (invert order and actions)."""
        for action, value in reversed(self.can_path):
            if action == "move":
                self.move_forward(-value)
            elif action == "turn":
                self.sharp_turn_in_degrees(-value)
        self.can_cancel_recording()

    def start_motors(self, left_speed, right_speed):
        """Run motors at given speeds."""
        if self.left_drive.speed() != left_speed:
            self.left_drive.run(left_speed)
        if self.right_drive.speed() != right_speed:
            self.right_drive.run(right_speed)

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
        if information["reflection"] > 99:
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
        elif information["color"] == Color.RED:
            return Color.ORANGE
        elif information["hsv"].s < 25 and information["reflection"] < 30:
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
        self.start_motors(CONSTANTS["BLACK_WHEEL_SPEED"], CONSTANTS["BLACK_WHEEL_SPEED"])
    
    def turn_green(self, direction):
        """Pivot toward a detected green marker and enter the spill.
        More robust: always attempt the turn, accept both GREEN or overbright GRAY as spill,
        and never return early leaving the robot stopped.
        """
        # Nudge forward a little from the detection point
        self.move_forward(8)

        # If we're already clearly on the spill (both GREEN or both GRAY), go straight in
        self.get_colors()
        if (
            (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
            or (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
        ):
            self.drivebase.stop()
            self.green_spill_ending()
            return

        # Decide turn direction based on the side that saw green
        degrees = -CONSTANTS["TURN_GREEN_DEGREES"] if direction == "left" else CONSTANTS["TURN_GREEN_DEGREES"]

        # Turn without waiting so we can detect when we fully enter the spill
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
            # Undo the small forward nudge so we don't drift off the line over time
            self.move_forward(-CONSTANTS["BACK_AFTER_GREEN_TURN_DISTANCE"])
            # Ensure we resume normal behavior and start a short cooldown
            self.robot_state = "line"
            now = self.clock.time()
            self._spill_cooldown_until = now + self._spill_retry_backoff_ms
            # Exponential backoff up to 5s to avoid repeated attempts/beeps
            self._spill_retry_backoff_ms = min(5000, int(self._spill_retry_backoff_ms * 2))

    def follow_line(self):
        # Simple PD line follow with dynamic slowdown; no blocking, no pivots
        left_ref = self.left_color_sensor_information["reflection"]
        right_ref = self.right_color_sensor_information["reflection"]

        error = (left_ref - right_ref) if not self.on_inverted else (right_ref - left_ref)

        kp = CONSTANTS["LINE_KP"]
        kd = CONSTANTS["LINE_KD"]
        d_err = error - self.prev_error
        self.prev_error = error
        turn_rate = kp * error + kd * d_err

        # Clamp turn rate
        max_turn = CONSTANTS.get("LINE_MAX_TURN_RATE", 320)
        if turn_rate > max_turn:
            turn_rate = max_turn
        elif turn_rate < -max_turn:
            turn_rate = -max_turn

        # Slow down slightly with larger error to make tight corners
        err_scale = min(1.0, abs(error) / max(1, CONSTANTS["CORNER_ERR_THRESHOLD"]))
        base_speed = CONSTANTS["MOVE_SPEED"]
        speed = max(CONSTANTS["LINE_MIN_SPEED"], int(base_speed * (1 - 0.4 * err_scale)))

        self.drivebase.drive(speed, turn_rate)
    
    def follow_color(self, color_to_follow=Color.YELLOW):
        if self.left_color == color_to_follow:
            self.turn_in_degrees(-CONSTANTS["TURN_YELLOW_DEGREES"])
        elif self.right_color == color_to_follow:
            self.turn_in_degrees(CONSTANTS["TURN_YELLOW_DEGREES"])
        else:
            self.move_forward(10, wait=False)

    def turn_and_detect_ultrasonic(self, degrees=360):
        """Single 360 scan. Returns (min_dist, at_angle)."""
        lowest_ultrasonic = 9999
        lowest_ultrasonic_angle = 0
        self.drivebase.reset()
        # Use fast or slow sweep rate based on can_sweep_slow_mode
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
        # Restore previous turn rate
        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)
        return lowest_ultrasonic, lowest_ultrasonic_angle

    def sweep_find_can_midpoint(self, sweep_deg=160, threshold=None, confirm=3):
        """Sweep left→right. Find first/last angles where can is seen (<= threshold).
        Returns (mid_angle_deg, approach_dist_mm) or (None, None).
        """
        if threshold is None:
            threshold = CONSTANTS["CAN_SCAN_MAX_MM"]

        # Move to left edge and zero angle reference
        self.sharp_turn_in_degrees(-sweep_deg // 2, wait=True)
        self.drivebase.reset()

        # Sweep to the right and record points (angle, distance)
        self.sharp_turn_in_degrees(sweep_deg, wait=False)
        points = []
        while not self.drivebase.done():
            d = self.read_ultra_mm()
            a = self.drivebase.angle()
            points.append((a, d))

        # Segment contiguous points within threshold; choose segment with closest min distance
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

        # Pick the segment whose minimum distance is smallest (closest object)
        best_seg = min(segments, key=lambda s: seg_min(s)[1])
        min_a, min_d = seg_min(best_seg)

        # Weighted centroid around local minimum to reduce bias
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
        """360 scan until first detection, then sweep left/right to edges and return midpoint.
        Returns (mid_angle_deg, min_dist_mm) or (None, None).
        """
        if threshold is None:
            threshold = CONSTANTS["CAN_SCAN_MAX_MM"]

        # 360 to find the first detection point (record actual turned angle)
        attempts = 0
        first_hit_angle = None
        first_hit_distance = 9999
        min_d = 9999
        while True:
            self.drivebase.reset()
            start_ang = self.drivebase.angle()
            # Temporarily set a specific sweep turn rate
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
                # ensure reset applied before starting turn
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
            # Restore previous turn rate if available
            if prev_tr is not None:
                self.drivebase.settings(turn_rate=prev_tr)
            # Record actual turned amount
            if self.can_recording:
                self.can_path.append(("turn", self.drivebase.angle() - start_ang))

            if first_hit_angle is not None:
                self.can_log("CAN 360: first hit at angle=", first_hit_angle, "min_d=", min_d)
                break

            # Not found this sweep
            if not self.can_sweep_slow_mode:
                attempts += 1
                if attempts < 2:
                    # Try a second fast sweep
                    self.can_log("CAN 360: no detection, fast attempt", attempts)
                    continue
                # After two failed sweeps, switch to slow mode for all future 360s
                self.can_sweep_slow_mode = True
                self.can_log("CAN 360: switching to slow sweep rate")
                # Perform one immediate slow sweep
                continue
            else:
                # Already slow and still not detected; give up
                self.can_log("CAN 360: no detection even in slow mode; giving up")
                return None, None

        # Use current heading as center, sweep left to detect left edge
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
        # Fallback: if we ended the sweep still seeing the can, use last_seen
        if left_edge is None and last_seen is not None:
            left_edge = last_seen

        # Sweep right to detect right edge (keep same angle frame as left)
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
        # Fallback: end-of-sweep still seeing can
        if right_edge is None and last_seen is not None:
            right_edge = last_seen

        if left_edge is None or right_edge is None:
            return None, None

        mid = (left_edge + right_edge) / 2.0
        return mid, min_d

    def find_can_angle_micro_oscillation(self, amp_deg=None, passes=None):
        """Oscillate around center heading and return the angle with the minimum filtered distance.
        Returns (angle_deg, min_dist_mm) or (None, None) if nothing plausible is seen.
        """
        if amp_deg is None:
            amp_deg = CONSTANTS["CAN_OSC_AMP_DEG"]
        if passes is None:
            passes = CONSTANTS["CAN_OSC_PASSES"]

        # Save/override turn rate for precise oscillation
        try:
            ss, sa, prev_tr, ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
        self.drivebase.settings(turn_rate=CONSTANTS["CAN_OSC_TURN_RATE"])

        self.drivebase.reset()  # center = 0 deg
        best_d = 9999
        best_a = 0

        def sample_during_turn(target_abs_angle):
            nonlocal best_d, best_a
            # Compute incremental delta to reach absolute angle
            delta = target_abs_angle - self.drivebase.angle()
            self.sharp_turn_in_degrees(delta, wait=False)
            while not self.drivebase.done():
                d = self.read_ultra_mm_can()
                a = self.drivebase.angle()
                self.can_log("CAN OSC d=", d, "a=", a)
                if d < best_d:
                    best_d = d
                    best_a = a

        # Perform back-and-forth oscillation passes
        for i in range(passes):
            # Left sweep to -amp
            sample_during_turn(-amp_deg)
            # Right sweep to +amp
            sample_during_turn(+amp_deg)

        # Return to center (optional; do not record in can path here)
        sample_during_turn(0)

        # Restore turn rate
        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)

        # Validate best distance against acceptance threshold
        if best_d <= CONSTANTS["CAN_FIRST_HIT_MAX_MM"]:
            return best_a, best_d
        return None, None

    def find_can_angle_micro_oscillation_full360(self, amp_deg=None, center_step_deg=None, passes=None):
        """Cover the full 360° by oscillating around advancing centers.
        Returns (angle_deg, min_dist_mm) or (None, None).
        """
        if amp_deg is None:
            amp_deg = CONSTANTS["CAN_OSC_AMP_DEG"]
        if center_step_deg is None:
            center_step_deg = CONSTANTS["CAN_OSC_CENTER_STEP_DEG"]
        if passes is None:
            passes = CONSTANTS["CAN_OSC_PASSES"]

        # Save/override turn rate for precise oscillation
        try:
            ss, sa, prev_tr, ta = self.drivebase.settings()
        except Exception:
            prev_tr = None
        self.drivebase.settings(turn_rate=CONSTANTS["CAN_OSC_TURN_RATE"])

        self.drivebase.reset()  # set absolute angle reference
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

        # Centers from 0 to <360 in steps
        centers = list(range(0, 360, max(1, int(center_step_deg))))
        for _ in range(passes):
            for c in centers:
                # Oscillate around center c
                sample_during_turn(c - amp_deg)
                sample_during_turn(c + amp_deg)

        # Return to nearest center (optional tidy)
        sample_during_turn(0)

        # Restore turn rate
        if prev_tr is not None:
            self.drivebase.settings(turn_rate=prev_tr)

        if best_a is None:
            return None, None
        if best_d <= CONSTANTS["CAN_FIRST_HIT_MAX_MM"]:
            return best_a, best_d
        return None, None

    def approach_can_at_angle(self, target_angle_deg, stop_offset_mm=20, max_step_mm=30):
        """Face target and step forward until within offset (no micro-sweeps).
        Returns the total forward distance moved (mm).
        """
        total_forward = 0
        current = self.drivebase.angle()
        # Face the target once, then keep heading fixed during the approach
        self.can_turn(target_angle_deg - current)
        while True:
            d = self.read_ultra_mm_can()
            self.can_log("CAN APPROACH d=", d)
            if d >= 9000:
                break  # lost target
            if d <= stop_offset_mm + 5:
                break

            step = max(10, min(max_step_mm, d - stop_offset_mm))
            self.can_log("CAN APPROACH step=", step)
            self.can_move_forward(step)
            total_forward += step
        return total_forward

    def push_can_off_spill(self, step_mm=20, confirm=3, max_push_mm=500):
        """Drive forward in steps while on green; stop when off-green confirmed.
        Returns total forward distance moved (mm).
        """
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
        """Face toward spill back (entry heading) and push with one continuous drive
        until both sensors see red or both see white.
        Returns total forward distance moved (mm).
        """
        # Face the original spill entry heading if known
        if self.spill_entry_heading is not None:
            self.face_heading(self.spill_entry_heading)

        # Use continuous drive forward at a steady speed, checking sensors on the fly.
        # Keep speed consistent with prior "slow" precision settings used in spill logic.
        push_speed = 80  # mm/s; matches slow precise movement set earlier

        # Reset distance tracking so we can record a single move for backtrack
        self.drivebase.reset()

        # Start continuous forward motion
        self.drivebase.drive(push_speed, 0)

        hit_confirm = 0
        while True:
            # Optional safety cap on max push distance
            pushed = abs(self.drivebase.distance())
            if pushed >= max_push_mm:
                break

            # Read colors to detect the boundary condition (two RED or two WHITE)
            self.get_colors()
            boundary = (
                (self.left_color == Color.RED and self.right_color == Color.RED)
                or (self.left_color == Color.WHITE and self.right_color == Color.WHITE)
            )
            if boundary:
                hit_confirm += 1
                if hit_confirm >= bound_confirm:
                    break
            else:
                hit_confirm = 0

            # Brief wait to avoid saturating the loop while driving
            wait(5)

        # Stop driving
        self.drivebase.stop()

        # Final distance pushed in mm
        pushed = abs(self.drivebase.distance())

        # Record as a single movement in the can path so backtracking reverses it precisely
        if self.can_recording and pushed > 0:
            self.can_path.append(("move", pushed))

        return pushed

    def green_spill_ending(self):
        # Reset spill retry backoff on successful entry
        self._spill_retry_backoff_ms = CONSTANTS.get("SPILL_RETRY_COOLDOWN_MS", 1200)
        # Begin path recording at spill entry to enable exact return later
        self.can_start_recording()
        # Remember entry heading to face spill back later
        try:
            self.spill_entry_heading = self.hub.imu.heading()
        except Exception:
            self.spill_entry_heading = None
        # move off the line a bit, verify green
        self.can_move_forward(10)
        self.get_colors()
        in_spill = (
            (self.left_color == Color.GREEN and self.right_color == Color.GREEN)
            or (self.left_color == Color.GRAY and self.right_color == Color.GRAY)
        )
        if not in_spill:
            # Not truly in green/gray spill: reverse recorded path to entry and abort
            self.can_backtrack()
            return

        # slow settings for precision
        self.drivebase.settings(
            straight_speed=80,
            straight_acceleration=450,
            turn_rate=140,
            turn_acceleration=1600
        )

        self.rotate_arm(-86, stop_method=Stop.HOLD)  # Arm up

        # center in spill
        self.can_move_forward(280)
        self.can_move_forward(-20)
        self.stop_motors()

        # --- Two fast 360s, then slow sweeps; on hit, sweep edges and take midpoint ---
        mid_angle, min_dist = self.find_can_edges_midpoint(
            sweep_deg=160, threshold=CONSTANTS["CAN_SCAN_MAX_MM"]
        )
        if mid_angle is None:
            # nothing found; exit without more spinning
            self.settings_default()
            return

        # Approach along the detected midpoint angle
        self.approach_can_at_angle(mid_angle, stop_offset_mm=20)

        # Grab
        self.rotate_arm(-95, stop_method=Stop.COAST, wait=True)

        # Push the can toward the back of the spill until boundary (two RED or two WHITE), then release
        push_forward = self.push_can_to_back_of_spill_until_boundary(
            step_mm=CONSTANTS["CAN_PUSH_STEP_MM"],
            bound_confirm=CONSTANTS["CAN_PUSH_CONFIRM"],
            max_push_mm=CONSTANTS["CAN_PUSH_MAX_MM"],
        )

        # Open/release and back up a bit to clear the can
        self.rotate_arm(180)
        self.can_move_forward(-CONSTANTS["CAN_RELEASE_BACK_MM"])  # clear the can

        # Return exactly to the spill entry by reversing the recorded path
        self.can_backtrack()

        # Face outward (opposite of entry heading) and drive forward to exit the green spill so line follow can resume
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

        # Nudge backward a little so the line reacquire starts just after the spill boundary
        self.move_forward(-CONSTANTS["LINE_REACQUIRE_BACK_MM"])
        # If we accidentally stepped back onto green, step forward minimally to clear it
        self.get_colors()
        if self.left_color == Color.GREEN and self.right_color == Color.GREEN:
            self.exit_green_spill_forward(
                step_mm=CONSTANTS["SPILL_EXIT_STEP_MM"],
                confirm=CONSTANTS["SPILL_EXIT_CONFIRM"],
                max_mm=CONSTANTS["SPILL_EXIT_STEP_MM"] * 3,
            )

        # Try to reacquire the line after exiting the spill by oscillating turns
        self.reacquire_line_oscillate(
            step_deg=CONSTANTS["LINE_REACQUIRE_TURN_STEP_DEG"],
            max_deg=CONSTANTS["LINE_REACQUIRE_TURN_MAX_DEG"],
            confirm=CONSTANTS["LINE_REACQUIRE_CONFIRM"],
        )

        # Resume default settings and line following
        self.settings_default()
        self.robot_state = "line"

    def exit_green_spill_forward(self, step_mm=20, confirm=3, max_mm=400):
        """From within a green spill, drive forward until both sensors are not green.
        Uses a small confirmation to avoid flapping.
        """
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
        """After exiting green, oscillate left/right around current heading to find the black line.
        Turns incrementally without driving forward. Returns True if found, else False.
        """
        # Helper to confirm line for N consecutive reads
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

        # Start centered
        current_offset = 0
        if confirm_line(confirm):
            return True

        # Sweep left/right with increasing amplitude
        amp = step_deg
        while amp <= max_deg:
            # Left to -amp
            delta = -amp - current_offset
            if delta != 0:
                self.sharp_turn_in_degrees(delta)
                current_offset += delta
            if confirm_line(confirm):
                return True

            # Right to +amp
            delta = amp - current_offset
            if delta != 0:
                self.sharp_turn_in_degrees(delta)
                current_offset += delta
            if confirm_line(confirm):
                return True

            amp += step_deg

        return False

    def align_to_line_in_place(self, timeout_ms=1500, err_tol=3):
        """Turn in place using PD on sensor reflectance until centered on the line.
        Does not drive forward to avoid overshooting across the line.
        """
        start = self.clock.time()
        prev_err = 0
        kp = CONSTANTS.get("LINE_KP", 3.2)
        kd = 0.0  # keep stable while stationary
        max_turn = CONSTANTS.get("LINE_MAX_TURN_RATE", 320)
        while self.clock.time() - start < timeout_ms:
            self.get_colors()
            left_ref = self.left_color_sensor_information["reflection"]
            right_ref = self.right_color_sensor_information["reflection"]
            error = (left_ref - right_ref) if not self.on_inverted else (right_ref - left_ref)
            d_err = error - prev_err
            prev_err = error
            if abs(error) <= err_tol:
                break
            turn_rate = kp * error + kd * d_err
            if turn_rate > max_turn:
                turn_rate = max_turn
            elif turn_rate < -max_turn:
                turn_rate = -max_turn
            # rotate in place
            self.drivebase.drive(0, turn_rate)
            wait(10)
        self.drivebase.stop()

    def avoid_obstacle(self):
        # Pause and lift arm to avoid snagging
        self.stop_motors()
        self.rotate_arm(-90)

        # Initial bypass turn and large curve around obstacle
        self.sharp_turn_in_degrees(CONSTANTS["OBSTACLE_INITIAL_TURN_DEGREES"])
        self.drivebase.curve(
            CONSTANTS["CURVE_RADIUS_OBSTACLE"],
            -CONSTANTS["OBSTACLE_TURN_DEGREES"],
            Stop.BRAKE,
            True,
        )

        # Stop after the avoidance arc, then reacquire by turning in place only (no forward motion)
        self.drivebase.stop()
        found = self.reacquire_line_oscillate(
            step_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_STEP_DEG", 10),
            max_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_MAX_DEG", 120),
            confirm=CONSTANTS.get("LINE_REACQUIRE_CONFIRM", 2),
        )
        if not found:
            # Bias a final turn inward and try once more, still without moving forward
            self.turn_in_degrees(CONSTANTS["OBSTACLE_FINAL_TURN_DEGREES"])
            found = self.reacquire_line_oscillate(
                step_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_STEP_DEG", 10),
                max_deg=CONSTANTS.get("LINE_REACQUIRE_TURN_MAX_DEG", 120),
                confirm=CONSTANTS.get("LINE_REACQUIRE_CONFIRM", 2),
            )
        # If we located the line, center on it before resuming
        if found:
            self.align_to_line_in_place(timeout_ms=1500, err_tol=3)

        # Resume line following
        self.robot_state = "line"
        # Schedule arm to come back down later using clock time (ms)
        self.move_arm_back_after_obstacle_time = self.clock.time() + CONSTANTS["OBSTACLE_ARM_RETURN_DELAY"]
    
    def update(self):
        self.get_colors()
        # Obstacle logic uses a median-filtered distance and a small confirmation window
        self.ultrasonic = self.read_ultra_mm_obstacle()
        # Debug ultrasonic print removed
        # Battery warning (printed once when crossing threshold)
        if not hasattr(self, "_battery_warned"):
            self._battery_warned = False
            self._last_batt_status = 0
        pct, volts = self.battery_percent()
        if pct is not None:
            now = self.clock.time()
            # periodic status
            if now - self._last_batt_status > CONSTANTS.get("BATTERY_STATUS_INTERVAL_MS", 30000):
                print("Battery:", f"{pct}%", f"({volts:.2f} V)")
                self._last_batt_status = now
            # low warning
            if (pct < CONSTANTS.get("BATTERY_WARN_PERCENT", 75)) and (not self._battery_warned):
                print("WARNING: Battery low:", f"{pct}%", f"({volts:.2f} V)")
                self._battery_warned = True
        # previous_state is updated after move() executes

        # Timed arm return after obstacle
        if self.move_arm_back_after_obstacle_time is not None:
            if self.clock.time() >= self.move_arm_back_after_obstacle_time:
                self.move_arm_back_after_obstacle_time = None
                self.rotate_arm(90, stop_method=Stop.COAST)

        # Enter spill only when both GREEN (or both GRAY if explicitly enabled)
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

        elif ALLOW_YELLOW and (self.left_color == Color.YELLOW or self.right_color == Color.YELLOW):
            self.robot_state = "yellow line"
            if not self.shortcut_information["is following shortcut"]:
                if self.left_color == Color.YELLOW:
                    self.turn_in_degrees(-90)
                    if self.shortcut_information["first turned"] is None:
                        self.shortcut_information["first turned"] = "left"
                if self.right_color == Color.YELLOW:
                    self.turn_in_degrees(90)
                    if self.shortcut_information["first turned"] is None:
                        self.shortcut_information["first turned"] = "right"
            self.shortcut_information["is following shortcut"] = True
        
        elif not self.shortcut_information["is following shortcut"] and \
            self.left_color in [Color.WHITE, Color.BLACK, Color.GRAY] and \
            self.right_color in [Color.WHITE, Color.BLACK, Color.GRAY] and \
            not (self.left_color == self.right_color and self.left_color == Color.GRAY):
            self.robot_state = "line"

        elif self.left_color == Color.GREEN or self.right_color == Color.GREEN:
            # Debounce green detection to avoid flapping
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
                # In cooldown, ignore transient greens
                self.robot_state = "line"
        else:
            self.robot_state = "line"

        # Only consider black-crossing turns when actively following a yellow shortcut
        if self.shortcut_information["is following shortcut"]:
            if self.left_color == Color.BLACK:
                self.shortcut_information["left seen black since"] = True
            if self.right_color == Color.BLACK:
                self.shortcut_information["right seen black since"] = True

            if (
                self.shortcut_information["left seen black since"]
                and self.shortcut_information["right seen black since"]
            ):
                first = self.shortcut_information.get("first turned")
                if first == "left":
                    self.turn_in_degrees(90)
                elif first == "right":
                    self.turn_in_degrees(-90)
                self.robot_state = "line"
                self.shortcut_information = self.default_shortcut_information.copy()

    def move(self):
        # Announce state transitions with distinct beeps
        if self.robot_state != self.previous_state:
            self.announce_state(self.robot_state)

        if self.robot_state == "gray":
            self.green_spill_ending()
        elif self.robot_state == "obstacle":
            self.avoid_obstacle()
        elif self.shortcut_information["is following shortcut"]:
            self.follow_color()
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

        # Record this state as the last seen
        self.previous_state = self.robot_state

    def debug(self):
        # No debug printing per request
        pass

    def run(self):
        self.battery_display()
        # Reset arm position if needed using a valid stop method
        self.rotate_arm(180, Stop.COAST)
        # Try to standardize heading at start for consistent gyro behavior
        try:
            # On flat surface, this helps align IMU heading to 0
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
        # Always stop motors on exit/error to avoid runaway
        try:
            robot.stop_motors()
        except Exception:
            pass


if __name__ == "__main__":
    main()
