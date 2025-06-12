from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Stop

# constants
DRIVEBASE_WHEEL_DIAMETER = 56
DRIVEBASE_AXLE_TRACK = 112
ROBOT_SPEED = 500
ROBOT_ACCELERATION = 750
ROBOT_TURN_RATE = 750
ROBOT_TURN_ACCELERATION = 3000
LOW_VOLTAGE = 7000
HIGH_VOLTAGE = 8300

# line following tuning
ROBOT_FORWARD_SPEED = 250           # deg/s motor speedi in degrees per second
PI = 3.14159265358979323846264338327950288419716939937510
LINE_SPEED = ROBOT_FORWARD_SPEED * PI * DRIVEBASE_WHEEL_DIAMETER / 360  # mm/s
KP = 3         # proportional gain
MAX_TURN_RATE = 500  # deg/s
GREEN_TURN_DEGREES = 120  # degrees to turn when green detected
GREEN_TURN_RADIUS = 60
GREEN_TURN_SPEED = 100

# ports
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
            DRIVEBASE_WHEEL_DIAMETER,
            DRIVEBASE_AXLE_TRACK
        )
        self.drivebase.use_gyro(True)
        self.drivebase.settings(
            ROBOT_SPEED,
            ROBOT_ACCELERATION,
            ROBOT_TURN_RATE,
            ROBOT_TURN_ACCELERATION
        )
        self.robot_state = "obstacle"
        self.green_cooldown = 0
        self.green_time = 0

    def intro_sound(self):
        self.hub.speaker.volume(100)
        intro = ["C3/4.", "B2/16_", "C3/16", "D3/16_", "C3/16", "B2/16_",
                 "A2/16", "C3/16", "R/16", "C3/16_", "A2/16", "C3/4"]
        pattern = ["C3/8", "R/8"] * 4
        motif = ["E4/8_", "D4/8_", "C4/8_", "D4/8_", "E4/8_", "D4/8_", "C4/8_", "B3/8_"]
        variation1 = ["G4/4_", "G4/4.", "A4/8_", "G4/8_", "F4/4_", "E4/4."]
        variation2 = ["G4/2_", "A4/2_", "B4/2_", "C5/2_", "B4/2_", "A4/2_"]
        finale = ["C5/1_", "R/8", "C5/8_", "B4/8_", "A4/8_", "G4/4_", "C4/2_"]
        notes = intro + (pattern + motif) * 18 + variation1 + variation2 + finale
        self.hub.speaker.play_notes(notes)

    def battery_display(self):
        v = self.hub.battery.voltage()
        vPct = rescale(v, LOW_VOLTAGE, HIGH_VOLTAGE, 1, 100)
        print(f"Battery %: {round(vPct, 1)}, Voltage: {v}")
        if vPct < 70:
            if vPct < 40:
                print("EMERGENCY: BATTERY LOW!")
                self.status_light(Color.RED)
            else:
                print("Battery below 70% - please charge")
                self.status_light(Color.YELLOW)
        else:
            self.status_light(Color.GREEN)

    def status_light(self, color):
        self.hub.light.off()
        self.hub.light.on(color)

    def turn_in_degrees(self, degrees):
        self.drivebase.curve(25, degrees, Stop.COAST, False)

    def short_turn_in_degrees(self, degrees):
        self.drivebase.turn(degrees)

    def move_forward(self, distance, speed=None):
        if speed:
            self.drivebase.settings(
                speed, ROBOT_ACCELERATION, ROBOT_TURN_RATE, ROBOT_TURN_ACCELERATION)
        self.drivebase.straight(distance)

    def start_motors(self, ls, rs):
        if self.left_drive.speed() != ls:
            self.left_drive.run(ls)
        if self.right_drive.speed() != rs:
            self.right_drive.run(rs)

    def stop_motors(self):
        self.left_drive.stop()
        self.right_drive.stop()

    def information_to_color(self, info):
        if info["color"] == Color.WHITE:
            return Color.WHITE
        if info["color"] == Color.GREEN and 135 < info["hsv"].h < 165:
            return Color.GREEN
        if info["color"] == Color.BLUE and info["hsv"].s > 80 and info["hsv"].v > 80:
            return Color.BLUE
        if info["color"] == Color.RED and info["hsv"].s > 85:
            return Color.RED
        if info["color"] == Color.RED:
            return Color.ORANGE
        if info["hsv"].s < 20 and 3 < info["reflection"] < 30:
            return Color.BLACK
        return Color.NONE

    def get_colors(self):
        l = {"reflection": self.color_sensor_left.reflection(),
             "color": self.color_sensor_left.color(),
             "hsv": self.color_sensor_left.hsv()}
        r = {"reflection": self.color_sensor_right.reflection(),
             "color": self.color_sensor_right.color(),
             "hsv": self.color_sensor_right.hsv()}
        self.left_color = self.information_to_color(l)
        self.right_color = self.information_to_color(r)

    def one_green(self, dir):
        # original code
        # d = -GREEN_TURN_DEGREES if dir == "left" else GREEN_TURN_DEGREES
        # self.drivebase.curve(GREEN_TURN_DEGREES, d, Stop.COAST, False)

        # better edition maybe, doesnt work yet
        direction = -GREEN_TURN_DEGREES if dir == "left" else GREEN_TURN_DEGREES
        self.drivebase.settings(turn_rate=GREEN_TURN_SPEED)
        self.drivebase.curve(GREEN_TURN_RADIUS, direction, Stop.COAST, False)
        # if dir == "left":
        #     offset_left = 20
        #     offset_right = 0
        # else:
        #     offset_right = 20
        #     offset_left = 0
        # self.start_motors(direction_speed - offset_left, direction_speed - offset_right)
        ticks = 0
        while not self.drivebase.done():    
            self.get_colors()
            if ticks > 18000/GREEN_TURN_SPEED:
                if dir == "left":
                    if self.right_color == Color.BLACK:
                        break
                if dir == "right":
                    if self.left_color == Color.BLACK:
                        break
            ticks += 1
            print(ticks)
        self.stop_motors()
        self.turn_in_degrees(-5 if dir=="right" else 5)
        self.green_cooldown = 180
        self.green_time = 0
        self.drivebase.settings(turn_rate=ROBOT_TURN_RATE)

    def both_green(self):
        self.turn_in_degrees(360)
        self.green_time = 0

    def follow_line(self):
        l = self.color_sensor_left.reflection()
        r = self.color_sensor_right.reflection()
        err = l - r
        rate = max(min(KP * err, MAX_TURN_RATE), -MAX_TURN_RATE)
        self.drivebase.drive(LINE_SPEED, rate)

    def avoid_obstacle(self):
        self.stop_motors()
        self.short_turn_in_degrees(90)
        self.drivebase.curve(180, -150, Stop.BRAKE, True)
        self.start_motors(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED)
        self.get_colors()
        while self.right_color == Color.WHITE:
            self.get_colors()
        self.turn_in_degrees(1)
        self.robot_state = "straight"
        self.arm_motor.run_angle(-500, 0, Stop.HOLD, wait=True)

    def update(self):
        self.get_colors()
        self.ultrasonic = self.ultrasonic_sensor.distance()
        if self.ultrasonic < 80:
            self.robot_state = "obstacle"
            return
        if self.green_cooldown == 0:
            if self.left_color == Color.GREEN and self.right_color == Color.GREEN:
                self.robot_state = "green both"
                return
            if self.left_color == Color.GREEN:
                self.green_time += 1
                print(self.green_time)
                self.robot_state = f"green left"
                return
            if self.right_color == Color.GREEN:
                self.green_time += 1
                print(self.green_time)
                self.robot_state = f"green right"
                return
        on_l = self.left_color == Color.WHITE
        on_r = self.right_color == Color.WHITE
        prev = self.robot_state
        if on_l and on_r:
            self.robot_state = "straight"
        elif on_l:
            self.robot_state = "new right"
        elif on_r:
            self.robot_state = "new left"
        else:
            self.robot_state = prev
        if self.green_cooldown > 0:
            self.green_cooldown -= 1

    def move(self):
        if self.robot_state == "obstacle":
            pass
            # self.avoid_obstacle()

        elif self.robot_state in ("green left", "green right"):
            if self.green_time > 15:
                self.one_green(self.robot_state.split()[1])
        if self.robot_state == "green both":
            self.both_green()
        else:
            self.follow_line()

    def debug(self):
        print(f"LC:{self.left_color} RC:{self.right_color} U:{self.ultrasonic} S:{self.robot_state}")

    def run(self):
        while True:
            self.update()
            self.move()
            self.debug()


def rescale(v, imin, imax, omin, omax):
    sign = v/abs(v)
    v = abs(v)
    v = max(min(v, imax), imin)
    val = (v-imin)*(omax/(imax-imin))
    val = max(min(val, omax), omin)
    return val*sign


def main():
    robot = Robot()
    robot.battery_display()
    robot.arm_motor.run_until_stalled(500, duty_limit=30)
    print("Calibrating...")
    robot.run()


main()
