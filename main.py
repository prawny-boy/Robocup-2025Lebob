from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Color, Axis, Direction, Stop
from pybricks.tools import StopWatch

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
ROBOT_FORWARD_SPEED = 200            # deg/s motor speedi in degrees per second
PI = 3.14159265358979323846264338327950288419716939937510
LINE_SPEED = ROBOT_FORWARD_SPEED * PI * DRIVEBASE_WHEEL_DIAMETER / 360  # mm/s
KP = 3         # proportional gain
MAX_TURN_RATE = 500  # deg/s
GREEN_TURN_DEGREES = 60  # degrees to turn when green detected

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

    def green(self, dir):
        d = -GREEN_TURN_DEGREES if dir == "left" else GREEN_TURN_DEGREES
        self.drivebase.curve(GREEN_TURN_DEGREES, d, Stop.COAST, True)

    def spill(self):
        # this function assumes that short_turn_in_degrees() turns on the spot, not curves
        max_dist_sense = 580 # maximum distance to sense the can
        max_dist = (0, 0) # (distance, degrees)
        min_dist = 0 # distance being sensed
        self.move_forward(50, ROBOT_FORWARD_SPEED) # move forward to get a better view
        deg = -1
        # some parts in this loop may be redundant
        while deg >= (-100):
            self.short_turn_in_degrees(-1) #turning in one degree increments
            #could just remove the loop and turn 100 without waiting, and making a loop that 
            #checks the distance sensor and compares it.
            min_dist = self.ultrasonic_sensor.distance() # might be redundant, can be optimized
            if min_dist > max_dist and min_dist <= max_dist_sense:
                max_dist = (min_dist, deg) # resetting the max distance and logging what degree it was found
            deg -= 1
        if max_dist[0] == 0: # if it is 1, it means that the can was not sensed
            deg = 1 # resets the above loop to turn right and sense the can
            self.short_turn_in_degrees(100) # resetting to the original position
            while deg <= 100:
                self.short_turn_in_degrees(1)
                min_dist = self.ultrasonic_sensor.distance()
                if min_dist > max_dist[0] and min_dist <= max_dist_sense:
                    max_dist = (min_dist, deg)
                deg += 1
            self.short_turn_in_degrees(-(100-max_dist[1])) # turning to the can right
        else:
            self.short_turn_in_degrees(100-max_dist[1]) # turning to the can left
        
        # this part is moving to the can and picking it up
        self.start_motors(ROBOT_FORWARD_SPEED, ROBOT_FORWARD_SPEED)
        start_time = StopWatch.time() # starting when the motors start
        while self.ultrasonic_sensor.distance() > 50: # "do until" loop, might need to change the distance
            pass
        self.stop_motors() # stopping the motors when the can is close enough
        elapsed_time = StopWatch.time() - start_time # calculating the elapsed time
        self.arm_motor.run_angle(500, 0, Stop.HOLD, wait=True) # moving the arm motor down
        self.start_motors(-ROBOT_FORWARD_SPEED, -ROBOT_FORWARD_SPEED) # moving backwards to the original position with the can
        start_time = StopWatch.time() 
        while (StopWatch.time() - start_time) < elapsed_time: # making sure that the robot moves back for 
                                                              # the same amount of time it moved forward
            pass
        self.stop_motors()
        self.short_turn_in_degrees(-max_dist[1]) # turning back to the original position
        self.short_turn_in_degrees(180) # turning around to the other side
        self.move_forward(50, ROBOT_FORWARD_SPEED) # moving forward to exit the spill area
        self.stop_motors() 
        self.short_turn_in_degrees(45) # turning the can away from the line and allowing the robot to continue
        self.stop_motors()
        self.arm_motor.run_angle(-500, 0, Stop.HOLD, wait=True) # lifting the arm motor back up
        self.short_turn_in_degrees(-45) # turning back to the original position
        self.robot_state = "straight" # resetting the robot state to straight
        return



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
        if self.left_color == Color.GREEN:
            self.robot_state = "green left"
            return
        if self.right_color == Color.GREEN:
            self.robot_state = "green right"
            return
        if self.right_color == Color.GRAY or self.left_color == Color.GRAY:
            self.robot_state = "spill"
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

    def move(self):
        if self.robot_state == "obstacle":
            self.avoid_obstacle()

        elif self.robot_state in ("green left", "green right"):
            self.stop_motors()
            self.green(self.robot_state.split()[1])

        elif self.robot_state == "spill":
            self.stop_motors()
            self.spill()

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
