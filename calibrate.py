from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Button, Stop
from pybricks.tools import wait


# Ports must match your build
PORT_LEFT = Port.E
PORT_RIGHT = Port.C
PORT_ARM = Port.F


def beep_ok(hub):
    try:
        hub.speaker.beep(800, 80)
    except Exception:
        pass


def beep_err(hub):
    try:
        hub.speaker.beep(300, 120)
    except Exception:
        pass


def wait_release(hub):
    # Wait until no buttons are pressed
    while True:
        pressed = hub.buttons.pressed()
        if not pressed:
            return
        wait(20)


def wait_button(hub):
    """Wait for a button press, then return a single Button after release.
    Handles that pressed() returns a set; prioritizes LEFT/RIGHT/CENTER.
    """
    while True:
        pressed = hub.buttons.pressed()
        if pressed:
            # pressed is a set; choose deterministically
            if Button.LEFT in pressed:
                btn = Button.LEFT
            elif Button.RIGHT in pressed:
                btn = Button.RIGHT
            elif Button.CENTER in pressed:
                btn = Button.CENTER
            else:
                # Fallback to some element
                try:
                    btn = next(iter(pressed))
                except Exception:
                    btn = None
            # Debounce: wait until released
            while hub.buttons.pressed():
                wait(20)
            if btn is not None:
                return btn
        wait(20)


def adjust_value(hub, label, value, step, min_val=None, max_val=None):
    """Adjust a numeric value using LEFT/RIGHT; press LEFT+RIGHT together to confirm.
    This avoids using CENTER, which can stop programs on some hubs.
    """
    while True:
        print(f"{label}: {value}")
        # Wait for a button interaction
        while True:
            pressed = hub.buttons.pressed() or set()
            if Button.LEFT in pressed and Button.RIGHT in pressed:
                # Confirm selection
                beep_ok(hub)
                # Debounce: wait release
                while hub.buttons.pressed():
                    wait(20)
                return value
            elif Button.LEFT in pressed:
                nv = value - step
                if min_val is not None:
                    nv = max(min_val, nv)
                value = nv
                beep_ok(hub)
                # Debounce: wait release
                while hub.buttons.pressed():
                    wait(20)
                break
            elif Button.RIGHT in pressed:
                nv = value + step
                if max_val is not None:
                    nv = min(max_val, nv)
                value = nv
                beep_ok(hub)
                # Debounce: wait release
                while hub.buttons.pressed():
                    wait(20)
                break
            else:
                # Ignore center/others
                pass
            wait(20)


def calibrate_wheel_diameter(hub):
    print("=== Wheel Diameter Calibration ===")
    print("Use LEFT/RIGHT to set your current wheel diameter (mm). Press LEFT+RIGHT together to accept.")
    wheel_d = adjust_value(hub, "Current wheel diameter (mm)", 56.0, 0.5, 30.0, 120.0)

    left = Motor(PORT_LEFT)
    right = Motor(PORT_RIGHT)
    db = DriveBase(left, right, wheel_d, 112.0)
    try:
        db.use_gyro(True)
    except Exception:
        pass
    db.settings(straight_speed=150, straight_acceleration=500)

    # Command distance
    commanded = 1000  # mm
    print("Place robot on a straight path. RIGHT to drive 1000 mm. LEFT to skip.")
    btn = wait_button(hub)
    if btn != Button.RIGHT:
        print("Skipped wheel calibration.")
        beep_err(hub)
        return None
    print("Driving straight...")
    db.straight(commanded)
    beep_ok(hub)

    print("Measure actual distance with a tape.")
    actual = adjust_value(hub, "Measured distance (mm)", commanded, 10.0, 100.0, 5000.0)
    if actual <= 0:
        print("Invalid measurement; skipping.")
        beep_err(hub)
        return None
    new_wheel_d = wheel_d * (actual / commanded)
    print(f"Suggested DRIVEBASE_WHEEL_DIAMETER: {new_wheel_d:.2f} mm (was {wheel_d})")
    wait(400)
    return new_wheel_d


def calibrate_axle_track(hub):
    print("=== Axle Track Calibration ===")
    print("Use LEFT/RIGHT to set your current axle track (mm). Press LEFT+RIGHT together to accept.")
    axle = adjust_value(hub, "Current axle track (mm)", 112.0, 1.0, 60.0, 200.0)

    # Also ask for wheel diameter so DriveBase kinematics are correct during turn
    wheel_d = adjust_value(hub, "Wheel diameter (mm)", 56.0, 0.5, 30.0, 120.0)

    left = Motor(PORT_LEFT)
    right = Motor(PORT_RIGHT)
    db = DriveBase(left, right, wheel_d, axle)
    try:
        db.use_gyro(False)  # ensure turn relies on geometry
    except Exception:
        pass
    # Reset heading for measurement
    try:
        hub.imu.reset_heading(0)
    except Exception:
        pass

    print("Clear space. RIGHT to turn 360°. LEFT to skip.")
    btn = wait_button(hub)
    if btn != Button.RIGHT:
        print("Skipped axle calibration.")
        beep_err(hub)
        return None
    print("Turning 360°...")
    db.turn(360)
    wait(200)
    # Read measured heading
    try:
        measured = hub.imu.heading()
    except Exception:
        measured = 360.0
    print(f"Measured heading change: {measured}°")
    if measured <= 0:
        print("Invalid IMU reading; skipping.")
        beep_err(hub)
        return None
    new_axle = axle * (measured / 360.0)
    print(f"Suggested DRIVEBASE_AXLE_TRACK: {new_axle:.2f} mm (was {axle})")
    # Re-enable gyro for normal use
    try:
        db.use_gyro(True)
    except Exception:
        pass
    wait(400)
    return new_axle


def calibrate_arm_zero(hub):
    print("=== Arm Zero Calibration (optional) ===")
    print("RIGHT: zero the arm to mechanical stop and set angle=0. LEFT: skip.")
    btn = wait_button(hub)
    if btn != Button.RIGHT:
        print("Skipped arm calibration.")
        beep_err(hub)
        return False
    arm = Motor(PORT_ARM)
    print("Moving to stop (hold robot)…")
    try:
        arm.run_until_stalled(-300, then=Stop.COAST, duty_limit=50)
        arm.reset_angle(0)
        print("Arm zeroed. angle=0")
        beep_ok(hub)
        return True
    except Exception as e:
        print("Arm calibration failed:", e)
        beep_err(hub)
        return False


def main():
    hub = PrimeHub()
    print("SPIKE/Pybricks Calibration")
    print("- Use LEFT/RIGHT to adjust values")
    print("- CENTER to confirm")
    print("- LEFT on a prompt to skip a step")

    wait_release(hub)

    new_wd = calibrate_wheel_diameter(hub)
    wait(500)
    new_axle = calibrate_axle_track(hub)
    wait(500)
    calibrate_arm_zero(hub)

    print("\n=== Summary (paste into CONSTANTS in main.py) ===")
    if new_wd is not None:
        print(f'    "DRIVEBASE_WHEEL_DIAMETER": {new_wd:.2f},')
    else:
        print("    (Wheel diameter unchanged)")
    if new_axle is not None:
        print(f'    "DRIVEBASE_AXLE_TRACK": {new_axle:.2f},')
    else:
        print("    (Axle track unchanged)")
    print("All done. Press RIGHT to beep (avoid center).")
    if wait_button(hub) == Button.RIGHT:
        beep_ok(hub)


if __name__ == "__main__":
    main()
