from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button, Direction
from pybricks.tools import wait

# Match your build and geometry (same as main.py)
PORT_LEFT = Port.E
PORT_RIGHT = Port.C
WHEEL_DIAMETER_MM = 56.0
AXLE_TRACK_MM = 112.0

# Test profile (deg/s on motors, duration ms)
TEST_SPEED_DEG_S = 600
TEST_DURATION_MS = 1500


def beep_ok(hub):
    try:
        hub.speaker.beep(800, 80)
    except Exception:
        pass


def wait_release(hub):
    while hub.buttons.pressed():
        wait(20)


def wait_button(hub):
    """Wait for LEFT or RIGHT, ignoring CENTER entirely."""
    while True:
        pressed = hub.buttons.pressed() or set()
        if Button.LEFT in pressed:
            wait_release(hub)
            return Button.LEFT
        if Button.RIGHT in pressed:
            wait_release(hub)
            return Button.RIGHT
        wait(20)


def compute_trim(delta_heading_deg: float, duration_ms: int, speed_deg_s: int,
                 wheel_d_mm: float, axle_track_mm: float):
    """Compute symmetric motor trim factors given observed heading drift.
    Returns (left_trim, right_trim).
    k = - δθ_deg * axle_track / (S * D * 60 * t)
    TL = 1 - k, TR = 1 + k
    Values are clamped to [0.90, 1.10].
    """
    t = max(0.001, duration_ms / 1000.0)
    S = max(1.0, float(speed_deg_s))
    D = max(1.0, float(wheel_d_mm))
    track = max(1.0, float(axle_track_mm))
    k = - (delta_heading_deg) * track / (S * D * 60.0 * t)
    TL = 1.0 - k
    TR = 1.0 + k
    # Clamp and round for readability
    TL = max(0.90, min(1.10, TL))
    TR = max(0.90, min(1.10, TR))
    return round(TL, 3), round(TR, 3)


def print_constants_block(left_trim: float, right_trim: float):
    print("\n=== Suggested motor trim (paste into CONSTANTS in main.py) ===")
    print(f'    "LEFT_MOTOR_TRIM": {left_trim},')
    print(f'    "RIGHT_MOTOR_TRIM": {right_trim},')


def main():
    hub = PrimeHub()
    print("SPIKE/Pybricks Motor Calibration (trim for straight drive)")
    print("- RIGHT starts a short straight run")
    print("- LEFT prints suggested trim constants at the end")
    print("Notes: Keep robot on flat surface. Keep clear while it moves.")

    left = Motor(PORT_LEFT, positive_direction=Direction.COUNTERCLOCKWISE)
    right = Motor(PORT_RIGHT)

    print("Press RIGHT to run the test.")
    while True:
        btn = wait_button(hub)
        if btn == Button.RIGHT:
            try:
                try:
                    hub.imu.reset_heading(0)
                except Exception:
                    pass
                left.run(TEST_SPEED_DEG_S)
                right.run(TEST_SPEED_DEG_S)
                wait(TEST_DURATION_MS)
            finally:
                # Always stop motors
                try:
                    left.stop()
                    right.stop()
                except Exception:
                    pass
            wait(150)
            try:
                delta = float(hub.imu.heading())
            except Exception:
                delta = 0.0
            beep_ok(hub)
            break

    # Wait for LEFT to print
    print("\nPress LEFT to print suggested trim constants.")
    while True:
        btn = wait_button(hub)
        if btn == Button.LEFT:
            LT, RT = compute_trim(delta, TEST_DURATION_MS, TEST_SPEED_DEG_S,
                                   WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
            print_constants_block(LT, RT)
            beep_ok(hub)
            break


if __name__ == "__main__":
    main()

