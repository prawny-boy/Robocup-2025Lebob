from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

# Ports must match your build (align with main.py)
PORT_LEFT_COLOR = Port.D
PORT_RIGHT_COLOR = Port.A


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


def capture_when_right(hub):
    while True:
        btn = wait_button(hub)
        if btn == Button.RIGHT:
            beep_ok(hub)
            return


def live_capture_color(hub, sensor: ColorSensor, label: str):
    """Capture a single snapshot when RIGHT is pressed; no live prints."""
    print(f"Place sensor over {label} then press RIGHT")
    capture_when_right(hub)
    # Take a short burst and compute medians
    refs, hs, ss, vs = [], [], [], []
    for _ in range(5):
        try:
            refs.append(sensor.reflection())
            hsv = sensor.hsv()
            hs.append(float(getattr(hsv, 'h', 0)))
            ss.append(float(getattr(hsv, 's', 0)))
            vs.append(float(getattr(hsv, 'v', 0)))
        except Exception:
            refs.append(0)
            hs.append(0.0)
            ss.append(0.0)
            vs.append(0.0)
        wait(10)

    def med(a):
        a = sorted(a)
        return a[len(a)//2]

    return {
        "reflection": int(med(refs)),
        "h": float(med(hs)),
        "s": float(med(ss)),
        "v": float(med(vs)),
    }


def calibrate_color_sensor(hub, sensor: ColorSensor, name: str):
    print(f"\n=== Calibrate {name} Color Sensor ===")
    white = live_capture_color(hub, sensor, f"{name} WHITE")
    black = live_capture_color(hub, sensor, f"{name} BLACK")
    yellow = live_capture_color(hub, sensor, f"{name} YELLOW")
    green = live_capture_color(hub, sensor, f"{name} GREEN")
    return {"white": white, "black": black, "yellow": yellow, "green": green}


def compute_constants(left_cal, right_cal):
    # Reflection threshold between black and white
    mids = []
    try:
        mids.append((left_cal["black"]["reflection"] + left_cal["white"]["reflection"]) / 2.0)
    except Exception:
        pass
    try:
        mids.append((right_cal["black"]["reflection"] + right_cal["white"]["reflection"]) / 2.0)
    except Exception:
        pass
    line_black_ref_threshold = int(round(sum(mids) / len(mids))) if mids else 35

    # Gray cutoff based on bright white
    whites = []
    for cal in (left_cal, right_cal):
        try:
            whites.append(float(cal["white"]["reflection"]))
        except Exception:
            pass
    gray_min = int(round(min(99.0, max(60.0, (min(whites) * 0.95) if whites else 99.0))))

    # Yellow window
    y_hues = []
    for cal in (left_cal, right_cal):
        try:
            y_hues.append(float(cal["yellow"]["h"]))
        except Exception:
            pass
    if y_hues:
        y_center = sum(y_hues) / len(y_hues)
        y_width = 12.0
        y_min = max(0.0, y_center - y_width)
        y_max = min(360.0, y_center + y_width)
    else:
        y_min, y_max = 45.0, 67.0

    # Green window and saturation min
    g_hues, g_sats = [], []
    for cal in (left_cal, right_cal):
        try:
            g_hues.append(float(cal["green"]["h"]))
            g_sats.append(float(cal["green"]["s"]))
        except Exception:
            pass
    if g_hues:
        g_center = sum(g_hues) / len(g_hues)
        g_min = max(0.0, g_center - 15.0)
        g_max = min(360.0, g_center + 15.0)
    else:
        g_min, g_max = 135.0, 165.0
    g_s_min = int(round(max(10.0, min(90.0, (sum(g_sats)/len(g_sats) * 0.6) if g_sats else 30.0))))

    return {
        "LINE_BLACK_REF_THRESHOLD": line_black_ref_threshold,
        "GRAY_REFLECTION_MIN": gray_min,
        "YELLOW_HUE_MIN": int(round(y_min)),
        "YELLOW_HUE_MAX": int(round(y_max)),
        "YELLOW_HUE_CENTER": round(((y_min + y_max) / 2.0), 1),
        "YELLOW_HUE_WIDTH": round(((y_max - y_min) / 2.0), 1),
        "GREEN_HUE_MIN": int(round(g_min)),
        "GREEN_HUE_MAX": int(round(g_max)),
        "GREEN_S_MIN": g_s_min,
    }


def print_constants_block(constants: dict):
    print("\n=== Summary (paste into CONSTANTS in main.py) ===")
    for k, v in constants.items():
        if isinstance(v, float):
            print(f'    "{k}": {v},')
        else:
            print(f'    "{k}": {int(v)},')


def main():
    hub = PrimeHub()
    print("SPIKE/Pybricks Color Sensor Calibration")
    print("- RIGHT captures a reading")
    print("- LEFT finishes and prints constants at the end")
    print("- CENTER is ignored")

    wait_release(hub)

    left = ColorSensor(PORT_LEFT_COLOR)
    right = ColorSensor(PORT_RIGHT_COLOR)

    left_cal = calibrate_color_sensor(hub, left, "LEFT")
    wait(300)
    right_cal = calibrate_color_sensor(hub, right, "RIGHT")

    print("\nPress LEFT when done to print the constants block.")
    while True:
        btn = wait_button(hub)
        if btn == Button.LEFT:
            consts = compute_constants(left_cal, right_cal)
            print_constants_block(consts)
            beep_ok(hub)
            break


if __name__ == "__main__":
    main()

