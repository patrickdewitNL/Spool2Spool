"""
Spool2Spool Controller -- live serial telemetry plotter

Reads the once-per-second telemetry line printed by both spool2spool-esp32.ino and
spool2spool-esp32-debug.ino (same format in both), and live-plots current value vs.
setpoint for each control loop -- meant for watching overshoot/settling while tuning
Kp (motor 2's speed loop) and KpAngle (motor 1's angle trim).

The ESP32 prints lines shaped like:
    RPM: 123.45 | Speed(m/min): 6.78 | TargetSpeed(m/min): 5.00 | Motor2 PWM: 130 | \
    Mode2: AUTO | Angle: 45.0 | TargetAngle: 45.0 | Motor1 PWM: 130 | Mode1: AUTO | \
    Potval1: 2048 | Potval2: 1024
This script only reads the serial port -- it never writes to it, so it's safe to run
alongside (or instead of) the Arduino IDE's own Serial Monitor (though most OSes only
let one program hold the port open at a time -- close the IDE's monitor first).

Usage:
    pip install pyserial matplotlib
    python plot_serial.py --port COM5
    python plot_serial.py --port COM5 --baud 115200 --window 120
"""

import argparse
import re
import time
from collections import deque

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# matches "Key: value" segments, stopping each value at the next " | " or end of line --
# the key charset covers everything used in the telemetry line, e.g. "Speed(m/min)"
FIELD_RE = re.compile(r"([A-Za-z0-9_ /()]+?):\s*([^|]+)")


def parse_line(line):
    """Turn a 'Key: value | Key: value | ...' line into {key: float-or-string}."""
    fields = {}
    for key, value in FIELD_RE.findall(line):
        key = key.strip()
        value = value.strip()
        try:
            fields[key] = float(value)
        except ValueError:
            fields[key] = value  # e.g. "AUTO" / "MAN"
    return fields


def main():
    parser = argparse.ArgumentParser(description="Live-plot Spool2Spool ESP32 telemetry")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=120,
                         help="How many samples (roughly seconds, at 1/s) to keep on screen")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)

    t0 = time.monotonic()
    times = deque(maxlen=args.window)
    speed = deque(maxlen=args.window)
    target_speed = deque(maxlen=args.window)
    motor2_pwm = deque(maxlen=args.window)
    angle = deque(maxlen=args.window)
    target_angle = deque(maxlen=args.window)
    motor1_pwm = deque(maxlen=args.window)

    # box style for the live numeric readouts drawn on top of each subplot
    box = dict(boxstyle="round", facecolor="white", edgecolor="gray", alpha=0.85)

    fig, ((ax_speed, ax_pwm2), (ax_angle, ax_pwm1)) = plt.subplots(2, 2, figsize=(11, 7))

    line_speed, = ax_speed.plot([], [], label="Speed (m/min)")
    line_target_speed, = ax_speed.plot([], [], "--", label="Target speed")
    ax_speed.set_title("Motor 2 speed loop")
    ax_speed.set_ylabel("m/min")
    ax_speed.legend(loc="upper left")
    txt_speed = ax_speed.text(0.98, 0.95, "", transform=ax_speed.transAxes,
                               ha="right", va="top", fontsize=9, family="monospace", bbox=box)

    line_pwm2, = ax_pwm2.plot([], [], color="tab:orange", label="Motor 2 PWM")
    ax_pwm2.set_title("Motor 2 PWM")
    ax_pwm2.set_ylabel("PWM (0-255)")
    ax_pwm2.set_ylim(0, 255)
    ax_pwm2.legend(loc="upper left")
    txt_pwm2 = ax_pwm2.text(0.98, 0.95, "", transform=ax_pwm2.transAxes,
                             ha="right", va="top", fontsize=9, family="monospace", bbox=box)

    line_angle, = ax_angle.plot([], [], label="Angle (deg)")
    line_target_angle, = ax_angle.plot([], [], "--", label="Target angle")
    ax_angle.set_title("Motor 1 angle trim")
    ax_angle.set_xlabel("time (s)")
    ax_angle.set_ylabel("degrees")
    ax_angle.legend(loc="upper left")
    txt_angle = ax_angle.text(0.98, 0.95, "", transform=ax_angle.transAxes,
                               ha="right", va="top", fontsize=9, family="monospace", bbox=box)

    line_pwm1, = ax_pwm1.plot([], [], color="tab:green", label="Motor 1 PWM")
    ax_pwm1.set_title("Motor 1 PWM")
    ax_pwm1.set_xlabel("time (s)")
    ax_pwm1.set_ylabel("PWM (0-255)")
    ax_pwm1.set_ylim(0, 255)
    ax_pwm1.legend(loc="upper left")
    txt_pwm1 = ax_pwm1.text(0.98, 0.95, "", transform=ax_pwm1.transAxes,
                             ha="right", va="top", fontsize=9, family="monospace", bbox=box)

    # RPM, modes, and raw pot readings aren't plotted anywhere -- show them as one line at
    # the top of the figure instead, refreshed the same way as everything else
    suptitle = fig.suptitle("", family="monospace", fontsize=10)

    fig.tight_layout(rect=(0, 0, 1, 0.95))

    latest = {}  # most recent parsed fields, for the numeric readouts

    def update(_frame):
        while ser.in_waiting:
            raw = ser.readline().decode(errors="ignore").strip()
            if "RPM" not in raw:
                continue  # skip boot/debug prints, only care about the telemetry line
            f = parse_line(raw)
            if "Speed(m/min)" not in f:
                continue
            latest.clear()
            latest.update(f)

            t = time.monotonic() - t0
            times.append(t)
            speed.append(f.get("Speed(m/min)", float("nan")))
            target_speed.append(f.get("TargetSpeed(m/min)", float("nan")))
            motor2_pwm.append(f.get("Motor2 PWM", float("nan")))
            angle.append(f.get("Angle", float("nan")))
            target_angle.append(f.get("TargetAngle", float("nan")))
            motor1_pwm.append(f.get("Motor1 PWM", float("nan")))

        if not times:
            return ()

        line_speed.set_data(times, speed)
        line_target_speed.set_data(times, target_speed)
        line_pwm2.set_data(times, motor2_pwm)
        line_angle.set_data(times, angle)
        line_target_angle.set_data(times, target_angle)
        line_pwm1.set_data(times, motor1_pwm)

        for ax in (ax_speed, ax_pwm2, ax_angle, ax_pwm1):
            ax.set_xlim(times[0], max(times[-1], times[0] + 1))
        ax_speed.relim()
        ax_speed.autoscale_view(scalex=False)
        ax_angle.relim()
        ax_angle.autoscale_view(scalex=False)

        # live numeric readouts -- exact current/target values, not just the plotted lines
        txt_speed.set_text(
            f"Speed:  {latest.get('Speed(m/min)', float('nan')):6.2f} m/min\n"
            f"Target: {latest.get('TargetSpeed(m/min)', float('nan')):6.2f} m/min"
        )
        txt_pwm2.set_text(f"PWM: {latest.get('Motor2 PWM', float('nan')):.0f}")
        txt_angle.set_text(
            f"Angle:  {latest.get('Angle', float('nan')):6.2f} deg\n"
            f"Target: {latest.get('TargetAngle', float('nan')):6.2f} deg"
        )
        txt_pwm1.set_text(f"PWM: {latest.get('Motor1 PWM', float('nan')):.0f}")
        suptitle.set_text(
            f"RPM: {latest.get('RPM', float('nan')):6.1f}   "
            f"Mode1: {latest.get('Mode1', '?'):>4}   Mode2: {latest.get('Mode2', '?'):>4}   "
            f"Pot1: {latest.get('Potval1', float('nan')):5.0f}   "
            f"Pot2: {latest.get('Potval2', float('nan')):5.0f}"
        )

        return (line_speed, line_target_speed, line_pwm2, line_angle, line_target_angle,
                line_pwm1, txt_speed, txt_pwm2, txt_angle, txt_pwm1, suptitle)

    ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
    plt.show()

    ser.close()


if __name__ == "__main__":
    main()
