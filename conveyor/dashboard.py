import sys

BLOCKS = " ▁▂▃▄▅▆▇█"


def bar(value, limit, width=30):
    n = int(round(max(0.0, min(1.0, value / limit)) * width))
    return "[" + "#" * n + "." * (width - n) + "]"


def spark(values, limit):
    out = ""
    for v in values:
        i = int(max(0.0, min(1.0, v / limit)) * (len(BLOCKS) - 1))
        out += BLOCKS[i]
    return out


def render(name, t, setpoint, speed, voltage, vmax, current, torque, boxes, history, duty=None):
    limit = max(setpoint * 1.5, 0.1)
    lines = [
        f"Conveyor Motor Speed Control   controller={name}   t={t:6.2f}s",
        "",
        f"setpoint  {setpoint:6.3f} m/s",
        f"speed     {speed:6.3f} m/s  {bar(speed, limit)}",
        f"error     {setpoint - speed:+6.3f} m/s",
        f"voltage   {voltage:6.2f} V    {bar(voltage, vmax)}",
        f"current   {current:6.2f} A",
        f"torque    {torque:6.2f} N.m",
        f"duty      {'-' if duty is None else f'{duty * 100:5.1f} %'}",
        f"boxes     {boxes} on belt",
        "",
        f"speed     {spark(history, limit)}",
        f"          0 .. {limit:.2f} m/s, last {len(history)} samples",
        "",
        "Ctrl+C to stop",
    ]
    sys.stdout.write("\x1b[H\x1b[J" + "\n".join(lines) + "\n")
    sys.stdout.flush()
