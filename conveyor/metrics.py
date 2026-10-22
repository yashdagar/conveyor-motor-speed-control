from .sim import DT


def summarize(log, setpoint, load_at):
    window = int(0.2 / DT)
    times = [r[0] for r in log]
    speeds = [r[1] for r in log]
    volts = [r[2] for r in log]
    tail = len(log) // 5
    rise = next((t for t, v in zip(times, speeds) if v >= 0.9 * setpoint), None)
    settled = speeds[-tail:]
    switches = sum(1 for a, b in zip(volts, volts[1:]) if (a > 0) != (b > 0))
    after = [(t, v) for t, v in zip(times, speeds) if t >= load_at]
    dip = setpoint - min(v for t, v in after[: len(after) // 2]) if load_at > 0 and after else None
    recovery = None
    if load_at > 0:
        for i, (t, v) in enumerate(after):
            if len(after[i:i + window]) == window and all(abs(setpoint - w) <= 0.02 for _, w in after[i:i + window]):
                recovery = t - load_at
                break
    return {
        "rise_time_s": rise,
        "overshoot_pct": max(0.0, (max(speeds) - setpoint) / setpoint * 100),
        "steady_state_error": sum(setpoint - v for v in settled) / len(settled),
        "ripple": max(settled) - min(settled),
        "load_dip": dip,
        "recovery_s": recovery,
        "mean_voltage": sum(volts) / len(volts),
        "switches": switches,
    }


def fmt(value):
    if value is None:
        return "-"
    if isinstance(value, int):
        return str(value)
    return f"{value:.4f}"
