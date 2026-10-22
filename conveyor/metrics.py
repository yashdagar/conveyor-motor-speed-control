def summarize(log, setpoint):
    times = [r[0] for r in log]
    speeds = [r[1] for r in log]
    torques = [r[2] for r in log]
    tail = len(log) // 5
    rise = next((t for t, v in zip(times, speeds) if v >= 0.9 * setpoint), None)
    peak = max(speeds)
    settled = speeds[-tail:]
    switches = sum(1 for a, b in zip(torques, torques[1:]) if (a > 0) != (b > 0))
    return {
        "rise_time_s": rise,
        "overshoot_pct": max(0.0, (peak - setpoint) / setpoint * 100),
        "steady_state_error": sum(setpoint - v for v in settled) / len(settled),
        "ripple": max(settled) - min(settled),
        "mean_torque": sum(torques) / len(torques),
        "switches": switches,
    }


def fmt(value):
    if value is None:
        return "never"
    if isinstance(value, int):
        return str(value)
    return f"{value:.4f}"
