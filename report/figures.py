import argparse
import cmath
import math
import os

import matplotlib
import pybullet as p

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from conveyor import controllers
from conveyor.cli import run
from conveyor.controllers import CONTROLLERS, PID
from conveyor.metrics import summarize
from conveyor.motor import DCMotor
from conveyor.sim import DT, Conveyor

OUT = os.path.join(os.path.dirname(__file__), "figures")
LABELS = {"bangbang": "Bang-bang", "pwm": "PWM", "pid": "PID"}


def args(**kw):
    base = dict(setpoint=0.5, duration=6.0, boxes=3, load_at=0.0, gui=False, fast=True, log=None)
    base.update(kw)
    return argparse.Namespace(**base)


def save(name):
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, name), dpi=160)
    plt.close()


def traces(a, names=CONTROLLERS):
    return {n: run(a, n, show=False) for n in names}


def smooth(values, window=int(0.1 / DT)):
    out = []
    acc = 0.0
    for i, v in enumerate(values):
        acc += v
        if i >= window:
            acc -= values[i - window]
        out.append(acc / min(i + 1, window))
    return out


def plot_logs(logs, setpoint, name, title, tmax=None):
    fig, ax = plt.subplots(3, 1, figsize=(7, 7.5), sharex=True)
    for n, log in logs.items():
        t = [r[0] for r in log]
        ax[0].plot(t, [r[1] for r in log], label=LABELS.get(n, n), lw=1)
        ax[1].plot(t, smooth([r[2] for r in log]), lw=1)
        ax[2].plot(t, smooth([r[3] for r in log]), lw=1)
    ax[0].axhline(setpoint, color="k", ls="--", lw=0.8, label="setpoint")
    ax[0].set_ylabel("belt speed (m/s)")
    ax[1].set_ylabel("voltage, 0.1 s mean (V)")
    ax[2].set_ylabel("current, 0.1 s mean (A)")
    ax[2].set_xlabel("time (s)")
    if tmax:
        ax[2].set_xlim(0, tmax)
    ax[0].legend(loc="lower right", fontsize=8)
    ax[0].set_title(title)
    save(name)


def step_response():
    a = args()
    logs = traces(a)
    plot_logs(logs, a.setpoint, "step.png", "Step to 0.5 m/s with 3 boxes on the belt")
    return {n: summarize(l, a.setpoint, 0.0) for n, l in logs.items()}


def load_step():
    a = args(duration=10.0, load_at=5.0)
    logs = traces(a)
    plot_logs(logs, a.setpoint, "load.png", "Load change: 3 boxes dropped at t = 5 s")
    return {n: summarize(l, a.setpoint, 5.0) for n, l in logs.items()}


def pwm_zoom():
    a = args(duration=4.0)
    logs = traces(a, ["pwm", "bangbang"])
    fig, ax = plt.subplots(3, 1, figsize=(7, 6.5), sharex=True)
    for n, log in logs.items():
        rows = [r for r in log if 3.0 <= r[0] <= 3.2]
        t = [r[0] for r in rows]
        ax[0].step(t, [r[2] for r in rows], where="post", lw=1, label=LABELS[n])
        ax[1].plot(t, [r[3] for r in rows], lw=1)
        ax[2].plot(t, [r[1] for r in rows], lw=1)
    ax[2].axhline(a.setpoint, color="k", ls="--", lw=0.8)
    ax[0].set_ylabel("voltage (V)")
    ax[1].set_ylabel("current (A)")
    ax[2].set_ylabel("belt speed (m/s)")
    ax[2].set_xlabel("time (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Switching detail, 3.0 to 3.2 s")
    save("switching.png")


def saturation():
    fig, ax = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    out = {}
    for sp in (0.5, 0.8, 1.1):
        a = args(setpoint=sp, duration=6.0)
        log = run(a, "pid", show=False)
        out[sp] = summarize(log, sp, 0.0)
        t = [r[0] for r in log]
        ax[0].plot(t, [r[1] for r in log], lw=1, label=f"setpoint {sp} m/s")
        ax[0].axhline(sp, color="k", ls=":", lw=0.6)
        ax[1].plot(t, [r[2] for r in log], lw=1)
    ax[0].set_ylabel("belt speed (m/s)")
    ax[1].set_ylabel("motor voltage (V)")
    ax[1].set_xlabel("time (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("PID with increasing setpoint, 24 V limit")
    save("saturation.png")
    return out


class PIDNoClamp(PID):
    def update(self, setpoint, speed, dt):
        error = setpoint - speed
        self.integral += error * dt
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        return controllers.clamp(self.kp * error + self.ki * self.integral + self.kd * derivative, 0.0, self.vmax)


def setpoint_profile(t):
    return 1.2 if t < 3.0 else 0.5


def windup():
    logs = {}
    for label, cls in (("with anti-windup", PID), ("without anti-windup", PIDNoClamp)):
        sim = Conveyor(boxes=3)
        motor = DCMotor()
        ctrl = cls(motor.vmax)
        log = []
        for _ in range(int(7.0 / DT)):
            sp = setpoint_profile(sim.time)
            speed = sim.speed()
            voltage = ctrl.update(sp, speed, DT)
            sim.step(motor.step(voltage, sim.omega(), DT))
            log.append((sim.time, speed, voltage, sp))
        sim.close()
        logs[label] = log
    fig, ax = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for label, log in logs.items():
        t = [r[0] for r in log]
        ax[0].plot(t, [r[1] for r in log], lw=1, label=label)
        ax[1].plot(t, [r[2] for r in log], lw=1)
    ax[0].plot([r[0] for r in log], [r[3] for r in log], "k--", lw=0.8, label="setpoint")
    ax[0].set_ylabel("belt speed (m/s)")
    ax[1].set_ylabel("motor voltage (V)")
    ax[1].set_xlabel("time (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Integrator wind-up: unreachable 1.2 m/s for 3 s, then 0.5 m/s")
    save("windup.png")
    out = {}
    for label, log in logs.items():
        after = [r for r in log if r[0] >= 3.0]
        back = next((r[0] - 3.0 for r in after if r[1] <= 0.5 + 0.02), None)
        out[label] = {"time_to_reach_0.5": back, "min_voltage_after_drop": min(r[2] for r in after)}
    return out


def tuning():
    a = args(duration=4.0)
    gains = [(10, 20, 0.0), (40, 150, 0.5), (120, 600, 0.5)]
    saved = CONTROLLERS["pid"]
    out = {}
    fig, ax = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for kp, ki, kd in gains:
        CONTROLLERS["pid"] = lambda v, kp=kp, ki=ki, kd=kd: PID(v, kp, ki, kd)
        log = run(a, "pid", show=False)
        out[(kp, ki, kd)] = summarize(log, a.setpoint, 0.0)
        t = [r[0] for r in log]
        ax[0].plot(t, [r[1] for r in log], lw=1, label=f"Kp={kp}, Ki={ki}, Kd={kd}")
        ax[1].plot(t, [r[2] for r in log], lw=1)
    CONTROLLERS["pid"] = saved
    ax[0].axhline(a.setpoint, color="k", ls="--", lw=0.8)
    ax[0].set_ylabel("belt speed (m/s)")
    ax[1].set_ylabel("motor voltage (V)")
    ax[1].set_xlabel("time (s)")
    ax[0].legend(fontsize=8)
    ax[0].set_title("PID gain sets, step to 0.5 m/s")
    save("tuning.png")
    return out


class Constant:
    name = "open-loop"

    def __init__(self, vmax):
        self.vmax = vmax

    def update(self, setpoint, speed, dt):
        return self.vmax


def linear_model():
    m = DCMotor()
    sim = Conveyor(boxes=0)
    r = sim.radius
    n = len(sim.joints)
    sim.close()
    j = n * 0.5 * 2.0 * r ** 2
    b = n * sim.damping
    return m, r, j, b


def open_loop():
    m, r, j, b = linear_model()
    a = args(duration=1.0, boxes=0)
    saved = CONTROLLERS["pid"]
    CONTROLLERS["pid"] = Constant
    log = run(a, "pid", show=False)
    CONTROLLERS["pid"] = saved
    t, i, w = [], 0.0, 0.0
    ts, vs = [], []
    for k in range(int(1.0 / DT)):
        di = (m.vmax - m.resistance * i - m.ke * w) / m.inductance
        dw = (m.kt * i - b * w) / j
        i += di * DT
        w += dw * DT
        ts.append((k + 1) * DT)
        vs.append(w * r)
    plt.figure(figsize=(7, 4))
    plt.plot([x[0] for x in log], [x[1] for x in log], lw=1.2, label="PyBullet, no boxes")
    plt.plot(ts, vs, "--", lw=1.2, label="linear model")
    plt.xlabel("time (s)")
    plt.ylabel("belt speed (m/s)")
    plt.title("Open-loop response to a 24 V step")
    plt.legend(fontsize=8)
    save("openloop.png")
    gain = r * m.kt * m.vmax / (m.resistance * b + m.kt * m.ke)
    return {"model_final": gain, "sim_final": log[-1][1]}


def bode():
    m, r, j, b = linear_model()
    freqs = [10 ** (k / 40) for k in range(-80, 121)]
    mag, phase = [], []
    for f in freqs:
        s = 1j * 2 * math.pi * f
        g = r * m.kt / ((m.inductance * s + m.resistance) * (j * s + b) + m.kt * m.ke)
        mag.append(20 * math.log10(abs(g)))
        phase.append(math.degrees(cmath.phase(g)))
    fig, ax = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    ax[0].semilogx(freqs, mag, lw=1.2)
    ax[0].set_ylabel("|G| (dB, m/s per V)")
    ax[1].semilogx(freqs, phase, lw=1.2)
    ax[1].set_ylabel("phase (deg)")
    ax[1].set_xlabel("frequency (Hz)")
    ax[0].set_title("Bode plot of the voltage-to-belt-speed transfer function")
    ax[0].grid(True, which="both", lw=0.3)
    ax[1].grid(True, which="both", lw=0.3)
    save("bode.png")
    p1 = b / j
    p2 = m.resistance / m.inductance
    return {"mech_pole_rad_s": p1, "elec_pole_rad_s": p2, "dc_gain": r * m.kt / (m.resistance * b + m.kt * m.ke)}


def screenshot():
    sim = Conveyor(boxes=3)
    ctrl = PID(24.0)
    motor = DCMotor()
    for _ in range(int(3.0 / DT)):
        sim.step(motor.step(ctrl.update(0.5, sim.speed(), DT), sim.omega(), DT))
    view = p.computeViewMatrixFromYawPitchRoll([sim.length / 2, 0, 0.1], 1.6, 50, -25, 0, 2)
    proj = p.computeProjectionMatrixFOV(50, 4 / 3, 0.1, 10)
    w, h, rgba, _, _ = p.getCameraImage(960, 720, view, proj, renderer=p.ER_TINY_RENDERER)
    sim.close()
    img = [[rgba[(y * w + x) * 4:(y * w + x) * 4 + 3] for x in range(w)] for y in range(h)]
    plt.imsave(os.path.join(OUT, "pybullet.png"), [[[c / 255 for c in px] for px in row] for row in img])


def main():
    os.makedirs(OUT, exist_ok=True)
    results = {
        "step": step_response(),
        "load": load_step(),
        "saturation": saturation(),
        "windup": windup(),
        "tuning": tuning(),
        "openloop": open_loop(),
        "bode": bode(),
    }
    pwm_zoom()
    screenshot()
    for name, table in results.items():
        print(f"\n== {name}")
        for k, v in table.items():
            print(f"{k}: {v}")


if __name__ == "__main__":
    main()
