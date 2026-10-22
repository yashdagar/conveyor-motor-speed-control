import argparse
import csv
import time
from collections import deque

from .controllers import CONTROLLERS
from .dashboard import render
from .metrics import fmt, summarize
from .motor import DCMotor
from .sim import DT, Conveyor


def run(args, controller_name, show):
    sim = Conveyor(gui=args.gui, boxes=args.boxes, load_at=args.load_at)
    motor = DCMotor()
    ctrl = CONTROLLERS[controller_name](motor.vmax)
    history = deque(maxlen=40)
    log = []
    steps = int(args.duration / DT)
    every = int(0.1 / DT)
    start = time.time()
    try:
        for i in range(steps):
            speed = sim.speed()
            voltage = ctrl.update(args.setpoint, speed, DT)
            torque = motor.step(voltage, sim.omega(), DT)
            sim.step(torque)
            log.append((sim.time, speed, voltage, motor.current, torque))
            if i % every == 0:
                history.append(speed)
                if show:
                    render(ctrl.name, sim.time, args.setpoint, speed, voltage, motor.vmax, motor.current,
                           torque, sim.boxes_on_belt(), history, getattr(ctrl, "duty", None))
            if not args.fast:
                lag = start + sim.time - time.time()
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        pass
    sim.close()
    return log


def write_log(path, log, setpoint):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "setpoint", "speed", "voltage", "current", "torque"])
        for t, v, u, i, tau in log:
            w.writerow([f"{t:.4f}", setpoint, f"{v:.5f}", f"{u:.3f}", f"{i:.4f}", f"{tau:.4f}"])


def print_summary(results):
    keys = next(iter(results.values())).keys()
    print(f"{'metric':22}" + "".join(f"{n:>14}" for n in results))
    for k in keys:
        print(f"{k:22}" + "".join(f"{fmt(results[n][k]):>14}" for n in results))


def main():
    ap = argparse.ArgumentParser(prog="conveyor", description="Conveyor motor speed control in PyBullet")
    ap.add_argument("controller", nargs="?", choices=list(CONTROLLERS) + ["compare"], default="pid")
    ap.add_argument("--setpoint", type=float, default=0.5, help="target belt speed in m/s")
    ap.add_argument("--duration", type=float, default=15.0, help="simulated seconds")
    ap.add_argument("--boxes", type=int, default=3, help="number of load boxes")
    ap.add_argument("--load-at", type=float, default=0.0, help="second at which the boxes are dropped on the belt")
    ap.add_argument("--gui", action="store_true", help="open the PyBullet 3D window")
    ap.add_argument("--fast", action="store_true", help="do not pace the simulation to wall-clock time")
    ap.add_argument("--log", help="write a CSV of time, setpoint, speed, voltage, current, torque")
    args = ap.parse_args()

    if args.controller == "compare":
        args.fast = True
        results = {}
        for name in CONTROLLERS:
            log = run(args, name, show=False)
            results[name] = summarize(log, args.setpoint, args.load_at)
            if args.log:
                write_log(args.log.replace(".csv", f"_{name}.csv"), log, args.setpoint)
        print(f"setpoint {args.setpoint} m/s, {args.boxes} boxes at t={args.load_at}s\n")
        print_summary(results)
        return

    log = run(args, args.controller, show=True)
    if args.log:
        write_log(args.log, log, args.setpoint)
    print()
    for k, v in summarize(log, args.setpoint, args.load_at).items():
        print(f"{k:22}{fmt(v)}")


if __name__ == "__main__":
    main()
