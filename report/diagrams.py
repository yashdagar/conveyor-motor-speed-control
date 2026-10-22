import os

import schemdraw
from schemdraw import dsp, flow

OUT = os.path.join(os.path.dirname(__file__), "figures")


def control_loop():
    with schemdraw.Drawing(show=False) as d:
        d.config(fontsize=10, unit=2)
        d += flow.Box(w=2.2, h=1).label("setpoint\n$v^*$")
        d += dsp.Arrow().right(1.2)
        d += (s := dsp.Sum().anchor("W"))
        d += dsp.Arrow().right(1.2).label("$e$", "top")
        d += flow.Box(w=3.2, h=1.3).label("controller\nbang-bang / PWM / PID")
        d += dsp.Arrow().right(1.2).label("$u$", "top")
        d += flow.Box(w=2.6, h=1.3).label("limit\n0 .. 24 V")
        d += dsp.Arrow().right(1.2).label("$V$", "top")
        d += (m := flow.Box(w=3.0, h=1.3).label("DC motor\n$R, L, K_t, K_e$"))
        d += dsp.Arrow().right(1.2).label(r"$\tau$", "top")
        d += (p := flow.Box(w=3.2, h=1.3).label("rollers + boxes\n(PyBullet)"))
        d += dsp.Arrow().right(1.2).label(r"$\omega$", "top")
        d += (v := flow.Box(w=2.4, h=1.3).label("speed\n$v = r\\,\\bar{\\omega}$"))
        d += dsp.Line().at(v.S).down(1.4)
        d += dsp.Line().left(v.S.x - s.S.x)
        d += dsp.Arrow().up(1.4 - 0.5).label("$-$", "right")
        d += dsp.Line().at(p.N).up(1.0)
        d += dsp.Line().left(p.N.x - m.N.x).label(r"back-EMF $K_e\,\omega$", "top")
        d += dsp.Arrow().down(1.0)
        d.save(os.path.join(OUT, "diagram_loop.png"), dpi=160)


def box(d, x, y, text, w=4.0):
    b = flow.Box(w=w, h=1.4).anchor("N").at((x, y)).label(text)
    d += b
    return b


def elbow(d, start, end, text=""):
    mid = (start.y + end.y) / 2
    d += flow.Line().at(start).down().toy(mid)
    d += (flow.Line().right() if end.x > start.x else flow.Line().left()).tox(end.x)
    d += flow.Arrow().down().toy(end.y).label(text, "right", ofst=0.1)


def architecture():
    with schemdraw.Drawing(show=False) as d:
        d.config(fontsize=10)
        cli = box(d, 0, 0, "cli.py\nargument parsing, main loop,\ncompare mode, CSV log", w=5.4)
        met = box(d, -7.2, -3.4, "metrics.py\nsummarize()")
        ctrl = box(d, -2.4, -3.4, "controllers.py\nBangBang, PWM, PID")
        motor = box(d, 2.4, -3.4, "motor.py\nDCMotor")
        dash = box(d, 7.2, -3.4, "dashboard.py\nrender()")
        sim = box(d, 2.4, -6.8, "sim.py\nConveyor (PyBullet)")
        fig = box(d, 2.4, -10.2, "report/figures.py\nexperiments and plots", w=4.6)
        elbow(d, cli.S, met.N, "at the end")
        elbow(d, cli.S, ctrl.N, "$v^*, v$")
        elbow(d, cli.S, motor.N, "$V, \\omega$")
        elbow(d, cli.S, dash.N, "every 0.1 s")
        d += flow.Arrow().at(motor.S).down(motor.S.y - sim.N.y).label(r"$\tau$", "right", ofst=0.1)
        d += flow.Arrow().at(fig.N).up(sim.S.y - fig.N.y)
        d += flow.Line().at(fig.E).right(10.6 - fig.E.x)
        d += flow.Line().up(cli.E.y - fig.E.y)
        d += flow.Arrow().left(10.6 - cli.E.x).label("runs scenarios", "top")
        d.save(os.path.join(OUT, "diagram_architecture.png"), dpi=160)


def loop_flow():
    with schemdraw.Drawing(show=False) as d:
        d.config(fontsize=10, unit=0.9)
        d += flow.Start(w=3, h=1).label("start run")
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.1).label("create Conveyor, DCMotor, controller\n$t = 0$")
        d += flow.Arrow().down()
        d += (read := flow.Box(w=6, h=1.1).label("read $v = r\\,\\bar{\\omega}$ and $\\omega$ from PyBullet"))
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.1).label("$V$ = controller.update($v^*$, $v$, $\\Delta t$)")
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.1).label("$i = i + \\Delta t\\,(V - Ri - K_e\\omega)/L$,  $\\tau = K_t i$")
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.3).label("apply $\\tau/N - b\\,\\omega_j$ to each roller\nstepSimulation(), spawn / recycle boxes")
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.3).label("log $(t, v, V, i, \\tau)$\nevery 0.1 s redraw the dashboard")
        d += flow.Arrow().down()
        d += (dec := flow.Decision(w=5, h=1.8, E="no", S="yes").label("$t \\geq$ duration\nor Ctrl+C?"))
        d += flow.Arrow().down()
        d += flow.Box(w=6, h=1.1).label("compute metrics, write CSV if asked")
        d += flow.Arrow().down()
        d += flow.Start(w=3, h=1).label("end")
        d += flow.Line().at(dec.E).right(1.2)
        d += flow.Line().up(dec.E.y - read.E.y).to((dec.E.x + 1.2, read.E.y))
        d += flow.Arrow().to(read.E)
        d.save(os.path.join(OUT, "diagram_flow.png"), dpi=160)


if __name__ == "__main__":
    control_loop()
    architecture()
    loop_flow()
