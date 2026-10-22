def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class BangBang:
    name = "bang-bang"

    def __init__(self, vmax, band=0.02):
        self.vmax = vmax
        self.band = band
        self.on = False

    def update(self, setpoint, speed, dt):
        if speed < setpoint - self.band:
            self.on = True
        elif speed > setpoint + self.band:
            self.on = False
        return self.vmax if self.on else 0.0


class PWM:
    name = "pwm"

    def __init__(self, vmax, kp=4.0, hz=50.0):
        self.vmax = vmax
        self.kp = kp
        self.period = 1 / hz
        self.phase = 0.0
        self.duty = 0.0

    def update(self, setpoint, speed, dt):
        if self.phase == 0.0:
            self.duty = clamp(self.kp * (setpoint - speed), 0.0, 1.0)
        on = self.phase < self.duty * self.period
        self.phase += dt
        if self.phase >= self.period - 1e-9:
            self.phase = 0.0
        return self.vmax if on else 0.0


class PID:
    name = "pid"

    def __init__(self, vmax, kp=40.0, ki=150.0, kd=0.5):
        self.vmax = vmax
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = None

    def update(self, setpoint, speed, dt):
        error = setpoint - speed
        self.integral = clamp(self.integral + error * dt, 0.0, self.vmax / self.ki)
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(out, 0.0, self.vmax)


CONTROLLERS = {"bangbang": BangBang, "pwm": PWM, "pid": PID}
