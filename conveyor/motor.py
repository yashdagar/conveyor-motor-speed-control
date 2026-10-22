class DCMotor:
    def __init__(self, resistance=2.0, inductance=0.02, kt=0.5, ke=0.5, vmax=24.0):
        self.resistance = resistance
        self.inductance = inductance
        self.kt = kt
        self.ke = ke
        self.vmax = vmax
        self.current = 0.0

    def step(self, voltage, omega, dt):
        voltage = max(0.0, min(self.vmax, voltage))
        di = (voltage - self.resistance * self.current - self.ke * omega) / self.inductance
        self.current += di * dt
        return self.kt * self.current
