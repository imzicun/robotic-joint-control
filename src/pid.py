class PID:
    """
    Simple PID controller for position control of a robotic joint.
    u = Kp*e + Ki*∫e dt + Kd*de/dt
    """
    def __init__(self, kp, ki, kd, u_min=None, u_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_min = u_min
        self.u_max = u_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_call = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_call = True

    def step(self, error, dt):
        # Proportional
        p = self.kp * error

        # Integral
        self.integral += error * dt
        i = self.ki * self.integral

        # Derivative
        if self.first_call:
            d = 0.0
            self.first_call = False
        else:
            de = (error - self.prev_error) / dt
            d = self.kd * de

        self.prev_error = error
        u = p + i + d

        # Optional saturation
        if self.u_min is not None:
            u = max(self.u_min, u)
        if self.u_max is not None:
            u = min(self.u_max, u)

        return u

