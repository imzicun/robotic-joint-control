import numpy as np

class RoboticJoint:
    """
    Simple 1-DOF rotational joint model:
        J * theta_ddot + b * theta_dot = u
    where:
        J = inertia
        b = viscous friction
        u = control torque
    State = [theta, theta_dot]
    """
    def __init__(self, J=0.01, b=0.1):
        self.J = J
        self.b = b
        self.theta = 0.0
        self.theta_dot = 0.0

    def reset(self, theta=0.0, theta_dot=0.0):
        self.theta = theta
        self.theta_dot = theta_dot

    def step(self, u, dt):
        """
        Advance the joint dynamics one time step using simple Euler integration.
        """
        theta_ddot = (u - self.b * self.theta_dot) / self.J
        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt
        return self.theta, self.theta_dot
