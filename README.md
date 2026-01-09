#⚙️ PID Control of a Robotic Joint
Simulated 1-DOF Rotational Joint + Step Response Analysis

This project simulates a 1-Degree-of-Freedom robotic joint and designs a PID controller to control its position. It models real joint dynamics, integrates them over time, applies PID control, and computes classical control performance metrics such as overshoot and settling time.

🚀 Features

Second-order joint dynamics:

J * θ̈ + b * θ̇ = u


PID controller with:

Proportional

Integral

Derivative terms

Torque saturation

Step response simulation

Plots for angle, angular velocity, and torque

Performance metrics:

Overshoot

Settling time

Clean modular Python architecture

📁 Repository Structure
robotic-joint-pid-control/
│
├── src/
│   ├── pid.py              # PID class
│   ├── plant.py            # Joint dynamic model
│   ├── metrics.py          # Overshoot & settling time
│   └── main.py             # Simulation & plotting
│
├── results/                # Saved step response plots
└── README.md


Generates:

Console output (overshoot, settling time)

Step response plot

Image saved to results/joint_pid_step_response.png

🧠 System Overview
Joint Dynamics

A simplified robotic joint:

J * θ̈ + b * θ̇ = u


J: inertia

b: viscous friction

u: control torque

PID Controller
u = Kp * e + Ki * ∫e dt + Kd * (de/dt)


Where:

e = target angle – current angle

Tuned gains determine overshoot / stability / response time


Industrial automation
