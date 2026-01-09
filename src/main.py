import numpy as np
import matplotlib.pyplot as plt

from pid import PID
from plant import RoboticJoint
from metrics import overshoot, settling_time


def simulate_step_response(
    theta_target=np.deg2rad(45.0),
    T=5.0,
    dt=0.001,
    kp=20.0,
    ki=5.0,
    kd=1.0,
    torque_limit=5.0,
):
    joint = RoboticJoint(J=0.01, b=0.1)
    pid = PID(kp, ki, kd, u_min=-torque_limit, u_max=torque_limit)

    n_steps = int(T / dt)
    t = np.linspace(0.0, T, n_steps)
    theta_hist = []
    theta_dot_hist = []
    u_hist = []

    for k in range(n_steps):
        error = theta_target - joint.theta
        u = pid.step(error, dt)

        theta, theta_dot = joint.step(u, dt)

        theta_hist.append(theta)
        theta_dot_hist.append(theta_dot)
        u_hist.append(u)

    theta_hist = np.array(theta_hist)
    theta_dot_hist = np.array(theta_dot_hist)
    u_hist = np.array(u_hist)

    return t, theta_hist, theta_dot_hist, u_hist, theta_target


def main():
    # Parameters can be tuned to show different behaviours
    t, theta, theta_dot, u, target = simulate_step_response(
        theta_target=np.deg2rad(45.0),
        T=4.0,
        dt=0.001,
        kp=30.0,
        ki=10.0,
        kd=2.0,
        torque_limit=10.0,
    )

    # Compute performance metrics
    os = overshoot(theta, target)
    ts = settling_time(t, theta, target, tol=0.02)

    print(f"Target angle: {np.rad2deg(target):.1f} deg")
    print(f"Overshoot: {os:.2f}%")
    print(f"Settling time (2% band): {ts:.3f} s")

    # Plot results
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    axs[0].plot(t, np.rad2deg(theta), label="Joint angle")
    axs[0].axhline(np.rad2deg(target), color="r", linestyle="--", label="Target")
    axs[0].set_ylabel("Angle [deg]")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(t, np.rad2deg(theta_dot))
    axs[1].set_ylabel("Angular velocity [deg/s]")
    axs[1].grid(True)

    axs[2].plot(t, u)
    axs[2].set_ylabel("Torque [Nm]")
    axs[2].set_xlabel("Time [s]")
    axs[2].grid(True)

    fig.suptitle("PID Control of a Robotic Joint (Step Response)", fontsize=14)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save to file for your README
    import os
    os.makedirs("results", exist_ok=True)
    fig.savefig("results/joint_pid_step_response.png", dpi=200)

    plt.show()


if __name__ == "__main__":
    main()
