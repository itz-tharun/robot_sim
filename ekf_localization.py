import numpy as np
import matplotlib.pyplot as plt


# -----------------------------
# Utility
# -----------------------------
def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


# -----------------------------
# Motion Model (Unicycle)
# -----------------------------
def motion_model(x, u, dt):
    theta = x[2]
    v, omega = u

    x_new = np.zeros(3)
    x_new[0] = x[0] + v * np.cos(theta) * dt
    x_new[1] = x[1] + v * np.sin(theta) * dt
    x_new[2] = wrap_angle(theta + omega * dt)

    return x_new


# -----------------------------
# Measurement Model (Range-Bearing)
# -----------------------------
def measurement_model(x, landmark):
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]

    r = np.sqrt(dx**2 + dy**2)
    phi = wrap_angle(np.arctan2(dy, dx) - x[2])

    return np.array([r, phi])


# -----------------------------
# Measurement Jacobian
# -----------------------------
def measurement_jacobian(x, landmark):
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]

    q = dx**2 + dy**2
    r = np.sqrt(q)

    H = np.zeros((2, 3))

    # Range derivatives
    H[0, 0] = -dx / r
    H[0, 1] = -dy / r
    H[0, 2] = 0

    # Bearing derivatives
    H[1, 0] = dy / q
    H[1, 1] = -dx / q
    H[1, 2] = -1

    return H


# -----------------------------
# Main Simulation
# -----------------------------
def main():

    dt = 0.1
    max_time = 60
    steps = int(max_time / dt)

    # World setup
    landmark = np.array([5.0, 5.0])
    goal = np.array([8.0, 10.0])

    # True and estimated states
    x_true = np.array([0.0, 0.0, 0.0])
    x_est  = np.array([0.0, 0.0, 0.0])

    # Initial covariance
    P = np.diag([0.1, 0.1, 0.05])

    # Process and measurement noise
    Q = np.diag([0.001, 0.001, 0.0005])
    R = np.diag([0.2**2, np.deg2rad(5)**2])

    # Control gains
    k_p = 0.8
    k_theta = 2.0

    true_traj = []
    est_traj = []

    for _ in range(steps):

        # ---------------------------------
        # CONTROLLER (using estimated pose)
        # ---------------------------------
        dx = goal[0] - x_est[0]
        dy = goal[1] - x_est[1]

        rho = np.sqrt(dx**2 + dy**2)

        if rho < 0.1:
            print("Goal reached.")
            break

        theta_target = np.arctan2(dy, dx)
        e_theta = wrap_angle(theta_target - x_est[2])

        v = k_p * rho
        omega = k_theta * e_theta

        # Optional velocity limits
        v = np.clip(v, -2.0, 2.0)
        omega = np.clip(omega, -3.0, 3.0)

        u = np.array([v, omega])

        # ---------------------------------
        # TRUE MOTION (with control noise)
        # ---------------------------------
        control_noise = np.random.multivariate_normal(
            mean=[0, 0],
            cov=np.diag([0.05**2, 0.02**2])
        )

        u_noisy = u + control_noise
        x_true = motion_model(x_true, u_noisy, dt)

        # ---------------------------------
        # SENSOR SIMULATION
        # ---------------------------------
        z_true = measurement_model(x_true, landmark)

        measurement_noise = np.random.multivariate_normal(
            mean=[0, 0],
            cov=R
        )

        z_measured = z_true + measurement_noise
        z_measured[1] = wrap_angle(z_measured[1])

        # ---------------------------------
        # EKF PREDICTION
        # ---------------------------------
        x_pred = motion_model(x_est, u, dt)

        theta = x_pred[2]

        F = np.array([
            [1, 0, -v*np.sin(theta)*dt],
            [0, 1,  v*np.cos(theta)*dt],
            [0, 0, 1]
        ])

        P_pred = F @ P @ F.T + Q

        # ---------------------------------
        # EKF UPDATE
        # ---------------------------------
        H = measurement_jacobian(x_pred, landmark)

        z_pred = measurement_model(x_pred, landmark)

        y = z_measured - z_pred
        y[1] = wrap_angle(y[1])

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_est = x_pred + K @ y
        x_est[2] = wrap_angle(x_est[2])

        I = np.eye(3)
        P = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R @ K.T

        true_traj.append(x_true.copy())
        est_traj.append(x_est.copy())

    true_traj = np.array(true_traj)
    est_traj = np.array(est_traj)

    # ---------------------------------
    # Plot
    # ---------------------------------
    plt.figure(figsize=(7,7))
    plt.plot(true_traj[:,0], true_traj[:,1], label="True Path")
    plt.plot(est_traj[:,0], est_traj[:,1], label="EKF Estimate")
    plt.scatter(goal[0], goal[1], c='green', marker='o', label="Goal")
    plt.scatter(landmark[0], landmark[1], c='red', marker='x', label="Landmark")
    plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.title("EKF-Based Navigation to Goal")
    plt.show()


if __name__ == "__main__":
    main()