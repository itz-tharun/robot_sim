import math
import matplotlib.pyplot as plt


def wrap_angle(angle):
    """Wrap angle to [-pi, pi] safely."""
    return math.atan2(math.sin(angle), math.cos(angle))


def simulate(k_v, k_theta):
    # Initial state
    x = 0.0
    y = 0.0
    theta = 0.0

    # Target
    x_target = 5.0
    y_target = 5.0

    dt = 0.05
    max_steps = 2000

    xs = [x]
    ys = [y]

    for _ in range(max_steps):

        # --- Compute errors ---
        dx = x_target - x
        dy = y_target - y

        distance_error = math.sqrt(dx**2 + dy**2)

        theta_target = math.atan2(dy, dx)
        theta_error = wrap_angle(theta_target - theta)

        # Stop condition
        if distance_error < 0.05:
            break

        # --- Proportional control ---
        v = k_v * distance_error
        omega = k_theta * theta_error

        # --- Optional saturation (VERY IMPORTANT) ---
        v = max(min(v, 2.0), -2.0)
        omega = max(min(omega, 5.0), -5.0)

        # --- Update state ---
        theta += omega * dt
        theta = wrap_angle(theta)

        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt

        xs.append(x)
        ys.append(y)

    return xs, ys


# ======= Run Different Gain Configurations =======

configs = [
    (1, 0.05),    # Smooth curve
    (0.5, 0.5),    # Spiral case
    (20, 20)      # Aggressive
]

for k_v, k_theta in configs:
    xs, ys = simulate(k_v, k_theta)

    plt.figure()
    plt.plot(xs, ys, label="Robot path")
    plt.scatter(5, 5, color="red", label="Target")
    plt.title(f"k_v={k_v}, k_theta={k_theta}")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

plt.show()
