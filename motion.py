import math
import random
import matplotlib.pyplot as plt


def simulate():
    # State (stored in radians)
    x_noisy=0.0
    y_noisy=0.0
    theta_noisy=0.0
    xs_noisy=[0]
    ys_noisy=[0]
    
    dt = 0.1  # small time step (seconds)

    print("Differential Drive Motion Simulator")
    print("Theta is stored in radians\n")

    while True:
        cmd = input("Press 1 to move, anything else to exit: ")
        if cmd != "1":
            break

        v = float(input("Enter linear velocity v (m/s): "))
        omega = float(input("Enter angular velocity ω (rad/s): "))
        T = float(input("Enter duration (seconds): "))

        steps = int(T / dt)

        for _ in range(steps):
            # Adding noise to motion
            v_noisy = v + random.gauss(0, 0.05)
            omega_noisy = omega + random.gauss(0, 0.02)

            
            # Update orientation with noisy (first)
            theta_noisy += omega_noisy * dt

            # Normalize angle (optional but good practice)
            theta_noisy = math.atan2(math.sin(theta_noisy), math.cos(theta_noisy))

            # Update position with noise
            x_noisy += v_noisy * math.cos(theta_noisy) * dt
            y_noisy += v_noisy * math.sin(theta_noisy) * dt

            xs_noisy.append(x_noisy)
            ys_noisy.append(y_noisy)
            
        print(f"\nFinal state:")
        print(f"x = {x_noisy:.3f} m")
        print(f"y = {y_noisy:.3f} m")
        print(f"theta = {theta_noisy:.3f} rad ({math.degrees(theta_noisy):.2f} deg)\n")
    return xs_noisy,ys_noisy
    

if __name__ == "__main__":
    xs_noisy,ys_noisy=simulate()
    plt.plot(xs_noisy,ys_noisy)
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Robot trajectory")
    plt.grid(True)
    plt.show()