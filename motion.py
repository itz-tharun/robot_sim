import math
import random
import matplotlib.pyplot as plt


def simulate():
    # State (stored in radians)
    x = 0.0
    y = 0.0
    theta = 0.0
    xs=[0]
    ys=[0]
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

            # Update orientation without noise (first)
            theta += omega * dt

            # Normalize angle (optional but good practice)
            theta = math.atan2(math.sin(theta), math.cos(theta))

            # Update position with noise
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt

            xs.append(x)
            ys.append(y)
            
        print(f"\nFinal state:")
        print(f"x = {x:.3f} m and x_noise = {x_noisy:.3f} m")
        print(f"y = {y:.3f} m and y_noise = {y_noisy:.3f} m")
        print(f"theta = {theta:.3f} rad ({math.degrees(theta):.2f} deg) and theta_noise = {theta_noisy:.3f} rad ({math.degrees(theta_noisy):.2f} deg)\n")
    return xs,ys,xs_noisy,ys_noisy
    

if __name__ == "__main__":
    xs,ys,xs_noisy,ys_noisy=simulate()

    plt.plot(xs,ys,label="Ideal") 

    plt.plot(xs_noisy,ys_noisy,label="Noisy")

    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Robot trajectory")
    plt.grid(True)
    plt.legend()
    plt.show()