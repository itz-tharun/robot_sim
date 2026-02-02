import math

def simulate():
    # State (stored in radians)
    x = 0.0
    y = 0.0
    theta = 0.0  

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
            # Update orientation first
            theta += omega * dt

            # Normalize angle (optional but good practice)
            theta = math.atan2(math.sin(theta), math.cos(theta))

            # Update position
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt

        print(f"\nFinal state:")
        print(f"x = {x:.3f} m")
        print(f"y = {y:.3f} m")
        print(f"theta = {theta:.3f} rad ({math.degrees(theta):.2f} deg)\n")

if __name__ == "__main__":
    simulate()
