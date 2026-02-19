import math
import matplotlib.pyplot as plt

def wrap_angle(angle):
    """Wrap angle to [-pi, pi] safely."""
    return math.atan2(math.sin(angle), math.cos(angle))

I = 1 # Moment of inertia
M = 1 # Mass of the robot 
k_p= 4.0 # Proportional gain for position control
k_theta= 4.0 # Proportional gain for orientation control
k_d= 0.1 # damping for critical damping use (K_d = 2*sqrt(I*K_theta)) = 4
dt = 0.01 # Time step


def simulate(x_tar, y_tar):

    # Initialize state
    x = 0 
    y = 0
    theta = 0
    xp=[x]
    yp=[y]

    omega = 0
    v = 0

    while abs(x - x_tar) > 0.0005 or abs(y - y_tar) > 0.0005:

        dx = x_tar - x
        dy = y_tar - y
        e_p = math.sqrt(dx**2 + dy**2)
        theta_tar = math.atan2(dy, dx)
        e_theta = wrap_angle(theta_tar - theta)

        dif_e_theta = -omega
        dif_e_p = - v 


        torque_theta = k_theta * e_theta + k_d * dif_e_theta
        alpha = torque_theta / I
        omega += alpha * dt
        theta += omega * dt
        theta = wrap_angle(theta)


        if abs(e_theta) < math.radians(15):
            torque_p = k_p * e_p + k_d * dif_e_p
            a = torque_p / M
            v += a * dt
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt

        xp.append(x)
        yp.append(y)      

    return xp, yp

if __name__== "__main__":
    x_tar = int(input("Enter X"))
    y_tar = int(input("Enter Y"))
    x, y = simulate(x_tar, y_tar)

    plt.plot(x, y, label="Robot path")
    plt.scatter(x_tar, y_tar, color="red", label="Target")
    plt.title("Torque_controlled_motion")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()  