import math
import matplotlib.pyplot as plt

kp = 0.5
ktheta = 2.0
dt = 0.01

def ang_wrap(ang):
    return math.atan2(math.sin(ang),math.cos(ang))


def simulate(x_tar,y_tar):

    x = 0
    y = 0
    theta = 0
    xp=[x]
    yp=[y]

    while abs(x - x_tar) > 0.0005 or abs(y - y_tar) > 0.0005:

        dx = x_tar - x
        dy = y_tar - y
        e_p = math.sqrt((dx)**2 + (dy)**2)
        theta_tar = math.atan2(dy,dx)
        e_theta = ang_wrap(theta_tar - theta)

        omega = ktheta * e_theta

        theta+= omega*dt
        theta=ang_wrap(theta)
        if e_theta < math.radians(15) :
            v = kp * e_p
            x+= v*math.cos(theta)*dt
            y+= v*math.sin(theta)*dt


        xp.append(x)
        yp.append(y)

    return xp,yp


if __name__== "__main__":
    x_tar = int(input("Enter X"))
    y_tar = int(input("Enter Y"))
    x,y = simulate(x_tar,y_tar)

    plt.plot(x, y, label="Robot path")
    plt.scatter(x_tar, y_tar, color="red", label="Target")
    plt.title("Velocity)controlled_motion")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.show()