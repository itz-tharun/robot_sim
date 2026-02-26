import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def motion_model(x_state, u, dt):
    # DO NOT modify input state directly
    theta = x_state[2]
    
    x_new = np.zeros(3)
    x_new[0] = x_state[0] + u[0]*np.cos(theta)*dt
    x_new[1] = x_state[1] + u[0]*np.sin(theta)*dt
    x_new[2] = wrap_angle(theta + u[1]*dt)
    
    return x_new

def main():

    time = 10.0
    dt = 0.01
    time_steps = int(time / dt)

    x_state = np.array([0.0, 0.0, 0.0])
    x_estimated_state = x_state.copy()   # FIXED (added parentheses)

    u = np.array([1.0, 0.1])

    x_estimated_trajectory = [0]
    y_estimated_trajectory = [0]
    x_actual_trajectory = [0]
    y_actual_trajectory = [0]

    for i in range(time_steps):

        # control noise (applied to TRUE robot)
        n = np.array([
            np.random.normal(0, 0.05),
            np.random.normal(0, 0.02)
        ])

        u_noisy = u + n

        # TRUE robot moves with noisy control
        x_state = motion_model(x_state, u_noisy, dt)

        # ESTIMATE assumes perfect control
        x_estimated_state = motion_model(x_estimated_state, u, dt)

        x_actual_trajectory.append(x_state[0])
        y_actual_trajectory.append(x_state[1])

        x_estimated_trajectory.append(x_estimated_state[0])
        y_estimated_trajectory.append(x_estimated_state[1])

    plt.figure()
    plt.plot(x_estimated_trajectory, y_estimated_trajectory, label='Estimated Trajectory')
    plt.plot(x_actual_trajectory, y_actual_trajectory, label='Actual Trajectory')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.title('Estimated vs Actual Trajectory')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()