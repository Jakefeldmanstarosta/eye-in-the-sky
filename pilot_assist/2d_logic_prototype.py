import numpy as np
import matplotlib.pyplot as plt

def fly(dt, duration, velocity, starting_position=(0,0)):
    """Simulate the flight of the drone at constant velocity.

    Args:
        dt (float): Length of each time step in seconds.
        duration (float): Duration of the flight in seconds.
        velocity (np.ndarray): Constant velocity (vx, vy) in m/s, shape (2,).
            x is forward, y is sideways.
        starting_position (tuple[float, float]): Starting position (x, y) in
            metres. Defaults to (0, 0).

    Returns:
        np.ndarray: Positions of the drone at each time step, shape (N, 2),
            where N is the number of steps. Each row is (x, y) in metres.
    """

    # how many steps required to complete the duration at step size dtdt
    steps = round((duration / dt) + 1)

    # create a position vector using numpy array of size (steps) x 2. array contains a position for every step in time. every position is given as a vector (x,y)
    position = np.zeros((steps, 2))

    # set starting position at first position vector at t=0 (x0,y0) in position array
    position[0] = starting_position

    # loop to simulate motion through time. start at second position since initial position is given. loop as many times as steps forward to reach total duration
    for step in range(1, steps):
        # set the current position equal to the previous position plus the difference in position in time dt (pos2 = pos1 + dpos = pos1 + vdt)
        position[step] = position[step-1] + velocity * dt

    # returns the numpy array with teh simulated flight position data 
    return position

def plot(position):
    """Plot the drone's flight path from above.

    Forward (x) is drawn up the screen and sideways (y) across it.

    Args:
        position (np.ndarray): Positions of the drone at each time step,
            shape (N, 2). Each row is (x, y) in metres.
    """

    fig, ax = plt.subplots()
    # slice position array by columns, taking the column of x positions through time and column of y positions through time, and plotting them. 
    # plot y column as x-axis (horizontal) and x column as y-axis (forward) (drone convention).
    ax.plot(position[:,1], position[:,0], '-o')
    ax.axis('equal')
    plt.show()


position = fly(0.1, 10, np.array([5,0]))
plot(position)