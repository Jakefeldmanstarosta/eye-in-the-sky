import numpy as np
import matplotlib.pyplot as plt

# amount of time to step forward in each loop
dt = 0.1

# function to plot drone flight. takes in flight duration, velocity, and a starting position. Returns the position array after the flight
def fly(duration, velocity, starting_position=(0,0)):
    # how many ticks to get through the duration at dt
    ticks = round((duration / dt) + 1)

    # create a position vector, (ticks) x 2 array in numpy. has a position for every tick. every position is an x,y vector.
    position = np.zeros((ticks, 2))

    # set starting position at first xy vector in position array
    position[0,0], position[0,1] = starting_position[0], starting_position[1]

    # loop through as many times as ticks
    for step in range(1, ticks):
        # set the current position equal to the previous position plus the difference in position in the time dt (dpos = vdt). start at 2nd position.
        position[step] = position[step-1] + velocity * dt

    return position

# function to plot the flight position data. takes in a position array, displays an xy plot
def plot(position):
    fig, ax = plt.subplots()
    # use array of x positions and array fo y positions from position array and plot them
    ax.plot(position[:,0], position[:,1], '-o')
    ax.axis('equal')
    plt.show()


position = fly(10, np.array([0,5]))
plot(position)

