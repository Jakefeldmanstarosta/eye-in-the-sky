import numpy as np
import matplotlib as plt

# amount of time to step forward in each loop
dt = 0.1

# function to plot drone flight. takes in flight duration, velocity, and a starting position. Returns a plot of the flight path
def fly(duration, velocity:tuple[float,float], starting_position=[0,0]):
    # how many ticks to get through the duration at dt
    ticks = int((duration / dt) + 1)
    # create empty arrays for x and y position. length of number of ticks
    x_pos = np.zeros(ticks)
    y_pos = np.zeros(ticks)


# loop through as many times as ticks
    for step in range(1, ticks):
        # set the current poistion equal to the previous position plus the delta position in the time dt (dpos = vdt). start at 2nd position.
        x_pos[step] = x_pos[step-1] + velocity[0] * dt
        y_pos[step] = y_pos[step-1] + velocity[1] * dt

    print(x_pos, y_pos)


fly(10,(0,5))
