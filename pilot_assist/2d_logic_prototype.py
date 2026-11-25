import numpy as np
import matplotlib.pyplot as plt

OBSTACLE = np.array([5,3])
TIME_TO_IMPACT_THRESHOLD = 3

def forward_then_right(time):
    """Scripted pilot that commands a velocity based on time.

    Flies forward at 5 m/s for the first 5 seconds, then sideways
    at 5 m/s.

    Args:
        time (float): Current time in seconds.

    Returns:
        np.ndarray: Commanded velocity (vx, vy) in m/s, shape (2,).
            x is forward, y is sideways.
    """
    if time < 5:
        velocity = np.array([1,0])
    else:
        velocity = np.array([0,1])

    return velocity

def straight(time):
    velocity = np.array([1,0])
    return velocity

def diagonal(time):
    velocity = np.array([2,1])
    return velocity


def fly(dt, duration, pilot, safety_filter, starting_position=(0,0)):
    """Simulate the flight of the drone using commanded velocities from a pilot.

    Args:
        dt (float): Length of each time step in seconds.
        duration (float): Duration of the flight in seconds.
        pilot (callable): Pilot that takes the current time in seconds and
            returns a commanded velocity (vx, vy) in m/s.
        starting_position (tuple[float, float]): Starting position (x, y) in
            metres. Defaults to (0, 0).

    Returns:
        np.ndarray: Positions of the drone at each time step, shape (N, 2),
            where N is the number of steps. Each row is (x, y) in metres.
    """

    # how many steps required to complete the duration at step size dt
    steps = round((duration / dt) + 1)

    # create a position vector using numpy array of size (steps) x 2. array contains a position for every step in time. every position is given as a vector (x,y)
    position = np.zeros((steps, 2))

    # set starting position at first position vector at t=0 (x0,y0) in position array
    position[0] = starting_position

    # loop to simulate motion through time. start at second position since initial position is given. loop as many times as steps forward to reach total duration
    for step in range(1, steps):      
        # time at the previous position (the one we're stepping from)
        elapsed_time = (dt * step) - dt  
        # set the current position equal to the previous position plus the difference in position in time dt (pos2 = pos1 + dpos = pos1 + vdt).
        # grabs v from the pilot
        position[step] = position[step-1] + safety_filter(pilot(elapsed_time), position[step-1]) * dt

    # returns the numpy array with the simulated flight position data 
    print(position[steps - 1])
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
    ax.plot(OBSTACLE[1],OBSTACLE[0], '-o')
    ax.axis('equal')
    plt.show()

def safety_filter(commanded_velocity, position):
    """Filter the commanded velocity to avoid collision by slowing down

        This is a one dimensional prototype
    
        Args:
            command_velocity (np.ndarray): Velocity pilot is commanding in m/s,
            position (np.ndarray): Position at the time of command

        Returns:
                untouched_velocity (np.ndarray): Filter wasn't necessary, doesn't change anything
                filtered_velocity (np.ndarray): Filter was used, velocity was adjusted to avoid collision
        """
    # find the displacement vector from the drone to the obstacle
    d_obstacle = OBSTACLE - position
    # take the norm of that displacement vector to find the distance
    distance_obstacle = np.linalg.norm(d_obstacle)
    # normalize the displacement vector using the norm just calculated to get the unit vector pointing in the direction of the obstacle
    d_hat = d_obstacle / distance_obstacle
    # find the magnitude of velocity parallel to the direction of the obstacle
    speed_par = np.dot(commanded_velocity, d_hat)
    # calculate the parallel component of the velocity vector  
    v_par = d_hat * speed_par
    # use previous result to find perpendicular component of the velocity vector
    v_perp = commanded_velocity - v_par
    # check if the velocity has component towards the obstacle. if not, do nothing and end the function
    print(d_obstacle, distance_obstacle, d_hat, speed_par, v_par, v_perp)
    if speed_par <= 0:
        return commanded_velocity

    # use the parallel velocity to check the time until impact in the parallel direction
    time_to_impact_parallel = distance_obstacle / speed_par
    # if the tim until impact in the parallel direction is less than 3, filter the speed, if not, do nothing
    if time_to_impact_parallel < TIME_TO_IMPACT_THRESHOLD:
        # filter the parallel velocity to make sure time until impact in that direction stays at 3 seconds
        filtered_speed_par = distance_obstacle / TIME_TO_IMPACT_THRESHOLD
        print(filtered_speed_par)
        # create new parallel velocity vector to account for the filtered parallel velocity. this is in the direction to the obstacle
        filtered_v_par = d_hat * filtered_speed_par
        # create final new velocity command by adding back the perpindicular component to the filtered parallel component and return it
        filtered_commanded_velocity = filtered_v_par + v_perp
        return filtered_commanded_velocity
    else:
        return commanded_velocity

def nothing_safety(commanded_velocity, position):
    return commanded_velocity

position = fly(0.5, 5, diagonal, safety_filter)
plot(position)