import xml.etree.ElementTree as et
import numpy as np
import matplotlib.pyplot as plt
from cmath import sqrt
from os import path


def read_obstacles_function(obstacles_file):
    """ Reads obstacles from provided file
    """
    " An obstacle has fields (x,y,R) defining its spatial extent"
    """ The obstacles files should be located somewhere near
    the following location : './resources/obstacles.obs' """

    "Next : add fields vx, vy for moving obstacles"

    # parse xml file and get root element
    tree = et.parse(obstacles_file)
    root = tree.getroot()

    # obstacles = 1D vector containing all the info about the obstacles
    # for static obstacles : x, y, R -- size(obstacles)%3 == 0
    # for moving obstacles : add vx, vy -- size(obstacles)%5 == 0
    # no test is made in that sense for now
    obstacles_list = []

    for index in range(0, len(root)):
        this_obstacle = root[index].attrib
        # conversion from string to float
        # can be bypassed if the xml is defined with full tags
        x = float(this_obstacle['x'])
        y = float(this_obstacle['y'])
        r = float(this_obstacle['R'])

        obstacles_list.append(x)
        obstacles_list.append(y)
        obstacles_list.append(r)

    return obstacles_list, 3


def check_validity(x, y, obstacles_list, obstacles_size):
    """ Checks whether point (x,y) lies in an obstacles
    """

    " x, y : coordinates of the point"
    " obstacles : a list containing the obstacles as a line (x,y,R,x,y,R...)"
    " size : the size of each obstacle. Might be used to resize [obstacles]"
    """ For each obstacle in [obstacles], the function will check that the
    distance between (x,y) and the center of the obstacle is greater
    than the radius of said obstacle"""

    obs_array = list_to_array(obstacles_list, obstacles_size)

    is_valid = True
    for obs in obs_array:
        xo = obs[0]
        yo = obs[1]
        ro = obs[2]
        if pow(x - xo, 2) + pow(y - yo, 2) <= pow(ro, 2):
            is_valid = False

    return is_valid


def check_validity_connection(x1, y1, x2, y2, obstacles_list, obstacles_size):
    """ Checks whether points (x1,y1) and (x2,y2) might be linked by
    a straight line that would not be interrupted by an obstacle
    """

    obs_array = list_to_array(obstacles_list, obstacles_size)

    is_valid = True
    for obs in obs_array:
        xo = obs[0]
        yo = obs[1]
        ro = obs[2]

        # define line joining p1, p2
        a = y1 - y2
        b = x2 - x1
        c = -(a * x1 + b * y1)

        # compute distance from center of obstacle to line
        dist_to_line = abs(a * xo + b * yo + c) / sqrt(pow(a, 2) + pow(b, 2))

        # check
        if dist_to_line <= ro:
            is_valid = False

    return is_valid


def plot_obstacles(obstacles_list, obstacles_size, x_vec=None, y_vec=None):
    """ Plots given obstacles and points on map
    """
    " obstacles_list : list of (x,y,R,x,y,R...) of given size "
    " x_vec, y_vec : lists of points to be plotted "

    if x_vec is None:
        x_vec = []
    if y_vec is None:
        y_vec = []

    # Set boundaries
    x_lim = 10
    y_lim = 5

    # Init figure
    plt.figure(figsize=(x_lim, y_lim))
    ax = plt.gca()
    ax.set_xlim((0, x_lim))
    ax.set_ylim((0, y_lim))

    # Draw each point with relevant color (green: valid/red: invalid)
    for i in range(len(x_vec)):
        if check_validity(x_vec[i], y_vec[i], obstacles_list, obstacles_size):
            color = 'go'
        else:
            color = 'ro'
        plt.plot(x_vec[i], y_vec[i], color, ms=10)

    # Draw each obstacle as a circle
    obs_array = list_to_array(obstacles_list, obstacles_size)
    for obs in obs_array:
        xy = (obs[0], obs[1])
        r = obs[2]
        c = plt.Circle(xy, r, color='b', fill=False)
        ax.add_artist(c)

    # Enable grid & show plot
    plt.grid()
    plt.show()
    return 0


def list_to_array(vec, size):
    """ Convert list to array using parameter size
    """

    """ Provided that length(vec)/size is an integer, transforms the given list
    into a numpy array"""
    n = len(vec) / size
    arr = np.reshape(np.array(vec), (n, size))
    return arr


if __name__ == "__main__":
    # Perform checks with dummy files
    file_path = path.join(
        path.abspath(path.abspath(__file__)),
        "resources",
        "obstacles.obs"
    )
    obstacles, size = read_obstacles_function(file_path)
    x_vec = [1]
    y_vec = [1]

    for i in range(len(x_vec)):
        print check_validity(x_vec[i], y_vec[i], obstacles, size)

    plot_obstacles(obstacles, size, x_vec, y_vec)
