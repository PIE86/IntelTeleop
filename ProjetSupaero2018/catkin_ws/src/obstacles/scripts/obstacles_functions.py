import xml.etree.ElementTree as et
import numpy as np
import matplotlib.pyplot as plt

def read_obstacles_function(obstacles_file):
    """ Reads obstacles from provided file"""
    
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
    obstacles = [];
    
    for i in range(0,len(root)):
        this_obstacle = root[i].attrib
        # conversion from string to float
        # can be bypassed if the xml is defined with full tags
        x = float(this_obstacle['x'])
        y = float(this_obstacle['y'])
        R = float(this_obstacle['R'])
        
        obstacles.append(x)
        obstacles.append(y)
        obstacles.append(R)
        
#         print ('This obstacle is at ({x},{y}) with radius {R}'.format(x=x,y=y,R=R))
        
    return obstacles, 3

# my_file = './resources/obstacles.obs'
# my_obstacles, size = read_obstacles(my_file)
# print my_obstacles
# print size


def check_validity(x, y, obstacles, size):
    """ Checks whether point (x,y) lies in an obstacles"""
    
    " x, y : coordinates of the point"
    " obstacles : a list containing the obstacles as a line (x,y,R,x,y,R...)"
    " size : the size of each obstacle. Might be used to resize [obstacles]"
    """ For each obstacle in [obstacles], the function will check that the 
      distance between (x,y) and the center of the obstacle is greater
      than the radius of said obstacle"""
        
    obs_array = list_to_array(obstacles, size)
    
    is_valid = True
    for i in obs_array:
        xo = i[0]
        yo = i[1]
        ro = i[2]
        if (pow(x-xo,2)+pow(y-yo,2) <= pow(ro,2)):
            is_valid = False  
        
    return is_valid
    
    
    
def plot_obstacles(obstacles_list, size, x_vec=[], y_vec=[]):
    """ Plots given obstacles and points on map """
    
    " obstacles_list : list of (x,y,R,x,y,R...) of given size "
    " x_vec, y_vec : lists of points to be plotted "
    
    xlim = 10
    ylim = 5
    
    plt.figure(figsize=(xlim,ylim))
    ax = plt.gca()
    ax.set_xlim((0,xlim))
    ax.set_ylim((0,ylim))
    
    for i in range(len(x_vec)):
        if check_validity(xvec[i], yvec[i], obstacles, size):
            color = 'go'
        else: color = 'ro'
        plt.plot(xvec[i], yvec[i], color, ms=10)
     
    obs_array = list_to_array(obstacles_list, size)
    for obs in obs_array:
        xy = (obs[0], obs[1])
        r = obs[2]
        c = plt.Circle(xy, r, color='b', fill=False)
        ax.add_artist(c)
             
    plt.grid()
    plt.show()
    return 0
    
def list_to_array(vec, size):
    n = len(vec)/size
    arr = np.reshape(np.array(vec), (n,size))
    return arr
    

if __name__=="__main__":
    file_path = '/home/chinch/MemoryEnhancedPredictiveControl/ProjetSupaero2018/catkin_ws/src/roadmap/resources/obstacles.obs'
    obstacles, size = read_obstacles_function(file_path)
    l = 50
    xvec = [1]#np.linspace(0,10,l)
    yvec = [1]#np.linspace(0,5,l)
    
    for i in range(len(xvec)):
        print check_validity(xvec[i], yvec[i], obstacles, size)
        
    plot_obstacles(obstacles, size, xvec, yvec)



    
    
    


