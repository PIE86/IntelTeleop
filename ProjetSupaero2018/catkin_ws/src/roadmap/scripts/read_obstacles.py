import xml.etree.ElementTree as et


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
    # for static obstacles : x, y, R -- size(obstacles)%3 = 0
    # for moving obstacles : add vx, vy -- size(obstacles)%5 = 0
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
    

    
    
    


