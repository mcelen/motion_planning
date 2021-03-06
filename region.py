from math import *
import random
import matplotlib.pyplot as plt

def dist(p1,p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2)

class Region:

    def __init__(self,ll,ur):
        """ Initializes with lower_left corner and upper_right corner coordinates. """
        self.lower_left = ll
        self.upper_right = ur

    def contains_point(self,pt):
        """ Returns True of Obstacle containst pt, and False if it does not. """
        if self.lower_left[0] <= pt[0] and self.lower_left[1] <= pt[1] and self.upper_right[0] >= pt[0] and self.upper_right[1] >= pt[1]:
            return True
        else:
            return False

    def line_hits_obstacle(self,p1,p2):
        """ Determines whether the line-segment with endpoints p1,p2 hits the obstacle. Returns true if it does. Assumes p1 and p2 are not in obstacle. """
        # first checks intersection with left edge, then procedes clockwise.
        if (p1[0]-self.lower_left[0])*(p2[0]-self.lower_left[0]) < 0:
            yhit = p1[1] + (self.lower_left[0]-p1[0])/(p2[0]-p1[0])*(p2[1]-p1[1])
            if self.upper_right[1] > yhit and self.lower_left[1] < yhit:
                return True
        if (p1[1]-self.upper_right[1])*(p2[1]-self.upper_right[1]) < 0:
            xhit = p1[0] + (self.upper_right[1]-p1[1])/(p2[1]-p1[1])*(p2[0]-p1[0])
            if self.upper_right[0] > xhit and self.lower_left[0] < xhit:
                return True
        if (p1[0]-self.upper_right[0])*(p2[0]-self.upper_right[0]) < 0:
            yhit = p1[1] + (self.upper_right[0]-p1[0])/(p2[0]-p1[0])*(p2[1]-p1[1])
            if self.upper_right[1] > yhit and self.lower_left[1] < yhit:
                return True
        if (p1[1]-self.lower_left[1])*(p2[1]-self.lower_left[1]) < 0:
            xhit = p1[0] + (self.lower_left[1]-p1[1])/(p2[1]-p1[1])*(p2[0]-p1[0])
            if self.upper_right[0] > xhit and self.lower_left[0] < xhit:
                return True
        return False

    def dist_to_point(self,pt):
        """ Returns the closest point in Obstacle to pt, as well as the distance between them, in tuple (closestpt,distance). """
        
        # Works by dividing into 9 cases, demonstrated below. The shaded region is the obstacle.
        # The regions are numbered by the order in which they are handled in the code.
        #
        #               |               |
        #       1       |      4        |       7  
        #               |               |                   
        #   ------------|---------------|------------
        #               |###############|           
        #       2       |##### 5 #######|       8
        #               |###############|           
        #   ------------|---------------|------------ 
        #               |               |
        #       3       |       6       |       9 
        #               |               |  
        #               |               |
        #

        if pt[0] < self.lower_left[0]:
            if pt[1] > self.upper_right[1]: # region 1
                closest = (self.lower_left[0],self.upper_right[1])
                return (closest,dist(pt,closest))
            elif pt[1] > self.lower_left[1]: # region 2
                closest = (self.lower_left[0],pt[1])
                return (closest,dist(pt,closest))
            else: # region 3
                closest = self.lower_left
                return (closest,dist(pt,closest))
        elif pt[0] < self.upper_right[0]:
            if pt[1] > self.upper_right[1]: # region 4
                closest = (pt[0],self.upper_right[1])
                return (closest,dist(pt,closest))
            elif pt[1] > self.lower_left[1]: # region 5
                closest = pt
                return (closest,0)
            else: # region 6
                closest = (pt[0],self.lower_left[1])
                return (closest,dist(pt,closest))
        else:
            if pt[1] > self.upper_right[1]: # region 7
                closest = self.upper_right
                return (closest,dist(pt,closest))
            elif pt[1] > self.lower_left[1]: # region 8
                closest = (self.upper_right[0],pt[1])
                return (closest,dist(pt,closest))
            else: # region 9
                closest = (self.upper_right[0],self.lower_left[1])
                return (closest,dist(pt,closest))

    def uniform_sample(self):
        """ Sample point uniformly at random from region. """
        pt = (random.random(),random.random()) # generate sample
        pt = (self.lower_left[0]*(pt[0]-1.)+self.upper_right[0]*pt[0],self.lower_left[1]*(pt[1]-1.)+self.upper_right[1]*pt[1]) # scale to live within self
        return pt

    def plot_region(self):
        """ Adds rectangular obstacle given by region to current plot. """
        x_ll=self.lower_left[0]
        y_ll=self.lower_left[1]
        x_ur=self.upper_right[0]
        y_ur=self.upper_right[1]
        x=[x_ll,x_ur,x_ur,x_ll,x_ll]
        y=[y_ll,y_ll,y_ur,y_ur,y_ll]
        plt.plot(x,y)
                        



