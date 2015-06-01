import classes
import random
from math import *
from heapq import *
from numpy import *

def in_any_obstacle(obs,pt):
    """ Determines whether pt is within any of the obstacles in obs. Returns True or False. """
    for o in obs:
        if o.contains_point(pt):
            return True
    return False

def SampleFree(k,obs,region):
    """ Samples k points from region that don't intersect any obtacles, uniformly at random."""
    # Does not check for duplicate points, but duplicates are exceedingly unlikely.
    points = []
    while len(points) < k:
        pt = (random.random(),random.random()) # generate sample
        pt = (region.lower_left[0]*(pt[0]-1.)+region.upper_right[0]*pt[0],region.lower_left[1]*(pt[1]-1.)+region.upper_right[1]*pt[1]) # scale to live within region
        if not in_any_obstacle(obs,pt):
            points.append(pt)
    return points

def dist(p1,p2):
    """ Returns distance between p1 and p2. """
    return sqrt((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2)

# TO IMPLEMENT
def CollisionFree(p1,p2,obs):
    x=linspace(p1[0],p2[0],100)
    y=linspace(p1[1],p2[1],100)
    for i in range(0,len(x)):
        if in_any_obstacle(obs,(x[i],y[i])):
            return False
    return True

def Near(V,obs,neighbors,zID,r):
    """ Updates neighbors to contain Nz for point zID. See documentation of data structure description. """

    # Check if zID alrady in Nz
    if zID in neighbors:
        return

    # If it is not, add it
    Nz = {}
    Nzset = set([])
    z = V[zID]
    for pID, pt in enumerate(V):
        if dist(pt,z) < r and (not pID == zID):
            Nz[pID] = [dist(pt,z),CollisionFree(z,pt,obs)] ## TO DO: False should be hit_obstac
            Nzset.add(pID)
    neighbors[zID] = [Nz,Nzset]

 
def ComputeMin(Ynear,Cost,xID,neighbors):
    """ Dynamic Programming Step: Computes argmin_{y in Ynear}(Cost(y,T=(V,E))+Cost(y,x)) and the associated cost. """

    # If these are the values that are returned, this is a signature of Ynear being empty.
    curr_cost=float("inf")
    ymin=-1

    # Iterative search for minimum.
    for yID in Ynear:
     next_cost=Cost[yID]+neighbors[xID][0][yID][0]
     if next_cost<curr_cost:
         curr_cost=next_cost
         ymin=yID

    return (ymin,curr_cost)

def GenerateBubble(pt,obs):
    """ Returns tuple (center,radius,closestpt), where radius is the that of the largest circle centered at pt that does not overlap and obstacle, and closestpt is the closest point to pt on an obstacle. """

    distances = []
    points = []

    # For each obstacle, find the closest point and its distance to pt.
    for obstacle in obs:
        result = obstacle.dist_to_point(pt) # result is tuple (closestpt,distance)
        distances.append(result[1])
        points.append(result[0])

    # Of all these points and distances, find the minimum.
    min_dist = min(distances)
    min_point = points[argmin(array(distances))]

    return (pt,min(distances),min_point)

    
def TranslateBubble(bubble,r_l,obs):
    """ Attempts to translate bubble along normal to closest obstacle so that it has a radius as close to r_l as possible. """

    # This function uses binary search.
    # bubble is (center,radius,closestpt)
    #
    # ======closestpt----------center----pt--------ptmax
    # ==========                   1-alpha  alpha
    # obstacle==
    # ==========
    # ==========
    # 
    # It extends the line from closestpt to center to ptmax such that ||ptmax - closestpt||=r_l
    # It then binary searches the interval [center,ptmax] for the point pt closest to ptmax
    # such that the bubble centered at pt and going through closestpt does not overlap any obstacles.
    # The binary search is parameterized by alpha in [0,1], the parameter describing the 
    # weighted convex combination of center and ptmax.

    # Find furthest point we will search, called ptmax
    pt = bubble[0]
    min_point = bubble[2]
    ptmax = (min_point[0] + r_l/dist(pt,min_point)*(pt[0]-min_point[0]),min_point[1] + r_l/dist(pt,min_point)*(pt[1]-min_point[1]))
    
    # Check if ptmax is at least distance r_l from all obstacles
    distances = []
    for obstacle in obs:
        distances.append(obstacle.dist_to_point(ptmax)[1])
    if min(distances) >= r_l:
        return (ptmax,r_l,min_point)
        
    # Binary search alpha in [0,1] the points alpha*ptmax + (1-alpha)*pt
    alpha_max = 1.
    alpha_min = 0.    
    tol = .001
    while alpha_max > alpha_min + tol:
        alpha = (alpha_max+alpha_min)/2
        ptnew = (alpha*ptmax[0] + (1-alpha)*pt[0],alpha*ptmax[1] + (1-alpha)*pt[1])
        distances = []
        for obstacle in obs:
            distances.append(obstacle.dist_to_point(ptnew)[1])
        if min(distances) >= alpha*r_l + (1-alpha)*bubble[1]-10e-10: # 10e-10 is to ensure robustness of inequailty to numerical error
            alpha_min = alpha
        else:
            alpha_max = alpha
    bestpt = (alpha_min*ptmax[0] + (1-alpha_min)*pt[0],alpha_min*ptmax[1] + (1-alpha_min)*pt[1])
    return (bestpt,alpha_min*r_l + (1-alpha_min)*bubble[1],min_point)
    
def BubbleGeneration(P,obs,r_l):
    """ Generates bubbles centered at P that do not intersect obstacles, and attempts to translate them if they are too small. Return list of their centers and radii. """
    A = [P[0]]
    R = [0]
    for i in range(1,len(P)):
        if dist(P[i],A[i-1])  < .5*R[i-1]:
            A.append(A[i-1])
            R.append(R[i-1])
            continue 
        bubble = GenerateBubble(P[i],obs)
        R.append(bubble[1])
        A.append(bubble[0])
        if R[i] < r_l:
            bubble = TranslateBubble(bubble,r_l,obs)
            R[i] = bubble[1]
            A[i] = bubble[0]
    return[A,R]

def FMT(k,rk,region,obs,goal,max_iter):
    """ Implements FMT Algorithm. Returns path to destination as list of points, and indication of failure, in tuple (path,failure). """

    ### DATA STRUCTURES INITIALIZATION ###

    # Sample points and initialize V
    xinit = (.5,.1)
    V = [xinit]
    for pt in SampleFree(k,obs,region):
        V.append(pt)
    V.append((.5,.5))

    # Initialize set W of points not in tree, excluding xinit (ie. 0)
    W = set(range(1,len(V)))

    # Initialize neighbors (see documentation for description)
    neighbors = {}
    Near(V,obs,neighbors,0,rk) # points nearby to xinit

    # Initialize frontier of tree H and Hheap
    H = set([0])
    Hheap=[]

    # Initilize Cost Dictionary 
    Cost = {}
    Cost[0] = 0

    # Initialize Tree Graph
    E={}

    ### IMPLEMENTATION OF ALGORITHM ###
    z = xinit
    zID = 0
    failure=0
    iteration = 0

    # Each iteration of the loop takes the point z of minimum cost in H, and uses it to search locally
    # for an addition to the tree (which may or may not connect z).
    # In the body of the loop, zID is still in H, but not in Hheap, as we just popped in from Hheap.
    while not goal.contains_point(z) and iteration < max_iter:
        # Initilize strcture for holding points added to tree
        Hnew = set([])

        # Find candidates to be added to tree
        Xnear = W & neighbors[zID][1]
        
        # For each candidate
        for xID in Xnear:

            # Find the closest point in the frontier of our tree to connect them to
            Near(V,obs,neighbors,xID,rk)
            Ynear = H & neighbors[xID][1]
            Ymin =  ComputeMin(Ynear,Cost,xID,neighbors) # Ymin is tuple (yID,cost(yID))

            # If the path to the closes point is collision free, include the candidate in the tree.
            if CollisionFree(V[Ymin[0]],V[xID],obs): 
                E[xID] = Ymin[0]            
                Hnew.add((Ymin[1],xID))
                W.remove(xID)
        
        # Update the frontier
        # H <-- (H U Hnew)\{z}
        for elem in Hnew:
            H.add(elem[1])
            heappush(Hheap,elem)
            Cost[elem[1]]=elem[0]
        H.remove(zID)

        # If H is empty, declare failure
        if not len(H): # If H is empty
            failure=1
            break

        # Next base point in min point in the frontier
        Zmin=heappop(Hheap) # returns tuple (cost of z, zID)
        zID=Zmin[1]
        z = V[zID]       
        iteration += 1

    if failure == 0:
        pathIDs = [zID]
        while not zID == 0:
            zID = E[zID]
            pathIDs.append(zID)
        pathIDs.reverse()
        path = [V[pID] for pID in pathIDs]
    else:   # dummy path variable for case of failure.
        path = 0


    return (path,failure)

