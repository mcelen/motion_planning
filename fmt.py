import region
from math import *
from heapq import *
from numpy import *
import matplotlib.pyplot as plt

def in_any_obstacle(obs,pt):
    """ Determines whether pt is within any of the obstacles in obs. Returns True or False. """
    for o in obs:
        if o.contains_point(pt):
            return True
    return False

def SampleFree(k,obs,reg):
    """ Samples k points from region that don't intersect any obtacles, uniformly at random."""
    # Does not check for duplicate points, but duplicates are exceedingly unlikely.
    points = []
    while len(points) < k:
        pt = reg.uniform_sample()
        if not in_any_obstacle(obs,pt):
            points.append(pt)
    return points

def dist(p1,p2):
    """ Returns distance between p1 and p2. """
    return sqrt((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2)

def CollisionFree(p1,p2,obs):
    """ Determines whether the line-segment with endpoints p1,p2 is collision-free. Returns true if it does. Assumes p1 and p2 are not in obstacle. """    for obstacle in obs:
        if obstacle.line_hits_obstacle(p1,p2):
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

def offline_sampling(k,rk,xinit,reg,obs):
    """ Sampling of points and computation of nearest neighbors. """"
    V = [xinit]
    for pt in SampleFree(k,obs,reg):
        V.append(pt)
    neighbors = {}
    for i in range(k+1):
        Near(V,obs,neighbors,i,rk)
    return(V,neighbors)

def FMT(k,rk,xinit,reg,obs,goal,max_iter,V=[],neighbors={},with_plotting = False):
    """ Implements FMT* Algorithm. Returns path to destination as list of points, and indication of failure, in tuple (path,failure). """

    ### DATA STRUCTURES INITIALIZATION ###

    # Sample points and initialize V
    if not V:
        V = [xinit]
        for pt in SampleFree(k,obs,reg):
            V.append(pt)

    # Initialize neighbors (see documentation for description)
    if not neighbors:
        neighbors = {}
        Near(V,obs,neighbors,0,rk) # points nearby to xinit

    # Initialize set W of points not in tree, excluding xinit (ie. 0)
    W = set(range(1,len(V)))

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

            # If the path to the closest point is collision free, include the candidate in the tree.
            if CollisionFree(V[Ymin[0]],V[xID],obs): 
                E[xID] = Ymin[0]
                if with_plotting:
                    plt.plot([V[Ymin[0]][0],V[xID][0]],[V[Ymin[0]][1],V[xID][1]],'b')          
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

    if failure == 0 and iteration != max_iter:
        pathIDs = [zID]
        while not zID == 0:
            zID = E[zID]
            pathIDs.append(zID)
        pathIDs.reverse()
        path = [V[pID] for pID in pathIDs]
    else:   # dummy path variable for case of failure.
        failure = 1
        path = 0


    return (path,failure)

