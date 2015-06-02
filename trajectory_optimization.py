from math import *
from numpy import *
from cvxpy import *
import region

def dist(p1,p2):
    """ Returns distance between p1 and p2. """
    return ((p1[0] - p2[0])**2 + (p1[1]-p2[1])**2)**(.5)

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
    R = [.8*r for r in R] # Shrinkage to avoid hitting obstacles during smoothing.
    return[A,R]

def elastic_stretching(P,v,u,bubbles):
	""" Returns new waypoints P of smoothed path """

	# P is n+1x2 numpy array: [[x0,y0],[x1,y1],...,[xn,yn]]
	# v is n-1x2 numpy array, with rows corrsponding to speeds at
	#	[x1,y1],...,[x(n-1),y(n-1)]
	# u is n-1x2 numpy array, with rows corrsponding to controls at
	#	[x1,y1],...,[x(n-1),y(n-1)]. Format is [ulong,ulat].
	# bubbles is a list containing first a list of the centers, and then a list of
	# 	the radii of the bubbles
	#		bubbles[0] = [[x,y],[x,y],...,[x,y]]
	#		bubbles[1] = [r,r,...,r]

	# Set up fixed parameters
	Rmin = .01
	mu = .8
	g = 9.98
	m = 1.

	# Set up parameters from input data
	centers = asarray(bubbles[0])
	radius = asarray(bubbles[1])
	n = P.shape[0] - 1

    ## Path Smoothing Optimization
    # Parameter Computation
	d = sum(sum((P[2:,:] - P[1:-1,:])**2,1)**(.5))/float(n)
	alpha = ((mu*g)**2-(u[:,0]/m)**2)**(.5)

    # Define Optimization Variables
	Q = Variable(n+1,2)

	# Define Objective Function
	objective = Minimize(norm(2*Q[2:-1,:]-Q[1:-2,:]-Q[3:,:],'fro'))

	# Add Constraints
	constraints = [Q[0,:] == P[0:1,:], Q[1,:] == P[0:1,:] + d*v[0:1,:]/norm(v[0,:])] # starting point
	more_constraints = [Q[-1,:] == P[-1:,:]] #,Q[-2,:] == P[-1:,:] - d*v[-1:,:]/norm(v[-1,:])] # ending point
	constraints.extend(more_constraints)
	for k in range(2,n):
	    more_constraints = [norm(Q[k,:] - centers[k:k+1,:]) <= radius[k]] # bubble constraint
	    constraints.extend(more_constraints)
	for k in range(1,n):
	    more_constraints = [norm(2*Q[k,:] - Q[k-1,:] - Q[k+1,:]) <= min(d**2/Rmin,alpha[k-1]*(d/sum(v[k-1,:]**2)**(.5)))] # smoothness
	    constraints.extend(more_constraints)

	# Solve Problem
	problem = Problem(objective,constraints)
	result = problem.solve()

	# Extract Trajectory from solution
	P = asarray(Q.value)

	return P


def speed_optimization(P):
	""" Return speeds and controls, defined at the midpoints between the waypoints in P, in tuple (v,u). """

	# P is n+1x2 numpy array: [[x0,y0],[x1,y1],...,[xn,yn]]

	# Set up fixed parameters
	Rmin = .01
	mu = .8
	g = 9.98
	m = 1.

	# Set up paramters from input data
	n = P.shape[0] - 1

	## Speed Optimization
	# Define Optimization Variables
	a = Variable(n)
	b = Variable(n+1)
	u = Variable(n,2)

	# Define Objective Function
	objective = Minimize(2./float(n)*sum_entries(inv_pos(sqrt(b[0:-1])+sqrt(b[1:]))))

	# Define Paramters
	sp = (P[1:,:] - P[0:-1,:])/(1./float(n))
	P = concatenate((P[0:1,:],P),0)
	P = concatenate((P,P[-1:,:]),0)
	spp = (P[3:,:]-P[2:-1,:]-P[1:-2,:]+P[0:-3,:])/(2./float(n)**2)
	Ulong = .5*mu*m*g;

	# Add Constraints
	constraints = [abs(u[:,0]) <= Ulong,(b[1:]-b[:-1])/(n-1.) == 2.*a,square(u)*ones((2,1)) <= (mu*m*g)**2,b[0] == 0,b[-1]==0]
	for i in  range(n):
	    R = asmatrix([[sp[i,0],sp[i,1]],[- sp[i,1],sp[i,0]]])/(sum(asarray(sp[i,:])**2)**(.5))
	    more_constraints = [u[i,:]*R == m*sp[i:i+1,:]*a[i] + m*spp[i:i+1,:]*(b[i]+b[i+1])/2.]
	    constraints.extend(more_constraints)

	# Solve Problem
	prob = Problem(objective,constraints)
	result = prob.solve(solver=SCS)

	# Extract Trajectory From Solution
	P = P[1:-1,:] # Get rid of points added to aid with boundary conditions
	v = .5*(sp[:-1,:] + sp[1:,:])*asarray(b.value[1:-1])**(.5)
	u = asarray(u.value)
	u = .5*(u[:-1,:] + u[1:,:])

	return (v,u)




