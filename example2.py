# This example demonstrates how speed optimization slows down for curves.
# We construct a very sharp curve on an otherwise straight path, and observe that
# the speed optimization causes us to slow down for that curve.
# It plots the results.

import region
from numpy import *
import matplotlib.pyplot as plt
from trajectory_optimization import *
from trajectory_plotting import *

# Construct our region
workspace = region.Region((0.,0.),(1.,1.))

# Construct our path and plot it
P = [[.1,i*.05] for i in range(1,19)]
P.extend([.15+.05*i,.9] for i in range(18))
bubbles = [P,[.025 for i in range(len(P))]]
P = asarray(P)
u = 5*ones((P.shape[0]-2,2))
v = P[2:,:] - P[0:-2,:]
P = elastic_stretching(P,v,u,bubbles)
workspace.plot_region()
PlotTrajectory(P)
PlotBubbles(bubbles)
plt.title("Sharp Turn Trajectory")
plt.savefig("ex2_trajectory.pdf")
plt.clf()

# Do speed optimization and see that it slows down for sharp turn
result = speed_optimization(P)
v = result[0]

v = concatenate((asarray([[0.,0.]]),v),axis = 0) # We must add zeros because speed_optimization does not return first and last waypoint
v = concatenate((v,asarray([[0.,0.]])),axis = 0)
plt.plot(v[:,0],v[:,1],'rx-')
plt.axis([v.min(0)[0]-.5, v.max(0)[0]+.5, v.min(0)[1]-.5, v.max(0)[1]+.5]) # set axis to be just large enough to contain trajectory nicely.
plt.title("Velocity for Example 2")
plt.xlabel("$v_x$")
plt.ylabel("$v_y$")
plt.savefig("ex2_velocity.pdf")

