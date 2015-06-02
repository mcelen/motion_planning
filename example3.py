# This example demonstrates calling our code to run our algorithms on a more complex geometry.
# It plots the results.

import region
from numpy import *
import matplotlib.pyplot as plt
from trajectory_optimization import *
from trajectory_plotting import *
from fmt import *

k = 1000
rk = .1
workspace = region.Region((0.,0.),(1.,2.))
# obs = [region.Region((.1,.1),(.4,.4)),region.Region((.6,.6),(.9,.9)),region.Region((.6,.1),(1.0,.5))]
obs = [region.Region((.1,.1),(.4,.4)),region.Region((.3,.6),(.9,.9)),region.Region((.6,.1),(1.0,.5)),region.Region((.1,.5),(.2,1.2)),region.Region((.3,1.2),(1.0,1.9))]
goal = region.Region((.1,1.5),(.2,1.7))
max_iter = 1000

# for obstacle in obs:
#     obstacle.plot_region()
# goal.plot_region()
# plt.savefig("ex3_unsmoothed_path.pdf")

xinit = (.5,.1)

# FMT Algorithm
for obstacle in obs:
    obstacle.plot_region()
goal.plot_region()
path = FMT(k,rk,xinit,workspace,obs,goal,max_iter,with_plotting = True)
plt.title("Fast Marching Tree at Termination")
plt.savefig("ex3_fmt.pdf")
plt.clf()

print path

if path[1] == 0:
	
	# Turn P into numpy array
	P = asarray(path[0])

	# Plot
	for obstacle in obs:
	    obstacle.plot_region()
	goal.plot_region()
	PlotTrajectory(P)

	# Save Figure and clear
	plt.title("Example 3 Unsmoothed Trajectory")
	plt.savefig("ex3_unsmoothed_path.pdf")
	plt.clf()

	# Make Bubbles
	bubbles = BubbleGeneration(P,obs,.1)

	# Elastic Stretching and Speed Optimization
	u = 5*ones((P.shape[0]-2,2))
	v = P[2:,:] - P[0:-2,:]
	for iteration in range(5):
		P = elastic_stretching(P,v,u,bubbles)
		result = speed_optimization(P)
		v = result[0]
		u = result[1]

	# Plot Smoothed Path and Clear
	for obstacle in obs:
	    obstacle.plot_region()
	goal.plot_region()
	PlotTrajectory(P)
	PlotBubbles(bubbles)
	plt.title("Example 3 Smoothed Trajectory")
	plt.savefig("ex3_smoothed_path.pdf")
	plt.clf()

	# Plot Speed Trajectory 
	v = concatenate((asarray([[0.,0.]]),v),axis = 0) # We must add zeros because speed_optimization does not return first and last waypoint
	v = concatenate((v,asarray([[0.,0.]])),axis = 0)
	plt.plot(v[:,0],v[:,1],'rx-')
	plt.axis([v.min(0)[0]-.5, 1, -1, v.max(0)[1]+.5]) # set axis to be just large enough to contain trajectory nicely.
	plt.title("Velocity for Example 3")
	plt.xlabel("$v_x$")
	plt.ylabel("$v_y$")
	plt.savefig("ex3_speed.pdf")

