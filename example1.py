# This example demonstrates calling our code to run our algorithms on a simple geometry.
# It plots the results.

import region
from numpy import *
import matplotlib.pyplot as plt
from trajectory_optimization import *
from trajectory_plotting import *
from fmt import *

k = 1000
rk = .05
workspace = region.Region((0.,0.),(1.,1.))
obs = [region.Region((.1,.1),(.4,.4)),region.Region((.6,.6),(.9,.9)),region.Region((.6,.1),(1.0,.5))]
goal = region.Region((.9,.9),(1.,1.))
max_iter = 1000

xinit = (.5,.1)

# FMT Algorithm
path = FMT(k,rk,xinit,workspace,obs,goal,max_iter)

if path[1] == 0:
	
	# Make bubbles
	P = asarray(path[0])
	bubbles = BubbleGeneration(P,obs,.1)

	# Plot
	for obstacle in obs:
	    obstacle.plot_region()
	goal.plot_region()
	PlotTrajectory(P)
	PlotBubbles(bubbles)

	# Save Figure and clear
	plt.savefig("ex1_unsmoothed_path.pdf")
	plt.clf()

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
	plt.savefig("ex1_smoothed_path.pdf")
	plt.clf()

	# Plot Speed Trajectory 
	y = array(v)
	x = arange(len(y))
	plt.plot(x,y)
	plt.savefig("ex1_speed.pdf")

