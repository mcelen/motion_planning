# This example demonstrates calling our code to run our algorithms on a simple geometry.
# It plots the results.

import classes
from numpy import *
import matplotlib.pyplot as plt
from trajectory_optimization import *
from trajectory_plotting import *
from fmt import *

k = 1000
rk = .05
k = 1000
rk = .05
region = classes.Obstacle((0.,0.),(1.,1.))
obs = [classes.Obstacle((.1,.1),(.4,.4)),classes.Obstacle((.6,.6),(.9,.9)),classes.Obstacle((.6,.1),(1.0,.5))]
goal = classes.Obstacle((.9,.9),(1.,1.))
max_iter = 1000

# FMT Algorithm
path = FMT(k,rk,region,obs,goal,max_iter)

if path[1] == 0:
	
	# Make bubbles
	P = asarray(path[0])
	bubbles = BubbleGeneration(P,obs,.1)

	# Plot
	for obstacle in obs:
	    PlotRegion(obstacle)
	PlotRegion(goal)
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
	    PlotRegion(obstacle)
	PlotRegion(goal)
	PlotTrajectory(P)
	PlotBubbles(bubbles)
	plt.savefig("ex1_smoothed_path.pdf")
	plt.clf()

	# Plot Speed Trajectory 
	y = array(v)
	x = arange(len(y))
	plt.plot(x,y)
	plt.savefig("ex1_speed.pdf")

