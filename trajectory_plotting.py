import classes
import matplotlib.pyplot as plt
from numpy import *

def PlotRegion(region):
	""" Adds rectangular obstacle given by region to current plot. """
	x_ll=region.lower_left[0]
	y_ll=region.lower_left[1]
	x_ur=region.upper_right[0]
	y_ur=region.upper_right[1]
	x=[x_ll,x_ur,x_ur,x_ll,x_ll]
	y=[y_ll,y_ll,y_ur,y_ur,y_ll]
	plt.plot(x,y)

def PlotBubbles(bubbles):
	""" Plots bubbles given by the centers and radii found in bubbles. """

	# bubbles is a list
	# bubbles[0] is a list of centers: [[x0,y0],[x1,y1],...,[xn,yn]]
	# bubbles[1] is a list of radii: [r0,r1,...,rn]

	fig = plt.gcf()
	for i in range(len(bubbles[0])):
		circle = plt.Circle(bubbles[0][i],bubbles[1][i],fill=False)
		fig.gca().add_artist(circle)

def PlotTrajectory(P):
	""" Plots waypoints and joining edges found in P. """

	# P is a numpy array: [[x0,y0],[x1,y1],...,[xn,yn]]

	x = P[:,0]
	y = P[:,1]
	plt.scatter(x,y)
	plt.plot(x,y)

