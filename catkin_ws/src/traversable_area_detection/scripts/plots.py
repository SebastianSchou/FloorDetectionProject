#!/usr/bin/env python2
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rosunit
import rosservice
import rospkg
import math

from traversable_area_detection.srv import *
import traversable_area_detection.msg

MAX_ANGLE_DIFFERENCE = 2.0 * 4.0 * math.pi / 180.0
MAX_DISTANCE_DIFFERENCE = 2.0 * 0.08
COLORS = ["Blue", "Green", "Red", "Yellow", "Purple", "White", "Light blue", "Light green", "light red"]

def hptCallback(data):
	noOfPlanes = len(data.planes)
	doesPlaneOfAllNodesExist = any([plane.id == 10000 for plane in data.planes])
	noOfPlanes -= 1 if doesPlaneOfAllNodesExist else 0
	maxDistance = 0
	if (noOfPlanes == 0):
		return
	noOfSubplotRows = 2
	noOfSubplotCols = int(math.ceil(float(noOfPlanes) / float(noOfSubplotRows)))
	plt.figure(101)
	plt.subplots_adjust(hspace = 0.4)
	subplotNo = 0
	for plane in data.planes:
		if (doesPlaneOfAllNodesExist and plane.id == 10000):
			continue
		subplotNo += 1
		if (noOfPlanes > 1):
			plt.subplot(noOfSubplotRows, noOfSubplotCols, subplotNo)
		rhoA = []
		phiA = []
		thetaA = []
		for node in plane.nodes:
			maxDistance = max(maxDistance, node.rho)
			rhoDiff = abs(node.rho - plane.distance)
			phiDiff = abs(node.phi - plane.phi) * 180.0 / math.pi
			thetaDiff = abs(node.theta - plane.theta)
			thetaDiff = 2 * math.pi - thetaDiff if thetaDiff > math.pi else thetaDiff
			thetaDiff *= 180.0 / math.pi
			rhoA.append(rhoDiff)
			phiA.append(phiDiff)
			thetaA.append(thetaDiff)
		#plt.axhline(2 * math.sqrt(np.sum(np.square(phiA)) / len(plane.nodes)))
		#plt.axvline(2 * math.sqrt(np.sum(np.square(thetaA)) / len(plane.nodes)))
		plt.axhline(MAX_ANGLE_DIFFERENCE * 180.0 / math.pi)
		plt.axvline(MAX_ANGLE_DIFFERENCE * 180.0 / math.pi)
		plt.scatter(thetaA, phiA, c=rhoA, cmap='viridis')
		plt.clim(0.0, 0.20)
		plt.xlabel(r"$\Delta\theta$ [degrees]")
		plt.ylabel(r"$\Delta\phi$ [degrees]")
		plt.title("Plane %d, type %s\nColor %s" % (plane.id, plane.type, COLORS[subplotNo - 1]))
		plt.grid(True)
		plt.axis('square')
	plt.subplots_adjust(bottom=0.1, right=0.8, top=0.9)
	cax = plt.axes([0.85, 0.1, 0.025, 0.8])
	cbar = plt.colorbar(cax=cax)
	cbar.ax.get_yaxis().labelpad = 15
	cbar.ax.set_ylabel(r"$\Delta\rho$ [m]", rotation=270)

	plt.figure(102)
	rhoA = []
	phiA = []
	thetaA = []
	planeIds = []
	for plane in data.planes:
		if (doesPlaneOfAllNodesExist and plane.id == 10000):
			continue
		rhoA.append(plane.distance)
		phiA.append(plane.phi * 180.0 / math.pi)
		thetaA.append(plane.theta * 180.0 / math.pi)
		planeIds.append("Plane %d" % plane.id)
	plt.scatter(thetaA, phiA, c=rhoA, cmap='viridis')
	cbar = plt.colorbar()
	cbar.ax.get_yaxis().labelpad = 15
	cbar.ax.set_ylabel(r"$\rho$ [m]", rotation=270)
	plt.xlabel(r"$\theta$ [degrees]")
	plt.ylabel(r"$\phi$ [degrees]")
	plt.title("Plane parameters (angle and distance)")
	plt.grid(True)
	for i, txt in enumerate(planeIds):
		plt.annotate(txt, (thetaA[i], phiA[i]))

	plt.figure(103)
	xA = []
	yA = []
	zA = []
	planeIds = []
	for plane in data.planes:
		if (doesPlaneOfAllNodesExist and plane.id == 10000):
			continue
		xA.append(plane.normal.x)
		yA.append(plane.normal.y)
		zA.append(plane.normal.z)
		planeIds.append("Plane %d" % plane.id)
	plt.scatter(xA, yA, c=zA, cmap='viridis')
	cbar = plt.colorbar()
	cbar.ax.get_yaxis().labelpad = 15
	cbar.ax.set_ylabel("z", rotation=270)
	plt.xlabel("x")
	plt.ylabel("y")
	plt.title("Plane parameters (normal vector)")
	plt.grid(True)
	for i, txt in enumerate(planeIds):
		plt.annotate(txt, (xA[i], yA[i]))

	plt.figure(104)
	plt.subplots_adjust(hspace = 0.4)
	subplotNo = 0
	for plane in data.planes:
		if (doesPlaneOfAllNodesExist and plane.id == 10000):
			continue
		subplotNo += 1
		if (noOfPlanes > 1):
			plt.subplot(noOfSubplotRows, noOfSubplotCols, subplotNo)
		rhoA = []
		phiA = []
		thetaA = []
		for node in plane.nodes:
			rhoA.append(node.rho)
			phiA.append(node.phi * 180.0 / math.pi)
			thetaA.append(node.theta * 180.0 / math.pi)
		plt.scatter(thetaA, phiA, c=rhoA, cmap='viridis')
		plt.clim(0, maxDistance)
		plt.xlabel(r"$\theta$ [degrees]")
		plt.ylabel(r"$\phi$ [degrees]")
		plt.title("Plane %d, type %s\nColor %s" % (plane.id, plane.type, COLORS[subplotNo - 1]))
		plt.grid(True)
	plt.subplots_adjust(bottom=0.1, right=0.8, top=0.9)
	cax = plt.axes([0.85, 0.1, 0.025, 0.8])
	cbar = plt.colorbar(cax=cax)
	cbar.ax.get_yaxis().labelpad = 15
	cbar.ax.set_ylabel(r"$\rho$ [m]", rotation=270)

	for plane in data.planes:
		if plane.id == 10000:
				plt.figure(105)
				rhoA = []
				phiA = []
				thetaA = []
				for node in plane.nodes:
					rhoA.append(node.rho)
					phiA.append(node.phi * 180.0 / math.pi)
					thetaA.append(node.theta * 180.0 / math.pi)
				plt.scatter(thetaA, phiA, c=rhoA, cmap='viridis')
				cbar = plt.colorbar()
				cbar.ax.get_yaxis().labelpad = 15
				cbar.ax.set_ylabel(r"$\rho$ [m]", rotation=270)
				plt.xlabel(r"$\theta$ [degrees]")
				plt.ylabel(r"$\phi$ [degrees]")
				plt.title("Nodes in image")
				plt.grid(True)
			

	plt.show()
		


if __name__ == "__main__":
	rospy.init_node("Plots", anonymous=True)
	subHoughPlaneTransform = rospy.Subscriber(
				"hough_plane_transform", traversable_area_detection.msg.HoughPlaneTransform, hptCallback)
	rospy.spin()
	