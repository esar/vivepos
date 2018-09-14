#!/usr/bin/python

import os
import sys
import math
import argparse
import serial
import numpy as np
from sympy import *

TICK_CLOCK_MHZ = 5.0
SWEEP_CENTER_IN_MICROSEC = 4000
SWEEP_180_IN_MICROSEC = 8333


def ticks2angle(ticks):
	return math.pi * (ticks / TICK_CLOCK_MHZ - SWEEP_CENTER_IN_MICROSEC) / SWEEP_180_IN_MICROSEC


def angles2vector(xAngle, yAngle):
	plane1 = np.float32([+cos(xAngle), 0, -sin(xAngle)])
	plane2 = np.float32([0, +cos(yAngle), +sin(yAngle)])
	vector = np.cross(plane2, plane1)
	vector /= np.linalg.norm(vector)
	return vector


def calcRotationMatrix(fromVector, toVector):
	# From http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q38
	# Via https://stackoverflow.com/questions/43507491/imprecision-with-rotation-matrix-to-align-a-vector-to-an-axis

	# Normalize vector length
	toVector /= np.linalg.norm(toVector)
	fromVector /= np.linalg.norm(fromVector)

	# Get axis
	uvw = np.cross(fromVector, toVector)

	# compute trig values - no need to go through arccos and back
	rcos = np.dot(fromVector, toVector)
	rsin = np.linalg.norm(uvw)

	# normalize and unpack axis
	if not np.isclose(rsin, 0):
		uvw /= rsin
	u, v, w = uvw

	# Compute rotation matrix - re-expressed to show structure
	return (
		rcos * np.eye(3) +
		rsin * np.float32([
			[ 0, -w,  v],
			[ w,  0, -u],
			[-v,  u,  0]
		]) +
		(1.0 - rcos) * uvw[:,None] * uvw[None,:]
	)


def calcSensorDistances(sensorGrid, sensorTicks):
	# From https://github.com/osresearch/lighthouse/blob/master/solve-lighthouse

	sensorPos = np.array([
		[+sensorGrid/2, -sensorGrid/2, 0],
		[+sensorGrid/2, +sensorGrid/2, 0],
		[-sensorGrid/2, +sensorGrid/2, 0],
		[-sensorGrid/2, -sensorGrid/2, 0]
	])

	# Calc distances between sensors
	r01 = N(np.linalg.norm(sensorPos[0] - sensorPos[1]))
	r02 = N(np.linalg.norm(sensorPos[0] - sensorPos[2]))
	r03 = N(np.linalg.norm(sensorPos[0] - sensorPos[3]))
	r12 = N(np.linalg.norm(sensorPos[1] - sensorPos[2]))
	r13 = N(np.linalg.norm(sensorPos[1] - sensorPos[3]))
	r23 = N(np.linalg.norm(sensorPos[2] - sensorPos[3]))

	v0 = angles2vector(ticks2angle(sensorTicks[0][0]), ticks2angle(sensorTicks[0][1]))
	v1 = angles2vector(ticks2angle(sensorTicks[1][0]), ticks2angle(sensorTicks[1][1]))
	v2 = angles2vector(ticks2angle(sensorTicks[2][0]), ticks2angle(sensorTicks[2][1]))
	v3 = angles2vector(ticks2angle(sensorTicks[3][0]), ticks2angle(sensorTicks[3][1]))

	v01 = np.dot(v0, v1)
	v02 = np.dot(v0, v2)
	v03 = np.dot(v0, v3)
	v12 = np.dot(v1, v2)
	v13 = np.dot(v1, v3)
	v23 = np.dot(v2, v3)

	k0, k1, k2, k3 = symbols('k0, k1, k2, k3')

	solution = nsolve((
		k0**2 + k1**2 - 2*k0*k1*v01 - r01**2,
		k0**2 + k2**2 - 2*k0*k2*v02 - r02**2,
		k0**2 + k3**2 - 2*k0*k3*v03 - r03**2,
		k2**2 + k1**2 - 2*k2*k1*v12 - r12**2,
		k3**2 + k1**2 - 2*k3*k1*v13 - r13**2,
		k2**2 + k3**2 - 2*k2*k3*v23 - r23**2,
	),
                (k0, k1, k2, k3),
		(1000, 1000, 1000, 1000),
		verify = False
	)

	p0 = np.float32(v0 * solution[0])
	p1 = np.float32(v1 * solution[1])
	p2 = np.float32(v2 * solution[2])
	p3 = np.float32(v3 * solution[3])

	err01 = np.linalg.norm(p0 - p1) - r01
	err02 = np.linalg.norm(p0 - p2) - r02
	err03 = np.linalg.norm(p0 - p3) - r03
	err12 = np.linalg.norm(p1 - p2) - r12
	err13 = np.linalg.norm(p1 - p3) - r13
	err23 = np.linalg.norm(p2 - p3) - r23

	return ([p0, p1, p2, p3], [err01, err02, err03, err12, err13, err23])


def solve(sensorGrid, sensorTicks):
	sensorPoints, errors = calcSensorDistances(sensorGrid, sensorTicks)

	# Calculate the normal vector of each corner of the sensor plane
	# as seen by the lighthouse
	planeNormals = [
		np.cross(sensorPoints[1] - sensorPoints[0], sensorPoints[0] - sensorPoints[3]),
		np.cross(sensorPoints[0] - sensorPoints[1], sensorPoints[2] - sensorPoints[1]),
		np.cross(sensorPoints[1] - sensorPoints[2], sensorPoints[3] - sensorPoints[2]),
		np.cross(sensorPoints[2] - sensorPoints[3], sensorPoints[0] - sensorPoints[3])
	]

	# Average the normals and convert to unit vector
	averageNormal = np.float32([0, 0, 0])
	for normal in planeNormals:
		averageNormal += normal
	averageNormal /= len(planeNormals)
	averageNormal /= np.linalg.norm(averageNormal)

	# Calculate rotation from plane normal as seen by lighthouse
	# to plane normal from the sensor's point of view
	rotationMatrixToFlat = calcRotationMatrix(averageNormal, np.float32([0, 0, 1]))
	
	# Calculate center of plane as seen by lighthouse
	center1 = sensorPoints[0] + ((sensorPoints[2] - sensorPoints[0]) / 2)
	center2 = sensorPoints[1] + ((sensorPoints[3] - sensorPoints[1]) / 2)
	averageCenter = (center1 + center2) / 2

	# Translate lighthouse and sensor points so that they are centered on the origin
	# and rotate so they are flat (Z is stright up from the sensor plane)
	lighthousePoint = rotationMatrixToFlat.dot(-averageCenter)
	lighthouseNormal = rotationMatrixToFlat.dot([0, 0, 1])
	sensorPoints[0] = rotationMatrixToFlat.dot(sensorPoints[0] - averageCenter)
	sensorPoints[1] = rotationMatrixToFlat.dot(sensorPoints[1] - averageCenter)
	sensorPoints[2] = rotationMatrixToFlat.dot(sensorPoints[2] - averageCenter)
	sensorPoints[3] = rotationMatrixToFlat.dot(sensorPoints[3] - averageCenter)

	# Calculate the rotation around the sensor normal vector
	# to get X and Y in the correct orientation and rotate the points into place
	rotationMatrix = calcRotationMatrix(sensorPoints[1] - sensorPoints[0], np.float32([0, 1, 0]))
	lighthousePoint = rotationMatrix.dot(lighthousePoint)
	lighthouseNormal = rotationMatrix.dot(lighthouseNormal)
	sensorPoints[0] = rotationMatrix.dot(sensorPoints[0])
	sensorPoints[1] = rotationMatrix.dot(sensorPoints[1])
	sensorPoints[2] = rotationMatrix.dot(sensorPoints[2])
	sensorPoints[3] = rotationMatrix.dot(sensorPoints[3])

	# Calculate the rotation matrix to convert from lighthouse to world	
	lighthouseRotationMatrix = calcRotationMatrix(np.float32([0, 0, 1]), lighthouseNormal)

	return (lighthouseRotationMatrix, lighthousePoint, sensorPoints, errors)


def collectSamples(input, count):
	total = 0
	counts = [0, 0, 0, 0]
	lighthouse1 = [
		[0, 0],
		[0, 0],
		[0, 0],
		[0, 0]
	]
	lighthouse2 = [
		[0, 0],
		[0, 0],
		[0, 0],
		[0, 0]
	]

	n = 0
	while n < count:
		line = input.readline()
		if not line.startswith('AT:'):
			continue

		line = line[line.find('(') + 1 : line.rfind(')')]
		cols = line.split(',', 6)

		i = int(cols[0])
		if i < 0 or i > len(lighthouse1):
			continue

		total += 1
		counts[i] += 1
		lighthouse1[i][0] += int(cols[1])
		lighthouse1[i][1] += int(cols[2])
		lighthouse2[i][0] += int(cols[3])
		lighthouse2[i][1] += int(cols[4])

		sys.stderr.write('.')
		n += 1

	sys.stdout.write('\n')

	for i in range(4):
		lighthouse1[i][0] /= counts[i]
		lighthouse1[i][1] /= counts[i]
		lighthouse2[i][0] /= counts[i]
		lighthouse2[i][1] /= counts[i]

	return (lighthouse1, lighthouse2)


def printSolutionText(solutions):
	for index, solution in enumerate(solutions):
		lr, lp, sp, err = solution
		print "lighthouse %u:" % index
		print "\trotation: "
		for row in lr:
			print "\t\t", row
		print "\tposition: "
		print "\t\t", lp
		print "\terrors: "
		print "\t\t0->1: %.04fmm" % err[0]
		print "\t\t0->2: %.04fmm" % err[1]
		print "\t\t0->3: %.04fmm" % err[2]
		print "\t\t1->2: %.04fmm" % err[3]
		print "\t\t1->3: %.04fmm" % err[4]
		print "\t\t2->3: %.04fmm" % err[5]
		print


def printSolutionC(solutions):
	print "static lighthouse_t lighhouses[%u] = {" % len(solutions)
	for i, solution in enumerate(solutions):
		lr, lp, sp, err = solution
		print "\t{"
		print "\t\t{"
		for j, row in enumerate(lr):
			if j != len(lr) - 1:
				c = ','
			else:
				c = ''
			print "\t\t\t%.05f, %.05f, %.05f%s" % (row[0], row[1], row[2], c)
		print "\t\t},"
		print "\t\t{"
		print "\t\t\t%.05f, %.05f, %.05f" % (lp[0], lp[1], lp[2])
		print "\t\t}"
		if i == len(solutions) - 1:
			print "\t}"
		else:
			print "\t},"
	print "};"


def printSolutionGnuplot(solutions):
	for solution in solutions:
		lr, lp, sp, err = solution

		v = lambda v: ' '.join([str(x) for x in v])

		# draw lighthouse normal
		print v(lp), v(lr.dot([0, 0, -100])), 0

		# draw vectors from lighthouse to sensors
		print v(lp), v(sp[0] - lp), 0
		print v(lp), v(sp[1] - lp), 1
		print v(lp), v(sp[2] - lp), 2
		print v(lp), v(sp[3] - lp), 3

		# draw sensor plane
		print v(sp[0]), v(sp[1] - sp[0]), 0
		print v(sp[0]), v(sp[2] - sp[0]), 0
		print v(sp[0]), v(sp[3] - sp[0]), 0
		print v(sp[1]), v(sp[2] - sp[1]), 1
		print v(sp[1]), v(sp[3] - sp[1]), 1
		print v(sp[2]), v(sp[3] - sp[2]), 2

	sys.stderr.write('plot with: splot "FILENAME" using 1:2:3:4:5:6:7 with vectors lc palette\n')



parser = argparse.ArgumentParser()
parser.add_argument('input', type=str,
	help='input source: filename, serial device (e.g. /dev/ttyS0), or "-" for stdin')
parser.add_argument('--format', type=str,
	help='output format: text (default), c, gnuplot',
	default='text')
parser.add_argument('--bps', type=int,
	help='BPS for serial input (default 115200)',
	default=115200)
parser.add_argument('--count', type=int,
	help='number of samples to collect and average (default 50)',
	default=50)
parser.add_argument('--grid', type=int,
	help='sensor grid spacing in mm (default 60)',
	default=60)
args = parser.parse_args()

input = None
if args.input == '-':
	input = sys.stdin
elif os.path.isfile(args.input):
	input = open(args.input)
elif os.path.exists(args.input):
	input = serial.Serial(args.input, args.bps)
else:
	print "can't open %s" % args.input
	sys.exit(1)

lighthouse1, lighthouse2 = collectSamples(input, args.count)
solution1 = solve(args.grid, lighthouse1)
solution2 = solve(args.grid, lighthouse2)

if args.format == 'text':
	printSolutionText((solution1, solution2))
elif args.format == 'c':
	printSolutionC((solution1, solution2))
elif args.format == 'gnuplot':
	printSolutionGnuplot((solution1, solution2))
else:
	print 'unknown output format'

