#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vivepos.h"

#define LSTSQ_ITERATIONS    10
#define LSTSQ_LIMIT         0.025


scalar_t r01 = 30.0;
scalar_t r02 = 42.426406871;
scalar_t r03 = 30.0;
scalar_t r12 = 30.0;
scalar_t r13 = 42.426406871;
scalar_t r23 = 30.0;


void vec3Cross(vec3_t out, const vec3_t const a, const vec3_t const b)
{
	out[0] = a[1]*b[2] - a[2]*b[1];
	out[1] = a[2]*b[0] - a[0]*b[2];
	out[2] = a[0]*b[1] - a[1]*b[0];
}

scalar_t vec3Dot(const vec3_t const a, const vec3_t const b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

scalar_t vec3Len(const vec3_t const v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void vec3Sub(vec3_t out, const vec3_t const a, const vec3_t const b)
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

void vec3Add(vec3_t out, const vec3_t const a, const vec3_t const b)
{
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
}

void vec3Mul(vec3_t out, const vec3_t const a, scalar_t b)
{
	out[0] = a[0] * b;
	out[1] = a[1] * b;
	out[2] = a[2] * b;
}

void vec3Div(vec3_t out, const vec3_t const a, scalar_t b)
{
	out[0] = a[0] / b;
	out[1] = a[1] / b;
	out[2] = a[2] / b;
}

void mat4Inv(mat4_4_t out, mat4_4_t in)
{
	scalar_t s0 = in[0*4+0] * in[1*4+1] - in[1*4+0] * in[0*4+1];
	scalar_t s1 = in[0*4+0] * in[1*4+2] - in[1*4+0] * in[0*4+2];
	scalar_t s2 = in[0*4+0] * in[1*4+3] - in[1*4+0] * in[0*4+3];
	scalar_t s3 = in[0*4+1] * in[1*4+2] - in[1*4+1] * in[0*4+2];
	scalar_t s4 = in[0*4+1] * in[1*4+3] - in[1*4+1] * in[0*4+3];
	scalar_t s5 = in[0*4+2] * in[1*4+3] - in[1*4+2] * in[0*4+3];

	scalar_t c5 = in[2*4+2] * in[3*4+3] - in[3*4+2] * in[2*4+3];
	scalar_t c4 = in[2*4+1] * in[3*4+3] - in[3*4+1] * in[2*4+3];
	scalar_t c3 = in[2*4+1] * in[3*4+2] - in[3*4+1] * in[2*4+2];
	scalar_t c2 = in[2*4+0] * in[3*4+3] - in[3*4+0] * in[2*4+3];
	scalar_t c1 = in[2*4+0] * in[3*4+2] - in[3*4+0] * in[2*4+2];
	scalar_t c0 = in[2*4+0] * in[3*4+1] - in[3*4+0] * in[2*4+1];

	// Should check for 0 determinant
	scalar_t invdet = 1 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

	out[0*4+0] = ( in[1*4+1] * c5 - in[1*4+2] * c4 + in[1*4+3] * c3) * invdet;
	out[0*4+1] = (-in[0*4+1] * c5 + in[0*4+2] * c4 - in[0*4+3] * c3) * invdet;
	out[0*4+2] = ( in[3*4+1] * s5 - in[3*4+2] * s4 + in[3*4+3] * s3) * invdet;
	out[0*4+3] = (-in[2*4+1] * s5 + in[2*4+2] * s4 - in[2*4+3] * s3) * invdet;

	out[1*4+0] = (-in[1*4+0] * c5 + in[1*4+2] * c2 - in[1*4+3] * c1) * invdet;
	out[1*4+1] = ( in[0*4+0] * c5 - in[0*4+2] * c2 + in[0*4+3] * c1) * invdet;
	out[1*4+2] = (-in[3*4+0] * s5 + in[3*4+2] * s2 - in[3*4+3] * s1) * invdet;
	out[1*4+3] = ( in[2*4+0] * s5 - in[2*4+2] * s2 + in[2*4+3] * s1) * invdet;

	out[2*4+0] = ( in[1*4+0] * c4 - in[1*4+1] * c2 + in[1*4+3] * c0) * invdet;
	out[2*4+1] = (-in[0*4+0] * c4 + in[0*4+1] * c2 - in[0*4+3] * c0) * invdet;
	out[2*4+2] = ( in[3*4+0] * s4 - in[3*4+1] * s2 + in[3*4+3] * s0) * invdet;
	out[2*4+3] = (-in[2*4+0] * s4 + in[2*4+1] * s2 - in[2*4+3] * s0) * invdet;

	out[3*4+0] = (-in[1*4+0] * c3 + in[1*4+1] * c1 - in[1*4+2] * c0) * invdet;
	out[3*4+1] = ( in[0*4+0] * c3 - in[0*4+1] * c1 + in[0*4+2] * c0) * invdet;
	out[3*4+2] = (-in[3*4+0] * s3 + in[3*4+1] * s1 - in[3*4+2] * s0) * invdet;
	out[3*4+3] = ( in[2*4+0] * s3 - in[2*4+1] * s1 + in[2*4+2] * s0) * invdet;
}

void matDot(scalar_t* out,
            scalar_t* a, uint8_t am, uint8_t an, 
            scalar_t* b, uint8_t bm, uint8_t bn)
{
	uint8_t i, j, k;

	for(i = 0; i < am; ++i)
	{
		for(j = 0; j < bn; ++j)
		{
			scalar_t sum = 0;
			for(k = 0; k < an; ++k)
				sum += a[i*an + k] * b[k*bn + j];
			out[i*bn + j] = sum;
		}
	}
}

void matDotAT(scalar_t* out, 
              scalar_t* a, uint8_t am, uint8_t an, 
              scalar_t* b, uint8_t bm, uint8_t bn)
{
	uint8_t i, j, k;

	for(i = 0; i < am; ++i)
	{
		for(j = 0; j < bn; ++j)
		{
			scalar_t sum = 0;
			for(k = 0; k < an; ++k)
				sum += a[k*am + i] * b[k*bn + j];
			out[i*am + j] = sum;
		}
	}
}

void matDotBT(scalar_t* out, 
              scalar_t* a, uint8_t am, uint8_t an, 
              scalar_t* b, uint8_t bm, uint8_t bn)
{
	uint8_t i, j, k;

	for(i = 0; i < am; ++i)
	{
		for(j = 0; j < bn; ++j)
		{
			scalar_t sum = 0;
			for(k = 0; k < bm; ++k)
				sum += a[i*am + k] * b[j*bm + k];
			out[i*bn + j] = sum;
		}
	}
}

void anglesToVector(vec3_t out, scalar_t xAngle, scalar_t yAngle)
{
	scalar_t length;
	vec3_t plane1 = { +cos(xAngle), 0, -sin(xAngle) };
	vec3_t plane2 = { 0, +cos(yAngle), +sin(yAngle) };

	vec3Cross(out, plane2, plane1);
	length = vec3Len(out);
	vec3Div(out, out, length);
}

// note: 'from' and 'to' must be unit vectors
void calcRotationMatrix(mat3_3_t out, vec3_t from, vec3_t to)
{
	vec3_t uvw;
	scalar_t rcos;
	scalar_t rsin;

	vec3Cross(uvw, from, to);
	rcos = vec3Dot(from, to);
	rsin = vec3Len(uvw);
	if(rsin != (scalar_t)0)
		vec3Div(uvw, uvw, rsin);

	out[0*3 + 0] =  rcos          + uvw[0] * uvw[0] * ((scalar_t)1.0 - rcos);
	out[0*3 + 1] = -uvw[2] * rsin + uvw[0] * uvw[1] * ((scalar_t)1.0 - rcos);
	out[0*3 + 2] =  uvw[1] * rsin + uvw[0] * uvw[2] * ((scalar_t)1.0 - rcos);
	out[1*3 + 0] =  uvw[2] * rsin + uvw[1] * uvw[0] * ((scalar_t)1.0 - rcos);
	out[1*3 + 1] =  rcos          + uvw[1] * uvw[1] * ((scalar_t)1.0 - rcos);
	out[1*3 + 2] = -uvw[0] * rsin + uvw[1] * uvw[2] * ((scalar_t)1.0 - rcos);
	out[2*3 + 0] = -uvw[1] * rsin + uvw[2] * uvw[0] * ((scalar_t)1.0 - rcos);
	out[2*3 + 1] =  uvw[0] * rsin + uvw[2] * uvw[1] * ((scalar_t)1.0 - rcos);
	out[2*3 + 2] =  rcos          + uvw[2] * uvw[2] * ((scalar_t)1.0 - rcos);
}

void calcAverageSensorNormal(vec3_t out, vec3_t points[4])
{
	vec3_t tmp0, tmp1, tmp2;

	// Accumulate the normals from the 4 corners of the plane
	vec3Sub(tmp0, points[1], points[0]);
	vec3Sub(tmp1, points[0], points[3]);
	vec3Cross(out, tmp0, tmp1);

	vec3Sub(tmp0, points[0], points[1]);
	vec3Sub(tmp1, points[2], points[1]);
	vec3Cross(tmp2, tmp0, tmp1);
	vec3Add(out, out, tmp2);

	vec3Sub(tmp0, points[1], points[2]);
	vec3Sub(tmp1, points[3], points[2]);
	vec3Cross(tmp2, tmp0, tmp1);
	vec3Add(out, out, tmp2);

	vec3Sub(tmp0, points[2], points[3]);
	vec3Sub(tmp1, points[0], points[3]);
	vec3Cross(tmp2, tmp0, tmp1);
	vec3Add(out, out, tmp2);

	// Average the 4 normals
	vec3Div(out, out, 4);

	// Scale to unit vector
	vec3Div(out, out, vec3Len(out));
}

void calcPlaneCenter(vec3_t out, vec3_t points[4])
{
	vec3_t center1, center2;

	// Find the center along one diagonal
	vec3Sub(center1, points[2], points[0]);
	vec3Div(center1, center1, 2);
	vec3Add(center1, center1, points[0]);

	// Find the center along the other diagonal
	vec3Sub(center2, points[3], points[1]);
	vec3Div(center2, center2, 2);
	vec3Add(center2, center2, points[1]);

	// Average the two
	vec3Add(out, center1, center2);
	vec3Div(out, out, 2);
}

void calcSolutionError(vec3_t points[4])
{
	vec3_t e01, e02, e03, e12, e13, e23;

	vec3Sub(e01, points[0], points[1]);
	vec3Sub(e02, points[0], points[2]);
	vec3Sub(e03, points[0], points[3]);
	vec3Sub(e12, points[1], points[2]);
	vec3Sub(e13, points[1], points[3]);
	vec3Sub(e23, points[2], points[3]);

	//printf("err01: %f\r\n", vec3Len(e01) - r01);
	//printf("err02: %f\r\n", vec3Len(e02) - r02);
	//printf("err03: %f\r\n", vec3Len(e03) - r03);
	//printf("err12: %f\r\n", vec3Len(e12) - r12);
	//printf("err13: %f\r\n", vec3Len(e13) - r13);
	//printf("err23: %f\r\n", vec3Len(e23) - r23);
}

scalar_t residual(scalar_t a, scalar_t b, scalar_t c, scalar_t d)
{
	return a*a + b*b - 2*a*b*c - d*d;
}

scalar_t partial(scalar_t a, scalar_t b, scalar_t c)
{
	return 2*(a - b*c);
}

// stack: local vars: 323 bytes, calling inv4: +52bytes
void calcSensorPositions(vec3_t points[4], sensor_t* sensors, uint8_t lighthouse)
{
	uint8_t i;
	vec6_t r;
	mat6_4_t J;
	mat6_4_t tmpA;
	mat6_4_t tmpB;
	scalar_t v01, v02, v03, v12, v13, v23;
	scalar_t distance[4] = {1000.0, 1000.0, 1000.0, 1000.0};


	// Convert angles to vectors that point to sensors
	anglesToVector(points[0], 
	               sensors[0].angleRadians[lighthouse][AXIS0],
	               sensors[0].angleRadians[lighthouse][AXIS1]);
	anglesToVector(points[1],
	               sensors[1].angleRadians[lighthouse][AXIS0],
	               sensors[1].angleRadians[lighthouse][AXIS1]);
	anglesToVector(points[2],
	               sensors[2].angleRadians[lighthouse][AXIS0],
	               sensors[2].angleRadians[lighthouse][AXIS1]);
	anglesToVector(points[3],
	               sensors[3].angleRadians[lighthouse][AXIS0],
	               sensors[3].angleRadians[lighthouse][AXIS1]);

	v01 = vec3Dot(points[0], points[1]);
	v02 = vec3Dot(points[0], points[2]);
	v03 = vec3Dot(points[0], points[3]);
	v12 = vec3Dot(points[1], points[2]);
	v13 = vec3Dot(points[1], points[3]);
	v23 = vec3Dot(points[2], points[3]);

	// Solve distances using Least Squares by Gauss-Newton algorithm
	for(i = 0; i < LSTSQ_ITERATIONS; ++i)
	{
		// Calculate residuals for this iteration
		r[0] = residual(distance[0], distance[1], v01, r01);
		r[1] = residual(distance[0], distance[2], v02, r02);
		r[2] = residual(distance[0], distance[3], v03, r03);
		r[3] = residual(distance[1], distance[2], v12, r12);
		r[4] = residual(distance[1], distance[3], v13, r13);
		r[5] = residual(distance[2], distance[3], v23, r23);

		// Populate Jacobian matrix with results of partial derivatives
		J[0*4+0] = partial(distance[0], distance[1], v01);
		J[0*4+1] = partial(distance[1], distance[0], v01);
		J[0*4+2] = 0;
		J[0*4+3] = 0;

		J[1*4+0] = partial(distance[0], distance[2], v02);
		J[1*4+1] = 0;
		J[1*4+2] = partial(distance[2], distance[0], v02);
		J[1*4+3] = 0;

		J[2*4+0] = partial(distance[0], distance[3], v03);
		J[2*4+1] = 0;
		J[2*4+2] = 0;
		J[2*4+3] = partial(distance[3], distance[0], v03);

		J[3*4+0] = 0;
		J[3*4+1] = partial(distance[1], distance[2], v12);
		J[3*4+2] = partial(distance[2], distance[1], v12);
		J[3*4+3] = 0;

		J[4*4+0] = 0;
		J[4*4+1] = partial(distance[1], distance[3], v13);
		J[4*4+2] = 0;
		J[4*4+3] = partial(distance[3], distance[1], v13);

		J[5*4+0] = 0;
		J[5*4+1] = 0;
		J[5*4+2] = partial(distance[2], distance[3], v23);
		J[5*4+3] = partial(distance[3], distance[2], v23);

		// Solve resulting system of four linear equations
		// (Jt * J)^-1 * Jt * r
		matDotAT(tmpA, J, 4, 6, J, 6, 4);
		mat4Inv(tmpB, tmpA);
		matDotBT(tmpA, tmpB, 4, 4, J, 4, 6);
		matDot(tmpB, tmpA, 4, 6, r, 6, 1);

		// Update estimated distances
		distance[0] -= tmpB[0];
		distance[1] -= tmpB[1];
		distance[2] -= tmpB[2];
		distance[3] -= tmpB[3];

		// Exit early if the last change was small
		if(fabs(tmpB[0]) < LSTSQ_LIMIT && 
		   fabs(tmpB[1]) < LSTSQ_LIMIT && 
		   fabs(tmpB[2]) < LSTSQ_LIMIT && 
		   fabs(tmpB[3]) < LSTSQ_LIMIT)
		{
			break;
		}
	}

	// Scale the direction vectors to turn them into sensor coordinates
	vec3Mul(points[0], points[0], distance[0]);
	vec3Mul(points[1], points[1], distance[1]);
	vec3Mul(points[2], points[2], distance[2]);
	vec3Mul(points[3], points[3], distance[3]);
}

void convertToMasterCoordinates(vec3_t slavePosition, 
                                mat3_3_t slaveRotation, 
                                vec3_t slavePoints[4],
                                vec3_t masterPoints[4])
{
	uint8_t i;
	mat3_3_t rotation, tmprot;
	vec3_t from, to;
	vec3_t tmp;
	scalar_t length;

	
	// Calculate sensor plane normal from slave's point of view
	calcAverageSensorNormal(from, slavePoints);

	// Calculate sensor plane normal from master's point of view
	calcAverageSensorNormal(to, masterPoints);

	// Calculate rotation required to align slave normal to master
	calcRotationMatrix(slaveRotation, from, to);

	// Find center of sensor plane from slave's point of view
	calcPlaneCenter(from, slavePoints);

	// Translate the slave lighthouse and sensors so that the sensor plane is 
	// centered on the origin, and rotate so that it's normal is aligned with
	// the master's normal
	tmp[0] = -from[0];
	tmp[1] = -from[1];
	tmp[2] = -from[2];
	matDot(slavePosition, slaveRotation, 3, 3, tmp, 3, 1);
	for(i = 0; i < 4; ++i)
	{
		vec3Sub(tmp, slavePoints[i], from);
		matDot(slavePoints[i], slaveRotation, 3, 3, tmp, 3, 1);
	}

	// Calculate rotation around the normal to align an edge of the sensor
	// plane as seen by the slave with the same edge as seen by the master
	vec3Sub(from, slavePoints[1], slavePoints[0]);
	length = vec3Len(from);
	vec3Div(from, from, length);
	vec3Sub(to, masterPoints[1], masterPoints[0]);
	length = vec3Len(to);
	vec3Div(to, to, length);
	calcRotationMatrix(rotation, from, to);
	
	// Rotate slave lighthouse to match, don't need to rotate the
	// slave's sensors, they should be identical to the master at the end
	tmp[0] = slavePosition[0];
	tmp[1] = slavePosition[1];
	tmp[2] = slavePosition[2];
	matDot(slavePosition, rotation, 3, 3, tmp, 3, 1);

	// Find center of sensor plane from master's point of view
	calcPlaneCenter(to, masterPoints);

	// Translate the slave lighthouse to account for moving
	// the sensor plane from the origin to the master's idea of
	// the sensor coordinates
	vec3Add(slavePosition, slavePosition, to);

	// Rotate slaveRotation matrix
	memcpy(tmprot, slaveRotation, 3*3*sizeof(scalar_t));
	matDot(slaveRotation, rotation, 3, 3, tmprot, 3, 3);
}

void rotateForGravity(vec3_t masterPosition, mat3_3_t masterRotation, vec3_t masterZ,
                      vec3_t slavePosition,  mat3_3_t slaveRotation,  vec3_t slaveZ)
{
	vec3_t from, to;
	mat3_3_t rotation, tmprot;

	// Scale master z vector to unit vector and rotate into world
	vec3Div(from, masterZ, vec3Len(masterZ));
	matDot(masterZ, masterRotation, 3, 3, from, 3, 1);

	// Scale slave z vector to unit vector and rotate into world
	vec3Mul(from, slaveZ, (scalar_t)1.0 / vec3Len(slaveZ));
	matDot(slaveZ, slaveRotation, 3, 3, from, 3, 1);

	// Average the two z vectors
	vec3Add(from, masterZ, slaveZ);
	vec3Div(from, from, 2);
	vec3Div(from, from, vec3Len(from));

	// Calculate rotation to align world with gravity	
	to[0] = 0; to[1] = 1; to[2] = 0;
	calcRotationMatrix(rotation, from, to);

	// Rotate the master, no need to rotate position, it's at the origin
	memcpy(tmprot, masterRotation, 3*3*sizeof(scalar_t));
	matDot(masterRotation, rotation, 3, 3, tmprot, 3, 3);

	// Rotate the slave
	from[0] = slavePosition[0]; from[1] = slavePosition[1]; from[2] = slavePosition[2];
	matDot(slavePosition, rotation, 3, 3, from, 3, 1);
	memcpy(tmprot, slaveRotation, 3*3*sizeof(scalar_t));
	matDot(slaveRotation, rotation, 3, 3, tmprot, 3, 3);
}

void solveLighthouse(lighthouse_t* lighthouses, sensor_t* sensors, ootx_msg_t* ootx)
{
	vec3_t masterPoints[4];
	vec3_t slavePoints[4];
	vec3_t masterZ =
	{
		ootx[MASTER].msg.accel_dir_x,
		ootx[MASTER].msg.accel_dir_y,
		ootx[MASTER].msg.accel_dir_z
	};
	vec3_t slaveZ =
	{
		ootx[SLAVE].msg.accel_dir_x,
		ootx[SLAVE].msg.accel_dir_y,
		ootx[SLAVE].msg.accel_dir_z
	};


	// Set master rotation matrix to identity
	lighthouses[0].rotationMatrix[0] = 1;
	lighthouses[0].rotationMatrix[1] = 0;
	lighthouses[0].rotationMatrix[2] = 0;

	lighthouses[0].rotationMatrix[3] = 0;
	lighthouses[0].rotationMatrix[4] = 1;
	lighthouses[0].rotationMatrix[5] = 0;

	lighthouses[0].rotationMatrix[6] = 0;
	lighthouses[0].rotationMatrix[7] = 0;
	lighthouses[0].rotationMatrix[8] = 1;

	// Set master origin to world origin
	lighthouses[0].origin[0] = 0;
	lighthouses[0].origin[1] = 0;
	lighthouses[0].origin[2] = 0;
	
	// Calculate the positions of the sensors as seen by the master lighthouse
	calcSensorPositions(masterPoints, sensors, MASTER);
	calcSolutionError(masterPoints);

	// Calculate the positions of the sensors as seen by the slave lighthouse
	calcSensorPositions(slavePoints, sensors, SLAVE);
	calcSolutionError(slavePoints);

	// Convert the slave positions/rotations to the master's coordinate space
	convertToMasterCoordinates(lighthouses[1].origin, lighthouses[1].rotationMatrix, 
	                           slavePoints, masterPoints);

	// Rotate everything for the gravity direction from the lighthouse accelerometer data
	rotateForGravity(lighthouses[0].origin, lighthouses[0].rotationMatrix, masterZ, 
	                 lighthouses[1].origin, lighthouses[1].rotationMatrix, slaveZ);
}

uint8_t intersectLines(vec3_t origin1, vec3_t ray1, vec3_t origin2, vec3_t ray2, vec3_t result, scalar_t* distance)
{
	scalar_t a, b, c, d, e;
	scalar_t denom;
	scalar_t t;
	vec3_t p1, p2;
	vec3_t w0;


	w0[0] = origin1[0] - origin2[0];
	w0[1] = origin1[1] - origin2[1];
	w0[2] = origin1[2] - origin2[2];

	a = ray1[0]*ray1[0] + ray1[1]*ray1[1] + ray1[2]*ray1[2];
	b = ray1[0]*ray2[0] + ray1[1]*ray2[1] + ray1[2]*ray2[2];
	c = ray2[0]*ray2[0] + ray2[1]*ray2[1] + ray2[2]*ray2[2];
	d = ray1[0]*w0[0]   + ray1[1]*w0[1]   + ray1[2]*w0[2];
	e = ray2[0]*w0[0]   + ray2[1]*w0[1]   + ray2[2]*w0[2];

	denom = a * c - b * b;

	if(denom > (scalar_t)-0.00001 && denom < (scalar_t)0.00001)
		return 0;

	t = (b * e - c * d) / denom;
	p1[0] = ray1[0]*t;
	p1[1] = ray1[1]*t;
	p1[2] = ray1[2]*t;
	p1[0] += origin1[0];
	p1[1] += origin1[1];
	p1[2] += origin1[2];

	t = (a * e - b * d) / denom;
	p2[0] = ray2[0]*t;
	p2[1] = ray2[1]*t;
	p2[2] = ray2[2]*t;
	p2[0] += origin2[0];
	p2[1] += origin2[1];
	p2[2] += origin2[2];
	
	result[0] = p1[0] + p2[0];
	result[1] = p1[1] + p2[1];
	result[2] = p1[2] + p2[2];
	result[0] *= (scalar_t)0.5;
	result[1] *= (scalar_t)0.5;
	result[2] *= (scalar_t)0.5;

	p1[0] -= p2[0];
	p1[1] -= p2[1];
	p1[2] -= p2[2];
	*distance = vec3Len(p1);

	return 1;
}

static void calcRayVect(scalar_t rotation[9], scalar_t angle1, scalar_t angle2, vec3_t result)
{
	vec3_t a = { cos(angle1), 0, -sin(angle1) };
	vec3_t b = { 0, cos(angle2), sin(angle2) };
	vec3_t ray;
	scalar_t length;

	vec3Cross(ray, b, a);

	length = vec3Len(ray);
	vec3Mul(ray, ray, (scalar_t)1.0/length);

	result[0] = rotation[0]*ray[0] + rotation[1]*ray[1] + rotation[2]*ray[2];
	result[1] = rotation[3]*ray[0] + rotation[4]*ray[1] + rotation[5]*ray[2];
	result[2] = rotation[6]*ray[0] + rotation[7]*ray[1] + rotation[8]*ray[2];
}

uint8_t calcPosition(lighthouse_t* lighthouses,
                     scalar_t angle1, scalar_t angle2, 
                     scalar_t angle3, scalar_t angle4,
                     vec3_t result, scalar_t* distance)
{
	vec3_t ray1;
	vec3_t ray2;

	calcRayVect(lighthouses[0].rotationMatrix, angle1, angle2, ray1);
	calcRayVect(lighthouses[1].rotationMatrix, angle3, angle4, ray2);

	return intersectLines(lighthouses[0].origin, ray1, lighthouses[1].origin, ray2, result, distance);
}

/*
void main()
{
	lighthouse_t lh[2];

	lh[0].ticks[0] = 19328;
	lh[0].ticks[1] = 10716;
	lh[0].ticks[2] = 18168;
	lh[0].ticks[3] = 10909;
	lh[0].ticks[4] = 17534;
	lh[0].ticks[5] = 10270;
	lh[0].ticks[6] = 18747;
	lh[0].ticks[7] = 10065;
	lh[0].z[0] = 5;
	lh[0].z[1] = 127;
	lh[0].z[2] = 2;
	
	lh[1].ticks[0] = 18031;
	lh[1].ticks[1] = 20023;
	lh[1].ticks[2] = 18017;
	lh[1].ticks[3] = 19459;
	lh[1].ticks[4] = 18405;
	lh[1].ticks[5] = 19339;
	lh[1].ticks[6] = 18416;
	lh[1].ticks[7] = 19884;
	lh[1].z[0] = -127;
	lh[1].z[1] = 32;
	lh[1].z[2] = 76;

	solveLighthouse(lh);

	printf("o: %f, %f, %f\n", lh[0].origin[0], lh[0].origin[1], lh[0].origin[2]);
	printf("r:\n");
	printf("\t%f, %f, %f\n", lh[0].rotationMatrix[0], lh[0].rotationMatrix[1], lh[0].rotationMatrix[2]);
	printf("\t%f, %f, %f\n", lh[0].rotationMatrix[3], lh[0].rotationMatrix[4], lh[0].rotationMatrix[5]);
	printf("\t%f, %f, %f\n", lh[0].rotationMatrix[6], lh[0].rotationMatrix[7], lh[0].rotationMatrix[8]);
	printf("so = np.array([%f, %f, %f])\n", lh[1].origin[0], lh[1].origin[1], lh[1].origin[2]);
	printf("sr = np.array([[%f, %f, %f],\n", lh[1].rotationMatrix[0], lh[1].rotationMatrix[1], lh[1].rotationMatrix[2]);
	printf("\t[%f, %f, %f]\n", lh[1].rotationMatrix[3], lh[1].rotationMatrix[4], lh[1].rotationMatrix[5]);
	printf("\t[%f, %f, %f]])\n", lh[1].rotationMatrix[6], lh[1].rotationMatrix[7], lh[1].rotationMatrix[8]);
}
*/
