#include <math.h>

#include "position.h"



static void vec_cross_product(const vec3_t const a, const vec3_t const b, vec3_t result)
{
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
}

static scalar_t vec_length(vec3_t v)
{
	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void calc_ray_vect(scalar_t rotation[9], scalar_t angle1, scalar_t angle2, vec3_t result)
{
	vec3_t a = { cos(angle1), 0, -sin(angle1) };
	vec3_t b = { 0, cos(angle2), sin(angle2) };
	vec3_t ray;
	scalar_t length;

	vec_cross_product(b, a, ray);

	length = vec_length(ray);
	ray[0] *= (scalar_t)1.0/length;
	ray[1] *= (scalar_t)1.0/length;
	ray[2] *= (scalar_t)1.0/length;

	result[0] = rotation[0]*ray[0] + rotation[1]*ray[1] + rotation[2]*ray[2];
	result[1] = rotation[3]*ray[0] + rotation[4]*ray[1] + rotation[5]*ray[2];
	result[2] = rotation[6]*ray[0] + rotation[7]*ray[1] + rotation[8]*ray[2];
}

static uint8_t intersect_lines(vec3_t origin1, vec3_t ray1, vec3_t origin2, vec3_t ray2, vec3_t result, scalar_t* distance)
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
	d = ray1[0]*w0[0] + ray1[1]*w0[1] + ray1[2]*w0[2];
	e = ray2[0]*w0[0] + ray2[1]*w0[1] + ray2[2]*w0[2];

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
	*distance = vec_length(p1);

	return 1;
}

uint8_t calc_position(lighthouse_t* lighthouses,
                      scalar_t angle1, scalar_t angle2, 
                      scalar_t angle3, scalar_t angle4,
                      vec3_t result, scalar_t* distance)
{
	vec3_t ray1;
	vec3_t ray2;

	calc_ray_vect(lighthouses[0].rotationMatrix, angle1, angle2, ray1);
	calc_ray_vect(lighthouses[1].rotationMatrix, angle3, angle4, ray2);

	return intersect_lines(lighthouses[0].origin, ray1, lighthouses[1].origin, ray2, result, distance);
}

