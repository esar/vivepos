#include <stdint.h>

typedef _Accum scalar_t;
//typedef float scalar_t;

typedef scalar_t vec3_t[3];

typedef struct
{
	scalar_t rotationMatrix[9];
	vec3_t origin;

} lighthouse_t;

uint8_t calc_position(lighthouse_t* lighthouses,
                      scalar_t angle1, scalar_t angle2, 
                      scalar_t angle3, scalar_t angle4,
                      vec3_t result, scalar_t* distance);
