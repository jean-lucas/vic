#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>


void vichw_init_ultrasonic(void);

/* Returns distance in cm */
uint32_t vichw_distance(void);

/* Nonzero if obstacle, 0 if no obstacle */
int vichw_is_obstacle(void);

#endif
