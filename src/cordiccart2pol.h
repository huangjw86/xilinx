#include "ap_fixed.h"
#ifndef CORDICCART2POL_H
#define CORDICCART2POL_H

#define NO_ITER 16

typedef int   coef_t;
typedef float data_t;
typedef float acc_t;

void cordiccart2pol(data_t x, data_t y, data_t *theta,  data_t *r);

#endif
