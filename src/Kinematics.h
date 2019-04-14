#include <math.h>

#define EFFECTOR_RADIUS	25.0
#define BASE_RADIUS 75.0
#define FOREARM_LENGTH 220.0
#define BICEPS_LENGTH 140.0

int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
