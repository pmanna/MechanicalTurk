#include <math.h>

#define EFFECTOR_RADIUS	25.0
#define BASE_RADIUS 75.0
#define FOREARM_LENGTH 220.0
#define BICEPS_LENGTH 140.0

int delta_calcForward(double theta1, double theta2, double theta3, double &x0, double &y0, double &z0);
int delta_calcInverse(double x0, double y0, double z0, double &theta1, double &theta2, double &theta3);
