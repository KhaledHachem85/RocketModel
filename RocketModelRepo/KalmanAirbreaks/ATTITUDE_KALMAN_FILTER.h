#ifndef ATTITUDE_KALMAN_FILTER_H
#define ATTITUDE_KALMAN_FILTER_H

#include <math.h>
#include <stdbool.h>

typedef struct {
	float angle;
	float variance;
	float Q;
	float R;
	float g_upper;
	float g_lower;
} AttitudeKF;

void attitudeKFInit(AttitudeKF *kf, float initAngle, float Q, float R);
void attitudeKFPredict(AttitudeKF *kf, float w, float dt);
void attitudeKFUpdate(AttitudeKF* kf, float theta, float totalG);

#endif