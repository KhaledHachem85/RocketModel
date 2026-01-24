#include "ATTITUDE_KALMAN_FILTER.h"

void attitudeKFInit(AttitudeKF* kf, float initAngle, float Q, float R) {
	kf->angle = initAngle;
	kf->Q = Q;
	kf->R = R;
	kf->variance = 0.1;

	kf->g_lower = 0.9f;
	kf->g_upper = 1.1f;
}

void attitudeKFPredict(AttitudeKF* kf, float w, float dt) {
	kf->angle += w * dt;
	kf->variance += kf->Q * dt;
}

void attitudeKFUpdate(AttitudeKF* kf, float theta, float totalG) {
	if (totalG < kf->g_lower || totalG > kf->g_upper) {
		return;
	}

	float K = kf->variance / (kf->variance + kf->R);

	float innovation = theta - kf->angle;
	kf->angle += K * innovation;

	kf->variance = (1.0 - K) * kf->variance;
}