#include "ALTITUDE_KALMAN_FILTER.h"

void altitudeKFInit(AltitudeKF* kf, float initZ, float Q, float R) {
	kf->z = initZ;
	kf->v = 0.0f;
	kf->Q = Q;
	kf->R = R;

	kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
}

void altitudeKFPredict(AltitudeKF *kf, float acc, float dt) {
	kf->z = kf->z + (kf->v * dt) + (0.5f * acc * dt * dt);
	kf->v = kf->v + (acc * dt);

	float p00 = kf->P[0][0] + dt * (kf->P[1][0] + kf->P[0][1] + dt * kf->P[1][1]);
	float p01 = kf->P[0][1] + dt * kf->P[1][1];
	float p10 = kf->P[1][0] + dt * kf->P[1][1];
	float p11 = kf->P[1][1] + kf->Q * dt;

	kf->P[0][0] = p00; kf->P[0][1] = p01;
	kf->P[1][0] = p01; kf->P[1][1] = p11;
}

void altitudeKFUpdate(AltitudeKF* kf, float baroAltitude) {

	if (fabsf(kf->v) > (0.7f * 343.0f)) {
		return;
	}

	float y = baroAltitude - kf->z;
	float S = kf->P[0][0] + kf->R;
	float K0 = kf->P[0][0] / S;
	float K1 = kf->P[1][0] / S;

	kf->z += K0 * y;
	kf->v += K1 * y;

	float p00 = (1.0f - K0) * kf->P[0][0];
	float p01 = (1.0f - K0) * kf->P[0][1];
	float p11 = kf->P[1][1] - (K1 * kf->P[0][1]);

	kf->P[0][0] = p00; kf->P[0][1] = p01;
	kf->P[1][0] = p01; kf->P[1][1] = p11;
}

float getVerticalAccel(float ax, float ay, float az, float pitch_deg, float roll_deg) {
	float p = pitch_deg * 0.01745329f;
	float r = roll_deg * 0.01745329f;

	float cosP = cosf(p);
	float sinP = sinf(p);
	float cosR = cosf(r);
	float sinR = sinf(r);

	float world_accel_z = -(ax * sinP) +
		(ay * cosP * sinR) +
		(az * cosP * cosR);

	return world_accel_z;
}
