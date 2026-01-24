#ifndef ALTITUDE_KALMAN_FILTER_H
#define ALTITUDE_KALMAN_FILTER_H

#include<math.h>

typedef struct {
	float z;
	float v;
	float P[2][2];
	float Q;
	float R;
} AltitudeKF;

void altitudeKFInit(AltitudeKF *kf, float initZ, float Q, float R);
void altitudeKFPredict(AltitudeKF *kf, float acceleration, float dt);
void altitudeKFUpdate(AltitudeKF* kf, float barometerAltitude);
float getVerticalAccel(float ax, float ay, float az, float pitch_deg, float roll_deg);

#endif