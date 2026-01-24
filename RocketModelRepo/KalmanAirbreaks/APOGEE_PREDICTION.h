#pragma once
#ifndef APOGEE_PREDICTOR_H
#define APOGEE_PREDICTOR_H

#include <math.h>

#define GRAVITY 9.80665f
#define RHO_0 1.225f
#define H_SCALE 8500.0f

#define SIM_DT 0.05f

float predict_apogee_sim(float z_curr, float v_curr, float mass, float area, float cd);

#endif