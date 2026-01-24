#include <math.h>
#include <stdio.h>
#include "APOGEE_PREDICTION.h"
#define GRAVITY 9.81f
#define RHO_0 1.225f
#define H_SCALE 8500.0f
#define SIM_DT 0.05f

float predict_apogee_sim(float z_curr, float v_curr, float mass, float area, float cd) {
    float z_sim = z_curr;
    float v_sim = v_curr;

    // Fast-forward physics until velocity is 0 (Apogee)
    while (v_sim > 0) {
        // 1. Calculate air density at current sim altitude
        float rho = RHO_0 * expf(-z_sim / H_SCALE);

        // 2. Calculate Drag Force (F = 0.5 * rho * v^2 * cd * A)
        float f_drag = 0.5f * rho * v_sim * v_sim * cd * area;

        // 3. Update Velocity (v_dot = -g - F/m)
        float v_dot = -GRAVITY - (f_drag / mass);
        v_sim += v_dot * SIM_DT;

        // 4. Update Position
        z_sim += v_sim * SIM_DT;

        // Safety break to prevent infinite loops if something goes wrong
        if (z_sim > 20000.0f || z_sim < 0.0f) break;
    }

    return z_sim;
}