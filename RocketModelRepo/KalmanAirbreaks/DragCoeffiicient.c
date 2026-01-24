#include "DRAG_COEFFICIENT.h"
#include <math.h>
#include <stdbool.h>

#define PI 3.14159265f
#define G_MS2 9.80665f
#define RHO_0 1.225f      // Sea level air density (kg/m3)
#define H_SCALE 8500.0f   // Scale height for atmosphere (m)

// Rocket Specifications
#define MASS_KG 10.0f
#define DIAMETER_M 0.30f  // 30cm converted to meters

// State check to ensure we are coasting
bool isCoasting(float acc_g, float velocity) {
    // 1. Motor must be off: Acceleration (net) will be negative 
    // because only drag and gravity are acting on it.
    // 2. Velocity must be positive (we haven't reached apogee yet).
    // Note: Since gravity is removed, a resting rocket is 0G. 
    // In flight, deceleration will show as a negative G value.
    return (acc_g < -0.1f && velocity > 20.0f);
}

float calculateCurrentCd(float acc_g, float velocity, float altitude) {
    if (!isCoasting(acc_g, velocity)) {
        return 0.0f; // Return 0 if not in valid coasting phase
    }

    // 1. Convert Acceleration from G to m/s2
    // We take the absolute value because we want the magnitude of deceleration
    float acc_ms2 = fabsf(acc_g * G_MS2);

    // 2. Calculate Cross-sectional Area
    float radius = DIAMETER_M / 2.0f;
    float area = PI * radius * radius;

    // 3. Barometric Air Density Model (Exponential Approximation)
    // rho = rho0 * e^(-altitude / scale_height)
    float rho = RHO_0 * expf(-altitude / H_SCALE);

    // 4. Solve for Cd
    // Force_drag = m * a
    // Force_drag = 0.5 * rho * v^2 * A * Cd
    // Cd = (2 * m * a) / (rho * v^2 * A)
    float denominator = 0.5f * rho * (velocity * velocity) * area;

    if (denominator == 0) return 0.0f;

    float cd = (MASS_KG * acc_ms2) / denominator;

    return cd;
}