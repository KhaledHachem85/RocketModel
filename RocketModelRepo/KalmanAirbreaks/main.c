#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "drag_data.h"
#include "DRAG_COEFFICIENT.h"
#include "altitude_kalman_filter.h"
#include "APOGEE_PREDICTION.h"

#define DATA_POINTS 616
#define MAX_LINE 1024
#define G 9.80665
#define MACH_1 343.0 

typedef struct {
    double time;
    double altitude;
    double vertical_accel_g;
    double lateral_accel;
    double velocity_z;
} RealData;

typedef struct {
    double time;
    double baro_alt;
    double accel_g;
    double velocity_z;
} SensorData;

typedef struct {
    double kf_altitude;
    double kf_velocity;
    double predicted_apogee;
    double calculated_cd;
} KFResults;

// Helper to get noise
double get_noise(double magnitude) {
    return ((double)rand() / (double)RAND_MAX * 2.0 - 1.0) * magnitude;
}

// Simulate messy sensors
double simulate_accelerometer(double actual_g) {
    return actual_g + get_noise(0.08) * actual_g;
}

double simulate_barometer(double actual_alt, double velocity) {
    double mach = velocity / MACH_1;
    double noise_mag = 0.3;
    if (mach > 0.7) {
        noise_mag += pow(10, (mach - 0.7) * 4.0);
    }
    return actual_alt + get_noise(noise_mag);
}

// The "Inverse" physics function to find Drag Coefficient
float calculate_current_cd(float altitude, float velocity, float acceleration_m_s2, float mass, float area) {
    float rho = RHO_0 * expf(-altitude / H_SCALE);

    // Avoid division by zero at apogee or very low speeds
    if (fabsf(velocity) < 0.5f) return 0.0f;

    // F_drag = total_force - gravity_force -> m*a - (-m*g)
    float drag_force = -mass * (acceleration_m_s2 + (float)G);

    // Cd = F_drag / (0.5 * rho * v^2 * A)
    float cd = drag_force / (0.5f * rho * velocity * velocity * area);

    return (cd < 0.0f) ? 0.0f : cd;
}
double get_interpolated_cd(double target_mach) {
    // 1. Handle Out-of-Bounds (Extrapolation)
    if (target_mach <= MACH_TABLE[0]) return CD_TABLE[0];
    if (target_mach >= MACH_TABLE[DATA_POINTS - 1]) return CD_TABLE[DATA_POINTS - 1];

    // 2. Binary Search to find the indices i and i+1 such that 
    // MACH_TABLE[i] <= target_mach < MACH_TABLE[i+1]
    int low = 0;
    int high = DATA_POINTS - 2;
    int i = 0;

    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (MACH_TABLE[mid] <= target_mach) {
            i = mid;
            low = mid + 1;
        }
        else {
            high = mid - 1;
        }
    }

    // 3. Linear Interpolation Formula: 
    // y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    double x0 = MACH_TABLE[i];
    double x1 = MACH_TABLE[i + 1];
    double y0 = CD_TABLE[i];
    double y1 = CD_TABLE[i + 1];

    // Calculate slope (m)
    double slope = (y1 - y0) / (x1 - x0);

    // Calculate final Cd
    return y0 + slope * (target_mach - x0);
}
// CSV Saving Functions
void save_telemetry(const char* filename, RealData* real, SensorData* sensor, KFResults* kf_res, int count) {
    FILE* fp = fopen(filename, "w");
    if (!fp) return;
    fprintf(fp, "Time,Real_Alt,Baro_Alt,kf_altitude,Real_G,Sens_G,Real_Vel,kf_velocity,Pred_Apogee\n");
    for (int i = 0; i < count; i++) {
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.2f\n",
            real[i].time, real[i].altitude, sensor[i].baro_alt, kf_res[i].kf_altitude,
            real[i].vertical_accel_g, sensor[i].accel_g, real[i].velocity_z, kf_res[i].kf_velocity,
            kf_res[i].predicted_apogee);
    }
    fclose(fp);
    printf("Telemetry saved to %s\n", filename);
}

void save_drag_data(const char* filename, RealData* real, KFResults* kf_res, int count) {
    FILE* fp = fopen(filename, "w");
    if (!fp) return;
    fprintf(fp, "Time,Altitude,Velocity,Acceleration_m_s2,Calculated_Cd\n");
    for (int i = 0; i < count; i++) {
        fprintf(fp, "%.3f,%.3f,%.4f,%.4f,%.6f\n",
            real[i].time, real[i].altitude, real[i].velocity_z,
            real[i].vertical_accel_g * G, kf_res[i].calculated_cd);
    }
    fclose(fp);
    printf("Drag analysis saved to %s\n", filename);
}

// Data Loading
RealData* read_telemetry(const char* filename, int* count) {
    FILE* fp = fopen(filename, "r");
    if (!fp) { perror("File Error"); return NULL; }
    char line[MAX_LINE];
    int capacity = 10, row = 0;
    RealData* list = malloc(capacity * sizeof(RealData));
    fgets(line, MAX_LINE, fp); // Skip header
    while (fgets(line, MAX_LINE, fp)) {
        if (row >= capacity) {
            capacity *= 2;
            list = realloc(list, capacity * sizeof(RealData));
        }
        if (sscanf(line, "%lf,%lf,%lf,%lf", &list[row].time, &list[row].altitude,
            &list[row].vertical_accel_g, &list[row].lateral_accel) == 4) {
            row++;
        }
    }
    fclose(fp);
    *count = row;
    return list;
}

int main() {
    srand((unsigned int)time(NULL));
    int num_records = 0;
    RealData* real = read_telemetry("telemetry.csv", &num_records);
    if (!real || num_records == 0) return 1;

    SensorData* sensors = malloc(num_records * sizeof(SensorData));
    KFResults* kf_res = malloc(num_records * sizeof(KFResults));

    const float rocket_diameter = 0.10f;
    const float rocket_area = 3.14159f * (rocket_diameter / 2.0f) * (rocket_diameter / 2.0f);
    const float rocket_mass = 30.0f;
    const float CD_NOMINAL = 0.48f;

    AltitudeKF kf;
    altitudeKFInit(&kf, (float)real[0].altitude, 0.01f, 0.5f);

    float current_prediction = 0.0f;
    float max_real_alt = 0.0f;

    printf("--- Flight Simulation Started ---\n");

    for (int i = 0; i < num_records; i++) {
        if (real[i].altitude > max_real_alt) max_real_alt = (float)real[i].altitude;

        if (i == 0) {
            real[i].velocity_z = 0.0;
            sensors[i].time = real[i].time;
            sensors[i].accel_g = simulate_accelerometer(real[i].vertical_accel_g);
            sensors[i].baro_alt = simulate_barometer(real[i].altitude, 0.0);
            current_prediction = (float)real[i].altitude;
            kf_res[i].calculated_cd = 0.0;
        }
        else {
            double dt = real[i].time - real[i - 1].time;
            sensors[i].time = real[i].time;
            sensors[i].accel_g = simulate_accelerometer(real[i].vertical_accel_g);
            sensors[i].baro_alt = simulate_barometer(real[i].altitude, real[i - 1].velocity_z);

            // Update ground truth velocity for comparison
            double avg_a = (real[i].vertical_accel_g + real[i - 1].vertical_accel_g) / 2.0 * G;
            real[i].velocity_z = real[i - 1].velocity_z + (avg_a * dt);

            // 1. Kalman Filter
            altitudeKFPredict(&kf, (float)sensors[i].accel_g * 9.81f, (float)dt);
            altitudeKFUpdate(&kf, (float)sensors[i].baro_alt);

            // 2. Apogee Prediction & Cd Calculation
            if (isCoasting((float)sensors[i].accel_g, (float)kf.v)) {
                double current_mach = (double)kf.v / MACH_1;
                float dynamic_cd = (float)get_interpolated_cd(current_mach);
                current_prediction = predict_apogee_sim(kf.z, kf.v, rocket_mass, rocket_area, dynamic_cd);

                // Calculate Cd based on REAL data for analysis
                kf_res[i].calculated_cd = (double)calculate_current_cd(
                    (float)real[i].altitude, (float)real[i].velocity_z,
                    (float)real[i].vertical_accel_g * (float)G, rocket_mass, rocket_area);

                if (i % 50 == 0) {
                    printf("T: %.1fs | Alt: %.0fm | Pred Apogee: %.1fm | Cd: %.3f\n",
                        real[i].time, kf.z, current_prediction, kf_res[i].calculated_cd);
                }
            }
            else {
                current_prediction = kf.z;
                kf_res[i].calculated_cd = 0.0; // Cd isn't valid during motor burn
            }
        }

        kf_res[i].kf_altitude = (double)kf.z;
        kf_res[i].kf_velocity = (double)kf.v;
        kf_res[i].predicted_apogee = (double)current_prediction;
    }

    printf("\n--- Final Results ---\n");
    printf("Real Apogee: %.2f m | Prediction Error: %.2f m\n",
        max_real_alt, fabsf(max_real_alt - current_prediction));

    save_telemetry("simulation_results.csv", real, sensors, kf_res, num_records);
    save_drag_data("drag_analysis.csv", real, kf_res, num_records);

    free(real); free(sensors); free(kf_res);
    return 0;
}