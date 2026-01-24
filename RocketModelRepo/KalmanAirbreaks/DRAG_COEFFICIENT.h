#ifndef DRAG_COEFFICIENT_H
#define DRAG_COEFFICIENT_H

#include <stdbool.h>

// Function prototypes so other files can "see" them
bool isCoasting(float acc_g, float velocity);
float calculateCurrentCd(float acc_g, float velocity, float altitude);

#endif // DRAG_COEFFICIENT_H