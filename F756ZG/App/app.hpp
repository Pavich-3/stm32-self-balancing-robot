#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t appInit(void);
void appRun(float* accX, float* accY, float* accZ, float* gyrX, float* gyrY, float* gyrZ, float* angle);

#ifdef __cplusplus
}
#endif

