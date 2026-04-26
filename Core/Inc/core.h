#ifndef __CORE_CONTROL_H__
#define __CORE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void Core_ControlInit(void);
void Core_ControlStep1ms(void);
void Core_ImuUpdate1ms(void);
void Core_BalanceStep1ms(void);
float Core_GetPitchRad(void);
float Core_GetPitchDeg(void);
void Core_SetPitchOffsetRad(float offset_rad);
float Core_GetPitchOffsetRad(void);
void Core_SetBalancePid(float kp, float ki, float kd);
void Core_GetBalancePid(float *kp, float *ki, float *kd);
void Core_ResetBalanceIntegrator(void);
bool Core_IsImuReady(void);
bool Core_IsImuSampleValid(void);
uint32_t Core_GetImuOkCount(void);
uint32_t Core_GetImuFailCount(void);
uint32_t Core_GetImuInitStage(void);
int32_t Core_GetImuInitLastRet(void);

#ifdef __cplusplus
}
#endif

#endif /* __CORE_CONTROL_H__ */