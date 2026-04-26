#ifndef __CORE_CONTROL_H__
#define __CORE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* __CORE_CONTROL_H__ */