#ifndef __BRUSHLESS_MOTOR_H__
#define __BRUSHLESS_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* 初始化无刷电机 FOC 控制模块。 */
void BLDC_Init(void);

/* 通过串口打印 I2C 总线设备扫描结果。 */
void BLDC_PrintI2CDeviceInfo(void);

/* 设置目标机械转速，单位为 rps。 */
void BLDC_SetTargetSpeedRps(float target_rps);

/* 设置第二个无刷电机目标机械转速，单位为 rps。 */
void BLDC_SetTargetSpeed2Rps(float target_rps);

/* 获取电机1当前目标机械转速，单位为 rps。 */
float BLDC_GetTargetSpeedRps(void);

/* 获取电机2当前目标机械转速，单位为 rps。 */
float BLDC_GetTargetSpeed2Rps(void);

/* 1ms 周期运行一次的 FOC 控制主函数。 */
void BLDC_ControlStep_1ms(void);

/* 获取当前估算机械转速，单位为 rps。 */
float BLDC_GetMeasuredSpeedRps(void);

/* 获取第二个无刷电机当前估算机械转速，单位为 rps。 */
float BLDC_GetMeasuredSpeed2Rps(void);

/* 获取当前机械角度，单位为弧度。 */
float BLDC_GetMechanicalAngleRad(void);

/* 获取第二个无刷电机当前机械角度，单位为弧度。 */
float BLDC_GetMechanicalAngle2Rad(void);

/* 获取电机1角度传感器最近一次 I2C 原始值（14bit）。 */
uint16_t BLDC_GetLastRawAngleM1(void);

/* 获取电机1角度传感器最近一次读取是否有效。 */
bool BLDC_GetLastRawAngleM1Valid(void);

/* 直接读取一次电机1角度传感器 I2C 原始值（14bit）。 */
bool BLDC_ReadRawAngleM1Now(uint16_t *raw_angle);

/* 获取电机1两路相电流 ADC 差分（去零点后的 raw counts）。 */
int16_t BLDC_GetCurrentRawA(void);
int16_t BLDC_GetCurrentRawB(void);

/* 获取电机1估算 d/q 轴电流（基于 ADC 两相采样，归一化单位）。 */
float BLDC_GetCurrentD(void);
float BLDC_GetCurrentQ(void);

/* 获取电机1三相电压指令（归一化，范围约 -1~1）。 */
float BLDC_GetUa(void);
float BLDC_GetUb(void);
float BLDC_GetUc(void);

/* 获取电机1当前是否处于起转辅助/闭环混合阶段。 */
bool BLDC_IsM1AssistModeActive(void);

/* 获取电机1当前实际施加的 q 轴等效指令。 */
float BLDC_GetM1AppliedVq(void);

/* 获取电机1当前辅助过渡计数。 */
uint16_t BLDC_GetM1AssistReleaseCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BRUSHLESS_MOTOR_H__ */
