/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    steering_engine.h
	* @brief   4-channel servo control on TIM2 PWM.
	******************************************************************************
	*/
/* USER CODE END Header */

#ifndef __STEERING_ENGINE_H__
#define __STEERING_ENGINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	STEERING_SERVO_LF = 0,
	STEERING_SERVO_RF,
	STEERING_SERVO_LB,
	STEERING_SERVO_RB,
	STEERING_SERVO_COUNT
} SteeringServoId;

void SteeringEngine_Init(void);
bool SteeringEngine_SetPercent(SteeringServoId servo_id, uint8_t percent);
bool SteeringEngine_SetSteeringPercent(uint8_t percent);
uint8_t SteeringEngine_GetPercent(SteeringServoId servo_id);

#ifdef __cplusplus
}
#endif

#endif /* __STEERING_ENGINE_H__ */
