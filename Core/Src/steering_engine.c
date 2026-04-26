#include "steering_engine.h"

#include "tim.h"

#include <string.h>

#define SERVO_PULSE_MIN_US   (1000U)
#define SERVO_PULSE_MAX_US   (2000U)
#define SERVO_PERCENT_MAX    (100U)

static const uint32_t k_servo_channels[STEERING_SERVO_COUNT] = {
	TIM_CHANNEL_1,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4,
};

static uint8_t g_servo_percent[STEERING_SERVO_COUNT] = {50U, 50U, 50U, 50U};
static uint8_t g_steering_percent = 50U;
static bool g_steering_inited = false;

static uint32_t SteeringPercentToPulse(uint8_t percent)
{
	uint32_t pulse_span = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
	return SERVO_PULSE_MIN_US + ((uint32_t)percent * pulse_span) / SERVO_PERCENT_MAX;
}

void SteeringEngine_Init(void)
{
	uint32_t i;

	for (i = 0U; i < STEERING_SERVO_COUNT; i++)
	{
		(void)HAL_TIM_PWM_Start(&htim2, k_servo_channels[i]);
	}

	for (i = 0U; i < STEERING_SERVO_COUNT; i++)
	{
		(void)SteeringEngine_SetPercent((SteeringServoId)i, g_servo_percent[i]);
	}

	g_steering_inited = true;
}

bool SteeringEngine_SetPercent(SteeringServoId servo_id, uint8_t percent)
{
	uint32_t pulse;

	if ((servo_id >= STEERING_SERVO_COUNT) || (percent > SERVO_PERCENT_MAX))
	{
		return false;
	}

	pulse = SteeringPercentToPulse(percent);
	__HAL_TIM_SET_COMPARE(&htim2, k_servo_channels[servo_id], pulse);
	g_servo_percent[servo_id] = percent;

	return true;
}

bool SteeringEngine_SetSteeringPercent(uint8_t percent)
{
	uint8_t left_front;
	uint8_t left_back;
	uint8_t right_front;
	uint8_t right_back;

	if (percent > SERVO_PERCENT_MAX)
	{
		return false;
	}

	/* 100 -> LF/RB full high, LB/RF full low; 0 -> opposite. */
	left_front = percent;
	left_back = (uint8_t)(SERVO_PERCENT_MAX - percent);
	right_front = (uint8_t)(SERVO_PERCENT_MAX - percent);
	right_back = percent;

	(void)SteeringEngine_SetPercent(STEERING_SERVO_LF, left_front);
	(void)SteeringEngine_SetPercent(STEERING_SERVO_LB, left_back);
	(void)SteeringEngine_SetPercent(STEERING_SERVO_RF, right_front);
	(void)SteeringEngine_SetPercent(STEERING_SERVO_RB, right_back);

	g_steering_percent = percent;
	return true;
}

uint8_t SteeringEngine_GetPercent(SteeringServoId servo_id)
{
	if (servo_id >= STEERING_SERVO_COUNT)
	{
		return 0U;
	}

	return g_servo_percent[servo_id];
}
