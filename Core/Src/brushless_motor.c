#include "brushless_motor.h"

#include "i2c.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define BLDC_PI                     (3.14159265358979323846f)
#define BLDC_TWO_PI                 (6.28318530717958647692f)
#define BLDC_DT_S                   (0.001f)
#define BLDC_SENSOR_RAW_MAX         (16384.0f)
#define BLDC_POLE_PAIRS             (7.0f)
#define BLDC_SPEED_PID_KP           (0.20f)
#define BLDC_SPEED_PID_KI           (0.05f)
#define BLDC_SPEED_PID_KD           (0.0f)
#define BLDC_SPEED_LPF_ALPHA        (0.06f)
#define BLDC_SPEED_OUTLIER_RPS      (8.0f)
#define BLDC_SPEED_GLITCH_OPPO_RPS  (1.5f)
#define BLDC_VQ_SLEW_PER_MS         (0.006f)
#define BLDC_CURR_ID_KP             (0.25f)
#define BLDC_CURR_ID_KI             (20.0f)
#define BLDC_CURR_IQ_KP             (0.25f)
#define BLDC_CURR_IQ_KI             (20.0f)
#define BLDC_MAX_IQ_REF             (0.35f)
#define BLDC_M1_ELEC_SIGN           (1.0f)
#define BLDC_M2_ELEC_SIGN           (1.0f)
#define BLDC_M1_ELEC_OFFSET_RAD     (0.0f)
#define BLDC_M2_ELEC_OFFSET_RAD     (0.0f)
#define BLDC_ALIGN_VD               (0.10f)
#define BLDC_ALIGN_SETTLE_MS        (300U)
#define BLDC_ALIGN_SAMPLE_COUNT     (32U)
#define BLDC_ALIGN_SAMPLE_DELAY_MS  (2U)
#define BLDC_STARTUP_ASSIST_VQ_MIN  (0.16f)
#define BLDC_STARTUP_ASSIST_VQ_MAX  (0.45f)
#define BLDC_STARTUP_ASSIST_MIN_RPS (1.2f)
#define BLDC_STARTUP_ASSIST_MAX_RPS (12.0f)
#define BLDC_STARTUP_ASSIST_ENTRY_RPS (0.25f)
#define BLDC_STARTUP_ASSIST_EXIT_RPS  (1.6f)
#define BLDC_STARTUP_ASSIST_EXIT_HOLD_MS (80U)
#define BLDC_STARTUP_ASSIST_TIMEOUT_MS   (600U)
#define BLDC_STARTUP_ASSIST_BLEND_MS     (200U)
#define BLDC_STARTUP_OPEN_LOOP_MS   (1200U)
#define BLDC_STARTUP_OPEN_LOOP_VQ   (0.05f)
#define BLDC_FORCE_OPEN_LOOP_STARTUP (0)
#define BLDC_FORCE_OPEN_LOOP_ALWAYS  (0)
#define BLDC_ENABLE_MOTOR2           (1)
#define BLDC_STARTUP_MIN_RPS        (1.0f)
#define BLDC_STARTUP_MAX_RPS        (8.0f)
#define BLDC_PWM_DUTY_MIN           (0.02f)
#define BLDC_PWM_DUTY_MAX           (0.98f)
#define BLDC_START_ASSIST_MIN_TARGET_RPS (0.8f)
#define BLDC_START_ASSIST_STALL_RPS  (0.6f)
#define BLDC_START_ASSIST_MAX_MS     (0U)
#define BLDC_BRINGUP_FORCE_OPEN_LOOP (0)

static float g_target_speed_rps = 0.0f;
static float g_target_speed2_rps = 0.0f;
static float g_mech_angle_rad = 0.0f;
static float g_mech_angle2_rad = 0.0f;
static float g_measured_speed_rps = 0.0f;
static float g_measured_speed2_rps = 0.0f;
static float g_prev_mech_angle_rad = 0.0f;
static float g_prev_mech_angle2_rad = 0.0f;
static float g_speed_integral = 0.0f;
static float g_speed2_integral = 0.0f;
static float g_speed_prev_error = 0.0f;
static float g_speed2_prev_error = 0.0f;
static float g_m1_elec_offset_runtime = BLDC_M1_ELEC_OFFSET_RAD;
static float g_m2_elec_offset_runtime = BLDC_M2_ELEC_OFFSET_RAD;
static bool g_initialized = false;
static uint16_t g_m1_raw_angle_last = 0U;
static bool g_m1_raw_angle_valid = false;
static float g_m1_u_a = 0.0f;
static float g_m1_u_b = 0.0f;
static float g_m1_u_c = 0.0f;
static float g_startup_angle2_rad = 0.0f;
static uint32_t g_m1_stall_assist_count = 0U;
static uint32_t g_m2_stall_assist_count = 0U;
static float g_m1_open_loop_elec_angle = 0.0f;
static bool g_m1_assist_mode = false;
static uint16_t g_m1_assist_release_count = 0U;
static float g_m1_vq_applied = 0.0f;
static bool g_m1_reverse_hold = false;

typedef struct
{
	uint16_t addr;
	uint8_t reg;
	bool valid;
	bool detected;
} BLDC_I2CSensorConfig;

static const uint16_t g_mt6701_addr_candidates[] =
{
	(0x06U << 1),
	(0x36U << 1),
	(0x46U << 1),
	(0x70U << 1)
};

static const uint8_t g_mt6701_reg_candidates[] =
{
	0x03U,
	0x0CU,
	0x0EU
};

static BLDC_I2CSensorConfig g_m1_sensor_cfg = {0U, 0U, false, false};
static BLDC_I2CSensorConfig g_m2_sensor_cfg = {0U, 0U, false, false};

static bool BLDC_IsRawAngleLikelyValid(uint16_t raw_angle)
{
	/* 0x0000/0x3FFF 长时间不变通常表示寄存器不对或磁场无效。 */
	return (raw_angle != 0x0000U) && (raw_angle != 0x3FFFU);
}

/* 将输入值限制在指定范围内，避免控制输出越界。 */
static float BLDC_Clamp(float value, float min_val, float max_val)
{
	if (value < min_val)
	{
		return min_val;
	}
	if (value > max_val)
	{
		return max_val;
	}
	return value;
}

/* 将角度归一化到 0~2pi，避免角度累积过大导致三角函数精度下降。 */
static float BLDC_NormalizeAngle(float angle)
{
	while (angle >= BLDC_TWO_PI)
	{
		angle -= BLDC_TWO_PI;
	}
	while (angle < 0.0f)
	{
		angle += BLDC_TWO_PI;
	}
	return angle;
}

/* 停止三相 PWM 输出。 */
static void BLDC_StopPwmOutput(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
}

static void BLDC_SetSpwmFromAlphaBeta(float v_alpha, float v_beta);

#if BLDC_FORCE_OPEN_LOOP_STARTUP
static void BLDC_UpdateElectricalOffsetFromStartup(float startup_mech_angle_rad,
																				 float measured_mech_angle_rad,
																				 float elec_sign,
																				 float *runtime_offset_rad)
{
	float elec_open_loop = BLDC_NormalizeAngle(startup_mech_angle_rad * BLDC_POLE_PAIRS + *runtime_offset_rad);
	float elec_from_sensor = BLDC_NormalizeAngle(elec_sign * measured_mech_angle_rad * BLDC_POLE_PAIRS);
	float elec_error = elec_open_loop - elec_from_sensor;

	if (elec_error > BLDC_PI)
	{
		elec_error -= BLDC_TWO_PI;
	}
	else if (elec_error < -BLDC_PI)
	{
		elec_error += BLDC_TWO_PI;
	}

	/* 低速阶段用小步长修正偏置，避免闭环接管时相位突变。 */
	*runtime_offset_rad = BLDC_NormalizeAngle(*runtime_offset_rad + 0.05f * elec_error);
}
#endif

/* 停止第二个无刷电机三相 PWM 输出。 */
static void BLDC_StopPwmOutput2(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
}

/* 通用 PI 控制器：按照误差更新积分并计算输出。 */
static float BLDC_PIController(float error, float kp, float ki, float dt, float *integral)
{
    float output;

	/* 这里保留积分项的累积，让速度环在稳态时能消除静差。 */
	*integral += error * ki * dt;

	/* P 项和 I 项直接相加，输出交给上层做连续性处理。 */
	output = kp * error + *integral;
	return output;
}

static bool BLDC_IsPositiveNegativeFlip(float previous_target_rps, float next_target_rps)
{
	const float deadband = 0.01f;

	if ((fabsf(previous_target_rps) < deadband) || (fabsf(next_target_rps) < deadband))
	{
		return false;
	}

	return ((previous_target_rps > 0.0f) && (next_target_rps < 0.0f)) ||
	       ((previous_target_rps < 0.0f) && (next_target_rps > 0.0f));
}

bool BLDC_IsM1AssistModeActive(void)
{
	return g_m1_assist_mode;
}

float BLDC_GetM1AppliedVq(void)
{
	return g_m1_vq_applied;
}

uint16_t BLDC_GetM1AssistReleaseCount(void)
{
	return g_m1_assist_release_count;
}

/* 尝试用指定地址/寄存器读取 MT6701 角度原始值。 */
static bool BLDC_ReadRawAngleFromI2C(I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t reg, uint16_t *raw_angle)
{
	uint8_t raw_data[2] = {0};

	if (HAL_I2C_Mem_Read(hi2c,
										 addr,
										 reg,
										 I2C_MEMADD_SIZE_8BIT,
										 raw_data,
										 sizeof(raw_data),
										 5U) != HAL_OK)
	{
		return false;
	}

	*raw_angle = (uint16_t)(((uint16_t)raw_data[0] << 8) | raw_data[1]);
	*raw_angle &= 0x3FFFU;
	return true;
}

/* 探测 MT6701 可能的 I2C 地址与角度寄存器组合，并缓存可用配置。 */
static bool BLDC_DetectSensorConfig(I2C_HandleTypeDef *hi2c, BLDC_I2CSensorConfig *cfg)
{
	uint32_t i;
	uint32_t j;
	uint16_t raw_angle;
	uint32_t k;

	if (cfg->detected)
	{
		return cfg->valid;
	}

	for (i = 0U; i < (sizeof(g_mt6701_addr_candidates) / sizeof(g_mt6701_addr_candidates[0])); i++)
	{
		uint16_t addr = g_mt6701_addr_candidates[i];

		if (HAL_I2C_IsDeviceReady(hi2c, addr, 2U, 5U) != HAL_OK)
		{
			continue;
		}

		for (j = 0U; j < (sizeof(g_mt6701_reg_candidates) / sizeof(g_mt6701_reg_candidates[0])); j++)
		{
			uint8_t reg = g_mt6701_reg_candidates[j];
			bool candidate_valid = false;

			if (BLDC_ReadRawAngleFromI2C(hi2c, addr, reg, &raw_angle))
			{
				if (BLDC_IsRawAngleLikelyValid(raw_angle))
				{
					candidate_valid = true;
				}

				/* 多读几次，避免把恒 0 的寄存器当成角度寄存器。 */
				for (k = 0U; (k < 4U) && (!candidate_valid); k++)
				{
					if (BLDC_ReadRawAngleFromI2C(hi2c, addr, reg, &raw_angle) &&
							BLDC_IsRawAngleLikelyValid(raw_angle))
					{
						candidate_valid = true;
					}
				}

				if (candidate_valid)
				{
					cfg->addr = addr;
					cfg->reg = reg;
					cfg->valid = true;
					cfg->detected = true;
					return true;
				}
			}
		}
	}

	cfg->valid = false;
	cfg->detected = true;
	return false;
}

static void BLDC_PrintI2cBusDevices(const char *bus_name, I2C_HandleTypeDef *hi2c)
{
	uint32_t addr;
	uint32_t found_count = 0U;
	bool found = false;

	printf("%s devices:", bus_name);
	for (addr = 0x08U; addr <= 0x77U; addr++)
	{
		if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2U, 5U) == HAL_OK)
		{
			printf(" 0x%02lX", addr);
			found_count++;
			found = true;
		}
	}

	if (!found)
	{
		printf(" none");
	}
	printf("  (count=%lu)\r\n", found_count);
}

void BLDC_PrintI2CDeviceInfo(void)
{
	BLDC_I2CSensorConfig sensor_cfg;

	printf("\r\n[I2C] bus scan start\r\n");
	BLDC_PrintI2cBusDevices("I2C1", &hi2c1);
	BLDC_PrintI2cBusDevices("I2C2", &hi2c2);
	BLDC_PrintI2cBusDevices("I2C3", &hi2c3);

	sensor_cfg = g_m1_sensor_cfg;
	if (BLDC_DetectSensorConfig(&hi2c1, &sensor_cfg))
	{
		printf("[I2C] M1 angle sensor: addr=0x%02X reg=0x%02X\r\n", sensor_cfg.addr >> 1, sensor_cfg.reg);
	}
	else
	{
		printf("[I2C] M1 angle sensor: not found\r\n");
	}

	sensor_cfg = g_m2_sensor_cfg;
	if (BLDC_DetectSensorConfig(&hi2c2, &sensor_cfg))
	{
		printf("[I2C] M2 angle sensor: addr=0x%02X reg=0x%02X\r\n", sensor_cfg.addr >> 1, sensor_cfg.reg);
	}
	else
	{
		printf("[I2C] M2 angle sensor: not found\r\n");
	}

	printf("[I2C] bus scan done\r\n");
}

/* 通过 I2C1 读取角度传感器原始角度并换算为机械角度弧度。 */
static bool BLDC_ReadMechanicalAngle(float *angle_rad)
{
	uint16_t raw_angle;

	if (!BLDC_DetectSensorConfig(&hi2c1, &g_m1_sensor_cfg))
	{
		g_m1_raw_angle_valid = false;
		return false;
	}

	if (!BLDC_ReadRawAngleFromI2C(&hi2c1, g_m1_sensor_cfg.addr, g_m1_sensor_cfg.reg, &raw_angle))
	{
		g_m1_sensor_cfg.detected = false;
		g_m1_sensor_cfg.valid = false;
		g_m1_raw_angle_valid = false;
		return false;
	}

	g_m1_raw_angle_last = raw_angle;
	g_m1_raw_angle_valid = true;
	*angle_rad = ((float)raw_angle / BLDC_SENSOR_RAW_MAX) * BLDC_TWO_PI; // 将原始值换算成机械角度弧度
	return true;
}

/* 通过 I2C2 读取第二个角度传感器原始角度并换算为机械角度弧度。 */
#if BLDC_ENABLE_MOTOR2
static bool BLDC_ReadMechanicalAngle2(float *angle_rad)
{
	uint16_t raw_angle;

	if (!BLDC_DetectSensorConfig(&hi2c2, &g_m2_sensor_cfg))
	{
		return false;
	}

	if (!BLDC_ReadRawAngleFromI2C(&hi2c2, g_m2_sensor_cfg.addr, g_m2_sensor_cfg.reg, &raw_angle))
	{
		g_m2_sensor_cfg.detected = false;
		g_m2_sensor_cfg.valid = false;
		return false;
	}

	*angle_rad = ((float)raw_angle / BLDC_SENSOR_RAW_MAX) * BLDC_TWO_PI;
	return true;
}
#endif

/* 反 Park 变换：把同步坐标系电压指令 vd/vq 转回 alpha-beta 坐标。 */
static void BLDC_InvParkTransform(float v_d, float v_q, float elec_angle, float *v_alpha, float *v_beta)
{
	float sin_theta = sinf(elec_angle);
	float cos_theta = cosf(elec_angle);

	*v_alpha = v_d * cos_theta - v_q * sin_theta;
	*v_beta = v_d * sin_theta + v_q * cos_theta;
}

/* SPWM 调制：根据 alpha-beta 电压生成三相占空比并写入 TIM3_CH1/2/3。 */
static void BLDC_SetSpwmFromAlphaBeta(float v_alpha, float v_beta)
{
	float va;
	float vb;
	float vc;
	float duty_a;
	float duty_b;
	float duty_c;
	uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

	va = v_alpha;
	vb = -0.5f * v_alpha + 0.8660254f * v_beta; // 反 Clarke 变换得到三相电压指令，0.8660254 是 sqrt(3)/2 的近似值
	vc = -0.5f * v_alpha - 0.8660254f * v_beta;

	g_m1_u_a = va;
	g_m1_u_b = vb;
	g_m1_u_c = vc;

	duty_a = 0.5f + 0.5f * va;
	duty_b = 0.5f + 0.5f * vb;
	duty_c = 0.5f + 0.5f * vc;

	duty_a = BLDC_Clamp(duty_a, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);
	duty_b = BLDC_Clamp(duty_b, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);
	duty_c = BLDC_Clamp(duty_c, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)period));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)period));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)period));
}

static void BLDC_SetSpwmFromAlphaBeta2(float v_alpha, float v_beta)
{
	float va;
	float vb;
	float vc;
	float duty_a;
	float duty_b;
	float duty_c;
	uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim4);

	va = v_alpha;
	vb = -0.5f * v_alpha + 0.8660254f * v_beta;
	vc = -0.5f * v_alpha - 0.8660254f * v_beta;

	duty_a = 0.5f + 0.5f * va;
	duty_b = 0.5f + 0.5f * vb;
	duty_c = 0.5f + 0.5f * vc;

	duty_a = BLDC_Clamp(duty_a, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);
	duty_b = BLDC_Clamp(duty_b, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);
	duty_c = BLDC_Clamp(duty_c, BLDC_PWM_DUTY_MIN, BLDC_PWM_DUTY_MAX);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)period));
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)period));
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)period));
}

/* 上电锁相后读取机械角，自动计算电角度零位偏置。 */
static bool BLDC_CalibrateM1ElectricalZero(void)
{
	float v_alpha;
	float v_beta;
	float angle_rad;
	float sin_sum = 0.0f;
	float cos_sum = 0.0f;
	float mean_mech_angle;
	float offset;
	uint32_t i;

	/* 先给定 d 轴定向电压，让转子稳定对齐到已知电角位置。 */
	BLDC_InvParkTransform(BLDC_ALIGN_VD, 0.0f, 0.0f, &v_alpha, &v_beta);
	BLDC_SetSpwmFromAlphaBeta(v_alpha, v_beta);
	HAL_Delay(BLDC_ALIGN_SETTLE_MS);

	for (i = 0U; i < BLDC_ALIGN_SAMPLE_COUNT; i++)
	{
		if (!BLDC_ReadMechanicalAngle(&angle_rad))
		{
			BLDC_StopPwmOutput();
			g_m1_u_a = 0.0f;
			g_m1_u_b = 0.0f;
			g_m1_u_c = 0.0f;
			return false;
		}

		sin_sum += sinf(angle_rad);
		cos_sum += cosf(angle_rad);
		HAL_Delay(BLDC_ALIGN_SAMPLE_DELAY_MS);
	}

	/* 对采样角做圆均值，避免跨 0 点时直接求平均产生误差。 */
	mean_mech_angle = atan2f(sin_sum, cos_sum);
	if (mean_mech_angle < 0.0f)
	{
		mean_mech_angle += BLDC_TWO_PI;
	}

	/* 零位偏置按当前机械零位和极对数换算到电角域。 */
	offset = BLDC_NormalizeAngle(-BLDC_M1_ELEC_SIGN * mean_mech_angle * BLDC_POLE_PAIRS);
	g_m1_elec_offset_runtime = offset;

	BLDC_StopPwmOutput();
	g_m1_u_a = 0.0f;
	g_m1_u_b = 0.0f;
	g_m1_u_c = 0.0f;

	printf("[BLDC] M1 elec zero calibrated, offset_mrad=%ld\r\n",
	       (long)(offset * 1000.0f));
	return true;
}

#if BLDC_ENABLE_MOTOR2
static bool BLDC_CalibrateM2ElectricalZero(void)
{
	float v_alpha;
	float v_beta;
	float angle_rad;
	float sin_sum = 0.0f;
	float cos_sum = 0.0f;
	float mean_mech_angle;
	float offset;
	uint32_t i;

	BLDC_InvParkTransform(BLDC_ALIGN_VD, 0.0f, 0.0f, &v_alpha, &v_beta);
	BLDC_SetSpwmFromAlphaBeta2(v_alpha, v_beta);
	HAL_Delay(BLDC_ALIGN_SETTLE_MS);

	for (i = 0U; i < BLDC_ALIGN_SAMPLE_COUNT; i++)
	{
		if (!BLDC_ReadMechanicalAngle2(&angle_rad))
		{
			BLDC_StopPwmOutput2();
			return false;
		}

		sin_sum += sinf(angle_rad);
		cos_sum += cosf(angle_rad);
		HAL_Delay(BLDC_ALIGN_SAMPLE_DELAY_MS);
	}

	mean_mech_angle = atan2f(sin_sum, cos_sum);
	if (mean_mech_angle < 0.0f)
	{
		mean_mech_angle += BLDC_TWO_PI;
	}

	offset = BLDC_NormalizeAngle(-BLDC_M2_ELEC_SIGN * mean_mech_angle * BLDC_POLE_PAIRS);
	g_m2_elec_offset_runtime = offset;

	BLDC_StopPwmOutput2();

	printf("[BLDC] M2 elec zero calibrated, offset_mrad=%ld\r\n",
	       (long)(offset * 1000.0f));
	return true;
}
#endif

/* 根据机械角度差分估算机械转速，并处理角度跨零点跳变。 */
static float BLDC_EstimateSpeedRps(float current_angle, float prev_angle)
{
	float dtheta = current_angle - prev_angle;

	/* 角度差超过半圈时，说明跨越了 0 点，需要折返到最短弧。 */
	if (dtheta > BLDC_PI)
	{
		dtheta -= BLDC_TWO_PI;
	}
	else if (dtheta < -BLDC_PI)
	{
		dtheta += BLDC_TWO_PI;
	} // 处理角度跨零点跳变，例如从 359 度跳到 0 度，dtheta 应该是 +1 度而不是 -359 度

	return dtheta / BLDC_TWO_PI / BLDC_DT_S; // 转换w为机械转速 rps
}

/* 初始化 PWM 输出、角度传感器相关状态与内部状态，进入可控运行状态。 */
void BLDC_Init(void)
{
	if (g_initialized)
	{
		return;
	}

	__HAL_TIM_SET_COUNTER(&htim3, 0U);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	__HAL_TIM_SET_COUNTER(&htim4, 0U);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	BLDC_StopPwmOutput();
	BLDC_StopPwmOutput2();
	g_m1_u_a = 0.0f;
	g_m1_u_b = 0.0f;
	g_m1_u_c = 0.0f;
	g_target_speed_rps = 0.0f; // 初始化目标速度为零，避免开机时控制器输出过大导致电机抖动
	g_target_speed2_rps = 0.0f; // 初始化第二个无刷电机目标速度为零，避免开机时控制器输出过大导致电机抖动
	g_measured_speed_rps = 0.0f; // 初始化估算速度为零，避免开机时速度环输出过大导致电机抖动
	g_measured_speed2_rps = 0.0f; // 初始化第二个无刷电机估算速度为零，避免开机时速度环输出过大导致电机抖动
	g_prev_mech_angle_rad = 0.0f; // 初始化前一个机械角度为零，避免开机时角度跳变导致速度估算异常
	g_prev_mech_angle2_rad = 0.0f; // 初始化第二个无刷电机前一个机械角度为零，避免开机时角度跳变导致速度估算异常
	g_mech_angle_rad = 0.0f; // 初始化机械角度为零，避免开机时角度跳变导致控制器输出异常
	g_mech_angle2_rad = 0.0f; // 初始化第二个无刷电机机械角度为零，避免开机时角度跳变导致控制器输出异常
	g_speed_integral = 0.0f; // 清除速度环积分，避免开机时控制器输出过大导致电机抖动
	g_speed2_integral = 0.0f; // 清除第二个无刷电机速度环积分，避免开机时控制器输出过大导致电机抖动
	g_speed_prev_error = 0.0f;
	g_speed2_prev_error = 0.0f;
	g_m1_elec_offset_runtime = BLDC_M1_ELEC_OFFSET_RAD;
	g_m2_elec_offset_runtime = BLDC_M2_ELEC_OFFSET_RAD;
	g_startup_angle2_rad = 0.0f;
	g_m1_stall_assist_count = 0U;
	g_m2_stall_assist_count = 0U;
	g_m1_open_loop_elec_angle = 0.0f;
	g_m1_assist_mode = false;
	g_m1_assist_release_count = 0U;
	g_m1_vq_applied = 0.0f;
	g_m1_sensor_cfg.valid = false;
	g_m1_sensor_cfg.detected = false;
	g_m2_sensor_cfg.valid = false;
	g_m2_sensor_cfg.detected = false;

	if (!BLDC_CalibrateM1ElectricalZero())
	{
		g_m1_elec_offset_runtime = BLDC_M1_ELEC_OFFSET_RAD;
		printf("[BLDC] M1 elec zero calibration failed, use default offset\r\n");
	}

#if BLDC_ENABLE_MOTOR2
	if (!BLDC_CalibrateM2ElectricalZero())
	{
		g_m2_elec_offset_runtime = BLDC_M2_ELEC_OFFSET_RAD;
		printf("[BLDC] M2 elec zero calibration failed, use default offset\r\n");
	}
#endif

	g_initialized = true; // 设置初始化完成标志，允许控制环开始工作
}

/* 更新目标机械转速，供 1ms 控制环读取。 */
void BLDC_SetTargetSpeedRps(float target_rps)
{
	if (BLDC_IsPositiveNegativeFlip(g_target_speed_rps, target_rps))
	{
		g_m1_reverse_hold = true;
		g_speed_integral = 0.0f;
		g_speed_prev_error = 0.0f;
		g_m1_vq_applied = 0.0f;
		BLDC_StopPwmOutput();
	}

	g_target_speed_rps = target_rps;
}

/* 更新第二个无刷电机目标机械转速，供 1ms 控制环读取。 */
void BLDC_SetTargetSpeed2Rps(float target_rps)
{
	g_target_speed2_rps = target_rps;
}

/* 执行一次速度环与角度同步控制，并输出到三相 PWM。 */
void BLDC_ControlStep_1ms(void)
{
	float electrical_angle;
	float speed_error;
	float v_d;
	float v_q;
	float v_alpha;
	float v_beta;
	float speed_diff;
	float target_abs;
	float meas_abs;
	float raw_speed;
    float vq_delta;
	float closed_vq;
	float open_vq;
	float blend;
	float dir;
	float assist_rps;

	if (!g_initialized)
	{
		return;
	}
	if (g_m1_reverse_hold)
	{
		BLDC_StopPwmOutput();
		g_m1_u_a = 0.0f;
		g_m1_u_b = 0.0f;
		g_m1_u_c = 0.0f;
		g_m1_vq_applied = 0.0f;
		g_m1_assist_mode = false;
		g_m1_assist_release_count = 0U;
		g_m1_stall_assist_count = 0U;
		g_m1_reverse_hold = false;
		return;
	}
	if (!BLDC_ReadMechanicalAngle(&g_mech_angle_rad))
	{
		BLDC_StopPwmOutput();
		g_speed_integral = 0.0f;
		g_speed_prev_error = 0.0f;
		g_m1_vq_applied = 0.0f;
		g_m1_u_a = 0.0f;
		g_m1_u_b = 0.0f;
		g_m1_u_c = 0.0f;
		return;
	}

	/* 角速度估算 + 离群抑制 + 低通，减少量化抖动引起的扭矩抖动。 */
	raw_speed = BLDC_EstimateSpeedRps(g_mech_angle_rad, g_prev_mech_angle_rad);
	g_prev_mech_angle_rad = g_mech_angle_rad;
	if ((fabsf(g_target_speed_rps) > BLDC_STARTUP_ASSIST_ENTRY_RPS) &&
		(raw_speed * g_target_speed_rps < 0.0f) &&
		(fabsf(raw_speed) < BLDC_SPEED_GLITCH_OPPO_RPS))
	{
		raw_speed = 0.0f;
	}
	if (fabsf(raw_speed - g_measured_speed_rps) > BLDC_SPEED_OUTLIER_RPS)
	{
		/* 异常跳变直接丢弃，防止单次毛刺把速度环拉反。 */
		raw_speed = g_measured_speed_rps;
	}
	/* 一阶低通平滑速度反馈，压掉编码器量化噪声和机械微抖。 */
	g_measured_speed_rps += BLDC_SPEED_LPF_ALPHA * (raw_speed - g_measured_speed_rps);

	speed_error = g_target_speed_rps - g_measured_speed_rps;
	target_abs = fabsf(g_target_speed_rps);
	meas_abs = fabsf(g_measured_speed_rps);

	if (target_abs < 0.05f)
	{
		BLDC_StopPwmOutput();
		g_m1_u_a = 0.0f;
		g_m1_u_b = 0.0f;
		g_m1_u_c = 0.0f;
		g_speed_integral = 0.0f;
		g_speed_prev_error = 0.0f;
		g_m1_vq_applied = 0.0f;
		g_m1_stall_assist_count = 0U;
		g_m1_assist_mode = false;
		g_m1_assist_release_count = 0U;
		return;
	}

	electrical_angle = BLDC_NormalizeAngle(BLDC_M1_ELEC_SIGN * g_mech_angle_rad * BLDC_POLE_PAIRS + g_m1_elec_offset_runtime);

	if ((!g_m1_assist_mode) &&
		(target_abs > BLDC_STARTUP_ASSIST_ENTRY_RPS) &&
		(meas_abs < BLDC_STARTUP_ASSIST_ENTRY_RPS))
	{
		/* 低速时先进入开环助启动，避免闭环在静摩擦区反复打转。 */
		g_m1_assist_mode = true;
		g_m1_assist_release_count = 0U;
		g_m1_open_loop_elec_angle = electrical_angle;
	}

	if (g_m1_assist_mode)
	{
		/* 助启动阶段同时维护开环角度和闭环估计，靠混合比平滑接管。 */
		assist_rps = BLDC_Clamp(target_abs, BLDC_STARTUP_ASSIST_MIN_RPS, BLDC_STARTUP_ASSIST_MAX_RPS);
		dir = (g_target_speed_rps >= 0.0f) ? 1.0f : -1.0f;
		open_vq = dir * (BLDC_STARTUP_ASSIST_VQ_MIN +
			(BLDC_STARTUP_ASSIST_VQ_MAX - BLDC_STARTUP_ASSIST_VQ_MIN) *
			BLDC_Clamp(target_abs / BLDC_STARTUP_ASSIST_MAX_RPS, 0.0f, 1.0f));
		closed_vq = BLDC_PIController(speed_error,
		                      BLDC_SPEED_PID_KP,
		                      BLDC_SPEED_PID_KI,
		                      BLDC_DT_S,
		                      &g_speed_integral);
		speed_diff = (speed_error - g_speed_prev_error) / BLDC_DT_S;
		g_speed_prev_error = speed_error;
		closed_vq += BLDC_SPEED_PID_KD * speed_diff;

		if (meas_abs > BLDC_STARTUP_ASSIST_EXIT_RPS)
		{
			if (g_m1_assist_release_count < BLDC_STARTUP_ASSIST_BLEND_MS)
			{
				g_m1_assist_release_count++;
			}
		}
		else
		{
			g_m1_assist_release_count = 0U;
		}

		/* release 计数越大，说明越接近闭环接管。 */
		blend = (float)g_m1_assist_release_count / (float)BLDC_STARTUP_ASSIST_BLEND_MS;
		blend = BLDC_Clamp(blend, 0.0f, 1.0f);

		if (g_m1_assist_release_count >= BLDC_STARTUP_ASSIST_BLEND_MS)
		{
			g_m1_assist_mode = false;
			g_m1_assist_release_count = 0U;
			g_m1_stall_assist_count = 0U;
		}
		else
		{
			float elec_step = dir * BLDC_TWO_PI * assist_rps * BLDC_POLE_PAIRS * BLDC_DT_S;
			float open_alpha;
			float open_beta;
			float closed_alpha;
			float closed_beta;

			/* 开环负责拉动转子，闭环负责纠正相位，最后按比例融合。 */
			g_m1_open_loop_elec_angle = BLDC_NormalizeAngle(g_m1_open_loop_elec_angle + elec_step);
			BLDC_InvParkTransform(0.0f, open_vq, g_m1_open_loop_elec_angle, &open_alpha, &open_beta);
			BLDC_InvParkTransform(0.0f, closed_vq, electrical_angle, &closed_alpha, &closed_beta);
			v_alpha = (1.0f - blend) * open_alpha + blend * closed_alpha;
			v_beta = (1.0f - blend) * open_beta + blend * closed_beta;
			BLDC_SetSpwmFromAlphaBeta(v_alpha, v_beta);
			g_m1_vq_applied = (1.0f - blend) * open_vq + blend * closed_vq;
			g_m1_stall_assist_count++;
			return;
		}
	}

	g_m1_stall_assist_count = 0U;
	v_q = BLDC_PIController(speed_error,
	                      BLDC_SPEED_PID_KP,
	                      BLDC_SPEED_PID_KI,
	                      BLDC_DT_S,
	                      &g_speed_integral);
	speed_diff = (speed_error - g_speed_prev_error) / BLDC_DT_S;
	g_speed_prev_error = speed_error;
	v_q += BLDC_SPEED_PID_KD * speed_diff;
	vq_delta = v_q - g_m1_vq_applied;
	if (vq_delta > BLDC_VQ_SLEW_PER_MS)
	{
		vq_delta = BLDC_VQ_SLEW_PER_MS;
	}
	else if (vq_delta < -BLDC_VQ_SLEW_PER_MS)
	{
		vq_delta = -BLDC_VQ_SLEW_PER_MS;
	}
	g_m1_vq_applied += vq_delta;
	v_q = g_m1_vq_applied;
	v_d = 0.0f;

	BLDC_InvParkTransform(v_d, v_q, electrical_angle, &v_alpha, &v_beta);
	BLDC_SetSpwmFromAlphaBeta(v_alpha, v_beta);

#if BLDC_ENABLE_MOTOR2
	if (!BLDC_ReadMechanicalAngle2(&g_mech_angle2_rad))
	{
		BLDC_StopPwmOutput2();
		g_speed2_integral = 0.0f;
		g_speed2_prev_error = 0.0f;
		return;
	}

	raw_speed = BLDC_EstimateSpeedRps(g_mech_angle2_rad, g_prev_mech_angle2_rad);
	g_prev_mech_angle2_rad = g_mech_angle2_rad;
	if (fabsf(raw_speed - g_measured_speed2_rps) > BLDC_SPEED_OUTLIER_RPS)
	{
		raw_speed = g_measured_speed2_rps;
	}
	g_measured_speed2_rps += BLDC_SPEED_LPF_ALPHA * (raw_speed - g_measured_speed2_rps);

	speed_error = g_target_speed2_rps - g_measured_speed2_rps;
	if (fabsf(g_target_speed2_rps) < 0.05f)
	{
		BLDC_StopPwmOutput2();
		g_speed2_integral = 0.0f;
		g_speed2_prev_error = 0.0f;
	}
	else
	{
		electrical_angle = BLDC_NormalizeAngle(BLDC_M2_ELEC_SIGN * g_mech_angle2_rad * BLDC_POLE_PAIRS + g_m2_elec_offset_runtime);
		v_q = BLDC_PIController(speed_error,
		                      BLDC_SPEED_PID_KP,
		                      BLDC_SPEED_PID_KI,
		                      BLDC_DT_S,
		                      &g_speed2_integral);
		speed_diff = (speed_error - g_speed2_prev_error) / BLDC_DT_S;
		g_speed2_prev_error = speed_error;
		v_q += BLDC_SPEED_PID_KD * speed_diff;
		v_d = 0.0f;

		BLDC_InvParkTransform(v_d, v_q, electrical_angle, &v_alpha, &v_beta);
		BLDC_SetSpwmFromAlphaBeta2(v_alpha, v_beta);
	}
#endif
}

/* 返回最近一次估算的机械转速。 */
float BLDC_GetMeasuredSpeedRps(void)
{
	return g_measured_speed_rps;
}

/* 返回当前机械角度弧度值。 */
float BLDC_GetMechanicalAngleRad(void)
{
	return g_mech_angle_rad;
}

/* 返回第二个无刷电机当前机械角度弧度值。 */
float BLDC_GetMechanicalAngle2Rad(void)
{
	return g_mech_angle2_rad;
}

/* 返回第二个无刷电机最近一次估算的机械转速。 */
float BLDC_GetMeasuredSpeed2Rps(void)
{
	return g_measured_speed2_rps;
}

uint16_t BLDC_GetLastRawAngleM1(void)
{
	return g_m1_raw_angle_last;
}

bool BLDC_GetLastRawAngleM1Valid(void)
{
	return g_m1_raw_angle_valid;
}

bool BLDC_ReadRawAngleM1Now(uint16_t *raw_angle)
{
	uint16_t raw = 0U;

	if (raw_angle == NULL)
	{
		return false;
	}

	if (!BLDC_DetectSensorConfig(&hi2c1, &g_m1_sensor_cfg))
	{
		g_m1_raw_angle_valid = false;
		return false;
	}

	if (!BLDC_ReadRawAngleFromI2C(&hi2c1, g_m1_sensor_cfg.addr, g_m1_sensor_cfg.reg, &raw))
	{
		g_m1_sensor_cfg.detected = false;
		g_m1_sensor_cfg.valid = false;
		g_m1_raw_angle_valid = false;
		return false;
	}

	g_m1_raw_angle_last = raw;
	g_m1_raw_angle_valid = true;
	*raw_angle = raw;
	return true;
}

int16_t BLDC_GetCurrentRawA(void)
{
	return 0;
}

int16_t BLDC_GetCurrentRawB(void)
{
	return 0;
}

float BLDC_GetCurrentD(void)
{
	return 0.0f;
}

float BLDC_GetCurrentQ(void)
{
	return 0.0f;
}

float BLDC_GetUa(void)
{
	return g_m1_u_a;
}

float BLDC_GetUb(void)
{
	return g_m1_u_b;
}

float BLDC_GetUc(void)
{
	return g_m1_u_c;
}
