#include "core.h"

#include "brushless_motor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CORE_PI                         (3.14159265358979323846f)
#define CORE_TWO_PI                     (6.28318530717958647692f)
#define CORE_DT_S                       (0.001f)
#define CORE_BALANCE_TARGET_PITCH_RAD    (0.0f)
#define CORE_BALANCE_KP_DEFAULT         (12.0f)
#define CORE_BALANCE_KI_DEFAULT         (0.8f)
#define CORE_BALANCE_KD_DEFAULT         (0.22f)
#define CORE_BALANCE_MAX_RPS             (8.0f)
#define CORE_BALANCE_INTEGRAL_LIMIT      (3.0f)
#define CORE_DMP_FIFO_RATE_HZ           (100U)
#define CORE_DMP_FEATURES               (DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL)

static float g_pitch_rad = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_pitch_offset_rad = 0.0f;
static float g_pitch_integral = 0.0f;
static float g_pitch_prev_error = 0.0f;
static float g_balance_kp = CORE_BALANCE_KP_DEFAULT;
static float g_balance_ki = CORE_BALANCE_KI_DEFAULT;
static float g_balance_kd = CORE_BALANCE_KD_DEFAULT;
static bool g_core_inited = false;
static bool g_mpu_ready = false;
static bool g_imu_sample_valid = false;

static float Core_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float Core_NormalizeAngle(float angle)
{
  while (angle >= CORE_TWO_PI)
  {
    angle -= CORE_TWO_PI;
  }
  while (angle < -CORE_PI)
  {
    angle += CORE_TWO_PI;
  }
  while (angle > CORE_PI)
  {
    angle -= CORE_TWO_PI;
  }
  return angle;
}

static float Core_QuaternionToPitchRad(const long *quat)
{
  float qw;
  float qx;
  float qy;
  float qz;
  float sinp;

  qw = (float)quat[0] / 1073741824.0f;
  qx = (float)quat[1] / 1073741824.0f;
  qy = (float)quat[2] / 1073741824.0f;
  qz = (float)quat[3] / 1073741824.0f;

  sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp >= 1.0f)
  {
    return CORE_PI * 0.5f;
  }
  if (sinp <= -1.0f)
  {
    return -CORE_PI * 0.5f;
  }

  return asinf(sinp);
}

static void Core_InitMpu(void)
{
  struct int_param_s int_param = {0};

  if (mpu_init(&int_param) != 0)
  {
    printf("[CORE] MPU init failed\r\n");
    g_mpu_ready = false;
    return;
  }

  if (dmp_load_motion_driver_firmware() != 0)
  {
    printf("[CORE] DMP firmware load failed\r\n");
    g_mpu_ready = false;
    return;
  }

  if (dmp_set_orientation(0U) != 0)
  {
    printf("[CORE] DMP orientation set failed\r\n");
    g_mpu_ready = false;
    return;
  }

  if (dmp_enable_feature(CORE_DMP_FEATURES) != 0)
  {
    printf("[CORE] DMP feature enable failed\r\n");
    g_mpu_ready = false;
    return;
  }

  if (dmp_set_fifo_rate(CORE_DMP_FIFO_RATE_HZ) != 0)
  {
    printf("[CORE] DMP fifo rate set failed\r\n");
    g_mpu_ready = false;
    return;
  }

  if (mpu_set_dmp_state(1U) != 0)
  {
    printf("[CORE] MPU DMP enable failed\r\n");
    g_mpu_ready = false;
    return;
  }

  g_mpu_ready = true;
  printf("[CORE] MPU6050 DMP ready\r\n");
}

void Core_ControlInit(void)
{
  if (g_core_inited)
  {
    return;
  }

  BLDC_Init();
  Core_InitMpu();
  g_pitch_rad = 0.0f;
  g_pitch_deg = 0.0f;
  g_pitch_offset_rad = 0.0f;
  g_pitch_integral = 0.0f;
  g_pitch_prev_error = 0.0f;
  g_balance_kp = CORE_BALANCE_KP_DEFAULT;
  g_balance_ki = CORE_BALANCE_KI_DEFAULT;
  g_balance_kd = CORE_BALANCE_KD_DEFAULT;
  g_imu_sample_valid = false;
  g_core_inited = true;
}

void Core_ImuUpdate1ms(void)
{
  short gyro[3] = {0};
  short accel[3] = {0};
  long quat[4] = {0};
  unsigned long timestamp = 0UL;
  unsigned char more = 0U;
  short sensors = 0;
  if (!g_core_inited || !g_mpu_ready)
  {
    g_imu_sample_valid = false;
    return;
  }

  if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0)
  {
    g_imu_sample_valid = false;
    return;
  }

  (void)gyro;
  (void)accel;
  (void)timestamp;
  (void)more;
  (void)sensors;

  g_pitch_rad = Core_NormalizeAngle(Core_QuaternionToPitchRad(quat) - g_pitch_offset_rad);
  g_pitch_deg = g_pitch_rad * 180.0f / CORE_PI;

  g_imu_sample_valid = true;
}

void Core_BalanceStep1ms(void)
{
  float pitch_error;
  float pitch_rate;
  float wheel_cmd_rps;

  if (!g_core_inited)
  {
    return;
  }

  if (!g_imu_sample_valid)
  {
    BLDC_SetTargetSpeedRps(0.0f);
    BLDC_SetTargetSpeed2Rps(0.0f);
    return;
  }

  pitch_error = CORE_BALANCE_TARGET_PITCH_RAD - g_pitch_rad;
  g_pitch_integral += pitch_error * CORE_DT_S;
  g_pitch_integral = Core_Clamp(g_pitch_integral, -CORE_BALANCE_INTEGRAL_LIMIT, CORE_BALANCE_INTEGRAL_LIMIT);
  pitch_rate = (pitch_error - g_pitch_prev_error) / CORE_DT_S;
  g_pitch_prev_error = pitch_error;

  wheel_cmd_rps = (g_balance_kp * pitch_error) +
                  (g_balance_ki * g_pitch_integral) +
                  (g_balance_kd * pitch_rate);
  wheel_cmd_rps = Core_Clamp(wheel_cmd_rps, -CORE_BALANCE_MAX_RPS, CORE_BALANCE_MAX_RPS);

  BLDC_SetTargetSpeedRps(wheel_cmd_rps);
  BLDC_SetTargetSpeed2Rps(wheel_cmd_rps);
}

void Core_ControlStep1ms(void)
{
  Core_ImuUpdate1ms();
  Core_BalanceStep1ms();
}

float Core_GetPitchRad(void)
{
  return g_pitch_rad;
}

float Core_GetPitchDeg(void)
{
  return g_pitch_deg;
}

void Core_SetPitchOffsetRad(float offset_rad)
{
  g_pitch_offset_rad = offset_rad;
  g_pitch_integral = 0.0f;
  g_pitch_prev_error = 0.0f;
}

float Core_GetPitchOffsetRad(void)
{
  return g_pitch_offset_rad;
}

void Core_SetBalancePid(float kp, float ki, float kd)
{
  g_balance_kp = kp;
  g_balance_ki = ki;
  g_balance_kd = kd;
  g_pitch_integral = 0.0f;
  g_pitch_prev_error = 0.0f;
}

void Core_GetBalancePid(float *kp, float *ki, float *kd)
{
  if (kp != NULL)
  {
    *kp = g_balance_kp;
  }
  if (ki != NULL)
  {
    *ki = g_balance_ki;
  }
  if (kd != NULL)
  {
    *kd = g_balance_kd;
  }
}

void Core_ResetBalanceIntegrator(void)
{
  g_pitch_integral = 0.0f;
  g_pitch_prev_error = 0.0f;
}