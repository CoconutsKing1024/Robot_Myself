#include "core.h"

#include "brushless_motor.h"
#include "i2c.h"
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
static uint32_t g_imu_ok_count = 0U;
static uint32_t g_imu_fail_count = 0U;
static uint32_t g_imu_stale_ms = 0U;
static uint32_t g_imu_init_stage = 0U;
static int32_t g_imu_init_last_ret = 0;

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
  uint8_t whoami = 0U;
  uint32_t i2c68_ok = 0U;
  uint32_t i2c69_ok = 0U;
  int ret = 0;

  g_imu_init_stage = 1U;
  g_imu_init_last_ret = 0;

  i2c68_ok = (HAL_I2C_IsDeviceReady(&hi2c3, (0x68U << 1), 2U, 10U) == HAL_OK) ? 1U : 0U;
  i2c69_ok = (HAL_I2C_IsDeviceReady(&hi2c3, (0x69U << 1), 2U, 10U) == HAL_OK) ? 1U : 0U;

  if ((i2c68_ok == 1U) &&
      (HAL_I2C_Mem_Read(&hi2c3, (0x68U << 1), 0x75U, I2C_MEMADD_SIZE_8BIT, &whoami, 1U, 20U) == HAL_OK))
  {
    printf("[CORE] probe 0x68 whoami=0x%02X\r\n", whoami);
  }
  if ((i2c69_ok == 1U) &&
      (HAL_I2C_Mem_Read(&hi2c3, (0x69U << 1), 0x75U, I2C_MEMADD_SIZE_8BIT, &whoami, 1U, 20U) == HAL_OK))
  {
    printf("[CORE] probe 0x69 whoami=0x%02X\r\n", whoami);
  }
  printf("[CORE] i2c_probe_imu 0x68=%lu 0x69=%lu\r\n",
         (unsigned long)i2c68_ok,
         (unsigned long)i2c69_ok);

  g_imu_init_stage = 2U;
  ret = mpu_init(&int_param);
  if (ret != 0)
  {
    g_imu_init_stage = 12U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] MPU init failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 3U;
  ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (ret != 0)
  {
    g_imu_init_stage = 23U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] MPU sensor enable failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 31U;
  ret = dmp_load_motion_driver_firmware();
  if (ret != 0)
  {
    g_imu_init_stage = 13U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] DMP firmware load failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 4U;
  ret = dmp_set_orientation(0U);
  if (ret != 0)
  {
    g_imu_init_stage = 14U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] DMP orientation set failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 5U;
  ret = dmp_enable_feature(CORE_DMP_FEATURES);
  if (ret != 0)
  {
    g_imu_init_stage = 15U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] DMP feature enable failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 6U;
  ret = dmp_set_fifo_rate(CORE_DMP_FIFO_RATE_HZ);
  if (ret != 0)
  {
    g_imu_init_stage = 16U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] DMP fifo rate set failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 7U;
  ret = mpu_set_dmp_state(1U);
  if (ret != 0)
  {
    g_imu_init_stage = 17U;
    g_imu_init_last_ret = (int32_t)ret;
    printf("[CORE] MPU DMP enable failed ret=%d\r\n", ret);
    g_mpu_ready = false;
    return;
  }

  g_imu_init_stage = 100U;
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
  g_imu_ok_count = 0U;
  g_imu_fail_count = 0U;
  g_imu_stale_ms = 0U;
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
    g_imu_fail_count++;

    if (g_imu_stale_ms < 2000U)
    {
      g_imu_stale_ms++;
    }
    else
    {
      /* Long consecutive FIFO failure means sample is stale. */
      g_imu_sample_valid = false;
    }
    return;
  }

  (void)gyro;
  (void)accel;
  (void)timestamp;
  (void)more;
  (void)sensors;

  g_pitch_rad = Core_NormalizeAngle(Core_QuaternionToPitchRad(quat) - g_pitch_offset_rad);
  g_pitch_deg = g_pitch_rad * 180.0f / CORE_PI;

  g_imu_ok_count++;
  g_imu_stale_ms = 0U;
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

bool Core_IsImuReady(void)
{
  return g_mpu_ready;
}

bool Core_IsImuSampleValid(void)
{
  return g_imu_sample_valid;
}

uint32_t Core_GetImuOkCount(void)
{
  return g_imu_ok_count;
}

uint32_t Core_GetImuFailCount(void)
{
  return g_imu_fail_count;
}

uint32_t Core_GetImuInitStage(void)
{
  return g_imu_init_stage;
}

int32_t Core_GetImuInitLastRet(void)
{
  return g_imu_init_last_ret;
}