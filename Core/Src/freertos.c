/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "brushless_motor.h"
#include "core.h"
#include "i2c.h"
#include "steering_engine.h"
#include "usart.h"

#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BLDC_CMD_DEFAULT_RPS            (0.0f)
#define BLDC_CMD_MAX_ABS_RPS            (8.0f)
#define BLDC_CMD_SLEW_RPS_PER_SEC       (4.0f)
#define BLDC_CMD_INPUT_MIN_PERCENT      (0)
#define BLDC_CMD_INPUT_MAX_PERCENT      (100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static float g_speed_cmd_rps = BLDC_CMD_DEFAULT_RPS;
static int8_t g_motor_dir_sign = 1;
static uint8_t g_uart2_rx_dma_buf[64];
static uint8_t g_uart2_rx_dma_read_idx = 0U;
volatile char g_stack_overflow_task[16] = {0};

typedef enum
{
  CONTROL_MODE_BALANCE = 0,
  CONTROL_MODE_TEST = 1
} ControlMode;

static volatile ControlMode g_control_mode = CONTROL_MODE_BALANCE;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Read_Allead */
osThreadId_t Read_AlleadHandle;
const osThreadAttr_t Read_Allead_attributes = {
  .name = "Read_Allead",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Balance */
osThreadId_t BalanceHandle;
const osThreadAttr_t Balance_attributes = {
  .name = "Balance",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Run_By_Cmd */
osThreadId_t Run_By_CmdHandle;
const osThreadAttr_t Run_By_Cmd_attributes = {
  .name = "Run_By_Cmd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Give_Data */
osMessageQueueId_t Give_DataHandle;
const osMessageQueueAttr_t Give_Data_attributes = {
  .name = "Give_Data"
};
/* Definitions for Steering_Cmd */
osMessageQueueId_t Steering_CmdHandle;
const osMessageQueueAttr_t Steering_Cmd_attributes = {
  .name = "Steering_Cmd"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static bool ParsePercentCommand(const char *line, int *percent_value);
static bool ParseDirectionCommand(const char *line, int8_t *dir_sign);
static bool ParseDirectionSpeedCommand(const char *line, int8_t *dir_sign, int *percent_value);
static bool ParseSteeringCommand(const char *line, int *percent_value);
static bool ParsePitchOffsetCommand(const char *line, float *offset_rad);
static bool ParsePidCommand(const char *line, float *kp, float *ki, float *kd);
static bool ParsePitchQueryCommand(const char *line);
static bool ParseControlModeCommand(const char *line, ControlMode *mode);
static void ProcessUartCommandByte(uint8_t ch);
static bool PollUartSpeedCommandDma(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Give_Data */
  Give_DataHandle = osMessageQueueNew (16, sizeof(uint16_t), &Give_Data_attributes);

  /* creation of Steering_Cmd */
  Steering_CmdHandle = osMessageQueueNew (16, sizeof(uint8_t), &Steering_Cmd_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Read_Allead */
  Read_AlleadHandle = osThreadNew(StartTask02, NULL, &Read_Allead_attributes);

  /* creation of Balance */
  BalanceHandle = osThreadNew(StartTask03, NULL, &Balance_attributes);

  /* creation of Run_By_Cmd */
  Run_By_CmdHandle = osThreadNew(StartTask04, NULL, &Run_By_Cmd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Read_Allead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint16_t speed_cmd_raw = 0U;
  uint32_t print_div = 0U;
  uint32_t i2c1_ok = 0U;
  uint32_t i2c2_ok = 0U;
  uint32_t i2c3_ok = 0U;

  (void)argument;

  g_uart2_rx_dma_read_idx = 0U;
  if (HAL_UART_Receive_DMA(&huart2, g_uart2_rx_dma_buf, sizeof(g_uart2_rx_dma_buf)) != HAL_OK)
  {
    printf("[CMD] UART DMA start failed\r\n");
  }

  printf("[TASK02] UART+I2C loop online\r\n");
  printf("[CMD] send 0~100 to set motor1 speed percent\r\n");
  printf("[CMD] send fwd/rev or fwd 50/rev 50 to switch direction\r\n");
  printf("[CMD] send steering 0~100 to set 4-servo mirrored output\r\n");
  printf("[CMD] send PZ <rad> to set pitch zero\r\n");
  printf("[CMD] send PID <kp> <ki> <kd> to tune balance PID\r\n");
  printf("[CMD] send PID? to query current PID, pitch offset and pitch\r\n");
  printf("[CMD] send BAL or TEST to switch control mode\r\n");

  i2c1_ok = (HAL_I2C_IsDeviceReady(&hi2c1, (0x36U << 1), 2U, 10U) == HAL_OK) ? 1U : 0U;
  i2c2_ok = (HAL_I2C_IsDeviceReady(&hi2c2, (0x36U << 1), 2U, 10U) == HAL_OK) ? 1U : 0U;
  i2c3_ok = ((HAL_I2C_IsDeviceReady(&hi2c3, (0x68U << 1), 2U, 10U) == HAL_OK) ||
             (HAL_I2C_IsDeviceReady(&hi2c3, (0x69U << 1), 2U, 10U) == HAL_OK)) ? 1U : 0U;
  printf("[I2C] link i2c1=%lu i2c2=%lu i2c3=%lu\r\n",
         (unsigned long)i2c1_ok,
         (unsigned long)i2c2_ok,
         (unsigned long)i2c3_ok);

  Core_ControlInit();

  /* Infinite loop */
  for(;;)
  {
    (void)PollUartSpeedCommandDma();

    if (osMessageQueueGet(Give_DataHandle, &speed_cmd_raw, NULL, 0U) == osOK)
    {
      float cmd_rps = ((float)(int16_t)speed_cmd_raw) / 100.0f;

      if (cmd_rps > BLDC_CMD_MAX_ABS_RPS)
      {
        cmd_rps = BLDC_CMD_MAX_ABS_RPS;
      }
      else if (cmd_rps < -BLDC_CMD_MAX_ABS_RPS)
      {
        cmd_rps = -BLDC_CMD_MAX_ABS_RPS;
      }

      g_speed_cmd_rps = cmd_rps * (float)g_motor_dir_sign;
    }

    /* task02: all I2C-related reads/control execution. */
    Core_ImuUpdate1ms();

    if (g_control_mode == CONTROL_MODE_TEST)
    {
      BLDC_SetTargetSpeedRps(g_speed_cmd_rps);
      BLDC_SetTargetSpeed2Rps(g_speed_cmd_rps);
    }

    BLDC_ControlStep_1ms();

    if (++print_div >= 20U)
    {
      int32_t target_mrps;
            int32_t speed_m1_mrps;
            int32_t speed_m2_mrps;
      int32_t pitch_cdeg;
            const char *mode_text;

      print_div = 0U;
      target_mrps = (int32_t)(g_speed_cmd_rps * 1000.0f);
            speed_m1_mrps = (int32_t)(BLDC_GetMeasuredSpeedRps() * 1000.0f);
            speed_m2_mrps = (int32_t)(BLDC_GetMeasuredSpeed2Rps() * 1000.0f);
      pitch_cdeg = (int32_t)(Core_GetPitchDeg() * 100.0f);
            mode_text = (g_control_mode == CONTROL_MODE_BALANCE) ? "BAL" : "TEST";

            printf("[RUN] mode=%s target_mrps=%ld speed1_mrps=%ld speed2_mrps=%ld pitch_deg=%ld.%02ld\r\n",
              mode_text,
             (long)target_mrps,
              (long)speed_m1_mrps,
              (long)speed_m2_mrps,
         (long)(pitch_cdeg / 100),
         (long)labs((long)(pitch_cdeg % 100)));
    }

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Balance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  (void)argument;

  printf("[TASK03] Balance logic loop online\r\n");

  /* Infinite loop */
  for(;;)
  {
    if (g_control_mode == CONTROL_MODE_BALANCE)
    {
      Core_BalanceStep1ms();
    }
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Run_By_Cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint8_t steering_percent = 50U;

  (void)argument;

  SteeringEngine_Init();
  printf("[TASK04] Steering loop online\r\n");

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(Steering_CmdHandle, &steering_percent, NULL, 10U) == osOK)
    {
      if (SteeringEngine_SetSteeringPercent(steering_percent))
      {
        printf("[CMD] ACK steering percent=%u\r\n", (unsigned int)steering_percent);
      }
      else
      {
        printf("[CMD] ACK steering set failed percent=%u\r\n", (unsigned int)steering_percent);
      }
    }
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static bool ParsePercentCommand(const char *line, int *percent_value)
{
  char *end_ptr = NULL;
  long parsed;

  if ((line == NULL) || (percent_value == NULL))
  {
    return false;
  }

  parsed = strtol(line, &end_ptr, 10);
  if ((end_ptr == line) || (*end_ptr != '\0'))
  {
    return false;
  }
  if ((parsed < BLDC_CMD_INPUT_MIN_PERCENT) || (parsed > BLDC_CMD_INPUT_MAX_PERCENT))
  {
    return false;
  }

  *percent_value = (int)parsed;
  return true;
}

static bool ParseDirectionCommand(const char *line, int8_t *dir_sign)
{
  char lower_line[16] = {0};
  uint32_t i;

  if ((line == NULL) || (dir_sign == NULL))
  {
    return false;
  }

  for (i = 0U; (i < (sizeof(lower_line) - 1U)) && (line[i] != '\0'); i++)
  {
    lower_line[i] = (char)tolower((unsigned char)line[i]);
  }
  lower_line[i] = '\0';

  if (strcmp(lower_line, "fwd") == 0)
  {
    *dir_sign = 1;
    return true;
  }
  if (strcmp(lower_line, "rev") == 0)
  {
    *dir_sign = -1;
    return true;
  }
  return false;
}

static bool ParseDirectionSpeedCommand(const char *line, int8_t *dir_sign, int *percent_value)
{
  char lower_line[16] = {0};
  char dir_token[8] = {0};
  int percent = 0;
  int matched;
  uint32_t i;

  if ((line == NULL) || (dir_sign == NULL) || (percent_value == NULL))
  {
    return false;
  }

  for (i = 0U; (i < (sizeof(lower_line) - 1U)) && (line[i] != '\0'); i++)
  {
    lower_line[i] = (char)tolower((unsigned char)line[i]);
  }
  lower_line[i] = '\0';

  matched = sscanf(lower_line, "%7s %d", dir_token, &percent);
  if (matched != 2)
  {
    return false;
  }

  if (strcmp(dir_token, "fwd") == 0)
  {
    *dir_sign = 1;
  }
  else if (strcmp(dir_token, "rev") == 0)
  {
    *dir_sign = -1;
  }
  else
  {
    return false;
  }

  if ((percent < BLDC_CMD_INPUT_MIN_PERCENT) || (percent > BLDC_CMD_INPUT_MAX_PERCENT))
  {
    return false;
  }

  *percent_value = percent;
  return true;
}

static bool ParseSteeringCommand(const char *line, int *percent_value)
{
  char cmd[9] = {0};
  int percent = 0;
  int matched;

  if ((line == NULL) || (percent_value == NULL))
  {
    return false;
  }

  matched = sscanf(line, "%8s %d", cmd, &percent);
  if (matched != 2)
  {
    return false;
  }

  for (uint32_t i = 0U; (i < (sizeof(cmd) - 1U)) && (cmd[i] != '\0'); i++)
  {
    cmd[i] = (char)tolower((unsigned char)cmd[i]);
  }

  if (strcmp(cmd, "steering") != 0)
  {
    return false;
  }

  if ((percent < BLDC_CMD_INPUT_MIN_PERCENT) || (percent > BLDC_CMD_INPUT_MAX_PERCENT))
  {
    return false;
  }

  *percent_value = percent;

  return true;
}

static bool ParsePitchOffsetCommand(const char *line, float *offset_rad)
{
  char cmd[4] = {0};
  float value = 0.0f;
  int matched;

  if ((line == NULL) || (offset_rad == NULL))
  {
    return false;
  }

  matched = sscanf(line, "%3s %f", cmd, &value);
  if (matched != 2)
  {
    return false;
  }

  for (uint32_t i = 0U; i < sizeof(cmd) && cmd[i] != '\0'; i++)
  {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
  }

  if (strcmp(cmd, "PZ") != 0)
  {
    return false;
  }

  *offset_rad = value;
  return true;
}

static bool ParsePidCommand(const char *line, float *kp, float *ki, float *kd)
{
  char cmd[4] = {0};
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  int matched;

  if ((line == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL))
  {
    return false;
  }

  matched = sscanf(line, "%3s %f %f %f", cmd, &p, &i, &d);
  if (matched != 4)
  {
    return false;
  }

  for (uint32_t idx = 0U; idx < sizeof(cmd) && cmd[idx] != '\0'; idx++)
  {
    cmd[idx] = (char)toupper((unsigned char)cmd[idx]);
  }

  if (strcmp(cmd, "PID") != 0)
  {
    return false;
  }

  *kp = p;
  *ki = i;
  *kd = d;
  return true;
}

static bool ParsePitchQueryCommand(const char *line)
{
  char cmd[8] = {0};

  if (line == NULL)
  {
    return false;
  }

  if (sscanf(line, "%3s", cmd) != 1)
  {
    return false;
  }

  for (uint32_t i = 0U; i < sizeof(cmd) && cmd[i] != '\0'; i++)
  {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
  }

  return (strcmp(cmd, "P?") == 0) ||
         (strcmp(cmd, "PSTAT") == 0) ||
         (strcmp(cmd, "PID?") == 0) ||
         (strcmp(cmd, "PIDSTAT") == 0);
}

static bool ParseControlModeCommand(const char *line, ControlMode *mode)
{
  char cmd[8] = {0};

  if ((line == NULL) || (mode == NULL))
  {
    return false;
  }

  if (sscanf(line, "%7s", cmd) != 1)
  {
    return false;
  }

  for (uint32_t i = 0U; i < sizeof(cmd) && cmd[i] != '\0'; i++)
  {
    cmd[i] = (char)toupper((unsigned char)cmd[i]);
  }

  if ((strcmp(cmd, "BAL") == 0) || (strcmp(cmd, "BALANCE") == 0))
  {
    *mode = CONTROL_MODE_BALANCE;
    return true;
  }

  if ((strcmp(cmd, "TEST") == 0) || (strcmp(cmd, "MOTOR") == 0))
  {
    *mode = CONTROL_MODE_TEST;
    return true;
  }

  return false;
}

static void ProcessUartCommandByte(uint8_t ch)
{
  static char rx_line[16] = {0};
  static uint32_t rx_len = 0U;

  if ((ch == '\r') || (ch == '\n'))
  {
    if (rx_len > 0U)
    {
      int percent = 0;
      int8_t dir_sign = g_motor_dir_sign;
      float offset_rad = 0.0f;
      float kp = 0.0f;
      float ki = 0.0f;
      float kd = 0.0f;
      ControlMode mode = g_control_mode;

      rx_line[rx_len] = '\0';
      if (ParsePitchOffsetCommand(rx_line, &offset_rad))
      {
        Core_SetPitchOffsetRad(offset_rad);
        printf("[CORE] ACK pitch_offset_rad=%.4f pitch_deg=%.2f\r\n",
               (double)Core_GetPitchOffsetRad(),
               (double)Core_GetPitchDeg());
        rx_len = 0U;
        return;
      }

      if (ParsePidCommand(rx_line, &kp, &ki, &kd))
      {
        Core_SetBalancePid(kp, ki, kd);
        printf("[CORE] ACK pid kp=%.4f ki=%.4f kd=%.4f\r\n", (double)kp, (double)ki, (double)kd);
        rx_len = 0U;
        return;
      }

      if (ParsePitchQueryCommand(rx_line))
      {
        Core_GetBalancePid(&kp, &ki, &kd);
         printf("[CORE] pitch_deg=%.2f pitch_offset_rad=%.4f pid_kp=%.4f pid_ki=%.4f pid_kd=%.4f\r\n",
           (double)Core_GetPitchDeg(),
           (double)Core_GetPitchOffsetRad(),
           (double)kp,
           (double)ki,
           (double)kd);
        rx_len = 0U;
        return;
      }

      if (ParseControlModeCommand(rx_line, &mode))
      {
        g_control_mode = mode;
        g_speed_cmd_rps = 0.0f;
        BLDC_SetTargetSpeedRps(0.0f);
        BLDC_SetTargetSpeed2Rps(0.0f);
        Core_ResetBalanceIntegrator();
        printf("[CMD] ACK mode=%s\r\n", (mode == CONTROL_MODE_BALANCE) ? "BAL" : "TEST");
        rx_len = 0U;
        return;
      }

      if (ParseSteeringCommand(rx_line, &percent))
      {
        uint8_t steering_percent = (uint8_t)percent;
        osStatus_t queue_status = osMessageQueuePut(Steering_CmdHandle, &steering_percent, 0U, 0U);

        if (queue_status == osOK)
        {
          printf("[CMD] ACK steering queued percent=%d\r\n", percent);
        }
        else
        {
          printf("[CMD] ACK steering queue failed percent=%d queue=%d\r\n", percent, (int)queue_status);
        }

        rx_len = 0U;
        return;
      }

      if (ParseDirectionSpeedCommand(rx_line, &dir_sign, &percent))
      {
        float cmd_rps = ((float)percent / 100.0f) * BLDC_CMD_MAX_ABS_RPS;
        uint16_t speed_cmd_raw = (uint16_t)((int16_t)(cmd_rps * 100.0f));
        osStatus_t queue_status;

        if (g_control_mode != CONTROL_MODE_TEST)
        {
          printf("[CMD] ACK speed ignored in BAL mode, send TEST first\r\n");
          rx_len = 0U;
          return;
        }

        g_motor_dir_sign = dir_sign;
        queue_status = osMessageQueuePut(Give_DataHandle, &speed_cmd_raw, 0U, 0U);
        if (queue_status == osOK)
        {
          printf("[CMD] ACK ok percent=%d target=%.2f rps direction=%s\r\n",
                 percent,
                 cmd_rps * (float)g_motor_dir_sign,
                 (g_motor_dir_sign > 0) ? "fwd" : "rev");
        }
        else
        {
          printf("[CMD] ACK fail percent=%d target=%.2f rps queue=%d\r\n",
                 percent,
                 cmd_rps * (float)g_motor_dir_sign,
                 (int)queue_status);
        }

        rx_len = 0U;
        return;
      }

      if (ParseDirectionCommand(rx_line, &g_motor_dir_sign))
      {
        printf("[CMD] ACK direction=%s\r\n", (g_motor_dir_sign > 0) ? "fwd" : "rev");
        rx_len = 0U;
        return;
      }

      if (ParsePercentCommand(rx_line, &percent))
      {
        float cmd_rps = ((float)percent / 100.0f) * BLDC_CMD_MAX_ABS_RPS;
        uint16_t speed_cmd_raw = (uint16_t)((int16_t)(cmd_rps * 100.0f));
        osStatus_t queue_status;

        if (g_control_mode != CONTROL_MODE_TEST)
        {
          printf("[CMD] ACK speed ignored in BAL mode, send TEST first\r\n");
          rx_len = 0U;
          return;
        }

        queue_status = osMessageQueuePut(Give_DataHandle, &speed_cmd_raw, 0U, 0U);
        if (queue_status == osOK)
        {
          printf("[CMD] ACK ok percent=%d target=%.2f rps direction=%s\r\n",
                 percent,
                 cmd_rps * (float)g_motor_dir_sign,
                 (g_motor_dir_sign > 0) ? "fwd" : "rev");
        }
        else
        {
          printf("[CMD] ACK fail percent=%d target=%.2f rps queue=%d\r\n",
                 percent,
                 cmd_rps,
                 (int)queue_status);
        }
      }
      else
      {
        printf("[CMD] ACK reject input=%s (expect BAL/TEST | steering <0~100> | PZ <rad> | PID <kp> <ki> <kd> | PID? | 0~100 | fwd/rev | fwd 50/rev 50)\r\n", rx_line);
      }

      rx_len = 0U;
    }
    return;
  }

  if (rx_len < (sizeof(rx_line) - 1U))
  {
    if ((ch >= 0x20U) && (ch <= 0x7EU))
    {
      rx_line[rx_len++] = (char)ch;
    }
  }
  else
  {
    rx_len = 0U;
    printf("[CMD] ACK reject input too long\r\n");
  }
}

static bool PollUartSpeedCommandDma(void)
{
  uint8_t dma_write_idx;
  bool got_data = false;

  dma_write_idx = (uint8_t)(sizeof(g_uart2_rx_dma_buf) - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
  while (g_uart2_rx_dma_read_idx != dma_write_idx)
  {
    got_data = true;
    ProcessUartCommandByte(g_uart2_rx_dma_buf[g_uart2_rx_dma_read_idx]);
    g_uart2_rx_dma_read_idx++;
    if (g_uart2_rx_dma_read_idx >= sizeof(g_uart2_rx_dma_buf))
    {
      g_uart2_rx_dma_read_idx = 0U;
    }
  }

  return got_data;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  uint32_t i = 0U;

  (void)xTask;
  if (pcTaskName != NULL)
  {
    for (i = 0U; i < (sizeof(g_stack_overflow_task) - 1U) && pcTaskName[i] != '\0'; i++)
    {
      g_stack_overflow_task[i] = pcTaskName[i];
    }
    g_stack_overflow_task[i] = '\0';
  }
  __disable_irq();
  for(;;)
  {
  }
}

/* USER CODE END Application */

