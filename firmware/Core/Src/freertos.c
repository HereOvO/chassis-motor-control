/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "chassis_mode.h"
#include "chassis_feedback.h"
#include "chassis_odometry.h"
#include "chassis_protocol.h"
#include "runtime_tune.h"
#include "motor_control.h"
#include "motor_feedback.h"
#include "motor_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Definitions for MotorStatusMutex */
osMutexId_t MotorStatusMutexHandle;
const osMutexAttr_t MotorStatusMutex_attributes = {
  .name = "MotorStatusMutex"
};

/* Definitions for chassis control task */
osThreadId_t ChassisControlTaskHandle;
const osThreadAttr_t ChassisControlTask_attributes = {
  .name = "ChassisControlTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

static void ChassisControlTask(void *argument);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN 0 */
static void motor_driver_apply_signed_output(uint8_t motor_id, float command)
{
  MotorDirection_t direction;
  float duty_norm;

  if (command > 0.0001f) {
    direction = MOTOR_DIRECTION_FORWARD;
    duty_norm = command;
  } else if (command < -0.0001f) {
    direction = MOTOR_DIRECTION_REVERSE;
    duty_norm = -command;
  } else {
    direction = MOTOR_DIRECTION_STOP;
    duty_norm = 0.0f;
  }

  MotorDriver_SetOutput(motor_id, direction, duty_norm);
}
/* USER CODE END 0 */

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  MotorStatusMutexHandle = osMutexNew(&MotorStatusMutex_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  ChassisControlTaskHandle = osThreadNew(ChassisControlTask, NULL, &ChassisControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
}

static void ChassisControlTask(void *argument)
{
  (void)argument;
  chassis_cmd_t cmd;
  float motor_output[MOTOR_COUNT];
  Motor_Status_t motor_status[MOTOR_COUNT];
  uint32_t feedback_elapsed_ms = 0U;
  uint32_t pid_elapsed_ms = 0U;
  uint32_t report_elapsed_ms = 0U;
  uint8_t motor_id;

  chassis_odometry_init();

  for (motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
    MotorFeedback_Reset(motor_id);
  }

  for (;;) {
    while (chassis_protocol_poll(&cmd)) {
      (void)chassis_mode_apply_cmd(&cmd);
    }

    ++feedback_elapsed_ms;
    ++pid_elapsed_ms;
    ++report_elapsed_ms;

    if (feedback_elapsed_ms >= MOTOR_SPEED_SAMPLE_TIME_MS) {
      const chassis_profile_t *profile;

      feedback_elapsed_ms = 0U;
      for (motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        MotorFeedback_Sample(motor_id);
        motor_status[motor_id] = MotorFeedback_GetStatus(motor_id);
      }

      profile = runtime_tune_get_active_profile();
      (void)chassis_odometry_update(profile, motor_status, MOTOR_SPEED_SAMPLE_PERIOD_S);
    }

    if (pid_elapsed_ms >= MOTOR_PID_CONTROL_PERIOD_MS) {
      pid_elapsed_ms = 0U;
      if (!chassis_mode_raw_pwm_active()) {
        MotorControlCore_Compute(motor_output);
        for (motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
          motor_driver_apply_signed_output(motor_id, motor_output[motor_id]);
        }
      }
    }

    if (report_elapsed_ms >= 100U) {
      report_elapsed_ms = 0U;
      chassis_feedback_report();
    }

    osDelay(1);
  }
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */
