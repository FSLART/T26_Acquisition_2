/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//Ola


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

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

/* USER CODE BEGIN PV */
volatile uint32_t time_ms = 0;

// CAN
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

// --- WHEEL SPEED SENSORS CONFIGURATION ---
typedef struct {
    volatile uint16_t last_capture;     // Store timer reading from previous tooth
    volatile uint16_t delta_counts;    // Time interval between 2 teeth (in microseconds)
    volatile float rpm;                 // Calculated speed in RPM
    volatile uint8_t first_capture;     // Flag to ignore first pulse at startup
    volatile uint32_t last_pulse_ms;    // Timestamp (in ms) of the last detected tooth
} WheelSensor;

// Instances for Wheel Sensors
WheelSensor sensor_wheel_1 = {0, 0, 0.0f, 1, 0}; // Connected to PB0 (TIM3_CH3)
WheelSensor sensor_wheel_2 = {0, 0, 0.0f, 1, 0}; // Connected to PB1 (TIM3_CH4)

// Tasks Prototypes
void execute_immediate_tasks(void);
void execute_10ms_tasks(void);
void execute_50ms_tasks(void);
void execute_100ms_tasks(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float MeasureWheelRPM(uint16_t delta_us);

int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*) data, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if (htim->Instance == TIM7) {
        time_ms++;
    }
}

// Hardware Capture Interrupt: Triggered automatically on rising edges (PB0 & PB1)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

        // --- SENSOR 1: PB0 (TIM3 Channel 3) ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            uint16_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

            if (!sensor_wheel_1.first_capture) {
                // 16-bit subtraction handles counter overflow (65535 -> 0) automatically
                sensor_wheel_1.delta_counts = (uint16_t)(current_capture - sensor_wheel_1.last_capture);

                if (sensor_wheel_1.delta_counts > 0) {
                    sensor_wheel_1.rpm = MeasureWheelRPM(sensor_wheel_1.delta_counts);
                    sensor_wheel_1.last_pulse_ms = time_ms; // Save timestamp of last pulse
                }
            } else {
                sensor_wheel_1.first_capture = 0;
            }
            sensor_wheel_1.last_capture = current_capture;
        }

        // --- SENSOR 2: PB1 (TIM3 Channel 4) ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            uint16_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

            if (!sensor_wheel_2.first_capture) {
                sensor_wheel_2.delta_counts = (uint16_t)(current_capture - sensor_wheel_2.last_capture);

                if (sensor_wheel_2.delta_counts > 0) {
                    sensor_wheel_2.rpm = MeasureWheelRPM(sensor_wheel_2.delta_counts);
                    sensor_wheel_2.last_pulse_ms = time_ms; // Save timestamp of last pulse
                }
            } else {
                sensor_wheel_2.first_capture = 0;
            }
            sensor_wheel_2.last_capture = current_capture;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);

  // Start Input Capture for both wheel speed sensors
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); // PB0
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); // PB1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // Execute immediate tasks
        execute_immediate_tasks();

        static uint32_t previus_tick_10ms = 0;
        static uint32_t previus_tick_50ms = 0;
        static uint32_t previus_tick_100ms = 0;

        // Execute 10ms Tasks
        if (time_ms - previus_tick_10ms >= 10) {
            execute_10ms_tasks();
            previus_tick_10ms = time_ms;
        }

        // Execute 50ms Tasks
        if (time_ms - previus_tick_50ms >= 50) {
            execute_50ms_tasks();
            previus_tick_50ms = time_ms;
        }

        // Execute 100ms Tasks
        if (time_ms - previus_tick_100ms >= 100) {
            execute_100ms_tasks();
            previus_tick_100ms = time_ms;
        }
    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void execute_immediate_tasks() {

}

void execute_10ms_tasks() {
    HAL_IWDG_Refresh(&hiwdg);

    // Standstill Detection: Force 0 RPM if no pulse received in the last 300ms
    if (time_ms - sensor_wheel_1.last_pulse_ms > 300) {
        sensor_wheel_1.rpm = 0.0f;
    }

    if (time_ms - sensor_wheel_2.last_pulse_ms > 300) {
        sensor_wheel_2.rpm = 0.0f;
    }
}

void execute_50ms_tasks() {

    // 1. Scale RPM by 10 for 1-decimal precision as uint16_t (e.g., 123.4 RPM -> 1234)
    uint16_t wheel1CAN = (uint16_t)(sensor_wheel_1.rpm * 10.0f);
    uint16_t wheel2CAN = (uint16_t)(sensor_wheel_2.rpm * 10.0f);

    // 2. CAN Frame Setup
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x710;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4;

    TxData[0] = wheel1CAN & 0xFF;         // Sensor 1 LSB
    TxData[1] = (wheel1CAN >> 8) & 0xFF;  // Sensor 1 MSB
    TxData[2] = wheel2CAN & 0xFF;         // Sensor 2 LSB
    TxData[3] = (wheel2CAN >> 8) & 0xFF;  // Sensor 2 MSB

    // 3. Send Message to CAN Buses
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }

    /*if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }*/
}

void execute_100ms_tasks() {
    HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

    // Velocity sensor debug
    printf("Wheel 1: %.1f RPM | Wheel 2: %.1f RPM\r\n", sensor_wheel_1.rpm, sensor_wheel_2.rpm);
}

// Measurements

// Calculates Wheel RPM based on time between teeth (microseconds)
float MeasureWheelRPM(uint16_t delta_us)
{
    const float TIMER_FREQ_HZ = 1000000.0f; // 1 MHz Timer Clock -> 1 us resolution
    const float NUMBER_OF_TEETH = 20.0f;   // Disc teeth count

    if (delta_us == 0) {
        return 0.0f;
    }

    // Formula: RPM = (60 sec * 1,000,000 us) / (20 teeth * delta_us)
    return (60.0f * TIMER_FREQ_HZ) / (NUMBER_OF_TEETH * (float)delta_us);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
