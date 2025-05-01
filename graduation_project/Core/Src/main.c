/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MOTOR_DISABLED = 0,
	MOTOR_ENABLED = 1
} MotorState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define min(a, b) ((a) < (b) ? (a) : (b))
#define StepPerRevolution 6400

// Enhanced tuning parameters
#define BASE_PULSE_WIDTH 150       // Increased for more stability
#define ACCELERATION_STEPS 800     // Longer acceleration
#define POT_SAMPLES 15             // More samples for better filtering
#define POT_DEADZONE 100           // Larger deadzone
#define POSITION_TOLERANCE 50      // Larger position tolerance
#define MIN_MOVE_STEPS 20          // Minimum steps to move
#define MAX_SPEED_CHANGE 50        // Limit speed changes between steps
#define HYSTERESIS 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim8;

/* Definitions for First */
osThreadId_t FirstHandle;
const osThreadAttr_t First_attributes = {
  .name = "First",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Second */
osThreadId_t SecondHandle;
const osThreadAttr_t Second_attributes = {
  .name = "Second",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Third */
osThreadId_t ThirdHandle;
const osThreadAttr_t Third_attributes = {
  .name = "Third",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//for first motor
int lastStepPosition1 = 0;
MotorState motorEnabled1 = MOTOR_DISABLED;
int lastStablePotValue1 = 0;
uint32_t adcValue1 = 0;
int lastMovementDirection1 = 0;
int currentSpeed1 = BASE_PULSE_WIDTH;

//for second motor
int lastStepPosition2 = 0;
MotorState motorEnabled2 = MOTOR_DISABLED;
int lastStablePotValue2 = 0;
uint32_t adcValue2 = 0;
int lastMovementDirection2 = 0;
int currentSpeed2 = BASE_PULSE_WIDTH;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
void FirstMotor(void *argument);
void SecondMotor(void *argument);
void ThirdMotor(void *argument);

/* USER CODE BEGIN PFP */
int readStablePot(void);
void moveToPositionSmooth(int targetStep);
int readStablePot2(void);
void moveToPositionSmooth2(int targetStep);
void HAL_Delay_us(uint32_t microseconds);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int readStablePot() {
	static int samples[POT_SAMPLES];
	static int index = 0;

	// Exponential moving average filter (better for real-time)
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		samples[index] = HAL_ADC_GetValue(&hadc1);
	}
	index = (index + 1) % POT_SAMPLES;

	// Exponential moving average (more responsive than simple average)
	float alpha = 0.2; // Smoothing factor
	static float filteredValue = 0;
	if(filteredValue == 0) {
		filteredValue = samples[0]; // Initialize
	}
	for(int i = 0; i < POT_SAMPLES; i++) {
		filteredValue = alpha * samples[i] + (1 - alpha) * filteredValue;
	}
	return (int)filteredValue;
}

void moveToPositionSmooth(int targetStep) {
	// Calculate movement with enhanced hysteresis
	int direction;
	int totalSteps = abs(targetStep - lastStepPosition1);

	// Enhanced hysteresis with direction memory
	if(abs(totalSteps) < MIN_MOVE_STEPS) {
		return; // Ignore tiny movements
	}

	direction = (targetStep > lastStepPosition1) ? 1 : -1;

	// Only reverse direction if we have significant movement
	if((direction * lastMovementDirection1) < 0 && totalSteps < (POSITION_TOLERANCE*2)) {
		direction = lastMovementDirection1; // Maintain direction
		totalSteps = abs(targetStep - lastStepPosition1); // Recalculate
	}

	// Handle wrap-around
	if(totalSteps > StepPerRevolution/2) {
		direction *= -1;
		totalSteps = StepPerRevolution - totalSteps;
	}

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	lastMovementDirection1 = direction;

	// Enhanced acceleration profile
	int accelSteps = min(ACCELERATION_STEPS, totalSteps/2);
	int decelStart = totalSteps - accelSteps;

	for(int i = 0; i < totalSteps; i++) {
		int targetPulseWidth = BASE_PULSE_WIDTH;

		// Smoother acceleration using cubic easing
		if(i < accelSteps) {
			float progress = (float)i / accelSteps;
			targetPulseWidth = BASE_PULSE_WIDTH*2 - (BASE_PULSE_WIDTH * progress * progress * progress);
		}
		else if(i >= decelStart) {
			float progress = (float)(i-decelStart) / accelSteps;
			targetPulseWidth = BASE_PULSE_WIDTH + (BASE_PULSE_WIDTH * progress * progress * progress);
		}

		// Limit speed changes for smoother motion
		if(abs(targetPulseWidth - currentSpeed1) > MAX_SPEED_CHANGE) {
			if(targetPulseWidth < currentSpeed1) {
				targetPulseWidth = currentSpeed1 - MAX_SPEED_CHANGE;
			} else {
				targetPulseWidth = currentSpeed1 + MAX_SPEED_CHANGE;
			}
		}
		currentSpeed1 = targetPulseWidth;

		// Execute step with precise timing
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay_us(targetPulseWidth/2);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_Delay_us(targetPulseWidth/2);

		// Update position
		lastStepPosition1 += direction;
		if(lastStepPosition1 >= StepPerRevolution) lastStepPosition1 = 0;
		if(lastStepPosition1 < 0) lastStepPosition1 = StepPerRevolution - 1;
	}
}

int readStablePot2() {
    static int samples[POT_SAMPLES];
    static int index = 0;

    HAL_ADC_Start(&hadc1);

    // Skip first conversion (channel 15 = pot1)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        HAL_ADC_GetValue(&hadc1); // discard channel 15
    }

    // Read second conversion (channel 14 = A2 = pot2)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        samples[index] = HAL_ADC_GetValue(&hadc1); // read channel 14
    }

    HAL_ADC_Stop(&hadc1);

    index = (index + 1) % POT_SAMPLES;

    // Exponential moving average
    float alpha = 0.2f;
    static float filteredValue = 0;
    if (filteredValue == 0) {
        filteredValue = samples[0]; // Initialize
    }
    for (int i = 0; i < POT_SAMPLES; i++) {
        filteredValue = alpha * samples[i] + (1 - alpha) * filteredValue;
    }
    return (int)filteredValue;
}



void moveToPositionSmooth2(int targetStep) {
	// Calculate movement with enhanced hysteresis
	int direction;
	int totalSteps = abs(targetStep - lastStepPosition2);

	// Enhanced hysteresis with direction memory
	if(abs(totalSteps) < MIN_MOVE_STEPS) {
		return; // Ignore tiny movements
	}

	direction = (targetStep > lastStepPosition2) ? 1 : -1;

	// Only reverse direction if we have significant movement
	if((direction * lastMovementDirection2) < 0 && totalSteps < (POSITION_TOLERANCE*2)) {
		direction = lastMovementDirection2; // Maintain direction
		totalSteps = abs(targetStep - lastStepPosition2); // Recalculate
	}

	// Handle wrap-around
	if(totalSteps > StepPerRevolution/2) {
		direction *= -1;
		totalSteps = StepPerRevolution - totalSteps;
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	lastMovementDirection2 = direction;

	// Enhanced acceleration profile
	int accelSteps = min(ACCELERATION_STEPS, totalSteps/2);
	int decelStart = totalSteps - accelSteps;

	for(int i = 0; i < totalSteps; i++) {
		int targetPulseWidth = BASE_PULSE_WIDTH;

		// Smoother acceleration using cubic easing
		if(i < accelSteps) {
			float progress = (float)i / accelSteps;
			targetPulseWidth = BASE_PULSE_WIDTH*2 - (BASE_PULSE_WIDTH * progress * progress * progress);
		}
		else if(i >= decelStart) {
			float progress = (float)(i-decelStart) / accelSteps;
			targetPulseWidth = BASE_PULSE_WIDTH + (BASE_PULSE_WIDTH * progress * progress * progress);
		}

		// Limit speed changes for smoother motion
		if(abs(targetPulseWidth - currentSpeed2) > MAX_SPEED_CHANGE) {
			if(targetPulseWidth < currentSpeed2) {
				targetPulseWidth = currentSpeed2 - MAX_SPEED_CHANGE;
			} else {
				targetPulseWidth = currentSpeed2 + MAX_SPEED_CHANGE;
			}
		}
		currentSpeed2 = targetPulseWidth;

		// Execute step with precise timing
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay_us(targetPulseWidth/2);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay_us(targetPulseWidth/2);

		// Update position
		lastStepPosition2 += direction;
		if(lastStepPosition2 >= StepPerRevolution) lastStepPosition2 = 0;
		if(lastStepPosition2 < 0) lastStepPosition2 = StepPerRevolution - 1;
	}
}

// Microsecond delay function
void HAL_Delay_us(uint32_t microseconds) {
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = (SystemCoreClock / 1000000) * microseconds;

	while ((DWT->CYCCNT - start) < cycles) {
		// Wait
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	// Enable DWT for microsecond delay
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	// Start TIM8 for ADC triggering
	HAL_TIM_Base_Start(&htim8);

	// Start with motor disabled
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
	motorEnabled1 = MOTOR_DISABLED;
	lastStablePotValue1 = readStablePot();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	motorEnabled2 = MOTOR_DISABLED;
	lastStablePotValue2 = readStablePot2();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of First */
  FirstHandle = osThreadNew(FirstMotor, NULL, &First_attributes);

  /* creation of Second */
  SecondHandle = osThreadNew(SecondMotor, NULL, &Second_attributes);

  /* creation of Third */
  ThirdHandle = osThreadNew(ThirdMotor, NULL, &Third_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 28000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUL_Motor3_Pin|DIR_Motor3_Pin|ENA_Motor3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PUL_Motor1_Pin|DIR_Motro1_Pin|ENA_Motor1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PUL_Motor2_Pin|DIR_Motor2_Pin|ENA_Motor2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUL_Motor3_Pin DIR_Motor3_Pin ENA_Motor3_Pin */
  GPIO_InitStruct.Pin = PUL_Motor3_Pin|DIR_Motor3_Pin|ENA_Motor3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL_Motor1_Pin DIR_Motro1_Pin ENA_Motor1_Pin */
  GPIO_InitStruct.Pin = PUL_Motor1_Pin|DIR_Motro1_Pin|ENA_Motor1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL_Motor2_Pin DIR_Motor2_Pin ENA_Motor2_Pin */
  GPIO_InitStruct.Pin = PUL_Motor2_Pin|DIR_Motor2_Pin|ENA_Motor2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_FirstMotor */
/**
 * @brief  Function implementing the First thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_FirstMotor */
void FirstMotor(void *argument)
{
  /* USER CODE BEGIN 5 */
	// Initialize motor state
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); // Start with motor disabled
	motorEnabled1 = MOTOR_DISABLED;
	lastStablePotValue1 = readStablePot();

	/* Infinite loop */
	for(;;)
	{
		int potValue = readStablePot();
		int targetStep = (int)((float)potValue / 65535.0f * (StepPerRevolution-1));

		// Only react to significant pot changes with hysteresis
		int potDifference = abs(potValue - lastStablePotValue1);

		if(potDifference > POT_DEADZONE ||
				(motorEnabled1 == MOTOR_ENABLED && potDifference > POT_DEADZONE/2)) {
			if(motorEnabled1 == MOTOR_DISABLED) {
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
				motorEnabled1 = MOTOR_ENABLED;
			}

			moveToPositionSmooth(targetStep);
			lastStablePotValue1 = potValue;
		}
		else if(motorEnabled1 == MOTOR_ENABLED &&
				abs(targetStep - lastStepPosition1) <= POSITION_TOLERANCE) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
			motorEnabled1 = MOTOR_DISABLED;
		}

		osDelay(10); // Delay using FreeRTOS (10ms delay)
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SecondMotor */
/**
 * @brief Function implementing the Second thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SecondMotor */
void SecondMotor(void *argument)
{
  /* USER CODE BEGIN SecondMotor */
	// Initialize motor state
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start with motor disabled
	motorEnabled2 = MOTOR_DISABLED;
	lastStablePotValue2 = readStablePot2();
	/* Infinite loop */
	for(;;)
	{
		int potValue = readStablePot2();
		int targetStep = (int)((float)potValue / 65535.0f * (StepPerRevolution-1));

		// Only react to significant pot changes with hysteresis
		int potDifference = abs(potValue - lastStablePotValue2);

		if(potDifference > POT_DEADZONE ||
				(motorEnabled2 == MOTOR_ENABLED && potDifference > POT_DEADZONE/2)) {
			if(motorEnabled2 == MOTOR_DISABLED) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				motorEnabled2 = MOTOR_ENABLED;
			}

			moveToPositionSmooth2(targetStep);
			lastStablePotValue2 = potValue;
		}
		else if(motorEnabled2 == MOTOR_ENABLED &&
				abs(targetStep - lastStepPosition2) <= POSITION_TOLERANCE) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
			motorEnabled2 = MOTOR_DISABLED;
		}

		osDelay(10); // Delay using FreeRTOS (10ms delay)
	}
  /* USER CODE END SecondMotor */
}

/* USER CODE BEGIN Header_ThirdMotor */
/**
* @brief Function implementing the Third thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ThirdMotor */
void ThirdMotor(void *argument)
{
  /* USER CODE BEGIN ThirdMotor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ThirdMotor */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
