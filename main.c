/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_INTERVAL 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
int id;
short xy[2];
unsigned char op;
unsigned char g_input;
int is_interrupt = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size, uint32_t timeout, int isInterrupt) {
	if (isInterrupt) {
		HAL_UART_Transmit_IT(huart, pData, size);
	}
	else {
		HAL_UART_Transmit(huart, pData, size, timeout);
	}
}

void HandleInput(unsigned char input, int isInterrupt) {
	Transmit(&huart6, &input, sizeof(input), 100, isInterrupt);
	int is_error = 0;

	if ('0' <= input && input <= '9') {
		if (xy[id] > 3276 || (xy[id] > 3275 && input > '7')) {
			is_error = 1;
		} else {
			xy[id] = xy[id] * 10 + input - '0';
		}
	} else if (input == '=') {
		short result = 0;

		if (id == 0) {
			is_error = 1;
		} else
			switch (op) {
			case '+':
				if (xy[1] > 0x7FFF - xy[0]) {
					is_error = 1;
				} else {
					result = xy[0] + xy[1];
				}
				break;

			case '-':
				result = xy[0] - xy[1];
				break;

			case '*':
				if (xy[1] != 0 && xy[0] >= 0x7FFF / xy[1]) {
					is_error = 1;
				} else {
					result = xy[0] * xy[1];
				}
				break;

			case '/':
				if (xy[1] == 0) {
					is_error = 1;
				} else {
					result = xy[0] / xy[1];
				}
				break;
			}

		if (!is_error) {
			unsigned char buf[8];
			int bufpos = sizeof(buf) / sizeof(*buf);
			buf[--bufpos] = '\n';
			buf[--bufpos] = '\r';
			int is_negative = result < 0;

			if (is_negative) {
				result = -result;
			}

			if (!result) {
				buf[--bufpos] = '0';
			} else
				while (result) {
					buf[--bufpos] = result % 10 + '0';
					result /= 10;
				}

			if (is_negative) {
				buf[--bufpos] = '-';
			}

			Transmit(&huart6, &buf[bufpos], sizeof(buf) - sizeof(*buf) * bufpos, 100, isInterrupt);
		}

		xy[0] = 0;
		xy[1] = 0;
		id = 0;
	} else
		switch (input) {
		case '+':
		case '-':
		case '*':
		case '/':
			op = input;
			id = 1;
			break;
		}

	if (is_error) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		char txt_err[] = "error\r\n";
		Transmit(&huart6, txt_err, sizeof(txt_err) - 1, 100, isInterrupt);
		xy[0] = 0;
		xy[1] = 0;
		id = 0;
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HandleInput(g_input, 1);
}

int ButtonGetState(void) {
	switch (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) {
	case GPIO_PIN_SET:
		return 0;
	default:
		return 1;
	}
}

int ButtonGetStateSmooth(void) {
	int sum = 0;
	for (int i = 0; i < BUTTON_INTERVAL; ++i) {
		sum += ButtonGetState();
		HAL_Delay(1);
	}
	return 2 * sum > BUTTON_INTERVAL ? 1 : 0;
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	int was_button_pressed = 0;
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	for (;;) {
		if (ButtonGetStateSmooth()) {
			was_button_pressed = 1;
		} else {
			if (was_button_pressed) {
				is_interrupt = !is_interrupt;
			}
			was_button_pressed = 0;
		}

		if (is_interrupt) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			switch (HAL_UART_Receive_IT(&huart6, &g_input, sizeof(g_input))) {
			case HAL_OK:
				break;

			case HAL_ERROR:
			case HAL_BUSY:
			case HAL_TIMEOUT:
				continue;
			}
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

			switch (HAL_UART_Receive(&huart6, &g_input, sizeof(g_input), 100)) {
			case HAL_OK:
				break;

			case HAL_ERROR:
			case HAL_BUSY:
			case HAL_TIMEOUT:
				continue;
			}

			HandleInput(g_input, 0);
		}

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
