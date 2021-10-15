/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
int id;
short xy[2];
unsigned char op;
unsigned char g_input;
int is_interrupt = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HandleInput(unsigned char input) {
	HAL_UART_Transmit(&huart6, &input, sizeof(input), 100);
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

			HAL_UART_Transmit(&huart6, &buf[bufpos],
					sizeof(buf) - sizeof(*buf) * bufpos, 100);
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
		HAL_UART_Transmit(&huart6, txt_err, sizeof(txt_err) - 1, 100);
		xy[0] = 0;
		xy[1] = 0;
		id = 0;
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HandleInput(g_input);
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
int main(void) {
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

			HandleInput(g_input);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
