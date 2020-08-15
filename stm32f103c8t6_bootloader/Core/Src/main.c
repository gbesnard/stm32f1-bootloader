/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*p_function)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APPLICATION_START_ADDRESS 0x08004000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t check_firmware_update(void);
static void flash_application_erase(void);
static void firmware_update(void);
static void app_branch(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief This function check if update possible (ie, data available on UART).
 */
static uint8_t check_firmware_update(void) {
	uint32_t wait_delay = 37000000u; // TODO: magic number, corresponding approximately to 30 secs.
	uint8_t res = 0u;
	uint8_t dummy_byte;

	 // Ready to receive update, switch on the LED.
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	// Wait a while for an update to become available, and if not, proceed to application jump.
	while (wait_delay != 0u && res == 0u)	{
		wait_delay--;

		// check if a dummy char is received to signal an update event.
		if (uart_driver_data_available() == 1u) {
			// Read and discard the dummy char received.
			(void)uart_driver_read(&dummy_byte);

			 res = 1u;
		}
	}

	return res;
}

/**
 * @brief This function erase the flash applicative area.
 */
static void flash_application_erase(void) {
	FLASH_EraseInitTypeDef erase_init_struct;
	uint32_t sector_error = 0u;

	// Prepare application flash erase.
	erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init_struct.PageAddress = APPLICATION_START_ADDRESS;

	// Compute number of pages to erase.
	erase_init_struct.NbPages = (FLASH_BANK1_END - APPLICATION_START_ADDRESS)
			/ FLASH_PAGE_SIZE;

	// Switch off the LED while erasing the flash.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	// Perform flash erase.
	if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
		Error_Handler();

	// Switch the LED back on once flash erase is done.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}
/**
 * @brief This function update the firmware by writing to flash the received data from UART.
 * TODO: Sanitize uart input and check for firmware version.
 */
static void firmware_update(void) {
	uint8_t byte_read = 0u;
	uint64_t data = 0u;
	uint32_t n_data = 0u;
	uart_driver_status_t uart_driver_status;
	uint32_t flash_addr = APPLICATION_START_ADDRESS;
	uint32_t rx_delay = 16000000u; // TODO: magic number, corresponding to approximately 30 secs.

	// Update until rx_delay is over.
	while (rx_delay != 0u) {
		uart_driver_status = uart_driver_read(&byte_read);

		if (uart_driver_status == UART_DRIVER_NO_DATA_AVAILABLE) {
			rx_delay--;
			continue;
		} else if (uart_driver_status == UART_DRIVER_ERROR) {
			return Error_Handler();
		} else if (uart_driver_status == UART_DRIVER_OK) {
			// Switch off the LED while flashing.
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

			// Update is not over, reset rx_delay.
			rx_delay = 100000u; // TODO: magic number

			data |= ((uint64_t)byte_read << ((n_data) * 8u));
			if (n_data++ == (sizeof(data) - 1u)) {
				n_data = 0u;
				// Write the firmware to FLASH.
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_addr, data) != HAL_OK) {
					Error_Handler();
				}

				// Increment address and reset data for next write.
				data = 0u;
				flash_addr += sizeof(data);
			}
		}
	}

	// Handle last doubleword write if it wasn't complete.
	if (n_data != 0u) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_addr, data) != HAL_OK) {
			Error_Handler();
		}
	}

	// Switch on the LED once flashing is done.
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int boot_main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	uart_driver_init();

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	// Initialize all configured peripherals.
	MX_GPIO_Init();

	/* USER CODE BEGIN 2 */

	// Update firmware if data available on UART.
	if (check_firmware_update() == 1u) {
		// Unlock flash.
		if (HAL_FLASH_Unlock() != HAL_OK) Error_Handler();

		// Erase applicative area and then write the firmware into flash.
		flash_application_erase();
		firmware_update();

		// Lock flash.
		if (HAL_FLASH_Lock() != HAL_OK) Error_Handler();
	}

	// Jump to application.
	app_branch();

	// Return statement never reached.
	return 0u;
}

/*
 * @brief Jump to main application.
 */
static void app_branch(void) {
	p_function app_entry;
 	uint32_t app_stack;

 	// Copy paste of system init function, as this will be bypassed by the direct branching to main. */

	// Set HSION bit.
	RCC->CR |= 0x00000001U;

	// Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits.
	RCC->CFGR &= 0xF8FF0000U;

	// Reset HSEON, CSSON and PLLON bits.
	RCC->CR &= 0xFEF6FFFFU;

	// Reset HSEBYP bit.
	RCC->CR &= 0xFFFBFFFFU;

	// Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits.
	RCC->CFGR &= 0xFF80FFFFU;

	// Disable all interrupts and clear pending bits.
	RCC->CIR = 0x009F0000U;

	// Get the application stack pointer (first entry in the application vector table).
	app_stack = (uint32_t) *((__IO uint32_t*)APPLICATION_START_ADDRESS);

	// Get the application entry point (second entry in the application vector table).
	app_entry = (p_function) *(__IO uint32_t*) (APPLICATION_START_ADDRESS + 4u);

	// Reconfigure vector table offset register to match the application location.
	SCB->VTOR = APPLICATION_START_ADDRESS;

	// Set the application stack pointer.
	__set_MSP(app_stack);

	// Start the application.
	app_entry();
}
/* USER CODE END 0 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
