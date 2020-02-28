/*
 * custom_uart_driver.c
 *
 *  Created on: Feb 9, 2020
 *      Author: khbx
 */

#include "main.h"
#include "custom_uart_driver.h"

static uint8_t buf_rx[16];
static void (*rx_cplt_callback)(uint8_t *);
/**
* @brief Non generic, hardcoded and error prone uart init driver.
*/
void custom_uart_driver_init() {
	const uint32_t pclk2 = 72000000;
	const uint32_t baud_rate = 115200;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Init low level hardware resources.
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // Enable clock for GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable clock for USART1

	// TODO: remove HAL usage, init gpio following this link
	// http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/
    GPIO_InitStruct.Pin = GPIO_PIN_9;				// PA9 ------> USART1_TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;				// PA10 ------> USART1_RX
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TODO USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1 |= USART_CR1_UE;		// USART enabled.
	USART1->CR1 &= ~USART_CR1_M;		// 1 Start bit, 8 Data bits, n Stop bit.
	USART1->CR2 &= ~USART_CR2_STOP_0;	// 1 Stop bit.
	USART1->CR2 &= ~USART_CR2_STOP_1;
	USART1->BRR = pclk2/baud_rate;		// USART1 is clocked with PCLK2 (72MHz).
  	USART1->CR1 |= USART_CR1_TE;		// tx is enabled.
  	USART1->CR1 |= USART_CR1_RE;		// rx is enabled.
  	USART1->CR1 |= USART_CR1_RXNEIE;	// rx interrupt is enabled.
}

/**
* @brief Non generic, hardcoded and error prone uart tx driver.
*/
void custom_uart_driver_tx(uint8_t *p_data, uint8_t length) {

  	for (int i = 0; i < length; i++) {
  		// Wait for transmit data register to be empty.
  		while ((USART1->SR & USART_SR_TXE) != USART_SR_TXE);

  		// Write the data to send in the USART_DR register (this clears the TXE bit).
  		USART1->DR = (p_data[i] & 0xFF);
  	}

	// Wait until TC=1. This indicates that the transmission of the last frame is complete.
  	while ((USART1->SR & USART_SR_TC) != USART_SR_TC);
}

/**
* @brief Non generic, hardcoded and error prone uart rx driver.
*/
void custom_uart_driver_rx() {
	static uint8_t bytes_read = 0u;

	// Should not have to wait as we are called by the USART1_IRQHandler.
	while ((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE);

	buf_rx[bytes_read++] = (USART1->DR & 0xFF);

	// Call completion callback after reading 16 bytes.
	if (bytes_read == 15) {
		bytes_read = 0u;
		rx_cplt_callback(buf_rx);
	}
}

/**
* @brief Set the user read completion callback.
*/
void set_rx_cplt_callback(void (*callback)(uint8_t *)) {
	rx_cplt_callback = callback;
}
