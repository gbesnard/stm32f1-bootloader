/*
 * uart_driver.c
 *
 *  Created on: Feb 9, 2020
 *      Author: khbx
 */

#include "main.h"
#include "uart_driver.h"

#define BUFFER_RX_SIZE  256u

typedef enum buffer_status {
	BUFFER_OK,
	BUFFER_EMPTY,
	BUFFER_FULL,
	BUFFER_ERROR
}buffer_status_t ;

typedef struct buffer_rx {
    uint8_t data[BUFFER_RX_SIZE];
    uint8_t write_index;
    uint8_t read_index;
} buffer_rx_t;

static volatile buffer_rx_t buff_rx;
static volatile uint8_t error_rx = 0u; // No error recovery is possible.

static buffer_status_t uart_driver_buffer_rx_write(uint8_t byte);
static buffer_status_t uart_driver_buffer_rx_read (uint8_t *byte);

/**
* @brief Driver low level initialization.
*/
uart_driver_status_t uart_driver_init() {
	const uint32_t pclk2 = 72000000;
	const uint32_t baud_rate = 115200;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uart_driver_status_t uart_driver_status;

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

  	// TODO: handles error cases.
	uart_driver_status = UART_DRIVER_OK;

	return uart_driver_status;
}

/**
* @brief Write bytes to uart.
*/
uart_driver_status_t uart_driver_write(uint8_t *p_data, uint8_t length) {
	uart_driver_status_t uart_driver_status;

	// Loop through all data to be transmitted.
  	for (int i = 0; i < length; i++) {
  		// Wait for transmit data register to be empty.
  		while ((USART1->SR & USART_SR_TXE) != USART_SR_TXE);

  		// Write the data to send in the USART_DR register (this clears the TXE bit).
  		USART1->DR = (p_data[i] & 0xFF);
  	}

	// Wait until TC=1. This indicates that the transmission of the last frame is complete.
  	while ((USART1->SR & USART_SR_TC) != USART_SR_TC);

  	// TODO: handles error cases.
	uart_driver_status = UART_DRIVER_OK;

	return uart_driver_status;
}

/**
* @brief Read one byte from uart if available.
* TODO: read more than one byte at a time.
*/
uart_driver_status_t uart_driver_read(uint8_t *byte) {
	buffer_status_t buf_status;
	uart_driver_status_t uart_driver_status;

	// Read byte from circular buffer if available.
	buf_status = uart_driver_buffer_rx_read (byte);

	if(buf_status == BUFFER_EMPTY) {
		uart_driver_status = UART_DRIVER_NO_DATA_AVAILABLE ;
	}
	else if (buf_status == BUFFER_OK) {
		uart_driver_status = UART_DRIVER_OK;
	}
	else {
		uart_driver_status = UART_DRIVER_ERROR;
	}

	return uart_driver_status;
}

/**
* @brief Receive data Interrupt Service Routine.
*/
void uart_driver_rx_isr() {
	buffer_status_t buff_status;
	uint8_t byte_rx;

	// Should not have to wait as we are called by the USART1_IRQHandler.
	while ((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE);
	byte_rx = (USART1->DR & 0xFF);

	// Write byte into rx circular buffer.
	buff_status = uart_driver_buffer_rx_write(byte_rx);
	if (buff_status == BUFFER_FULL)
	{
		error_rx = 1u;
	}
}

/**
 *  @brief Write one byte into rx circular buffer.
 */
static buffer_status_t uart_driver_buffer_rx_write(uint8_t byte) {
	uint8_t next_index = 0u;

	next_index = (((buff_rx.write_index) + 1) % BUFFER_RX_SIZE);

	if (next_index == buff_rx.read_index) {
		return BUFFER_FULL;
	}

	buff_rx.data[buff_rx.write_index] = byte;
	buff_rx.write_index = next_index;
	return BUFFER_OK;
}

/**
 *  @brief Read one byte from rx circular buffer.
 */
static buffer_status_t uart_driver_buffer_rx_read (uint8_t *byte) {
	if (error_rx == 1u) {
		return BUFFER_ERROR;
	}

	if (buff_rx.write_index == buff_rx.read_index) {
		return BUFFER_EMPTY;
	}

	*byte = buff_rx.data[buff_rx.read_index];
	buff_rx.read_index = ((buff_rx.read_index + 1) % BUFFER_RX_SIZE);
	return BUFFER_OK;
}
