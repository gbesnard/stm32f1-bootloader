/*
 * uart_driver.h
 *
 *  Created on: Feb 9, 2020
 *      Author: khbx
 */

#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include <stdint.h>

typedef enum uart_driver_status {
	UART_DRIVER_OK,
	UART_DRIVER_NO_DATA_AVAILABLE,
	UART_DRIVER_ERROR
}uart_driver_status_t ;

// Driver high level functions.
uart_driver_status_t uart_driver_init();
uart_driver_status_t uart_driver_read(uint8_t *byte);
uart_driver_status_t uart_driver_write(uint8_t *p_data, uint8_t length);
uint8_t uart_driver_data_available(void);

// Driver ISR function.
void uart_driver_rx_isr();

#endif /* __UART_DRIVER_H */
