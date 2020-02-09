/*
 * test.h
 *
 *  Created on: Feb 9, 2020
 *      Author: khbx
 */

#ifndef CUSTOM_INC_CUSTOM_UART_DRIVER_H_
#define CUSTOM_INC_CUSTOM_UART_DRIVER_H_

/**
* @brief Non generic, hardcoded and error prone uart init driver.
*/
void custom_uart_driver_init();

/**
* @brief Non generic, hardcoded and error prone uart tx driver.
*/
void custom_uart_driver_tx(uint8_t *p_data, uint8_t length);

/**
* @brief Non generic, hardcoded and error prone uart rx driver.
*/
void custom_uart_driver_rx();

/**
* @brief Set the user read completion callback.
*/
void set_rx_cplt_callback(void (*callback)(uint8_t *));

#endif /* CUSTOM_INC_CUSTOM_UART_DRIVER_H_ */
