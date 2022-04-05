/**************************************************************************
 *
 * Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Module Description:
 * Handle the configuration and operation of the RS485 bus.
 **************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "hardware.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/bits.h"
#include "bacnet/basic/sys/fifo.h"
#include "led.h"
#include "rs485.h"
#include "main.h"
#include "usart.h"

/* buffer for storing received bytes - size must be power of two */
static uint8_t Receive_Buffer_Data[512];
static FIFO_BUFFER Receive_Buffer;
/* amount of silence on the wire */
static struct mstimer Silence_Timer;
/* baud rate */
static uint32_t Baud_Rate = 38400;
uint8_t rx_byte;
/* The minimum time after the end of the stop bit of the final octet of a */
/* received frame before a node may enable its EIA-485 driver: 40 bit times. */
/* At 9600 baud, 40 bit times would be about 4.166 milliseconds */
/* At 19200 baud, 40 bit times would be about 2.083 milliseconds */
/* At 38400 baud, 40 bit times would be about 1.041 milliseconds */
/* At 57600 baud, 40 bit times would be about 0.694 milliseconds */
/* At 76800 baud, 40 bit times would be about 0.520 milliseconds */
/* At 115200 baud, 40 bit times would be about 0.347 milliseconds */
/* 40 bits is 4 octets including a start and stop bit with each octet */
#define Tturnaround (40UL)

/*************************************************************************
 * Description: Reset the silence on the wire timer.
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_silence_reset(void) {
	mstimer_set(&Silence_Timer, 0);
}

/*************************************************************************
 * Description: Determine the amount of silence on the wire from the timer.
 * Returns: true if the amount of time has elapsed
 * Notes: none
 **************************************************************************/
bool rs485_silence_elapsed(uint32_t interval) {
	return (mstimer_elapsed(&Silence_Timer) > interval);
}

/*************************************************************************
 * Description: Baud rate determines turnaround time.
 * Returns: amount of milliseconds
 * Notes: none
 **************************************************************************/
static uint16_t rs485_turnaround_time(void) {
	/* delay after reception before transmitting - per MS/TP spec */
	/* wait a minimum  40 bit times since reception */
	/* at least 2 ms for errors: rounding, clock tick */
	if (Baud_Rate) {
		return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
	} else {
		return 2;
	}
}

/*************************************************************************
 * Description: Use the silence timer to determine turnaround time.
 * Returns: true if turnaround time has expired.
 * Notes: none
 **************************************************************************/
bool rs485_turnaround_elapsed(void) {
	return (mstimer_elapsed(&Silence_Timer) > rs485_turnaround_time());
}

/*************************************************************************
 * Description: Determines if an error occured while receiving
 * Returns: true an error occurred.
 * Notes: none
 **************************************************************************/
bool rs485_receive_error(void) {
	return false;
}

/*********************************************************************//**
 * @brief
 *USARTx
 *interrupt
 *handler
 *sub-routine
 * @param[in]
 *None
 * @return
 *None
 **********************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_BUS) {
		/* Read one byte from the receive data register */
		(void) FIFO_Put(&Receive_Buffer, rx_byte);
		HAL_UART_Receive_IT(huart, &rx_byte, 1);
	}
}

/*************************************************************************
 * DESCRIPTION: Return true if a byte is available
 * RETURN:      true if a byte is available, with the byte in the parameter
 * NOTES:       none
 **************************************************************************/
bool rs485_byte_available(uint8_t *data_register) {
	bool data_available = false; /* return value */

	if (!FIFO_Empty(&Receive_Buffer)) {
		if (data_register) {
			*data_register = FIFO_Get(&Receive_Buffer);
		}
		rs485_silence_reset();
		data_available = true;
		led_rx_on_interval(10);
	}

	return data_available;
}

/*************************************************************************
 * DESCRIPTION: Sends a byte of data
 * RETURN:      nothing
 * NOTES:       none
 **************************************************************************/
void rs485_byte_send(uint8_t tx_byte) {
	led_tx_on_interval(10);
	HAL_UART_Transmit(&UART_BUS, tx_byte, 1, 500);
	rs485_silence_reset();
}

/*************************************************************************
 * Description: Determines if a byte in the USART has been shifted from
 *   register
 * Returns: true if the USART register is empty
 * Notes: none
 **************************************************************************/
bool rs485_byte_sent(void) {
	return HAL_UART_GetState(&UART_BUS) != HAL_UART_STATE_BUSY_TX;
}

/*************************************************************************
 * Description: Determines if the entire frame is sent from USART FIFO
 * Returns: true if the USART FIFO is empty
 * Notes: none
 **************************************************************************/
bool rs485_frame_sent(void) {
	return HAL_UART_GetState(&UART_BUS) != HAL_UART_STATE_BUSY_RX;
}

/*************************************************************************
 * DESCRIPTION: Send some data and wait until it is sent
 * RETURN:      true if a collision or timeout occurred
 * NOTES:       none
 **************************************************************************/
void rs485_bytes_send(uint8_t *buffer, /* data to send */
uint16_t nbytes) { /* number of bytes of data */
	uint8_t tx_byte;

	while (nbytes) {
		/* Send the data byte */
		tx_byte = *buffer;
		/* Send one byte */
		HAL_UART_Transmit(&UART_BUS, tx_byte, 1, 100);
		while (!rs485_byte_sent()) {
			/* do nothing - wait until Tx buffer is empty */
		}
		buffer++;
		nbytes--;
	}
	/* was the frame sent? */
	while (!rs485_frame_sent()) {
		/* do nothing - wait until the entire frame in the
		 Transmit Shift Register has been shifted out */
	}
	rs485_silence_reset();

	return;
}

/*************************************************************************
 * Description: Configures the baud rate of the USART
 * Returns: nothing
 * Notes: none
 **************************************************************************/
static void rs485_baud_rate_configure(void) {
	//UART_BUS.Instance = USART2;
	UART_BUS.Init.BaudRate = Baud_Rate;
	UART_BUS.Init.WordLength = UART_WORDLENGTH_8B;
	UART_BUS.Init.StopBits = UART_STOPBITS_1;
	UART_BUS.Init.Parity = UART_PARITY_NONE;
	UART_BUS.Init.Mode = UART_MODE_TX_RX;
	UART_BUS.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART_BUS.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&UART_BUS) != HAL_OK) {
		Error_Handler();
	}
}

/*************************************************************************
 * Description: Sets the baud rate to non-volatile storeage and configures USART
 * Returns: true if a value baud rate was saved
 * Notes: none
 **************************************************************************/
bool rs485_baud_rate_set(uint32_t baud) {
	bool valid = true;

	switch (baud) {
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 76800:
	case 115200:
		Baud_Rate = baud;
		rs485_baud_rate_configure();
		break;
	default:
		valid = false;
		break;
	}

	return valid;
}

/*************************************************************************
 * Description: Determines the baud rate in bps
 * Returns: baud rate in bps
 * Notes: none
 **************************************************************************/
uint32_t rs485_baud_rate(void) {
	return Baud_Rate;
}

/*************************************************************************
 * Description: Enable the Request To Send (RTS) aka Transmit Enable pin
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_rts_enable(bool enable) {
	if (enable) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
}

/*************************************************************************
 * Description: Initialize the room network USART
 * Returns: nothing
 * Notes: none
 **************************************************************************/
void rs485_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
/* Configure the Request To Send (RTS) aka Transmit Enable pin */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	rs485_baud_rate_set(Baud_Rate);

	FIFO_Init(&Receive_Buffer, &Receive_Buffer_Data[0],
			(unsigned) sizeof(Receive_Buffer_Data));
	rs485_silence_reset();
}
