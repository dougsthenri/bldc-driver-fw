/*
 * mc_cmd.h
 *
 *  Created on: Feb 17, 2026
 *      Author: Douglas Almeida
 *
 *
 * DRIVER COMMANDS
 *
 * ss:N
 * Select a slave device, of numerical address N, as target for commands sent on the bus.
 *
 * r:D
 * Set direction of rotation, where D is:
 * ---------------------------
 * D		| Direction
 * ---------------------------
 * +		| Clockwise
 * -		| Counterclockwise
 * ---------------------------
 *
 * d:N
 * Set PWM duty cycle, where N is a value in tenths of percentage between 0 and 1000.
 *
 * m:S
 * Switch the motor on/off, where S is:
 * ----------------
 * S		| State
 * ----------------
 * 0		| Off
 * 1		| On
 * ----------------
 *
 * sd:N
 * Set startup PWM duty cycle, where N is a value in tenths of percentage between 0 and 1000.
 *
 * sr
 * Reply with status report: Current step number and hall sensor readings.
 *
 * Every command and response must be terminated by a newline character.
 * Valid commands are answered with the "ACK" response and illegal ones with "NAK". Unrecognised commands are ignored.
 */

#ifndef INC_MC_CMD_H_
#define INC_MC_CMD_H_

#include "main.h"
#include "mc_core.h"

mc_func_status_t mc_cmd_init(UART_HandleTypeDef *, DMA_HandleTypeDef *);
mc_func_status_t mc_cmd_task(void);

#endif /* INC_MC_CMD_H_ */
