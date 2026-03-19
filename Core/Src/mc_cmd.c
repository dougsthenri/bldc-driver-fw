/*
 * mc_cmd.c
 *
 *  Created on: Feb 17, 2026
 *      Author: Douglas Almeida
 */

#include "mc_cmd.h"

#define RX_BUFFER_SIZE 256

#define SM_IS_DIGIT(X) ((X) >= '0' && (X) <= '9')

typedef enum {
	SM_IDLE = 0,
	SM_READ_M, SM_READ_M_COLON,
	SM_READ_D, SM_READ_D_COLON,
	SM_READ_R, SM_READ_R_COLON,
	SM_READ_S, SM_READ_S_D, SM_READ_S_D_COLON,
	SM_READ_ARG_DIGIT,
	SM_CMD_READY
} sm_state_t;

typedef void (*cmd_handler_t)(void);

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_usart_rx;

	uint8_t rx_buffer[RX_BUFFER_SIZE];		//Circular DMA receive buffer
	uint16_t rx_buffer_old_pos;				//Last processed position on the buffer
	volatile uint8_t data_received_flag;	//Flag to signal the command task

	/* Command parsing state machine */
	sm_state_t cmd_sm_state;
	cmd_handler_t sm_current_cmd;
	uint32_t sm_integer_arg;
} cmd_if_t;

cmd_if_t command_if = {0};

const uint8_t cmd_ack_reply[] = {'A', 'C', 'K', '\n'};
const uint8_t cmd_nak_reply[] = {'N', 'A', 'K', '\n'};

// "<StatusNumber> StepNumber:[Hall1 Hall2 Hall3]"
uint8_t status_report_reply[] = {'<', '0', '>', ' ', '0', ':', '[', '0', '0', '0', ']', '\n'};

static void parse_command(uint8_t);
void cmd_set_direction_cw(void);
void cmd_set_direction_ccw(void);
void cmd_set_duty_cycle(void);
void cmd_turn_motor_on(void);
void cmd_turn_motor_off(void);
void cmd_report_status(void);
void cmd_set_startup_dc(void);


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == command_if.huart->Instance) {
        // Signal the task that new data has been received
    	command_if.data_received_flag = 1;
    }
}


static void parse_command(uint8_t c) {
	switch (command_if.cmd_sm_state) {
	case SM_IDLE:
		if (c == 'm') {
			command_if.cmd_sm_state = SM_READ_M;
		} else if (c == 'd') {
			command_if.cmd_sm_state = SM_READ_D;
		} else if (c == 'r') {
			command_if.cmd_sm_state = SM_READ_R;
		} else if (c == 's') {
			command_if.cmd_sm_state = SM_READ_S;
		}
		break;
	case SM_READ_M:
		if (c == ':') {
			command_if.cmd_sm_state = SM_READ_M_COLON;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_M_COLON:
		if (c == '0') {
			command_if.sm_current_cmd = cmd_turn_motor_off;
			command_if.cmd_sm_state = SM_CMD_READY;
		} else if (c == '1') {
			command_if.sm_current_cmd = cmd_turn_motor_on;
			command_if.cmd_sm_state = SM_CMD_READY;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_D:
		if (c == ':') {
			command_if.cmd_sm_state = SM_READ_D_COLON;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_D_COLON:
		if (SM_IS_DIGIT(c)) {
			command_if.sm_current_cmd = cmd_set_duty_cycle;
			command_if.sm_integer_arg = c - '0';
			command_if.cmd_sm_state = SM_READ_ARG_DIGIT;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_R:
		if (c == ':') {
			command_if.cmd_sm_state = SM_READ_R_COLON;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_R_COLON:
		if (c == '+') {
			command_if.sm_current_cmd = cmd_set_direction_cw;
			command_if.cmd_sm_state = SM_CMD_READY;
		} else if (c == '-') {
			command_if.sm_current_cmd = cmd_set_direction_ccw;
			command_if.cmd_sm_state = SM_CMD_READY;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_S:
		if (c == 'd') {
			command_if.cmd_sm_state = SM_READ_S_D;
		} else if (c == 'r') {
			command_if.sm_current_cmd = cmd_report_status;
			command_if.cmd_sm_state = SM_CMD_READY;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_S_D:
		if (c == ':') {
			command_if.cmd_sm_state = SM_READ_S_D_COLON;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_S_D_COLON:
		if (SM_IS_DIGIT(c)) {
			command_if.sm_current_cmd = cmd_set_startup_dc;
			command_if.sm_integer_arg = c - '0';
			command_if.cmd_sm_state = SM_READ_ARG_DIGIT;
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_READ_ARG_DIGIT:
		if (c == '\n' || c == '\r') {
			command_if.sm_current_cmd();
			command_if.cmd_sm_state = SM_IDLE;
		} else if (SM_IS_DIGIT(c)) {
			command_if.sm_integer_arg = command_if.sm_integer_arg * 10 + (c - '0');
		} else {
			command_if.cmd_sm_state = SM_IDLE;
		}
		break;
	case SM_CMD_READY:
		if (c == '\n' || c == '\r') {
			// Execute current command
			command_if.sm_current_cmd();
		}
		command_if.cmd_sm_state = SM_IDLE;
		break;
	}
}


void cmd_set_direction_cw(void) {
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	mc_set_dir_cw();
}


void cmd_set_direction_ccw(void) {
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	mc_set_dir_ccw();
}


void cmd_set_duty_cycle(void) {
	if (command_if.sm_integer_arg > 1000) {
		// Reject command
		HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_nak_reply, sizeof cmd_nak_reply);

		return; //Value out of range
	}
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	uint16_t pulse_value = (MC_TIMER_ARR * command_if.sm_integer_arg) / 1000;
	mc_set_run_pulse_value(pulse_value);
}


void cmd_turn_motor_on(void) {
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	mc_start_motor();
}


void cmd_turn_motor_off(void) {
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	mc_stop_motor();
}


void cmd_report_status(void) {
	// Note status
	status_report_reply[1] = hmc.status + '0';
	// Note current step
	status_report_reply[4] = hmc.current_step + '0';
	// Note current hall status
	uint8_t hall_status = mc_get_hall_status();
	status_report_reply[7] = hall_status & 0x04 ? '1' : '0';
	status_report_reply[8] = hall_status & 0x02 ? '1' : '0';
	status_report_reply[9] = hall_status & 0x01 ? '1' : '0';

	// Reply to command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)status_report_reply, sizeof status_report_reply);
}


void cmd_set_startup_dc(void) {
	if (command_if.sm_integer_arg > 1000) {
		// Reject command
		HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_nak_reply, sizeof cmd_nak_reply);

		return; //Value out of range
	}
	// Confirm command
	HAL_UART_Transmit_IT(command_if.huart, (const uint8_t *)cmd_ack_reply, sizeof cmd_ack_reply);

	uint16_t pulse_value = (MC_TIMER_ARR * command_if.sm_integer_arg) / 1000;
	mc_set_startup_pulse_value(pulse_value);
}


mc_func_status_t mc_cmd_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx) {
	if ((huart == NULL) || (hdma_usart_rx == NULL)) {
		return MC_FUNC_FAIL;
	}
	command_if.huart = huart;
	command_if.hdma_usart_rx = hdma_usart_rx;
	command_if.cmd_sm_state = SM_IDLE;

	// Start DMA reception in Circular Mode with IDLE line interrupt for the USART
	HAL_UARTEx_ReceiveToIdle_DMA(command_if.huart, command_if.rx_buffer, RX_BUFFER_SIZE);

	return MC_FUNC_OK;
}


mc_func_status_t mc_cmd_task(void) {
	if (command_if.data_received_flag) {
		command_if.data_received_flag = 0;

		// Use the DMA counter to find exactly where the hardware is writing
		uint16_t current_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(command_if.hdma_usart_rx);

		while (command_if.rx_buffer_old_pos != current_pos) {
			parse_command(command_if.rx_buffer[command_if.rx_buffer_old_pos]);
			command_if.rx_buffer_old_pos++;
			if (command_if.rx_buffer_old_pos >= RX_BUFFER_SIZE) {
				command_if.rx_buffer_old_pos = 0;
			}
		}
	}

	return MC_FUNC_OK;
}
