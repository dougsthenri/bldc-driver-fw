/*
 * mc_core.c
 *
 *  Created on: Feb 17, 2026
 *      Author: Douglas Almeida
 */

#include "mc_core.h"
#include "main.h"

void hall_status_to_current_step(void);
void preload_configuration_for_step(uint8_t);
void preload_next_step(void);
void set_pulse_for_current_step(uint16_t);

mc_handle_t hmc = {0};


void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
	mc_stop_motor();
	hmc.status = MC_OVERCURRENT;
	// Set up visual alarm
	HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);
	hmc.alarm_task_start_tick = HAL_GetTick();
	hmc.is_alarm_task_en = 1;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == hmc.hall_if_timer->Instance) {
		// Motor movement detected
		hmc.status = MC_RUN;
		hmc.pulse_value = hmc.run_pulse_value; //Set PWM pulse for impending commutation
		hall_status_to_current_step(); //Update current step

		//TODO: Probe speed: Every time a Hall sensor toggles, the current timer value is captured into CCR1
//		uint32_t movement_period = (hmc.hall_if_timer)->Instance->CCR1;
		//...
	}
}


void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == hmc.mc_timer->Instance) {
		hmc.preloaded_step = 0;
		preload_next_step();
	}
}


uint8_t mc_get_hall_status(void) {
	return HAL_GPIO_ReadPin(Hall1_GPIO_Port, Hall1_Pin) << 2 |\
	  	   HAL_GPIO_ReadPin(Hall2_GPIO_Port, Hall2_Pin) << 1 |\
		   HAL_GPIO_ReadPin(Hall3_GPIO_Port, Hall3_Pin); //Bit field [H1,H2,H3]
}


void mc_alarm_task(void) {
	if (hmc.is_alarm_task_en) {
		// Toggle status LED every 500 ms
		if (HAL_GetTick() - hmc.alarm_task_start_tick >= 500) {
			hmc.alarm_task_start_tick = HAL_GetTick(); //Recurring task
			HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
		}
	}
}


void hall_status_to_current_step(void) {
	uint8_t hall_status = mc_get_hall_status();
	switch (hall_status) {
		case 0x1: //001
			hmc.current_step = 1;
			break;
		case 0x3: //011
			hmc.current_step = 2;
			break;
		case 0x2: //010
			hmc.current_step = 3;
			break;
		case 0x6: //110
			hmc.current_step = 4;
			break;
		case 0x4: //100
			hmc.current_step = 5;
			break;
		case 0x5: //101
			hmc.current_step = 6;
			break;
		default: //Invalid hall state
			Error_Handler();
	}
}


void preload_configuration_for_step(uint8_t step_number) {
	TIM_TypeDef *timer = hmc.mc_timer->Instance;
	switch (step_number) {
		case 1: //[H1,H2,H3] = 001
			// U+ V-
			WRITE_REG(timer->CCR1, hmc.pulse_value);
			WRITE_REG(timer->CCR2, 0);
			SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1NE);
			break;
		case 2: //[H1,H2,H3] = 011
			// U+ W-
			WRITE_REG(timer->CCR1, hmc.pulse_value);
			WRITE_REG(timer->CCR3, 0);
			SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC3NE);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1NE);
			break;
		case 3: //[H1,H2,H3] = 010
			// V+ W-
			WRITE_REG(timer->CCR2, hmc.pulse_value);
			WRITE_REG(timer->CCR3, 0);
			SET_BIT(timer->CCER, TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC3NE);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2NE);
			break;
		case 4: //[H1,H2,H3] = 110
			// V+ U-
			WRITE_REG(timer->CCR1, 0);
			WRITE_REG(timer->CCR2, hmc.pulse_value);
			SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2NE);
			break;
		case 5: //[H1,H2,H3] = 100
			// W+ U-
			WRITE_REG(timer->CCR1, 0);
			WRITE_REG(timer->CCR3, hmc.pulse_value);
			SET_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3NE);
			break;
		case 6: //[H1,H2,H3] = 101
			// W+ V-
			WRITE_REG(timer->CCR2, 0);
			WRITE_REG(timer->CCR3, hmc.pulse_value);
			SET_BIT(timer->CCER, TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E);
			CLEAR_BIT(timer->CCER, TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3NE);
			break;
		default:
			break;
	}
}


void preload_next_step(void) {
	if (hmc.direction == MC_ROTATION_CW) {
		hmc.preloaded_step = hmc.current_step == 6 ? 1 : hmc.current_step + 1;
	} else {
		hmc.preloaded_step = hmc.current_step == 1 ? 6 : hmc.current_step - 1;
	}
	preload_configuration_for_step(hmc.preloaded_step);
}


void set_pulse_for_current_step(uint16_t pulse_value) {
	uint32_t volatile* ptr = &(hmc.mc_timer->Instance->CCR1);
	ptr += ((hmc.current_step - 1) >> 1);
	*ptr = pulse_value;
}


mc_func_status_t mc_core_init(TIM_HandleTypeDef *hall_if_timer, TIM_HandleTypeDef *mc_timer) {
	if ((hall_if_timer == NULL) || (mc_timer == NULL)) {
		return MC_FUNC_FAIL;
	}
	hmc.hall_if_timer = hall_if_timer;
	hmc.mc_timer = mc_timer;
	hall_status_to_current_step();

	// Configure the hardware synchronization bits (CCPC and CCUS) for the Slave Timer (TIM1)
	HAL_TIMEx_ConfigCommutEvent_IT(hmc.mc_timer, TIM_TS_ITR1, TIM_COMMUTATION_TRGI);

	hmc.status = MC_STOP;

	return MC_FUNC_OK;
}


mc_func_status_t mc_set_dir_cw(void) {
	//TODO: Stop motor first, if running
	hmc.direction = MC_ROTATION_CW;

	return MC_FUNC_OK;
}


mc_func_status_t mc_set_dir_ccw(void) {
	//TODO: Stop motor first, if running
	hmc.direction = MC_ROTATION_CCW;

	return MC_FUNC_OK;
}


mc_func_status_t mc_set_startup_pulse_value(uint16_t puse_value) {
	hmc.startup_pulse_value = puse_value;

	return MC_FUNC_OK;
}


mc_func_status_t mc_set_run_pulse_value(uint16_t puse_value) {
	hmc.run_pulse_value = puse_value;
	if (hmc.status == MC_RUN) {
		__disable_irq();
		set_pulse_for_current_step(hmc.run_pulse_value);
		__enable_irq();
	}

	return MC_FUNC_OK;
}


mc_func_status_t mc_start_motor(void) {
	if (hmc.status == MC_STOP) {
		// Signal motor on
		HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);

		hmc.status = MC_STARTUP;
		hall_status_to_current_step();

		// Enable the driver
		HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
		HAL_Delay(2); //Short delay to allow the driver to stabilize

		__disable_irq();
		hmc.pulse_value = hmc.startup_pulse_value;

		// Start the Hall Interface Timer (TIM2) in Hall Sensor Mode with Interrupts enabled
		HAL_TIMEx_HallSensor_Start_IT(hmc.hall_if_timer);
		//TODO: Enable the Update Interrupt for detecting motor stall. Don't forget to clear it too when stopping
//		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

		// Start the Advanced Motor Control Timer (TIM1)
		HAL_TIM_Base_Start(hmc.mc_timer);
		__HAL_TIM_ENABLE_IT(hmc.mc_timer, TIM_IT_BREAK | TIM_IT_COM);
		__HAL_TIM_MOE_ENABLE(hmc.mc_timer); //Enable the Main Output (MOE) bit

		// Prevent a commutation interrupt from firing prematurely by clearing pending interrupt flags
		__HAL_TIM_CLEAR_IT(hmc.hall_if_timer, TIM_IT_CC1/* | TIM_IT_UPDATE*/);
		__HAL_TIM_CLEAR_IT(hmc.mc_timer, TIM_IT_BREAK | TIM_IT_COM);
		__enable_irq();

		preload_next_step();
		SET_BIT(TIM1->EGR, TIM_EGR_COMG); //Generate a commutation event
	}

	return MC_FUNC_OK;
}


mc_func_status_t mc_stop_motor(void) {
	__disable_irq();
	// Stop the Master Timer (TIM2)
	HAL_TIMEx_HallSensor_Stop_IT(hmc.hall_if_timer);
	__HAL_TIM_CLEAR_IT(hmc.hall_if_timer, TIM_IT_CC1/* | TIM_IT_UPDATE*/);
	__HAL_TIM_SET_COUNTER(hmc.hall_if_timer, 0);

	// Stop the Advanced Motor Control Timer (TIM1)
	HAL_TIM_Base_Stop(hmc.mc_timer);
	__HAL_TIM_MOE_DISABLE(hmc.mc_timer); //Disable the Main Output (MOE) bit
	__HAL_TIM_DISABLE_IT(hmc.mc_timer, TIM_IT_BREAK | TIM_IT_COM);
	__HAL_TIM_CLEAR_IT(hmc.mc_timer, TIM_IT_BREAK | TIM_IT_COM);
	__HAL_TIM_SET_COUNTER(hmc.mc_timer, 0);

	set_pulse_for_current_step(0);

	// Disable the driver
	HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

	hmc.status = MC_STOP;
	__enable_irq();

	// Signal motor off
	HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

	return MC_FUNC_OK;
}
