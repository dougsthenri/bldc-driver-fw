/*
 * mc_core.h
 *
 *  Created on: Feb 17, 2026
 *      Author: Douglas Almeida
 */

#ifndef INC_MC_CORE_H_
#define INC_MC_CORE_H_

#include "main.h"

// Function status
typedef enum {
	MC_FUNC_OK = ((uint8_t) 0),
	MC_FUNC_FAIL
} mc_func_status_t;

typedef enum {
	MC_ROTATION_CW = ((uint8_t) 0),
	MC_ROTATION_CCW
} mc_dir_rot_t;

typedef enum {
  MC_INIT = ((uint8_t) 0),
  MC_STOP,			//Motor stopped
  MC_STARTUP,		//Motor is starting
  MC_RUN,			//Motor is running
  MC_OVERCURRENT	//Fault condition to be cleared by command
} mc_status_t;

typedef struct {
	volatile mc_status_t status; //Motor status

	TIM_HandleTypeDef *hall_if_timer;
	TIM_HandleTypeDef *mc_timer;

	mc_dir_rot_t direction;					//Defaults to clockwise
	volatile uint16_t startup_pulse_value;	//Value set from command
	volatile uint16_t run_pulse_value;		//Value set from command
	volatile uint16_t pulse_value;			//Current value, defaults to no power output
	volatile uint8_t current_step;
	volatile uint8_t preloaded_step;

	// Timed tasks
	volatile uint8_t is_alarm_task_en; //Visual indication for alarm state
	volatile uint32_t alarm_task_start_tick;
} mc_handle_t;

extern mc_handle_t hmc;

mc_func_status_t mc_core_init(TIM_HandleTypeDef *, TIM_HandleTypeDef *);
mc_func_status_t mc_set_dir_cw(void);
mc_func_status_t mc_set_dir_ccw(void);
mc_func_status_t mc_set_run_pulse_value(uint16_t);
mc_func_status_t mc_start_motor(void);
mc_func_status_t mc_stop_motor(void);
mc_func_status_t mc_set_startup_pulse_value(uint16_t);
uint8_t mc_get_hall_status(void);
void mc_alarm_task(void);

#endif /* INC_MC_CORE_H_ */
