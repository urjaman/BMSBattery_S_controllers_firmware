/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho,Björn Schmidt 2015, 2106, 2017, 2019
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s_iwdg.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "gpio.h"
#include "pwm.h"
#include "config.h"
#include "adc.h"

#include "ACAcontrollerState.h"
#include "ACAcommons.h"


/* Local only */
static uint8_t ui8_half_rotation_flag = 0;
static uint8_t ui8_foc_enable_flag = 0;
static uint8_t ui8_assumed_motor_position = 0;
static uint8_t ui8_motor_rotor_hall_position = 0; // in 360/256 degrees
static uint8_t ui8_sinetable_precalc = 0;
static uint8_t ui8_interpolation_start_position = 0;
static uint8_t ui8_interpolation_angle = 0;
static int8_t hall_sensors_last = 0;
static int8_t hall_sensors_last_raw = 0;
static uint16_t ui16_ADC_iq_current_accumulated = 4096;

// Local except for diagnostics (main)
uint16_t ui16_PWM_cycles_counter = 0;
uint16_t ui16_PWM_cycles_counter_6 = 0;
uint16_t ui16_PWM_cycles_counter_total = 0;
uint16_t ui16_ADC_iq_current = 0; // main plus BO
int8_t hall_sensors; // main only; plus it's an int8, should be fine.
uint8_t ui8_possible_motor_state = 0;
uint8_t ui8_dynamic_motor_state = 0;
uint8_t ui8_position_correction_value = 127; // in 360/256 degrees
// Only for diagnostics
uint8_t uint8_t_60deg_pwm_cycles[6];
uint8_t uint8_t_hall_case[7];


// Motor->PWM (we call pwm; same context.)
uint8_t ui8_sinetable_position = 0; // in 360/256 degrees

// Slow loop -> motor (by the motor_slow_update_post)
static uint16_t BatteryCurrent;

// Motor-> slow loop (_pre)
static uint16_t motor_speed_erps;

// Direct from cruise_control.c
volatile uint8_t motor_direction_reverse;

#define HALL_REMAP(x) ((x&1) | ((x&2)<<1) | ((x&4) >> 1))

void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER) {
	adc_trigger();
	hall_sensors_read_and_action();

	motor_fast_loop();

	// clear the interrupt pending bit for TIM1
	TIM1_ClearITPendingBit(TIM1_IT_UPDATE);
}

void hall_sensor_init(void) {

	GPIO_Init(HALL_SENSORS__PORT,
			(GPIO_Pin_TypeDef) (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN),
			GPIO_MODE_IN_FL_NO_IT);
}

void hall_sensors_read_and_action(void) {
	// read hall sensors signal pins and mask other pins
	int8_t hs = (GPIO_ReadInputData(HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));
	if ((hs != hall_sensors_last_raw) || (ui8_possible_motor_state == MOTOR_STATE_COAST)) // let's run the code when motor is stopped/coast so it can pick right motor position for correct startup
	{
		hall_sensors_last_raw = hs;
		hall_sensors = HALL_REMAP(hs);

		if (hall_sensors_last >0 && hall_sensors_last < 7) {
			uint8_t_60deg_pwm_cycles[hall_sensors_last-1] = ui16_PWM_cycles_counter_6;
		}
		updateHallOrder(hall_sensors); // this stores the hall order elsewhere, only for debug

		hall_sensors_last = hall_sensors;

		//printf("hall change! %d, %d \n", hall_sensors, hall_sensors_last );

		if (ui8_possible_motor_state == MOTOR_STATE_COAST) {
			ui8_possible_motor_state = MOTOR_STATE_RUNNING_NO_INTERPOLATION;
		}

		switch (hall_sensors) {
			case 3://rotor position 180 degree
				// full electric revolution recognized, update counters
				uint8_t_hall_case[3] = ui8_adc_read_phase_B_current();


				if (ui8_half_rotation_flag) {
					ui8_half_rotation_flag = 0;
					if (ui16_PWM_cycles_counter > 20) ui16_PWM_cycles_counter_total = ui16_PWM_cycles_counter;

					ui16_PWM_cycles_counter = 0;
					motor_speed_erps = ((uint16_t) ui16_pwm_cycles_second) / ui16_PWM_cycles_counter_total; // this division takes ~4.2us

				}

				if (motor_speed_erps == -1) {
					motor_speed_erps = 0;
				}
				// update motor state based on motor speed
				if (motor_speed_erps > 75) {
					ui8_possible_motor_state = MOTOR_STATE_RUNNING_INTERPOLATION_360;
				}else if (motor_speed_erps > 3) {
					ui8_possible_motor_state = MOTOR_STATE_RUNNING_INTERPOLATION_60;
				} else {
					ui8_possible_motor_state = MOTOR_STATE_RUNNING_NO_INTERPOLATION;
				}

				ui8_motor_rotor_hall_position = ui8_s_hall_angle3_180;
				break;

			case 1:
				uint8_t_hall_case[4] = ui8_adc_read_phase_B_current();
				ui8_motor_rotor_hall_position = ui8_s_hall_angle1_240;
				break;

			case 5: //rotor position 300 degree

				uint8_t_hall_case[5] = ui8_adc_read_phase_B_current();
				ui8_motor_rotor_hall_position = ui8_s_hall_angle5_300;
				break;

			case 4: //rotor position 0 degree
				ui8_half_rotation_flag = 1;
				ui8_foc_enable_flag = 1;

				uint8_t_hall_case[0] = ui8_adc_read_phase_B_current();
 				//debug_pin_reset();
				ui8_motor_rotor_hall_position = ui8_s_hall_angle4_0;
				break;

			case 6://rotor position 60 degree
				uint8_t_hall_case[1] = ui8_adc_read_phase_B_current();
				ui8_motor_rotor_hall_position = ui8_s_hall_angle6_60;
				break;

			case 2://rotor position 120 degree
				uint8_t_hall_case[2] = ui8_adc_read_phase_B_current();
				ui8_motor_rotor_hall_position = ui8_s_hall_angle2_120;
				break;

			default:
				return;
				break;
		}

		ui16_PWM_cycles_counter_6 = 0;
		if (motor_direction_reverse) {
		//	ui8_possible_motor_state = MOTOR_STATE_RUNNING_NO_INTERPOLATION;
			ui8_motor_rotor_hall_position -= 100;
		}

	}
}

//#define USE_FIELD_WEAKENING

/* Slow loop -> ISR communication for field weakening */
#ifdef USE_FIELD_WEAKENING
static volatile uint8_t curr_target_ctrl = 126;
static volatile uint8_t max_angle_ctrl = 143;
#endif

static void updateCorrection(uint8_t reverse) {

	if (ui8_duty_cycle_target > 5) {
		ui16_ADC_iq_current_accumulated -= ui16_ADC_iq_current_accumulated >> 3;
		ui16_ADC_iq_current_accumulated += ui16_adc_read_phase_B_current();
		ui16_ADC_iq_current = ui16_ADC_iq_current_accumulated >> 3; // this value is regualted to be zero by FOC
	}

	if ((ui16_aca_flags & ANGLE_CORRECTION_ENABLED) != ANGLE_CORRECTION_ENABLED) {
		ui8_position_correction_value = 127; //set advance angle to neutral value
		return;
	}
#if 0
	if (reverse) {
		ui8_position_correction_value = 127; // neutral
		return;
	}
#endif

#ifdef USE_FIELD_WEAKENING
	uint8_t curr_target = curr_target_ctrl;
	uint8_t max_angle = max_angle_ctrl;
#else
	const uint8_t curr_target = 126;
	const uint8_t max_angle = 143;
#endif

	if (motor_speed_erps > 3 && BatteryCurrent > ui16_current_cal_b + 3) { //normal riding,
		if (ui16_ADC_iq_current >> 2 > (curr_target+2) && ui8_position_correction_value < max_angle) {
			ui8_position_correction_value++;
		} else if (ui16_ADC_iq_current >> 2 < (curr_target) && ui8_position_correction_value > 111) {
			ui8_position_correction_value--;
		}
	} else if (motor_speed_erps > 3 && BatteryCurrent < ui16_current_cal_b - 3) {//regen
		ui8_position_correction_value = 127; //set advance angle to neutral value
	} else if (motor_speed_erps < 3) {
		ui8_position_correction_value = 127; //reset advance angle at very low speed)
	}

}

// runs every 64us (PWM frequency)

void motor_fast_loop(void) {


	/* FIXME: These counters are... not well implemented. */
	if (ui16_time_ticks_for_uart_timeout < 65530) {
		ui16_time_ticks_for_uart_timeout++;
	}
	if (ui16_time_ticks_for_speed_calculation < 65530) {
		ui16_time_ticks_for_speed_calculation++;
	} //increase SPEED Counter but avoid overflow
	if (ui16_time_ticks_for_pas_calculation < 65530) {
		ui16_time_ticks_for_pas_calculation++;
	} //increase PAS Counter but avoid overflow
	if (GPIO_ReadInputPin(PAS__PORT, PAS__PIN) && ui16_PAS_High_Counter < 65530) {
		ui16_PAS_High_Counter++;
	}
	uint8_t reverse = motor_direction_reverse;

	// count number of fast loops / PWM cycles
	if (ui16_PWM_cycles_counter >= PWM_CYCLES_COUNTER_MAX) {
		//ui16_PWM_cycles_counter = 0;
		//ui16_PWM_cycles_counter_6 = 0;
		ui16_PWM_cycles_counter_total = 0xffff; //(SVM_TABLE_LEN_x1024) / PWM_CYCLES_COUNTER_MAX;
		ui8_position_correction_value = 127;
		hall_sensors_last = 0;
		motor_speed_erps = 0;


		// next code is need for motor startup correctly
		ui8_possible_motor_state = MOTOR_STATE_COAST;
		hall_sensors_read_and_action();
	}


	//  // calculate the interpolation angle
	//  // interpolation seems a problem when motor starts, so avoid to do it at very low speed
	if (((ui8_possible_motor_state == MOTOR_STATE_RUNNING_INTERPOLATION_60)||(ui8_possible_motor_state == MOTOR_STATE_RUNNING_INTERPOLATION_360)) && ((ui16_aca_experimental_flags & DISABLE_INTERPOLATION) != DISABLE_INTERPOLATION)) {

		if (
				((ui16_aca_experimental_flags & DISABLE_60_DEG_INTERPOLATION) == DISABLE_60_DEG_INTERPOLATION)||
				(((ui16_aca_experimental_flags & SWITCH_360_DEG_INTERPOLATION) == SWITCH_360_DEG_INTERPOLATION) && (ui8_possible_motor_state == MOTOR_STATE_RUNNING_INTERPOLATION_360))
				){

			if (ui16_PWM_cycles_counter>255){
				ui8_interpolation_angle = (ui16_PWM_cycles_counter <<5) / (ui16_PWM_cycles_counter_total>>3);
			}else{
				ui8_interpolation_angle = (ui16_PWM_cycles_counter <<8) / (ui16_PWM_cycles_counter_total);
			}
			ui8_interpolation_start_position = ui8_s_hall_angle3_180; // that's where ui16_PWM_cycles_counter is being reset
			ui8_dynamic_motor_state = MOTOR_STATE_RUNNING_INTERPOLATION_360;
		}else{

			if (ui16_PWM_cycles_counter_6>255){
				ui8_interpolation_angle = (ui16_PWM_cycles_counter_6 <<5) / (ui16_PWM_cycles_counter_total>>3);
			}else{
				ui8_interpolation_angle = (ui16_PWM_cycles_counter_6 << 8) / ui16_PWM_cycles_counter_total;
			}
			ui8_interpolation_start_position = ui8_motor_rotor_hall_position;
			ui8_dynamic_motor_state = MOTOR_STATE_RUNNING_INTERPOLATION_60;
		}

		ui16_PWM_cycles_counter_6++;
	}else {// MOTOR_STATE_COAST || MOTOR_STATE_RUNNING_NO_INTERPOLATION

		ui8_interpolation_angle = 0;

		ui8_interpolation_start_position = ui8_motor_rotor_hall_position;
		ui8_dynamic_motor_state = MOTOR_STATE_RUNNING_NO_INTERPOLATION;

	}
	ui16_PWM_cycles_counter++;
	if (reverse) {
		ui8_sinetable_precalc = ui8_interpolation_start_position + ui8_s_motor_angle + ui8_position_correction_value -127 - ui8_interpolation_angle;
	} else {
		ui8_sinetable_precalc = ui8_interpolation_start_position + ui8_s_motor_angle + ui8_position_correction_value -127 + ui8_interpolation_angle;
	}

	if ((ui16_aca_experimental_flags & AVOID_MOTOR_CYCLES_JITTER) != AVOID_MOTOR_CYCLES_JITTER){
		ui8_sinetable_position = ui8_sinetable_precalc;
	}else{
		// if we would go back slightly, stay at current position; if we would go back a lot, still apply (startup/coast/corner cases)
		if (((uint8_t)(ui8_sinetable_precalc-ui8_sinetable_position))<248){
			ui8_sinetable_position = ui8_sinetable_precalc;

		}else{
			// debug anti jitter occurrences and angles
			ui8_variableDebugB += 1;
			ui8_variableDebugC = ui8_interpolation_angle;
			ui8_variableDebugA = ui8_motor_rotor_hall_position;
		}
	}

	//ui8_assumed_motor_position = ui8_interpolation_start_position + ui8_interpolation_angle + ui8_s_motor_angle + ui8_position_correction_value - 127;
	if (reverse) {
		ui8_assumed_motor_position = ui8_interpolation_start_position + ui8_s_motor_angle - ui8_interpolation_angle;
	} else {
		ui8_assumed_motor_position = ui8_interpolation_start_position + ui8_interpolation_angle + ui8_s_motor_angle;
	}

	// check if FOC control is needed
	if ((ui8_foc_enable_flag) && ((ui8_assumed_motor_position) >= (ui8_correction_at_angle)) && ((ui8_assumed_motor_position) < (ui8_correction_at_angle + 4))) {
		// make sure we just execute one time per ERPS, so reset the flag
		ui8_foc_enable_flag = 0;

		//ui8_variableDebugA = ui8_assumed_motor_position;
		//ui8_variableDebugB = ui8_assumed_motor_position + ui8_position_correction_value - 127;

		updateCorrection(reverse);
	}


	//reset watchdog
	IWDG->KR = IWDG_KEY_REFRESH;
	pwm_duty_cycle_controller();
}

void watchdog_init(void) {
	IWDG_Enable();
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_4);

	//  Timeout period
	//  The timeout period can be configured through the IWDG_PR and IWDG_RLR registers. It
	//  is determined by the following equation:
	//  T = 2 * T LSI * P * R
	//  where:
	//  T = Timeout period
	//  T LSI = 1/f LSI
	//  P = 2 (PR[2:0] + 2)
	//  R = RLR[7:0]+1
	//
	//  0.0001 = 2 * (1 / 128000) * 4 * R
	//  R = 1.6 ; rounding to R = 2
	//  R = 2 means a value of reload register = 1
	IWDG_SetReload(2); // 187.5us; for some reason, a value of 1 don't work, only 2
	IWDG_ReloadCounter();
}


// Move these includes here if you want to see the identifiers that the ISR code uses
// without much regard for safety (most of whats left (except those timers) are effectively constants (except if adjusted during debug).
//#include "ACAcontrollerState.h"
//#include "ACAcommons.h"


// The concept here is loaned from linux; except that SDCC doesnt do typeof(x), so we need one per width.
#define READ_ONCE_U16(x)	(*(const volatile uint16_t *)&(x))
#define WRITE_ONCE_U16(x, val)						\
do {									\
	*(volatile uint16_t *)&(x) = (val);				\
} while (0)

/* Communicate between "fast loop" (ISR) and rest of the system. */

void motor_slow_update_pre(void) {
	disableInterrupts();
	ui16_motor_speed_erps = READ_ONCE_U16(motor_speed_erps);
	enableInterrupts();
}

void motor_slow_update_post(void) {
#ifdef USE_FIELD_WEAKENING
#define FIELD_WEAK_MIN_SPEED 90
#define FIELD_WEAK_MAX_CURR 40
#define FIELD_WEAK_MAX_ANGLE 15
	const uint8_t default_curr_target = 126;
	const uint8_t max_angle_def = 143;
	uint8_t curr_target = curr_target_ctrl;
	uint8_t max_angle = max_angle_ctrl;
	if (ui16_motor_speed_erps > FIELD_WEAK_MIN_SPEED) {
		static uint8_t fweak_counter = 0;
		if (fweak_counter++ >= 5) {
			fweak_counter = 0;
			if ( (ui16_BatteryCurrent < (uint32_current_target-10)) && (ui16_setpoint > 250) &&
				 (curr_target > (default_curr_target-FIELD_WEAK_MAX_CURR)) ) {
					curr_target -= 1;
					max_angle = max_angle_def + FIELD_WEAK_MAX_ANGLE;
			} else if ((ui16_BatteryCurrent >= (uint32_current_target-2)) && (curr_target < default_curr_target)) {
					curr_target += 1;
			}
		}
	} else {
		curr_target = default_curr_target;
		max_angle = max_angle_def;
	}
	// No harm should happen if the ISR happens between these writes, so no need to disableInterrupts() yet.
	curr_target_ctrl = curr_target;
	max_angle_ctrl = max_angle;
#endif

	disableInterrupts();
	WRITE_ONCE_U16(BatteryCurrent, ui16_BatteryCurrent);
	enableInterrupts();
}
