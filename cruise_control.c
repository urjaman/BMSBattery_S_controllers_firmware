/*
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "ACAcommons.h"
#include "ACAcontrollerState.h"
#include "cruise_control.h"
#include "brake.h"
#include "gpio.h"
#include "motor.h"

#define USE_CRUISE_REVERSE

static uint16_t cruise_control_speed = 0;

void cruise_control_init(void) {
	; // we used to have something to init ...
}

static int16_t ccthr_i;

uint8_t cruise_control_enabled(void) {
	return !!cruise_control_speed;
}

uint16_t cruise_control_throttle(uint16_t erps) {
	/* CC Off? Quick exit. */
	if (!cruise_control_speed) {
		ccthr_i = 0;
		return 0;
	}

	int16_t erp_delta = cruise_control_speed - erps;
	// bits: 2 sign, overflow; 10 output, 4 internal fraction.
	int16_t ccthr = erp_delta * 5; // 0.25 (p, and then pi)

	ccthr_i += (erp_delta * 3); // 0.125
	if (ccthr_i > 0x3FC0) {
		//printf("ih\r\n");
		ccthr_i = 0x3FC0;
	}
	if (ccthr_i < 0) {
//		printf("iL\r\n");
		ccthr_i = 0;
	}

	ccthr += ccthr_i;
	if (ccthr < 0) {
//		printf("rL\r\n");
		ccthr = 0;
	}
	if (ccthr > 0x3FC0) {
		//printf("rH\r\n");
		ccthr = 0x3FC0;
	}
	return ccthr >> 4;
}

void enable_cruise_control(uint16_t erps)
{
	cruise_control_speed = erps;
	if (!erps) {
		ccthr_i = 0;
		return;
	}
	if (ui16_BatteryCurrent < ui16_current_cal_b) {
		ccthr_i = 0;
	} else {
		ccthr_i = ((ui16_BatteryCurrent - ui16_current_cal_b) * 0x3FC0UL) / ui16_battery_current_max_value;
	}
}

void stop_cruise_control (void)
{
	cruise_control_speed = 0;
	ccthr_i = 0;
}

static uint8_t cruise_btn_state = 0;
static uint8_t cruise_btn_ctr = 0;
static uint8_t brake_state = 0;
void cruise_control_update(void)
{
	uint8_t cruise_pressed = 0;
	uint8_t cruise_button = !GPIO_ReadInputPin(CRUISE_PORT, CRUISE_PIN);
	if ((!cruise_btn_state) && (cruise_button)) {
		cruise_pressed = 1;
		cruise_btn_state = 1;
		cruise_btn_ctr++;
	}
	if ((cruise_btn_state) && (!cruise_button)) {
		cruise_btn_state = 0;
	}

	if (brake_is_set()) {
		stop_cruise_control();
		if (!brake_state) cruise_btn_ctr = 0;
		brake_state = 1;
#ifdef USE_CRUISE_REVERSE
		if ((0 == ui16_virtual_erps_speed)&&(motor_direction_reverse)) {
			motor_direction_reverse = 0;
		}
#endif
	} else {
#ifdef USE_CRUISE_REVERSE
		if ( (0 == ui16_virtual_erps_speed) && (brake_state) &&
			(!cruise_btn_state) && (cruise_btn_ctr == 2) &&
			(ui16_momentary_throttle) ) {
			motor_direction_reverse = 1;
		}
		// dont enable cruise while reversing...
		if (motor_direction_reverse) {
			cruise_pressed = 0;
		}
#endif
		brake_state = 0;
		if (cruise_pressed) enable_cruise_control(ui16_virtual_erps_speed);
	}

}
