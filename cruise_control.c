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
	int16_t ccthr = erp_delta * 4; // 0.25 (p, and then pi)

	ccthr_i += (erp_delta * 2); // 0.125
	if (ccthr_i > 0x3FFF) {
		//printf("ih\r\n");
		ccthr_i = 0x3FFF;
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
	if (ccthr > 0x3FFF) {
		//printf("rH\r\n");
		ccthr = 0x3FFF;
	}
	return ccthr >> 4;
}

void enable_cruise_control(uint16_t erps)
{
	cruise_control_speed = erps;
	ccthr_i = ui16_throttle_accumulated; // extra bits match with our 4 fract
}

void stop_cruise_control (void)
{
	cruise_control_speed = 0;
	ccthr_i = 0;
}

void cruise_control_update(void)
{
	if (brake_is_set()) {
		stop_cruise_control();
	} else {
		  if (GPIO_ReadInputPin(CRUISE_PORT, CRUISE_PIN) == 0) {
			  enable_cruise_control(ui32_erps_filtered);
		  }
	}
}
