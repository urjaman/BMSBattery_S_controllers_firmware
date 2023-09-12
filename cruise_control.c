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
static int16_t cc_domain_spd;

uint16_t cruise_status_flags = 0;
int16_t cruise_last_delta;

void cruise_control_init(void) {

	// TODO: Transform the codebase from /100 to /256 (or another simple shift...)
	cc_domain_spd = ((ui16_speed_kph_to_erps_ratio * 3) + 50) / 100;

}

static int16_t ccthr_i;

uint8_t cruise_control_enabled(void) {
	return !!cruise_control_speed;
}

uint8_t cruise_control_regen(uint16_t erps) {
	uint8_t PAS = PAS_is_active && (ui16_time_ticks_for_pas_calculation < timeout);
	if (!cruise_control_speed) return 0;
	if ((ui16_momentary_throttle) || (PAS)) return 0;
	int16_t extra_speed = erps - (cruise_control_speed + cc_domain_spd);
	if (extra_speed < 0) return 0;
	if (extra_speed > cc_domain_spd) return 100;
	return (extra_speed * 100) / cc_domain_spd;
}

uint16_t cruise_control_throttle(uint16_t erps) {
	const int16_t max_val = 0x3FC0;
	/* CC Off? Quick exit. */
	cruise_status_flags = 0;
	if (!cruise_control_speed) {
		ccthr_i = 0;
		return 0;
	}

	cruise_status_flags |= 1;

	int16_t erp_delta = cruise_control_speed - erps;
	cruise_last_delta = erp_delta;

	// hard clamp underspeed
	if (erp_delta > cc_domain_spd) {
		cruise_status_flags |= 2;
		ccthr_i = max_val;
		return max_val >> 4;
	}

	// hard clamp overspeed
	if (erp_delta < -cc_domain_spd) {
		cruise_status_flags |= 4;
		ccthr_i = 0;
		return 0;
	}

	// PI controller
	// bits: 2 sign, overflow; 10 output, 4 internal fraction.
	int16_t ccthr = erp_delta * 256; // /16 (p, and then pi)

	ccthr_i += (erp_delta * 10); // /16

	// overflow limiting
	if (ccthr_i > max_val) {
		//printf("ih\r\n");
		ccthr_i = max_val;
		cruise_status_flags |= 0x10;

	}
	if (ccthr_i < 0) {
//		printf("iL\r\n");
		ccthr_i = 0;
		cruise_status_flags |= 0x20;
	}
	// p+i
	ccthr += ccthr_i;
	if (ccthr < 0) {
//		printf("rL\r\n");
		ccthr = 0;
		cruise_status_flags |= 0x40;
	}
	if (ccthr > max_val) {
		//printf("rH\r\n");
		ccthr = max_val;
		cruise_status_flags |= 0x80;
	}
#if 1
	// soft clamp going too slow
	if (erp_delta > (cc_domain_spd/2)) {
		int32_t excess = erp_delta - cc_domain_spd/2;
		int16_t minthr = (excess * max_val) / (cc_domain_spd/2);
		cruise_status_flags |= 0x0100;
		if (ccthr < minthr) {
			ccthr_i = minthr;
			ccthr = minthr;
			cruise_status_flags |= 0x0200;
		}
	}
	// soft clamp going too fast
	if (erp_delta < -(cc_domain_spd/2)) {
		int32_t excess = erp_delta - (-(cc_domain_spd/2));
		int16_t maxthr = max_val - ((excess * max_val) / (-(cc_domain_spd/2)));
		cruise_status_flags |= 0x1000;
		if (ccthr > maxthr) {
			ccthr_i = maxthr;
			ccthr = maxthr;
			cruise_status_flags |= 0x2000;
		}
	}
#endif

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
