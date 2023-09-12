/*
 * Released under the GPL License, Version 3
 */

#ifndef _CRUISE_CONTROL_H
#define _CRUISE_CONTROL_H

#include "main.h"
void cruise_control_init(void);
uint16_t cruise_control_throttle(uint16_t erps);

void enable_cruise_control(uint16_t erps);
void stop_cruise_control (void);
void cruise_control_update(void);
uint8_t cruise_control_enabled(void);


uint8_t cruise_control_regen(uint16_t erps);

// flags for debug from the cruise controller
extern uint16_t cruise_status_flags;
extern int16_t cruise_last_delta;

#endif /* _CRUISE_CONTROL_H */
