/*
 * Released under the GPL License, Version 3
 */

#ifndef _CRUISE_CONTROL_H
#define _CRUISE_CONTROL_H

#include "main.h"
void cruise_control_init(void);
uint16_t cruise_control_throttle(uint16_t erps, uint16_t ext_throttle);

void enable_cruise_control(uint16_t erps);
void stop_cruise_control (void);
void cruise_control_update(void);

#endif /* _CRUISE_CONTROL_H */
