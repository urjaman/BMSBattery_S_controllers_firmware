/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"

void gpio_init (void)
{
	// General MCU principles: every pin should have a defined state;
	// if not, the pin might float randomly and toggle spuriously and consume current in the input buffer.
	// Thus, set every unused pin to input pullup.
	//               LEARN        unk          unk          unk          unk
	GPIO_Init(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6,
				GPIO_MODE_IN_PU_NO_IT);
	//               unk (no connect)
	GPIO_Init(GPIOB, GPIO_PIN_3,
				GPIO_MODE_IN_PU_NO_IT);
	//               unk          unk
	GPIO_Init(GPIOC, GPIO_PIN_4 | GPIO_PIN_6,
				GPIO_MODE_IN_PU_NO_IT);
	//               SWIM         unk          unk
	GPIO_Init(GPIOD, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4,
				GPIO_MODE_IN_PU_NO_IT);
	//               unk          unk
	GPIO_Init(GPIOE, GPIO_PIN_3 | GPIO_PIN_5,
				GPIO_MODE_IN_PU_NO_IT);
	//               unk          unk
	GPIO_Init(GPIOG, GPIO_PIN_0 | GPIO_PIN_1,
				GPIO_MODE_IN_PU_NO_IT);
	// Note: on an unknown board, you should also check that every pulled up pin
	// reads back as 1 (which i did for mine). If something reads back as 0, assume
	// it's either used for something that drives it, or is hard-wired to ground,
	// and remove it from the pullup list.
}

void debug_pin_init (void)
{
  GPIO_Init(DEBUG__PORT,
	    DEBUG__PIN,
	    GPIO_MODE_OUT_PP_HIGH_FAST);
}

void debug_pin_set (void)
{
  GPIO_WriteHigh(DEBUG__PORT, DEBUG__PIN);
}

void debug_pin_reset (void)
{
  GPIO_WriteLow(DEBUG__PORT, DEBUG__PIN);
}

// added by DerBastler - Light
void light_pin_init (void)
{
  GPIO_Init(LIGHT__PORT,
	    LIGHT__PIN,
	    GPIO_MODE_OUT_PP_HIGH_FAST);
}
// added by DerBastler - Light
void light_pin_set (void)
{
  GPIO_WriteHigh(LIGHT__PORT, LIGHT__PIN);
}
// added by DerBastler - Light
void light_pin_reset (void)
{
  GPIO_WriteLow(LIGHT__PORT, LIGHT__PIN);
}


