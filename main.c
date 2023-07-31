/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "main.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "timers.h"
#include "pwm.h"
#include "PAS.h"
#include "SPEED.h"
#include "cruise_control.h"
#include "ACAsetPoint.h"
#include "config.h"
#include "display.h"
#include "display_kingmeter.h"
#include "ACAcontrollerState.h"
#include "BOdisplay.h"
#include "ACAeeprom.h"
#include "ACAcommons.h"

//uint16_t ui16_LPF_angle_adjust = 0;
//uint16_t ui16_LPF_angle_adjust_temp = 0;

uint16_t ui16_log1 = 0;
uint8_t ui8_slowloop_flag = 0;
uint8_t ui8_veryslowloop_counter = 0;
uint8_t ui8_ultraslowloop_counter = 0;
uint16_t ui16_log2 = 0;
uint8_t ui8_log = 0;
uint8_t ui8_i = 0; //counter for ... next loop

float float_kv = 0;
float float_R = 0;
uint8_t a = 0; //loop counter

static int16_t i16_deziAmps;


/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main(void);

//With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
//in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
//to place the function prototype in a header file as long as the header file is included in the file that
//contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
//not be in place so the ISR will not be executed when the interrupt occurs.

//Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
//all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// and the solution was to avoid using local variables and define them as global instead

// Brake signal interrupt
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);
// Speed signal interrupt
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
// PAS signal interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// Timer2/slow control loop
void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 receivce handler
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
volatile int32_t test_value = -15000;

extern int8_t hall_sensors;
#define F_SETPT 0

int main(void) {
#if F_SETPT
	int16_t f_setpt = 0;
#endif
	//set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

	gpio_init();
	brake_init();
	while (brake_is_set()); // hold here while brake is pressed -- this is a protection for development
	debug_pin_init();
	light_pin_init();
	timer2_init();
	uart_init();
	eeprom_init();
	controllerstate_init();
	initErpsRatio();
	pwm_init();
	hall_sensor_init();
	adc_init();
	PAS_init();
	SPEED_init();
	aca_setpoint_init();
	cruise_control_init();
#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC
	display_init();
#endif

	//  ITC_SetSoftwarePriority (ITC_IRQ_TIM1_OVF, ITC_PRIORITYLEVEL_2);

	enableInterrupts();

	watchdog_init(); //init watchdog after enabling interrupt to have fast loop running already

#if (SVM_TABLE == SVM)
	TIM1_SetCompare1(126 << 1);
	TIM1_SetCompare2(126 << 1);
	TIM1_SetCompare3(126 << 1);
#elif (SVM_TABLE == SINE) || (SVM_TABLE == SINE_SVM)
	TIM1_SetCompare1(126 << 2);
	TIM1_SetCompare2(126 << 2);
	TIM1_SetCompare3(126 << 2);
#endif

	hall_sensors_read_and_action(); // needed to start the motor
	//printf("Back in Main.c\n");

	for (a = 0; a < NUMBER_OF_PAS_MAGS; a++) {// array init
		ui16_torque[a] = 0;
	}
#ifdef DIAGNOSTICS
	printf("System initialized\r\n");
#endif

	int32_t test_result = test_value * 100;

#if 0
	printf("test_result = %04X trh=%04X\n", (uint16_t)test_result, (uint16_t)(((uint32_t)test_result) >> 16));

		printf("%03d %03d %03d %03d %03d %03d\r\n",
			ui8_s_hall_angle4_0, ui8_s_hall_angle6_60, ui8_s_hall_angle2_120,
			ui8_s_hall_angle3_180, ui8_s_hall_angle1_240, ui8_s_hall_angle5_300);
#endif

	while (1) {

		uart_send_if_avail();

		updateSpeeds();
		updatePasStatus();

#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC
		display_update();
#endif




		// scheduled update of setpoint and duty cycle (slow loop, 50 Hz)
		if (ui8_slowloop_flag) {
			//printf("MainSlowLoop\n");
		    debug_pin_set();
			ui8_slowloop_flag = 0; //reset flag for slow loop
			ui8_veryslowloop_counter++; // increase counter for very slow loop

			motor_slow_update_pre();
			checkPasInActivity();
			updateRequestedTorque(); //now calculates tq for sensor as well
			updateSlowLoopStates();
			updateX4();
			updateLight();
			cruise_control_update();

			ui16_setpoint = (uint16_t) aca_setpoint(ui16_time_ticks_between_pas_interrupt, ui16_setpoint); //update setpoint
#if F_SETPT
			if (ui16_setpoint < f_setpt) ui16_setpoint = f_setpt;
#endif
			pwm_set_duty_cycle((uint8_t) ui16_setpoint);
			motor_slow_update_post();

			//pwm_set_duty_cycle ((uint8_t)ui16_sum_throttle);

			/****************************************************************************/
			//very slow loop for communication
			if (ui8_veryslowloop_counter > 5) {
				int ch = uart_getch();
				static uint8_t last_hallL = 0;
				static uint8_t hall_changes = 0;
				ui8_ultraslowloop_counter++;
				ui8_veryslowloop_counter = 0;

				if (ui8_ultraslowloop_counter > 10) {
					ui8_ultraslowloop_counter = 0;
					ui8_uptime++;
#ifdef DIAGNOSTICS

#if 1
				printf("C%04X BC%03d/%03d "
				//"PAS%u-%05u "
				"S%u SP%03d "
				"MA%03d "
				"PCV%d "
				"IQ%04d"
				//"B%u"
				"\r\n",
				ui16_control_state,
				(int)(ui16_BatteryCurrent - ui16_current_cal_b),
				(int)(uint32_current_target - ui16_current_cal_b),
//				PAS_is_active, ui16_time_ticks_between_pas_interrupt,
				ui16_motor_speed_erps,
				ui16_setpoint,
				ui8_s_motor_angle,
				ui8_position_correction_value,
				ui16_ADC_iq_current
				//ui8_BatteryVoltage
				);
#endif

#endif
				}

#ifdef DIAGNOSTICS
				//uint32_torquesensorCalibration=80;

#if 0
				// "timeout" is the PAS timeout. someone rename that define ASAP.
				if (ui16_time_ticks_between_pas_interrupt < timeout)
					printf("%04d / %04d\r\n", ui16_PAS_High, ui16_time_ticks_between_pas_interrupt);
#endif

#if 0
				if ((ui16_motor_speed_erps >= 60) && (ui16_motor_speed_erps < 110) && (!brake_is_set())) {
					printf("HCYC: %03d %03d %03d %03d %03d %03d\r\n",
					uint8_t_60deg_pwm_cycles[4-1],
					uint8_t_60deg_pwm_cycles[6-1],
					uint8_t_60deg_pwm_cycles[2-1],
					uint8_t_60deg_pwm_cycles[3-1],
					uint8_t_60deg_pwm_cycles[1-1],
					uint8_t_60deg_pwm_cycles[5-1]
					);
				}
#endif

				if (ch >= 0) {
				//	printf("CH:%02X\r\n", ch);
					if (ch == 'x') ui8_s_motor_angle++;
					if (ch == 'z') ui8_s_motor_angle--;
#if 0
					if (ch == 'a') f_setpt--;
					if (ch == 's') f_setpt += 5;
					if (ch == 'q') f_setpt = 0;
					if ((ch == 'w')&&(f_setpt)) f_setpt = 64;
					if (ch == 'c') ui16_aca_experimental_flags &= ~(USE_ALTERNATE_WAVETABLE|USE_ALTERNATE_WAVETABLE_B);
					if (ch == 'v') {
						ui16_aca_experimental_flags &= ~(USE_ALTERNATE_WAVETABLE|USE_ALTERNATE_WAVETABLE_B);
						ui16_aca_experimental_flags |= USE_ALTERNATE_WAVETABLE; // pure sine
					}
					if (ch == 'b') {
						ui16_aca_experimental_flags &= ~(USE_ALTERNATE_WAVETABLE|USE_ALTERNATE_WAVETABLE_B);
						ui16_aca_experimental_flags |= USE_ALTERNATE_WAVETABLE_B; // third harmonic
					}
					if (ch == 'n') {
						ui16_aca_experimental_flags |= USE_ALTERNATE_WAVETABLE | USE_ALTERNATE_WAVETABLE_B; // nip tuck
					}

#endif

#if 0
					if (ch == '8') ui8_s_motor_angle = (MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT-85) & 0xFF;
					if (ch == '9') ui8_s_motor_angle = MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT;
					if (ch == '0') ui8_s_motor_angle = (MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT+85) & 0xFF;
					if (ch == 'p') pwm_swap_phases_ac ^= 1;
#endif
				}


				if ((hall_sensors&1) != last_hallL) {
					hall_changes++;
					last_hallL = hall_sensors&1;
				}
				//printf("erps %d, motorstate %d, cyclecountertotal %d\r\n", ui16_motor_speed_erps, ui8_possible_motor_state|ui8_dynamic_motor_state, ui16_PWM_cycles_counter_total);

				//printf("cheatstate, %d, km/h %lu, Voltage, %d, setpoint %d, erps %d, current %d, correction_value, %d\n", ui8_offroad_state, ui32_speed_sensor_rpks, ui8_BatteryVoltage, ui16_setpoint, ui16_motor_speed_erps, ui16_BatteryCurrent, ui8_position_correction_value);

				//printf("kv %d, erps %d, R %d\n", (uint16_t)(float_kv*10.0) , ui16_motor_speed_erps, (uint16_t)(float_R*1000.0));

				/*for(a = 0; a < 6; a++) {			// sum up array content
						 putchar(uint8_t_hall_case[a]);
						 }
				putchar(ui16_ADC_iq_current>>2);
				putchar(ui8_position_correction_value);
				putchar(255);*/
				// printf("%u, %d, %d, %d, %d, %d, %d\r\n", ui8_position_correction_value, (uint16_t) uint8_t_hall_case[0], (uint16_t)uint8_t_hall_case[1],(uint16_t) uint8_t_hall_case[2],(uint16_t) uint8_t_hall_case[3], (uint16_t)uint8_t_hall_case[4], (uint16_t)uint8_t_hall_case[5]);
	//			 printf("\r%03d %03d %03d %03d %03d %03d ", (uint16_t) uint8_t_hall_case[0], (uint16_t)uint8_t_hall_case[1],(uint16_t) uint8_t_hall_case[2],(uint16_t) uint8_t_hall_case[3], (uint16_t)uint8_t_hall_case[4], (uint16_t)uint8_t_hall_case[5]);

		//		printf(" P%d, BC%d, SP%d, S%d, IQ%d, BV%d, MA%d, SAC%d, H%d",
		//		ui8_position_correction_value, ui16_BatteryCurrent, ui16_setpoint, ui16_motor_speed_erps, ui16_ADC_iq_current>>2,ui16_adc_read_battery_voltage(), ui8_s_motor_angle,
		//		pwm_swap_phases_ac, hall_sensors);

#if 0
				// GPIO debug, for finding unknown buttons (or anything else where you're curious about pin states)
				printf("\r%02X %02X %02X %02X %02X %02X",
					GPIOA->IDR, GPIOB->IDR, GPIOC->IDR, GPIOD->IDR, GPIOE->IDR, GPIOG->IDR );
#endif

				//printf("correction angle %d, Current %d, Voltage %d, sumtorque %d, setpoint %d, km/h %lu\n",ui8_position_correction_value, i16_deziAmps, ui8_BatteryVoltage, ui16_sum_throttle, ui16_setpoint, ui32_speed_sensor_rpks);
#endif
			}//end of very slow loop

			debug_pin_reset();
		}// end of slow loop
		wfi();
	}// end of while(1) loop
}


