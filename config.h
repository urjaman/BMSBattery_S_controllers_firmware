/*
 * config.h
 *
 *  Automatically created by OSEC Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define NUMBER_OF_PAS_MAGS 8
#define limit 50
#define timeout 3125
#define wheel_circumference 1055L
#define limit_without_pas 50
#define ADC_THROTTLE_MIN_VALUE 72
#define ADC_THROTTLE_MAX_VALUE 172
#define BATTERY_VOLTAGE_MIN_VALUE 111
#define BATTERY_CURRENT_MAX_VALUE 85L
#define PHASE_CURRENT_MAX_VALUE 180L
#define REGEN_CURRENT_MAX_VALUE 10L
#define MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT 200
#define current_cal_a 100
#define LEVEL_1 50
#define LEVEL_2 62
#define LEVEL_3 74
#define LEVEL_4 87
#define LEVEL_5 100
#define MORSE_TIME_1 50
#define MORSE_TIME_2 50
#define MORSE_TIME_3 50
#define RAMP_END 1500
#define P_FACTOR 0.8
#define I_FACTOR 0.2
#define GEAR_RATIO 15L
#define PAS_THRESHOLD 1.9
#define RAMP_START 64000
#define limit_with_throttle_override 80
#define CORRECTION_AT_ANGLE 127
#define DIAGNOSTICS
#define ANGLE_4_0 1
#define ANGLE_6_60 43
#define ANGLE_2_120 86
#define ANGLE_3_180 128
#define ANGLE_1_240 171
#define ANGLE_5_300 213
#define TQS_CALIB 0.0
#define ACA 20682
#define EEPROM_INIT_MAGIC_BYTE 205 // makes sure (chance of fail 1/255) eeprom is invalidated after flashing new config
#define ADC_BATTERY_VOLTAGE_K 69
#define ACA_EXPERIMENTAL 1152
#define BATTERY_VOLTAGE_MAX_VALUE 156

#endif /* CONFIG_H_ */
