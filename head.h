#ifndef HEAD_H
#define HEAD_H

#include "algorithm.h"

//DRIVE MOTOR PIN CONFIGURATION
#define DC_PWM_PIN    7
#define DC_DIR_PIN    5
#define DC_SPEED_PWM  35

//STEERING MOTOR PIN CONFIGURATION
#define STEER_PWM_PIN    6
#define STEER_DIR_PIN    4
#define STEER_SPEED_PWM  120

#define FRONT         0
#define FRONT_DIR     HIGH
#define REAR          1
#define REAR_DIR      LOW
#define LEFT          0
#define LEFT_DIR      LOW
#define MID           2
#define RIGHT         1
#define RIGHT_DIR    HIGH
#define STOP          2
#define SCALE         1.2
#define Delay_time_ms    650

//INIT SENSOR PINS
#define S0_TRIG     (40)
#define S0_ECHO     (38)
#define S1_TRIG     (36)
#define S1_ECHO     (34)
#define S2_TRIG     (32)
#define S2_ECHO     (30)
#define S3_TRIG     (28)
#define S3_ECHO     (26)
//#define S4_TRIG     (1)
//#define S4_ECHO     (1)
#define S5_TRIG     (24)
#define S5_ECHO     (22)

#endif
