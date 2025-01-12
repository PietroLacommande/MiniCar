#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "utils.h"
#include "autosarTypes.h"



//Pins used to command motor through LM298
#define input1 GPIO_NUM_18
#define input2 GPIO_NUM_19
#define input3 GPIO_NUM_32
#define input4 GPIO_NUM_33
#define enableA GPIO_NUM_25
#define enableB GPIO_NUM_26
#define OUTPUT_PINS ((1ULL << input1) | (1ULL << input2) |(1ULL << input3) | (1ULL << input4) | (1ULL << enableA) | (1ULL << enableB)) // Define pins as a bitmask


Std_ReturnType setUpWheelLogic(void);
void SpeedAndDirectionControl(int xAxis, int yAxis, int* motorSpeedA, int* motorSpeedB, bool* isReversed);
void initMotorPWM();
void set_motor_speed(int motorSpeedA, int motorSpeedB);

#endif //MOTOR_CONTROL_H