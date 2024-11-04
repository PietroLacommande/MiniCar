//
// Created by Pietr on 2024-11-03.
//

#ifndef UNTITLED6_MOTORCONTROL_H
#define UNTITLED6_MOTORCONTROL_H

#define input1 18
#define input2 19
#define input3 32
#define input4 33
#define enableA 25
#define enableB 26



void setUpWheelLogic(int pin1, int pin2, int pin3, int pin4, int pinA,  int pinB);


void setUpWheelLogic(int pin1, int pin2, int pin3, int pin4);



void SpeedAndDirectionControl(int xAxis, int yAxis, int* motorSpeedA, int* motorSpeedB);

#endif //UNTITLED6_MOTORCONTROL_H
