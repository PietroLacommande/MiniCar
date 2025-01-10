#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

//Encoder defines
#define encoderHoles 20
#define wheelCircumference 0.22



void initPCNT();
void onTimer(bool* direction);
void initTimer();

#endif