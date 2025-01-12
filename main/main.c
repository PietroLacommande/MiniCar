#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motorControl.h"
#include "encoderManager.h"
#include "adcManager.h"
#include "autosarTypes.h"

bool isReversed= false;
adc_continuous_handle_t adcHandle1;

//main function
void app_main(void)
{
    int xAxis=0;
    int yAxis=0;
    int motorSpeedA=0;
    int motorSpeedB=0;

    
    adcInit(&adcHandle1);
    adcStart(&adcHandle1);
    setUpWheelLogic();
    initMotorPWM();
    initPCNT();
    initTimer();
    

    while(1){
        adc_read_task(&xAxis, &yAxis,&adcHandle1);
        SpeedAndDirectionControl(xAxis,yAxis,&motorSpeedA,&motorSpeedB,&isReversed);
        set_motor_speed(motorSpeedA, motorSpeedB);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

