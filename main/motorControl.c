#include "motorControl.h"

//Structure configuration for motor
gpio_config_t o_configMotor = {
    .pin_bit_mask= OUTPUT_PINS,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en= GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type =GPIO_INTR_DISABLE
};


//Function definitions
Std_ReturnType setUpWheelLogic(){
    Std_ReturnType returnVal = E_NOT_OK;
    esp_err_t err= gpio_config(&o_configMotor);
    if(err!=ESP_OK){
        return returnVal;
    }
    returnVal= E_OK;
    return returnVal;
}
void SpeedAndDirectionControl(int xAxis, int yAxis, int* motorSpeedA, int* motorSpeedB, bool* isReversed) {
    if (yAxis < 1350) {
        // Set Motor A backward
        gpio_set_level(input1, HIGH);
        gpio_set_level(input2, LOW);
        // Set Motor B backward
        gpio_set_level(input3, HIGH);
        gpio_set_level(input4, LOW);
        // Convert the declining Y-axis readings for going backward from 470
        *motorSpeedA = map(yAxis, 1500, 0, 80, 255);
        *motorSpeedB = map(yAxis, 1500, 0, 80, 255);
        *isReversed = true;
    } else if (yAxis > 2150) {
        // Set Motor A forward
        gpio_set_level(input1, LOW);
        gpio_set_level(input2, HIGH);
        // Set Motor B forward
        gpio_set_level(input3, LOW);
        gpio_set_level(input4, HIGH);
        // Convert the declining Y-axis readings for going backward from 470
        *motorSpeedA = map(yAxis, 2000, 4095, 80, 255);
        *motorSpeedB = map(yAxis, 2000, 4095, 80, 255);
        *isReversed = false;
    } else {
        *motorSpeedA = 0;
        *motorSpeedB = 0;
        *isReversed = false;
    }


    // X-axis used for left and right control
    if (xAxis < 1500) {
        // Convert the declining X-axis readings from 470 to 0 into increasing values
        int xMapped = map(xAxis, 1500, 0, 80, 255);
        // Move to left - decrease left motor speed, increase right motor speed
        *motorSpeedA = *motorSpeedA + xMapped;
        *motorSpeedB = *motorSpeedB - xMapped;
        // Confine the range from 0 to 255
        if (*motorSpeedA < 0) {
            *motorSpeedA = 0;
        }
        if (*motorSpeedB > 255) {
            *motorSpeedB = 255;
        }
    }

    else if (xAxis >2000) {
        // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255
        int xMapped = map(xAxis, 2000, 4095, 0, 255);
        // Move right - decrease right motor speed, increase left motor speed
        *motorSpeedA = *motorSpeedA - xMapped;
        *motorSpeedB = *motorSpeedB + xMapped;
        // Confine the range from 0 to 255
        if (*motorSpeedA > 255) {
            *motorSpeedA = 255;
        }
        if (*motorSpeedB < 0) {
            *motorSpeedB = 0;
        }
    }


    // Prevent buzzing at low speeds (Adjust according to your motors. My motors buzz below 70)
    if (*motorSpeedA < 70) {
        *motorSpeedA = 0;
    }

    if (*motorSpeedB < 70) {
        *motorSpeedB = 0;
    }


}
void initMotorPWM(){
    //Configure timer
    ledc_timer_config_t configTimer = {
        .speed_mode= LEDC_LOW_SPEED_MODE,
        .timer_num= LEDC_TIMER_0,
        .freq_hz = 6000,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = LEDC_TIMER_8_BIT
    };
    ledc_timer_config(&configTimer);

    //Configure channels
    ledc_channel_config_t configChannelA = {
        .gpio_num = enableA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint= 0 
    };
    ledc_channel_config(&configChannelA);
    
    ledc_channel_config_t configChannelB =
    {
        .gpio_num = enableB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1, 
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint= 0 
    };
    ledc_channel_config(&configChannelB);

}
void set_motor_speed(int motorSpeedA, int motorSpeedB) {
    //change motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, motorSpeedA);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    //chnage motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, motorSpeedB);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
