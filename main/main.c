#include <stdio.h>
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#define HIGH 1
#define LOW 0
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_6 //gpio34
#define input1 GPIO_NUM_18
#define input2 GPIO_NUM_19
#define input3 GPIO_NUM_32
#define input4 GPIO_NUM_33
#define enableA GPIO_NUM_25
#define enableB GPIO_NUM_26

#define OUTPUT_PINS ((1ULL << input1) | (1ULL << input2) |(1ULL << input3) | (1ULL << input4) | (1ULL << enableA) | (1ULL << enableB)) // Define pins as a bitmask

//AUTOSAR types
typedef uint8_t Std_ReturnType;
#define E_OK        (uint8_t)0   
#define E_NOT_OK    (uint8_t)1   

//ADC
#define DMA_BUFFER_SIZE 32
#define CONV_FRAME_SIZE SOC_ADC_DIGI_DATA_BYTES_PER_CONV
#define SAMPLE_FREQUENCY 20000 

unsigned int LMcounter_Left=0;
unsigned int LMcounter_Right=0;

//Function declatations
Std_ReturnType setUpWheelLogic(void);
void SpeedAndDirectionControl(int xAxis, int yAxis, int* motorSpeedA, int* motorSpeedB, bool* isReversed);
int map(int x, int in_min, int in_max, int out_min, int out_max);
Std_ReturnType adcInit();
void setUpTimer();
void onTimer();
void initGpioInterupt();
void IRAM_ATTR speedSensorRight();
void IRAM_ATTR speedSensorLeft();

//Structure configuration for motor
gpio_config_t o_configMotor = {
    .pin_bit_mask= OUTPUT_PINS,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en= GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type =GPIO_INTR_DISABLE
};
//Structure configuration for joystick button
gpio_config_t i_configButton = {
    .pin_bit_mask= (1ULL<< GPIO_NUM_15), //TODO
    .mode = GPIO_MODE_INPUT,
    .pull_up_en= GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type =GPIO_INTR_POSEDGE
};

adc_continuous_handle_t adc_handle;

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
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
void onTimer(){
    //TODO
}

void initTimer() {

   esp_timer_handle_t timerHandle;
   esp_timer_create_args_t timer_config = {
    .callback = onTimer,
    .arg = NULL,
    .name = "Timer1s"
   };

   esp_timer_create(&timer_config,&timerHandle);

   esp_timer_start_periodic(timerHandle, 1000000); //every 1s
}
void initGpioInterupt(){
    gpio_config_t configGPIO = {
        
    };



};
void IRAM_ATTR speedSensorRight(){
    LMcounter_Right++;
};
void IRAM_ATTR speedSensorLeft(){
    LMcounter_Left++;
};

//initializing the ADC continous mode driver
Std_ReturnType adcInit(){
    Std_ReturnType returnVal = E_NOT_OK;
    esp_err_t value;
    
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = DMA_BUFFER_SIZE,
        .conv_frame_size = CONV_FRAME_SIZE,
    };
    
    value = adc_continuous_new_handle(&adc_config, &adc_handle);
    
    if(value!= ESP_OK){
        return returnVal;
    }

    adc_digi_pattern_config_t adcJoystickConf[2]={
        {.atten =  ADC_ATTEN_DB_12,   ///< Attenuation of this ADC channel
        .channel = ADC_CHANNEL_0,    ///< ADC channel
        .unit =  ADC_UNIT_1,     ///< ADC unit
        .bit_width = ADC_BITWIDTH_12},
        {.atten =  ADC_ATTEN_DB_12,   
        .channel = ADC_CHANNEL_3,   
        .unit =  ADC_UNIT_1,    
        .bit_width = ADC_BITWIDTH_12}
    };

    //set up adc configs
    adc_continuous_config_t adcJoystick ={
        .pattern_num = 2,
        .sample_freq_hz = SAMPLE_FREQUENCY,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .adc_pattern = adcJoystickConf,    
    };

    value= adc_continuous_config(adc_handle,&adcJoystick);
    if (value != ESP_OK){
        adc_continuous_deinit(adc_handle);
        return returnVal;
    }
    returnVal = E_OK;
    return returnVal; 
}
Std_ReturnType adcStart(adc_continuous_handle_t adcHandle){
    Std_ReturnType returnVal = E_NOT_OK;
    
    esp_err_t value = adc_continuous_start(adcHandle);
    if(value!= ESP_OK){
        return returnVal;
    }
    returnVal = E_OK;
    return returnVal;
}
// Function to process joystick data
void process_joystick_data(const adc_digi_output_data_t *data, size_t num_samples, int* xAxis, int* yAxis) {
    for (size_t i = 0; i < num_samples; ++i) {
        if (data[i].type1.channel == 0) {
            int raw_value = data[i].type1.data;  // Raw ADC value
            printf("Joystick X: Raw=%d  ", raw_value);
            *xAxis = raw_value;
        }
        else if(data[i].type1.channel == 3){
            int raw_value = data[i].type1.data;  // Raw ADC value
            printf("Joystick Y: Raw=%d\n", raw_value);
            *yAxis = raw_value;
        }
    }
}
// ADC read task
Std_ReturnType adc_read_task(int* xAxis, int* yAxis) {
    Std_ReturnType returnVal= E_NOT_OK;
    if(xAxis== NULL || yAxis==NULL){
        return returnVal;
    }
    uint8_t buffer[CONV_FRAME_SIZE];
    uint32_t bytes_read = 0;

    // Read ADC data from the DMA buffer
    if (adc_continuous_read(adc_handle, buffer, sizeof(buffer), &bytes_read, ADC_MAX_DELAY) == ESP_OK) {
        size_t num_samples = bytes_read / sizeof(adc_digi_output_data_t);
        process_joystick_data((adc_digi_output_data_t *)buffer, num_samples, xAxis, yAxis);
    }
    returnVal = E_OK;
    return returnVal;
}

//main function
void app_main(void)
{
    int xAxis=0;
    int yAxis=0;
    int motorSpeedA=0;
    int motorSpeedB=0;
    bool isReversed= false;
    initTimer();
    adcInit();
    adcStart(adc_handle);
    setUpWheelLogic();
    initMotorPWM();

    while(1){
        adc_read_task(&xAxis, &yAxis);
        SpeedAndDirectionControl(xAxis,yAxis,&motorSpeedA,&motorSpeedB,&isReversed);
        printf("speedA: %d", motorSpeedA);
        printf("speedB: %d", motorSpeedB);
        set_motor_speed(motorSpeedA, motorSpeedB);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

