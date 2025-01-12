#include "adcManager.h"

adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = DMA_BUFFER_SIZE,
        .conv_frame_size = CONV_FRAME_SIZE,
    };
    
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

//initializing the ADC continous mode driver
Std_ReturnType adcInit(adc_continuous_handle_t* adc_handle){
    Std_ReturnType returnVal = E_NOT_OK;
    esp_err_t espValue;

    if(adc_handle!=NULL){
        espValue = adc_continuous_new_handle(&adc_config, adc_handle);
        
        if(espValue== ESP_OK){
            espValue= adc_continuous_config(*adc_handle,&adcJoystick);
        }
        if (espValue == ESP_OK){
            returnVal = E_OK;
        }
    }

    else{
        //do nothing
    }

    return returnVal; 
}

void adcDeInit(adc_continuous_handle_t* adc_handle){
    if(adc_handle != NULL){
        adc_continuous_deinit(*adc_handle);
    }
}

Std_ReturnType adcStart(adc_continuous_handle_t* adcHandle){
    Std_ReturnType returnVal = E_NOT_OK;
    esp_err_t value;

    if(adcHandle!= NULL){
        value = adc_continuous_start(*adcHandle);
        if(value== ESP_OK){
            returnVal = E_OK;
        }
    }

    else{
        //do nothing
    }   
    return returnVal;
}

// Function to process joystick data
void static process_joystick_data(const adc_digi_output_data_t *data, size_t num_samples, int* xAxis, int* yAxis) {
    for (size_t i = 0; i < num_samples; ++i) {
        if (data[i].type1.channel == 0) {
            int raw_value = data[i].type1.data;  // Raw ADC value
            // printf("Joystick X: Raw=%d  ", raw_value);
            *xAxis = raw_value;
        }
        else if(data[i].type1.channel == 3){
            int raw_value = data[i].type1.data;  // Raw ADC value
            // printf("Joystick Y: Raw=%d\n", raw_value);
            *yAxis = raw_value;
        }
    }
}
// ADC read task
Std_ReturnType adc_read_task(int* xAxis, int* yAxis, adc_continuous_handle_t* adc_handle) {
    Std_ReturnType returnVal= E_NOT_OK;
    if(xAxis!=NULL && yAxis!=NULL){
        uint8_t buffer[CONV_FRAME_SIZE];
        uint32_t bytes_read = 0;
        // Read ADC data from the DMA buffer
        if (adc_continuous_read(*adc_handle, buffer, sizeof(buffer), &bytes_read, ADC_MAX_DELAY) == ESP_OK) {
            size_t num_samples = bytes_read / sizeof(adc_digi_output_data_t);
            process_joystick_data((adc_digi_output_data_t *)buffer, num_samples, xAxis, yAxis);
        }
        returnVal = E_OK;
    }
    
    return returnVal;
}
