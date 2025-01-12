#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include "esp_adc/adc_continuous.h"
#include "autosarTypes.h"


//ADC
#define DMA_BUFFER_SIZE 32
#define CONV_FRAME_SIZE SOC_ADC_DIGI_DATA_BYTES_PER_CONV
#define SAMPLE_FREQUENCY 20000 
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_6 //gpio34

Std_ReturnType adcInit(adc_continuous_handle_t* adc_handle);
Std_ReturnType adcStart(adc_continuous_handle_t* adcHandle);
Std_ReturnType adc_read_task(int* xAxis, int* yAxis, adc_continuous_handle_t* adc_handle);

#endif //ADC_MANAGER_H