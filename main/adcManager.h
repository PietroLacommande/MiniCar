#include "esp_adc/adc_continuous.h"

//ADC
#define DMA_BUFFER_SIZE 32
#define CONV_FRAME_SIZE SOC_ADC_DIGI_DATA_BYTES_PER_CONV
#define SAMPLE_FREQUENCY 20000 
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_6 //gpio34

//AUTOSAR types
typedef uint8_t Std_ReturnType;
#define E_OK        (uint8_t)0   
#define E_NOT_OK    (uint8_t)1 

Std_ReturnType adcInit(adc_continuous_handle_t* adc_handle);
Std_ReturnType adcStart(adc_continuous_handle_t* adcHandle);
Std_ReturnType adc_read_task(int* xAxis, int* yAxis, adc_continuous_handle_t* adc_handle);