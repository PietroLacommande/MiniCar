#include "encoderManager.h"

float speedRight=0;
float rotationRight=0;
int countRight=0;

float speedLeft=0;
float rotationLeft=0;
int countLeft=0;

extern bool isReversed; //TODO
pcnt_unit_handle_t pcntHandleRight;
pcnt_unit_handle_t pcntHandleLeft;

pcnt_unit_config_t pcntConfig = {
        .low_limit = -1,
        .high_limit = 32767,    
        .intr_priority =0
    };

esp_timer_create_args_t timer_config = {
    .callback = onTimer,
    .arg = &isReversed,
    .name = "Timer1s"
   };

pcnt_channel_handle_t channelHandleRight;
pcnt_channel_handle_t channelHandleLeft;

pcnt_chan_config_t channelConfRight= {
    .edge_gpio_num= GPIO_NUM_17,
    .level_gpio_num =-1,
};
pcnt_chan_config_t channelConfLeft= {
    .edge_gpio_num= GPIO_NUM_5,
    .level_gpio_num =-1,
};

//This will count the number of increments of the wheel encoders
void initPCNT(){
    pcnt_new_unit(&pcntConfig, &pcntHandleRight);
    pcnt_new_unit(&pcntConfig, &pcntHandleLeft);

    
    pcnt_new_channel(pcntHandleRight, &channelConfRight, &channelHandleRight);
    pcnt_new_channel(pcntHandleLeft, &channelConfLeft, &channelHandleLeft);

    pcnt_channel_set_edge_action(channelHandleRight,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_edge_action(channelHandleLeft,PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_HOLD);

    pcnt_unit_enable(pcntHandleRight);
    pcnt_unit_start(pcntHandleRight);

    pcnt_unit_enable(pcntHandleLeft);
    pcnt_unit_start(pcntHandleLeft);
    //TODO maybe add a glitch filter

}

void onTimer(bool* direction){
    pcnt_unit_get_count(pcntHandleRight, &countRight);
    pcnt_unit_get_count(pcntHandleLeft, &countLeft);

    rotationRight = (float)countRight/encoderHoles;
    speedRight = rotationRight * wheelCircumference;

    rotationLeft = (float)countLeft/encoderHoles;
    speedLeft = rotationLeft * wheelCircumference;

    pcnt_unit_clear_count(pcntHandleRight); //clear count
    pcnt_unit_clear_count(pcntHandleLeft); //clear count

    printf("speedRight: %s%.2f m/s\n", isReversed ? "-" : "", speedRight);
    printf("speedLeft: %s%.2f m/s\n", isReversed ? "-" : "", speedLeft);
}

void initTimer() {
   esp_timer_handle_t timerHandle;
   esp_timer_create(&timer_config,&timerHandle);
   esp_timer_start_periodic(timerHandle, 1000000); //every 1s
}