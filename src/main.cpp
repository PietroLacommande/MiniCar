#include <Arduino.h>
#include "MotorControl.h"

#define joyStickX 34
#define joyStickY 35
#define encoderHoles 20

unsigned int LMcounter=0;
hw_timer_t *timer = NULL;  // Define a pointer to the timer

int motorSpeedA=0;
int motorSpeedB=0;

void speedSensor(){
    LMcounter++;
}

void IRAM_ATTR onTimer() {
    timerDetachInterrupt(timer);
    Serial.print("Motor speed ");
    int rotation = LMcounter/encoderHoles;
    Serial.print(rotation,DEC);
    Serial.println(" Rotations per second");
    LMcounter=0;
    timerAttachInterrupt(timer, &onTimer,true);
}



void setup() {

    setUpWheelLogic(input1,input2,input3,input4,enableA,enableB);
    Serial.begin(115200);
    timer = timerBegin(0,80,true); //Clock is now 1MHz due to 80 divider
    timerAttachInterrupt(timer, &onTimer,true);
    timerAlarmWrite(timer,1000000,true); //1000000 clicks gives 1 sec
    timerAlarmEnable(timer);

    pinMode(17, INPUT);    // Set encoder pin as input
    attachInterrupt(digitalPinToInterrupt(17),speedSensor,RISING);

}

void loop() {

    int xAxis= analogRead(joyStickX);
    int yAxis= analogRead(joyStickY);
//    Serial.print("x-axis:");
//    Serial.println(xAxis);
//
//    Serial.print("y-axis:");
//    Serial.println(yAxis);


    SpeedAndDirectionControl(xAxis,yAxis,&motorSpeedA, &motorSpeedB);

    //affect motor PWM
    analogWrite(enableA, motorSpeedA);
    analogWrite(enableB, motorSpeedB);

//
//
//    Serial.print("motorSpeedA: ");
//    Serial.println(motorSpeedA);
//
//    Serial.print("motorSpeedB: ");
//    Serial.println(motorSpeedB);

    delay(1000);
}
