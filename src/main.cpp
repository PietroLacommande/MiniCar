#include <Arduino.h>
#include "MotorControl.h"


#define joyStickX 34
#define joyStickY 35


int motorSpeedA=0;
int motorSpeedB =0;


void setup() {

    setUpWheelLogic(in1,in2,in3,in4,enableA,enableB);
    Serial.begin(115200);

}

void loop() {

    int xAxis= analogRead(joyStickX);
    int yAxis= analogRead(joyStickY);


    SpeedAndDirectionControl(xAxis,yAxis,&motorSpeedA, &motorSpeedB);

    analogWrite(enableA, motorSpeedA);
    analogWrite(enableB, motorSpeedB);

    delay(10);
}
