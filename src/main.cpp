#include <Arduino.h>
#include "MotorControl.h"

#define joyStickX 34
#define joyStickY 35
#define encoderHoles 20
#define wheelCircumference 0.22

unsigned int LMcounter_Left=0;
unsigned int LMcounter_Right=0;
hw_timer_t *timer = NULL;  // Define a pointer to the timer

int motorSpeedA=0;
int motorSpeedB=0;
bool isBackwards = false;

void speedSensorRight(){
    LMcounter_Right++;
}

void speedSensorLeft(){
    LMcounter_Left++;
}



void IRAM_ATTR onTimer() {
    timerDetachInterrupt(timer);
    float rotation_Left = (float)LMcounter_Left / encoderHoles;
    float speed_Left = rotation_Left * wheelCircumference;

    float rotation_Right = (float)LMcounter_Right / encoderHoles;
    float speed_Right = rotation_Right * wheelCircumference;

    if (isBackwards) {
        Serial.print("Left wheel speed: ");
        Serial.print("-");
        Serial.print(speed_Left, 2);
        Serial.println(" m/s, ");

        Serial.print("Right wheel speed: ");
        Serial.print("-");
        Serial.print(speed_Right, 2);
        Serial.println(" m/s");

    }

    else {
        Serial.print("Left wheel speed: ");
        Serial.print(speed_Left, 2);
        Serial.println(" m/s, ");

        Serial.print("Right wheel speed: ");
        Serial.print(speed_Right, 2);
        Serial.println(" m/s");
    }


    LMcounter_Left = 0;
    LMcounter_Right = 0;
    timerAttachInterrupt(timer, &onTimer, true);
}





void setup() {

    setUpWheelLogic(input1,input2,input3,input4,enableA,enableB);
    Serial.begin(115200);
    timer = timerBegin(0,80,true); //Clock is now 1MHz due to 80 divider
    timerAttachInterrupt(timer, &onTimer,true);
    timerAlarmWrite(timer,1000000,true); //1000000 clicks gives 1 sec
    timerAlarmEnable(timer);

    pinMode(17, INPUT);    // Set encoder pin as input
    pinMode(5, INPUT);    // Set encoder pin as input

    attachInterrupt(digitalPinToInterrupt(17),speedSensorRight,RISING);
    attachInterrupt(digitalPinToInterrupt(5),speedSensorLeft,RISING);
}

void loop() {

    int x= analogRead(joyStickX);
    int y= analogRead(joyStickY);
//    Serial.print("x-axis:");
//    Serial.println(xAxis);
//
//    Serial.print("y-axis:");
//    Serial.println(yAxis);

    SpeedAndDirectionControl(x, y, &motorSpeedA, &motorSpeedB, &isBackwards);
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

    delay(100);
}
