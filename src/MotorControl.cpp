//
// Created by Pietr on 2024-11-03.
//
#include "MotorControl.h"
#include <Arduino.h>

void setUpWheelLogic(int pin1, int pin2, int pin3, int pin4, int pinA,  int pinB){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
}


void setUpWheelLogic(int pin1, int pin2, int pin3, int pin4){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);
}

void SpeedAndDirectionControl(int xAxis, int yAxis, int* motorSpeedA, int* motorSpeedB) {
    if (yAxis < 1850) {
        // Set Motor A backward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        // Set Motor B backward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        // Convert the declining Y-axis readings for going backward from 470
        *motorSpeedA = map(yAxis, 1850, 0, 80, 255);
        *motorSpeedB = map(yAxis, 1850, 0, 80, 255);
    } else if (yAxis > 1900) {
        // Set Motor A forward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        // Set Motor B forward
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        // Convert the declining Y-axis readings for going backward from 470
        *motorSpeedA = map(yAxis, 1900, 4095, 80, 255);
        *motorSpeedB = map(yAxis, 1900, 4095, 80, 255);
    } else {
        *motorSpeedA = 0;
        *motorSpeedB = 0;
    }


    // X-axis used for left and right control
    if (xAxis < 1850) {
        // Convert the declining X-axis readings from 470 to 0 into increasing values
        int xMapped = map(xAxis, 1850, 0, 80, 255);
        // Move to left - decrease left motor speed, increase right motor speed
        *motorSpeedA = *motorSpeedA - xMapped;
        *motorSpeedB = *motorSpeedB + xMapped;
        // Confine the range from 0 to 255
        if (*motorSpeedA < 0) {
            *motorSpeedA = 0;
        }
        if (*motorSpeedB > 255) {
            *motorSpeedB = 255;
        }
    }

    if (xAxis > 1900) {
        // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255
        int xMapped = map(xAxis, 1900, 4095, 0, 255);
        // Move right - decrease right motor speed, increase left motor speed
        *motorSpeedA = *motorSpeedA + xMapped;
        *motorSpeedB = *motorSpeedB - xMapped;
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

