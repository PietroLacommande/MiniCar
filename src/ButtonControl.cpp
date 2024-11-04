//
// Created by Pietr on 2024-11-04.
//


#include "ButtonControl.h"


//Function executed when there is an interrupt
void doubleClickFunct() {
//    unsigned long currentTime = millis();  // Get the current time
//
//    if (currentTime - lastDebounceTime > MaxDebounceDelay) {  // Debounce check
//        if (firstClickDetected && (currentTime - firstClickTime < DoubleClickThreshold)) {
//            // If a second click is detected within the double-click threshold
//            interruptCounter = 0;  // Reset counter on double-click
//            firstClickDetected = false;  // Reset flag
//            Serial.println("Double click detected: Counter reset to 0");
//        } else {
//            // First click detected or it's been too long for a double-click
//            firstClickTime = currentTime;
//            firstClickDetected = true;
//            interruptCounter++;  // Increment on single click
//            Serial.println("Single click detected: Counter incremented");
//        }
//        lastDebounceTime = currentTime;  // Update last debounce time
//    }
}