//
// Created by Pietr on 2024-11-04.
//

#include <Arduino.h>
#include "ButtonControl.h"
#include "p_i_d_controller.h"

// //Function executed when there is an interrupt
// void doubleClickFunct() {
//    unsigned long currentTime = millis();  // Get the current time
//
//     if (currentTime - lastDebounceTime > MaxDebounceDelay) {  // Debounce check
//         if (firstClickDetected && (currentTime - firstClickTime < DoubleClickThreshold)) {
//             // If a second click is detected within the double-click threshold
//             interruptCounter = 0;  // Reset counter on double-click
//             firstClickDetected = false;  // Reset flag
//             Serial.println("Double click detected: Counter reset to 0");
//         } else {
//             // First click detected or it's been too long for a double-click
//             firstClickTime = currentTime;
//             firstClickDetected = true;
//             interruptCounter++;  // Increment on single click
//             Serial.println("Single click detected: Counter incremented");
//         }
//         lastDebounceTime = currentTime;  // Update last debounce time
//     }
// }


// Déclarations des variables globales
unsigned long lastDebounceTime = 0;  // Pour le débogage du "debounce"
unsigned long firstClickTime = 0;    // Pour suivre le temps du premier clic
bool firstClickDetected = false;     // Drapeau pour détecter le premier clic
const unsigned long MaxDebounceDelay = 100;  // Délai maximal pour "debounce"
const unsigned long DoubleClickThreshold = 400;  // Seuil de double-clic (en millisecondes)
volatile int interruptCounter = 0;  // Compteur des interruptions


// PID pidController(0,0,0);  // Déclarez l'objet


// Fonction exécutée lorsqu'il y a une interruption
void IRAM_ATTR doubleClickFunct() {
    unsigned long currentTime = millis();  // Obtenez le temps courant

    if (currentTime - lastDebounceTime > MaxDebounceDelay) {  // Vérification "debounce"
        if (firstClickDetected && (currentTime - firstClickTime < DoubleClickThreshold)) {
            // Si un deuxième clic est détecté dans le seuil de double-clic
            interruptCounter = 0;  // Réinitialiser le compteur sur double-clic
            firstClickDetected = false;  // Réinitialiser le drapeau
            Serial.print("Double clic detected: Pas de PID");
            Serial.println(interruptCounter);


            // Réinitialiser les valeurs PID
            // pidController.resetTunings(); // pas bon car tu ne reset pas le bon PID

        } else {
            // Premier clic détecté ou il s'est passé trop de temps pour un double-clic
            firstClickTime = currentTime;
            firstClickDetected = true;
            interruptCounter++;  // Incrémenter sur simple clic
            Serial.print("Simple clic detected: Compteur Kvalue: ");
            Serial.println(interruptCounter);
        }
        lastDebounceTime = currentTime;  // Mettre à jour le dernier temps de "debounce"
    }
}