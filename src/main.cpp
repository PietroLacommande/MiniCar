#include <Arduino.h>
#include "MotorControl.h"
#include "ButtonControl.h"
#include "p_i_d_controller.h"

#define joyStickX 34
#define joyStickY 35
#define encoderHoles 20
#define wheelCircumference 0.22
#define buttonPin 16  // Define the button pin

unsigned int LMcounter_Left=0;
unsigned int LMcounter_Right=0;
hw_timer_t *timer = NULL;  // Define a pointer to the timer

int motorSpeedA=0;
int motorSpeedB=0;
bool isBackwards = false;


unsigned int displayCounter = 0; //initialisation of the counter for the display

float Kp_Left = 0.0;
float Ki_Left = 0.0;
float Kd_Left = 0.0;
//
// float Kp_Right = 0.0;
// float Ki_Right = 0.0;
// float Kd_Right = 0.0;


PID pidLeft(Kp_Left, Ki_Left, Kd_Left);  // Ajustez les valeurs Kp, Ki, Kd (celles de gauche)
// PID pidRight(Kp_Right, Ki_Right, Kd_Right);  // Ajustez les valeurs Kp, Ki, Kd

float targetSpeedLeft = 0.0;
// float targetSpeedRight = 0.0;



void speedSensorRight(){
    LMcounter_Right++;
}

void speedSensorLeft(){
    LMcounter_Left++;
}



void IRAM_ATTR onTimer() {
    //timerDetachInterrupt(timer);
    float rotation_Left = (float)LMcounter_Left / encoderHoles;
    float speed_Left = rotation_Left * wheelCircumference;

    float rotation_Right = (float)LMcounter_Right / encoderHoles;
    float speed_Right = rotation_Right * wheelCircumference;

    float dt = 0.1;  // 0,1 seconde
    displayCounter++; // Incrémenter le compteur d'affichage

    float correctionLeft = pidLeft.calculate(targetSpeedLeft, speed_Left, dt);
    // float correctionRight = pidRight.calculate(targetSpeedRight, speed_Right, dt);

    motorSpeedA = constrain(motorSpeedA + correctionLeft, 80, 255);
    // motorSpeedB = constrain(motorSpeedB + correctionRight, 0, 255);


    // Display every 10 interruptions (every 1 second)
    if (displayCounter >= 10) {
        Serial.print("Left wheel speed: ");
        Serial.print(isBackwards ? "-" : "");
        Serial.print(speed_Left, 2);
        Serial.print(" m/s, ");
        //
        // Serial.print("Right wheel speed: ");
        // Serial.print(isBackwards ? "-" : "");
        // Serial.print(speed_Right, 2);
        // Serial.println(" m/s");

        displayCounter = 0; // Réinitialiser le compteur d'affichage
    }


    //Reset counters
    LMcounter_Left = 0;
    LMcounter_Right = 0;
    //timerAttachInterrupt(timer, &onTimer, true);
}

// void updatePIDConstants() {
//     if (Serial.available() > 0) {
//         // Variables temporaires pour lire les nouvelles valeurs
//         float newKp_Left, newKi_Left, newKd_Left; // variables to read the new values (Left)
//         float newKp_Right, newKi_Right, newKd_Right;// variables to read the new values (Right)
//
//         // Lire les nouvelles valeurs depuis le moniteur série
//         Serial.println("Enter PID constants in the format: Kp_Left Ki_Left Kd_Left Kp_Right Ki_Right Kd_Right");
//
//         newKp_Left = Serial.parseFloat();
//         newKi_Left = Serial.parseFloat();
//         newKd_Left = Serial.parseFloat();
//         newKp_Right = Serial.parseFloat();
//         newKi_Right = Serial.parseFloat();
//         newKd_Right = Serial.parseFloat();
//
//         // Mettre à jour les constantes PID
//         if (newKp_Left != 0 && newKi_Left != 0 && newKd_Left != 0 && newKp_Right != 0 && newKi_Right != 0 && newKd_Right != 0) {
//             Kp_Left = newKp_Left;
//             Ki_Left = newKi_Left;
//             Kd_Left = newKd_Left;
//             Kp_Right = newKp_Right;
//             Ki_Right = newKi_Right;
//             Kd_Right = newKd_Right;
//
//             // Met à jour les objets PID avec les nouvelles valeurs
//             pidLeft.setTunings(Kp_Left, Ki_Left, Kd_Left);
//             pidRight.setTunings(Kp_Right, Ki_Right, Kd_Right);
//
//             Serial.println("Updated PID constants:");
//             Serial.print("Left: Kp = ");
//             Serial.print(Kp_Left);
//             Serial.print(", Ki = ");
//             Serial.print(Ki_Left);
//             Serial.print(", Kd = ");
//             Serial.println(Kd_Left);
//
//             Serial.print("Right: Kp = ");
//             Serial.print(Kp_Right);
//             Serial.print(", Ki = ");
//             Serial.print(Ki_Right);
//             Serial.print(", Kd = ");
//             Serial.println(Kd_Right);
//         }
//     }
// }


void requestPIDConstants() {
    Serial.println("Enter PID constants in the format: Kp_Left Ki_Left Kd_Left Kp_Right Ki_Right Kd_Right");

    // Variables temporaires pour lire les nouvelles valeurs
    float newKp_Left, newKi_Left, newKd_Left;
    float newKp_Right, newKi_Right, newKd_Right;

    // Lire les nouvelles valeurs depuis le moniteur série
    while (Serial.available() == 0) {
        delay(100); // Attendre que l'utilisateur entre les données
    }

    // newKp_Left = Serial.parseFloat();
    // newKi_Left = Serial.parseFloat();
    // newKd_Left = Serial.parseFloat();
    // newKp_Right = Serial.parseFloat();
    // newKi_Right = Serial.parseFloat();
    // newKd_Right = Serial.parseFloat();


    Serial.print("Enter Kp_Left: ");
    while (Serial.available() == 0) {}
    newKp_Left = Serial.readStringUntil('\n').toFloat();

    Serial.print("Enter Ki_Left: ");
    while (Serial.available() == 0) {}
    newKi_Left = Serial.readStringUntil('\n').toFloat();

    Serial.print("Enter Kd_Left: ");
    while (Serial.available() == 0) {}
    newKd_Left = Serial.readStringUntil('\n').toFloat();


    // Serial.print("Enter Kp_Right: ");
    // while (Serial.available() == 0) {}
    // newKp_Right = Serial.readStringUntil('\n').toFloat();
    //
    // Serial.print("Enter Ki_Right: ");
    // while (Serial.available() == 0) {}
    // newKi_Right = Serial.readStringUntil('\n').toFloat();
    //
    // Serial.print("Enter Kd_Right: ");
    // while (Serial.available() == 0) {}
    // newKd_Right = Serial.readStringUntil('\n').toFloat();


    // Mettre à jour les constantes PID
    if (newKp_Left != 0 && newKi_Left != 0 && newKd_Left != 0 && newKp_Right != 0 && newKi_Right != 0 && newKd_Right != 0) {
        Kp_Left = newKp_Left;
        Ki_Left = newKi_Left;
        Kd_Left = newKd_Left;
        // Kp_Right = newKp_Right;
        // Ki_Right = newKi_Right;
        // Kd_Right = newKd_Right;

        // Mettre à jour les objets PID avec les nouvelles valeurs
        pidLeft.setTunings(Kp_Left, Ki_Left, Kd_Left);
        // pidRight.setTunings(Kp_Right, Ki_Right, Kd_Right);


        Serial.println("Updated PID constants:");
        // Serial.print("Left: Kp = ");
        // Serial.print(Kp_Left);
        // Serial.print(", Ki = ");
        // Serial.print(Ki_Left);
        // Serial.print(", Kd = ");
        // Serial.println(Kd_Left);

        Serial.print("Right: Kp = ");
        // Serial.print(Kp_Right);
        Serial.print(", Ki = ");
        // Serial.print(Ki_Right);
        Serial.print(", Kd = ");
        // Serial.println(Kd_Right);
    }
}


void setup() {

    setUpWheelLogic(input1,input2,input3,input4,enableA,enableB);
    Serial.begin(115200);
    requestPIDConstants(); // Ask the user the desired K values
    Serial.println("Enter PID constants in the format: Kp_Left Ki_Left Kd_Left Kp_Right Ki_Right Kd_Right");
    timer = timerBegin(0,80,true); //Clock is now 1MHz due to 80 divider
    timerAttachInterrupt(timer, &onTimer,true);
    timerAlarmWrite(timer,100000,true); //100000 clicks gives 100 ms or 0.1 sec
    timerAlarmEnable(timer);

    pinMode(17, INPUT);    // Set encoder pin as input
    pinMode(5, INPUT);    // Set encoder pin as input

    attachInterrupt(digitalPinToInterrupt(17),speedSensorRight,RISING);
    attachInterrupt(digitalPinToInterrupt(5),speedSensorLeft,RISING);


    // Interruption button set up
    pinMode(buttonPin, INPUT_PULLUP);  // uses the pin defined by buttonPin
    attachInterrupt(digitalPinToInterrupt(buttonPin), doubleClickFunct, RISING);  // Uses the interrupt function doubleClickFunct

}


void loop() {

    //updatePIDConstants();  // Vérifiez et mettez à jour les constantes PID si nécessaire

    int x= analogRead(joyStickX);
    int y= analogRead(joyStickY);
//    Serial.print("x-axis:");
//    Serial.println(xAxis);
//
//    Serial.print("y-axis:");
//    Serial.println(yAxis);

    SpeedAndDirectionControl(x, y, &motorSpeedA, &motorSpeedB, &isBackwards);

// with set the motors speed to the target speed
    targetSpeedLeft = motorSpeedA;
    // targetSpeedRight = motorSpeedB;


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
